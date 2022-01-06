// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Google, Inc.
 *
 * Author:
 *	Sami Tolvanen <samitolvanen@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"pgo: " fmt

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include "pgo.h"

static struct dentry *directory;

struct prf_private_data {
	void *buffer;
	size_t size;
	struct prf_cpu_object *link;
};

/* vmlinux's prf object */
static struct prf_object prf_vmlinux;

/* The prf_list */
LIST_HEAD(prf_list);

/*
 * Raw profile data format:
 *
 *	- llvm_prf_header
 *	- __llvm_prf_data
 *	- __llvm_prf_cnts
 *	- __llvm_prf_names
 *	- zero padding to 8 bytes
 *	- for each llvm_prf_data in __llvm_prf_data:
 *		- llvm_prf_value_data
 *			- llvm_prf_value_record + site count array
 *				- llvm_prf_value_node_data
 *				...
 *			...
 *		...
 */

static void prf_fill_header(struct prf_object *po, void **buffer)
{
	struct llvm_prf_header *header = *(struct llvm_prf_header **)buffer;

#ifdef CONFIG_64BIT
	header->magic = LLVM_INSTR_PROF_RAW_MAGIC_64;
#else
	header->magic = LLVM_INSTR_PROF_RAW_MAGIC_32;
#endif
	header->version = LLVM_VARIANT_MASK_IR_PROF | LLVM_INSTR_PROF_RAW_VERSION;
	header->data_size = prf_data_count(po);
	header->binary_ids_size = 0;
	header->padding_bytes_before_counters = 0;
	header->counters_size = prf_cnts_count(po);
	header->padding_bytes_after_counters = 0;
	header->names_size = prf_names_count(po);
	header->counters_delta = (u64)po->cnts - (u64)po->data;
	header->names_delta = (u64)po->names;
	header->value_kind_last = LLVM_INSTR_PROF_IPVK_LAST;

	*buffer += sizeof(*header);
}

/*
 * Copy the source into the buffer, incrementing the pointer into buffer in the
 * process.
 */
static void prf_copy_to_buffer(void **buffer, const void *src, unsigned long size)
{
	memcpy(*buffer, src, size);
	*buffer += size;
}

static u32 __prf_get_value_size(struct llvm_prf_data *p, u32 *value_kinds)
{
	struct llvm_prf_value_node **nodes =
		(struct llvm_prf_value_node **)p->values;
	u32 kinds = 0;
	u32 size = 0;
	unsigned int kind;
	unsigned int n;
	unsigned int s = 0;

	for (kind = 0; kind < ARRAY_SIZE(p->num_value_sites); kind++) {
		unsigned int sites = p->num_value_sites[kind];

		if (!sites)
			continue;

		/* Record + site count array */
		size += prf_get_value_record_size(sites);
		kinds++;

		if (!nodes)
			continue;

		for (n = 0; n < sites; n++) {
			u32 count = 0;
			struct llvm_prf_value_node *site = nodes[s + n];

			while (site && ++count <= U8_MAX)
				site = site->next;

			size += count *
				sizeof(struct llvm_prf_value_node_data);
		}

		s += sites;
	}

	if (size)
		size += sizeof(struct llvm_prf_value_data);

	if (value_kinds)
		*value_kinds = kinds;

	return size;
}

static u32 prf_get_value_size(struct prf_cpu_object *pco)
{
	u32 size = 0;
	struct llvm_prf_data *p;
	struct llvm_prf_data *end = pco->data + pco->obj->data_num;

	for (p = pco->data; p < end; p++)
		size += __prf_get_value_size(p, NULL);

	return size;
}

/* Serialize the profiling's value. */
static void prf_serialize_value(struct llvm_prf_data *p, void **buffer)
{
	struct llvm_prf_value_data header;
	struct llvm_prf_value_node **nodes =
		(struct llvm_prf_value_node **)p->values;
	unsigned int kind;
	unsigned int n;
	unsigned int s = 0;

	header.total_size = __prf_get_value_size(p, &header.num_value_kinds);

	if (!header.num_value_kinds)
		/* Nothing to write. */
		return;

	prf_copy_to_buffer(buffer, &header, sizeof(header));

	for (kind = 0; kind < ARRAY_SIZE(p->num_value_sites); kind++) {
		struct llvm_prf_value_record *record;
		u8 *counts;
		unsigned int sites = p->num_value_sites[kind];

		if (!sites)
			continue;

		/* Profiling value record. */
		record = *(struct llvm_prf_value_record **)buffer;
		*buffer += prf_get_value_record_header_size();

		record->kind = kind;
		record->num_value_sites = sites;

		/* Site count array. */
		counts = *(u8 **)buffer;
		*buffer += prf_get_value_record_site_count_size(sites);

		/*
		 * If we don't have nodes, we can skip updating the site count
		 * array, because the buffer is zero filled.
		 */
		if (!nodes)
			continue;

		for (n = 0; n < sites; n++) {
			u32 count = 0;
			struct llvm_prf_value_node *site = nodes[s + n];

			while (site && ++count <= U8_MAX) {
				prf_copy_to_buffer(buffer, site,
						   sizeof(struct llvm_prf_value_node_data));
				site = site->next;
			}

			counts[n] = (u8)count;
		}

		s += sites;
	}
}

static void prf_serialize_values(struct prf_cpu_object *pco, void **buffer)
{
	struct llvm_prf_data *p;
	struct llvm_prf_data *end = pco->data + pco->obj->data_num;

	for (p = pco->data; p < end; p++)
		prf_serialize_value(p, buffer);
}

static inline unsigned long prf_get_padding(unsigned long size)
{
	return 7 & (sizeof(u64) - size % sizeof(u64));
}

/* Note: caller *must* take prf_lock_exclusive() */
static unsigned long prf_buffer_size(struct prf_cpu_object *pco)
{
	return sizeof(struct llvm_prf_header) +
			prf_data_size(pco->obj)	+
			prf_cnts_size(pco->obj) +
			prf_names_size(pco->obj) +
			prf_get_padding(prf_names_size(pco->obj)) +
			prf_get_value_size(pco);
}

/*
 * Serialize the profiling data into a format LLVM's tools can understand.
 * Note: p->buffer must point into vzalloc()'d
 * area of at least prf_buffer_size() in size.
 * Note: caller *must* take prf_lock_exclusive()
 */
static void prf_serialize(struct prf_private_data *p)
{
	void *buffer;
	struct prf_cpu_object *pco = p->link;

	buffer = p->buffer;

	prf_fill_header(pco->obj, &buffer);
	prf_copy_to_buffer(&buffer, pco->data,  prf_data_size(pco->obj));
	prf_copy_to_buffer(&buffer, pco->obj->cnts,  prf_cnts_size(pco->obj));
	prf_copy_to_buffer(&buffer, pco->obj->names, prf_names_size(pco->obj));
	buffer += prf_get_padding(prf_names_size(pco->obj));

	prf_serialize_values(pco, &buffer);
}

/* open() implementation for PGO. Creates a copy of the profiling data set. */
static int prf_open(struct inode *inode, struct file *file)
{
	struct prf_private_data *data;
	int err = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* take exclusive lock and stop the profiler. */
	prf_lock_exclusive();

	/* get prf_cpu_object of this inode */
	data->link = inode->i_private;

	/* allocate buffer for the profile data */
	data->size = prf_buffer_size(data->link);
	data->buffer = vzalloc(data->size);

	if (!data->buffer) {
		err = -ENOMEM;
		goto out_err;
	}

	/* serialize the profiler dataset */
	prf_serialize(data);

	file->private_data = data;

out_err:
	if (err) {
		/* clean up on error */
		if (data)
			kfree(data->buffer);

		kfree(data);
	}

	prf_unlock_exclusive();

	return err;
}

/* read() implementation for PGO. */
static ssize_t prf_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	struct prf_private_data *data = file->private_data;

	BUG_ON(!data);

	return simple_read_from_buffer(buf, count, ppos, data->buffer,
				       data->size);
}

/* release() implementation for PGO. Release resources allocated by open(). */
static int prf_release(struct inode *inode, struct file *file)
{
	struct prf_private_data *data = file->private_data;

	if (data) {
		vfree(data->buffer);
		kfree(data);
	}

	return 0;
}

static const struct file_operations prf_fops = {
	.owner		= THIS_MODULE,
	.open		= prf_open,
	.read		= prf_read,
	.llseek		= default_llseek,
	.release	= prf_release
};

static void pgo_percpu_free(struct prf_cpu_object *pco)
{
	int cpu;

	if (pco) {
		for_each_possible_cpu(cpu) {
			kfree(pco[cpu].nodes);
			kfree(pco[cpu].data);
			kfree(pco[cpu].vnds);
			debugfs_remove(pco[cpu].file);
		}
	}
	kfree(pco);
}

/* Count number of value sites of prf_object. */
static unsigned long pgo_num_value_sites(struct prf_object *po)
{
	u32 num_sites = 0;
	struct llvm_prf_data *p;
	struct llvm_prf_data *end = po->data + po->data_num;
	int kind;

	for (p = po->data; p < end; ++p) {
		if (p->values) {
			for (kind = 0; kind < ARRAY_SIZE(p->num_value_sites); kind++)
				num_sites += p->num_value_sites[kind];
		}
	}
	return num_sites;
}

/* Patch pco->data[].values to point into per-cpu area */
static void pgo_percpu_init_site_ptrs(struct prf_cpu_object *pco)
{
	struct llvm_prf_data *p = pco->data;
	struct llvm_prf_data *end = p + pco->obj->data_num;
	struct llvm_prf_value_node **pcurr = pco->nodes;
	int kind;

	for (; p < end; ++p) {
		if (p->values) {
			p->values = pcurr;
			/* advance the ptr */
			for (kind = 0; kind < ARRAY_SIZE(p->num_value_sites); kind++)
				pcurr += p->num_value_sites[kind];
		}
	}
}

/*
 * Take prf_object and initialize ->pcpu array
 * based on the set prf sections and name.
 * This creates debugfs entries for the object:
 * vmlinux.0.profraw, vmlinux.1.profraw, etc.
 */
static struct prf_cpu_object *pgo_percpu_init(struct prf_object *po)
{
	int cpu;
	struct prf_cpu_object *pco;
	char fname[MODULE_NAME_LEN + 11]; /* +strlen("000.profraw") */
	int num_value_sites = pgo_num_value_sites(po);

	/* alloc percpu structures */
	pco = kcalloc(num_online_cpus(), sizeof(po->pcpu[0]), GFP_KERNEL);
	if (!pco)
		goto err_free;

	for_each_online_cpu(cpu) {
		pco[cpu].cpu = cpu;
		pco[cpu].obj = po;

		/* alloc per-cpu site ptr table */
		pco[cpu].nodes =
			kcalloc(num_value_sites, sizeof(pco[0].nodes), GFP_KERNEL);
		if (!pco[cpu].nodes)
			goto err_free;

		/* init per-cpu __llvm_prf_data data */
		pco[cpu].data =
			kmemdup(po->data, sizeof(po->data[0]) * po->data_num, GFP_KERNEL);
		if (!pco[cpu].data)
			goto err_free;

		/* assign site ptr table to pco[cpu].data */
		pgo_percpu_init_site_ptrs(&pco[cpu]);

		/* alloc per-cpu __llvm_prf_vnds memory */
		pco[cpu].vnds =
			kcalloc(po->vnds_num, sizeof(*pco[cpu].vnds), GFP_KERNEL);
		if (!pco[cpu].vnds)
			goto err_free;

		/* create debugfs entry for the cpu */
		fname[0] = 0;
		snprintf(fname, sizeof(fname), "%s.%d.profraw", po->name, cpu);

		pco[cpu].file = debugfs_create_file(fname, 0600, directory, &pco[cpu], &prf_fops);
		if (!pco[cpu].file) {
			pr_err("Failed to setup pgo: %s", fname);
			goto err_free;
		}
	}
	return pco;

err_free:
	pr_err("-ENOMEM");
	/* free pcpu array */
	pgo_percpu_free(pco);
	return NULL;
}

static void pgo_module_init(struct module *mod)
{
	struct prf_object *po;

	/* Alloc prf_object entry for the module */
	po = kzalloc(sizeof(*po), GFP_KERNEL);
	if (!po)
		return; /* -ENOMEM */

	/* Setup prf_object instance */
	po->name = mod->name;

	po->data = mod->prf_data;
	po->data_num = prf_get_count(mod->prf_data,
					(char *)mod->prf_data + mod->prf_data_size,
					sizeof(po->data[0]));

	po->cnts = mod->prf_cnts;
	po->cnts_num = prf_get_count(mod->prf_cnts,
					(char *)mod->prf_cnts + mod->prf_cnts_size,
					sizeof(po->cnts[0]));

	po->names = mod->prf_names;
	po->names_num = prf_get_count(mod->prf_names,
					(char *)mod->prf_names + mod->prf_names_size,
					sizeof(po->names[0]));

	po->vnds_num = prf_get_count(mod->prf_vnds,
					(char *)mod->prf_vnds + mod->prf_vnds_size,
					sizeof(struct llvm_prf_value_node));

	/* Initialize rest of the structure */
	po->pcpu = pgo_percpu_init(po);
	if (!po->pcpu)
		return;

	/* Enable profiling for the module */
	prf_lock_exclusive();
	list_add_tail_rcu(&po->link, &prf_list);
	prf_unlock_exclusive();
}

static void pgo_module_free(struct rcu_head *rp)
{
	struct prf_object *po = container_of(rp, struct prf_object, rcu);

	pgo_percpu_free(po->pcpu);
	kfree(po);
}

static int pgo_module_notifier(struct notifier_block *nb, unsigned long event,
			       void *pdata)
{
	struct module *mod = pdata;
	struct prf_object *po;
	int cpu;

	if (event == MODULE_STATE_LIVE) {
		/* Can we enable profiling for the module? */
		if (mod->prf_data && mod->prf_cnts && mod->prf_names &&
		    mod->prf_vnds && mod->prf_vnds_size > 0) {
			/* Setup module profiling */
			pgo_module_init(mod);

			pr_info("%s: Enabled", mod->name);
		} else {
			/* Some modules can't be profiled */
			pr_warn("%s: Disabled, no counters", mod->name);
		}
	}

	if (event == MODULE_STATE_GOING) {
		/* Find the prf_object from the list */
		rcu_read_lock();

		list_for_each_entry_rcu(po, &prf_list, link) {
			if (strcmp(po->name, mod->name) == 0)
				goto mod_found;
		}
		/* No such module */
		po = NULL;

mod_found:
		rcu_read_unlock();

		if (po) {
			/* Remove from profiled modules */
			prf_lock_exclusive();
			list_del_rcu(&po->link);

			/* Unlink debugfs entries now */
			for_each_possible_cpu(cpu) {
				debugfs_remove(po->pcpu[cpu].file);
				po->pcpu[cpu].file = NULL;
			}

			prf_unlock_exclusive();

			/* Cleanup memory */
			call_rcu(&po->rcu, pgo_module_free);

			pr_debug("%s: Unregister", mod->name);
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block pgo_module_nb = {
	.notifier_call = pgo_module_notifier
};

/* Create debugfs entries. */
static int __init pgo_init(void)
{
	directory = debugfs_create_dir("pgo", NULL);
	if (!directory)
		goto err_remove;

	/* Setup vmlinux profiler object */
	memset(&prf_vmlinux, 0, sizeof(prf_vmlinux));

	prf_vmlinux.name = "vmlinux";
	prf_vmlinux.data = __llvm_prf_data_start;
	prf_vmlinux.data_num =
		prf_get_count(__llvm_prf_data_start, __llvm_prf_data_end,
			      sizeof(__llvm_prf_data_start[0]));

	prf_vmlinux.cnts = __llvm_prf_cnts_start;
	prf_vmlinux.cnts_num =
		prf_get_count(__llvm_prf_cnts_start, __llvm_prf_cnts_end,
			      sizeof(__llvm_prf_cnts_start[0]));

	prf_vmlinux.names = __llvm_prf_names_start;
	prf_vmlinux.names_num =
		prf_get_count(__llvm_prf_names_start, __llvm_prf_names_end,
			      sizeof(__llvm_prf_names_start[0]));

	prf_vmlinux.vnds_num =
		prf_get_count(__llvm_prf_vnds_start, __llvm_prf_vnds_end,
			      sizeof(__llvm_prf_vnds_start[0]));

	/* Init the vmlinux per-cpu entries */
	prf_vmlinux.pcpu = pgo_percpu_init(&prf_vmlinux);
	if (!prf_vmlinux.pcpu)
		goto err_remove;

	/* Enable profiling. */
	prf_lock_exclusive();
	list_add_tail_rcu(&prf_vmlinux.link, &prf_list);
	prf_unlock_exclusive();

	/* Show notice why the system slower: */
	pr_info("Clang PGO instrumentation is active");

	/* Register module notifer. */
	register_module_notifier(&pgo_module_nb);

	return 0;

err_remove:
	pr_err("initialization failed\n");
	return -EIO;
}

/* Remove debugfs entries. */
static void __exit pgo_exit(void)
{
	debugfs_remove_recursive(directory);
}

module_init(pgo_init);
module_exit(pgo_exit);
