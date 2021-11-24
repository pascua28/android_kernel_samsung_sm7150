/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef _PGO_H
#define _PGO_H

#include <linux/rculist.h>

/*
 * Note: These internal LLVM definitions must match the compiler version.
 * See llvm/include/llvm/ProfileData/InstrProfData.inc in LLVM's source code.
 */

#define LLVM_INSTR_PROF_RAW_MAGIC_64	\
		((u64)255 << 56 |	\
		 (u64)'l' << 48 |	\
		 (u64)'p' << 40 |	\
		 (u64)'r' << 32 |	\
		 (u64)'o' << 24 |	\
		 (u64)'f' << 16 |	\
		 (u64)'r' << 8  |	\
		 (u64)129)
#define LLVM_INSTR_PROF_RAW_MAGIC_32	\
		((u64)255 << 56 |	\
		 (u64)'l' << 48 |	\
		 (u64)'p' << 40 |	\
		 (u64)'r' << 32 |	\
		 (u64)'o' << 24 |	\
		 (u64)'f' << 16 |	\
		 (u64)'R' << 8  |	\
		 (u64)129)

#define LLVM_INSTR_PROF_RAW_VERSION 		8
#define LLVM_INSTR_PROF_DATA_ALIGNMENT		8
#define LLVM_INSTR_PROF_IPVK_FIRST		0
#define LLVM_INSTR_PROF_IPVK_LAST		1
#define LLVM_INSTR_PROF_MAX_NUM_VAL_PER_SITE	255

#define LLVM_VARIANT_MASK_IR_PROF	(0x1ULL << 56)
#define LLVM_VARIANT_MASK_CSIR_PROF	(0x1ULL << 57)

/**
 * struct llvm_prf_header - represents the raw profile header data structure.
 * @magic: the magic token for the file format.
 * @version: the version of the file format.
 * @binary_ids_size
 * @data_size: the number of entries in the profile data section.
 * @padding_bytes_before_counters: the number of padding bytes before the
 *   counters.
 * @counters_size: the size in bytes of the LLVM profile section containing the
 *   counters.
 * @padding_bytes_after_counters: the number of padding bytes after the
 *   counters.
 * @names_size: the size in bytes of the LLVM profile section containing the
 *   counters' names.
 * @counters_delta: the beginning of the LLMV profile counters section.
 * @names_delta: the beginning of the LLMV profile names section.
 * @value_kind_last: the last profile value kind.
 */
struct llvm_prf_header {
	u64 magic;
	u64 version;
	u64 binary_ids_size;
	u64 data_size;
	u64 padding_bytes_before_counters;
	u64 counters_size;
	u64 padding_bytes_after_counters;
	u64 names_size;
	u64 counters_delta;
	u64 names_delta;
	u64 value_kind_last;
};

/**
 * struct llvm_prf_data - represents the per-function control structure.
 * @name_ref: the reference to the function's name.
 * @func_hash: the hash value of the function.
 * @counter_ptr: a pointer to the profile counter.
 * @function_ptr: a pointer to the function.
 * @values: the profiling values associated with this function.
 * @num_counters: the number of counters in the function.
 * @num_value_sites: the number of value profile sites.
 */
struct llvm_prf_data {
	const u64 name_ref;
	const u64 func_hash;
	const void *counter_ptr;
	const void *function_ptr;
	void *values;
	const u32 num_counters;
	const u16 num_value_sites[LLVM_INSTR_PROF_IPVK_LAST + 1];
} __aligned(LLVM_INSTR_PROF_DATA_ALIGNMENT);

/**
 * struct llvm_prf_value_node_data - represents the data part of the struct
 *   llvm_prf_value_node data structure.
 * @value: the value counters.
 * @count: the counters' count.
 */
struct llvm_prf_value_node_data {
	u64 value;
	u64 count;
};

/**
 * struct llvm_prf_value_node - represents an internal data structure used by
 *   the value profiler.
 * @value: the value counters.
 * @count: the counters' count.
 * @next: the next value node.
 */
struct llvm_prf_value_node {
	u64 value;
	u64 count;
	struct llvm_prf_value_node *next;
};

/**
 * struct llvm_prf_value_data - represents the value profiling data in indexed
 *   format.
 * @total_size: the total size in bytes including this field.
 * @num_value_kinds: the number of value profile kinds that has value profile
 *   data.
 */
struct llvm_prf_value_data {
	u32 total_size;
	u32 num_value_kinds;
};

/**
 * struct llvm_prf_value_record - represents the on-disk layout of the value
 *   profile data of a particular kind for one function.
 * @kind: the kind of the value profile record.
 * @num_value_sites: the number of value profile sites.
 * @site_count_array: the first element of the array that stores the number
 *   of profiled values for each value site.
 */
struct llvm_prf_value_record {
	u32 kind;
	u32 num_value_sites;
	u8 site_count_array[];
};

#define prf_get_value_record_header_size()		\
	offsetof(struct llvm_prf_value_record, site_count_array)
#define prf_get_value_record_site_count_size(sites)	\
	roundup((sites), 8)
#define prf_get_value_record_size(sites)		\
	(prf_get_value_record_header_size() +		\
	 prf_get_value_record_site_count_size((sites)))

/*
 * struct prf_cpu_object - per-cpu entry for prf_object
 */
struct prf_cpu_object {
	/* work copy of llvm_prf_data */
	struct llvm_prf_data *data;
	/* vnode data */
	struct llvm_prf_value_node *vnds;
	/* index for next free vnode */
	int current_node;

	/* debugfs file of this profile data set */
	int cpu;
	struct dentry *file;
	struct prf_object *obj;
};

/*
 * struct prf_object - profiler data set object
 * The prf_object maintains related information
 * for the profiler hook to operate on and also
 * the related information for serializing the data
 */
struct prf_object {
	struct list_head link;
	struct rcu_head rcu;

	/*
	 * name of this prf_object
	 * refers to struct module->name
	 * or "vmlinux"
	 */
	const char *name;

	/* data provided by the compiler. read-only. */
	struct llvm_prf_data *data;
	int data_num;
	u64 *cnts;
	int cnts_num;
	const char *names;
	int names_num;
	int vnds_num;

	/* percpu profiler data */
	struct prf_cpu_object *pcpu;
};

/*
 * List of profiler objects.
 * - readers must take rcu_read_lock()
 * - updaters must take the prf_lock_exclusive()
 */
extern struct list_head prf_list;

/* Data sections */
extern struct llvm_prf_data __llvm_prf_data_start[];
extern struct llvm_prf_data __llvm_prf_data_end[];

extern u64 __llvm_prf_cnts_start[];
extern u64 __llvm_prf_cnts_end[];

extern char __llvm_prf_names_start[];
extern char __llvm_prf_names_end[];

extern struct llvm_prf_value_node __llvm_prf_vnds_start[];
extern struct llvm_prf_value_node __llvm_prf_vnds_end[];

/*
 * Locking for the profiler data structures.
 * This is needed to ensure exclusive access to profiler data.
 */
extern void prf_lock_exclusive(void);
extern void prf_unlock_exclusive(void);

#define __DEFINE_PRF_OBJ_SIZE(s)                                           \
	static inline unsigned long prf_##s##_size(struct prf_object *po)      \
	{                                                                      \
		return po->s##_num * sizeof(po->s[0]);                         \
	}                                                                      \
	static inline unsigned long prf_##s##_count(struct prf_object *po)     \
	{                                                                      \
		return po->s##_num;                                            \
	}

__DEFINE_PRF_OBJ_SIZE(data);
__DEFINE_PRF_OBJ_SIZE(cnts);
__DEFINE_PRF_OBJ_SIZE(names);

#undef __DEFINE_PRF_OBJ_SIZE

/* count number of items in range */
static inline unsigned int prf_get_count(const void *_start, const void *_end,
					 unsigned int objsize)
{
	unsigned long start = (unsigned long)_start;
	unsigned long end = (unsigned long)_end;

	return roundup(end - start, objsize) / objsize;
}

#endif /* _PGO_H */
