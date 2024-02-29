// SPDX-License-Identifier: GPL-2.0

#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/poll.h>

static bool psi_enable = true;

static int __init setup_psi(char *str)
{
	return kstrtobool(str, &psi_enable) == 0;
}
__setup("psi=", setup_psi);

static int psi_memory_show(struct seq_file *m, void *v)
{
	return 0;
}

static int psi_memory_open(struct inode *inode, struct file *file)
{
	return single_open(file, psi_memory_show, NULL);
}

static ssize_t psi_memory_write(struct file *file, const char __user *user_buf,
				size_t nbytes, loff_t *ppos)
{
	return 0;
}

static unsigned int psi_fop_poll(struct file *file, poll_table *wait)
{
	return 0;
}

static int psi_fop_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations psi_memory_fops = {
	.open           = psi_memory_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.write          = psi_memory_write,
	.poll           = psi_fop_poll,
	.release        = psi_fop_release,
};

static int __init psi_proc_init(void)
{
	proc_mkdir("pressure", NULL);
	proc_create("pressure/memory", 0, NULL, &psi_memory_fops);

	return 0;
}
late_initcall_sync(psi_proc_init);
