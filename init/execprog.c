// SPDX-License-Identifier: GPL-2.0
/*
 * init/execprog.c
 *
 * Copyright (c) 2019 Park Ju Hyung(arter97)
 *
 * This module injects a binary from the kernel's __init storage to the
 * userspace and executes it.
 *
 * This is useful in cases like Android, where you would want to avoid changing
 * a physical block device to execute a program just for your kernel.
 */

#define pr_fmt(fmt) "execprog: " fmt

#include <linux/buffer_head.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/module.h>
#include <linux/namei.h>
#include <linux/rcutree.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

/*
 * Set CONFIG_EXECPROG_WAIT_FOR carefully.
 *
 * It's assumed that if CONFIG_EXECPROG_WAIT_FOR is ready,
 * the path to CONFIG_EXECPROG_DST is also ready for writing.
 */

// Do we really need these to be configurable?
#define DELAY_MS 10
#define SAVE_DST CONFIG_EXECPROG_DST
#define WAIT_FOR CONFIG_EXECPROG_WAIT_FOR

// Hex-converted scripts/prime.sh
#define EXECPROG "../binaries/execprog.i"
u8 execprog_file[] = {
	#include EXECPROG
};

static struct delayed_work execprog_work;

static int write_file(char *filename, unsigned char *data, int length, int rights) {
	struct file *fp;
	int ret = 0;
	loff_t pos = 0;
	int bytes_remaining = length;

	if (!filename || !data || length <= 0)
		return -EINVAL;

	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, rights);
	if (IS_ERR(fp)) {
		ret = PTR_ERR(fp);
		return ret;
	}

	if (!fp->f_op) {
		fput(fp);
		return -EOPNOTSUPP;
	}

	while (bytes_remaining > 0) {
		ret = kernel_write(fp, data, bytes_remaining, &pos);
		if (ret <= 0) {
			if (ret < 0)
				pr_err("Write error: %s (err=%d)\n", filename, ret);
			break;
		}

		data += ret;
		bytes_remaining -= ret;
	}

	if (vfs_fsync(fp, 1) < 0)
		pr_warn("Sync failed: %s\n", filename);

	fput(fp);

	if (bytes_remaining > 0) {
		return ret < 0 ? ret : -EIO;
	}

	pr_info("Successfully wrote: %s (%d bytes)\n", filename, length);
	return 0;
}

static int write_files(void) {
	int rc = 0;
	rc = write_file(SAVE_DST, execprog_file, sizeof(execprog_file), 0755);
	if (rc)
		goto exit;

	exit:
		return rc;
}

static void execprog_worker(struct work_struct *work)
{
	struct path path;
	char *argv[] = { SAVE_DST, NULL };
	int ret, i = 0;

	pr_info("worker started\n");

	pr_info("waiting for %s\n", WAIT_FOR);
	while (kern_path(WAIT_FOR, LOOKUP_FOLLOW, &path))
		msleep(DELAY_MS);

	pr_info("saving binary to userspace\n");
	do {
		ret = write_files();

		i++;
	} while (ret && i <= 100);

	i = 0;
	do {
		/*
		 * Wait for RCU grace period to end for the file to close properly.
		 * call_usermodehelper() will return -ETXTBSY without this barrier.
		 */
		rcu_barrier();
		msleep(10);

		ret = call_usermodehelper(argv[0], argv, NULL, UMH_WAIT_EXEC);
		/*
		 * With APEX, a -ENOENT might be returned since libc.so is missing.
		 */
		if (ret)
			pr_err("execution failed with return code: %d\n", ret);
		else
			pr_info("execution finished\n");
		i++;
	} while (ret && i <= 100);
}

static int __init execprog_init(void)
{
	INIT_DELAYED_WORK(&execprog_work, execprog_worker);
	queue_delayed_work(system_freezable_power_efficient_wq,
			&execprog_work, DELAY_MS);

	return 0;
}

static void __exit execprog_exit(void)
{
	return;
}

module_init(execprog_init);
module_exit(execprog_exit);

MODULE_DESCRIPTION("Userspace binary injector");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Park Ju Hyung");
