// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Samuel Pascua <pascua.samuel.14@gmail.com>.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/rom_notifier.h>

#define BUF_SIZE 12288
#define BUILDPROP "/system/build.prop"

bool is_aosp __read_mostly = false;

static char *get_android_prop(char *buf, const char *prop) {
	char *token, *keyval, *tmp;

	while ((token = strsep(&buf, "\n")) != NULL) {
		keyval = token;
		tmp = strsep(&keyval, "=");

		if (!tmp)
			continue;

		if (strcmp(tmp, prop) == 0) {
			if (!keyval)
				return NULL;
			return kstrdup(keyval, GFP_KERNEL);
		}
	}
	return NULL;
}

static int read_file(const char *filename, char **out_buf) {
	struct file *f;
	char *buf;
	mm_segment_t old_fs;
	loff_t pos = 0;
	ssize_t bytes_read;
	int ret = 0;

	buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	f = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(f)) {
		ret = -PTR_ERR(f);
		kfree(buf);
		return ret;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	bytes_read = kernel_read(f, buf, BUF_SIZE - 1, &pos);
	if (bytes_read < 0) {
		ret = bytes_read;
		kfree(buf);
		buf = NULL;
	} else {
		buf[bytes_read] = '\0';
		*out_buf = buf;
	}

	set_fs(old_fs);
	filp_close(f, NULL);

	return ret;
}

void rom_notifier_init(void) {
	char *buf = NULL;
	char *oneui_prop = NULL;
	char *device_prop = NULL;
	int ret;

	ret = read_file(BUILDPROP, &buf);
	if (ret)
		return;

	oneui_prop = get_android_prop(buf, "ro.build.version.oneui");
	device_prop = get_android_prop(buf, "ro.product.system.device");

	if (!oneui_prop || (device_prop && strcmp(device_prop, "generic") != 0))
		is_aosp = true;

	kfree(oneui_prop);
	kfree(device_prop);
	kfree(buf);
}
EXPORT_SYMBOL(rom_notifier_init);
