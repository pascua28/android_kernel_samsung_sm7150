// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Samuel Pascua <pascua.samuel.14@gmail.com>.
 */

#include <linux/init.h>
#include <linux/rom_notifier.h>
#include <linux/string.h>

bool is_aosp __read_mostly = false;
static int __init parse_aosp(char *str)
{
	if (!strncmp(str, "1", 1))
		is_aosp = true;

	return 0;
}
__setup("android.is_aosp=", parse_aosp);
