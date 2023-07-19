/*
 * MTP wrapper to switch between default and samsung.
 * By missusing panic_on_oops it's possible to switch between them,
 * so we don't have to create a new proc file.
 *
 * Copyright (C) 2019 Willi Ye
 * Author: Willi Ye <williye97@gmail.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include "f_mtp.h"

static struct usb_function_instance *mtp_alloc_inst(void)
{
    return panic_on_oops == 2 ? alloc_inst_mtp_ptp(true) : alloc_inst_mtp_ptp_samsung(true);
}

static struct usb_function *mtp_alloc(struct usb_function_instance *fi)
{
    return panic_on_oops == 2 ? function_alloc_mtp_ptp(fi, true) : function_alloc_mtp_ptp_samsung(fi, true);
}

DECLARE_USB_FUNCTION_INIT(mtp, mtp_alloc_inst, mtp_alloc);
MODULE_LICENSE("GPL");
