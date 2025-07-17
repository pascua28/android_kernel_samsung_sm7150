#
# Makefile for the Imagis touch driver
#

# Each configuration option enables a list of files.

obj-$(CONFIG_TOUCHSCREEN_IST40XX) += ist40xx.o ist40xx_misc.o ist40xx_sys.o ist40xx_update.o ist40xx_sec.o ist40xx_cmcs.o

ifeq ($(CONFIG_POLLY_CLANG),y)
ORIG_CFLAGS := $(KBUILD_CFLAGS)
KBUILD_CFLAGS = $(subst $(POLLY_CFLAGS), , $(ORIG_CFLAGS))
endif
