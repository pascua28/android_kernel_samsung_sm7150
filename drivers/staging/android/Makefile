ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_SAMSUNG_FREECESS)		+= freecess.o
obj-$(CONFIG_SAMSUNG_FREECESS)		+= freecess_pkg.o
obj-$(CONFIG_SAMSUNG_LMKD_DEBUG)		+= lmkd_debug.o
lmk_tng-y := lowmemorykiller_tng.o lowmemorykiller_tasks.o \
             lowmemorykiller_oom.o lowmemorykiller_vmpressure.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER_TNG) += lmk_tng.o psi.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER_STATS) += lowmemorykiller_stats.o
