/*
 * Copyright (c) 2013-2015, 2017-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_MSM_AUDIO_ION_H
#define _LINUX_MSM_AUDIO_ION_H
#include <dsp/q6asm-v2.h>
#include <sound/pcm.h>
#include <linux/msm_ion.h>
#include <linux/dma-mapping.h>

enum {
	HLOS_TO_ADSP = 1,
	ADSP_TO_HLOS,
};
#define VMID_CP_ADSP_SHARED 33
enum {
	MSM_AUDIO_ION_INV_CACHES = 0,
	MSM_AUDIO_ION_CLEAN_CACHES,
};

int msm_audio_ion_alloc(void **handle, size_t bufsz,
			dma_addr_t *paddr, size_t *pa_len, void **vaddr);

int msm_audio_ion_import(void **handle, int fd,
			unsigned long *ionflag, size_t bufsz,
			dma_addr_t *paddr, size_t *pa_len, void **vaddr);
int msm_audio_ion_free(void *handle);
int msm_audio_ion_mmap(struct audio_buffer *abuff, struct vm_area_struct *vma);
int msm_audio_ion_cache_operations(struct audio_buffer *abuff, int cache_op);

u32 msm_audio_populate_upper_32_bits(dma_addr_t pa);
int msm_audio_ion_get_smmu_info(struct device **cb_dev, u64 *smmu_sid);

int msm_audio_ion_dma_map(dma_addr_t *phys_addr, dma_addr_t *iova_base,
			u32 size, enum dma_data_direction dir);
int msm_audio_ion_phys_assign(void **mem_hdl, int fd, dma_addr_t *paddr,
			      size_t *pa_len, u8 assign_type, int id);
int msm_audio_ion_phys_free(void *mem_hdl, dma_addr_t *paddr,
			size_t *pa_len, u8 assign_type, int id, int key);
bool msm_audio_is_hypervisor_supported(void);
#endif /* _LINUX_MSM_AUDIO_ION_H */
