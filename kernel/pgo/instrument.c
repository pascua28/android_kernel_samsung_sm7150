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

#include <asm/sections.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/rcupdate.h>
#include "pgo.h"

/*
 * This mutex protects the profile data serialization
 * and the prf_list structure.
 */
static DEFINE_MUTEX(pgo_mutex);

/*
 * Atomic flag to disable profiler hook.
 * - the profiler hook is disabled during boot
 *   to keep the profiler state consistent until
 *   pgo is fully initialized.
 */
static atomic_t prf_disable = ATOMIC_INIT(1);

void prf_lock_exclusive(void)
{
	/* take pgo_mutex */
	mutex_lock(&pgo_mutex);

	/* disable profiler hook */
	atomic_set(&prf_disable, 1);

	/*
	 * wait for GP:
	 * No cpu may be running with prf_disable == 0
	 */
	synchronize_rcu();
}

void prf_unlock_exclusive(void)
{
	/* enable profiler hook again */
	atomic_set(&prf_disable, 0);

	/* unlock */
	mutex_unlock(&pgo_mutex);
}

/*
 * check if profiler hook is enabled.
 * Must be used within RCU read side lock.
 */
static inline int prf_is_enabled(void)
{
	return atomic_read(&prf_disable) == 0;
}

/*
 * Return prf_object for the llvm_prf_data or NULL
 * if we should not attempt any profiling.
 */
static struct prf_object *find_prf_object(struct llvm_prf_data *p)
{
	struct prf_object *po = NULL;
	struct llvm_prf_data *data_end;

	list_for_each_entry_rcu(po, &prf_list, link) {
		/*
		 * Check that p is within:
		 * [po->data, po->data + prf_data_count(po)] section.
		 */
		data_end = po->data + prf_data_count(po);
		if (memory_contains(po->data, data_end, p, sizeof(*p)))
			goto found;
	}
	/* not found */
	po = NULL;

found:
	return po;
}

static inline unsigned int prf_get_index(const void *_start, const void *_end,
	unsigned int objsize)
{
	unsigned long start = (unsigned long)_start;
	unsigned long end =	(unsigned long)_end;

	return (end - start) / objsize;
}

/*
 * Counts the number of times a target value is seen.
 *
 * Records the target value for the index if not seen before. Otherwise,
 * increments the counter associated w/ the target value.
 */
void __llvm_profile_instrument_target(u64 target_value, void *data, u32 index);
void __llvm_profile_instrument_target(u64 target_value, void *data, u32 index)
{
	struct llvm_prf_data *p = (struct llvm_prf_data *)data;
	struct llvm_prf_value_node **counters;
	struct llvm_prf_value_node *curr;
	struct llvm_prf_value_node *min = NULL;
	struct llvm_prf_value_node *prev = NULL;
	u64 min_count = U64_MAX;
	u8 values = 0;
	unsigned long flags;
	struct prf_object *po;
	struct prf_cpu_object *pco;
	int cpu;
	int bucket;

	rcu_read_lock();

	/* check if profiling is allowed */
	if (!prf_is_enabled())
		goto out_rcu;

	if (!p || !p->values)
		goto out_rcu;

	/* get prf_object */
	po = find_prf_object(p);
	if (!po)
		goto out_rcu;

	/* get index to p within po->pcpu[].data */
	bucket = prf_get_index(po->data, data, sizeof(*p));

	/* get prf_cpu_object */
	preempt_disable();
	cpu = smp_processor_id();
	pco = &po->pcpu[cpu];

	counters = (struct llvm_prf_value_node **)pco->data[bucket].values;
	curr = counters[index];

	while (curr) {
		if (target_value == curr->value) {
			curr->count++;
			goto out_unlock;
		}

		if (curr->count < min_count) {
			min_count = curr->count;
			min = curr;
		}

		prev = curr;
		curr = curr->next;
		values++;
	}

	if (values >= LLVM_INSTR_PROF_MAX_NUM_VAL_PER_SITE) {
		if (!min->count || !(--min->count)) {
			curr = min;
			curr->value = target_value;
			curr->count++;
		}
		goto out_unlock;
	}

	if (WARN_ON_ONCE(pco->current_node >= po->vnds_num))
		goto out_unlock; /* Out of nodes */

	local_irq_save(flags);

	/* reserve the vnode */
	curr = &pco->vnds[pco->current_node++];

	curr->value = target_value;
	curr->count++;

	if (!counters[index])
		WRITE_ONCE(counters[index], curr);
	else if (prev && !prev->next)
		WRITE_ONCE(prev->next, curr);

	local_irq_restore(flags);
out_unlock:
	preempt_enable_no_resched();
out_rcu:
	rcu_read_unlock();
}
EXPORT_SYMBOL(__llvm_profile_instrument_target);

/* Counts the number of times a range of targets values are seen. */
void __llvm_profile_instrument_range(u64 target_value, void *data,
				     u32 index, s64 precise_start,
				     s64 precise_last, s64 large_value)
{
	if (large_value != S64_MIN && (s64)target_value >= large_value)
		target_value = large_value;
	else if ((s64)target_value < precise_start ||
		 (s64)target_value > precise_last)
		target_value = precise_last + 1;

	__llvm_profile_instrument_target(target_value, data, index);
}
EXPORT_SYMBOL(__llvm_profile_instrument_range);

static u64 inst_prof_get_range_rep_value(u64 value)
{
	if (value <= 8)
		/* The first ranges are individually tracked, use it as is. */
		return value;
	else if (value >= 513)
		/* The last range is mapped to its lowest value. */
		return 513;
	else if (hweight64(value) == 1)
		/* If it's a power of two, use it as is. */
		return value;

	/* Otherwise, take to the previous power of two + 1. */
	return ((u64)1 << (64 - __builtin_clzll(value) - 1)) + 1;
}

/*
 * The target values are partitioned into multiple ranges. The range spec is
 * defined in compiler-rt/include/profile/InstrProfData.inc.
 */
void __llvm_profile_instrument_memop(u64 target_value, void *data,
				     u32 counter_index)
{
	u64 rep_value;

	/* Map the target value to the representative value of its range. */
	rep_value = inst_prof_get_range_rep_value(target_value);
	__llvm_profile_instrument_target(rep_value, data, counter_index);
}
EXPORT_SYMBOL(__llvm_profile_instrument_memop);
