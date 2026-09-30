/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (c) 2023 Arm Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM Cortex-M wrapper for ISRs with parameter
 *
 * Wrapper installed in vector table for handling dynamic interrupts that accept
 * a parameter.
 */
#include <zephyr/kernel.h>
#include <zephyr/pm/dbg_pm_wake.h>
#include <zephyr/irq.h>
#include <zephyr/pm/pm.h>
#include <cmsis_core.h>

/**
 *
 * @brief Wrapper around ISRs when inserted in software ISR table
 *
 * When inserted in the vector table, _isr_wrapper() demuxes the ISR table
 * using the running interrupt number as the index, and invokes the registered
 * ISR with its corresponding argument. When returning from the ISR, it
 * determines if a context switch needs to happen (see documentation for
 * z_arm_pendsv()) and pends the PendSV exception if so: the latter will
 * perform the context switch itself.
 *
 */

volatile uint32_t dbg_wake_irq_count[CONFIG_NUM_IRQS];
volatile uint32_t dbg_wake_irq_total;
volatile int32_t dbg_wake_irq_last = -1;
 
void _isr_wrapper(void)
{
#ifdef CONFIG_PM
	bool dbg_sti_external_candidate = false;
	int32_t dbg_sti_candidate_irq = -1;
#endif

#ifdef CONFIG_TRACING_ISR
	sys_trace_isr_enter();
#endif

#ifdef CONFIG_PM
	unsigned int key = arch_irq_lock();

	if (_kernel.idle != 0) {
		uint32_t vector = __get_IPSR();

		if (vector >= 16U) {
			uint32_t irq = vector - 16U;

			if (irq < CONFIG_NUM_IRQS) {
				dbg_wake_irq_count[irq]++;
				dbg_wake_irq_total++;
				dbg_wake_irq_last = (int32_t)irq;
			}

			if (dbg_pm_sti_armed) {
				dbg_sti_external_candidate = true;
				dbg_sti_candidate_irq = (int32_t)irq;
			}
		}

		_kernel.idle = 0;
		pm_system_resume();
	}

	arch_irq_unlock(key);
#endif

#if defined(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER)
	int32_t irq_number = z_soc_irq_get_active();
#else
	int32_t irq_number = __get_IPSR();
#endif

	irq_number -= 16;

	const struct _isr_table_entry *entry =
		&_sw_isr_table[irq_number];

	(entry->isr)(entry->arg);

#ifdef CONFIG_PM
	if (dbg_sti_external_candidate && dbg_pm_sti_armed) {
		dbg_pm_wake.external_irq_wakes++;
		dbg_pm_wake.last_external_irq =
			dbg_sti_candidate_irq;

		dbg_pm_sti_armed = 0;
	}
#endif

#if defined(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER)
	z_soc_irq_eoi(irq_number);
#endif

#ifdef CONFIG_TRACING_ISR
	sys_trace_isr_exit();
#endif

	z_arm_exc_exit();
}