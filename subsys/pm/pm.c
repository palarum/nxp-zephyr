/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/dbg_pm_wake.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <zephyr/tracing/tracing.h>

#include "pm_stats.h"
#include "device_system_managed.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pm, CONFIG_PM_LOG_LEVEL);

static ATOMIC_DEFINE(z_post_ops_required, CONFIG_MP_MAX_NUM_CPUS);
static sys_slist_t pm_notifiers = SYS_SLIST_STATIC_INIT(&pm_notifiers);

/* Convert exit-latency-us to ticks using specified method. */
#define EXIT_LATENCY_US_TO_TICKS(us)						    \
	IS_ENABLED(CONFIG_PM_PREWAKEUP_CONV_MODE_NEAR) ? k_us_to_ticks_near32(us) : \
	IS_ENABLED(CONFIG_PM_PREWAKEUP_CONV_MODE_CEIL) ? k_us_to_ticks_ceil32(us) : \
		k_us_to_ticks_floor32(us)

/* State pointers which are set to NULL indicate ACTIVE state. */
static const struct pm_state_info *z_cpus_pm_state[CONFIG_MP_MAX_NUM_CPUS];
static const struct pm_state_info *z_cpus_pm_forced_state[CONFIG_MP_MAX_NUM_CPUS];

static struct k_spinlock pm_forced_state_lock;
static struct k_spinlock pm_notifier_lock;

/*
 * Function called to notify when the system is entering / exiting a
 * power state
 */
static inline void pm_state_notify(bool entering_state)
{
	struct pm_notifier *notifier;
	k_spinlock_key_t pm_notifier_key;
	union {
		void (*without_substate)(enum pm_state state);
		void (*with_substate)(enum pm_state state, uint8_t substate_id);
	} callback;

	pm_notifier_key = k_spin_lock(&pm_notifier_lock);
	SYS_SLIST_FOR_EACH_CONTAINER(&pm_notifiers, notifier, _node) {
		if (entering_state) {
			/* should be equivalent to also setting the "with substate" */
			callback.without_substate = notifier->state_entry;
		} else {
			/* should be equivalent to also setting the "with substate" */
			callback.without_substate = notifier->state_exit;
		}

		if (callback.with_substate && notifier->report_substate) {
			callback.with_substate(z_cpus_pm_state[CPU_ID]->state,
					       z_cpus_pm_state[CPU_ID]->substate_id);
		} else if (callback.without_substate) {
			callback.without_substate(z_cpus_pm_state[CPU_ID]->state);
		} else {
			/* intentionally empty */
		};
	}
	k_spin_unlock(&pm_notifier_lock, pm_notifier_key);
}

static inline int32_t ticks_expiring_sooner(int32_t ticks1, int32_t ticks2)
{
	/*
	 * Ticks are relative numbers that defines the number of ticks
	 * until the next event.
	 * Its maximum value is K_TICKS_FOREVER ((uint32_t)-1) which is -1
	 * when we cast it to (int32_t)
	 * We need to find out which one is the closest
	 */

	__ASSERT(ticks1 >= -1, "ticks1 has unexpected negative value");
	__ASSERT(ticks2 >= -1, "ticks2 has unexpected negative value");

	if (ticks1 == K_TICKS_FOREVER) {
		return ticks2;
	}
	if (ticks2 == K_TICKS_FOREVER) {
		return ticks1;
	}
	/* At this step ticks1 and ticks2 are positive */
	return MIN(ticks1, ticks2);
}

void pm_system_resume(void)
{
	uint8_t id = CPU_ID;

	/*
	 * This notification is called from the ISR of the event
	 * that caused exit from kernel idling after PM operations.
	 *
	 * Some CPU low power states require enabling of interrupts
	 * atomically when entering those states. The wake up from
	 * such a state first executes code in the ISR of the interrupt
	 * that caused the wake. This hook will be called from the ISR.
	 * For such CPU LPS states, do post operations and restores here.
	 * The kernel scheduler will get control after the ISR finishes
	 * and it may schedule another thread.
	 */
	if (atomic_test_and_clear_bit(z_post_ops_required, id)) {
#ifdef CONFIG_PM_DEVICE_SYSTEM_MANAGED
		if (atomic_add(&_cpus_active, 1) == 0) {
			if ((z_cpus_pm_state[id]->state != PM_STATE_RUNTIME_IDLE) &&
					!z_cpus_pm_state[id]->pm_device_disabled) {
				pm_resume_devices();
			}
		}
#endif
		pm_state_exit_post_ops(z_cpus_pm_state[id]->state,
				       z_cpus_pm_state[id]->substate_id);
		pm_state_notify(false);
#ifdef CONFIG_SYS_CLOCK_EXISTS
		sys_clock_idle_exit();
#endif /* CONFIG_SYS_CLOCK_EXISTS */
		z_cpus_pm_state[id] = NULL;
	}
}

bool pm_state_force(uint8_t cpu, const struct pm_state_info *info)
{
	k_spinlock_key_t key;

	__ASSERT(info->state < PM_STATE_COUNT,
		 "Invalid power state %d!", info->state);

	info = pm_state_get(cpu, info->state, info->substate_id);
	if (info == NULL) {
		/* Return false if the state could not be retrieved */
		return false;
	}

	key = k_spin_lock(&pm_forced_state_lock);
	z_cpus_pm_forced_state[cpu] = info;
	k_spin_unlock(&pm_forced_state_lock, key);

	return true;
}

struct dbg_pm_sample {
	uint32_t suspend_calls;

	int32_t ticks_min;
	int32_t ticks_max;

	uint32_t ticks_lt_10;
	uint32_t ticks_10_49;
	uint32_t ticks_50_59;
	uint32_t ticks_60_99;
	uint32_t ticks_100_999;
	uint32_t ticks_ge_1000;
};

volatile struct dbg_pm_sample dbg_pm_sample = {
	.ticks_min = INT32_MAX,
	.ticks_max = INT32_MIN,
};

struct dbg_pm_selected_sample {
	uint32_t selected_runtime_idle;
	uint32_t selected_suspend_idle;
	uint32_t selected_other;
	uint32_t selected_none;

	uint32_t entered_runtime_idle;
	uint32_t entered_suspend_idle;
	uint32_t entered_other;

	uint32_t suspend_idle_device_fail;

	int32_t last_kernel_ticks;
	int32_t last_events_ticks;
	int32_t last_effective_ticks;

	/* Raw kernel_ticks for actual suspend-to-idle entries */
	uint32_t sti_kernel_forever;
	uint32_t sti_kernel_lt_10;
	uint32_t sti_kernel_10_49;
	uint32_t sti_kernel_50_59;
	uint32_t sti_kernel_60_99;
	uint32_t sti_kernel_100_999;
	uint32_t sti_kernel_ge_1000;

	/* Effective ticks used by PM policy for actual suspend-to-idle entries */
	uint32_t sti_effective_forever;
	uint32_t sti_effective_lt_10;
	uint32_t sti_effective_10_49;
	uint32_t sti_effective_50_59;
	uint32_t sti_effective_60_99;
	uint32_t sti_effective_100_999;
	uint32_t sti_effective_ge_1000;
};

volatile struct dbg_pm_selected_sample dbg_pm_selected;
volatile struct dbg_pm_wake dbg_pm_wake;
volatile uint32_t dbg_pm_sti_armed;

void dbg_pm_reset_counters(void)
{
	memset((void *)&dbg_pm_wake,
	       0,
	       sizeof(dbg_pm_wake));

	dbg_pm_wake.last_external_irq = -1;
	dbg_pm_wake.last_clock_ticks = -1;

	dbg_pm_sti_armed = 0;

	memset((void *)&dbg_pm_selected,
	       0,
	       sizeof(dbg_pm_selected));

	memset((void *)&dbg_pm_sample,
	       0,
	       sizeof(dbg_pm_sample));

	dbg_pm_sample.ticks_min = INT32_MAX;
	dbg_pm_sample.ticks_max = INT32_MIN;
}

bool pm_system_suspend(int32_t kernel_ticks)
{
	dbg_pm_sample.suspend_calls++;

	if (kernel_ticks >= 0) {
		if (kernel_ticks < dbg_pm_sample.ticks_min) {
			dbg_pm_sample.ticks_min = kernel_ticks;
		}

		if (kernel_ticks > dbg_pm_sample.ticks_max) {
			dbg_pm_sample.ticks_max = kernel_ticks;
		}

		if (kernel_ticks < 10) {
			dbg_pm_sample.ticks_lt_10++;
		} else if (kernel_ticks < 50) {
			dbg_pm_sample.ticks_10_49++;
		} else if (kernel_ticks < 60) {
			dbg_pm_sample.ticks_50_59++;
		} else if (kernel_ticks < 100) {
			dbg_pm_sample.ticks_60_99++;
		} else if (kernel_ticks < 1000) {
			dbg_pm_sample.ticks_100_999++;
		} else {
			dbg_pm_sample.ticks_ge_1000++;
		}
	}

	uint8_t id = CPU_ID;
	k_spinlock_key_t key;
	int32_t ticks;
	int32_t events_ticks;
	uint32_t exit_latency_ticks;

	SYS_PORT_TRACING_FUNC_ENTER(pm, system_suspend, kernel_ticks);

	if (!pm_policy_state_any_active() &&
	    (z_cpus_pm_forced_state[id] == NULL)) {

		return false;
	}

	/*
	 * Determine time until the next event.
	 */
	events_ticks = pm_policy_next_event_ticks();
	ticks = ticks_expiring_sooner(kernel_ticks, events_ticks);

#ifdef CONFIG_PM_CUSTOM_TICKS_HOOK
	int32_t custom_ticks = pm_policy_next_custom_ticks();

	ticks = ticks_expiring_sooner(custom_ticks, ticks);
#endif

	/*
	 * Select the PM state.
	 */
	key = k_spin_lock(&pm_forced_state_lock);

	if (z_cpus_pm_forced_state[id] != NULL) {
		z_cpus_pm_state[id] = z_cpus_pm_forced_state[id];
		z_cpus_pm_forced_state[id] = NULL;
	} else {
		z_cpus_pm_state[id] = pm_policy_next_state(id, ticks);
	}

	k_spin_unlock(&pm_forced_state_lock, key);

	/*
	 * Record policy inputs/result.
	 */
	dbg_pm_selected.last_kernel_ticks = kernel_ticks;
	dbg_pm_selected.last_events_ticks = events_ticks;
	dbg_pm_selected.last_effective_ticks = ticks;

	if (z_cpus_pm_state[id] == NULL) {

		dbg_pm_selected.selected_none++;

	} else if (z_cpus_pm_state[id]->state == PM_STATE_RUNTIME_IDLE) {

		dbg_pm_selected.selected_runtime_idle++;

	} else if (z_cpus_pm_state[id]->state == PM_STATE_SUSPEND_TO_IDLE) {

		dbg_pm_selected.selected_suspend_idle++;

	} else {

		dbg_pm_selected.selected_other++;
	}

	if (z_cpus_pm_state[id] == NULL) {
		LOG_DBG("No PM operations done.");

		SYS_PORT_TRACING_FUNC_EXIT(pm,
					   system_suspend,
					   ticks,
					   PM_STATE_ACTIVE);

		return false;
	}

#ifdef CONFIG_PM_DEVICE_SYSTEM_MANAGED
	if (atomic_sub(&_cpus_active, 1) == 1) {

		if ((z_cpus_pm_state[id]->state != PM_STATE_RUNTIME_IDLE) &&
		    !z_cpus_pm_state[id]->pm_device_disabled) {

			if (!pm_suspend_devices()) {

				if (z_cpus_pm_state[id]->state ==
				    PM_STATE_SUSPEND_TO_IDLE) {

					dbg_pm_selected.suspend_idle_device_fail++;
				}

				pm_resume_devices();

				z_cpus_pm_state[id] = NULL;

				(void)atomic_add(&_cpus_active, 1);

				SYS_PORT_TRACING_FUNC_EXIT(pm,
							   system_suspend,
							   ticks,
							   PM_STATE_ACTIVE);

				return false;
			}
		}
	}
#endif

	/*
	 * Account for exit latency when programming the timer.
	 */
	exit_latency_ticks =
		EXIT_LATENCY_US_TO_TICKS(
			z_cpus_pm_state[id]->exit_latency_us);

	if ((exit_latency_ticks > 0) &&
	    (ticks != K_TICKS_FOREVER)) {

		k_spinlock_key_t clock_key = sys_clock_lock();

		sys_clock_set_timeout(
			MAX(0,
			    (int64_t)ticks -
			    (int64_t)exit_latency_ticks),
			true);

		sys_clock_unlock(clock_key);
	}

	/*
	 * Prevent scheduling until the resume notification is complete.
	 */
	k_sched_lock();

	if (IS_ENABLED(CONFIG_PM_STATS)) {
		pm_stats_start();
	}

	/*
	 * Enter power state.
	 */
	pm_state_notify(true);
	atomic_set_bit(z_post_ops_required, id);

	enum pm_state dbg_state =
		z_cpus_pm_state[id]->state;

	/*
	 * Record the state which actually made it through
	 * device PM and is about to be entered.
	 */
	if (dbg_state == PM_STATE_RUNTIME_IDLE) {

		dbg_pm_selected.entered_runtime_idle++;

	} else if (dbg_state == PM_STATE_SUSPEND_TO_IDLE) {

		dbg_pm_selected.entered_suspend_idle++;

		/*
		 * Histogram of raw kernel_ticks.
		 */
		if (kernel_ticks == K_TICKS_FOREVER) {
			dbg_pm_selected.sti_kernel_forever++;

		} else if (kernel_ticks < 10) {
			dbg_pm_selected.sti_kernel_lt_10++;

		} else if (kernel_ticks < 50) {
			dbg_pm_selected.sti_kernel_10_49++;

		} else if (kernel_ticks < 60) {
			dbg_pm_selected.sti_kernel_50_59++;

		} else if (kernel_ticks < 100) {
			dbg_pm_selected.sti_kernel_60_99++;

		} else if (kernel_ticks < 1000) {
			dbg_pm_selected.sti_kernel_100_999++;

		} else {
			dbg_pm_selected.sti_kernel_ge_1000++;
		}

		/*
		 * Histogram of effective ticks actually presented
		 * to the PM policy.
		 */
		if (ticks == K_TICKS_FOREVER) {
			dbg_pm_selected.sti_effective_forever++;

		} else if (ticks < 10) {
			dbg_pm_selected.sti_effective_lt_10++;

		} else if (ticks < 50) {
			dbg_pm_selected.sti_effective_10_49++;

		} else if (ticks < 60) {
			dbg_pm_selected.sti_effective_50_59++;

		} else if (ticks < 100) {
			dbg_pm_selected.sti_effective_60_99++;

		} else if (ticks < 1000) {
			dbg_pm_selected.sti_effective_100_999++;

		} else {
			dbg_pm_selected.sti_effective_ge_1000++;
		}

	} else {

		dbg_pm_selected.entered_other++;
	}

	/*
	 * Arm wake attribution immediately before entering
	 * suspend-to-idle.
	 */
	bool dbg_is_sti =
		(dbg_state == PM_STATE_SUSPEND_TO_IDLE);

    if (dbg_is_sti) {
    	dbg_pm_wake.last_external_irq = -1;
    	dbg_pm_wake.last_clock_ticks = -1;
    
    	dbg_pm_sti_armed = 1;
    }

	/*
	 * Actually enter the selected power state.
	 *
	 * Execution resumes here after the hardware wakes.
	 */
	pm_state_set(z_cpus_pm_state[id]->state,
		     z_cpus_pm_state[id]->substate_id);

    /*
     * We are now immediately after the hardware wake.
     *
     * Interrupts may not have been serviced yet, so inspect
     * Cortex-M pending state here. Do NOT clear
     * dbg_pm_sti_armed here.
     */
    if (dbg_is_sti) {
    	uint32_t icsr = SCB->ICSR;
    
    	uint32_t vector =
    		(icsr & SCB_ICSR_VECTPENDING_Msk) >>
    		SCB_ICSR_VECTPENDING_Pos;
    
    	dbg_pm_wake.sti_exits++;
    
    	dbg_pm_wake.last_icsr = icsr;
    	dbg_pm_wake.last_vector = vector;
    
    	/*
    	 * Cortex-M exception numbers:
    	 *
    	 *  0     = nothing pending
    	 * 14     = PendSV
    	 * 15     = SysTick
    	 * 16+    = external NVIC IRQ
    	 */
    	if (vector == 0U) {
    		dbg_pm_wake.vectpending_none++;
    
    	} else if (vector == 15U) {
    		dbg_pm_wake.vectpending_systick++;
    
    	} else if (vector == 14U) {
    		dbg_pm_wake.vectpending_pendsv++;
    
    	} else if (vector >= 16U) {
    		dbg_pm_wake.vectpending_external++;
    
    		/*
    		 * This is only the VECTPENDING observation.
    		 * It is okay to record it here.
    		 */
    		dbg_pm_wake.last_external_irq =
    			(int32_t)(vector - 16U);
    
    	} else {
    		dbg_pm_wake.vectpending_other++;
    	}
    
    	if ((icsr & SCB_ICSR_PENDSTSET_Msk) != 0U) {
    		dbg_pm_wake.systick_pending_bit++;
    	}
    
    	if ((icsr & SCB_ICSR_ISRPENDING_Msk) != 0U) {
    		dbg_pm_wake.external_pending_bit++;
    	}
    }	

	/*
	 * Wake-up sequence starts here.
	 */
	if (IS_ENABLED(CONFIG_PM_STATS)) {

		pm_stats_stop();

		pm_stats_update(
			z_cpus_pm_state[id] ?
			z_cpus_pm_state[id]->state :
			PM_STATE_ACTIVE);
	}

	pm_system_resume();

	k_sched_unlock();

	SYS_PORT_TRACING_FUNC_EXIT(
		pm,
		system_suspend,
		ticks,
		z_cpus_pm_state[id] ?
			z_cpus_pm_state[id]->state :
			PM_STATE_ACTIVE);

	return true;
}

void pm_notifier_register(struct pm_notifier *notifier)
{
	k_spinlock_key_t pm_notifier_key = k_spin_lock(&pm_notifier_lock);

	sys_slist_append(&pm_notifiers, &notifier->_node);
	k_spin_unlock(&pm_notifier_lock, pm_notifier_key);
}

int pm_notifier_unregister(struct pm_notifier *notifier)
{
	int ret = -EINVAL;
	k_spinlock_key_t pm_notifier_key;

	pm_notifier_key = k_spin_lock(&pm_notifier_lock);
	if (sys_slist_find_and_remove(&pm_notifiers, &(notifier->_node))) {
		ret = 0;
	}
	k_spin_unlock(&pm_notifier_lock, pm_notifier_key);

	return ret;
}

const struct pm_state_info *pm_state_next_get(uint8_t cpu)
{
	static const struct pm_state_info active = {
		.state = PM_STATE_ACTIVE
	};

	return z_cpus_pm_state[cpu] ? z_cpus_pm_state[cpu] : &active;
}
