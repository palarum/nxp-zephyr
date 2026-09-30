/* dbg_pm_wake.h */

#ifndef DBG_PM_WAKE_H
#define DBG_PM_WAKE_H

#include <stdint.h>

struct dbg_pm_wake {
	uint32_t sti_exits;

	uint32_t vectpending_none;
	uint32_t vectpending_systick;
	uint32_t vectpending_pendsv;
	uint32_t vectpending_external;
	uint32_t vectpending_other;

	uint32_t systick_pending_bit;
	uint32_t external_pending_bit;

	uint32_t os_timer_irq_wakes;
	uint32_t sys_clock_wakes;
	uint32_t external_irq_wakes;

	uint32_t last_icsr;
	uint32_t last_vector;
	int32_t last_external_irq;
	int32_t last_clock_ticks;
};

extern volatile struct dbg_pm_wake dbg_pm_wake;
extern volatile uint32_t dbg_pm_sti_armed;

void dbg_pm_reset_counters(void);

#endif