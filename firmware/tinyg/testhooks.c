/*
 * testhooks.c - hardware-in-the-loop test hooks (compiled only with TINYG_TEST_HOOKS)
 */
#include "tinyg.h"
#include "config.h"
#include "hardware.h"
#include "encoder.h"
#include "switch.h"
#include "testhooks.h"
#include <math.h>
#include <util/delay.h>

#ifdef TINYG_TEST_HOOKS

volatile test_hook_t test_hook;
volatile test_plan_t test_plan;

void testhooks_replan_delay()
{
	for (uint16_t us = test_plan.replan_delay_us; us > 0; us--) {
		_delay_us(1);
	}
}

void testhooks_block_started(float length, float head, float body, float tail)
{
	test_plan.starts++;
	if (fabs(head + body + tail - length) > 0.0001 * (1 + length)) {
		test_plan.torn_starts++;
	}
}

void testhooks_init()
{
	test_hook.pending = 0;
	test_hook.applied = 0;
	for (uint8_t i = 0; i < TEST_HOOK_MOUNTS; i++) {
		test_hook.mount[i].enabled = false;
	}
	test_hook.magic = TEST_HOOK_MAGIC;
}

static void _apply(uint8_t axis, uint8_t bit, uint8_t action)
{
	if (axis >= 4) {
		return;
	}
	PORT_t *port = hw.sw_port[axis];
	uint8_t mask = (1 << bit);
	register8_t *pinctrl = &port->PIN0CTRL + bit;
	switch (action) {
		case TEST_HOOK_DRIVE_LOW: { port->OUTCLR = mask; port->DIRSET = mask; break; }
		case TEST_HOOK_RELEASE:   { port->DIRCLR = mask; break; }
		case TEST_HOOK_INVERT:    { *pinctrl |= PORT_INVEN_bm; break; }
		case TEST_HOOK_NORMAL:    { *pinctrl &= ~PORT_INVEN_bm; break; }
	}
}

static void _service_mounts()
{
	for (uint8_t i = 0; i < TEST_HOOK_MOUNTS; i++) {
		volatile test_mount_t *m = &test_hook.mount[i];
		if (!m->enabled || m->motor >= MOTORS) {
			continue;
		}
		uint8_t sreg = SREG;			// steps[] and steps_run are updated by the stepper ISRs
		cli();
		int32_t steps = test_hook.steps[m->motor] + en.en[m->motor].steps_run;
		SREG = sreg;
		if (m->closed == TEST_HOOK_UNKNOWN) {
			m->min_steps = m->max_steps = steps;
		} else {
			if (steps < m->min_steps) { m->min_steps = steps; }
			if (steps > m->max_steps) { m->max_steps = steps; }
		}
		uint8_t closed = (m->sense > 0) ? (steps >= m->at) : (steps <= m->at);
		if (closed == m->closed) {
			continue;
		}
		if (closed && m->closed == false) {
			m->trips++;
			m->trip_steps = steps;
		}
		m->closed = closed;
		_apply(i >> 1, (i & 1) ? SW_MAX_BIT_bp : SW_MIN_BIT_bp, closed ? m->closed_action : m->open_action);
	}
}

stat_t testhooks_callback()
{
	_service_mounts();
	if (!test_hook.pending) {
		return (STAT_OK);
	}
	_apply(test_hook.axis, test_hook.bit, test_hook.action);
	test_hook.applied++;
	test_hook.pending = 0;
	return (STAT_OK);
}

#endif // TINYG_TEST_HOOKS
