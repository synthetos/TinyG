/*
 * testhooks.c - hardware-in-the-loop test hooks (compiled only with TINYG_TEST_HOOKS)
 */
#include "tinyg.h"
#include "config.h"
#include "hardware.h"
#include "testhooks.h"

#ifdef TINYG_TEST_HOOKS

volatile test_hook_t test_hook;

void testhooks_init()
{
	test_hook.pending = 0;
	test_hook.applied = 0;
	test_hook.magic = TEST_HOOK_MAGIC;
}

stat_t testhooks_callback()
{
	if (!test_hook.pending) {
		return (STAT_OK);
	}
	if (test_hook.axis < 4) {
		PORT_t *port = hw.sw_port[test_hook.axis];
		uint8_t mask = (1 << test_hook.bit);
		register8_t *pinctrl = &port->PIN0CTRL + test_hook.bit;
		switch (test_hook.action) {
			case TEST_HOOK_DRIVE_LOW: { port->OUTCLR = mask; port->DIRSET = mask; break; }
			case TEST_HOOK_RELEASE:   { port->DIRCLR = mask; break; }
			case TEST_HOOK_INVERT:    { *pinctrl |= PORT_INVEN_bm; break; }
			case TEST_HOOK_NORMAL:    { *pinctrl &= ~PORT_INVEN_bm; break; }
		}
	}
	test_hook.applied++;
	test_hook.pending = 0;
	return (STAT_OK);
}

#endif // TINYG_TEST_HOOKS
