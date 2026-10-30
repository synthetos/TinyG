/*
 * testhooks.h - hardware-in-the-loop test hooks (compiled only with TINYG_TEST_HOOKS)
 *
 * Never part of a release build. See tests/hil/tinyg_hil/switches.py.
 */
#ifndef TESTHOOKS_H_ONCE
#define TESTHOOKS_H_ONCE

#ifdef TINYG_TEST_HOOKS

/*
 * Switch mailbox: the debugger writes a request while the CPU is halted; the running firmware
 * applies it from the main loop, so the port's edge detector and the switch ISR see a real change.
 */
typedef struct testHookSwitch {
	uint8_t magic;			// TEST_HOOK_MAGIC once initialized (lets the harness detect the hook)
	uint8_t pending;		// set by the debugger, cleared by the firmware after applying
	uint8_t axis;			// switch pair index (0=X, 1=Y, 2=Z, 3=A)
	uint8_t bit;			// SW_MIN_BIT_bp or SW_MAX_BIT_bp
	uint8_t action;			// TEST_HOOK_RELEASE, TEST_HOOK_DRIVE_LOW, TEST_HOOK_INVERT, TEST_HOOK_NORMAL
	uint8_t applied;		// count of applied requests (wraps)
} test_hook_t;

#define TEST_HOOK_MAGIC		0x5A
#define TEST_HOOK_RELEASE	0		// pin back to input (board pull-up)
#define TEST_HOOK_DRIVE_LOW	1		// pin as output driving 0
#define TEST_HOOK_INVERT	2		// set INVEN on the pin (inverted input sense)
#define TEST_HOOK_NORMAL	3		// clear INVEN

extern volatile test_hook_t test_hook;

void testhooks_init(void);
stat_t testhooks_callback(void);

#endif // TINYG_TEST_HOOKS
#endif // TESTHOOKS_H_ONCE
