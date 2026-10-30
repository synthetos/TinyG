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
/*
 * Mounted switch: a switch fixed at a physical position. test_hook.steps[] counts every step a
 * motor has made since power-up and is never rewritten (homing, G28.3 and G92 rewrite the machine
 * position and the encoder counts), so a mount stays put in physical space. The main loop drives
 * the pin when the count crosses the threshold, so the switch ISR, homing and limit logic see a real
 * edge without the debugger halting the CPU during motion.
 */
typedef struct testHookMount {
	uint8_t enabled;		// written last by the debugger
	uint8_t motor;			// motor index (0 = motor 1) whose steps place the switch
	int8_t sense;			// +1: closed while steps >= at; -1: closed while steps <= at
	uint8_t closed_action;	// TEST_HOOK_DRIVE_LOW or TEST_HOOK_RELEASE, for the switch type
	uint8_t open_action;
	uint8_t closed;			// firmware-maintained; the debugger writes TEST_HOOK_UNKNOWN to force an update
	uint8_t trips;			// count of open -> closed transitions (wraps)
	int32_t at;				// threshold in physical steps
	int32_t trip_steps;		// physical steps at the last open -> closed transition
} test_mount_t;

#define TEST_HOOK_MOUNTS	8		// switch pair index * 2 + (0 = min, 1 = max)
#define TEST_HOOK_UNKNOWN	0xFF

typedef struct testHookSwitch {
	uint8_t magic;			// TEST_HOOK_MAGIC once initialized (lets the harness detect the hook)
	uint8_t pending;		// set by the debugger, cleared by the firmware after applying
	uint8_t axis;			// switch pair index (0=X, 1=Y, 2=Z, 3=A)
	uint8_t bit;			// SW_MIN_BIT_bp or SW_MAX_BIT_bp
	uint8_t action;			// TEST_HOOK_RELEASE, TEST_HOOK_DRIVE_LOW, TEST_HOOK_INVERT, TEST_HOOK_NORMAL
	uint8_t applied;		// count of applied requests (wraps)
	int32_t steps[MOTORS];	// physical step count per motor (see test_mount_t)
	test_mount_t mount[TEST_HOOK_MOUNTS];
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
