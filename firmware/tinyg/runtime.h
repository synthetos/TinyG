/*
 * runtime.h - runtime session family "rt": idle heartbeat and quiescence token
 *
 *	JSON only and RAM only: everything here is off after every reset and is never persisted.
 *	  {"rt":null}               hb, qt and up
 *	  {"rt":{"hb":N}}           heartbeat every N seconds (1-60, 0 off) while the machine is idle
 *	  {"rt":{"qt":"<32 hex>"}}  quiescence token: accepted only while nothing is moving, held or queued;
 *	                            cleared at reset and by any state-changing input (rt_qt_invalidate)
 *	The heartbeat is its own line, {"hb":{"stat":n,"hold":n,"up":n,"qt":...}}: never a response, and only
 *	sent when it can go whole at once. See .analysis/heartbeat-design.md.
 */
#ifndef RUNTIME_H_ONCE
#define RUNTIME_H_ONCE

#define RT_HB_MAX 60						// longest heartbeat interval, seconds
#define RT_FI_MAX 60						// longest feed-limit summary interval, seconds (longer is clamped)
#define RT_QT_LEN 32						// quiescence token: exactly this many hex digits
#define RT_TICKS_PER_SECOND 1024UL			// SysTickTimer counts 10 per 9.77 ms RTC tick

#define RT_ENVELOPE_NONE 0					// json_parser: no rt key
#define RT_ENVELOPE_PURE 1					// rt alone (tid allowed)
#define RT_ENVELOPE_BAD 2					// rt mixed with other keys, or a token write with other rt operations

typedef struct rtSingleton {
	bool allowed;							// the line being executed is a pure {"rt":...} envelope
	bool check_only;						// rt_set_grp's first pass: setters validate without applying
	bool fl_taken;							// this line's feed-limit snapshot is taken (json_parser clears it)
	uint8_t hb;								// heartbeat interval in seconds, 0 = off
	uint32_t hb_quiet_since;				// SysTick time the board last went quiet (or last beat)
	volatile uint8_t qt_valid;				// cleared from interrupts too (flush, job kill)
	char qt[RT_QT_LEN+1];
	volatile uint8_t job;					// host streaming a job with more motion to send (step 3); cleared from interrupts
	uint8_t fg;								// host opted in to fg frames after each fs (step 3); reset clears it
} rtSingleton_t;
extern rtSingleton_t rt;

void rt_init(void);
stat_t rt_heartbeat_callback(void);
void rt_qt_invalidate(void);				// a state-changing input arrived: the token no longer holds

stat_t rt_get_hb(nvObj_t *nv);
stat_t rt_set_hb(nvObj_t *nv);
stat_t rt_get_qt(nvObj_t *nv);
stat_t rt_set_qt(nvObj_t *nv);
stat_t rt_get_up(nvObj_t *nv);
stat_t rt_set_grp(nvObj_t *nv);
stat_t rt_get_flk(nvObj_t *nv);
stat_t rt_get_flm(nvObj_t *nv);
stat_t rt_get_flp(nvObj_t *nv);
stat_t rt_get_flf(nvObj_t *nv);
stat_t rt_get_fi(nvObj_t *nv);
stat_t rt_set_fi(nvObj_t *nv);
stat_t rt_get_fs(nvObj_t *nv);
stat_t rt_feedlimit_callback(void);
stat_t rt_get_job(nvObj_t *nv);
stat_t rt_set_job(nvObj_t *nv);
stat_t rt_get_fg(nvObj_t *nv);
stat_t rt_set_fg(nvObj_t *nv);
void rt_job_end(void);						// reset, % flush, job kill, program end: the host job is over

#endif // End of include guard: RUNTIME_H_ONCE
