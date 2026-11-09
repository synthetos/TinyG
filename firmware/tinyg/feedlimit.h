/*
 * feedlimit.h - feed-limit advisor, step 1: why the running move's planned speed is below its F
 *
 *	mp_aline() tags each block, once, with the constraints known when it is created (FL_STATIC_TAGS): the
 *	axis cap, a corner limit on its entry (only while the move before it is still queued; E196) and G61
 *	exact stop. The tags never change on replanning. When the exec starts a block it stages the tags with
 *	the plan it took (cruise, F, cruise_vmax, naiive_move_time) in the slot that is not current; the step
 *	loader makes that slot current when the block's first segment loads, so the report describes the move
 *	stepping now, not the one being prepared. The profile tags (FL_PROFILE_TAGS) are derived from the
 *	staged plan when a report reads it. Read through the rt family ({rt:{flm:null}} and friends, runtime.c).
 *	A tag means the constraint took part in planning the block, not that it limits every instant of it;
 *	an absent tag is not proof the constraint was absent (no corner tag when the previous block had
 *	already been freed). See .analysis/feed-advisor-design.md.
 */
#ifndef FEEDLIMIT_H_ONCE
#define FEEDLIMIT_H_ONCE

// constraint tags (block and report)
#define FL_AXIS_CAP		0x01		// an axis's feedrate max (fr) makes the move slower than its F
#define FL_JUNCTION		0x02		// entry limited by the corner with the previous move
#define FL_EXACT_STOP	0x04		// G61 exact stop: entry and exit planned to zero
#define FL_SHORT		0x08		// too short to reach its cruise speed between entry and exit
#define FL_MIN_SEGMENT	0x10		// single-segment short-block profile (the F and B'' cases)
#define FL_COMMAND		0x20		// a queued command (S, M3-M5, M7-M9, ...) next to the move makes it rest at that end (step 3)
#define FL_STATIC_TAGS	(FL_AXIS_CAP | FL_JUNCTION | FL_EXACT_STOP)	// set once at creation (mp_aline)
#define FL_PROFILE_TAGS	(FL_SHORT | FL_MIN_SEGMENT)	// derived from the plan when a report reads it

// report kinds
#define FL_KIND_NONE	0			// no move running (idle, hold, dwell, command)
#define FL_KIND_FEED	1			// G1/G2/G3 feed move
#define FL_KIND_RAPID	2			// G0 traverse

// fl_staged and st_pre.fl_load: what a segment load does to the current report
#define FL_LOAD_SAME	0			// another segment of the running block (or nothing staged)
#define FL_LOAD_SLOT0	1			// first segment of the block staged in fl_slot[0]: make it current
#define FL_LOAD_SLOT1	2			// first segment of the block staged in fl_slot[1]
#define FL_LOAD_NONE	3			// a dwell or command: no move running
#define FL_CUR_NONE		2			// fl_cur when no move is current (fl_cur = FL_LOAD_* - 1)

// The exec stages each block in one of two slots with plain copies only; the step loader makes it current
// with the block's first segment by publishing the slot's index, so the interrupt copies nothing. The
// float work that turns the plan into tags (fl_profile_tags()) runs when a report reads it.
typedef struct flBlock {
	uint8_t tags;					// FL_STATIC_TAGS of the block
	uint8_t kind;					// FL_KIND_FEED or FL_KIND_RAPID
	uint8_t feed_ok;				// F is a path speed in mm/min (a G94 feed move: not G0, not G93)
	uint8_t pct_ok;					// planned-cruise percent is meaningful (feed_ok, linear XYZ)
	float cruise;					// planned cruise velocity, mm/min
	float feed;						// the block's F, mm/min (requested path speed)
	float cruise_vmax;				// cruise asked for after axis caps, mm/min
	float naiive_move_time;			// length over the average of the requested entry and exit (plan_zoid.c), minutes
	uint32_t line;					// the block's line number (runtime: what is executing)
	uint8_t all_tags;				// static and profile tags, set by the exec while accounting is on (fl events)
} flBlock_t;

// true if an axis takes part in a move: its unit-vector component is not +0 or -0 (no float compare)
static inline uint8_t fl_axis_moves(float unit)
{
	uint32_t bits;
	memcpy(&bits, &unit, sizeof(bits));
	return ((bits << 1) != 0);
}

extern flBlock_t fl_slot[2];		// staged blocks: the current one and the next
extern uint8_t fl_stage;			// exec only: the slot it filled last
extern volatile uint8_t fl_staged;	// FL_LOAD_SLOTn for the next prepared segment, or FL_LOAD_SAME
extern volatile uint8_t fl_cur;		// current slot, or FL_CUR_NONE; set by the step loader (HI interrupt)

// Accounting (step 2), only while a host has it on ({rt:{fi:N}}, runtime.c). Each prepared ALINE segment
// carries a record (st_pre.fl_rec/fl_um); the loader installs it when the segment loads and retires it when
// the segment has finished, adding into fl_pend[fl_bank]. The main loop flips fl_bank with interrupts masked
// (a few cycles), then reads and clears the other bank with them enabled: the loader only writes fl_bank.
#define FL_CAUSES		7			// fs "c" order: axis cap, corner, exact stop, short, single segment, other, command
#define FL_REC_OTHER	0x40		// record: below F with no tag (ramps, end of path, downstream braking)
#define FL_C_OTHER		5			// c[] index of FL_REC_OTHER (bits 0x01..0x10 are c[0..4])
#define FL_C_COMMAND	6			// c[] index of FL_COMMAND (its bit 0x20 predates step 3's c order)
#define FL_CLASSES		3			// command boundary classes (fs "b" order)
#define FL_CLS_SPINDLE	0			// S, M3, M4, M5
#define FL_CLS_COOLANT	1			// M7, M8, M9
#define FL_CLS_OTHER	2			// any other queued command
#define FL_REC_ELIGIBLE	0x80		// record: a G94 linear XYZ feed segment prepared outside a hold

typedef struct flPending {
	uint32_t ticks[FL_CAUSES + 1];	// DDA ticks: [0..5] below F with that cause, [FL_CAUSES] all eligible
	uint32_t um;					// commanded length of all eligible segments, micrometres
	uint32_t cls_ticks[FL_CLASSES];	// DDA ticks below F of command-tagged segments, by the command's class
	uint32_t breaks[FL_CLASSES];	// motion breaks at command boundaries while a host job runs, by class
	uint8_t wrapped;				// a pending total wrapped (the main loop could not fold in time)
} flPending_t;

extern volatile uint8_t fl_on;		// crediting enabled (main loop sets it; exec and loader read it)
extern volatile flPending_t fl_pend[2];	// loader adds into fl_pend[fl_bank]; the main loop takes the other
extern volatile uint8_t fl_bank;
extern uint16_t fl_um_q16;			// exec: sub-micrometre remainder (1/65536 um) carried to the next segment
extern volatile uint8_t fl_activity;	// exec: a block started since the last final summary (main loop clears it)
extern volatile uint8_t fl_job_moved;	// exec: a block started during the host job (step 3 gaps; main loop clears it)
extern volatile uint8_t fl_underrun;	// exec: found nothing to run during a job, outside a hold (main loop takes it)

void fl_stage_block(const struct mpBuffer *bf);
void fl_segment(float velocity, float length);
uint8_t fl_profile_tags(float naiive_move_time, float cruise, float cruise_vmax);

#endif // End of include guard: FEEDLIMIT_H_ONCE
