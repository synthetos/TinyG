/*
 * feedlimit.h - feed-limit advisor, step 1: why the running move's planned speed is below its F
 *
 *	The planner tags each block with the constraints that shaped its plan (FL_* bits): the axis cap at
 *	creation, the corner and exact-stop limits of its entry, and from each (re)plan the trapezoid that
 *	was chosen. The tags are published with the plan in the same protected store (E180). When the exec
 *	starts a block it stages the block's tags, planned cruise and F; the step loader makes them current
 *	when that block's first segment loads, so the report describes the move stepping now, not the one
 *	being prepared. Read through the rt family ({rt:{flm:null}} and friends, runtime.c).
 *	A tag means the constraint took part in planning the block, not that it limits every instant of it.
 *	See .analysis/feed-advisor-design.md.
 */
#ifndef FEEDLIMIT_H_ONCE
#define FEEDLIMIT_H_ONCE

// constraint tags (block and report)
#define FL_AXIS_CAP		0x01		// an axis's feedrate max (fr) makes the move slower than its F
#define FL_JUNCTION		0x02		// entry limited by the corner with the previous move
#define FL_EXACT_STOP	0x04		// G61 exact stop: entry and exit planned to zero
#define FL_SHORT		0x08		// too short to reach its cruise speed between entry and exit
#define FL_MIN_SEGMENT	0x10		// single-segment short-block profile (the F and B'' cases)
									// 0x20 reserved
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
	float naiive_move_time;			// move time at cruise, minutes
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

void fl_stage_block(const struct mpBuffer *bf);
uint8_t fl_profile_tags(float naiive_move_time, float cruise, float cruise_vmax);

#endif // End of include guard: FEEDLIMIT_H_ONCE
