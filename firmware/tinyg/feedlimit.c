/*
 * feedlimit.c - feed-limit advisor, step 1 (see feedlimit.h)
 */
#include "tinyg.h"
#include "config.h"
#include "planner.h"
#include "util.h"
#include "canonical_machine.h"
#include "stepper.h"
#include "feedlimit.h"

flBlock_t fl_slot[2];
uint8_t fl_stage;
volatile uint8_t fl_staged;
volatile uint8_t fl_cur = FL_CUR_NONE;
volatile uint8_t fl_on;
volatile flPending_t fl_pend[2];
volatile uint8_t fl_bank;
uint16_t fl_um_q16;
volatile uint8_t fl_activity;

static uint8_t fl_blk_ok;			// exec: the running block's segments are eligible (G94 linear XYZ feed)
static uint8_t fl_blk_mask;			// exec: its cause bits (static and profile tags, or FL_REC_OTHER)
static float fl_blk_below;			// exec: a segment slower than this (mm/min) is below F
static uint8_t fl_blk_cls;			// exec: class of the command next to it (FL_CLS_*), for FL_COMMAND time

/*
 * fl_stage_block() - stage a block's report at its exec start (called from the exec interrupt)
 *
 *	Fills the slot that is not current. If the block staged before never had a segment prepared (its
 *	marker is still pending) its slot never became current, and is reused.
 */
void fl_stage_block(const mpBuf_t *bf)
{
	if (fl_staged == FL_LOAD_SAME) {				// the last staged block was loaded and is current
		fl_stage ^= 1;
	}
	flBlock_t *s = &fl_slot[fl_stage];
	s->tags = bf->fl_tags;
	s->kind = (bf->gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE) ? FL_KIND_RAPID : FL_KIND_FEED;
	s->feed_ok = (s->kind == FL_KIND_FEED) && (bf->gm.feed_rate_mode != INVERSE_TIME_MODE);
	s->pct_ok = s->feed_ok &&
				!(fl_axis_moves(bf->unit[AXIS_A]) | fl_axis_moves(bf->unit[AXIS_B]) | fl_axis_moves(bf->unit[AXIS_C]));
	s->cruise = bf->cruise_velocity;
	s->feed = bf->gm.feed_rate;
	s->cruise_vmax = bf->cruise_vmax;
	s->naiive_move_time = bf->naiive_move_time;
	s->line = bf->gm.linenum;
	fl_staged = FL_LOAD_SLOT0 + fl_stage;
	if (fl_on) {									// accounting: classify the block once, for all its segments
		fl_activity = true;							// a final summary is due once this has run
		fl_blk_ok = s->pct_ok;
		float reached = s->feed * 0.01;				// the same 1% (or 2 mm/min) as fl_profile_tags()
		if (reached < 2) { reached = 2; }
		fl_blk_below = s->feed - reached;
		uint8_t profile;
		if (s->cruise_vmax == s->feed) {			// not axis-capped: fl_profile_tags() with this threshold
			profile = (s->naiive_move_time <= NOM_SEGMENT_TIME) ? FL_MIN_SEGMENT :
					  ((s->cruise < fl_blk_below) ? FL_SHORT : 0);
		} else {
			profile = fl_profile_tags(s->naiive_move_time, s->cruise, s->cruise_vmax);
		}
		s->all_tags = s->tags | profile;
		fl_blk_mask = (s->all_tags != 0) ? s->all_tags : FL_REC_OTHER;
		fl_blk_cls = bf->fl_cls;
	}
}

/*
 * fl_segment() - the accounting record of the segment being prepared (exec, while it owns st_pre; on only)
 *
 *	A segment prepared while a feedhold is active (its deceleration) is not eligible; segments prepared
 *	before the hold keep their records. Below F: every cause bit of the block; the time counts under each.
 *	The length is the exec's velocity x time (a commanded-path estimate). It goes to Q16 micrometres with one
 *	multiply and the fraction is carried as an integer: each segment loses under 1/65536 um to truncation (plus
 *	float rounding), and a segment over ~61 mm loses its sub-micrometre part. cd stays an estimate.
 */
void fl_segment(float velocity, float length)
{
	uint8_t rec = 0;
	if (fl_blk_ok && (cm.hold_state == FEEDHOLD_OFF)) {
		rec = FL_REC_ELIGIBLE;
		if (velocity < fl_blk_below) { rec |= fl_blk_mask; }
		st_pre.fl_cls = fl_blk_cls;
		float q16 = length * (1000.0f * 65536.0f);
		if (q16 < 4.0e9f) {							// up to ~61 mm in one segment (a 5-10 ms segment is far less)
			uint32_t um = (uint32_t)q16 + fl_um_q16;
			st_pre.fl_um = um >> 16;
			fl_um_q16 = (uint16_t)um;
		} else {
			st_pre.fl_um = (uint32_t)(length * 1000);
		}
	}
	st_pre.fl_rec = rec;
}

/*
 * fl_profile_tags() - FL_SHORT / FL_MIN_SEGMENT for the plan a block runs with
 *
 *	Called when a report reads the running block, from the plan the exec staged; never in the
 *	planner's replan loop or the exec, which run for every block.
 */
uint8_t fl_profile_tags(float naiive_move_time, float cruise, float cruise_vmax)
{
	if (naiive_move_time <= NOM_SEGMENT_TIME) {
		return (FL_MIN_SEGMENT);					// the F or B'' single-segment profile
	}
	float reached = cruise_vmax * 0.01;				// within 1% (or 2 mm/min) counts as reached
	if (reached < 2) { reached = 2; }
	return ((cruise < (cruise_vmax - reached)) ? FL_SHORT : 0);
}
