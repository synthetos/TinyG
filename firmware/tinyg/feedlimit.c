/*
 * feedlimit.c - feed-limit advisor, step 1 (see feedlimit.h)
 */
#include "tinyg.h"
#include "config.h"
#include "planner.h"
#include "util.h"
#include "feedlimit.h"

flBlock_t fl_slot[2];
uint8_t fl_stage;
volatile uint8_t fl_staged;
volatile uint8_t fl_cur = FL_CUR_NONE;

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
	fl_staged = FL_LOAD_SLOT0 + fl_stage;
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
