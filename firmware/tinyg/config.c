/*
 * config.c - application independent configuration handling
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 *	See config.h for a Config system overview and a bunch of details.
 */

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "report.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "json_parser.h"
#include "text_parser.h"
#include "persistence.h"
#include "hardware.h"
#include "settings.h"
#include "help.h"
#include "util.h"
#include "xio.h"
#include "runtime.h"

static void _set_defa(nvObj_t *nv);
/*
 * Settings that could not be loaded at boot run on their defaults, and the host is told once TX starts (N61).
 *	A stored value the setter refuses is replaced by its default in RAM. The EEPROM value is kept: a refusal can come
 *	from the firmware rather than the value (a draft of N60 refused every setting at boot), and the user's saved
 *	settings must survive that. A default that nv_set() refuses is applied by calling the row's own setter, past
 *	nv_set()'s gates, so its side effects (steps per unit, driver pins, echo flags, jerk caches) happen as usual.
 *	A setter that refuses its own default is a firmware defect (test_config_defaults checks every default against
 *	its setter): that setting is left as it was and reported separately as not restored, never as restored.
 */
#define CONFIG_REPORT_NAMES 8               // settings named in a report; any more are counted

typedef struct cfgReportList {
    index_t index[CONFIG_REPORT_NAMES];
    index_t count;                          // settings in the list, named or not
    index_t last;                           // the last one added, to add each once
} cfgReportList_t;

static cfgReportList_t config_restored = { .last = NO_MATCH };  // running on their defaults
static cfgReportList_t config_failed = { .last = NO_MATCH };    // default refused by its own setter

static void _report_list_add(cfgReportList_t *list, index_t index)
{
    if (index == list->last) {
        return;
    }
    list->last = index;
    if (list->count < CONFIG_REPORT_NAMES) {
        list->index[list->count] = index;
    }
    list->count++;
}

static void _report_list_send(cfgReportList_t *list, stat_t status)
{
    if (list->count == 0) {
        return;
    }
    char msg[(TOKEN_LEN + 1) * CONFIG_REPORT_NAMES + 16];
    char *wr = msg;
    index_t named = (list->count < CONFIG_REPORT_NAMES) ? list->count : CONFIG_REPORT_NAMES;
    for (index_t i = 0; i < named; i++) {
        if (i) { *wr++ = ','; }
        strncpy_P(wr, cfgArray[list->index[i]].token, TOKEN_LEN);
        wr[TOKEN_LEN] = NUL;
        wr += strlen(wr);
    }
    *wr = NUL;
    if (list->count > named) {
        sprintf_P(wr, PSTR(" (+%u more)"), (unsigned)(list->count - named));
    }
    list->count = 0;
    list->last = NO_MATCH;
    rpt_exception(status, msg);             // printed whatever the JSON verbosity
}

static stat_t _load_config_default(nvObj_t *nv)
{
    nv->value_flt = GET_TABLE_FLOAT(default_value);
    if (cfg_is_type(nv->index) != TYPE_FLOAT) {
        nv->value_int = (uint32_t)nv->value_flt;
        nv->valuetype = TYPE_INTEGER;
    } else {
        nv->valuetype = TYPE_FLOAT;
    }
    stat_t status = nv_set(nv);
    if (status != STAT_OK) {
        status = ((fptrCmd)GET_TABLE_WORD(set))(nv);        // the row's own setter, past nv_set()'s gates
        if (status != STAT_OK) {
            _report_list_add(&config_failed, nv->index);   // left as it was: never reported as restored
        }
    }
    return (status);
}

// a stored value the setter refused: run on the default and report it
static void _load_config_fallback(nvObj_t *nv)
{
    if (_load_config_default(nv) == STAT_OK) {
        _report_list_add(&config_restored, nv->index);
    }
}

void config_report_load_failure(void)
{
    _report_list_send(&config_restored, STAT_SETTINGS_RESTORED_TO_DEFAULTS);
    _report_list_send(&config_failed, STAT_CONFIG_ASSERTION_FAILURE);
}

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

nvStr_t nvStr;
nvList_t nvl;

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/* Primary access points to functions bound to text mode / JSON functions
 * These gatekeeper functions check index ranges so others don't have to
 *
 * nv_set() 	- Write a value or invoke a function - operates on single valued elements or groups
 * nv_get() 	- Build a nvObj with the values from the target & return the value
 *			   	  Populate nv body with single valued elements or groups (iterates)
 * nv_print()	- Output a formatted string for the value.
 * nv_persist() - persist value to non-volatile storage. Takes special cases into account
 *
 *	!!!! NOTE: nv_persist() cannot be called from an interrupt on the AVR due to the AVR1008 EEPROM workaround
 */
/*
 * nv_refused_in_cycle() - a persisted setting written while a cycle runs (moving, homing, probing, jogging or in
 *	feedhold) or while queued work is pending or running (a lone G4 dwell does not start a cycle) is refused before
 *	it applies (N60). The busy test is the one the switch setters use. Never before config_init() has finished:
 *	it loads every setting through nv_set() after canonical_machine_init() has queued commands (and has already set
 *	MACHINE_READY), and refusing them left a board on zeroed settings (jv=0: silent). Applying it and failing only the EEPROM write left the board on a
 *	value that reverted at the next reset. The status report list counts as a setting: it is saved slot by slot.
 *	Actions, rt leaves and GETs are not affected; G10 offsets keep their own deferred write-back.
 */
static bool config_loaded;                  // set when config_init() has loaded every setting (N60)

bool config_is_loaded(void) { return (config_loaded); }

bool nv_refused_in_cycle(nvObj_t *nv)
{
	if (!config_loaded ||
	    ((cm.cycle_state == CYCLE_OFF) && !mp_has_runnable_buffer() && mp_runtime_is_idle())) {
		return (false);
	}
	fptrCmd set = (fptrCmd)GET_TABLE_WORD(set);
	return (cfg_has_flag(nv->index, F_PERSIST) || (set == sr_set) || (set == srs_set));
}

stat_t nv_set(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
	if (nv_refused_in_cycle(nv)) {
		return (STAT_SETTING_REFUSED_IN_CYCLE);	// nothing changes; the job continues (N60)
	}
	fptrCmd set = (fptrCmd)GET_TABLE_WORD(set);
	if ((set != set_grp) && (set != set_nul) && (set != rt_set_grp) && (set != rt_set_hb) && (set != rt_set_qt) &&
		(set != rt_set_fi)) {
		rt_qt_invalidate();					// a setting or action write, before it takes effect; a group
	}										// counts through its children, and rt operations never
	stat_t status = set(nv);
	if ((status == STAT_OK) && nv_index_is_single(nv->index)) {
		nv->precision = (int8_t)GET_TABLE_WORD(precision);	// SET echoes use the row's display precision, as GETs do (E076)
	}
	return (status);
}

stat_t nv_get(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
	fptrCmd get = (fptrCmd)GET_TABLE_WORD(get);
	if (cfg_get_is_action(get)) {
		rt_qt_invalidate();					// a GET that acts, before it takes effect
	}
	return (get(nv));
}

void nv_print(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) {
        return;
    }
	((fptrCmd)GET_TABLE_WORD(print))(nv);
}

stat_t nv_persist(nvObj_t *nv)	// nv_persist() cannot be called from an interrupt on the AVR due to the AVR1008 EEPROM workaround
{
	if (nv_index_lt_groups(nv->index) == false) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
	if (cfg_has_flag(nv->index, F_PERSIST)) {
        return(write_persistent_value(nv));
    }
	return (STAT_OK);
}

/************************************************************************************
 * NVM layout version (E022)
 *
 *	NVM record 0 used to hold the firmware build number, and config_init() loaded defaults whenever it
 *	differed from the running build - so every release wiped every setting. Record 0 now holds a
 *	layout marker that changes only when the persisted layout changes: NVM records are indexed by
 *	cfgArray position: inserting, removing or moving any earlier row changes persisted indexes.
 *	Layout 3 removes output-pause settings; layout 2 is migrated through a power-cut journal.
 *	Layout 2 migrates layout 1 (including legacy 449.01) because the old indexes are unchanged.
 *	Erased EEPROM and unknown layouts initialize defaults.
 */
#define CONFIG_NVM_LAYOUT			3
#define CONFIG_NVM_LAYOUT_MARKER	(10000.0 + CONFIG_NVM_LAYOUT)	// can't be mistaken for a build number
#define CONFIG_NVM_LAYOUT_1_BUILD	449.01							// record 0 of a layout-1 board before markers

static bool _nvm_layout_is_current(nvObj_t *nv)
{
	if (nv->value_int == 0xFFFFFFFF) {
		return (false);
	}
	if (fp_EQ(nv->value_flt, CONFIG_NVM_LAYOUT_MARKER)) {
		return (true);
	}
	return ((CONFIG_NVM_LAYOUT == 1) && fp_EQ(nv->value_flt, CONFIG_NVM_LAYOUT_1_BUILD));
}

static bool _nvm_layout_is_legacy(nvObj_t *nv)
{
    return ((CONFIG_NVM_LAYOUT >= 2) &&
            (fp_EQ(nv->value_flt, 10001.0) || fp_EQ(nv->value_flt, CONFIG_NVM_LAYOUT_1_BUILD)));
}

static void _write_nvm_layout_marker(nvObj_t *nv)
{
	nv->index = 0;
	nv->value_flt = CONFIG_NVM_LAYOUT_MARKER;
	write_persistent_value(nv);
}

/************************************************************************************
 * Master settings migration (N92)
 *
 *	Master builds 440.20 and 440.21 keep the build number in record 0 and every F_INITIALIZE row as a
 *	float at its master index * 4 (config_master.h). A board upgraded from master used to load the
 *	defaults: switch type, protocol and every machine setting changed at once. _migrate_master() puts
 *	each master value into the live settings by token, then writes the edge layout and the marker last.
 *	A value edge refuses runs on its default and is reported (status 118). Mapped on purpose:
 *	  - jv 1 or 2 becomes 3: master echoed configuration values at every verbosity;
 *	  - zsn 0 becomes 4: master probed on Z-min whatever its mode, edge needs a probe-mode switch;
 *	  - rxm becomes 0: master's character-mode protocol, which its hosts speak;
 *	  - saved status report indexes are translated by token into the current table.
 *
 *	The two layouts overlap in EEPROM, and a record is written one byte at a time, so a power loss part
 *	way through must not re-read a master record an edge value has already replaced:
 *	  - Before the first overlapping write, each master value whose record is written no later than its
 *	    own edge row is copied to a journal record past the edge table.
 *	  - Rows are written in index order. A checkpoint "edge rows below N are written" goes to one of two
 *	    slots at the top of EEPROM before any row write that would lose the master source of a row
 *	    already written since the last checkpoint, and before the status report defaults and the marker.
 *	  - A checkpoint is a sequence number, N with a tag, and a check byte. Its check byte is first set to
 *	    a value no partly written slot can match and is written last, so a torn checkpoint is invalid
 *	    and the other slot still holds the previous one.
 *	  - A boot that finds a checkpoint (record 0 is master, or torn on its way to the marker) loads the
 *	    rows below N as edge values, reads the rest from the journal or from master records not yet
 *	    written, and finishes with the same result. The checkpoints are cleared after the marker.
 */
#include "config_master.h"

#define MASTER_BUILD_A	440.20
#define MASTER_BUILD_B	440.21
#define NVM_RECORDS		((E2END + 1) / NVM_VALUE_LEN)
#define CHECKPOINT_A	(NVM_RECORDS - 2)		// the two checkpoint slots; the journal ends below them
#define CHECKPOINT_B	(NVM_RECORDS - 1)
#define CHECKPOINT_TAG	0xB0					// high nibble of byte 2; bytes: seq, N low, tag | N high, check

static bool _nvm_layout_is_master(nvObj_t *nv)
{
	return ((nv->value_int != 0xFFFFFFFF) &&
	        (fp_EQ(nv->value_flt, (float)MASTER_BUILD_A) || fp_EQ(nv->value_flt, (float)MASTER_BUILD_B)));	// as stored
}

static uint8_t _checkpoint_check(uint32_t v)			// byte 3 of a checkpoint, over bytes 0-2
{
	return ((uint8_t)(v ^ (v >> 8) ^ (v >> 16) ^ 0xA5));
}

static index_t _checkpoint_rows(uint32_t v)			// N of a valid checkpoint, else 0
{
	index_t rows = (index_t)(((v >> 8) & 0xFF) | ((v >> 8) & 0x0F00));
	if ((((v >> 16) & 0xF0) != CHECKPOINT_TAG) || ((uint8_t)(v >> 24) != _checkpoint_check(v)) ||
	    (rows < 1) || (rows > nv_index_max())) {
		return (0);
	}
	return (rows);
}

static index_t _master_checkpoint(index_t *slot, uint8_t *seq)	// the newest valid checkpoint's N, or 0
{
	nvObj_t a, b;
	a.index = CHECKPOINT_A;
	b.index = CHECKPOINT_B;
	read_persistent_value(&a);
	read_persistent_value(&b);
	index_t na = _checkpoint_rows(a.value_int);
	index_t nb = _checkpoint_rows(b.value_int);
	if ((nb > 0) && ((na == 0) || ((int8_t)((uint8_t)b.value_int - (uint8_t)a.value_int) > 0))) {
		*slot = CHECKPOINT_B;
		*seq = (uint8_t)b.value_int;
		return (nb);
	}
	*slot = CHECKPOINT_A;
	*seq = (uint8_t)a.value_int;
	return (na);
}

static void _write_master_checkpoint(index_t rows)	// edge rows below this one hold edge values
{
	index_t slot;
	uint8_t seq;
	index_t newest = _master_checkpoint(&slot, &seq);
	if (newest == rows) { return; }
	if (newest > 0) {
		slot = (slot == CHECKPOINT_A) ? CHECKPOINT_B : CHECKPOINT_A;	// keep the newest until this one is valid
		seq++;
	} else {
		seq = 0;
	}
	uint32_t n = seq | ((uint32_t)(rows & 0xFF) << 8) | ((uint32_t)(CHECKPOINT_TAG | (rows >> 8)) << 16);
	n |= (uint32_t)_checkpoint_check(n) << 24;
	nvObj_t r;
	r.index = slot;
	read_persistent_value(&r);
	uint8_t hold = 0;										// a check byte that matches no state on the way to n
	for (bool clash = true; clash; ) {
		uint32_t s = r.value_int;
		clash = false;
		for (uint8_t k = 0; k < 4; k++) {
			clash |= (hold == _checkpoint_check(s));
			s = (s & ~(0xFFUL << (8 * k))) | (n & (0xFFUL << (8 * k)));
		}
		if (clash) { hold++; }
	}
	write_persistent_byte(slot, 3, hold);
	for (uint8_t k = 0; k < 4; k++) {						// bytes 0-2, then the check byte commits it
		write_persistent_byte(slot, k, (uint8_t)(n >> (8 * k)));
	}
}

static void _clear_master_checkpoints(void)
{
	nvObj_t r;
	for (r.index = CHECKPOINT_A; r.index <= CHECKPOINT_B; r.index++) {
		read_persistent_value(&r);
		if (_checkpoint_rows(r.value_int) > 0) {
			write_persistent_byte(r.index, 3, ~_checkpoint_check(r.value_int));
		}
	}
}

static bool _master_journaled(index_t mi, index_t ei)	// is master record mi written before edge row ei?
{
	return ((mi <= ei) && (mi < nv_index_max()) && cfg_has_flag(mi, F_INITIALIZE));
}

static bool _master_source_pending(index_t row, index_t safe)	// would writing row lose the source of a row in [safe, row)?
{
	if (row >= MASTER_NV_ROWS) { return (false); }
	const char *p = master_tokens;
	for (index_t mi = 0; mi < row; mi++) { p += strlen_P(p) + 1; }
	if (pgm_read_byte(p) == NUL) { return (false); }
	char token[TOKEN_LEN+1];
	strncpy_P(token, p, TOKEN_LEN);
	token[TOKEN_LEN] = NUL;
	index_t ei = nv_get_index("", token);
	return ((ei != NO_MATCH) && cfg_has_flag(ei, F_INITIALIZE) && (ei >= safe) && (ei < row));
}

static bool _is_sr_slot(const char *token)				// se00 - se29: status report list slots
{
	return ((token[0] == 's') && (token[1] == 'e') && (token[2] >= '0') && (token[2] <= '9'));
}

static index_t _master_report_index(float old)
{
    if (!isfinite(old) || old < 0 || old >= MASTER_NV_ROWS || old != floorf(old)) {
        return NO_MATCH;
    }
    const char *p = master_report_tokens;
    for (index_t i = 0; i < (index_t)old; i++) { p += strlen_P(p) + 1; }
    char token[TOKEN_LEN + 1];
    strncpy_P(token, p, TOKEN_LEN);
    token[TOKEN_LEN] = NUL;
    return nv_get_index("", token);
}

static void _migrate_master(nvObj_t *nv, index_t done)	// done: the checkpoint's N, 0 on the first boot
{
	nvObj_t m;												// master values: read_persistent_value() reads index * 4
	index_t journal = nv_index_max();
    // Preserve the saved list before any master source records can be overwritten.
    if (done == 0) {
        for (uint8_t i = 0; i < NV_STATUS_REPORT_LEN; i++) {
            m.index = MASTER_SR_FIRST + i;
            read_persistent_value(&m);
            m.index = CHECKPOINT_A - NV_STATUS_REPORT_LEN + i;
            write_persistent_value(&m);
        }
    }

	for (nv->index=0; nv_index_is_single(nv->index); nv->index++) {	// defaults first, written later
		if (cfg_has_flag(nv->index, F_INITIALIZE)) {
			strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);
			_load_config_default(nv);
		}
	}
	for (nv->index=1; nv->index < done; nv->index++) {		// written before an interruption: edge values
		if (cfg_has_flag(nv->index, F_INITIALIZE)) {
			strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);
			if (_is_sr_slot(nv->token)) { continue; }		// written last, from the saved report journal
			read_persistent_value(nv);
			nv->valuetype = (cfg_is_type(nv->index) == TYPE_FLOAT) ? TYPE_FLOAT : TYPE_INTEGER;
			if (nv_set(nv) != STAT_OK) {
				_load_config_fallback(nv);
			}
		}
	}
	const char *p = master_tokens;
	for (index_t mi = 0; mi < MASTER_NV_ROWS; p += strlen_P(p) + 1, mi++) {
		if (pgm_read_byte(p) == NUL) { continue; }			// a row master did not load at boot
		strncpy_P(nv->token, p, TOKEN_LEN);
		nv->token[TOKEN_LEN] = NUL;
		nv->index = nv_get_index("", nv->token);
		if ((nv->index == NO_MATCH) || !cfg_has_flag(nv->index, F_INITIALIZE)) { continue; }
		m.index = mi;
		if (_master_journaled(mi, nv->index)) {
			if (journal >= CHECKPOINT_A - NV_STATUS_REPORT_LEN) {					// no room to keep it safely
				_report_list_add(&config_restored, nv->index);
				continue;
			}
			m.index = journal++;
			if (done == 0) {								// first boot: copy it before any overlapping write
				nvObj_t src;
				src.index = mi;
				read_persistent_value(&src);
				m.value_int = src.value_int;
				write_persistent_value(&m);
			}
		}
		if (nv->index < done) { continue; }					// loaded above as an edge value
		read_persistent_value(&m);
		if (cfg_is_type(nv->index) == TYPE_DATA) {
			nv->value_int = m.value_int;					// raw bits on both
			nv->valuetype = TYPE_INTEGER;
		} else if (!isfinite(m.value_flt)) {
			_report_list_add(&config_restored, nv->index);	// keeps its default
			continue;
		} else if (cfg_is_type(nv->index) == TYPE_FLOAT) {
			nv->value_flt = m.value_flt;
			nv->valuetype = TYPE_FLOAT;
		} else {
			nv->value_int = (uint32_t)lround(m.value_flt);	// master kept integers as floats
			nv->valuetype = TYPE_INTEGER;
			if ((strcmp(nv->token, "jv") == 0) && ((nv->value_int == 1) || (nv->value_int == 2))) {
				nv->value_int = 3;
			}
			if ((strcmp(nv->token, "zsn") == 0) && (nv->value_int == 0)) {
				nv->value_int = 4;
			}
		}
		if (nv_set(nv) != STAT_OK) {
			_load_config_fallback(nv);
		}
	}
	strcpy(nv->token, "rxm");
	if ((nv->index = nv_get_index("", nv->token)) != NO_MATCH) {
		nv->value_int = 0;
		nv->valuetype = TYPE_INTEGER;
		nv_set(nv);
	}
	index_t safe = (done > 1) ? done : 1;					// rows below it count as written on a resume
	_write_master_checkpoint(safe);							// the journal is complete
	for (nv->index = safe; nv_index_is_single(nv->index); nv->index++) {	// the edge layout
		if (cfg_has_flag(nv->index, F_INITIALIZE) && cfg_has_flag(nv->index, F_PERSIST)) {
			strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);
			if (_is_sr_slot(nv->token)) { continue; }		// written from the report journal below
			nv_get(nv);
			m.index = nv->index;
			read_persistent_value(&m);
			if (m.value_int != nv->value_int) {
				if (_master_source_pending(nv->index, safe)) {
					safe = nv->index;
					_write_master_checkpoint(safe);
				}
				nv_persist(nv);
			}
		}
	}
	_write_master_checkpoint(nv_index_max());
    index_t report_first = nv_get_index("", "se00");
    for (uint8_t i = 0; i < NV_STATUS_REPORT_LEN; i++) {
        m.index = CHECKPOINT_A - NV_STATUS_REPORT_LEN + i;
        read_persistent_value(&m);
        m.value_int = _master_report_index(m.value_flt);
        m.index = report_first + i;
        write_persistent_value(&m);
    }
    sr_init_status_report_P(PSTR(""));
	_write_nvm_layout_marker(nv);								// commit the migration
	_clear_master_checkpoints();
}

// Layout 2 removed three adjacent output-pause settings and their later activation stamp.
// Keep original suffix records and status selections in unused EEPROM before touching either.
// The journal stamp commits only after the copy; the layout marker commits only after the move.
#define LAYOUT2_JOURNAL_STAMP 0xC302A65FUL
#define LAYOUT2_JOURNAL_FLAG (CHECKPOINT_A - 1)

static bool _layout2_journal_ready(void)
{
    nvObj_t n;
    n.index = LAYOUT2_JOURNAL_FLAG;
    read_persistent_value(&n);
    return n.value_int == LAYOUT2_JOURNAL_STAMP;
}

static index_t _layout2_index(uint32_t old, index_t first, index_t after_stamp)
{
    if (old == (uint32_t)NO_MATCH || old >= (uint32_t)(nv_index_max() + 4)) { return NO_MATCH; }
    if (old < first) { return (index_t)old; }
    if (old < first + 3 || old == after_stamp + 3) { return NO_MATCH; }
    return (index_t)(old - ((old < after_stamp + 3) ? 3 : 4));
}

static void _migrate_layout2(nvObj_t *nv)
{
    index_t first = nv_get_index("", "prbp");
    index_t after_stamp = nv_get_index("", "rtjob");
    index_t reports = nv_get_index("", "se00");
    index_t end = first;
    while (nv_index_is_single(end)) { end++; }
    index_t journal = end + 4; // above every old single-valued record, including removed rows
    index_t count = end - first;
    if (first == NO_MATCH || after_stamp == NO_MATCH || reports == NO_MATCH ||
        journal + count + NV_STATUS_REPORT_LEN > LAYOUT2_JOURNAL_FLAG) {
        cm_panic_P(STAT_CONFIG_ASSERTION_FAILURE, PSTR("Layout journal capacity"));
        return;
    }
    nvObj_t n;
    if (!_layout2_journal_ready()) {
        for (index_t i = 0; i < count + NV_STATUS_REPORT_LEN; i++) {
            n.index = (i < count) ? first + i + ((first + i < after_stamp) ? 3 : 4)
                                  : reports + i - count;
            read_persistent_value(&n);
            n.index = journal + i;
            if (write_persistent_value(&n) != STAT_OK) { return; }
        }
        n.index = LAYOUT2_JOURNAL_FLAG;
        n.value_int = LAYOUT2_JOURNAL_STAMP;
        if (write_persistent_value(&n) != STAT_OK) { return; }
    }
    for (index_t i = 0; i < count + NV_STATUS_REPORT_LEN; i++) {
        n.index = journal + i;
        read_persistent_value(&n);
        if (i < count) {
            n.index = first + i;
        } else {
            n.index = reports + i - count;
            n.value_int = _layout2_index(n.value_int, first, after_stamp);
        }
        if (write_persistent_value(&n) != STAT_OK) { return; }
    }
    _write_nvm_layout_marker(nv);
    nv->index = 0;
    read_persistent_value(nv);
    if (nv->value_flt != CONFIG_NVM_LAYOUT_MARKER) { return; }
    n.index = LAYOUT2_JOURNAL_FLAG;
    n.value_int = 0;
    write_persistent_value(&n);
}

/************************************************************************************
 * config_init() - called once on hard reset
 *
 * Load a current layout, migrate the append-only layout-1 prefix, or initialize defaults
 * for an erased or unknown layout. Migration writes new initialized rows before its marker.
 *
 *	You can assume the cfg struct has been zeroed by a hard reset.
 *	Do not clear it as the version and build numbers have already been set by controller_init()
 *
 * NOTE: Config assertions are handled from the controller
 */
void config_init()
{
	nvObj_t *nv = nv_reset_nv_list(NUL);
	config_init_assertions();

#ifdef __ARM
// The following code is offered until persistence is implemented.
// Then you can use the AVR code (or something like it)
	cfg.comm_mode = JSON_MODE;					// initial value until EEPROM is read
	_set_defa(nv);
#endif
#ifdef __AVR
	cm_set_units_mode(MILLIMETERS);				// must do inits in millimeter mode
	nv->index = 0;								// this will read the first record in NVM

	read_persistent_value(nv);
    if (fp_EQ(nv->value_flt, 10002.0) || _layout2_journal_ready()) {
        _migrate_layout2(nv);
        nv->index = 0;
        read_persistent_value(nv);
        if (!_nvm_layout_is_current(nv)) { return; }
    }
    bool migrate = _nvm_layout_is_legacy(nv);
    bool initialized = true;
	index_t checkpoint_slot;
	uint8_t checkpoint_seq;
	index_t master_done = _master_checkpoint(&checkpoint_slot, &checkpoint_seq);
	if (_nvm_layout_is_master(nv) ||						// a board upgraded from master 440.20/440.21 (N92),
	    ((master_done > 0) && !migrate && !_nvm_layout_is_current(nv))) {	// or its marker write was cut
		rpt_print_loading_configs_message();
		_migrate_master(nv, master_done);
	} else if (!_nvm_layout_is_current(nv) && !migrate) {	// case (1) NVM is not setup or has another layout
		_set_defa(nv);
	} else {									// case (2) NVM is setup and in revision
        index_t first_new = nv_index_max();
        if (migrate) {
            strcpy_P(nv->token, PSTR("spep"));
            first_new = nv_get_index("", nv->token);
            if (first_new == NO_MATCH) {
                cm_panic_P(STAT_CONFIG_ASSERTION_FAILURE, PSTR("Missing layout-2 boundary"));
                return;
            }
        }
		rpt_print_loading_configs_message();
		for (nv->index=0; nv_index_is_single(nv->index); nv->index++) {
            if (cfg_has_flag(nv->index, F_INITIALIZE)) {
				strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);	// read the token from the array
                if ((migrate && (nv->index >= first_new))) {
                    if (_load_config_default(nv) == STAT_OK) {
                        if (nv_persist(nv) != STAT_OK) { initialized = false; }
                    } else {
                        initialized = false;
                    }
                } else {
                    read_persistent_value(nv);
                    nv->valuetype = (cfg_is_type(nv->index) == TYPE_FLOAT) ? TYPE_FLOAT : TYPE_INTEGER;
                    if (nv_set(nv) != STAT_OK) {
                        _load_config_fallback(nv);
                    }
                }
			}
		}
        sr_init_status_report_P(PSTR(""));      // load status report setup from NVram
        if (migrate && initialized) { _write_nvm_layout_marker(nv); } // commit the migration last
	}
#endif
	if (master_done > 0) {
		_clear_master_checkpoints();				// left by a migration that wrote its marker
	}
	config_loaded = true;                       // from here a SET while work runs is refused (N60)
}

/*
 * set_defaults() - reset persistence with default values for machine profile
 * _set_defa() - helper function and called directly from config_init()
 */

static void _set_defa(nvObj_t *nv)
{
	if (config_loaded && cm_persistence_busy()) {	// a lone dwell starts no cycle (N68)
    	rpt_exception_P(STAT_COMMAND_NOT_ACCEPTED, PSTR("Cannot reset to defaults when moving"));
        return;
	}
	cm_set_units_mode(MILLIMETERS);				// must do inits in MM mode
	for (nv->index=0; nv_index_is_single(nv->index); nv->index++) {
        if (cfg_has_flag(nv->index, F_INITIALIZE)) {
            strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);
            if (_load_config_default(nv) == STAT_OK) {
                nv_persist(nv);
            }
		}
	}
    sr_init_status_report_P(SR_DEFAULTS);       // reset status reports to defaults
	_write_nvm_layout_marker(nv);               // (...and the layout marker replaces it)
	rpt_print_initializing_message();           // don't start TX until all the NVM persistence is done
}

stat_t set_defaults(nvObj_t *nv)
{
	// failsafe. nv->value_int must be true or no action occurs
	if (nv->value_int != true) {
        return(help_defa(nv));
    }
	_set_defa(nv);

	// The values in nv are now garbage. Mark the nv as $defa so it displays nicely.
    nv_reset_nv_list("r");
	strncpy(nv->token, "defa", TOKEN_LEN);
	nv->index = nv_get_index("", nv->token);	// correct, but not required
	nv->valuetype = TYPE_INTEGER;
	nv->value_int = 1;
	return (STAT_OK);
}

/*
 * config_init_assertions()
 * config_test_assertions() - check memory integrity of config sub-system
 */

void config_init_assertions()
{
	cfg.magic_start = MAGICNUM;
	cfg.magic_end = MAGICNUM;
	nvl.magic_start = MAGICNUM;
	nvl.magic_end = MAGICNUM;
	nvStr.magic_start = MAGICNUM;
	nvStr.magic_end = MAGICNUM;
}

stat_t config_test_assertions()
{
    if ((BAD_MAGIC(cfg.magic_start)) ||
        (BAD_MAGIC(cfg.magic_end)) ||
        (BAD_MAGIC(nvl.magic_start)) ||
        (BAD_MAGIC(nvl.magic_end)) ||
        (BAD_MAGIC(nvStr.magic_start)) ||
        (BAD_MAGIC(nvStr.magic_end)) ) {

        return(cm_panic_P(STAT_CONFIG_ASSERTION_FAILURE, PSTR("config_test_assertions()")));
    }
    return (STAT_OK);
}

/***** Generic Internal Functions *********************************************
 *
 * cfg_has_flag() - test for flag set in cfgArray table
 * cfg_is_type()  - return data type of config item
 */

bool cfg_has_flag(index_t index, uint8_t bitmask)
{
#ifdef __AVR
    return (pgm_read_byte(&cfgArray[index].flags) & bitmask);
#else
    return (cfgArray[index].flags) & bitmask);
#endif
}

valueType cfg_is_type(index_t index)
{
#ifdef __AVR
    return (pgm_read_byte(&cfgArray[index].flags) & F_TYPE_MASK);
#else
    return (cfgArray[index].flags) & F_TYPE_MASK);
#endif
}

/* Generic gets()
 *	get_nul()  - get nothing (returns STAT_PARAMETER_CANNOT_BE_READ)
 *  get_str()  - get value from str (no action required)
 *	get_ui8()  - get value as uint8_t
 *	get_u16()  - get value as uint16_t
 *	get_u32()  - get value as uint32_t
 *  get_int()  - get value as a signed integer (up to 32 bits)
 *	get_flt()  - get value as float
 *	get_data() - get value as 32 bit integer blind cast
 */
stat_t get_nul(nvObj_t *nv)
{
	nv->valuetype = TYPE_NULL;
	return (STAT_NOOP);
}

stat_t get_str(nvObj_t *nv)
{
    nv->valuetype = TYPE_STRING;
    return (STAT_OK);
}

stat_t get_ui8(nvObj_t *nv)
{
	nv->value_int = *((uint8_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_u16(nvObj_t *nv)
{
	nv->value_int = *((uint16_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_u32(nvObj_t *nv)
{
    nv->value_int = *((uint32_t *)GET_TABLE_WORD(target));
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t get_data(nvObj_t *nv)
{
	uint32_t *v = (uint32_t*)&nv->value_flt;
	*v = *((uint32_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_DATA;
	return (STAT_OK);
}

stat_t get_int(nvObj_t *nv)
{
//	nv->value_int = *((int32_t *)GET_TABLE_WORD(target));
	nv->value_int = *((int8_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_SIGNED;
	return (STAT_OK);
}

stat_t get_flt(nvObj_t *nv)
{
	nv->value_flt = *((float *)GET_TABLE_WORD(target));
	nv->precision = (int8_t)GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

/* Generic sets()
 *	set_nul()   - set nothing (returns STAT_PARAMETER_IS_READ_ONLY)
 *	set_not()   - set nothing (returns STAT_OK)
 *	set_ui8()   - set value as uint8_t value
 *	set_u16()   - set value as uint16 value
 *	set_u32()   - set value as uint32 value
 *	set_01()    - set a 0 or 1 uint8_t value with validation
 *	set_012()   - set a 0, 1 or 2 uint8_t value with validation
 *	set_0123()  - set a 0, 1, 2 or 3 uint8_t value with validation
 *	set_data()  - set value as 32 bit integer blind cast
 *	set_flt()   - set value as float
 *  set_str()   - set string value - just for test, performs no action
 */
stat_t set_nul(nvObj_t *nv) { return (STAT_PARAMETER_IS_READ_ONLY); }

stat_t set_not(nvObj_t *nv) { return (STAT_OK); }

stat_t set_ui8(nvObj_t *nv)
{
    if (nv->value_int > UINT8_MAX) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	*((uint8_t *)GET_TABLE_WORD(target)) = (uint8_t )nv->value_int;
	return(STAT_OK);
}

stat_t set_u16(nvObj_t *nv)
{
    if (nv->value_int > UINT16_MAX) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	*((uint16_t *)GET_TABLE_WORD(target)) = (uint16_t )nv->value_int;
	return(STAT_OK);
}

stat_t set_u32(nvObj_t *nv)
{
    *((uint32_t *)GET_TABLE_WORD(target)) = (uint32_t )nv->value_int;
    return(STAT_OK);
}

stat_t set_01(nvObj_t *nv)
{
	if (nv->value_int > 1) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	return (set_ui8(nv));
}

stat_t set_012(nvObj_t *nv)
{
	if (nv->value_int > 2) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	return (set_ui8(nv));
}

stat_t set_0123(nvObj_t *nv)
{
	if (nv->value_int > 3) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	return (set_ui8(nv));
}

stat_t set_data(nvObj_t *nv)
{
	uint32_t *v = (uint32_t*)&nv->value_flt;
	*((uint32_t *)GET_TABLE_WORD(target)) = *v;
	nv->valuetype = TYPE_DATA;
	return(STAT_OK);
}

stat_t set_flt(nvObj_t *nv)
{
	*((float *)GET_TABLE_WORD(target)) = nv->value_flt;
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return(STAT_OK);
}

stat_t set_str(nvObj_t *nv)
{
    return (STAT_OK);
}

/************************************************************************************
 * Group operations
 *
 *	Group operations work on parent/child groups where the parent is one of:
 *	  axis group 			x,y,z,a,b,c
 *	  motor group			1,2,3,4
 *	  PWM group				p1
 *	  coordinate group		g54,g55,g56,g57,g58,g59,g92
 *	  system group			"sys" - a collection of otherwise unrelated variables
 *
 *	Text mode can only GET groups. For example:
 *	  $x					get all members of an axis group
 *	  $1					get all members of a motor group
 *	  $<grp>				get any named group from the above lists
 *
 *	In JSON groups are carried as parent / child objects & can get and set elements:
 *	  {"x":n}                       get all X axis parameters
 *	  {"x":{"vm":n}}                get X axis velocity max
 *	  {"x":{"vm":1000}}             set X axis velocity max
 *	  {"x":{"vm":n,"fr":n}}         get X axis velocity max and feed rate
 *	  {"x":{"vm":1000,"fr";900}}    set X axis velocity max and feed rate
 *	  {"x":{"am":1,"fr":800,....}}  set multiple or all X axis parameters
 */

/*
 * get_grp() - read data from axis, motor, system or other group
 *
 *	get_grp() is a group expansion function that expands the parent group and returns
 *	the values of all the children in that group. It expects the first nvObj in the
 *	nvBody to have a valid group name in the token field. This first object will be set
 *	to a TYPE_PARENT. The group field is left NUL - as the group field refers to a parent
 *	group, which this group has none.
 *
 *	All subsequent nvObjs in the body will be populated with their values (unless there
 *  are no more nvObj struct available). The token field will be populated as will the
 *  parent name in the group field.
 *
 *	The sys group is an exception where the children carry a blank group field, even though
 *	the sys parent is labeled as a TYPE_PARENT.
 *
 *  Example: {x:n}
 */

stat_t get_grp(nvObj_t *nv)
{
    nvObj_t *nv_parent = nv;                        // used for error recovery
    nvObj_t *nv_tmp;                                // used for error recovery
	char *parent = nv->token;				        // token in the parent nv object is the group
    uint8_t child_depth = nv->depth+1;
	char group[GROUP_LEN+1];						// group string retrieved from cfgArray child
	nv->valuetype = TYPE_PARENT;					// make first object the parent
	cm_position_snapshot_begin();					// a position group reads every axis at one instant (E166)

    // scan the config_app table looking for tokens belonging to the parent group
	for (index_t i=0; nv_index_is_single(i); i++) {
		strcpy_P(group, cfgArray[i].group);			// don't need strncpy as it's always terminated
		if (strcmp(parent, group) != 0) {           // if no match try the next one
            continue;
        }
        nv_tmp = nv;
        if ((nv = nv_next(nv)) == NULL) {           // get next nvObj or fail
            do {                                    // undo the group expansion
                nv_reset_nv(nv_tmp);
            } while (--nv_tmp >= nv_parent);
            cm_position_snapshot_end();
            return (STAT_JSON_TOO_MANY_PAIRS);
        }
		nv_populate_nv_by_index(nv, i);
        nv->depth = child_depth;
	}
	cm_position_snapshot_end();
	return (STAT_OK);
}

/*
 * set_grp() - get or set one or more values in a group
 *
 *	This functions is called "_set_group()" but technically it's a getter and
 *	a setter. It iterates the group children and either gets the value or sets
 *	the value for each depending on the nv->valuetype.
 *
 *    SET example:   {x:{vm:10000, fr:8000}}
 *    GET example:   {x:{vm:n, fr:n}}
 *    Mixed example: {x:{vm:10000, fr:n}}
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */

stat_t set_grp(nvObj_t *nv)
{
	if (cs.comm_mode == TEXT_MODE) {
        return (STAT_UNRECOGNIZED_NAME);
    }
	nvObj_t *child = nv;                        // refuse the whole group before any child applies (N60)
	for (uint8_t i=0; i<NV_MAX_OBJECTS; i++) {
		if ((child = nv_next(child)) == NULL) break;
		if (child->valuetype == TYPE_EMPTY) break;
		if ((child->valuetype != TYPE_NULL) && (child->index < nv_index_max()) && nv_refused_in_cycle(child)) {
			return (STAT_SETTING_REFUSED_IN_CYCLE);
		}
	}
	for (uint8_t i=0; i<NV_MAX_OBJECTS; i++) {
		if ((nv = nv_next(nv)) == NULL) break;
		if (nv->valuetype == TYPE_EMPTY) break;
		else if (nv->valuetype == TYPE_NULL) { // NULL means GET the value
			ritorno(nv_get(nv));
		} else {
			ritorno(nv_set(nv));
			ritorno(nv_persist(nv));
		}
	}
	return (STAT_OK);
}

/*
 * nv_group_is_prefixed() - hack
 * nv_group_is_typesafe() - hack
 *
 *	These little functions deal with exception cases for some groups.
 *  It would be better put flags on these cfg items directly and remove these
 *  functions, which we will probably do once we bump the flags byte to 16 bits.
 *
 *  nv_group_is_prefixed() returns false if the member of the group does not
 *  use the parent token as a prefix to the child elements. SYS is a good example.
 *
 *  nv_group_is_typesafe() returns false if JSON type checking should be disabled
 *  when collecting children for this group.
 */

bool nv_group_is_prefixed(char *group)
{
	if ((strcmp_P(group, PSTR("sys"))) == 0) { return (false); }
	if ((strcmp_P(group, PSTR("set"))) == 0) { return (false); }
	if ((strcmp_P(group, PSTR("srs"))) == 0) { return (false); }
	if ((strcmp_P(group, PSTR("sr"))) == 0)  { return (false); }
	return (true);
}

bool nv_group_is_typesafe(char *group)
{
    if ((strcmp_P(group, PSTR("set"))) == 0) { return (false); }
    if ((strcmp_P(group, PSTR("sr"))) == 0)  { return (false); }
    return (true);
}

/***********************************************************************************
 ***** nvObj functions ************************************************************
 ***********************************************************************************/

/***********************************************************************************
 * nvObj helper functions and other low-level nv helpers
 */

/* nv_get_index() - get index from mnenonic token + group
 *
 * nv_get_index() is the most expensive routine in the whole config. It does a
 * linear table scan of the PROGMEM strings, which of course could be further
 * optimized with indexes or hashing.
 */
index_t nv_get_index(const char *group, const char *token)
{
    // A full mnemonic must fit and match; never truncate a name into an action.
    size_t group_len = strlen(group), token_len = strlen(token);
    if ((group_len > GROUP_LEN) || (token_len > TOKEN_LEN) ||
        (group_len + token_len > TOKEN_LEN)) { return (NO_MATCH); }

	char c;
	char str[TOKEN_LEN + GROUP_LEN+1];	// should actually never be more than TOKEN_LEN+1
	strncpy(str, group, GROUP_LEN+1);
	strncat(str, token, TOKEN_LEN+1);

	index_t i;
	index_t index_max = nv_index_max();

	for (i=0; i < index_max; i++) {
		if ((c = GET_TOKEN_BYTE(token[0])) != str[0]) {	continue; }					// 1st character mismatch
		if ((c = GET_TOKEN_BYTE(token[1])) == NUL) { if (str[1] == NUL) return(i);}	// one character match
		if (c != str[1]) continue;													// 2nd character mismatch
		if ((c = GET_TOKEN_BYTE(token[2])) == NUL) { if (str[2] == NUL) return(i);}	// two character match
		if (c != str[2]) continue;													// 3rd character mismatch
		if ((c = GET_TOKEN_BYTE(token[3])) == NUL) { if (str[3] == NUL) return(i);}	// three character match
		if (c != str[3]) continue;													// 4th character mismatch
		if ((c = GET_TOKEN_BYTE(token[4])) == NUL) { if (str[4] == NUL) return(i);}	// four character match
		if (c != str[4]) continue;													// 5th character mismatch
		if (str[5] == NUL) { return (i); } // exact five-character match
	}
	return (NO_MATCH);
}

/*
 * nv_get_type() - returns command type as a NV_TYPE enum
 */

uint8_t nv_get_type(nvObj_t *nv)
{
	if (nv->token[0] == NUL) { return (NV_TYPE_NULL); }
	if (strcmp_P(nv->token, PSTR("gc")) == 0) { return (NV_TYPE_GCODE); }

	if (strcmp_P(nv->token, PSTR("sr")) == 0) { return (NV_TYPE_REPORT); }
//	if (strcmp_P(nv->token, PSTR("sr")) == 0) {
//        if (nv->valuetype == TYPE_PARENT) {
//            return (NV_TYPE_REPORT);            // actual SR{}s
//        } else {
//	        return (NV_TYPE_CONFIG);            // (sr:f} and {sr:t}
//        }
//    }

	if (strcmp_P(nv->token, PSTR("er")) == 0) { return (NV_TYPE_REPORT); }
	if (strcmp_P(nv->token, PSTR("qr")) == 0) { return (NV_TYPE_REPORT); }
	if (strcmp_P(nv->token, PSTR("n"))  == 0) { return (NV_TYPE_LINENUM); }
	if (strcmp_P(nv->token, PSTR("msg")) == 0) { return (NV_TYPE_MESSAGE); }
	if (strcmp_P(nv->token, PSTR("err")) == 0) { return (NV_TYPE_MESSAGE); }  // errors are reported as messages
	return (NV_TYPE_CONFIG);
}

/******************************************************************************
 * nvObj low-level object and list operations
 *
 * nv_prev()            - return pointer to previous NV or NULL if at start
 * nv_next()            - return pointer to next NV or NULL if at end
 * nv_next_empty()      - return pointer to next empty NV or NULL if none
 *
 * nv_reset_nv()		- quick clear for a new nv object
 * nv_reset_nv_list()	- clear entire header, body and footer for a new use
 * nv_relink_nv_list()  - relink nx and pv pointers removing EMPTY and SKIP
 * nv_populate_nvObj_by_index() - setup a nv object by providing the index
 *
 * nv_copy_string()		- used to write a string to shared string storage and link it
 * nv_add_object()		- write contents of parameter to  first free object in the body
 * nv_add_integer()		- add an integer value to end of nv body (Note 1)
 * nv_add_float()		- add a floating point value to end of nv body
 * nv_add_string()		- add a string object to end of nv body
 * nv_add_message()     - add a message to NV body if messages are enabled
 * nv_add_message_P()   - add a PSTR message to NV body if messages are enabled
 *
 *	Note: Functions that return a nv pointer point to the object that was modified or
 *	a NULL pointer if there was an error.
 *
 *	Note: Adding a really large integer (like a checksum value) may lose precision due
 *	to the cast to a float. Sometimes it's better to load an integer as a string if
 *	all you want to do is display it.
 *
 *	Note: A trick is to cast all string constants for nv_copy_string(), nv_add_object(),
 *	nv_add_string() and nv_add_message() to (const char *). Examples:
 *
 *		nv_add_string((const char *)"msg", string);
 *
 *	On the AVR this will save a little static RAM. The "msg" string will occupy flash
 *	as an initializer and be instantiated in stack RAM when the function executes.
 *	On the ARM (however) this will put the string into flash and skip RAM allocation.
 */

/*
 * Wrappers for nx/pv allowing later substitution w/more space efficient methods
 * Return pointer to next or previous NV object or NULL it at end or beginning
 */
nvObj_t *nv_prev(nvObj_t *nv) { return (nv->pv); }
nvObj_t *nv_next(nvObj_t *nv) { return (nv->nx); }

nvObj_t *nv_next_empty(nvObj_t *nv) {
    do {
        if (nv->valuetype == TYPE_EMPTY) {
            return (nv);
        }
    } while ((nv = nv->nx) != NULL);
    return (NULL);
}

nvObj_t *nv_reset_nv(nvObj_t *nv)			// clear a single nvObj structure
{                                           // depth and pointers are NOT affected
	nv->valuetype = TYPE_EMPTY;				// selective clear is actually faster than calling memset()
	nv->index = NO_MATCH;
	nv->value_int = 0xFFFFFFFF;
	nv->precision = 0;
	nv->token[0] = NUL;
	nv->group[0] = NUL;
	nv->str = NULL;
	return (nv);							// return pointer to nv as a convenience to callers
}

/*
 * nv_reset_nv_list() - clear the NV list and set up as a parent or a plain list
 *
 *  - Clear all nvObjs in the nvList:
 *      - set nvObj[i].valuetype = TYPE_EMPTY
 *      - zero all values and NUL terminate all strings
 *
 *  - nvObj[0] is reserved for 'r' header (NV_HEAD)
 *      - set nvObj[0].valuetype = TYPE_EMPTY
 *      - set nvObj[0].depth = 0
 *
 *  - if "parent" arg is NUL:
 *      - set nvObj[1..N].depth = 1             // set all depths to 1
 *
 *  - else if "parent" arg is non-NUL:
 *      - set nvObj[1].valuetype = TYPE_PARENT
 *      - set nvObj[1].token = "parent"
 *      - set nvObj[1].depth = 1
 *      - set nvObj[2..N].depth = 2             // set all remaining depths to 2
 *
 *  -  return nv pointing to NV_BODY (nvObj[1])
 */

nvObj_t *nv_reset_nv_list(char *parent)
{
    // set up linked list and initialize elements
	nvStr.wp = 0;							    // reset the shared string
    uint8_t nv_num=0;
	nvObj_t *nv = NV_HEAD;                      // nvl.list[0]
    uint8_t depth = (*parent != NUL) ? 2 : 1;   // element depth = 2 if there is a parent
	for (uint8_t i=0; i<NV_LIST_LEN; i++, nv++) {
		nv->pv = (nv-1);	                    // the ends are bogus & corrected later
		nv->nx = (nv+1);
        nv->nv_num = nv_num++;
		nv->depth = depth;
		nv->index = 0;
		nv->precision = 0;
		nv->valuetype = TYPE_EMPTY;
		nv->token[0] = NUL;
	}
	(--nv)->nx = NULL;                          // correct the ends
	nv = nvl.list;
	nv->pv = NULL;

    // reserve r{} element
    nv->depth = 0;

    // setup parent element if one was requested. This is a convenience for calling routines
    nv = NV_BODY;
    if (*parent != NUL) {
	    nv->depth = 1;
	    nv->valuetype = TYPE_PARENT;
	    strcpy(nv->token, parent);
    }
    return (NV_BODY);
}

/*
 * nv_relink_nv_list() - relink nx and pv pointers removing EMPTY and SKIP
 *
 *  Relink list and return pointer to first non-zero element, or NULL if not found
 */
nvObj_t *nv_relink_nv_list()
{
    nvObj_t *nv = NV_HEAD;  // nvl.list     // read pointer - advances for each loop iteration
    nvObj_t *pv;                            // previous non-EMPTY/SKIP pair
    nvObj_t *hd;                            // adjusted head
    uint8_t i=0, j=0;

    // skip past empty/skip leading elements
    for ( ; j<NV_LIST_LEN; j++, i++) {
        if ((nv->valuetype == TYPE_EMPTY) || (nv->valuetype == TYPE_SKIP)) {
            nv = nv_next(nv);
            continue;
        }
        break;
    }
    if (j == NV_LIST_LEN) {                 // empty list
        return (NULL);
    }
    hd = nv;                                // mark the head
    pv = nv;
	for ( ; i<NV_LIST_LEN; i++, nv++) {
        // skip over TYPE_EMPTY and TYPE_SKIP
//        if (nv->valuetype < TYPE_NULL) {
        if ((nv->valuetype == TYPE_EMPTY) || (nv->valuetype == TYPE_SKIP)) { // skip past
            continue;
        }
        pv->nx = nv;                        // Note: the first pair is messed up but gets corrected
        nv->pv = pv;
        pv = nv;
	}
    hd->pv = NULL;                          // correct the ends
    pv->nx = NULL;
    return (hd);
}

/*
 * nv_populate_nv_by_index() - fill in details of an nvObj given the index field
 */

void nv_populate_nv_by_index(nvObj_t *nv, index_t index)
{
	if (index >= nv_index_max()) { return; } // sanity

	nv->str = NULL;
	nv->index = index;
    nv->valuetype = cfg_is_type(index);
    if (nv->valuetype == TYPE_FLOAT) {
       	nv->precision = (int8_t)GET_TABLE_WORD(precision);
    }
	strcpy_P(nv->token, cfgArray[nv->index].token);         // NB: token field is always NUL terminated

    if (cfg_has_flag(nv->index, F_NOSTRIP)) {
        nv->group[0] = NUL;                                 // don't carry the group
    } else {
	    strcpy_P(nv->group, cfgArray[nv->index].group);     // NB: group string is always NUL terminated
        strcpy(nv->token, &nv->token[strlen(nv->group)]);   // strip group from the token
    }
    // run the getter to get the value into nv->value_int or nv->value_flt; through nv_get() so a GET that
    // acts (an md or me in a status report or group) ends runtime quiescence as a direct GET does
	(void)nv_get(nv);
}

/*
 * nv_copy_string() - copy source string to the string allocation buffer and set pointer in NV obj
 */
stat_t nv_copy_string(nvObj_t *nv, const char *src)
{
	if ((nvStr.wp + strlen(src) + 1) > NV_SHARED_STRING_LEN) {	// room for the NUL too (E048)
        return (STAT_BUFFER_FULL);
    }
	char *dst = &(nvStr.string[nvStr.wp]);
	strcpy(dst, src);						// copy string to current head position
											// string has already been tested for overflow, above
	nvStr.wp += strlen(src)+1;				// advance head for next string
	nv->str = dst;
	return (STAT_OK);
}

/* UNUSED
stat_t nv_copy_string_P(nvObj_t *nv, const char *src_P)
{
	char buf[NV_SHARED_STRING_LEN];
	strncpy_P(buf, src_P, NV_SHARED_STRING_LEN);
	return (nv_copy_string(nv, buf));
}
*/

static void _add_object_helper(nvObj_t *nv, const char *token, valueType valuetype)
{
	nv->valuetype = valuetype;
	strncpy(nv->token, token, TOKEN_LEN);

    if ((nv_prev(nv))->valuetype == TYPE_PARENT) {
        nv->depth = (nv_prev(nv))->depth + 1;
    } else {
        nv->depth = (nv_prev(nv))->depth;
    }
}

nvObj_t *nv_add_object(const char *token)  // add an object to the body using a token
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
                return (NULL);               // not supposed to find a NULL; here for safety
            }
			continue;
		}
		// load the index from the token or die trying
		if ((nv->index = nv_get_index((const char *)"",token)) == NO_MATCH) { return (NULL);}
		nv_populate_nv_by_index(nv, nv->index);
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_integer(const char *token, const uint32_t value)// add TYPE_INTEGER object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
        _add_object_helper(nv, token, TYPE_INTEGER);
		nv->value_int = value;
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_data(const char *token, const uint32_t value)// add TYPE_DATA object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
        _add_object_helper(nv, token, TYPE_DATA);
		float *v = (float*)&value;
		nv->value_flt = *v;
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_float(const char *token, const float value)	// add TYPE_FLOAT object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
        _add_object_helper(nv, token, TYPE_FLOAT);
		nv->value_flt = value;
		return (nv);
	}
	return (NULL);
}

// ASSUMES A RAM STRING. If you need to post a FLASH string use pstr2str to convert it to a RAM string
nvObj_t *nv_add_string(const char *token, const char *string) // add a string object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
        _add_object_helper(nv, token, TYPE_STRING);
		if (nv_copy_string(nv, string) != STAT_OK) {
            return (NULL);
        }
		nv->index = nv_get_index((const char *)"", nv->token);
        return (nv);
	}
	return (NULL);
}

/*
 * nv_add_message() - queue a RAM string as a message in the response (conditionally)
 * nv_add_message_P()
 *
 *  Adds a message to the NV list if in JSON mode and messages are enabled for display
 *	Note: If you need to post a FLASH string use pstr2str to convert it to a RAM string
 */

nvObj_t *nv_add_message(const char *msg)
{
	if ((cs.comm_mode == JSON_MODE) && (js.echo_json_messages != true)) { return (NULL);}
	return(nv_add_string((const char *)"msg", msg));
}

nvObj_t *nv_add_message_P(const char *msg_P)
{
    char msg[LINE_MSG_LEN];
    strcpy_P(msg, msg_P);
    return(nv_add_message(msg));
}

/**** nv_print_list() - print nv_array as JSON or text **********************
 *
 * 	Generate and print the JSON and text mode output strings. Use this function
 *	for all text and JSON output that wants to be in a response header.
 *	Don't just printf stuff.
 *
 *	Inputs:
 *	  json_flags = JSON_OBJECT_FORMAT - print just the body w/o header or footer
 *	  json_flags = JSON_RESPONSE_FORMAT - print a full "r" object with footer
 *
 *	  text_flags = TEXT_INLINE_PAIRS - print text as name/value pairs on a single line
 *	  text_flags = TEXT_INLINE_VALUES - print text as comma separated values on a single line
 *	  text_flags = TEXT_MULTILINE_FORMATTED - print text one value per line with formatting string
 */

void nv_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
	if (cs.comm_mode == JSON_MODE) {
		json_print_list(status, json_flags);
	} else {
		text_print_list(status, text_flags);
	}
}
