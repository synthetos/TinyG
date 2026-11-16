/*
 * coolant.c - canonical coolant commands and output state.
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2016 Robert Giseburt
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
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "cm_internal.h"
#include "planner.h"
#include "feedlimit.h"
#include "gpio.h"
#include "hardware.h"
#include "util.h"

static void _exec_mist_coolant_control(float *value, bool *flags);

/***********************************
 * Miscellaneous Functions (4.3.9) *
 ***********************************/
/*
 * cm_mist_coolant_control() - M7
 * cm_flood_coolant_control() - M8, M9
 */

stat_t cm_mist_coolant_control(const uint8_t mist_coolant)
{
	float value[AXES] = { (float)mist_coolant };
	mp_queue_command_class(_exec_mist_coolant_control, value, FLAGS_ONE, FL_CLS_COOLANT);
	return (STAT_OK);
}
// Coolant outputs as last driven, for the g2core-named cof and com readouts: the model runs ahead of the
// outputs (M2/M30 turns it off when parsed), so the readouts follow the executors instead.
static volatile uint8_t coolant_out_mist, coolant_out_flood;

stat_t coolant_get_cof(nvObj_t *nv) { nv->value_int = coolant_out_flood; nv->valuetype = TYPE_INTEGER; return (STAT_OK); }
stat_t coolant_get_com(nvObj_t *nv) { nv->value_int = coolant_out_mist; nv->valuetype = TYPE_INTEGER; return (STAT_OK); }

static void _exec_mist_coolant_control(float *value, bool *flags)
{
    (void)flags;
	cm.gm.mist_coolant = cm_outputs_inhibited() ? false : (uint8_t)value[0];
	coolant_out_mist = cm.gm.mist_coolant;

#ifdef __AVR
	if (cm.gm.mist_coolant == true) {
		gpio_set_bit_on(MIST_COOLANT_BIT);
    } else {
	    gpio_set_bit_off(MIST_COOLANT_BIT);
    }
#endif // __AVR

#ifdef __ARM
	if (cm.gm.mist_coolant == true) {
		coolant_enable_pin.set();
    } else {
	    coolant_enable_pin.clear();
    }
#endif // __ARM
}

stat_t cm_flood_coolant_control(const uint8_t flood_coolant)
{
	float value[AXES] = { (float)flood_coolant };
	mp_queue_command_class(cm_exec_flood_coolant_control, value, FLAGS_ONE, FL_CLS_COOLANT);
	return (STAT_OK);
}
/*
 * cm_coolant_off_immediate() - turn mist and flood coolant off now (alarm, shutdown, panic)
 */
void cm_coolant_off_immediate()
{
    float value[] = { 0,0,0,0,0,0 };
    cm_exec_flood_coolant_control(value, FLAGS_NONE);   // the M9 path also turns mist off
}

void cm_exec_flood_coolant_control(float *value, bool *flags)
{
    (void)flags;
	cm.gm.flood_coolant = cm_outputs_inhibited() ? false : (uint8_t)value[0];
	coolant_out_flood = cm.gm.flood_coolant;

#ifdef __AVR
	if (cm.gm.flood_coolant == true) {
		gpio_set_bit_on(FLOOD_COOLANT_BIT);
	} else {
		gpio_set_bit_off(FLOOD_COOLANT_BIT);
		float vect[] = { 0,0,0,0,0,0 };				    // turn off mist coolant
		_exec_mist_coolant_control(vect, FLAGS_NONE);   // M9 special function
	}
#endif // __AVR

#ifdef __ARM
	if (cm.gm.flood_coolant == true) {
		coolant_enable_pin.set();
	} else {
		coolant_enable_pin.clear();
		float vect[] = { 0,0,0,0,0,0 };				    // turn off mist coolant
		_exec_mist_coolant_control(vect, FLAGS_NONE);   // M9 special function
	}
#endif // __ARM
}
