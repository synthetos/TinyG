/*
 * coolant.cpp - canonical machine coolant driver
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart, Jr.
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

#include "tinyg.h"              // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "coolant.h"
#include "planner.h"
#include "hardware.h"
#include "gpio.h"               // needed for AVR only
#include "util.h"

/**** Allocate structures ****/

cmCoolantSingleton_t coolant;

/**** Static functions ****/

static void _exec_coolant_control(float *value, bool *flags);

/*
 * coolant_init()
 * coolant_reset()
 */
void coolant_init()
{
    coolant.mist_enable = COOLANT_OFF;
    coolant.flood_enable = COOLANT_OFF;
}

void coolant_reset()
{
    coolant_init();
    cm_coolant_off_immediate();
}

/*
 * cm_coolant_off_immediate() - turn off all coolant
 * cm_coolant_optional_pause() - pause coolants if option is true
 * cm_coolant_resume() - restart paused coolants
 */

void cm_coolant_off_immediate()
{
    float value[] = { 0,0,0,0,0,0 };
    bool flags[] = { 1,1,0,0,0,0 };
    _exec_coolant_control(value, flags);
}

void cm_coolant_optional_pause(bool option)
{
    if (option) {
        float value[] = { 0,0,0,0,0,0 };
        bool flags[] = { 0,0,0,0,0,0 };

        if (coolant.flood_enable == COOLANT_ON) {
            coolant.flood_enable = COOLANT_PAUSE;   // mark as paused
            flags[COOLANT_FLOOD] = 1;               // set flag so it will turn off
        }
        if (coolant.mist_enable == COOLANT_ON) {
            coolant.mist_enable = COOLANT_PAUSE;     // mark as paused
            flags[COOLANT_MIST] = 1;
        }
        _exec_coolant_control(value, flags);        // execute (w/o changing local state)
    }
}

void cm_coolant_resume()
{
//    float value[] = { 1,1,0,0,0,0 };  // ++++ Will this work? No need to set 'value' below
    float value[] = { 0,0,0,0,0,0 };
    bool flags[] = { 0,0,0,0,0,0 };

    if (coolant.flood_enable == COOLANT_PAUSE) {
        coolant.flood_enable = COOLANT_ON;
        value[COOLANT_FLOOD] = 1;
        flags[COOLANT_FLOOD] = true;
    }
    if (coolant.mist_enable == COOLANT_PAUSE) {
        coolant.mist_enable = COOLANT_ON;
        value[COOLANT_MIST] = 1;
        flags[COOLANT_MIST] = true;
    }
    _exec_coolant_control(value, flags);
}

/*
 * cm_mist_coolant_control() - access points from Gcode parser
 * cm_flood_coolant_control() - access points from Gcode parser
 * _exec_coolant_control() - combined flood and mist coolant control
 *
 *  - value[0] is flood state
 *  - value[1] is mist state
 *  - uses flags to determine which to run
 */

stat_t cm_flood_coolant_control(uint8_t flood_enable)
{
    float value[] = { (float)flood_enable, 0,0,0,0,0 };
    bool flags[] = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_coolant_control, value, flags);
    return (STAT_OK);
}

stat_t cm_mist_coolant_control(uint8_t mist_enable)
{
    float value[] = { 0, (float)mist_enable, 0,0,0,0 };
    bool flags[] = { 0,1,0,0,0,0 };
    mp_queue_command(_exec_coolant_control, value, flags);
    return (STAT_OK);
}

#ifdef __ARM
    // NOTE: flood and mist coolants are mapped to the same pin - see hardware.h
    #define _set_flood_enable_bit_hi() flood_enable_pin.set()
    #define _set_flood_enable_bit_lo() flood_enable_pin.clear()
    #define _set_mist_enable_bit_hi() mist_enable_pin.set()
    #define _set_mist_enable_bit_lo() mist_enable_pin.clear()
#endif
#ifdef __AVR
    #define _set_flood_enable_bit_hi() gpio_set_bit_on(COOLANT_BIT)
    #define _set_flood_enable_bit_lo() gpio_set_bit_off(COOLANT_BIT)
    #define _set_mist_enable_bit_hi() gpio_set_bit_on(COOLANT_BIT)
    #define _set_mist_enable_bit_lo() gpio_set_bit_off(COOLANT_BIT)
#endif

static void _exec_coolant_control(float *value, bool *flags)
{
    if (flags[COOLANT_FLOOD]) {
        coolant.flood_enable = (cmCoolantEnable)value[COOLANT_FLOOD];
        if (!((coolant.flood_enable & 0x01) ^ coolant.flood_polarity)) {    // inverted XOR
            _set_flood_enable_bit_hi();
        } else {
            _set_flood_enable_bit_lo();
        }
    }
    if (flags[COOLANT_MIST]) {
        coolant.mist_enable = (cmCoolantEnable)value[COOLANT_MIST];
        if (!((coolant.mist_enable & 0x01) ^ coolant.mist_polarity)) {
            _set_mist_enable_bit_hi();
        } else {
            _set_mist_enable_bit_lo();
        }
    }
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_coph[] PROGMEM = "[coph] coolant pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_comp[] PROGMEM = "[comp] coolant mist polarity%7d [0=low is ON,1=high is ON]\n";
const char fmt_cofp[] PROGMEM = "[cofp] coolant flood polarity%6d [0=low is ON,1=high is ON]\n";
const char fmt_com[] PROGMEM = "Mist coolant:%6d [0=OFF,1=ON]\n";
const char fmt_cof[] PROGMEM = "Flood coolant:%5d [0=OFF,1=ON]\n";

void cm_print_coph(nvObj_t *nv) { text_print(nv, fmt_coph);}  // TYPE_INT
void cm_print_comp(nvObj_t *nv) { text_print(nv, fmt_comp);}  // TYPE_INT
void cm_print_cofp(nvObj_t *nv) { text_print(nv, fmt_cofp);}  // TYPE_INT
void cm_print_com(nvObj_t *nv) { text_print(nv, fmt_com);}    // TYPE_INT
void cm_print_cof(nvObj_t *nv) { text_print(nv, fmt_cof);}    // TYPE_INT

#endif // __TEXT_MODE
