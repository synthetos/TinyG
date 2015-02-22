/*
 * encoder.c - encoder interface
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2015 Alden S. Hart, Jr.
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
#include "encoder.h"

#ifdef __cplusplus
extern "C"{
#endif

/**** Allocate Structures ****/

enEncoders_t en;

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/

/*
 * encoder_init() - initialize encoders
 */

void encoder_init()
{
	memset(&en, 0, sizeof(en));		// clear all values, pointers and status
	encoder_init_assertions();
}

/*
 * encoder_init_assertions() - initialize encoder assertions
 * encoder_test_assertions() - test assertions, return error code if violation exists
 */

void encoder_init_assertions()
{
	en.magic_end = MAGICNUM;
	en.magic_start = MAGICNUM;
}

stat_t encoder_test_assertions()
{
	if (en.magic_end   != MAGICNUM) return (STAT_ENCODER_ASSERTION_FAILURE);
	if (en.magic_start != MAGICNUM) return (STAT_ENCODER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * en_set_encoder_steps() - set encoder values to a current step count
 *
 *	Sets the encoder_position steps. Takes floating point steps as input,
 *	writes integer steps. So it's not an exact representation of machine
 *	position except if the machine is at zero.
 */

void en_set_encoder_steps(uint8_t motor, float steps)
{
	en.en[motor].encoder_steps = (int32_t)round(steps);
}

/*
 * en_read_encoder()
 *
 *	The stepper ISR count steps into steps_run(). These values are accumulated to
 *	encoder_position during LOAD (HI interrupt level). The encoder position is
 *	therefore always stable. But be advised: the position lags target and position
 *	valaues elsewherein the system becuase the sample is taken when the steps for
 *	that segment are complete.
 */

float en_read_encoder(uint8_t motor)
{
	return((float)en.en[motor].encoder_steps);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif
