/*
 * xmega_adc.c - adc support functions for xmega family
 * Part of TinyG project
 *
 * Copyright (c) 2017 thereza / reza of the Liteplacer.com forum.
 * Copyright (c) 2021 mark@makr.zone
 * 
 * This code is a baked-down version of thereza's solution, posted here:
 * https://www.liteplacer.com/phpBB/viewtopic.php?f=10&t=74&start=10#p483
 * https://www.liteplacer.com/phpBB/viewtopic.php?f=10&t=154#p993
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "xmega_adc.h"

/* --------------------------------- BEGIN ATMEL CODE */

/* 
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#define ADC_ConvMode_Signed   1
#define ADC_ConvMode_Unsigned 0

#define ADC_ConvMode_and_Resolution_Config(_adc, _signedMode, _resolution) \
((_adc)->CTRLB = ((_adc)->CTRLB & (~(ADC_RESOLUTION_gm|ADC_CONMODE_bm)))| \
(_resolution| ( _signedMode? ADC_CONMODE_bm : 0)))

#define ADC_Prescaler_Config(_adc, _div) \
((_adc)->PRESCALER = ((_adc)->PRESCALER & (~ADC_PRESCALER_gm)) | _div)

#define ADC_Reference_Config(_adc, _convRef) \
((_adc)->REFCTRL = ((_adc)->REFCTRL & ~(ADC_REFSEL_gm)) | _convRef)

#define ADC_Ch_InputMode_and_Gain_Config(_adc_ch, _inputMode, _gain) \
(_adc_ch)->CTRL = ((_adc_ch)->CTRL & \
(~(ADC_CH_INPUTMODE_gm|ADC_CH_GAINFAC_gm))) | \
((uint8_t) _inputMode|_gain)

#define ADC_Ch_InputMux_Config(_adc_ch, _posInput, _negInput) \
((_adc_ch)->MUXCTRL = (uint8_t) _posInput | _negInput)

#define ADC_Enable(_adc) ((_adc)->CTRLA |= ADC_ENABLE_bm)

#define ADC_Ch_Conversion_Start(_adc_ch) ((_adc_ch)->CTRL |= ADC_CH_START_bm)

#define ADC_Ch_Conversion_Complete(_adc_ch) \
(((_adc_ch)->INTFLAGS & ADC_CH_CHIF_bm) != 0x00)

uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
}

#define ADCACAL0_offset 0x20
#define ADCACAL1_offset 0x21
#define ADCBCAL0_offset 0x24
#define ADCBCAL1_offset 0x25

void ADC_CalibrationValues_Load(ADC_t * adc)
{
	if(&ADCA == adc){
		/* Get ADCACAL0 from production signature . */
		adc->CALL = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL0_offset );
		adc->CALH = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL1_offset );
		}else {
		/* Get ADCBCAL0 from production signature */
		adc->CALL = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCBCAL0_offset );
		adc->CALH = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCBCAL1_offset );
	}
}

uint16_t ADC_ResultCh_GetWord_Unsigned(ADC_CH_t * adc_ch, uint8_t offset)
{
	uint16_t answer;
	
	/* Clear interrupt flag.*/
	adc_ch->INTFLAGS = ADC_CH_CHIF_bm;
	
	/* Return result register contents*/
	answer = adc_ch->RES - offset;
	
	return answer;
}

/* --------------------------------- END ATMEL CODE ---------------------------*/


/* There are two ADC pins available
 * J15 pin 2 = pB0
 * J13 pin 2 = PB3 <- this is the best option as it's only confiured at /SS2 For an external SPI Interface
*/
void adc_init() 
{
	/* Move stored calibration values to ADC. */
	ADC_CalibrationValues_Load(&ADCB);
	
	/* Set up ADC to have unsigned conversion mode and 12 bit resolution. */
	ADC_ConvMode_and_Resolution_Config(&ADCB, ADC_ConvMode_Unsigned, ADC_RESOLUTION_12BIT_gc);
	
	/* Set sample rate. */
	ADC_Prescaler_Config(&ADCB, ADC_PRESCALER_DIV32_gc);
	
	/* Set reference voltage on ADC to be VCC/1.6 V.*/
	ADC_Reference_Config(&ADCB, ADC_REFSEL_VCC_gc);
	
	/* Setup channel 0 */
	ADC_Ch_InputMode_and_Gain_Config(&ADCB.CH0, ADC_CH_INPUTMODE_SINGLEENDED_gc, ADC_CH_GAIN_1X_gc);
	
	/* Set input to the channels in ADC */
	ADC_Ch_InputMux_Config(&ADCB.CH0, ADC_CH_MUXPOS_PIN3_gc, 0);

	/* Enable ADC */
	ADC_Enable(&ADCB);
}

uint16_t adc_read() 
{
	// get value
	ADC_Ch_Conversion_Start(&ADCB.CH0);
	while(!ADC_Ch_Conversion_Complete(&ADCB.CH0));
	uint16_t result = ADC_ResultCh_GetWord_Unsigned(&ADCB.CH0, 0);
	
	// return it
	return (result);
}

