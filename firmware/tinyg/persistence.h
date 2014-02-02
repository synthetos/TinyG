/*
 * persistence.h - persistence code
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart Jr.
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

#ifndef PERSISTENCE_H_ONCE
#define PERSISTENCE_H_ONCE

#include "config.h"					// needed for cmdObj_t definition

#ifdef __cplusplus
extern "C"{
#endif 

#define NVM_VALUE_LEN 4				// NVM value length (float, fixed length)
#define NVM_BASE_ADDR 0x0000		// base address of usable NVM

//**** persistence singleton ****

typedef struct nvmSingleton {
	uint16_t nvm_base_addr;			// NVM base address
	uint16_t nvm_profile_base;		// NVM base address of current profile
} nvmSingleton_t;

//**** persistence function prototypes ****

void persistence_init(void);
stat_t read_persistent_value(cmdObj_t *cmd);
stat_t write_persistent_value(cmdObj_t *cmd);

#ifdef __DEBUG
void cfg_dump_NVM(const uint16_t start_record, const uint16_t end_record, uint8_t *label);
#endif

/*** Unit tests ***/

/* unit test setup */
//#define __UNIT_TEST_PERSISTENCE			// uncomment to enable config unit tests
#ifdef __UNIT_TEST_PERSISTENCE
void cfg_unit_tests(void);
#define	PERSISTENCE_UNITS persist_unit_tests();
#else
#define	PERSISTENCE_UNITS
#endif // __UNIT_TEST_PERSISTENCE

#ifdef __cplusplus
}
#endif

#endif // End of include guard: PERSISTENCE_H_ONCE
