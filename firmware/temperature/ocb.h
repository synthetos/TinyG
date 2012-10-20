/*
 * ocb.h - Open Controller Bus (OCB) program definitions
 * Part of Open Controller Bus project
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * Open Controller Bus (OCB) is licensed under the OSHW 1.0 license
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ocb_h
#define ocb_h

#include "ocb_defs.h"	// Device types and other evolving definitions are isolated here

// function prototypes

void ocb_init(void);
void ocb_main_loop(void);

// OCB definitions

#define OCB_ERR_BYTE 0xA5
#define OCB_OK_BYTE 0x5A

#define OCB_ADDR 0		// SPI phasing
#define OCB_DATA 1

#define OCB_READ 0		// Command register values
#define OCB_WRITE 1

enum OCBCommon {  		// all are read-only except as noted
	OCB_COMMAND = 0,	// writable
	OCB_STATUS,
	OCB_POLL,
	OCB_WAIT_TIME,
	OCB_ADDR_PAGE,		// writable
	OCB_RESET,			// writable
	OCB_RESERVED1,
	OCB_RESERVED2,
	OCB_DEVICE_TYPE,
	OCB_DEVICE_ID_HI,
	OCB_DEVICE_ID_LO,
	OCB_DEVICE_REV_MAJOR,
	OCB_DEVICE_REV_MINOR,
	OCB_DEVICE_UUID_1,
	OCB_DEVICE_UUID_2,
	OCB_DEVICE_UUID_3,
	OCB_COMMON_MAX		// always last
};

// Common register storage and naming 

uint8_t ocb_array[16];	// it's here so it can be used by both master and slave

#define ocb_command ocb_array[OCB_COMMAND]
#define ocb_status ocb_array[OCB_STATUS]
#define ocb_poll ocb_array[OCB_POLL]
#define ocb_wait_time ocb_array[OCB_WAIT_TIME]
#define ocb_addr_page ocb_array[OCB_ADDR_PAGE]
#define ocb_reset ocb_array[OCB_RESET]
#define ocb_device_type ocb_array[OCB_DEVICE_TYPE]
#define ocb_device_id_hi ocb_array[OCB_DEVICE_ID_HI]
#define ocb_device_id_lo ocb_array[OCB_DEVICE_ID_LO]
#define ocb_device_rev_major ocb_array[OCB_DEVICE_REV_MAJOR]
#define ocb_device_rev_minor ocb_array[OCB_DEVICE_REV_MINOR]
#define ocb_device_uuid_1 ocb_array[OCB_DEVICE_UUID_1]
#define ocb_device_uuid_2 ocb_array[OCB_DEVICE_UUID_2]
#define ocb_device_uuid_3 ocb_array[OCB_DEVICE_UUID_3]

#endif
