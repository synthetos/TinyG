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

#ifndef kinen_h
#define kinen_h

// function prototypes

void ki_init(void);
void ki_main_loop(void);

// Kinen definitions

#define KINEN_ERR_BYTE 0xA5
#define KINEN_OK_BYTE 0x5A

#define KINEN_ADDR 0		// SPI phasing
#define KINEN_DATA 1

#define KINEN_READ 0		// Command register values
#define KINEN_WRITE 1


enum KINENCommon {  		// all are read-only except as noted
	KINEN_COMMAND = 0,	// writable
	KINEN_STATUS,
	KINEN_POLL,
	KINEN_WAIT_TIME,
	KINEN_ADDR_PAGE,		// writable
	KINEN_RESET,			// writable
	KINEN_RESERVED1,
	KINEN_RESERVED2,
	KINEN_DEVICE_TYPE,
	KINEN_DEVICE_ID_HI,
	KINEN_DEVICE_ID_LO,
	KINEN_DEVICE_REV_MAJOR,
	KINEN_DEVICE_REV_MINOR,
	KINEN_DEVICE_UUID_1,
	KINEN_DEVICE_UUID_2,
	KINEN_DEVICE_UUID_3,
	KINEN_COMMON_MAX		// always last
};

// Common register storage and naming 

uint8_t ki_array[16];	// it's here so it can be used by both master and slave

#define ki_command 			ki_array[KINEN_COMMAND]
#define ki_status			ki_array[KINEN_STATUS]
#define ki_poll				ki_array[KINEN_POLL]
#define ki_wait_time		ki_array[KINEN_WAIT_TIME]
#define ki_addr_page		ki_array[KINEN_ADDR_PAGE]
#define ki_reset			ki_array[KINEN_RESET]
#define ki_device_type		ki_array[KINEN_DEVICE_TYPE]
#define ki_device_id_hi		ki_array[KINEN_DEVICE_ID_HI]
#define ki_device_id_lo		ki_array[KINEN_DEVICE_ID_LO]
#define ki_device_rev_major ki_array[KINEN_DEVICE_REV_MAJOR]
#define ki_device_rev_minor ki_array[KINEN_DEVICE_REV_MINOR]
#define ki_device_uuid_1 	ki_array[KINEN_DEVICE_UUID_1]
#define ki_device_uuid_2 	ki_array[KINEN_DEVICE_UUID_2]
#define ki_device_uuid_3 	ki_array[KINEN_DEVICE_UUID_3]


// Kinen Device Types

#define KINEN_DEVICE_TYPE_NULL 0
#define KINEN_DEVICE_TYPE_STEPPER_CONTROLLER 1

// Kinen Status Codes

#define	KINEN_SC_OK 0						// function completed OK
#define	KINEN_SC_ERROR 1					// generic error return (EPERM)
#define	KINEN_SC_EAGAIN 2					// call again (for iterators and non-blocking)
#define	KINEN_SC_INVALID_ADDRESS 3		// address not in range
#define KINEN_SC_READ_ONLY_ADDRESS 4		// tried to write tot a read-only location


#endif
