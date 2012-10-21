/*
 * kinen_core.h - Kinen core program definitions
 * Part of Kinen Motion Control Project
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * The Kinen Motion Control System is licensed under the OSHW 1.0 license
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

void kinen_init(void);
uint8_t kinen_callback(void);

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

#define DEVICE_TYPE_NULL 0
#define DEVICE_TYPE_DUMB_STEPPER_CONTROLLER 1
#define DEVICE_TYPE_SMART_STEPPER_CONTROLLER 2
#define DEVICE_TYPE_EXTRUDER_CONTROLLER 3
#define DEVICE_TYPE_TEMPERATURE_CONTROLLER 4

// Kinen Status Codes

#define	SC_OK 0							// function completed OK
#define	SC_ERROR 1						// generic error return (EPERM)
#define	SC_EAGAIN 2						// function would block here (call again)
#define	SC_NOOP 3						// function had no-operation
#define	SC_COMPLETE 4					// operation is complete
#define SC_TERMINATE 5					// operation terminated (gracefully)
#define SC_ABORT 6						// operaation aborted
#define	SC_EOL 7						// function returned end-of-line
#define	SC_EOF 8						// function returned end-of-file 
#define	SC_FILE_NOT_OPEN 9
#define	SC_FILE_SIZE_EXCEEDED 10
#define	SC_NO_SUCH_DEVICE 11
#define	SC_BUFFER_EMPTY 12
#define	SC_BUFFER_FULL_FATAL 13 
#define	SC_BUFFER_FULL_NON_FATAL 14

// System errors (HTTP 500's if you will)
#define	SC_INTERNAL_ERROR 20			// unrecoverable internal error
#define	SC_INTERNAL_RANGE_ERROR 21		// number range other than by user input
#define	SC_FLOATING_POINT_ERROR 22		// number conversion error
#define	SC_DIVIDE_BY_ZERO 23
#define	SC_INVALID_ADDRESS 24			// address not in range
#define SC_READ_ONLY_ADDRESS 25			// tried to write tot a read-only location

// Input errors (HTTP 400's, if you will)
#define	SC_UNRECOGNIZED_COMMAND 40		// parser didn't recognize the command
#define	SC_EXPECTED_COMMAND_LETTER 41	// malformed line to parser
#define	SC_BAD_NUMBER_FORMAT 42			// number format error
#define	SC_INPUT_EXCEEDS_MAX_LENGTH 43	// input string is too long 
#define	SC_INPUT_VALUE_TOO_SMALL 44		// input error: value is under minimum
#define	SC_INPUT_VALUE_TOO_LARGE 45		// input error: value is over maximum
#define	SC_INPUT_VALUE_RANGE_ERROR 46	// input error: value is out-of-range
#define	SC_INPUT_VALUE_UNSUPPORTED 47	// input error: value is not supported
#define	SC_JSON_SYNTAX_ERROR 48			// JSON string is not well formed
#define	SC_JSON_TOO_MANY_PAIRS 49		// JSON string or has too many JSON pairs
#define	SC_NO_BUFFER_SPACE 50			// Buffer pool is full and cannot perform this operation

#endif
