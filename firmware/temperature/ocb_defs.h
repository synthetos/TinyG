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
/*
 * This file contains definitions of OCB device types, registered device IDs 
 * and other information that may need to be updated periodically.
 */

#ifndef ocb_defs_h
#define ocb_defs_h

// OCB Device Types

#define OCB_DEVICE_TYPE_NULL 0
#define OCB_DEVICE_TYPE_STEPPER_CONTROLLER 1

// OCB Status Codes

#define	OCB_SC_OK 0						// function completed OK
#define	OCB_SC_ERROR 1					// generic error return (EPERM)
#define	OCB_SC_EAGAIN 2					// call again (for iterators and non-blocking)
#define	OCB_SC_INVALID_ADDRESS 3		// address not in range
#define OCB_SC_READ_ONLY_ADDRESS 4		// tried to write tot a read-only location

/*
#define	OCB_SC_NOOP 3					// function had no-operation
#define	OCB_SC_COMPLETE 4				// operation is complete
#define OCB_SC_TERMINATE 5				// operation terminated (gracefully)
#define OCB_SC_ABORT 6					// operation aborted
#define	OCB_SC_EOL 7					// function returned end-of-line
#define	OCB_SC_EOF 8					// function returned end-of-file 
#define	OCB_SC_FILE_NOT_OPEN 9
#define	OCB_SC_FILE_SIZE_EXCEEDED 10
#define	OCB_SC_NO_SUCH_DEVICE 11
#define	OCB_SC_BUFFER_EMPTY 12
#define	OCB_SC_BUFFER_FULL_FATAL 13 
#define	OCB_SC_BUFFER_FULL_NON_FATAL 14	// NOTE: XIO codes align to here
*/
#endif
