/*
 * kinen_core.c - Kinen Motion Control System main file
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
#include <stdio.h>
#include <stdbool.h>

#include "kinen_core.h"
#include "kinen_slave_328p.h"

/*
 * kinen_init() - set up Kinen subsystems; master or slave
 *
 *	Would like some kind of auto-detect here. For now it's just commenting
 */
void kinen_init(void)
{
//	kinen_master_init();
	kinen_slave_init();
}

/*
 * kinen_callback() - kinen event handler
 *
 *	This function should be called from the main loop at a high-priority level.
 *	It is used to perform any polling, long-running continuations, or other 
 *	scheduled tasks
 *
 *	Returns SC_OK or error code for normal operations 
 *	Returns SC_NOOP if no operation was performed
 *	Returns EAGAIN if the function should block lower-priority functions
 *	in the main DISPATCH loop
 */
uint8_t kinen_callback(void)
{
//	return (SC_OK);			// return from a successful operation
//	return (SC_EAGAIN);		// return if lower priority tasks should be blocked
	return (SC_NOOP);		// return if no operation was performed 
}

