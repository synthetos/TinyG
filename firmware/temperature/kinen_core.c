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
 * kinen_init() - set up Kine subsystems; master or slave
 */
void kinen_init(void)
{
//	kinen_master_init();
	kinen_slave_init();
}

/*
 * kinen_callback() - kinen event handler
 *
 *	This function should be called from the main loop at a high-priority level
 *	Returns OK or error. Returns EAGAIN if function would block
 */
uint8_t kinen_callback(void)
{
	return (SC_NOOP);
}

