/*
 * ocb.c - Open Controller Bus driver
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
#include <stdio.h>
#include <stdbool.h>

#include "kinen.h"
#include "kinen_slave_328p.h"			// include headers for specific master or slave used 

/*
 * ki_init() - set up OCB master and slave
 */
void ki_init(void)
{
//	ki_master_init();
	ki_slave_init();
}

/*
 * ki_main_loop() - main event handler
 */
void ki_main_loop(void)
{
	while (true) {
		continue;
	}
}
