/*
 * ocb_slave_328p - Open Controller Bus slave driver for Atmega328P 
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
/* The SPI peripheral is used for the SPI slave using its MOSI, MISO, SCK, SS pins.
 * The slave device takes OCB instructions from the motherboard.
 * A bit-banger master SPI is provided to talk to a downstream device
 */

#ifndef ocb_slave_328p_h
#define ocb_slave_328p_h

void ocb_slave_init(void);

#endif
