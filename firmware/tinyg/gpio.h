/*
 * gpio.c - general purpose IO bits - including limit switches, inputs, outputs
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef gpio_h
#define gpio_h

void IndicatorLed_set(void);
void IndicatorLed_clear(void);
void IndicatorLed_toggle(void);

void gpio_led_on(uint8_t led);
void gpio_led_off(uint8_t led);
void gpio_led_toggle(uint8_t led);

uint8_t gpio_read_bit(uint8_t b);
void gpio_set_bit_on(uint8_t b);
void gpio_set_bit_off(uint8_t b);

/* unit test setup */

//#define __UNIT_TEST_GPIO				// uncomment to enable GPIO unit tests
#ifdef __UNIT_TEST_GPIO
void gpio_unit_tests(void);
#define	GPIO_UNITS gpio_unit_tests();
#else
#define	GPIO_UNITS
#endif // __UNIT_TEST_GPIO

#endif
