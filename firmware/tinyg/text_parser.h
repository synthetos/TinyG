/*
 * text_parser.h - text parser and text mode support for tinyg2
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef TEXT_PARSER_H_ONCE
#define TEXT_PARSER_H_ONCE

#include <stdbool.h>

/*
 * Global Scope Functions
 */

stat_t text_parser(char_t *str);
void text_response(const uint8_t status, char_t *buf);
void text_print_list(stat_t status, uint8_t flags);
void text_print_inline_pairs(cmdObj_t *cmd);
void text_print_inline_values(cmdObj_t *cmd);
void text_print_multiline_formatted(cmdObj_t *cmd);

/* unit test setup */

//#define __UNIT_TEST_TEXT				// uncomment to enable TEXT unit tests
#ifdef __UNIT_TEST_TEXT
void text_unit_tests(void);
#define	TEXT_UNITS text_unit_tests();
#else
#define	TEXT_UNITS
#endif // __UNIT_TEST_TEXT

#endif
