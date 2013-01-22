/*
 * json_parser.c - JSON parser for rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2012 Alden S. Hart, Jr.
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

#ifndef json_parser_h
#define json_parser_h

/* JSON array definitions / revisions */
// for now there is only one JSON array in use - the footer
// if you add these make sure there are no collisions w/present or past numbers

#define FOOTER_REVISION 1

#define JSON_OUTPUT_STRING_MAX (OUTPUT_BUFFER_LEN)
#define JSON_MAX_DEPTH 4

/*
 * Global Scope Functions
 */

void js_json_parser(char *str);
uint16_t js_serialize_json(cmdObj_t *cmd, char *out_buf);
//void js_print_list(uint8_t status);
void js_print_json_object(cmdObj_t *cmd);
void js_print_json_response(cmdObj_t *cmd, uint8_t status);

/* unit test setup */

#define __UNIT_TEST_JSON				// uncomment to enable JSON unit tests
#ifdef __UNIT_TEST_JSON
void js_unit_tests(void);
#define	JSON_UNITS js_unit_tests();
#else
#define	JSON_UNITS
#endif // __UNIT_TEST_JSON

#endif
