/*
 * json_parser.c - JSON parser for rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2012 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef json_parser_h
#define json_parser_h

/*
 * Global Scope Functions
 */

void js_init(void);						// Initialize the parser
uint8_t js_json_parser(char *in_str, char *out_str);
uint16_t js_make_json_string(cmdObj *cmd, char *str);
uint8_t js_make_json_response(uint8_t status, char *out_str);


/* unit test setup */

//#define __UNIT_TEST_JSON				// uncomment to enable JSON unit tests
#ifdef __UNIT_TEST_JSON
void js_unit_tests(void);
#define	JSON_UNITS js_unit_tests();
#else
#define	JSON_UNITS
#endif

#endif
