/*
 * help.h - collected help and assorted display routines
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef help_h
#define help_h

#ifdef __TEXT_MODE
	stat_t help_general(void);
	stat_t help_config(cmdObj_t *cmd);
	stat_t help_test(cmdObj_t *cmd);
	stat_t help_defaults(cmdObj_t *cmd);
	stat_t help_boot_loader(cmdObj_t *cmd);
#else
	#define help_general tx_print_stub
	#define help_config tx_print_stub
	#define help_test tx_print_stub
	#define help_defaults tx_print_stub
	#define help_boot_loader tx_print_stub
#endif

#endif
