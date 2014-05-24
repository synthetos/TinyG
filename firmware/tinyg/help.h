/*
 * help.h - collected help and assorted display routines
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
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

#ifndef HELP_H_ONCE
#define HELP_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __HELP_SCREENS

	stat_t help_general(nvObj_t *nv);
	stat_t help_config(nvObj_t *nv);
	stat_t help_test(nvObj_t *nv);
	stat_t help_defa(nvObj_t *nv);
	stat_t help_boot_loader(nvObj_t *nv);

#else

	stat_t help_stub(nvObj_t *nv);
	#define help_general help_stub
	#define help_config help_stub
	#define help_test help_stub
	#define help_defa help_stub
	#define help_boot_loader help_stub

#endif // __HELP_SCREENS

#ifdef __cplusplus
}
#endif

#endif // End of include guard: HELP_H_ONCE
