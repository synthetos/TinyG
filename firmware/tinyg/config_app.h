/*
 * config_app.h - application-specific part of configuration data
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

#ifndef CONFIG_APP_H_ONCE
#define CONFIG_APP_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

//#include <stdbool.h>


 /***********************************************************************************
  **** APPLICATION_SPECIFIC DEFINITIONS AND SETTINGS ********************************
  ***********************************************************************************/

enum cmdType {						// classification of commands
	CMD_TYPE_NULL = 0,
	CMD_TYPE_CONFIG,				// configuration commands
	CMD_TYPE_GCODE,					// gcode
	CMD_TYPE_REPORT,				// SR, QR and any other report
	CMD_TYPE_MESSAGE,				// cmd object carries a message
	CMD_TYPE_LINENUM				// cmd object carries a gcode line number
};

/***********************************************************************************
 **** APPLICATION_SPECIFIC CONFIG STRUCTURE(S) *************************************
 ***********************************************************************************/

typedef struct cfgParameters {
	uint16_t magic_start;			// magic number to test memory integrity

	// communications settings
	uint8_t comm_mode;				// TG_TEXT_MODE or TG_JSON_MODE
	uint8_t enable_cr;				// enable CR in CRFL expansion on TX
	uint8_t enable_echo;			// enable text-mode echo
	uint8_t enable_flow_control;	// enable XON/XOFF or RTS/CTS flow control
//	uint8_t ignore_crlf;			// ignore CR or LF on RX --- these 4 are shadow settings for XIO cntrl bits

	uint8_t usb_baud_rate;			// see xio_usart.h for XIO_BAUD values
	uint8_t usb_baud_flag;			// technically this belongs in the controller singleton

	// Non-volatile RAM
//	uint16_t nvm_base_addr;			// NVM base address
//	uint16_t nvm_profile_base;		// NVM base address of current profile

	uint16_t magic_end;
} cfgParameters_t;
extern cfgParameters_t cfg;


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

stat_t set_baud_callback(void);

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

	void co_print_ec(cmdObj_t *cmd);
	void co_print_ee(cmdObj_t *cmd);
	void co_print_ex(cmdObj_t *cmd);
	void co_print_baud(cmdObj_t *cmd);
	void co_print_net(cmdObj_t *cmd);
	void co_print_rx(cmdObj_t *cmd);

#else 

	#define co_print_ec tx_print_nul
	#define co_print_ee tx_print_nul
	#define co_print_ex tx_print_nul
	#define co_print_baud tx_print_nul
	#define co_print_net tx_print_nul
	#define co_print_rx tx_print_nul

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif // __cplusplus

#endif //End of include guard: CONFIG_APP_H_ONCE
