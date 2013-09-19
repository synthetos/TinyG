/*
 * report.h - TinyG status report and other reporting functions
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef report_h
#define report_h

/**** Configs, Definitions and Structures ****/

#define CMD_STATUS_REPORT_LEN CMD_MAX_OBJECTS 	// max number of status report elements - see cfgArray
									// **** must also line up in cfgArray, se00 - seXX ****

enum srVerbosity {					// status report enable and verbosity
	SR_OFF = 0,						// no reports
	SR_FILTERED,					// reports only values that have changed from the last report
	SR_VERBOSE						// reports all values specified
};

enum qrVerbosity {					// planner queue enable and verbosity
	QR_OFF = 0,						// no response is provided
	QR_FILTERED,					// queue depth reported only above hi-water mark and below lo-water mark  
	QR_VERBOSE,						// queue depth reported for all buffers
	QR_TRIPLE						// queue depth reported for all buffers, and buffers added, buffered removed
};

typedef struct srSingleton {

	/*** config values (PUBLIC) ***/
	uint8_t status_report_verbosity;					// see enum in this file for settings
	uint32_t status_report_interval;					// in milliseconds

	/*** runtime values (PRIVATE) ***/
	uint8_t status_report_requested;					// status report has been requested
	uint32_t status_report_systick;						// SysTick value for next status report
	index_t status_report_list[CMD_STATUS_REPORT_LEN];	// status report elements to report
	float status_report_value[CMD_STATUS_REPORT_LEN];	// previous values for filtered reporting

} srSingleton_t;

typedef struct qrSingleton {		// data for queue reports

	/*** config values (PUBLIC) ***/
	uint8_t queue_report_verbosity;	// queue reports enabled and verbosity level
	uint8_t queue_report_hi_water;
	uint8_t queue_report_lo_water;

	/*** runtime values (PRIVATE) ***/
	uint8_t request;				// set to true to request a report
	uint8_t buffers_available;		// stored value used by callback
	uint8_t prev_available;			// used to filter reports
	uint8_t buffers_added;			// buffers added since last report
	uint8_t buffers_removed;		// buffers removed since last report

} qrSingleton_t;

/**** Externs - See report.c for allocation ****/

extern srSingleton_t sr;
extern qrSingleton_t qr;

/**** Function Prototypes ****/

char *get_status_message(stat_t status);
//char *rpt_get_status_message(uint8_t status, char *msg);
void rpt_print_message(char *msg);
void rpt_exception(uint8_t status, int16_t value);
void rpt_print_loading_configs_message(void);
void rpt_print_initializing_message(void);
void rpt_print_system_ready_message(void);

void rpt_init_status_report(void);
stat_t rpt_set_status_report(cmdObj_t *cmd);
void rpt_decr_status_report(void);
void rpt_request_status_report(uint8_t request_type);
stat_t rpt_status_report_callback(void);
void rpt_run_text_status_report(void);
void rpt_populate_unfiltered_status_report(void);
uint8_t rpt_populate_filtered_status_report(void);

void rpt_clear_queue_report(void);
//void rpt_request_queue_report(void);
void rpt_request_queue_report(int8_t buffers);
stat_t rpt_queue_report_callback(void);

// If you are looking for the defaults for the status report see config.h

/* unit test setup */
//#define __UNIT_TEST_REPORT	// uncomment to enable report unit tests
#ifdef __UNIT_TEST_REPORT
void sr_unit_tests(void);
#define	REPORT_UNITS sr_unit_tests();
#else
#define	REPORT_UNITS
#endif // __UNIT_TEST_REPORT

#endif

