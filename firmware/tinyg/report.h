/*
 * report.h - TinyG status report and other reporting functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
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

#ifndef REPORT_H_ONCE
#define REPORT_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

/**** Configs, Definitions and Structures ****/
//
// Notes:
//		- The NV_STATUS_REPORT_LEN define is in config.h
//		- The status report defaults can be found in settings.h

#define MIN_ARC_QR_INTERVAL 200					// minimum interval between QRs during arc generation (in system ticks)

enum srVerbosity {								// status report enable and verbosity
	SR_OFF = 0,									// no reports
	SR_FILTERED,								// reports only values that have changed from the last report
	SR_VERBOSE									// reports all values specified
};

enum cmStatusReportRequest {
	SR_TIMED_REQUEST = 0,						// request a status report at next timer interval
	SR_IMMEDIATE_REQUEST						// request a status report ASAP
};

enum qrVerbosity {								// planner queue enable and verbosity
	QR_OFF = 0,									// no response is provided
	QR_SINGLE,									// queue depth reported
	QR_TRIPLE									// queue depth reported for buffers, buffers added, buffered removed
};

typedef struct srSingleton {

	/*** config values (PUBLIC) ***/
	uint8_t status_report_verbosity;
	uint32_t status_report_interval;					// in milliseconds

	/*** runtime values (PRIVATE) ***/
	uint8_t status_report_requested;					// flag that SR has been requested
	uint32_t status_report_systick;						// SysTick value for next status report
	index_t stat_index;									// table index value for stat - determined during initialization
	index_t status_report_list[NV_STATUS_REPORT_LEN];	// status report elements to report
	float status_report_value[NV_STATUS_REPORT_LEN];	// previous values for filtered reporting

} srSingleton_t;

typedef struct qrSingleton {		// data for queue reports

	/*** config values (PUBLIC) ***/
	uint8_t queue_report_verbosity;	// queue reports enabled and verbosity level

	/*** runtime values (PRIVATE) ***/
	uint8_t queue_report_requested;	// set to true to request a report
	uint8_t buffers_available;		// stored buffer depth passed to by callback
	uint8_t prev_available;			// buffers available at last count
	uint16_t buffers_added;			// buffers added since last count
	uint16_t buffers_removed;		// buffers removed since last report
	uint8_t motion_mode;			// used to detect arc movement
	uint32_t init_tick;				// time when values were last initialized or cleared

} qrSingleton_t;

typedef struct rxSingleton {
    uint8_t rx_report_requested;
    uint16_t space_available;       // space available in usb rx buffer at time of request
} rxSingleton_t;

/**** Externs - See report.c for allocation ****/

extern srSingleton_t sr;
extern qrSingleton_t qr;
extern rxSingleton_t rx;

/**** Function Prototypes ****/

void rpt_print_message(char *msg);
stat_t rpt_exception(uint8_t status);

stat_t rpt_er(nvObj_t *nv);
void rpt_print_loading_configs_message(void);
void rpt_print_initializing_message(void);
void rpt_print_system_ready_message(void);

void sr_init_status_report(void);
stat_t sr_set_status_report(nvObj_t *nv);
stat_t sr_request_status_report(uint8_t request_type);
stat_t sr_status_report_callback(void);
stat_t sr_run_text_status_report(void);

stat_t sr_get(nvObj_t *nv);
stat_t sr_set(nvObj_t *nv);
stat_t sr_set_si(nvObj_t *nv);
//void sr_print_sr(nvObj_t *nv);

void qr_init_queue_report(void);
void qr_request_queue_report(int8_t buffers);
stat_t qr_queue_report_callback(void);

void rx_request_rx_report(void);
stat_t rx_report_callback(void);

stat_t qr_get(nvObj_t *nv);
stat_t qi_get(nvObj_t *nv);
stat_t qo_get(nvObj_t *nv);

#ifdef __TEXT_MODE

	void sr_print_sr(nvObj_t *nv);
	void sr_print_si(nvObj_t *nv);
	void sr_print_sv(nvObj_t *nv);
	void qr_print_qv(nvObj_t *nv);
	void qr_print_qr(nvObj_t *nv);
	void qr_print_qi(nvObj_t *nv);
	void qr_print_qo(nvObj_t *nv);

#else

	#define sr_print_sr tx_print_stub
	#define sr_print_si tx_print_stub
	#define sr_print_sv tx_print_stub
	#define qr_print_qv tx_print_stub
	#define qr_print_qr tx_print_stub
	#define qr_print_qi tx_print_stub
	#define qr_print_qo tx_print_stub

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif

#endif // End of include guard: REPORT_H_ONCE
