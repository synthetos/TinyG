#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "../tinyg.h"
#include "../config.h"
#include "../planner.h"
#include "../switch.h"
#include "../stepper.h"
#include "../xio.h"

#include "sim.h"

static FILE* in;

void sim_init(int argc, char** argv) {
  if (argc == 1) {
    in = stdin;
    return;
  }
  if (argc > 2) {
    fprintf(stderr, "Too many arguments. Usage: ./tinyg.elf [in_file]\n");
    _exit(1);
  }
  in = fopen(argv[1], "r");
  if (in == NULL) {
    fprintf(stderr, "Failed to open input file %s: %s\n", argv[1], strerror(errno));
    _exit(1);
  }
}

#define XIO_OK 0
#define XIO_ERR 1

void xio_init(void) {}

FILE *xio_open(uint8_t dev, const char *addr, flags_t flags) { return NULL; }

int xio_ctrl(const uint8_t dev, const flags_t flags) { return 0; }

void cli(void) {}

int xio_set_baud(const uint8_t dev, const uint8_t baud_rate) { return 0; }

uint8_t xio_test_assertions() { return (STAT_OK); }

buffer_t xio_get_tx_bufcount_usart(const xioUsart_t *dx) {
  return 0;
}

int xio_gets(const uint8_t dev, char *buf, const int size) {
  static int i = 0;
  i++;
  if (i % 10 != 0) {
      return XIO_EAGAIN;
  }
  if (NULL == fgets(buf, size, in)) {
    if (errno == 0) {
      // EOF on stdin. The test is done, quit successfully.
      _exit(0);
    }
    perror("fgets");
    return XIO_ERR;
  }
  fprintf(stderr, "\n%s", buf);
  return XIO_OK;
}

// Stepper
stConfig_t st_cfg;
stPrepSingleton_t st_pre;

void stepper_init() {}
void st_deenergize_motors() {}
stat_t stepper_test_assertions() { return (STAT_OK); }

void st_print_ma(cmdObj_t *cmd) { }
void st_print_mi(cmdObj_t *cmd) { }
void st_print_mt(cmdObj_t *cmd) { }
void st_print_pl(cmdObj_t *cmd) { }
void st_print_pm(cmdObj_t *cmd) { }
void st_print_po(cmdObj_t *cmd) { }
void st_print_sa(cmdObj_t *cmd) { }
void st_print_tr(cmdObj_t *cmd) { }


stat_t st_set_md(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_me(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_mi(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_mt(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_pl(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_pm(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_sa(cmdObj_t *cmd) { return (STAT_OK); }
stat_t st_set_tr(cmdObj_t *cmd) { return (STAT_OK); }

void st_prep_null() {}
void st_prep_dwell(float microseconds) {}

stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time) {
  return (STAT_OK);
}

uint8_t stepper_isbusy() { return false; }
stat_t st_motor_power_callback() { return (STAT_OK); }

void st_request_exec_move() {
  if (mp_free_run_buffer()) cm_cycle_end();                       // free buffer & perform cycle_end if planner is empty
}


// Switch
void switch_init() {}
uint8_t get_limit_switch_thrown(void) { return false; }
uint8_t read_switch(uint8_t sw_num) { return SW_OPEN; }
uint8_t get_switch_mode(uint8_t sw_num) { return SW_MODE_HOMING_LIMIT; }
void sw_print_st(cmdObj_t *cmd) { }
stat_t sw_set_sw(cmdObj_t *cmd) { return (STAT_OK); }
stat_t sw_set_st(cmdObj_t *cmd) { return (STAT_OK); }

// Network
void network_init() {}

// Persistence
void persistence_init() {}

stat_t read_persistent_value(cmdObj_t *cmd) {
	// TODO: implement reading persistent values.
	cmd->value = 0;
	return (STAT_OK);
}

stat_t write_persistent_value(cmdObj_t *cmd) {
	// TODO: implement writing persistent values.
	return (STAT_OK);
}

// Util
uint32_t SysTickTimer_getValue() {
  static uint64_t start = 0;
  struct timeval tv;
  if (0 != gettimeofday(&tv, NULL)) {
    perror("gettimeofday");
    _exit(1);
  }
  uint64_t cur = tv.tv_sec * 1000 + (tv.tv_usec / 1000);
  if (start == 0) {
    start = cur;
  }
  cur -= start;

  // Now, it fits to 32 bits.
  return (uint32_t)cur;
}
