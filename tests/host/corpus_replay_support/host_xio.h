#include "tinyg.h"
#define XIO_DEV_USB 0
#define ASCII_LF 10
#define ASCII_CR 13
bool xio_isbusy(void);
static inline uint8_t xio_usb_take_rx_seen(void) { return 0; }
static int USBu;
static inline uint16_t xio_get_rx_bufcount_usart(const int *d) { return 0; }
void xio_flush_read(void);
void xio_reset_usb_rx_buffers(void);
void xio_drop_usb_rx_before_mark(void);
void xio_flush_readline(void);
void xio_flush_rx(void);
#define NUL 0
void xio_cancel_pgm_source(void);
void xio_flush_input(void);

bool xio_rx_data_pending(void);
bool xio_usb_is_quiet(void);
uint8_t xio_usb_rx_unread(void);
bool xio_usb_tx_ready(void);
bool xio_usb_try_send_frame(const char *s);
bool xio_usb_try_send_line(const char *s);
