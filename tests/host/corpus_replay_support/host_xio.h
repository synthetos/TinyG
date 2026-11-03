#include "tinyg.h"
#define XIO_DEV_USB 0
#define ASCII_LF 10
#define ASCII_CR 13
bool xio_isbusy(void);
void xio_flush_read(void);
void xio_reset_usb_rx_buffers(void);
void xio_drop_usb_rx_before_mark(void);
void xio_flush_readline(void);
void xio_flush_rx(void);
#define NUL 0
void xio_flush_input(void);

bool xio_rx_data_pending(void);
bool xio_usb_is_quiet(void);
bool xio_usb_tx_ready(void);
bool xio_usb_try_send_frame(const char *s);
bool xio_usb_try_send_line(const char *s);
