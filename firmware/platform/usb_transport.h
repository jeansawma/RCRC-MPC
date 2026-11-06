#ifndef USB_TRANSPORT_H
#define USB_TRANSPORT_H
#include <stddef.h>
#include <stdint.h>
/* Nonblocking native USB CDC; 0 means not configured or buffer full. */
int usb_transport_init(void);
void usb_transport_poll(void);
int usb_transport_readline(char *dst, size_t capacity);
int usb_transport_write(const char *text, size_t length);
/* DTR asserted, configured, and USB SOF received within the last 10 ms. */
int usb_transport_connected(void);
/* Session epoch changes on reset, deinit, suspend, disconnect, DTR transition,
 * or loss of USB SOFs. Callers must require a fresh HELLO/ARM after a change. */
uint32_t usb_transport_session(void);
#endif
