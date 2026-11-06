#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>
#include <stddef.h>
/* Platform owns timing/peripherals. App runs once for each fresh ADC sample. */
void board_init(void);
void board_poll(void);
int board_sample_ready(float *v1, float *v2, float *vbus);
void board_set_output(float duty, int enable, int finite_set);
int board_set_sample_us(uint32_t us);
uint32_t board_millis(void);
uint32_t board_micros(void);
int board_usb_readline(char *dst, size_t capacity);
int board_usb_write(const char *text, size_t length);
/* Buttons bits UP=1 DOWN=2 LEFT=4 RIGHT=8, logical pressed. */
uint8_t board_buttons(void);
void board_oled_lines(const char *const lines[8]);
/* Returns latched platform faults (ADC timeout, timing overrun, etc.). */
uint32_t board_faults(void);
int board_usb_connected(void);
uint32_t board_usb_session(void);
/* Only recoverable faults clear, and only when the bridge is disabled. */
void board_clear_faults(void);
#endif
