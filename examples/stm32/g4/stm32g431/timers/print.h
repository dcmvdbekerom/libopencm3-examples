#ifndef PRINT_H_
#define PRINT_H_

//#define _GNU_SOURCE

#include <ctype.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// #include <errno.h>
// #include <stddef.h>
// #include <sys/types.h>

static uint16_t start_ndx;
static uint16_t end_ndx;
static char buf[BUFLEN+1];
#define buf_len ((end_ndx - start_ndx) % BUFLEN)
static inline int inc_ndx(int n) { return ((n + 1) % BUFLEN); }
static inline int dec_ndx(int n) { return (((n + BUFLEN) - 1) % BUFLEN); }


int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);

void get_buffered_line(void);
static void usart_setup(uint32_t dev);



#define BUFLEN 127

#define STDOUT_USART USART3
#define USART_TX_PIN GPIO9
#define USART_AF GPIO_AF7


#endif