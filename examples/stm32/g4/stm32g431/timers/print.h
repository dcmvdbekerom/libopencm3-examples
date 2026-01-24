#ifndef PRINT_H_
#define PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define _GNU_SOURCE

#include <ctype.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// #include <errno.h>
// #include <stddef.h>
// #include <sys/types.h>

#define BUFLEN 127



int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);

//redefine syscalls
#include <sys/stat.h>
int _close(int);
int _fstat(int, struct stat *);
int _isatty(int);
int _lseek(int, int, int);
int _getpid(void);
int _kill(int, int);

void get_buffered_line(void);
void usart_setup(uint32_t dev);




#define STDOUT_USART USART3
#define USART_TX_PIN GPIO9
#define USART_AF GPIO_AF7

#ifdef __cplusplus
}
#endif

#endif