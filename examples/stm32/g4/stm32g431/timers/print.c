
#include "print.h"


static uint16_t start_ndx;
static uint16_t end_ndx;
static char buf[BUFLEN+1];
#define buf_len ((end_ndx - start_ndx) % BUFLEN)
static inline int inc_ndx(int n) { return ((n + 1) % BUFLEN); }
static inline int dec_ndx(int n) { return (((n + BUFLEN) - 1) % BUFLEN); }




/* back up the cursor one space */
static inline void back_up(void)
{
	end_ndx = dec_ndx(end_ndx);
	usart_send_blocking(STDOUT_USART, '\010');
	usart_send_blocking(STDOUT_USART, ' ');
	usart_send_blocking(STDOUT_USART, '\010');
}

/*
 * A buffered line editing function.
 */
void get_buffered_line(void) {
	char	c;

	if (start_ndx != end_ndx) {
		return;
	}
	while (1) {
		c = usart_recv_blocking(STDOUT_USART);
		if (c == '\r') {
			buf[end_ndx] = '\n';
			end_ndx = inc_ndx(end_ndx);
			buf[end_ndx] = '\0';
			usart_send_blocking(STDOUT_USART, '\r');
			usart_send_blocking(STDOUT_USART, '\n');
			return;
		}
		/* ^H or DEL erase a character */
		if ((c == '\010') || (c == '\177')) {
			if (buf_len == 0) {
				usart_send_blocking(STDOUT_USART, '\a');
			} else {
				back_up();
			}
		/* ^W erases a word */
		} else if (c == 0x17) {
			while ((buf_len > 0) &&
					(!(isspace((int) buf[end_ndx])))) {
				back_up();
			}
		/* ^U erases the line */
		} else if (c == 0x15) {
			while (buf_len > 0) {
				back_up();
			}
		/* Non-editing character so insert it */
		} else {
			if (buf_len == (BUFLEN - 1)) {
				usart_send_blocking(STDOUT_USART, '\a');
			} else {
				buf[end_ndx] = c;
				end_ndx = inc_ndx(end_ndx);
				usart_send_blocking(STDOUT_USART, c);
			}
		}
	}
}

/*
 * Called by libc stdio fwrite functions
 */
int _write(int fd, char *ptr, int len)
{
	int i = 0;

	/*
	 * Write "len" of char from "ptr" to file id "fd"
	 * Return number of char written.
	 *
	 * Only work for STDOUT, STDIN, and STDERR
	 */
	if (fd > 2) {
		return -1;
	}
	while (*ptr && (i < len)) {
		usart_send_blocking(STDOUT_USART, *ptr);
		// if (*ptr == '\n') {
			// usart_send_blocking(STDOUT_USART, '\r');
		// }
		i++;
		ptr++;
	}
	return i;
}

/*
 * Called by the libc stdio fread fucntions
 *
 * Implements a buffered read with line editing.
 */
int _read(int fd, char *ptr, int len)
{
	int	my_len;

	if (fd > 2) {
		return -1;
	}

	get_buffered_line();
	my_len = 0;
	while ((buf_len > 0) && (len > 0)) {
		*ptr++ = buf[start_ndx];
		start_ndx = inc_ndx(start_ndx);
		my_len++;
		len--;
	}
	return my_len; /* return the length we got */
}


// redefine syscalls:
int _close(int file) {(void)file; return -1; }
int _fstat(int file, struct stat *st) {(void)file; (void)st; return 0; }
int _isatty(int file) {(void)file; return 1; }
int _lseek(int file, int ptr, int dir) {(void)file; (void)ptr; (void)dir; return 0; }
int _getpid(void) {return 1; }
int _kill(int pid, int sig) {(void)pid; (void)sig; return -1; }


void usart_setup(uint32_t dev)
{
    
    rcc_periph_clock_enable(RCC_GPIOB);
  	rcc_periph_clock_enable(RCC_USART3);
    
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_PIN);
    gpio_set_af(GPIOB, USART_AF, USART_TX_PIN);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, USART_TX_PIN);

    
	/* Setup USART parameters. */
	usart_set_baudrate(dev, 115200);
	usart_set_databits(dev, 8);
	usart_set_parity(dev, USART_PARITY_NONE);
	usart_set_stopbits(dev, USART_CR2_STOPBITS_1);
	usart_set_mode(dev, USART_MODE_TX_RX);
	usart_set_flow_control(dev, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(dev);

}
