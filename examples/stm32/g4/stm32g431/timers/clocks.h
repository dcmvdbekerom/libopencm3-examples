#ifndef CLOCKS_H_
#define CLOCKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
// extern uint16_t my_buf; 
// extern uint16_t my_buf2;

void clock_setup(void);
void timer_disable_strobe(void);
void timer_enable_strobe(void);
void timer_setup(void);
void timer_isr(void);

#define TIM2_PIN GPIO10
#define TIM2_AF GPIO_AF10
#define TIM2_RATE 16000000

#define TIM4_PIN GPIO12
#define TIM4_AF GPIO_AF10
#define N_CHANNEL 8

#ifdef __cplusplus
}
#endif

#endif