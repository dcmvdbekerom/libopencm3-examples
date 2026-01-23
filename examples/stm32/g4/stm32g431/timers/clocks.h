#ifndef CLOCKS_H_
#define CLOCKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>



void clock_setup(void);
void timer_disable_strobe(void);
void timer_enable_strobe(void);
void timer_setup(void);


#define TIM2_PIN GPIO5
#define TIM2_AF GPIO_AF1
#define TIM2_RATE 16000000

#define TIM3_PIN GPIO6
#define TIM3_AF GPIO_AF2
#define N_CHANNEL 8

#ifdef __cplusplus
}
#endif

#endif