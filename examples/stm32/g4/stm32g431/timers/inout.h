#ifndef INOUT_H_
#define INOUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/cm3/nvic.h>



//static void gpio_setup(void);
void spi_setup(void);
void led_setup(void);
void led_toggle(void);
void button_setup(void);
int button_pressed(void);
void button_isr(void);


#define LED_PORT GPIOC
#define LED_PIN GPIO6

#define BUTTON_PORT GPIOC
#define BUTTON_PIN GPIO13

#ifdef __cplusplus
}
#endif


#endif 