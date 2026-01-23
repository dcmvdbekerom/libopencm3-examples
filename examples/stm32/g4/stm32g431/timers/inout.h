#ifndef INOUT_H_
#define INOUT_H_

#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/cm3/nvic.h>

//static void gpio_setup(void);
static void led_setup(void);
static void led_toggle(void);
static void button_setup(void);
void button_isr(void);


#define LED_PORT GPIOC
#define LED_PIN GPIO6

#define BUTTON_PORT GPIOC
#define BUTTON_PIN GPIO13




#endif 