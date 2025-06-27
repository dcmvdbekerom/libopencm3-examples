/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (c) 2015 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define PORT_LED GPIOB
#define PIN_LED GPIO2
struct rcc_clock_scale uc_clock;


static void gpio_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}


#define TIM2_PINS GPIO2
static void timer_setup(void) {
    /* Reset TIM2 */
    
    //Setup TIM2 as an edge triggered clock
    // A2 (TIM2_CH3 = OUT)
    
    timer_disable_counter(TIM2);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM2);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM2_PINS);
    gpio_set_af(GPIOA, GPIO_AF1, TIM2_PINS); 
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2_PINS);

    // // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    // timer_slave_set_mode(TIM2, TIM_SMCR_SMS_ECM1); 
    // // TI1 Edge Detector (TI1F_ED)
    // timer_slave_set_trigger(TIM2, TIM_SMCR_TS_ETRF);   
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_OFF);
    //timer_slave_set_trigger(TIM2, TIM2_CH3

    timer_set_prescaler(TIM2, uc_clock.apb1_frequency / 1000000 - 1);

    timer_set_period(TIM2, 1);

    /* Configure Output Compare (OC) mode on CH1 */
    
    timer_set_oc_value(TIM2, TIM_OC3, 1);
    timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC3);

    /* Enable the timer */
    timer_enable_counter(TIM2);
}




int main(void)
{
    //reroute the interrupt vector
    //SCB_VTOR = APP_BASE;
    
	int i;
    uc_clock = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ];
    rcc_clock_setup_pll(&uc_clock);

	gpio_setup();
    timer_setup();

	/* Blink the LED (PC8) on the board. */
	while (1) {
		gpio_toggle(GPIOB, GPIO2);	/* LED on/off */
		for (i = 0; i < 2000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}

		gpio_toggle(GPIOB, GPIO2);	/* LED on/off */
		for (i = 0; i < 2000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
        
        		gpio_toggle(GPIOB, GPIO2);	/* LED on/off */
		for (i = 0; i < 20000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}

	}

	return 0;
}
