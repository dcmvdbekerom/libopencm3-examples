/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
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

#define PORT_LED GPIOC
#define PIN_LED GPIO6

static void gpio_setup(void)
{
	/* Enable GPIOC clock. */
	/* Manually: */
	//RCC_AHBENR |= RCC_AHBENR_GPIOCEN;
	/* Using API functions: */
	rcc_periph_clock_enable(RCC_GPIOC);


	/* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
	/* Using API functions: */
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}

#define TIM2_PINS GPIO3|GPIO5
void timer_setup(void) {
    /* Reset TIM2 */
    timer_disable_counter(TIM2);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM2);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM2_PINS);
    gpio_set_af(GPIOA, GPIO_AF2, TIM2_PINS); 
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2_PINS);

    // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_ECM1); 
    
    // TI1 Edge Detector (TI1F_ED)
    timer_slave_set_trigger(TIM2, TIM_SMCR_TS_ETRF);   
    //timer_slave_set_trigger(TIM2, TIM_SMCR_TS_TI1F_ED);// Use TI1 (PA5) as clock 

    /* Set prescaler to 0 (count every pulse) */
    timer_set_prescaler(TIM2, 0);

    /* Set auto-reload register (counts up to 8) */
    timer_set_period(TIM2, 7);

    /* Configure Output Compare (OC) mode on CH1 */
    
    timer_set_oc_value(TIM2, TIM_OC4, 1);
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC4);

    /* Enable the timer */
    timer_enable_counter(TIM2);
}


// TIM2, 3, 4, 14, 15, 16, 17
int main(void)
{
	int i;
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

	gpio_setup();
    timer_setup();

	/* Blink the LED (PC8) on the board. */
	while (1) {

		/* Using API functions gpio_set()/gpio_clear(): */
		gpio_set(PORT_LED, PIN_LED);		/* LED off */
		for (i = 0; i < 6400; i++)	/* Wait a bit. */
			__asm__("nop");
            
		gpio_clear(PORT_LED, PIN_LED);	/* LED on */
		for (i = 0; i < 6400; i++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
