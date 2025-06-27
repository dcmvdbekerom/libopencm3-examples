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
#include <libopencm3/stm32/spi.h>

#define PORT_LED GPIOB
#define PIN_LED GPIO2
struct rcc_clock_scale uc_clock;


static void gpio_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}

#define SPI1_PINS GPIO4|GPIO5|GPIO6|GPIO7
static void spi_setup(void) {
    
    rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_SPI1);
    
 	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI1_PINS);
    gpio_set_af(GPIOA, GPIO_AF5, SPI1_PINS); 
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI1_PINS);

    
    
    
    spi_disable(SPI1);
    
    spi_init_master(SPI1, 
        SPI_CR1_BAUDRATE_FPCLK_DIV_8,  
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1,
        SPI_CR1_DFF_8BIT,
        SPI_CR1_MSBFIRST);
    
    spi_enable(SPI1);
    
}

#define TIM5_PINS GPIO0
static void timer_setup(void) {
    /* Reset TIM2 */
    
    //Setup TIM2 as an edge triggered clock
    // A2 (TIM2_CH3 = OUT)
    
    timer_disable_counter(TIM5);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM5);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM5_PINS);
    gpio_set_af(GPIOA, GPIO_AF2, TIM5_PINS); 
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM5_PINS);

    // // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    // timer_slave_set_mode(TIM2, TIM_SMCR_SMS_ECM1); 
    // // TI1 Edge Detector (TI1F_ED)
    // timer_slave_set_trigger(TIM2, TIM_SMCR_TS_ETRF);   
    timer_slave_set_mode(TIM5, TIM_SMCR_SMS_OFF);
    //timer_slave_set_trigger(TIM2, TIM2_CH3

    timer_set_prescaler(TIM5, uc_clock.apb1_frequency / 1000000 - 1);

    timer_set_period(TIM5, 1);

    /* Configure Output Compare (OC) mode on CH1 */
    
    timer_set_oc_value(TIM5, TIM_OC1, 1);
    timer_set_oc_mode(TIM5, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM5, TIM_OC1);

    /* Enable the timer */
    timer_enable_counter(TIM5);
}


uint16_t count;

int main(void)
{
    //reroute the interrupt vector
    //SCB_VTOR = APP_BASE;
    
	int i;
    uc_clock = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ];
    rcc_clock_setup_pll(&uc_clock);

	gpio_setup();
    timer_setup();
    spi_setup();
    
    count = 0;

	/* Blink the LED (PC8) on the board. */
	while (1) {
		// gpio_toggle(GPIOB, GPIO2);	/* LED on/off */
		// for (i = 0; i < 20000; i++) {	/* Wait a bit. */
			// __asm__("nop");
		// }

		// gpio_toggle(GPIOB, GPIO2);	/* LED on/off */
		// for (i = 0; i < 20000; i++) {	/* Wait a bit. */
			// __asm__("nop");
		// }
        
        gpio_toggle(GPIOB, GPIO2);	/* LED on/off */
        spi_write(SPI1, (uint8_t)(count>>6));
        count++;	
        
        for (i = 0; i < 200000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
        

	}

	return 0;
}
