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
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#define PORT_LED GPIOC
#define PIN_LED GPIO6

#define SPI1_PINS GPIO4|GPIO5|GPIO6|GPIO7

static void gpio_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOA);
  
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PIN_LED);
    
}

static void spi1_setup(void) {
    // Enable clocks for GPIOA and SPI1
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);

    // Configure PA4 (NSS), PA5 (SCK), PA7 (MOSI) as alternate function
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI1_PINS);
    gpio_set_af(GPIOA, GPIO_AF0, SPI1_PINS);  // AF0 for SPI1
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SPI1_PINS);

    // Configure SPI1 in master mode
    // spi_reset(SPI1);
    spi_init_master(SPI1,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_2,  // Adjust speed as needed
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_LSBFIRST);
    
    SPI1_CR2 & ~SPI_CR1_SSM;    
    SPI1_CR2 |= SPI_CR2_NSSP;
    
    
    
    // spi_enable_software_slave_management(SPI1);
    // spi_set_nss_high(SPI1);
    spi_enable(SPI1);
}


int main(void)
{
	int i;

    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    spi1_setup();
    
    
    // gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4);
    // gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO4);
    
    //gpio_setup();
    
    
	/* Blink the LED (PC8) on the board. */
	while (1) {

		//gpio_set(PORT_LED, PIN_LED);	
		for (i = 0; i < 10; i++)
			__asm__("nop");
        gpio_clear(PORT_LED, PIN_LED);	



        
    	for (i = 0; i < 1000; i++)	
			__asm__("nop");       

            
        gpio_set(PORT_LED, PIN_LED);	
		for (i = 0; i < 10; i++)	
			__asm__("nop");
		//gpio_clear(PORT_LED, PIN_LED);	


	}

	return 0;
}
