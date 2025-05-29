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
//#include <libopencm3/cm3/scb.h>
//#include <libopencm3/cm3/vector.h>

//#define APP_BASE 0x08004000

static void gpio_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
}

int main(void)
{
    //reroute the interrupt vector
    //SCB_VTOR = APP_BASE;
    
	int i;

    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	gpio_setup();

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
