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
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>


#define PORT_LED GPIOC
#define PIN_LED GPIO6
static void gpio_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOC);
  
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
    gpio_set_output_options(PORT_LED, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PIN_LED);
    
}

#define I2C1_PINS GPIO6|GPIO7
static void i2c_init(void) {
    // Enable I2C peripheral clock
    rcc_periph_clock_enable(RCC_I2C1);
    
    // Enable GPIOB clock (assuming you are using I2C1 on pins PB8 (SCL) and PB9 (SDA))
    rcc_periph_clock_enable(RCC_GPIOB);

    // Configure PB8 (SCL) and PB9 (SDA) as alternate function (AF1 for I2C1)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C1_PINS);
    gpio_set_af(GPIOB, GPIO_AF6, I2C1_PINS);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2_PINS);

 	i2c_peripheral_disable(I2C1);

    
    // int clock_megahz = 64;
    // int prescaler = clock_megahz / 32 - 1;

    // i2c_set_prescaler(I2C1, prescaler);
    // i2c_set_scl_low_period (I2C1, 14);
    // i2c_set_scl_high_period(I2C1,  6);
    // i2c_set_data_hold_time (I2C1,  2); 
    // i2c_set_data_setup_time(I2C1,  2);
    i2c_disable_autoend(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, 64);

    
    //i2c_set_bytes_to_transfer(I2C1, 4);
    i2c_set_7bit_addr_mode(I2C1);
    i2c_peripheral_enable(I2C1);
    
}

#define TIM2_PINS GPIO3|GPIO5
void timer_setup(void) {
    
    //TIM2 provides a synthetic "ACK" to I2C1
   
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM2);
     
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM2_PINS);
    gpio_set_af(GPIOA, GPIO_AF2, TIM2_PINS); 
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, TIM2_PINS);
 
    timer_disable_counter(TIM2);
   
    // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_ECM1); 
    
    // TI1 Edge Detector (TI1F_ED)
    timer_slave_set_trigger(TIM2, TIM_SMCR_TS_ETRF); 
    timer_slave_set_polarity(TIM2, TIM_ET_FALLING);    
    //timer_slave_set_trigger(TIM2, TIM_SMCR_TS_TI1F_ED);// count every edge

    /* Set prescaler to 0 (count every pulse) */
    timer_set_prescaler(TIM2, 0);

    /* Set auto-reload register (counts up to 8) */
    timer_set_period(TIM2, 9);

    /* Configure Output Compare (OC) mode on CH1 */
    timer_set_oc_value(TIM2, TIM_OC4, 9);
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM2, TIM_OC4);

    /* Enable the timer */
    timer_enable_counter(TIM2);
}



int main(void)
{
	int i;

    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    i2c_init();
    timer_setup();
    
    gpio_setup();
    timer_set_counter(TIM2, 0);

	while (1) {

		//gpio_set(PORT_LED, PIN_LED);	
		for (i = 0; i < 10; i++)
			__asm__("nop");
        gpio_clear(PORT_LED, PIN_LED);	

        i2c_send_start(I2C1);

		for (i = 0; i < 20; i++)	
			__asm__("nop");

        // i2c_send_7bit_address(I2C1, address, I2C_WRITE);
        // while (!i2c_get_flag(I2C1, I2C_ISR_TCR));
        
        i2c_send_data(I2C1, 0xAA);
        
        for (i = 0; i < 20; i++)	
			__asm__("nop");
        
        i2c_send_data(I2C1, 0xFF);
        
        i2c_send_data(I2C1, 0x00);
        
        i2c_send_data(I2C1, 0x55);
        
        i2c_send_stop(I2C1);    
    
    	for (i = 0; i < 1000; i++)	
			__asm__("nop");       

            
        gpio_set(PORT_LED, PIN_LED);	
		for (i = 0; i < 10; i++)	
			__asm__("nop");
		//gpio_clear(PORT_LED, PIN_LED);	


	}

	return 0;
}
