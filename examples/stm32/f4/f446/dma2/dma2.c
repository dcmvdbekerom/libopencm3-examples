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
#include <libopencm3/stm32/dma.h>


#define PORT_LED GPIOB
#define PIN_LED GPIO2
struct rcc_clock_scale uc_clock;


static void gpio_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}



#define SOFT_I2C_PINS GPIO0|GPIO1|GPIO2|GPIO3
#define SOFT_I2C_PORT GPIOC

#define CMD_SIZE 6

uint32_t bsrr_buf[CMD_SIZE];
uint32_t ccr_buf[CMD_SIZE];

static void dma_setup(uint32_t dma, uint8_t stream){
    
    rcc_periph_clock_enable(RCC_GPIOA); //TODO: not variable
    rcc_periph_clock_enable(RCC_DMA2); //TODO: not variable

    dma_disable_stream(dma, stream);
    //dma_stream_reset(dma, stream);
    //while (dma_get_stream_en(dma, stream)) {};
    
    dma_channel_select(dma, stream, DMA_SxCR_CHSEL_6); //channel 6
    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
      
    //dma_set_peripheral_address(dma, stream, GPIOC_BSRR);
    dma_set_peripheral_address(dma, stream, (uint32_t) &TIM1_CCR3);
    dma_set_peripheral_size(dma, stream, DMA_SxCR_PSIZE_32BIT);    
    
    //dma_set_memory_address(dma, stream, (uint32_t) bsrr_buf);
    dma_set_memory_address(dma, stream, (uint32_t) ccr_buf);
    dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_32BIT);
    
    dma_enable_memory_increment_mode(dma, stream);
    dma_set_number_of_data(dma, stream, CMD_SIZE);
    dma_enable_circular_mode(dma, stream);
    
    dma_enable_stream(dma, stream);
    
    // gpio_mode_setup(SOFT_I2C_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SOFT_I2C_PINS);
    // gpio_set_output_options(SOFT_I2C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SOFT_I2C_PINS);


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

#define TIM_PINS GPIO8|GPIO10
#define PERIOD 8
static void timer_setup(uint32_t timer) {
    /* Reset TIM2 */
    
    //Setup TIM2 as an edge triggered clock
    // A2 (TIM2_CH3 = OUT)
    
    timer_disable_counter(timer);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM1);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM_PINS);
    gpio_set_af(GPIOA, GPIO_AF1, TIM_PINS); 
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TIM_PINS);

    // // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    // timer_slave_set_mode(TIM2, TIM_SMCR_SMS_ECM1); 
    // // TI1 Edge Detector (TI1F_ED)
    // timer_slave_set_trigger(TIM2, TIM_SMCR_TS_ETRF);   
    //timer_slave_set_mode(TIM5, TIM_SMCR_SMS_OFF);
    //timer_slave_set_trigger(TIM2, TIM2_CH3

    timer_set_prescaler(timer, 2 * uc_clock.apb2_frequency / 1000000 / PERIOD - 1);

    timer_set_period(timer, PERIOD - 1);

    /* Configure Output Compare (OC) mode on CH1 */
    
    timer_set_oc_value(timer, TIM_OC1, PERIOD/2 );
    timer_set_oc_mode(timer, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(timer, TIM_OC1);

    timer_set_oc_value(timer, TIM_OC2, PERIOD/4 );
    timer_set_oc_mode(timer, TIM_OC2, TIM_OCM_ACTIVE);
    
    timer_set_oc_mode(timer, TIM_OC3, TIM_OCM_PWM2);  
    TIM1_CCMR2 &= ~TIM_CCMR2_OC3PE;
    timer_enable_oc_output(timer, TIM_OC3);


    TIM1_CR1 &= ~TIM_CR1_URS;
    timer_enable_update_event(timer); //TIM1_CR1 &= ~TIM_CR1_UDIS;
    //TIM1_DIER |= TIM_DIER_UDE;
    TIM1_DIER |= TIM_DIER_CC2DE;
    
    
    // Channel 6
    //        | S0 | S1 | S2 | S3 | S4 | S5 | S6 | S7 |
    // TIM1_: |TRIG|CH1 |CH2 |CH1 |CH4 | UP |CH3 |    |
    //                             TRIG
    //                             COM
    
    
    // timer_set_dma_on_update_event(timer);


    /* Enable the timer */
    timer_enable_break_main_output(timer);
    //TIM1_BDTR |= TIM_BDTR_MOE;
    timer_enable_counter(timer);
}


uint16_t count;

uint8_t cmd_buf[CMD_SIZE] = {0x09,
                             0x0A,
                             0x06,
                             0x07,
                             0x0F,
                             0x09};




int main(void)
{
    //reroute the interrupt vector
    //SCB_VTOR = APP_BASE;
    
	int i;
    uc_clock = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ];
    rcc_clock_setup_pll(&uc_clock);
    
    for (i=0; i<CMD_SIZE; i++){
        bsrr_buf[i] = ((~cmd_buf[i]) << 16) | cmd_buf[i];
        ccr_buf[i] = (cmd_buf[i]&0x1) * PERIOD;
    }

	gpio_setup();
    timer_setup(TIM1);
    spi_setup();
    dma_setup(DMA2, DMA_STREAM2);
    //GPIOC_ODR |= 0x00000001;
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
