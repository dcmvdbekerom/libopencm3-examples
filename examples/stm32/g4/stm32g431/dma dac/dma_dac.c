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

#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>

#define PORT_LED GPIOC
#define PIN_LED GPIO6


#define TIM1_PORT GPIOA
#define TIM1_PIN GPIO8
#define TIM1_AF GPIO_AF6
#define TIM1_RATE 5000000




#define TIM3_PORT GPIOA
#define TIM3_PIN GPIO4
#define TIM3_AF GPIO_AF2 //CH2

#define N_SAMPLE 0x1000 //12 bit
uint16_t val = 0;
uint8_t  vala[N_SAMPLE];
uint16_t valb[N_SAMPLE];
uint8_t  valc[N_SAMPLE];

#define BITS(x,y) (((val & (1<<y))>>y) << x)

#define OUTA_PINS GPIO15|GPIO12|GPIO11|GPIO10|GPIO9 
#define OUTB_PINS GPIO9 |GPIO8 |GPIO7 |GPIO6 |GPIO5 |GPIO4 |GPIO3
#define OUTC_PINS GPIO11|GPIO10


// vala[i] = (BITS(15, 8)|BITS(12,11)|BITS(11,10)|BITS(10,13)|BITS( 9,12)) >> 8;
// valb[i] = (BITS( 9, 1)|BITS( 8, 0)|BITS( 7, 3)|BITS( 6, 2)|BITS( 5, 5)|BITS( 4, 4)|BITS( 3, 7));
// valc[i] = (BITS(11, 6)|BITS(10, 9)) >> 8;

static void timer_setup(void) {
    //Setup a timer TIM1
    
    timer_disable_counter(TIM1);
    timer_disable_counter(TIM3);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

  	rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM3);

    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_DMAMUX1);
  	
    
	gpio_mode_setup(TIM1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM1_PIN);
    gpio_set_af(TIM1_PORT, TIM1_AF, TIM1_PIN); //TIM1_CH1 at PA8 (AF6)
    gpio_set_output_options(TIM1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM1_PIN);
    
    gpio_mode_setup(TIM3_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM3_PIN);
    gpio_set_af(TIM3_PORT, TIM3_AF, TIM3_PIN); //TIM3_CH2 at PA4 (AF2)
    gpio_set_output_options(TIM3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM3_PIN);

    // gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    // gpio_set_af(GPIOA, GPIO_AF0, GPIO8); //TIM3_CH2 at PA4 (AF2)
    // gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO8);

    // DAC signals:
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUTA_PINS);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, OUTA_PINS);

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUTB_PINS);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, OUTB_PINS);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUTC_PINS);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, OUTC_PINS);



    timer_set_master_mode(TIM1, TIM_CR2_MMS_UPDATE);
    //timer_slave_set_mode(TIM2, TIM_SMCR_SMS_OFF); //default
    timer_set_prescaler(TIM1,  rcc_apb1_frequency/TIM1_RATE/2 - 1);
    //timer_set_prescaler(TIM1,  3);
    timer_set_period(TIM1, 1);
    
    
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_ECM1); //count on external trigger
    timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR0); // tim_itr0 = tim1_trgo
    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE); //forward TIM3's update
    timer_set_period(TIM3, N_SAMPLE - 1);

    // timer_slave_set_mode(TIM3, TIM_SMCR_SMS_ECM1); //count on external trigger
    // timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR1); // tim_itr3 = tim4_trgo
    // timer_slave_set_prescaler(TIM3,  TIM_IC_PSC_8); //divide the TIM4 signal by 8
    // timer_set_period(TIM3, 0x10);
    
    //enable trigger interrupt
    //timer_enable_irq(TIM3, TIM_DIER_TIE);
    //nvic_enable_irq(NVIC_TIM3_IRQ);
    
    timer_enable_irq(TIM1, TIM_DIER_UDE); //enable DMA request on trigger    
    
    /* Enable DMA Ch1 to transfer data to GPIOA_ODR*/
    dma_disable_channel(DMA1, DMA_CHANNEL1);
    
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&GPIOA_ODR+1); // set peripheral address
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_8BIT );
    
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&vala); // set memory address (high byte)
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_8BIT );
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);	
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, N_SAMPLE);// configure number of data to transfer
    dma_set_read_from_memory(DMA1, DMA_CHANNEL1);
    
    dmamux_set_dma_channel_request(DMAMUX1, DMA_CHANNEL1, DMAMUX_CxCR_DMAREQ_ID_TIM1_UP);	
    dma_enable_channel(DMA1, DMA_CHANNEL1); // activate channel

    
    /* Enable DMA Ch2 to transfer data to GPIOB_ODR*/
    dma_disable_channel(DMA1, DMA_CHANNEL2);
    
    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&GPIOB_ODR); // set peripheral address
    dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_16BIT );
    
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)&valb); // set memory address (high byte)
    dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_16BIT );
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
    
    dma_enable_circular_mode(DMA1, DMA_CHANNEL2);	
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, N_SAMPLE);// configure number of data to transfer
    dma_set_read_from_memory(DMA1, DMA_CHANNEL2);
    
    dmamux_set_dma_channel_request(DMAMUX1, DMA_CHANNEL2, DMAMUX_CxCR_DMAREQ_ID_TIM1_UP);	
    dma_enable_channel(DMA1, DMA_CHANNEL2); // activate channel

    /* Enable DMA Ch3 to transfer data to GPIOC_ODR*/
    dma_disable_channel(DMA1, DMA_CHANNEL3);
    
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&GPIOC_ODR+1); // set peripheral address
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT );
    
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)&valc); // set memory address (high byte)
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT );
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
    
    dma_enable_circular_mode(DMA1, DMA_CHANNEL3);	
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, N_SAMPLE);// configure number of data to transfer
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
    
    dmamux_set_dma_channel_request(DMAMUX1, DMA_CHANNEL3, DMAMUX_CxCR_DMAREQ_ID_TIM1_UP);	
    dma_enable_channel(DMA1, DMA_CHANNEL3); // activate channel



    /* Configure Output Compare (OC) mode on CH4 */
    timer_set_oc_value(TIM1, TIM_OC1, 1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2); //center strobe on rising edge(*)
    timer_enable_oc_output(TIM1, TIM_OC1);

    timer_set_oc_value(TIM3, TIM_OC2, N_SAMPLE-1);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM2); //center strobe on rising edge(*)
    timer_enable_oc_output(TIM3, TIM_OC2);


    timer_enable_counter(TIM1);
    timer_enable_counter(TIM3);
    
    timer_enable_break_main_output(TIM1);
    // timer_enable_counter(TIM3);

}


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




int main(void)
{
	int i;

    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_170MHZ]); //24, 48, 96, 170
    for(i=0; i<N_SAMPLE; i+=1){
        val += 4;
        vala[i] = (BITS(15, 8)|BITS(12,11)|BITS(11,10)|BITS(10,13)|BITS( 9,12)) >> 8;
        valb[i] = (BITS( 9, 1)|BITS( 8, 0)|BITS( 7, 3)|BITS( 6, 2)|BITS( 5, 5)|BITS( 4, 4)|BITS( 3, 7));
        valc[i] = (BITS(11, 6)|BITS(10, 9)) >> 8;
    }

	gpio_setup();
    timer_setup();

	/* Blink the LED (PC8) on the board. */
	while (1) {
        //GPIOB_ODR = 0x00;
		gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 500000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}		
        gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 500000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}		
        gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 500000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
        //GPIOB_ODR = 0xFF;
		gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 3000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}

	}
        
        

	return 0;
}
