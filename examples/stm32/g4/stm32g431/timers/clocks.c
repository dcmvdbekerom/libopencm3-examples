
#include "clocks.h"

void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_96MHZ]); //24, 48, 96, 170
}


void timer_setup(void) {
    /* Reset TIM2 */
    
    //Setup a timer chain TIM2 -> TIM4 -> TIM3
    // TIM2 updates  at  16 MHz 
    // TIM4 updates  at   1 MHz
    // TIM3 triggers at 125 kHz; every trigger, DMA transfers count register to the USART3_TX.
    
    timer_disable_counter(TIM2);
    timer_disable_counter(TIM4);
    timer_disable_counter(TIM3);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM4);
  	rcc_periph_clock_enable(RCC_TIM3);
  	


	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM2_PIN|TIM4_PIN);
    gpio_set_af(GPIOA, TIM2_AF, TIM2_PIN); //TIM2_CH4 at PA10 (AF10)
    gpio_set_af(GPIOA, TIM4_AF, TIM4_PIN); //TIM4_CH2 at PA12 (AF10)
    //gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM2_PIN|TIM3_PIN);
    // set OSPEED to different values results in a better aligned phase (for some reason).
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM2_PIN);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM4_PIN);


    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    //timer_slave_set_mode(TIM2, TIM_SMCR_SMS_OFF); //default
    timer_set_prescaler(TIM2,  rcc_apb1_frequency/TIM2_RATE/2 - 1);
    timer_set_period(TIM2, 1);
    
    timer_slave_set_mode(TIM4, TIM_SMCR_SMS_ECM1); //count on external trigger
    timer_slave_set_trigger(TIM4, TIM_SMCR_TS_ITR1); // tim_itr1 = tim2_trgo
    timer_set_master_mode(TIM4, TIM_CR2_MMS_UPDATE); //forward TIM4's update

    timer_set_period(TIM4, N_CHANNEL - 1);

    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_ECM1); //count on external trigger
    timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR3); // tim_itr3 = tim4_trgo
    timer_slave_set_prescaler(TIM3,  TIM_IC_PSC_8); //divide the TIM4 signal by 8
    timer_set_period(TIM3, 0xFFF - 1);
    
    /* Enable DMA to transfer data at TIM3_CNT*/
    // TIM_DIER(TIM3) |= TIM_DIER_TDE; //enable DMA request on trigger
    
    // #define DMA_CH DMA_CHANNEL1
    // dma_disable_channel(DMA1, DMA_CH);
    // dma_set_peripheral_address(DMA1, DMA_CH, TIM3_CNT); // set peripheral address
    // dma_set_memory_address(DMA1, DMA_CH, USART3_TX); // set memory address
    // dma_set_number_of_data(DMA1, DMA_CH, 1);// configure number of data to transfer
    // // configure other parameters:
    // dma_set_peripheral_size(DMA1, DMA_CH, DMA_CCR_PSIZE_16BIT );
    // dma_set_memory_size(DMA1, DMA_CH, DMA_CCR_MSIZE_16BIT );
    // dma_set_read_from_peripheral();
    
    // dmamux_set_dma_channel_request(DMAMUX1, DMA_CH, DMAMUX_CxCR_DMAREQ_ID_TIM3_TRIG);	
    
    // dma_enable_channel(DMA1, DMA_CH); // activate channel


    /* Configure Output Compare (OC) mode on CH4 */
    timer_set_oc_value(TIM2, TIM_OC4, 1);
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1); //center strobe on rising edge(*)
    //timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM2); //center strobe on falling edge(*)
    //(*) depends on prescaler setting whether it is falling or rising
    timer_enable_oc_output(TIM2, TIM_OC4);

    timer_set_oc_value(TIM4, TIM_OC2, 1);
    timer_disable_strobe();
    timer_enable_oc_output(TIM4, TIM_OC2);

    timer_enable_counter(TIM2);
    timer_enable_counter(TIM4);
    timer_enable_counter(TIM3);

}


void timer_disable_strobe(void)
{
    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_FORCE_LOW);    
}


void timer_enable_strobe(void)
{
    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM1);
}
