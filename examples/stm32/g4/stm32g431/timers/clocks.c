
#include "clocks.h"

void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_96MHZ]); //24, 48, 96, 170
}


void timer_setup(void) {
    /* Reset TIM2 */
    
    //Setup TIM2 as an edge triggered clock
    // A5 (TIM2_CH1 = OUT)
    
    timer_disable_counter(TIM2);
    timer_disable_counter(TIM3);
    
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_TIM2);
  	rcc_periph_clock_enable(RCC_TIM3);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TIM2_PIN|TIM3_PIN);
    gpio_set_af(GPIOA, TIM2_AF, TIM2_PIN); //TIM2_CH1 at PA5
    gpio_set_af(GPIOA, TIM3_AF, TIM3_PIN); //TIM3_CH1 at PA6   
    //gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM2_PIN|TIM3_PIN);
    // set OSPEED to different values results in a better aligned phase (for some reason).
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM2_PIN);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, TIM3_PIN);


    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    //timer_slave_set_mode(TIM2, TIM_SMCR_SMS_OFF); //default
    timer_set_prescaler(TIM2,  rcc_apb1_frequency/TIM2_RATE/2 - 1);
    //timer_set_prescaler(TIM2,  rcc_apb1_frequency/1000000/2 - 1); //1 MHz
    timer_set_period(TIM2, 1);
    
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_ECM1); //count on external trigger
    timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR1); // tim_itr1 = tim2_trgo
    //timer_slave_set_prescaler(TIM3,  0); //default
    timer_set_period(TIM3, N_CHANNEL - 1);

    /* Configure Output Compare (OC) mode on CH1 */
    timer_set_oc_value(TIM2, TIM_OC1, 1);
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1); //center strobe on rising edge(*)
    //timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM2); //center strobe on falling edge(*)
    //(*) depends on prescaler setting whether it is falling or rising
    timer_enable_oc_output(TIM2, TIM_OC1);

    timer_set_oc_value(TIM3, TIM_OC1, 1);
    //timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FORCE_LOW); // start-up with strobe disabled
    timer_disable_strobe();
    timer_enable_oc_output(TIM3, TIM_OC1);

    timer_enable_counter(TIM2);
    timer_enable_counter(TIM3);

}


void timer_disable_strobe(void)
{
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FORCE_LOW);    
}


void timer_enable_strobe(void)
{
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
}
