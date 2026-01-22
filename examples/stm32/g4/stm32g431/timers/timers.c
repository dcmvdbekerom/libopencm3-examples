
//#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h> //atoi()
// #include <errno.h>
// #include <stddef.h>
// #include <sys/types.h>


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/usart.h>
#include "stm32-stdio.h"


// FILE *fp;



//static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
//static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

// static ssize_t _iord(void *_cookie, char *_buf, size_t _n)
// {
	// /* dont support reading now */
	// (void)_cookie;
	// (void)_buf;
	// (void)_n;
	// return 0;
// }

// static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n)
// {
	// uint32_t dev = (uint32_t)_cookie;

	// int written = 0;
	// while (_n-- > 0) {
		// usart_send_blocking(dev, *_buf++);
		// written++;
	// };
	// return written;
// }

#define USART3_TX_PIN GPIO9
#define USART3_AF GPIO_AF7
static void usart_setup(uint32_t dev)
{
    
    rcc_periph_clock_enable(RCC_GPIOB);
  	rcc_periph_clock_enable(RCC_USART3);
    
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, USART3_TX_PIN);
    gpio_set_af(GPIOB, USART3_AF, USART3_TX_PIN);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, USART3_TX_PIN);

    
	/* Setup USART parameters. */
	usart_set_baudrate(dev, 115200);
	usart_set_databits(dev, 8);
	usart_set_parity(dev, USART_PARITY_NONE);
	usart_set_stopbits(dev, USART_CR2_STOPBITS_1);
	usart_set_mode(dev, USART_MODE_TX_RX);
	usart_set_flow_control(dev, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(dev);

	// cookie_io_functions_t stub = { _iord, _iowr, NULL, NULL };
	// FILE *fpl = fopencookie((void *)dev, "rw+", stub);
	// /* Do not buffer the serial line */
	// setvbuf(fpl, NULL, _IONBF, 0);
	// return fpl;

}



#define PORT_LED GPIOC
#define PIN_LED GPIO6

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_96MHZ]); //24, 48, 96, 170
}



static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}



#define TIM2_PIN GPIO5
#define TIM2_AF GPIO_AF1
#define TIM2_RATE 16000000

#define TIM3_PIN GPIO6
#define TIM3_AF GPIO_AF2
#define N_CHANNEL 8

static void timer_disable_strobe(void)
{
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FORCE_LOW);    
}

static void timer_enable_strobe(void)
{
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
}

static void timer_setup(void) {
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

static void button_setup(void){
    rcc_periph_clock_enable(RCC_GPIOC);     // button port
    rcc_periph_clock_enable(RCC_SYSCFG);    // needed for EXTI
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO13);
    
    exti_select_source(EXTI13, GPIOC);
    exti_set_trigger(EXTI13, EXTI_TRIGGER_BOTH); //or RISING or FALLING 
    exti_enable_request(EXTI13);
    
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

}

void exti15_10_isr(void)
{
    exti_reset_request(EXTI13); // clear pending flag
    if (gpio_get(GPIOC, GPIO13)){
        printf("Button pressed!\n");
    }
    else{
        printf("Button released!\n");
    }
}


int main(void)
{
    int i, k=0;
 	

    clock_setup();
    timer_setup();
	gpio_setup();
    
    // fp = usart_setup(USART3);
    usart_setup(USART3);
    button_setup();
    
    timer_enable_strobe();
    
	/* Blink the LED (PC6) on the board. */
	while (1) {

        k += 1;
        printf("<< counting %d >>\n", k);
    

		gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 2000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}		
        gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 2000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}		
        gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 2000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}

		gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		for (i = 0; i < 10000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}

	}
        
        

	return 0;
}
