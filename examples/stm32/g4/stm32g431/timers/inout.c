
#include "inout.h"

// static void gpio_setup(void)
// {
	// led_setup();
	// button_setup();
// }


static void led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

static void led_toggle(void)
{
	gpio_toggle(PORT_LED, PIN_LED);
}

static int button_pressed(void)
{
	return gpio_get(BUTTON_PORT, BUTTON_PIN);
}

static void button_setup(void){
    rcc_periph_clock_enable(RCC_GPIOC);     // button port
    rcc_periph_clock_enable(RCC_SYSCFG);    // needed for EXTI
    gpio_mode_setup(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BUTTON_PIN);
    
    exti_select_source(EXTI13, BUTTON_PORT);
    exti_set_trigger(EXTI13, EXTI_TRIGGER_BOTH); //or RISING or FALLING 
    exti_enable_request(EXTI13);
    
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

}


void exti15_10_isr(void)
{
    exti_reset_request(EXTI13); // clear pending flag
	button_isr();
    // if (gpio_get(BUTTON_PORT, BUTTON_PIN)){
        // printf("Button pressed!\n");
    // }
    // else{
        // printf("Button released!\n");
    // }
}