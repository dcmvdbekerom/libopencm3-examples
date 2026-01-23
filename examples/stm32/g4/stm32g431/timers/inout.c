
#include "inout.h"

// static void gpio_setup(void)
// {
	// led_setup();
	// button_setup();
// }

#define SPI3_PORT GPIOB
#define SPI3_PINS GPIO3|GPIO5
#define SPI3_AF   GPIO_AF6
void spi_setup(void){
    // AF5: B3=SCK B5=MOSI
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI3);
    
    gpio_mode_setup(SPI3_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI3_PINS);
    gpio_set_af(SPI3_PORT, SPI3_AF, SPI3_PINS);
    gpio_set_output_options(SPI3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, SPI3_PINS);


    spi_disable(SPI3);
    spi_init_master(SPI3, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_LSBFIRST);
    
    spi_set_data_size(SPI3, SPI_CR2_DS_16BIT );
    spi_enable(SPI3);

}

void led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

void led_toggle(void)
{
	gpio_toggle(LED_PORT, LED_PIN);
}

int button_pressed(void)
{
	return gpio_get(BUTTON_PORT, BUTTON_PIN);
}

void button_setup(void){
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