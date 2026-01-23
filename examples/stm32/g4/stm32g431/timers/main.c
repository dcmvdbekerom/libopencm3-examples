
#include "main.h"


void button_isr(void) {
	if (button_pressed()) {
        printf("Button pressed !!!\n");
    }
    else {
        printf("Button released !!!\n");
    }
}


int main(void)
{
    int k=0;
 	
    clock_setup();
    timer_setup();
	led_setup();
	button_setup();
    usart_setup(USART3);
    spi_setup();
    
    timer_enable_strobe();	
    
	/* Blink the LED on the board. */
	while (1) {

        k += 1;
        printf("<< counting %d >>\n", k);
        spi_send(SPI1, k);

		led_toggle();	
		wait(200000);
		
        led_toggle();	
		wait(200000);
		
        led_toggle();	
		wait(200000);

		led_toggle();
		wait(1000000);

	}

	return 0;
}
