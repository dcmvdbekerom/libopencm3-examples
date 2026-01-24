
#include "main.h"


void button_isr(void) {
	if (button_pressed()) {
        printf("\n[X] Button pressed  (%d)\n", n_btn);
    }
    else {
        printf("[ ] Button released (%d)\n", n_btn);
        n_btn++;
    }
}

void timer_isr(void){
    //send SPI
    SPI3_DR = n;
}



int main(void)
{
    
 	
    clock_setup();
    timer_setup();
	led_setup();
	button_setup();
    usart_setup(USART3);
    spi_setup();
    
    timer_enable_strobe();	
    
	/* Blink the LED on the board. */
	while (1) {

        n += 1;
        //printf("<< counting %d >>\n", n);
        //spi_send(SPI3, n);
        //SPI3_DR = n;

		led_toggle();	
		wait(2000000);
		
        led_toggle();	
		wait(2000000);
		
        led_toggle();	
		wait(2000000);

		led_toggle();
		wait(10000000);

	}

	return 0;
}
