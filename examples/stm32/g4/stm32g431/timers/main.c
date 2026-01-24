
#include "main.h"


void button_isr(void) {
	if (button_pressed()) {
        printf("\n[X] Button pressed  (%d)\n", n_btn);
        // SPI3_DR = my_buf;
        //timer_isr();
    }
    else {
        printf("[ ] Button released (%d)\n", n_btn);
        // timer_generate_event(TIM3, TIM_EGR_TG );

        n_btn++;
    }
    
    // if (timer_get_flag(TIM3, TIM_SR_TIF )){
        // printf("TIF=1\n");

    // }
    // else{
        // printf("TIF=0\n");
    // }
}

void timer_isr(void){
    //send SPI
    //SPI3_DR = n;
    //SPI3_DR = TIM3_CNT;
}



int main(void)
{
    
    clock_setup();
    timer_setup();
	led_setup();
	button_setup();
    usart_setup(USART3);
    spi_setup();
    dac_setup();
    
    
    timer_enable_strobe();	
    
    uint16_t dac_mid = 0x1000/2;
    uint16_t dac_span = 0x1000;
    
    uint16_t dac_ch1 = dac_mid - dac_span/2;
    uint16_t dac_ch2 = dac_mid;
    
	/* Blink the LED on the board. */
	while (1) {
        dac_ch1 += 0x10;
        if (dac_ch1 >= dac_mid + dac_span/2) dac_ch1 -= dac_span; 
        dac_ch2 -= 0x10;
        if (dac_ch2 <  dac_mid - dac_span/2) dac_ch2 += dac_span;
        
        dac_update(dac_ch1, dac_ch2);
        
        n += 1;
        //printf("<< counting %d >>\n", n);
        //spi_send(SPI3, n);
        //SPI3_DR = n;
        //timer_isr();

		// led_toggle();	
		// wait(20000);
		
        // led_toggle();	
		// wait(20000);
		
        // led_toggle();	
		// wait(20000);

		led_toggle();
		wait(1000000);

	}

	return 0;
}
