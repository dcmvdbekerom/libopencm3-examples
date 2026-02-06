
#include "main.h"

int gains[] = {1,2,5,10,20,50,100,200};
uint16_t offsets[] = {0xFFF, 1856, 1840, 1872, 0xC56, 0xC34, 0xC29, 0xC24};


const uint16_t dac_mid = 1900;// 0x1000/2;
const uint16_t dac_span = 0x100;

uint16_t dac_ch1 = dac_mid - dac_span/2;
uint16_t dac_ch2 = dac_mid;

int vref_enable = 0;

void button_isr(void) {
	if (button_pressed()) {

        if (1){//(n_btn&0x1) {        
            printf("\n[X] Button pressed  (gain = %d; DAC = %d) - CH1\n", gains[n_btn], dac_ch1);

        }
        else{
            printf("\n[X] Button pressed  (gain = %d) - CH0\n", gains[n_btn]);
        }
        vref_enable = 0;
        //assert ~CS
        dac_update(0x0FFF,0x0FFF);
        wait(60); //5us
        dac_update(0x0000,0x0000);
        wait(60);
        
        //SPI
        
        if (1){//(n_btn&0x1) {        
           pga_write(PGA_MUX_CH1, n_btn);
        }
        else{
            pga_write(PGA_MUX_VCAL_CH0, PGA_GAIN_1X);
        }
        wait(100);
        
        //deassert ~CS
        wait(60);
        dac_update(0x0FFF,0x0FFF);
        wait(60);
        dac_update(1900,1900);
        vref_enable = 1;
        //wait(60); //5us
        //dac_update(0x041F,0x041F);
    
        
        n_btn++;
        if (n_btn >= 8) n_btn = 0;
        // SPI3_DR = my_buf;
        //timer_isr();
    }
    else {
       // printf("[ ] Button released (%d)\n", n_btn);
        // timer_generate_event(TIM3, TIM_EGR_TG );

        //n_btn++;
        //if (n_btn >= 8) n_btn = 0;
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
    vref_enable = 1;
    
	/* Blink the LED on the board. */
	while (1) {
        dac_ch1 += 0x2;
        if (dac_ch1 >= dac_mid + dac_span/2) dac_ch1 -= dac_span; 
        //dac_ch2 -= 0x10;
        //if (dac_ch2 <  dac_mid - dac_span/2) dac_ch2 += dac_span;
        
        //if (vref_enable) dac_update(dac_ch1, dac_ch1);
        
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
