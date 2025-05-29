/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
 * Copyright (c) 2023, HiFiPhile
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/* metadata:
   name: STM32 G0B1 Nucleo
   url: https://www.st.com/en/evaluation-tools/nucleo-g0b1re.html
*/

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

// G0B1RE Nucleo does not has usb connection. We need to manually connect
// - PA12 for D+, CN10.12
// - PA11 for D-, CN10.14

// LED
#define LED_PORT              GPIOC
#define LED_PIN               GPIO6
#define LED_STATE_ON          0

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO13
#define BUTTON_STATE_ACTIVE   0

// UART Enable for STLink VCOM
#define UART_DEV              USART2
#define UART_CLK_EN           __HAL_RCC_USART2_CLK_ENABLE
#define UART_GPIO_PORT        GPIOA
#define UART_GPIO_AF          GPIO_AF1_USART2
#define UART_TX_PIN           GPIO2
#define UART_RX_PIN           GPIO3


#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crs.h> //manually added


//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
#if 1
// Clock configure for STM32G0B1RE Nucleo
static inline void board_clock_init(void)
{

    /** Configure the main internal regulator output voltage */
    //PWR_CR1 = (PWR_CR1 & ~PWR_CR1_VOS) | ( PWR_CR1_SCALE1);
    pwr_set_vos_scale(PWR_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure. */

      // 1. Enable HSI oscillator
    rcc_osc_on(RCC_HSI) ;             // Turn on HSI
    while (!rcc_is_osc_ready(RCC_HSI)){};   // Wait until HSI ready

    // // 2. Configure HSI divider (HSIDIV bits 7:6 in RCC_CR)
    // RCC_CR &= ~RCC_CR_HSIDIV;                 // Clear HSIDIV bits
    // RCC_CR |= RCC_CR_HSIDIV_0;                // HSIDIV = DIV1 (0b00, so could omit)
    
    
    
    
    
#define RCC_HSICALIBRATION_DEFAULT     64U  

    // 3. HSI calibration (bits 15:8 in RCC_CR)
    RCC_ICSCR = (RCC_ICSCR & ~RCC_ICSCR_HSICAL_MASK) | RCC_HSICALIBRATION_DEFAULT;

    // 4. Configure PLL
    // Disable PLL first
    // RCC_CR &= ~RCC_CR_PLLON;
    // while ((RCC_CR & RCC_CR_PLLRDY) != 0){};   // Wait until PLL disabled

    // // Set PLL source to HSI (bit PLLSRC in RCC_PLLCFGR)
        // RCC_PLLCFGR = (0 << RCC_PLLCFGR_PLLM_Pos)    // PLLM = DIV1 (0 means DIV1)
             // | (8 << RCC_PLLCFGR_PLLN_Pos)    // PLLN = 8
             // | (0 << RCC_PLLCFGR_PLLP_Pos)    // PLLP = DIV2 (0 means DIV2)
             // | (0 << RCC_PLLCFGR_PLLQ_Pos)    // PLLQ = DIV2 (0 means DIV2)
             // | (0 << RCC_PLLCFGR_PLLR_Pos)    // PLLR = DIV2 (0 means DIV2)
             // | RCC_PLLCFGR_PLLSRC_HSI          // PLL source = HSI
             // | RCC_PLLCFGR_PLLREN               // Enable PLLR output (usually needed)
             // ;
    // // 5. Enable PLL
    // RCC_CR |= RCC_CR_PLLON;
    // while ((RCC_CR & RCC_CR_PLLRDY) == 0){};   // Wait until PLL ready
      
    rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSE, 
                     RCC_PLLCFGR_PLLM_DIV(1),
                     RCC_PLLCFGR_PLLN_MUL(8) ,
                     RCC_PLLCFGR_PLLP_DIV(2) ,
                     RCC_PLLCFGR_PLLQ_DIV(2) ,
                     RCC_PLLCFGR_PLLR_DIV(2));  
    rcc_enable_pllr(true);  
  

// #define FLASH_LATENCY_2                 FLASH_ACR_LATENCY_1
// // Configure Flash latency (FLASH_ACR register)
    // FLASH_ACR &= ~FLASH_ACR_LATENCY;
    // FLASH_ACR |= FLASH_LATENCY_2;      // 2 wait states
 // while( (FLASH_ACR&FLASH_ACR_LATENCY) != FLASH_LATENCY_2) {};
    
    
    flash_set_ws(FLASH_ACR_LATENCY_2WS);
    flash_wait_for_last_operation();
   
// #define RCC_SYSCLK_DIV1                0x00000000U 
// #define RCC_HCLK_DIV1                  0x00000000U  
 
    // RCC_CFGR = (RCC_CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE))
        // | RCC_CFGR_SW_PLLRCLK  // SYSCLK = PLL
        // | RCC_SYSCLK_DIV1      // AHB prescaler = /1 //HAL macro
        // | RCC_HCLK_DIV1 ;     // APB1 prescaler = /1 //HAL macro

    // // Wait until PLL is used as system clock
    // while ((RCC_CFGR & RCC_CFGR_SW) != RCC_CFGR_SW_PLLRCLK) { }
    
    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
    rcc_set_ppre(RCC_CFGR_PPRE_NODIV);
    rcc_set_sysclk_source(RCC_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);


  // Configure CRS clock source
  // __HAL_RCC_CRS_CLK_ENABLE();
  // RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  // RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  // RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  // RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  // RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  // RCC_CRSInitStruct.ErrorLimitValue = 34;
  // RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  //HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
  
  
    // Enable CRS peripheral clock
    // RCC_APBENR1 |= RCC_APBENR1_CRSEN;
    rcc_periph_clock_enable(RCC_CRS);

    // Build CFGR value
    CRS_CFGR = (CRS_CFGR_RELOAD_VAL	(47999) |  // RELOAD value (48MHz / 1kHz - 1)
                CRS_CFGR_FELIM_VAL	(34)    |  // Error limit
                CRS_CFGR_SYNCDIV_NODIV      |  // SYNC_DIV1
                CRS_CFGR_SYNCSRC_USB_SOF );   // SYNC_SOURCE_USB
    CRS_CFGR &= ~CRS_CFGR_SYNCPOL;          // Rising polarity (default)


    // Set HSI48 calibration value
    CRS_CR = (CRS_CR & ~CRS_CR_TRIM) | (32 << CRS_CR_TRIM_SHIFT);

    // Enable automatic trimming and frequency error counter
    //CRS_CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
    crs_autotrim_usb_enable();


  /* Select HSI48 as USB clock source */
  // RCC_PeriphCLKInitTypeDef usb_clk = {0 };
  // usb_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  // usb_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  // HAL_RCCEx_PeriphCLKConfig(&usb_clk);

    RCC_CCIPR2 &= ~(RCC_CCIPR2_USBSEL_MASK  << RCC_CCIPR2_USBSEL_SHIFT);
    RCC_CCIPR2 |=  (RCC_CCIPR2_USBSEL_HSI48 << RCC_CCIPR2_USBSEL_SHIFT);  // 00: HSI48 selected as USB clock

  // Enable HSI48
  // RCC_OscInitTypeDef osc_hsi48 = {0};
  // osc_hsi48.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  // osc_hsi48.HSI48State = RCC_HSI48_ON;
  // HAL_RCC_OscConfig(&osc_hsi48);
  
  
    RCC_CR |= RCC_CR_HSI48ON;

    // Wait until HSI48 is ready
    while ((RCC_CR & RCC_CR_HSI48RDY) == 0) {};
}
// DvdB: uncomment code that doesnt do anything
// #else

// // Clock configure for STM32G0 nucleo with B0 mcu variant for someone that is skilled enough
// // to rework and solder the B0 chip. Note: SB17 may need to be soldered as well (check user manual)
// static inline void board_clock_init(void)
// {
  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  // RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct = { 0 };

  // /** Configure the main internal regulator output voltage */
  // HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  // /** Initializes the RCC Oscillators according to the specified parameters
  // * in the RCC_OscInitTypeDef structure. */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  // RCC_OscInitStruct.PLL.PLLN = 12;
  // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  // RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  // RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  // HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // /* Select HSI48 as USB clock source */
  // PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  // PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  // HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  // /** Initializes the CPU, AHB and APB buses clocks */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              // |RCC_CLOCKTYPE_PCLK1;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  // HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
// }
#endif

#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
