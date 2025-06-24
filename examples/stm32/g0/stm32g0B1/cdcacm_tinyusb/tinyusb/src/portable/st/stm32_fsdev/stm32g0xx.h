
// minimal defines required taken from stm43g0xx.h, which is part of CMSIS headers


#ifndef CMSIS_MINIMAL_H_
#define CMSIS_MINIMAL_H_



#define __DSB() __asm__("dsb")
#define __ISB() __asm__("isb")

#include <libopencm3/cm3/nvic.h>
#define NVIC_EnableIRQ(x) (nvic_enable_irq(x))
#define NVIC_DisableIRQ(x) (nvic_disable_irq(x))

typedef enum
{
/******  Cortex-M0+ Processor Exceptions Numbers ***************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M Hard Fault Interrupt                                   */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M SV Call Interrupt                                     */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M Pend SV Interrupt                                     */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M System Tick Interrupt                                 */
/******  STM32G0xxxx specific Interrupt Numbers ****************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_VDDIO2_IRQn             = 1,      /*!< PVD through EXTI line 16, PVM (monit. VDDIO2) through EXTI line 34*/
  RTC_TAMP_IRQn               = 2,      /*!< RTC interrupt through the EXTI line 19 & 21                       */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                            */
  RCC_CRS_IRQn                = 4,      /*!< RCC and CRS global Interrupt                                      */
  EXTI0_1_IRQn                = 5,      /*!< EXTI 0 and 1 Interrupts                                           */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupts                                      */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupts                                      */
  USB_UCPD1_2_IRQn            = 8,      /*!< USB, UCPD1 and UCPD2 global Interrupt                             */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupts                           */
  DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn = 11, /*!< DMA1 Ch4 to Ch7, DMA2 Ch1 to Ch5 and DMAMUX1 Overrun Interrupts */
  ADC1_COMP_IRQn              = 12,     /*!< ADC1, COMP1,COMP2, COMP3 Interrupts (combined with EXTI 17 & 18)  */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupts            */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 15,     /*!< TIM2 Interrupt                                                    */
  TIM3_TIM4_IRQn              = 16,     /*!< TIM3, TIM4 global Interrupt                                       */
  TIM6_DAC_LPTIM1_IRQn        = 17,     /*!< TIM6, DAC and LPTIM1 global Interrupts                            */
  TIM7_LPTIM2_IRQn            = 18,     /*!< TIM7 and LPTIM2 global Interrupt                                  */
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                                            */
  TIM15_IRQn                  = 20,     /*!< TIM15 global Interrupt                                            */
  TIM16_FDCAN_IT0_IRQn        = 21,     /*!< TIM16, FDCAN1_IT0 and FDCAN2_IT0 Interrupt                        */
  TIM17_FDCAN_IT1_IRQn        = 22,     /*!< TIM17, FDCAN1_IT1 and FDCAN2_IT1 Interrupt                        */
  I2C1_IRQn                   = 23,     /*!< I2C1 Interrupt  (combined with EXTI 23)                           */
  I2C2_3_IRQn                 = 24,     /*!< I2C2, I2C3 Interrupt (combined with EXTI 24 and EXTI 22)          */
  SPI1_IRQn                   = 25,     /*!< SPI1/I2S1 Interrupt                                               */
  SPI2_3_IRQn                 = 26,     /*!< SPI2/I2S2, SPI3/I2S3 Interrupt                                    */
  USART1_IRQn                 = 27,     /*!< USART1 Interrupt                                                  */
  USART2_LPUART2_IRQn         = 28,     /*!< USART2 + LPUART2 Interrupt                                        */
  USART3_4_5_6_LPUART1_IRQn   = 29,     /*!< USART3, USART4, USART5, USART6, LPUART1 globlal Interrupts (combined with EXTI 28) */
  CEC_IRQn                    = 30,     /*!< CEC Interrupt(combined with EXTI 27)                               */
} IRQn_Type;

#define PERIPH_BASE           (0x40000000UL)
#define APBPERIPH_BASE        (PERIPH_BASE)
//#define USB_BASE              (APBPERIPH_BASE + 0x00005C00UL)
#define USB_DRD_BASE          (APBPERIPH_BASE + 0x00005C00UL)
#define USB_DRD_PMAADDR       (APBPERIPH_BASE + 0x00009800UL)

// #define MMIO32(addr) (*(volatile uint32_t *)(addr))
// #define FSDEV_REG_CNTR      MMIO32(USB_DRD_BASE + 0x40)
// #define FSDEV_REG_ISTR      MMIO32(USB_DRD_BASE + 0x44)
// #define FSDEV_REG_FNR       MMIO32(USB_DRD_BASE + 0x48)
// #define FSDEV_REG_DADDR     MMIO32(USB_DRD_BASE + 0x4C)
// #define FSDEV_REG_BTABLE    MMIO32(USB_DRD_BASE + 0x40)


//USB_EP_VTRX
#define USB_CHEP_VTRX_Pos              (15U)
#define USB_CHEP_VTRX_Msk              (0x1UL << USB_CHEP_VTRX_Pos)     /*!< 0x00008000 */
#define USB_CHEP_VTRX                  USB_CHEP_VTRX_Msk                /*!< USB valid transaction received Mask */
#define USB_EP_VTRX                    USB_CHEP_VTRX_Msk                /*!< USB Endpoint valid transaction received Mask */
  
//USB_EP_VTTX 
#define USB_CHEP_VTTX_Pos              (7U)
#define USB_CHEP_VTTX_Msk              (0x1UL << USB_CHEP_VTTX_Pos)     /*!< 0x00000080 */
#define USB_CHEP_VTTX                  USB_CHEP_VTTX_Msk                /*!< Valid USB transaction transmitted Mask */
#define USB_EP_VTTX                    USB_CHEP_VTTX_Msk                /*!< USB Endpoint valid transaction transmitted Mask */

//USB_CHEP_UTYPE
#define USB_CHEP_UTYPE_Pos             (9U)
#define USB_CHEP_UTYPE_Msk             (0x3UL << USB_CHEP_UTYPE_Pos)    /*!< 0x00000600 */
#define USB_CHEP_UTYPE                 USB_CHEP_UTYPE_Msk               /*!< USB type of transaction Mask */

//USB_CHEP_REG_MASK
#define USB_CHEP_REG_MASK              (0x07FF8F8FU)

//USB_CHEP_TX_DTOGMASK
#define USB_CHEP_TX_STTX_Pos           (4U)
#define USB_CHEP_TX_STTX_Msk           (0x3UL << USB_CHEP_TX_STTX_Pos)  /*!< 0x00000030 */
#define USB_CHEP_TX_STTX               USB_CHEP_TX_STTX_Msk  
#define USB_CHEP_TX_DTOGMASK           (USB_CHEP_TX_STTX | USB_CHEP_REG_MASK)

//USB_CHEP_RX_DTOGMASK
#define USB_CHEP_RX_STRX_Pos           (12U)
#define USB_CHEP_RX_STRX_Msk           (0x3UL << USB_CHEP_RX_STRX_Pos)  /*!< 0x00003000 */
#define USB_CHEP_RX_STRX               USB_CHEP_RX_STRX_Msk             /*!< Status bits, for reception transfers Mask */
#define USB_CHEP_RX_DTOGMASK           (USB_CHEP_RX_STRX | USB_CHEP_REG_MASK)

#define USB_CHEP_TX_DTOG1                          (0x00000010UL)           /*!< Channel/EndPoint TX Data Toggle bit1 */
#define USB_CHEP_TX_DTOG2                          (0x00000020UL)           /*!< Channel/EndPoint TX Data Toggle bit2 */
#define USB_CHEP_RX_DTOG1                          (0x00001000UL)           /*!< Channel/EndPoint RX Data Toggle bit1 */
#define USB_CHEP_RX_DTOG2                          (0x00002000UL)           /*!< Channel/EndPoint RX Data Toggle bit1 */

//USB_CH_RX_VALID
#define USB_CH_RX_VALID                            (0x00003000UL)           /*!< Channel RX VALID */

//USB_EP_KIND_MASK
#define USB_CHEP_KIND_Pos              (8U)
#define USB_CHEP_KIND_Msk              (0x1UL << USB_CHEP_KIND_Pos)     /*!< 0x00000100 */
#define USB_CHEP_KIND                  USB_CHEP_KIND_Msk                /*!< EndPoint KIND Mask */
#define USB_EP_KIND                    USB_CHEP_KIND_Msk                /*!< EndPoint KIND Mask */
#define USB_EP_KIND_MASK               ((~USB_EP_KIND) & USB_CHEP_REG_MASK) /*!< EP_KIND EndPoint KIND */

//USB_CNTR_USBRST
#define USB_CNTR_USBRST_Pos             (0U)
#define USB_CNTR_USBRST_Msk             (0x1UL << USB_CNTR_USBRST_Pos)  /*!< 0x00000001 */
#define USB_CNTR_USBRST                 USB_CNTR_USBRST_Msk             /*!< USB Reset Mask */

//USB_CNTR_L2RES
#define USB_CNTR_L2RES_Pos              (4U)
#define USB_CNTR_L2RES_Msk              (0x1UL << USB_CNTR_L2RES_Pos)   /*!< 0x00000010 */
#define USB_CNTR_L2RES                  USB_CNTR_L2RES_Msk              /*!< L2 Remote Wakeup / Resume driver Mask */

//USB_ISTR_IDN
#define USB_ISTR_IDN_Pos                (0U)
#define USB_ISTR_IDN_Msk                (0xFUL << USB_ISTR_IDN_Pos)     /*!< 0x0000000F */
#define USB_ISTR_IDN                    USB_ISTR_IDN_Msk                /*!< EndPoint IDentifier (read-only bit) Mask */

//USB_CHEP_ADDR
#define USB_CHEP_ADDR_Pos              (0U)
#define USB_CHEP_ADDR_Msk              (0xFUL << USB_CHEP_ADDR_Pos)     /*!< 0x0000000F */
#define USB_CHEP_ADDR                  USB_CHEP_ADDR_Msk                /*!< Endpoint address Mask */

//USB_CNTR_SUSPRDY
#define USB_CNTR_SUSPRDY_Pos            (2U)
#define USB_CNTR_SUSPRDY_Msk            (0x1UL << USB_CNTR_SUSPRDY_Pos) /*!< 0x00000004 */
#define USB_CNTR_SUSPRDY                USB_CNTR_SUSPRDY_Msk            /*!< Suspend state effective Mask */

//USB_CNTR_SUSPEN
#define USB_CNTR_SUSPEN_Pos             (3U)
#define USB_CNTR_SUSPEN_Msk             (0x1UL << USB_CNTR_SUSPEN_Pos)  /*!< 0x00000008 */
#define USB_CNTR_SUSPEN                 USB_CNTR_SUSPEN_Msk             /*!< Suspend state enable Mask */

#define USB_EP_TYPE_MASK                           (0x00000600UL)           /*!< Channel/EndPoint TYPE Mask */
#define USB_EP_BULK                                (0x00000000UL)           /*!< Channel/EndPoint BULK */
#define USB_EP_CONTROL                             (0x00000200UL)           /*!< Channel/EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS                         (0x00000400UL)           /*!< Channel/EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT                           (0x00000600UL)           /*!< Channel/EndPoint INTERRUPT */

//USB_EP_DTOG_TX
#define USB_CHEP_DTOG_TX_Pos           (6U)
#define USB_CHEP_DTOG_TX_Msk           (0x1UL << USB_CHEP_DTOG_TX_Pos)  /*!< 0x00000040 */
#define USB_EP_DTOG_TX                 USB_CHEP_DTOG_TX_Msk             /*!< EP Data Toggle, for transmission transfers Mask */

//USB_EP_DTOG_RX
#define USB_CHEP_DTOG_RX_Pos           (14U)
#define USB_CHEP_DTOG_RX_Msk           (0x1UL << USB_CHEP_DTOG_RX_Pos)  /*!< 0x00004000 */
#define USB_EP_DTOG_RX                 USB_CHEP_DTOG_RX_Msk             /*!< EP Data Toggle, for reception transfers Mask */

//USB_EP_SETUP 
#define USB_CHEP_SETUP_Pos             (11U)
#define USB_CHEP_SETUP_Msk             (0x1UL << USB_CHEP_SETUP_Pos)    /*!< 0x00000800 */
#define USB_EP_SETUP                   USB_CHEP_SETUP_Msk               /*!< EP Setup transaction completed Mask */

//USB_CNTR_PDWN
#define USB_CNTR_PDWN_Pos               (1U)
#define USB_CNTR_PDWN_Msk               (0x1UL << USB_CNTR_PDWN_Pos)    /*!< 0x00000002 */
#define USB_CNTR_PDWN                   USB_CNTR_PDWN_Msk               /*!< Power DoWN Mask */

//USB_CNTR_CTRM
#define USB_CNTR_CTRM_Pos               (15U)
#define USB_CNTR_CTRM_Msk               (0x1UL << USB_CNTR_CTRM_Pos)    /*!< 0x00008000 */
#define USB_CNTR_CTRM                   USB_CNTR_CTRM_Msk               /*!< Correct Transfer Mask */

//USB_CNTR_PMAOVRM
#define USB_CNTR_PMAOVRM_Pos            (14U)
#define USB_CNTR_PMAOVRM_Msk            (0x1UL << USB_CNTR_PMAOVRM_Pos) /*!< 0x00004000 */
#define USB_CNTR_PMAOVRM                USB_CNTR_PMAOVRM_Msk            /*!< DMA OVeR/underrun Mask */

// #define USB_CNTR_ERRM_Pos               (13U)
// #define USB_CNTR_ERRM_Msk               (0x1UL << USB_CNTR_ERRM_Pos)    /*!< 0x00002000 */
// #define USB_CNTR_ERRM                   USB_CNTR_ERRM_Msk               /*!< ERRor Mask */

//USB_CNTR_WKUPM
#define USB_CNTR_WKUPM_Pos              (12U)
#define USB_CNTR_WKUPM_Msk              (0x1UL << USB_CNTR_WKUPM_Pos)   /*!< 0x00001000 */
#define USB_CNTR_WKUPM                  USB_CNTR_WKUPM_Msk              /*!< WaKe UP Mask */

//USB_CNTR_SUSPM
#define USB_CNTR_SUSPM_Pos              (11U)
#define USB_CNTR_SUSPM_Msk              (0x1UL << USB_CNTR_SUSPM_Pos)   /*!< 0x00000800 */
#define USB_CNTR_SUSPM                  USB_CNTR_SUSPM_Msk              /*!< SUSPend Mask */

//USB_CNTR_RESETM
#define USB_CNTR_RESETM_Pos             (10U)
#define USB_CNTR_RESETM_Msk             (0x1UL << USB_CNTR_RESETM_Pos)  /*!< 0x00000400 */
#define USB_CNTR_RESETM                 USB_CNTR_RESETM_Msk             /*!< RESET Mask */
//#define USB_CNTR_DCON                   USB_CNTR_RESETM_Msk             /*!< Disconnection Connection Mask */

//USB_CNTR_SOFM
#define USB_CNTR_SOFM_Pos               (9U)
#define USB_CNTR_SOFM_Msk               (0x1UL << USB_CNTR_SOFM_Pos)    /*!< 0x00000200 */
#define USB_CNTR_SOFM                   USB_CNTR_SOFM_Msk               /*!< Start Of Frame Mask */

//USB_CNTR_ESOFM
#define USB_CNTR_ESOFM_Pos              (8U)
#define USB_CNTR_ESOFM_Msk              (0x1UL << USB_CNTR_ESOFM_Pos)   /*!< 0x00000100 */
#define USB_CNTR_ESOFM                  USB_CNTR_ESOFM_Msk              /*!< Expected Start Of Frame Mask */

//USB_ISTR_ESOF
#define USB_ISTR_ESOF_Pos               (8U)
#define USB_ISTR_ESOF_Msk               (0x1UL << USB_ISTR_ESOF_Pos)    /*!< 0x00000100 */
#define USB_ISTR_ESOF                   USB_ISTR_ESOF_Msk               /*!< Expected Start Of Frame (clear-only bit) Mask */

//USB_ISTR_SOF
#define USB_ISTR_SOF_Pos                (9U)
#define USB_ISTR_SOF_Msk                (0x1UL << USB_ISTR_SOF_Pos)     /*!< 0x00000200 */
#define USB_ISTR_SOF                    USB_ISTR_SOF_Msk                /*!< Start Of Frame (clear-only bit) Mask */

//USB_ISTR_RESET
#define USB_ISTR_RESET_Pos              (10U)
#define USB_ISTR_RESET_Msk              (0x1UL << USB_ISTR_RESET_Pos)   /*!< 0x00000400 */
#define USB_ISTR_RESET                  USB_ISTR_RESET_Msk              /*!< RESET Mask */

// #define USB_ISTR_DCON_Pos               (10U)
// #define USB_ISTR_DCON_Msk               (0x1UL << USB_ISTR_DCON_Pos)    /*!< 0x00000400 */
// #define USB_ISTR_DCON                   USB_ISTR_DCON_Msk               /*!< HOST MODE-Device Connection or disconnection Mask */

//USB_ISTR_SUSP
#define USB_ISTR_SUSP_Pos               (11U)
#define USB_ISTR_SUSP_Msk               (0x1UL << USB_ISTR_SUSP_Pos)    /*!< 0x00000800 */
#define USB_ISTR_SUSP                   USB_ISTR_SUSP_Msk               /*!< SUSPend (clear-only bit) Mask */

//USB_ISTR_WKUP
#define USB_ISTR_WKUP_Pos               (12U)
#define USB_ISTR_WKUP_Msk               (0x1UL << USB_ISTR_WKUP_Pos)    /*!< 0x00001000 */
#define USB_ISTR_WKUP                   USB_ISTR_WKUP_Msk               /*!< WaKe UP (clear-only bit) Mask */

// #define USB_ISTR_ERR_Pos                (13U)
// #define USB_ISTR_ERR_Msk                (0x1UL << USB_ISTR_ERR_Pos)     /*!< 0x00002000 */
// #define USB_ISTR_ERR                    USB_ISTR_ERR_Msk                /*!< ERRor (clear-only bit) Mask */

//USB_ISTR_PMAOVR
#define USB_ISTR_PMAOVR_Pos             (14U)
#define USB_ISTR_PMAOVR_Msk             (0x1UL << USB_ISTR_PMAOVR_Pos)  /*!< 0x00004000 */
#define USB_ISTR_PMAOVR                 USB_ISTR_PMAOVR_Msk             /*!< PMA OVeR/underrun (clear-only bit) Mask */

//USB_ISTR_CTR
#define USB_ISTR_CTR_Pos                (15U)
#define USB_ISTR_CTR_Msk                (0x1UL << USB_ISTR_CTR_Pos)     /*!< 0x00008000 */
#define USB_ISTR_CTR                    USB_ISTR_CTR_Msk                /*!< Correct TRansfer (clear-only bit) Mask */

//USB_DADDR_EF
#define USB_DADDR_EF_Pos                (7U)
#define USB_DADDR_EF_Msk                (0x1UL << USB_DADDR_EF_Pos)     /*!< 0x00000080 */
#define USB_DADDR_EF                    USB_DADDR_EF_Msk                /*!< Enable Function Mask */

//USB_FNR_FN
#define USB_FNR_FN_Pos                  (0U)
#define USB_FNR_FN_Msk                  (0x7FFUL << USB_FNR_FN_Pos)     /*!< 0x000007FF */
#define USB_FNR_FN                      USB_FNR_FN_Msk                  /*!< Frame Number Mask */


#endif //CMSIS_MINIMAL_H_

