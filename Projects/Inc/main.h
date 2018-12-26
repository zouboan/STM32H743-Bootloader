/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h" 
#include "usbd_cdc_interface.h"

/* Exported types ------------------------------------------------------------*/
#define LED_GREEN       LED1
#define LED_ORANGE      LED2
#define LED_RED         LED3
#define LED_BLUE        LED4

/* Exported constants --------------------------------------------------------*/
/* Defines related to Clock configuration */
/* Uncomment to enable the adaquate Clock Source */
#define RTC_CLOCK_SOURCE_LSI
/*#define RTC_CLOCK_SOURCE_LSE*/

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0x0130    
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SystemClock_Config(void);
void cfini(void);
void Error_Handler(void);
extern void usb_buf_put(uint8_t b);
extern void uart_buf_put(uint8_t b);
extern unsigned uart_head, uart_tail;
extern uint8_t uart_rx_buf[256];
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
