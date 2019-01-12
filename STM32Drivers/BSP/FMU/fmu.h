/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H743I_EVAL_H
#define __STM32H743I_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Boot device selection list*/
#define USB0_DEV	0x01
#define SERIAL0_DEV	0x02
#define SERIAL1_DEV	0x04
/* flash parameters that we should not really know */
static struct {
	uint32_t	sector_number;
	uint32_t	size;
} flash_sectors[] = {

	/* Physical FLASH sector 0 is reserved for bootloader and is not
	 * the table below.
	 * N sectors may aslo be reserved for the app fw in which case
	 * the zero based define BOARD_FIRST_FLASH_SECTOR_TO_ERASE must
	 * be defined to begin the erase above of the reserved sectors.
	 * The default value of BOARD_FIRST_FLASH_SECTOR_TO_ERASE is 0
	 * and begins flash erase operations at phsical sector 1 the 0th entry
	 * in the table below.
	 * A value of 1 for BOARD_FIRST_FLASH_SECTOR_TO_ERASE would reserve
	 * the 0th entry and begin erasing a index 1 the third physical sector
	 * on the device.
	 *
	 * When BOARD_FIRST_FLASH_SECTOR_TO_ERASE is defined APP_RESERVATION_SIZE
	 * must also be defined to remove that additonal reserved FLASH space
	 * from the BOARD_FLASH_SIZE. See APP_SIZE_MAX below.
	 */

	{0x01, 128 * 1024},
	{0x02, 128 * 1024},
	{0x03, 128 * 1024},
	{0x04, 128 * 1024},
	{0x05, 128 * 1024},
	{0x06, 128 * 1024},
	{0x07, 128 * 1024},
	/* flash sectors only in 2MiB devices */
	{0x08, 128 * 1024},
	{0x09, 128 * 1024},
	{0x0a, 128 * 1024},
	{0x0b, 128 * 1024},
	{0x0c, 128 * 1024},
	{0x0d, 128 * 1024},
	{0x0e, 128 * 1024},
	{0x0f, 128 * 1024},
};
/** @addtogroup BSP
  * @{
  */
# define APP_LOAD_ADDRESS               0x08020000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV6
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
//# define USBDEVICESTRING                "PX4 BL FMU v3.x"
//# define USBPRODUCTID                   0x0011
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     9
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff1e880)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 15)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       25

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB4ENR_GPIOEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART  			USART2
# define BOARD_USART_CLOCK_REGISTER 		RCC_APB1LENR
# define BOARD_USART_CLOCK_BIT      		RCC_APB1LENR_USART2EN

# define BOARD_PORT_USART   						GPIOD
# define BOARD_PORT_USART_AF 						GPIO_AF7
# define BOARD_PIN_TX     							GPIO5
# define BOARD_PIN_RX		     						GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB4ENR
# define BOARD_USART_PIN_CLOCK_BIT  		RCC_AHB4ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1

extern UART_HandleTypeDef UartHandle;
/** @addtogroup STM32H743I_EVAL
  * @{
  */
#if !defined(USBMFGSTRING)
# define USBMFGSTRING "3D Robotics"
#endif

#if !defined(USBVENDORID)
#  define USBVENDORID 0x26AC
#endif

#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE 0
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 0
#endif

#if defined(OVERRIDE_USART_BAUDRATE)
#  define USART_BAUDRATE OVERRIDE_USART_BAUDRATE
#else
#  define USART_BAUDRATE 115200
#endif

#if INTERFACE_USART
#  if !defined(BOARD_PORT_USART_TX)
#    define BOARD_PORT_USART_TX BOARD_PORT_USART
#    define BOARD_PORT_USART_RX BOARD_PORT_USART
#  endif
#  if !defined(BOARD_USART_PIN_CLOCK_BIT_TX)
#    define BOARD_USART_PIN_CLOCK_BIT_TX BOARD_USART_PIN_CLOCK_BIT
#    define BOARD_USART_PIN_CLOCK_BIT_RX BOARD_USART_PIN_CLOCK_BIT
#  endif
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif
#if !defined(UART_DATA_ALIGN)
# define UART_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

/** @defgroup STM32H743I_EVAL_LOW_LEVEL LOW LEVEL
  * @{
  */
	
#define BOOTLOADER_RESERVATION_SIZE	  (128 * 1024)
#define UDID_START		        				0x1FF1E800

// address of MCU IDCODE
#define DBGMCU_IDCODE									0x5C001000 //0xE00E1000
#define STM32_UNKNOWN									0
#define STM32H7x3											0x450

#define REVID_MASK										0xFFFF0000
#define DEVID_MASK										0xFFF

#ifndef BOARD_PIN_VBUS
#define BOARD_PIN_VBUS                 GPIO_PIN_9
#define BOARD_PORT_VBUS                GPIOA
#define BOARD_CLOCK_VBUS               RCC_AHB4ENR_GPIOAEN
#endif

#define FIRST_BAD_SILICON_OFFSET 1

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

/* enum for whether bootloading via USB or USART */
enum {
	NONE,
	USART,
	USB
};

static unsigned usb_head, usb_tail;
static uint8_t usb_rx_buf[256] USB_DATA_ALIGN;

static enum led_state {LED_BLINK, LED_ON, LED_OFF} _led_state;
/* board info forwarded from board-specific code to booloader */
struct boardinfo {
	uint32_t	board_type;
	uint32_t	board_rev;
	uint32_t	fw_size;
} __attribute__((packed));

extern struct boardinfo board_info;

extern void jump_to_app(void);
extern void bootloader(unsigned timeout);
extern void delay(unsigned msec);

#define BL_WAIT_MAGIC	0x19710317		/* magic number in PWR regs to wait in bootloader */

/* generic timers */
#define NTIMERS		4
#define TIMER_BL_WAIT	0
#define TIMER_CIN	1
#define TIMER_LED	2
#define TIMER_DELAY	3

/* generic receive buffer for async reads */
void usb_buf_put(uint8_t b);
void uart_buf_put(uint8_t b);
extern int usb_buf_get(void);
extern int uart_buf_get(void);
/*****************************************************************************
 * Chip/board functions.
 */

/* LEDs */
#define LED_ACTIVITY	1
#define LED_BOOTLOADER	2

#ifdef BOOT_DELAY_ADDRESS
# define BOOT_DELAY_SIGNATURE1	0x92c2ecff
# define BOOT_DELAY_SIGNATURE2	0xc5057d5d
# define BOOT_DELAY_MAX		30
#endif

#define MAX_DES_LENGTH 20

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

/* flash helpers from main_*.c */
extern void board_deinit(void);
extern uint32_t board_get_devices(void);
extern void clock_deinit(void);
extern uint32_t flash_func_sector_size(unsigned sector);
extern void flash_func_erase_sector(unsigned sector);
uint32_t flash_func_read_word(uint32_t address);
extern uint32_t flash_func_read_otp(uint32_t address);
extern uint32_t flash_func_read_sn(uint32_t address);

extern uint32_t get_mcu_id(void);
int get_mcu_desc(int max, uint8_t *revstr);
extern int check_silicon(void);
extern int8_t CDC_Transmit_FS(uint8_t * Buf, uint16_t Len);

// bootloader flash update protocol.
//
// Command format:
//
//      <opcode>[<command_data>]<EOC>
//
// Reply format:
//
//      [<reply_data>]<INSYNC><status>
//
// The <opcode> and <status> values come from the PROTO_ defines below,
// the <*_data> fields is described only for opcodes that transfer data;
// in all other cases the field is omitted.
//
// Expected workflow (protocol 3) is:
//
// GET_SYNC		verify that the board is present
// GET_DEVICE		determine which board (select firmware to upload)
// CHIP_ERASE		erase the program area and reset address counter
// loop:
//      PROG_MULTI      program bytes
// GET_CRC		verify CRC of entire flashable area
// RESET		finalise flash programming, reset chip and starts application
//

#define BL_PROTOCOL_VERSION 		5		// The revision of the bootloader protocol
// protocol bytes
#define PROTO_INSYNC				0x12    // 'in sync' byte sent before status
#define PROTO_EOC					0x20    // end of command

// Reply bytes
#define PROTO_OK					0x10    // INSYNC/OK      - 'ok' response
#define PROTO_FAILED				0x11    // INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID				0x13	// INSYNC/INVALID - 'invalid' response for bad commands
#define PROTO_BAD_SILICON_REV 		0x14 	// On the F4 series there is an issue with < Rev 3 silicon
// see https://pixhawk.org/help/errata
// Command bytes
#define PROTO_GET_SYNC				0x21    // NOP for re-establishing sync
#define PROTO_GET_DEVICE			0x22    // get device ID bytes
#define PROTO_CHIP_ERASE			0x23    // erase program area and reset program address
#define PROTO_PROG_MULTI			0x27    // write bytes at program address and increment
#define PROTO_GET_CRC				0x29	// compute & return a CRC
#define PROTO_GET_OTP				0x2a	// read a byte from OTP at the given address
#define PROTO_GET_SN				0x2b    // read a word from UDID area ( Serial)  at the given address
#define PROTO_GET_CHIP				0x2c    // read chip version (MCU IDCODE)
#define PROTO_SET_DELAY				0x2d    // set minimum boot delay
#define PROTO_GET_CHIP_DES			0x2e    // read chip version In ASCII
#define PROTO_BOOT					0x30    // boot the application
#define PROTO_DEBUG					0x31    // emit debug information - format not defined
#define PROTO_SET_BAUD				0x33    // set baud rate on uart

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV	1	// bootloader revision
#define PROTO_DEVICE_BOARD_ID	2	// board ID
#define PROTO_DEVICE_BOARD_REV	3	// board revision
#define PROTO_DEVICE_FW_SIZE	4	// size of flashable area
#define PROTO_DEVICE_VEC_AREA	5	// contents of reserved vectors 7-10

static uint8_t bl_type;
static uint8_t last_input;
/*****************************************************************************
 * Interface in/output.
 */

extern void cinit(void *config, uint8_t interface);
extern void cfini(void);
extern int cin(uint32_t devices);
extern void cout(uint8_t *buf, unsigned len);

static const uint32_t	bl_proto_rev = BL_PROTOCOL_VERSION;	// value returned by PROTO_DEVICE_BL_REV

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.

/* State of an inserted USB cable */
static uint8_t usb_connected = 0;

/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Exported_Types LOW LEVEL Exported Types
  * @{
  */
typedef enum
{
#if defined(USE_IOEXPANDER)
LED1 = 0,
LED_GREEN = LED1,
LED2 = 1,
LED_ORANGE = LED2,
LED3 = 2,
LED_RED = LED3,
LED4 = 3,
LED_BLUE = LED4
#else
LED1 = 0,
LED_GREEN = LED1,
LED3 = 1,
LED_RED = LED3,
#endif /* USE_IOEXPANDER */
}Led_TypeDef;

#if defined(USE_IOEXPANDER)
typedef enum
{
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1
}JOYMode_TypeDef;

typedef enum
{
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
}JOYState_TypeDef;
#endif /* USE_IOEXPANDER */

typedef enum
{
  COM1 = 0,
  COM2 = 1
}COM_TypeDef;
/**
  * @}
  */

/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Exported_Constants LOW LEVEL Exported Constants
  * @{
  */

/**
  * @brief  Define for STM32H743I_EVAL board
  */
#if !defined (USE_STM32H743I_EVAL)
 #define USE_STM32H743I_EVAL
#endif

/** @addtogroup STM32H743I_EVAL_LOW_LEVEL_LED
  * @{
  */
#if defined(USE_IOEXPANDER)
#define LEDn                             ((uint32_t)4)
#define LED2_PIN                         IO_PIN_10
#define LED4_PIN                         IO_PIN_11
#else
#define LEDn                             ((uint8_t)2)
#endif /* USE_IOEXPANDER */
/*
#define LED1_GPIO_PORT                   GPIOF
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()
#define LED1_PIN                         GPIO_PIN_10
*/
#define LED1_GPIO_PORT                   GPIOE
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()
#define LED1_PIN                         GPIO_PIN_12

#define LED3_GPIO_PORT                   GPIOB
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()
#define LED3_PIN                         GPIO_PIN_4

/**
  * @}
  */

/**
  * @brief MFX_IRQOUt pin
  */
#define MFX_IRQOUT_PIN                    GPIO_PIN_8
#define MFX_IRQOUT_GPIO_PORT              GPIOI
#define MFX_IRQOUT_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOI_CLK_ENABLE()
#define MFX_IRQOUT_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOI_CLK_DISABLE()
#define MFX_IRQOUT_EXTI_IRQn              EXTI9_5_IRQn


/** @addtogroup STM32H743I_EVAL_LOW_LEVEL_COM
  * @{
  */
#define COMn                             ((uint8_t)1)

/**
 * @brief Definition for COM port1, connected to USART1
 */
#define EVAL_COM1                          USART1
#define EVAL_COM1_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define EVAL_COM1_CLK_DISABLE()            __HAL_RCC_USART1_CLK_DISABLE()

#define EVAL_COM1_TX_PIN                   GPIO_PIN_14
#define EVAL_COM1_TX_GPIO_PORT             GPIOB
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
#define EVAL_COM1_TX_AF                    GPIO_AF4_USART1

#define EVAL_COM1_RX_PIN                   GPIO_PIN_15
#define EVAL_COM1_RX_GPIO_PORT             GPIOB
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
#define EVAL_COM1_RX_AF                    GPIO_AF4_USART1

#define EVAL_COM1_IRQn                     USART1_IRQn

#define EVAL_COMx_CLK_ENABLE(__INDEX__)            do { if((__INDEX__) == COM1) EVAL_COM1_CLK_ENABLE(); } while(0)
#define EVAL_COMx_CLK_DISABLE(__INDEX__)           (((__INDEX__) == 0) ? EVAL_COM1_CLK_DISABLE() : 0)

#define EVAL_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) EVAL_COM1_TX_GPIO_CLK_ENABLE(); } while(0)
#define EVAL_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_DISABLE() : 0)

#define EVAL_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) EVAL_COM1_RX_GPIO_CLK_ENABLE(); } while(0)
#define EVAL_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_DISABLE() : 0)


#if defined(USE_IOEXPANDER)
/**
  * @brief Joystick Pins definition
  */
#define JOY_SEL_PIN                    IO_PIN_0
#define JOY_DOWN_PIN                   IO_PIN_1
#define JOY_LEFT_PIN                   IO_PIN_2
#define JOY_RIGHT_PIN                  IO_PIN_3
#define JOY_UP_PIN                     IO_PIN_4
#define JOY_NONE_PIN                   JOY_ALL_PINS
#define JOY_ALL_PINS                   (IO_PIN_0 | IO_PIN_1 | IO_PIN_2 | IO_PIN_3 | IO_PIN_4)

/**
  * @brief Eval Pins definition connected to MFX
  */
#define TS_INT_PIN                     IO_PIN_14

/**
  * @brief Eval Pins definition connected to MFX
  */

#define XSDN_PIN                       IO_PIN_10
#define SD_LDO_SEL_PIN                 IO_PIN_13
#define RSTI_PIN                       IO_PIN_11
#define CAM_PLUG_PIN                   IO_PIN_12
#define LCD_INT_PIN                    IO_PIN_14
#define AUDIO_INT_PIN                  IO_PIN_5
#define OTG_FS1_OVER_CURRENT_PIN       IO_PIN_6
#define OTG_FS1_POWER_SWITCH_PIN       IO_PIN_7
#define OTG_FS2_OVER_CURRENT_PIN       IO_PIN_8
#define OTG_FS2_POWER_SWITCH_PIN       IO_PIN_9
#define SD_DETECT_PIN                  IO_PIN_15

#endif /* USE_IOEXPANDER */


/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Exported_Macros LOW LEVEL Exported Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Exported_Functions LOW LEVEL Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *husart);
void             BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart);
#if defined(USE_IOEXPANDER)
uint8_t          BSP_JOY_Init(JOYMode_TypeDef JoyMode);
void             BSP_JOY_DeInit(void);
JOYState_TypeDef BSP_JOY_GetState(void);
uint8_t          BSP_TS3510_IsDetected(void);
#endif /* USE_IOEXPANDER */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H743I_EVAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
