/* Includes ------------------------------------------------------------------*/
#include "fmu.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32H743I_EVAL
  * @{
  */

/** @addtogroup STM32H743I_EVAL_LOW_LEVEL
  * @{
  */

//extern volatile unsigned timer[NTIMERS];	/* each timer decrements every millisecond if > 0 */
volatile unsigned timer[NTIMERS];
/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Private_TypesDefinitions STM32H743I-EVAL LOW LEVEL Private Types Definitions
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Private_Defines STM32H743I-EVAL LOW LEVEL Private Defines
  * @{
  */
unsigned uart_head, uart_tail;
uint8_t uart_rx_buf[256];// UART_DATA_ALIGN;
/**
 * @brief STM32H743I EVAL BSP Driver version number V1.2.0
   */
#define __STM32H743I_EVAL_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32H743I_EVAL_BSP_VERSION_SUB1   (0x02) /*!< [23:16] sub1 version */
#define __STM32H743I_EVAL_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32H743I_EVAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32H743I_EVAL_BSP_VERSION         ((__STM32H743I_EVAL_BSP_VERSION_MAIN << 24)\
                                             |(__STM32H743I_EVAL_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32H743I_EVAL_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32H743I_EVAL_BSP_VERSION_RC))
/**
  * @}
  */

/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Private_Macros STM32H743I-EVAL LOW LEVEL Private Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32H743I_EVAL_LOW_LEVEL_Private_Variables STM32H743I-EVAL LOW LEVEL Private Variables
  * @{
  */

#if defined(USE_IOEXPANDER)

const uint32_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN};
#else

const uint32_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED3_PIN};
#endif /* USE_IOEXPANDER */


USART_TypeDef* COM_USART[COMn] = {EVAL_COM1};

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN};

const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF};

const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF};

#if defined(USE_IOEXPANDER)
/* IOExpander IO functions */
void            IOE_Init(void);
void            IOE_ITConfig(void);
void            IOE_Delay(uint32_t Delay);
void            IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t        IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void            IOE_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

/* MFX IO functions */
void            MFX_IO_Init(void);
void            MFX_IO_DeInit(void);
void            MFX_IO_ITConfig(void);
void            MFX_IO_Delay(uint32_t Delay);
void            MFX_IO_Write(uint16_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         MFX_IO_Read(uint16_t Addr, uint8_t Reg);
uint16_t        MFX_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void            MFX_IO_Wakeup(void);
void            MFX_IO_EnableWakeupPin(void);
#endif /* USE_IOEXPANDER */

  /**
  * @brief  This method returns the STM32H743I EVAL BSP Driver revision
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32H743I_EVAL_BSP_VERSION;
}

/**
  * @brief  Configures LED on GPIO and/or on MFX.
  * @param  Led: LED to be configured.
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  /* On RevB board, LED1 and LED3 are connected to GPIOs */
  GPIO_InitTypeDef  gpio_initstruct;
  GPIO_TypeDef*     gpio_led;

  if ((Led == LED1) || (Led == LED3))
  {
    if (Led == LED1)
    {
      gpio_led = LED1_GPIO_PORT;
      /* Enable the GPIO_LED clock */
      LED1_GPIO_CLK_ENABLE();
    }
    else
    {
      gpio_led = LED3_GPIO_PORT;
      /* Enable the GPIO_LED clock */
      LED3_GPIO_CLK_ENABLE();
    }

    /* Configure the GPIO_LED pin */
    gpio_initstruct.Pin = GPIO_PIN[Led];
		
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initstruct.Pull = GPIO_PULLUP;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

/*
		gpio_initstruct.Mode = GPIO_MODE_INPUT;
    gpio_initstruct.Pull = GPIO_NOPULL;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
*/		
    HAL_GPIO_Init(gpio_led, &gpio_initstruct);

    /* By default, turn off LED */
    HAL_GPIO_WritePin(gpio_led, GPIO_PIN[Led], GPIO_PIN_SET);
  }
  else
  {
#if defined(USE_IOEXPANDER)
  /* On RevB and above eval board, LED2 and LED4 are connected to MFX */
  BSP_IO_Init();  /* Initialize MFX */
  BSP_IO_ConfigPin(GPIO_PIN[Led], IO_MODE_OUTPUT_PP_PU);
  BSP_IO_WritePin(GPIO_PIN[Led], BSP_IO_PIN_SET);
#endif  /* USE_IOEXPANDER */

  }
}


/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be configured.
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_led;

  /* On RevB led1 and Led3 are on GPIO while Led2 and Led4 on Mfx*/
  if ((Led == LED1) || (Led == LED3))
  {
    if (Led == LED1)
    {
      gpio_led = LED1_GPIO_PORT;
    }
    else
    {
      gpio_led = LED3_GPIO_PORT;
    }
    /* Turn off LED */
    HAL_GPIO_WritePin(gpio_led, GPIO_PIN[Led], GPIO_PIN_RESET);
    /* Configure the GPIO_LED pin */
    gpio_init_structure.Pin = GPIO_PIN[Led];
    HAL_GPIO_DeInit(gpio_led, gpio_init_structure.Pin);
  }
  else
  {
#if defined(USE_IOEXPANDER)   /* (USE_IOEXPANDER always defined for RevA) */
    /* GPIO_PIN[Led]  depends on the board revision: */
    /*  - in case of RevA all leds are deinit  */
    /*  - in case of RevB just led 2 and led4 are deinit */
    BSP_IO_ConfigPin(GPIO_PIN[Led], IO_MODE_OFF);
#endif /* USE_IOEXPANDER */
  }
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{

  if ((Led == LED1) || (Led == LED3)) /* Switch On LED connected to GPIO */
  {
    if (Led == LED1)
    {
      HAL_GPIO_WritePin(LED1_GPIO_PORT, GPIO_PIN[Led], GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(LED3_GPIO_PORT, GPIO_PIN[Led], GPIO_PIN_RESET);
    }
  }
  else
  {
#if defined(USE_IOEXPANDER)            /* Switch On LED connected to MFX */
      BSP_IO_WritePin(GPIO_PIN[Led], BSP_IO_PIN_RESET);
#endif /* USE_IOEXPANDER */
  }
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  if ((Led == LED1) || (Led == LED3)) /* Switch Off LED connected to GPIO */
  {
    if (Led == LED1)
    {
      HAL_GPIO_WritePin(LED1_GPIO_PORT, GPIO_PIN[Led], GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(LED3_GPIO_PORT, GPIO_PIN[Led], GPIO_PIN_SET);
    }
  }
  else
  {
#if defined(USE_IOEXPANDER)            /* Switch Off LED connected to MFX */
      BSP_IO_WritePin(GPIO_PIN[Led], BSP_IO_PIN_SET);
#endif /* USE_IOEXPANDER */
  }
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  if ((Led == LED1) || (Led == LED3)) /* Toggle LED connected to GPIO */
  {
    if (Led == LED1)
    {
      HAL_GPIO_TogglePin(LED1_GPIO_PORT, GPIO_PIN[Led]);
    }
    else
    {
      HAL_GPIO_TogglePin(LED3_GPIO_PORT, GPIO_PIN[Led]);
    }
  }
  else
  {
#if defined(USE_IOEXPANDER)            /* Toggle LED connected to MFX */
      BSP_IO_TogglePin(GPIO_PIN[Led]);
#endif /* USE_IOEXPANDER */
  }
}

/**
  * @brief  Configures COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1
  *            @arg  COM2
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Enable GPIO clock */
  EVAL_COMx_TX_GPIO_CLK_ENABLE(COM);
  EVAL_COMx_RX_GPIO_CLK_ENABLE(COM);

  /* Enable USART clock */
  EVAL_COMx_CLK_ENABLE(COM);

  /* Configure USART Tx as alternate function */
  gpio_init_structure.Pin = COM_TX_PIN[COM];
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Alternate = COM_TX_AF[COM];
  HAL_GPIO_Init(COM_TX_PORT[COM], &gpio_init_structure);

  /* Configure USART Rx as alternate function */
  gpio_init_structure.Pin = COM_RX_PIN[COM];
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Alternate = COM_RX_AF[COM];
  HAL_GPIO_Init(COM_RX_PORT[COM], &gpio_init_structure);

  /* USART configuration */
  huart->Instance = COM_USART[COM];
  HAL_UART_Init(huart);
}

/**
  * @brief  DeInit COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1
  *            @arg  COM2
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart)
{
  /* USART configuration */
  huart->Instance = COM_USART[COM];
  HAL_UART_DeInit(huart);

  /* Enable USART clock */
  EVAL_COMx_CLK_DISABLE(COM);

  /* DeInit GPIO pins can be done in the application
     (by surcharging this __weak function) */

  /* GPIO pins clock, DMA clock can be shut down in the application
     by surcharging this __weak function */
}

uint32_t flash_func_read_word(uint32_t address)
{
	if (address & 3) {
		return 0;
	}
	uint32_t data = *(uint32_t *)(address + APP_LOAD_ADDRESS);
	__DSB();
	return data;
}
uint32_t flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		return flash_sectors[sector].size;
	}

	return 0;
}

void flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS || sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	/* Caculate the logical base address of the sector
	 * flash_func_read_word will add APP_LOAD_ADDRESS	 */
	uint32_t address = 0;

	for (unsigned i = BOARD_FIRST_FLASH_SECTOR_TO_ERASE; i < sector; i++) {
		address += flash_func_sector_size(i);
	}

	/* blank-check the sector */
	unsigned size = flash_func_sector_size(sector);
	uint8_t blank = 1;

	for (unsigned i = 0; i < size; i += sizeof(uint32_t)) {
		if (flash_func_read_word(address + i) != 0xffffffff) {
			blank = 0;
			break;
		}
	}

	/* erase the sector if it failed the blank check */
	if (!blank) {
		//flash_erase_sector(flash_sectors[sector].sector_number, FLASH_CR_PROGRAM_X32);
		if(sector<7)
		{ FLASH_Erase_Sector(flash_sectors[sector].sector_number, FLASH_BANK_1, FLASH_VOLTAGE_RANGE_3); }
		else
		{ FLASH_Erase_Sector(flash_sectors[sector].sector_number-8, FLASH_BANK_2, FLASH_VOLTAGE_RANGE_3); }
	}
}

inline int cin(uint32_t devices)
{
#if INTERFACE_USB
	if((bl_type == NONE || bl_type == USB) && (devices & USB0_DEV) != 0) {
		int usb_in = usb_buf_get();

		if (usb_in >= 0) {
			last_input = USB;
			return usb_in;
		}
	}
#endif

#if INTERFACE_USART
	if ((bl_type == NONE || bl_type == USART) && (devices & SERIAL0_DEV) != 0) {
		int	uart_in = uart_buf_get();

		if (uart_in >= 0) {
			last_input = USART;
			return uart_in;
		}
	}
#endif

	return -1;
}

inline void cout(uint8_t *buf, unsigned len)
{
#if INTERFACE_USB

	if (bl_type == USB) {
		CDC_Transmit_FS(buf,len);
	}

#endif
#if INTERFACE_USART

	if (bl_type == USART) {
		HAL_UART_Transmit_DMA(&UartHandle, buf, len);
	}

#endif
}

#if INTERFACE_USB
void usb_buf_put(uint8_t b)
{
	unsigned usb_next = (usb_head + 1) % sizeof(usb_rx_buf);

	if (usb_next != usb_tail) {
		usb_rx_buf[usb_head] = b;
		usb_head = usb_next;
	}
}
#endif

#if INTERFACE_USART
void uart_buf_put(uint8_t b)
{

	unsigned uart_next = (uart_head + 1) % sizeof(uart_rx_buf);

	if (uart_next != uart_tail) {
		uart_rx_buf[uart_head] = b;
		uart_head = uart_next;
	}
}
#endif

#if INTERFACE_USB
int usb_buf_get(void)
{
	int	ret = -1;

	if (usb_tail != usb_head) {
		ret = usb_rx_buf[usb_tail];
		usb_tail = (usb_tail + 1) % sizeof(usb_rx_buf);
	}
	return ret;
}
#endif

#if INTERFACE_USART
int uart_buf_get(void)
{
	int	ret = -1;

	if (uart_tail != uart_head) {
		ret = uart_rx_buf[uart_tail];	
		uart_tail = (uart_tail + 1) % sizeof(uart_rx_buf);
	}
	return ret;
}
#endif

__asm void do_jump(uint32_t stacktop, uint32_t entrypoint)
{    
	msr msp, r0
	bx	r1
}

void jump_to_app(void)
{
	const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;

	/*
	 * We refuse to program the first word of the app until the upload is marked
	 * complete by the host.  So if it's not 0xffffffff, we should try booting it.
	 */
	if (app_base[0] == 0xffffffff) {
		return;
	}

	/*
	 * The second word of the app is the entrypoint; it must point within the
	 * flash area (or we have a bad flash).
	 */
	if (app_base[1] < APP_LOAD_ADDRESS) {
		return;
	}

	if (app_base[1] >= (APP_LOAD_ADDRESS + board_info.fw_size)) {
		return;
	}

	/* just for paranoia's sake */
	HAL_FLASH_Lock();

	/* kill the systick interrupt */
	//systick_interrupt_disable();
	//systick_counter_disable();

	/* deinitialise the interface */
	cfini();

	/* reset the clock */
	HAL_RCC_DeInit();

	/* deinitialise the board */
	board_deinit();

	/* switch exception handlers to the application */
	SCB->VTOR = APP_LOAD_ADDRESS;

	/* extract the stack and entrypoint from the app vector table and go */
	do_jump(app_base[0], app_base[1]);
	// just to keep noreturn happy
	for (;;) ;
}

/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
	unsigned i;
	
  HAL_IncTick();

	for (i = 0; i < NTIMERS; i++)
		if (timer[i] > 0) {
			timer[i]--;
		}

	if ((_led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
		BSP_LED_Toggle(LED1);
		timer[TIMER_LED] = 50;
	}
}

void delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;

	while (timer[TIMER_DELAY] > 0)
		;
}

static void led_set(enum led_state state)
{
	_led_state = state;

	switch (state) {
	case LED_OFF:
		BSP_LED_Off(LED1);
		break;

	case LED_ON:
		BSP_LED_On(LED1);
		break;

	case LED_BLINK:
		/* restart the blink state machine ASAP */
		timer[TIMER_LED] = 0;
		break;
	}
}

static void sync_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_OK	// "OK"
	};

	cout(data, sizeof(data));
}

static void invalid_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_INVALID	// "invalid command"
	};

	cout(data, sizeof(data));
}

static void failure_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_FAILED	// "command failed"
	};

	cout(data, sizeof(data));
}

static volatile unsigned cin_count;

static int cin_wait(unsigned timeout)
{
	int c = -1;

	/* start the timeout */
	timer[TIMER_CIN] = timeout;

	do {
		c = cin(board_get_devices());

		if (c >= 0) {
			cin_count++;
			break;
		}

	} while (timer[TIMER_CIN] > 0);

	return c;
}

/**
 * Function to wait for EOC
 *
 * @param timeout length of time in ms to wait for the EOC to be received
 * @return true if the EOC is returned within the timeout perio, else false
 */
inline static uint8_t wait_for_eoc(unsigned timeout)
{
	return cin_wait(timeout) == PROTO_EOC;
}

static void cout_word(uint32_t val)
{
	cout((uint8_t *)&val, 4);
}

static int cin_word(uint32_t *wp, unsigned timeout)
{
	union {
		uint32_t w;
		uint8_t b[4];
	} u;

	for (unsigned i = 0; i < 4; i++) {
		int c = cin_wait(timeout);

		if (c < 0) {
			return c;
		}

		u.b[i] = c & 0xff;
	}

	*wp = u.w;
	return 0;
}

static uint32_t crc32(const uint8_t *src, unsigned len, unsigned state)
{
	static uint32_t crctab[256];

	/* check whether we have generated the CRC table yet */
	/* this is much smaller than a static table */
	if (crctab[1] == 0) {
		for (unsigned i = 0; i < 256; i++) {
			uint32_t c = i;

			for (unsigned j = 0; j < 8; j++) {
				if (c & 1) {
					c = 0xedb88320U ^ (c >> 1);

				} else {
					c = c >> 1;
				}
			}

			crctab[i] = c;
		}
	}

	for (unsigned i = 0; i < len; i++) {
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	}

	return state;
}

void bootloader(unsigned timeout)
{
	bl_type = NONE; // The type of the bootloader, whether loading from USB or USART, will be determined by on what port the bootloader recevies its first valid command.

	uint32_t	address = board_info.fw_size;	/* force erase before upload will work */
	uint32_t	first_word = 0xffffffff;

	/* (re)start the timer system */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	
	//systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	//systick_interrupt_enable();
	//systick_counter_enable();

	/* if we are working with a timeout, start it running */
	if (timeout) {
		timer[TIMER_BL_WAIT] = timeout;
	}

	/* make the LED blink while we are idle */
	led_set(LED_BLINK);

	while(1) {
		volatile int c;
		int arg;
		static union {
			uint8_t		c[256];
			uint32_t	w[64];
			uint64_t	dw[32];
		} flash_buffer;

		do {
			/* if we have a timeout and the timer has expired, return now */
			if (timeout && !timer[TIMER_BL_WAIT]) {
				return;
			}
			/* try to get a byte from the host */
			c = cin_wait(0);

		} while (c < 0);

		// handle the command byte
		switch (c) {

		// sync
		//
		// command:		GET_SYNC/EOC
		// reply:		INSYNC/OK
		//
		case PROTO_GET_SYNC:

			/* expect EOC */
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

			break;

		// get device info
		//
		// command:		GET_DEVICE/<arg:1>/EOC
		// BL_REV reply:	<revision:4>/INSYNC/EOC
		// BOARD_ID reply:	<board type:4>/INSYNC/EOC
		// BOARD_REV reply:	<board rev:4>/INSYNC/EOC
		// FW_SIZE reply:	<firmware size:4>/INSYNC/EOC
		// VEC_AREA reply	<vectors 7-10:16>/INSYNC/EOC
		// bad arg reply:	INSYNC/INVALID
		//
		case PROTO_GET_DEVICE:
			/* expect arg then EOC */
			arg = cin_wait(1000);

			if (arg < 0) {
				goto cmd_bad;
			}

			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

			switch (arg) {
			case PROTO_DEVICE_BL_REV:
				cout((uint8_t *)&bl_proto_rev, sizeof(bl_proto_rev));
				break;

			case PROTO_DEVICE_BOARD_ID:
				cout((uint8_t *)&board_info.board_type, sizeof(board_info.board_type));
				break;

			case PROTO_DEVICE_BOARD_REV:
				cout((uint8_t *)&board_info.board_rev, sizeof(board_info.board_rev));
				break;

			case PROTO_DEVICE_FW_SIZE:
				cout((uint8_t *)&board_info.fw_size, sizeof(board_info.fw_size));
				break;

			case PROTO_DEVICE_VEC_AREA:
				for (unsigned p = 7; p <= 10; p++) {
					uint32_t bytes = flash_func_read_word(p * 4);

					cout((uint8_t *)&bytes, sizeof(bytes));
				}

				break;

			default:
				goto cmd_bad;
			}

			break;

		// erase and prepare for programming
		// command:		ERASE/EOC
		// success reply:	INSYNC/OK
		// erase failure:	INSYNC/FAILURE
		case PROTO_CHIP_ERASE:

			/* expect EOC */
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

			// clear the bootloader LED while erasing - it stops blinking at random
			// and that's confusing
			led_set(LED_ON);
			// erase all sectors
			HAL_FLASH_Unlock();

			for (int i = 0; flash_func_sector_size(i) != 0; i++) {
				flash_func_erase_sector(i);
			}

			// enable the LED while verifying the erase
			led_set(LED_OFF);
			// verify the erase
			for (address = 0; address < board_info.fw_size; address += 4)
				if (flash_func_read_word(address) != 0xffffffff) {
					address = flash_func_read_word(address);
					goto cmd_fail;
				}

			address = 0;
			FLASH_WaitForLastOperation(1000, FLASH_BANK_1);
			FLASH_WaitForLastOperation(1000, FLASH_BANK_2);
			// resume blinking
			led_set(LED_BLINK);
			break;

		// program bytes at current address
		// command:		PROG_MULTI/<len:1>/<data:len>/EOC
		// success reply:	INSYNC/OK
		// invalid reply:	INSYNC/INVALID
		// readback failure:	INSYNC/FAILURE
		case PROTO_PROG_MULTI:		// program bytes
			// expect count
			arg = cin_wait(50);

			if (arg < 0) {
				goto cmd_bad;
			}
			// sanity-check arguments
			if (arg % 4) {
				goto cmd_bad;
			}

			if ((address + arg) > board_info.fw_size) {
				goto cmd_bad;
			}

			if (arg > sizeof(flash_buffer.c)) {
				goto cmd_bad;
			}

			for (int i = 0; i < arg; i++) {
				c = cin_wait(1000);

				if (c < 0) {
					goto cmd_bad;
				}
				flash_buffer.c[i] = c;
			}

			if (!wait_for_eoc(200)) {
				goto cmd_bad;
			}
			if (address == 0) {
				// save the first word and don't program it until everything else is done
				first_word = flash_buffer.w[0];
				// replace first word with bits we can overwrite later
				flash_buffer.w[0] = 0xffffffff;
				delay(2);
			}
			if((arg%32)==0)
			{ arg /= 32; }
			else
			{ 
				uint8_t temp = 32-(arg%32);
				for (int i = 0; i < temp; i++)
				flash_buffer.c[arg+i] = 0xFF;
				arg = arg/32 + 1;				
			}

			for (int i = 0; i < arg; i++) {
				
				// program the word
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, APP_LOAD_ADDRESS+address, (uint64_t)(&flash_buffer.dw[i*4]))==HAL_ERROR)
				{
					//delay(2);
					goto cmd_fail;
				}
				address += 32;
			}

			break;

		// fetch CRC of the entire flash area
		// command:			GET_CRC/EOC
		// reply:			<crc:4>/INSYNC/OK
		case PROTO_GET_CRC:

			// expect EOC
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}
			// compute CRC of the programmed area
			uint32_t sum = 0;
			for (unsigned p = 0; p < board_info.fw_size; p += 4) {
				uint32_t bytes;

				if ((p == 0) && (first_word != 0xffffffff)) {
					bytes = first_word;

				} else {
					bytes = flash_func_read_word(p);
				}

				sum = crc32((uint8_t *)&bytes, sizeof(bytes), sum);
			}
			delay(2);
			cout_word(sum);
			break;

		// read a word from the OTP
		// command:			GET_OTP/<addr:4>/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_OTP:
			// expect argument
			{
				uint32_t index = 0;

				if (cin_word(&index, 100)) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(flash_func_read_otp(index));
			}
			break;

		// read the SN from the UDID
		// command:			GET_SN/<addr:4>/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_SN:
			// expect argument
			{
				uint32_t index = 0;

				if (cin_word(&index, 100)) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(flash_func_read_sn(index));
			}
			break;

		// read the chip ID code
		// command:			GET_CHIP/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_CHIP: {
				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(get_mcu_id());
			}
			break;

		// read the chip  description
		// command:			GET_CHIP_DES/EOC
		// reply:			<value:4>/INSYNC/OK
		case PROTO_GET_CHIP_DES: {
				uint8_t buffer[MAX_DES_LENGTH];
				unsigned len = MAX_DES_LENGTH;

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				len = get_mcu_desc(len, buffer);
				cout_word(len);
				delay(2);
				cout(buffer, len);
			}
			break;

#ifdef BOOT_DELAY_ADDRESS

		case PROTO_SET_DELAY: {
				/*
				  Allow for the bootloader to setup a
				  boot delay signature which tells the
				  board to delay for at least a
				  specified number of seconds on boot.
				 */
				int v = cin_wait(100);

				if (v < 0) {
					goto cmd_bad;
				}

				uint8_t boot_delay = v & 0xFF;

				if (boot_delay > BOOT_DELAY_MAX) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
				uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

				if (sig1 != BOOT_DELAY_SIGNATURE1 ||
				    sig2 != BOOT_DELAY_SIGNATURE2) {
					goto cmd_bad;
				}

				uint32_t value = (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00) | boot_delay;
				for (int i = 0; i < 8; i++)
				{
					flash_buffer.w[i] = flash_func_read_word(BOOT_DELAY_ADDRESS);
					flash_buffer.w[0] = value;
				}
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, BOOT_DELAY_ADDRESS, (uint64_t)(&flash_buffer.dw[0]))==HAL_ERROR)
				{
					goto cmd_fail;
				}				
				if (flash_func_read_word(BOOT_DELAY_ADDRESS) != value) {
					goto cmd_fail;
				}
			}
			break;
#endif

		// finalise programming and boot the system
		// command:			BOOT/EOC
		// reply:			INSYNC/OK
		case PROTO_BOOT:

			// expect EOC
			if (!wait_for_eoc(1000)) {
				goto cmd_bad;
			}
			// program the deferred first word
			if (first_word != 0xffffffff) {
				for (int i = 0; i < 8; i++)
				{
					flash_buffer.w[i] = flash_func_read_word(i*4);
					flash_buffer.w[0] = first_word;
				}
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, APP_LOAD_ADDRESS, (uint64_t)(&flash_buffer.dw[0]))==HAL_ERROR)
				{
					goto cmd_fail;
				}
				// revert in case the flash was bad...
				first_word = 0xffffffff;
			}

			// send a sync and wait for it to be collected
			sync_response();
			delay(100);

			// quiesce and jump to the app
			return;

		case PROTO_DEBUG:
			// XXX reserved for ad-hoc debugging as required
			break;

		default:
			continue;
		}

		// we got a command worth syncing, so kill the timeout because
		// we are probably talking to the uploader
		timeout = 0;

		// Set the bootloader port based on the port from which we received the first valid command
		if (bl_type == NONE) {
			bl_type = last_input;
		}
		delay(2);
		// send the sync response for this command
		sync_response();
		continue;
cmd_bad:
		// send an 'invalid' response but don't kill the timeout - could be garbage
		invalid_response();
		continue;

cmd_fail:
		// send a 'command failed' response but don't kill the timeout - could be garbage
		failure_response();
		continue;
	}
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK IOE ***********************************/
#if defined(USE_IOEXPANDER)

/**
  * @brief  Configures IOE low level interrupt.
  * @retval None
  */
void IOE_ITConfig(void)
{
  /* IO expander IT config done by BSP_TS_ITConfig function */
}
/**
  * @brief  IOE delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/********************************* LINK MFX ***********************************/

/**
  * @brief  DeInitializes MFX low level.
  * @retval None
  */
void MFX_IO_DeInit(void)
{
}

/**
  * @brief  Configures MFX low level interrupt.
  * @retval None
  */
void MFX_IO_ITConfig(void)
{
  static uint8_t mfx_io_it_enabled = 0;
  GPIO_InitTypeDef  gpio_init_structure;

  if(mfx_io_it_enabled == 0)
  {
    mfx_io_it_enabled = 1;
    /* Enable the GPIO EXTI clock */
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio_init_structure.Pin   = GPIO_PIN_8;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init_structure.Mode  = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(GPIOI, &gpio_init_structure);

    /* Enable and set GPIO EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x0F);
    HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));
  }
}

/**
  * @brief  MFX writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
void MFX_IO_Write(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_Write((uint8_t) Addr, Reg, Value);
}

/**
  * @brief  MFX reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t MFX_IO_Read(uint16_t Addr, uint8_t Reg)
{
  return I2Cx_Read((uint8_t) Addr, Reg);
}

/**
  * @brief  MFX reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t MFX_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 return I2Cx_ReadMultiple((uint8_t)Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  MFX delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void MFX_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  Used by Lx family but requested for MFX component compatibility.
  * @retval None
  */
void MFX_IO_Wakeup(void)
{
}

/**
  * @brief  Used by Lx family but requested for MXF component compatibility.
  * @retval None
  */
void MFX_IO_EnableWakeupPin(void)
{
}

#endif /* USE_IOEXPANDER */
