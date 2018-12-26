
/* Includes ------------------------------------------------------------------ */
#include "fmu.h"
#include "main.h"
/** @addtogroup STM32H7xx_HAL_Applications
  * @{
  */
/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
};
/** @addtogroup CDC_Standalone
  * @{
  */

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
USBD_HandleTypeDef USBD_Device;
extern PCD_HandleTypeDef hpcd;
RTC_HandleTypeDef RtcHandle;

/* magic numbers from reference manual */
typedef enum mcu_rev_e {
	MCU_REV_STM32H7_REV_Z = 0x1001,
	MCU_REV_STM32H7_REV_Y = 0x1003,
} mcu_rev_e;

typedef struct mcu_des_t {
	uint16_t mcuid;
	const char *desc;
	char  rev;
} mcu_des_t;

// The default CPU ID  of STM32_UNKNOWN is 0 and is in offset 0
// Before a rev is known it is set to ?
// There for new silicon will result in STM32F4..,?
mcu_des_t mcu_descriptions[] = {
	{ STM32_UNKNOWN,	"STM32H???",    '?'},
	{ STM32H7x3, 	    "STM32H7x3",	  '?'},
};

typedef struct mcu_rev_t {
	mcu_rev_e revid;
	char  rev;
} mcu_rev_t;

/*
 * This table is used in 2 ways. One to look look up the revision
 * of a given chip. Two to see it a revsion is in the group of "Bad"
 * silicon.
 *
 * Therefore when adding entries for good silicon rev, they must be inserted
 * at the beginning of the table. The value of FIRST_BAD_SILICON_OFFSET will
 * also need to be increased to that of the value of the first bad silicon offset.
 *
 */
const mcu_rev_t silicon_revs[] = {
	{MCU_REV_STM32H7_REV_Z, 'Z'}, /* Revision A */
	{MCU_REV_STM32H7_REV_Y, 'Y'}, /* Revision Z */
};

/* Private function prototypes ----------------------------------------------- */
static void MPU_Config(void);

static void CPU_CACHE_Enable(void);

static uint32_t board_get_rtc_signature(void);
static void board_set_rtc_signature(uint32_t sig);
/* Private functions --------------------------------------------------------- */

uint32_t flash_func_read_otp(uint32_t address)
{

	return 0;
}

uint32_t get_mcu_id(void)
{
	return *(uint32_t *)DBGMCU_IDCODE;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	uint32_t idcode = (*(uint32_t *)DBGMCU_IDCODE);
	int32_t mcuid = idcode & DEVID_MASK;
	mcu_rev_e revid = (idcode & REVID_MASK) >> 16;

	mcu_des_t des = mcu_descriptions[STM32_UNKNOWN];

	for (int i = 0; i < arraySize(mcu_descriptions); i++) {
		if (mcuid == mcu_descriptions[i].mcuid) {
			des = mcu_descriptions[i];
			break;
		}
	}

	for (int i = 0; i < arraySize(silicon_revs); i++) {
		if (silicon_revs[i].revid == revid) {
			des.rev = silicon_revs[i].rev;
		}
	}

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;

	while (strp < endp && *des.desc) {
		*strp++ = *des.desc++;
	}

	if (strp < endp) {
		*strp++ = ',';
	}

	if (strp < endp) {
		*strp++ = des.rev;
	}

	return  strp - revstr;
}


int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uint32_t address)
{
	// read a byte out from unique chip ID area
	// it's 12 bytes, or 3 words.
	return *(uint32_t *)(address + UDID_START);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint8_t try_boot = 1;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */	
  /* This project calls firstly two functions in order to configure MPU feature
  and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable()*/

  /* Configure the MPU attributes as Write Through for SDRAM*/
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

   /* STM32H7xx HAL library initialization:
     - Systick timer is configured by default as source of time base, but user
       can eventually implement his proper time base source (a general purpose
       timer for application or other time source), keeping in mind that Time base
       duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
       handled in milliseconds basis.
     - Set NVIC Group Priority to 4
     - Low Level Initialization: global MSP (MCU Support Package) initialization
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  board_init();
  
	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {
		/*Don't even try to boot before dropping to the bootloader. */
		try_boot = 0;
		/*Don't drop out of the bootloader until something has been uploaded. */
		timeout = 0;
		/*Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.		 */
		board_set_rtc_signature(0);
	}

#ifdef BOOT_DELAY_ADDRESS
	{
		/*
		  if a boot delay signature is present then delay the boot
		  by at least that amount of time in seconds. This allows
		  for an opportunity for a companion computer to load a
		  new firmware, while still booting fast by sending a BOOT
		  command
		 */
		uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
		uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

		if (sig2 == BOOT_DELAY_SIGNATURE2 &&
		    (sig1 & 0xFFFFFF00) == (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00)) {
			unsigned boot_delay = sig1 & 0xFF;

			if (boot_delay <= BOOT_DELAY_MAX) {
				try_boot = 0;

				if (timeout < boot_delay * 1000) {
					timeout = boot_delay * 1000;
				}
			}
		}
	}
#endif

#if INTERFACE_USB
	/*Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.	 */
	if (HAL_GPIO_ReadPin(BOARD_PORT_VBUS, BOARD_PIN_VBUS) != GPIO_PIN_RESET) {
		usb_connected = 1;
		/* don't try booting before we set up the bootloader */
		try_boot = 0;
	}
#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app();

		// If it failed to boot, reset the boot signature and stay in bootloader
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}
	
#if INTERFACE_USART
	PX4_uart_Init();
#endif
	
	/* start the interface */
#if INTERFACE_USB
	//cinit(BOARD_INTERFACE_CONFIG_USB, USB);
	
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  /* Start Device Process */
  USBD_Start(&USBD_Device);

  HAL_PWREx_EnableUSBVoltageDetector();	
#endif

	BSP_LED_On(LED_GREEN);
 // BSP_LED_Init(LED_ORANGE);
 // BSP_LED_Init(LED_RED);
  //BSP_LED_Init(LED_BLUE);

	while(1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}


uint32_t board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}
	return devices;
}

static void board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;
  /* Configure LED1, LED2, LED3 and LED4 */
  BSP_LED_Init(LED_GREEN);

 /*##-1- Configure the RTC peripheral #######################################*/
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
      - Hour Format    = Format 12
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
  __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);
  RtcHandle.Instance            = RTC;
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {  
		Error_Handler();/* Initialization Error */ 
  }    
}

void board_deinit(void)
{
	BSP_LED_DeInit(LED_GREEN);	
}

static uint32_t board_get_rtc_signature(void)
{
	/* enable the backup registers */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTC_ENABLE();
	uint32_t result = HAL_RTCEx_BKUPRead(&RtcHandle,RTC_BKP_DR0);
	/* disable the backup registers */
	__HAL_RCC_RTC_DISABLE(); 
	HAL_PWR_DisableBkUpAccess();

	return result;
}

static void board_set_rtc_signature(uint32_t sig)
{
	/* enable the backup registers */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTC_ENABLE(); 
	HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,sig);
	/* disable the backup registers */
	__HAL_RCC_RTC_DISABLE(); 
	HAL_PWR_DisableBkUpAccess();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 160
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            PLL3_M                         = 25
  *            PLL3_N                         = 336
  *            PLL3_P                         = 2
  *            PLL3_Q                         = 7
  *            PLL3_R                         = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /*!< Supply configuration update enable */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  /* PLL1 for System Clock */
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
	//RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

  /* PLL3 for USB Clock */
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 336;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 7;

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
  RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  
  __HAL_RCC_CSI_ENABLE() ;//activate CSI clock mondatory for I/O Compensation Cell

  
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;// Enable SYSCFG clock mondatory for I/O Compensation Cell 

  
  HAL_EnableCompensationCell();// Enables the I/O Compensation Cell 

}

inline void cfini(void)
{
#if INTERFACE_USB
	USBD_DeInit(&USBD_Device);
#endif
#if INTERFACE_USART
	HAL_UART_DeInit(&UartHandle);
#endif
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Write Through for External SDRAM.
  * @note   The Base Address is 0xD0000000 .
  *         The Configured Region Size is 32MB because same as SDRAM size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
	
	/*making the SRAM buffers cache enabled with write-through policy*/
	SCB->CACR|=1<<2;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
   * number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
   * * line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
