
/* Includes */
#include "main.h"
#include "stm32f3xx_it.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>


/* Defines */
#define DELAY_BEFORE_EXECUTING 900000
#define DELAY_AFTER_WATERING 180000
#define DELAY_AFTER_LOOP 7200000
#define ALARM_LIGHTS_DELAY 500


/* Define Variables */
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;


/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void Humidify(void);
static int Convert_to_percents(int, int, int, int, int);
static int Read_From_Sensor(void);
static void Flash_LED(void);
static void Ext_Delay(void);


/* Code */

/* State machine declaration */
struct State;
typedef void state_fn(struct State *);

/* System operating mods:
 * Provides different watering time.
 */
typedef enum
{
	LOW,
	MEDIUM,
	HIGH,
	ULTRA,
} Mode;

/* Watering time for different mods */
int Mode_Delay[4] = { 2000, 5000, 10000, 15000 };

/* State machine implementation. Idea of state machine:
 * Three working states with precise logic segregation.
 * Idle like main working state, Watering for watering case only and
 * Panic for non system-responsible cases, e.g lack of water in tank
 * In main loop execute function which corresponds to the current state
 * In every state function change current state by conditions
 */

struct State
{
    state_fn * next; // Pointer to the next state
    int cnt_unsucc_waterings;
};

/* Declare three state functions. */
static state_fn Watering, Idle, Panic;

Mode current_mode = LOW;


/* The application entry point. */
int main(void)
{
  /* Controller Configuration */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();

  /* Initializing starting state */
  struct State state = { Idle, 0 };

  /* Delay before program executing.
   * Goal: give time to setting system before first watering
   */
  HAL_Delay(DELAY_BEFORE_EXECUTING);

  /* Main infinite loop */
  while (1)
  {

	  /* Execute current state ("Idle" for first time) */
	  state.next(&state);
  }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ADC1 Initialization Function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 Initialization Function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 Initialization Function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 Initialization Function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * USB Initialization Function */
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }

}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* Read sensor data from analog port */
static int Read_From_Sensor(void)
{
	uint16_t raw;
	int percentage;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc1);

	percentage = Convert_to_percents(raw, 1230, 3140, 0, 100);

	return percentage;
}

/* convert analog port output (1230-3140) to percents (1-100)*/
static int Convert_to_percents(int input, int input_min, int input_max, int output_min, int output_max)
{
	return ((((input - input_min)*(output_max - output_min))/(input_max - input_min)) + output_min);
}

/* Watering state function for watering control */
static void Watering(struct State * state)
{
    int dryness_before = Read_From_Sensor();
    Humidify();

    /* Wait for the water to be absorbed into the ground.
     * After, reads moisture data again */
    HAL_Delay(DELAY_AFTER_WATERING);
    int dryness_after = Read_From_Sensor();

    /* If moisture change after watering, reset unsuccessful watering counter
     * Else increment counter */
    if (abs(dryness_after - dryness_before) >= 10)
    	state->cnt_unsucc_waterings = 0;
    else
    	state->cnt_unsucc_waterings++;

    /* Set up next state to Idle */
    state->next = Idle;
}

/* Idle state function for system control logic */
static void Idle(struct State * state)
{

    // char msg[10]; *Debug info too
    int dryness = Read_From_Sensor();

   /*
    * Debug info, uncomment (with previous commented string) for display moisture values
    * sprintf(msg, "%hu%%\r\n", dryness);
    * HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
    */

    if (dryness > 87)
    	/*
    	 * Set up state to Panic if counter of unsuccessful watering == 2
    	 * Else set up state to Watering
    	 */
        state->next = state->cnt_unsucc_waterings >= 2 ? Panic : Watering;
    else
    	/*
    	 * If watering is not needed, sets program to sleep
    	 * Unobvious forth state Sleep
    	 */
        HAL_Delay(DELAY_AFTER_LOOP); // set proper value
}


/* Panic state function for system work troubles */
static void Panic(struct State * state)
{
	/* Flashing LED for certain time,
	 * Set up Watering state and try to make watering
	 */
	Flash_LED();
	state->next = Watering;
}

/* Function for pure watering.
 * Watering state function it's something like watering handler,
 * while Humidify responsible for watering only
 */
static void Humidify(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	HAL_GPIO_TogglePin (GPIOE, LD7_Pin);

	HAL_Delay(Mode_Delay[current_mode]);

	HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	HAL_GPIO_TogglePin (GPIOE, LD7_Pin);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

}

/* Flashing red LEDs for take user attention to system usage problem */
static void Flash_LED(void)
{
	int repeat = 0;

	while(repeat++ < 3600) //3600 (every hour)
	{
		HAL_GPIO_TogglePin (GPIOE, LD3_Pin);
		HAL_GPIO_TogglePin (GPIOE, LD10_Pin);
		HAL_Delay(ALARM_LIGHTS_DELAY);
		HAL_GPIO_TogglePin (GPIOE, LD3_Pin);
		HAL_GPIO_TogglePin (GPIOE, LD10_Pin);
		HAL_Delay(ALARM_LIGHTS_DELAY);
	}
}

/* Custom delay to use inside interrupt handlers */
static void Ext_Delay(void)
{
	int c;
	for (c = 1; c <= 500000; c++)
	{}
}

/* Handle user button interrupt */
void EXTI0_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  current_mode = (current_mode + 1) % 4;

  /* Visualize mode changing by blinking LEDs */
  switch(current_mode)
  {
  case LOW:
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  Ext_Delay();
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  break;

  case MEDIUM:
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD9_Pin);
	  Ext_Delay();
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD9_Pin);
	  break;

  case HIGH:
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD9_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD8_Pin);
	  Ext_Delay();
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD9_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD8_Pin);
	  break;

  case ULTRA:
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD9_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD8_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD10_Pin);
	  Ext_Delay();
	  HAL_GPIO_TogglePin (GPIOE, LD6_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD9_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD8_Pin);
	  HAL_GPIO_TogglePin (GPIOE, LD10_Pin);
	  break;

  default:
	  break;
  }
}

/* This function is executed in case of error occurrence. */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {

  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
