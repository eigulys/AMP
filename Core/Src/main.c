/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <i2c-lcd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_buf_len 1000
//#define ADC_buf_len 293
#define ADC_REF 3300
//#define MAX_VALUES_BUFFER_SIZE 3


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint16_t adc_buf[ADC_buf_len];
float rms_value;
uint32_t sum_squares = 0;
float rms_current = 0.0f;
//uint32_t max_value = 0;
//uint16_t max_values_buffer[MAX_VALUES_BUFFER_SIZE];
float avg_value = 0;
double suntas = 0.44; // resistance in ohms

volatile uint32_t sum_of_squares = 0;
volatile uint16_t max_value = 0;

//volatile uint8_t operation_flag = 0;
char msg[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

//void display_rms_value(float value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///*
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_UART_Transmit_IT (&huart2, adc_buf,sizeof(adc_buf));
//}
//*/
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2)
    {

        HAL_UART_Transmit(&huart2, (uint8_t*)adc_buf, sizeof(adc_buf), HAL_MAX_DELAY);
    }
    else if (htim->Instance == TIM3) // Update LCD when TIM3 period elapses
    {
        display_rms_value(rms_value);
    }
}
*/
/*
void display_rms_value(float value)
{
    char buffer[16]; // Buffer to hold the string representation of the RMS value

    // Convert the float value to a string with two decimal places
    snprintf(buffer, sizeof(buffer), "RMS: %d mV", (int)value);

    //lcd_init();           // Clear the LCD screen
    lcd_send_cmd (0x80);     // Set cursor to the beginning of the first line
    lcd_send_string(buffer); // Display the RMS value on the LCD


}

*/
///*


//*/
/*
void UART_Transmit_ADC_Buffer(uint16_t* buffer, uint16_t size)
{
    // Initialize UART transmission buffer
    char uart_buffer[10]; // Buffer to store formatted ADC values

    // Iterate through the ADC buffer and send each value via UART
    for (int i = 0; i < size; i++)
    {
        // Format the ADC value as a string
        snprintf(uart_buffer, sizeof(uart_buffer), "%hu\r\n", buffer[i]);

        // Transmit the formatted ADC value via UART
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    }
}

*/
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
				sprintf(msg, "%hu\r\n", *adc_buf);
				HAL_UART_Transmit_IT(&huart2, (uint8_t *)msg, strlen(msg));
//					HAL_UART_Transmit_IT(&huart2, (uint8_t *)adc_buf, sizeof(adc_buf));
}
*/
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	    max_value = 0;
    for (int i = 0; i < ADC_buf_len; i++) {
        if (adc_buf[i] > max_value) {
            max_value = adc_buf[i];
        }
    }
    // Calculate sum of squares and find max value
    for (int i = 0; i < ADC_buf_len; i++) {
        sum_of_squares += adc_buf[i] * adc_buf[i];

    }

    // Calculate RMS value
    rms_value = (sqrtf((float)sum_of_squares / ADC_buf_len))*3.3/4096;
		//rms_value = max_value;
		rms_current = max_value*3.3/4096;

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */






  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  //HAL_TIM_Base_Start(&htim1); // Start TIM1



  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	lcd_init ();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_buf_len);
  HAL_TIM_Base_Start_IT(&htim2); // Start TIM2
  HAL_TIM_Base_Start_IT(&htim3); // Start TIM3


//	  lcd_send_string("a tu veiki?");
//		HAL_Delay(10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    max_value = 0;
  for (int i = 0; i < ADC_buf_len; i++) {
      if (adc_buf[i] > max_value) {
          max_value = adc_buf[i];
      }
  }
  	  rms_value = max_value/4096;
//		avg_value = 0;
//  for (int i = 0; i < ADC_buf_len; i++)
//  {
//      avg_value += adc_buf[i];
//  }
//   avg_value = avg_value/ADC_buf_len;

  // Calculate RMS value
//  int sum = 0;
//  for (int i = 0; i < ADC_buf_len; i++)
//  {
//      sum += pow((adc_buf[i] - avg_value), 2);
//  }
//  rms_value = sqrt(sum / ADC_buf_len);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* Paleidžiam pertrauktis */

	//HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */



  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



//void skaiciavimai()
//{
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // Calculate average value
//		    max_value = 0;
//	  for (int i = 0; i < ADC_buf_len; i++) {
//	      if (adc_buf[i] > max_value) {
//	          max_value = adc_buf[i];
//	      }
//	  }
//	  	  avg_value = max_value;
//	  	  rms_value = max_value/4095;

//    avg_value = 0;
//    for (int i = 0; i < ADC_buf_len; i++)
//    {
//        avg_value += adc_buf[i];
//    }
//    avg_value = avg_value/ADC_buf_len;
//
//    // Calculate RMS value
//    float sum = 0;
//    for (int i = 0; i < ADC_buf_len; i++)
//    {
//        sum += pow((adc_buf[i] - avg_value), 2);
//    }
//    rms_value = (sqrt((sum / ADC_buf_len)/4095)) * ADC_REF;
//HAL_Delay(1);

//		operation_flag |= 1; // UART
//    operation_flag |= 2; // LCD

//	}

void display_rms_value(float value, float value2)
{
    char buffer[16]; // Buffer to hold the string representation of the RMS value
    char buffer_2[16];

    // Convert the float value to a string with two decimal places
    snprintf(buffer, sizeof(buffer), "RMS: %d mV", (int)value);
	  snprintf(buffer_2, sizeof(buffer_2), "max: %d mA", (int)value2);

    lcd_clear();           // Clear the LCD screen
    lcd_put_cur(0,0);     // Set cursor to the beginning of the first line
    lcd_send_string(buffer); // Display the RMS value on the LCD
    lcd_put_cur(1,0);     // Set cursor to the beginning of the first line
    lcd_send_string(buffer_2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM2)
	{
				sprintf(msg, "%hu\r\n", *adc_buf);
				HAL_UART_Transmit_IT(&huart2, (uint8_t *)msg, strlen(msg));
//				HAL_UART_Transmit_IT(&huart2, (uint8_t *)adc_buf, sizeof(adc_buf));
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	}

if (htim->Instance == TIM3)
{
//		  	skaiciavimai();
			display_rms_value(avg_value, rms_value);
//	 		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
}

}


/*
void MX_LCD_Init(void)
{
    lcd_init();
}
*/
/*
		    char buffer[16]; // Buffer to hold the string representation of the RMS value

    // Convert the float value to a string with two decimal places
    snprintf(buffer, sizeof(buffer), "RMS: %.2f", rms_value);

    //lcd_init();           // Clear the LCD screen
    lcd_send_cmd (0x80);     // Set cursor to the beginning of the first line
    lcd_send_string(buffer); // Display the RMS value on the LCD
*/








/*
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* Hadc) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* Hadc) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
