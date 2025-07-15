/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * ...
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM3_Init(void);
void MX_ADC1_Init(void);

void UART_PutString(char *str);
void Set_Heater_PWM(float heater_percent);
float Read_Temperature(void);
void Error_Handler(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();

  Set_Heater_PWM(0);

  UART_PutString("Temperature Control System with LM35 Sensor\r\n");
  UART_PutString("Reference temperature set to 30.00 C\r\n");
  UART_PutString("Press ESC to pause/resume the system\r\n");

  float currentTemp = 0.0f;
  float heater_output = 0.0f;
  float integral = 0.0f;
  float prev_error = 0.0f;
  float Kp = 20.0f, Ki = 2.0f, Kd = 10.0f;
  float REF_TEMP = 30.0f;
  float upper_threshold = 30.0f;
  float lower_threshold = 29.8f;

  uint8_t paused = 0;

  while (1)
  {
    uint8_t rx_byte = 0;
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 0) == HAL_OK)
    {
      if (rx_byte == 0x1B) // ESC key ASCII code
      {
        paused = !paused;
        if (paused)
        {
          UART_PutString("System Paused\r\n");
          Set_Heater_PWM(0); // turn heater off when paused
          integral = 0.0f;   // reset PID integral term on pause
          prev_error = 0.0f; // reset derivative term on pause
        }
        else
        {
          UART_PutString("System Resumed\r\n");
        }
      }
    }

    if (!paused)
    {
      currentTemp = Read_Temperature();

      char msg[64];
      snprintf(msg, sizeof(msg), "Current Temp: %.2f C\r\n", currentTemp);
      UART_PutString(msg);

      if(currentTemp >= upper_threshold)
      {
        heater_output = 0.0f;
        integral = 0.0f;
        prev_error = 0.0f;
      }
      else if(currentTemp <= lower_threshold)
      {
        float error = REF_TEMP - currentTemp;
        integral += error * 0.5f;
        if(integral > 50.0f) integral = 50.0f;
        if(integral < -50.0f) integral = -50.0f;
        float derivative = (error - prev_error) / 0.5f;
        heater_output = Kp * error + Ki * integral + Kd * derivative;
        if(heater_output > 100.0f) heater_output = 100.0f;
        if(heater_output < 0.0f) heater_output = 0.0f;
        prev_error = error;
      }
      else
      {
        heater_output = 0.0f;
        integral = 0.0f;
        prev_error = 0.0f;
      }

      Set_Heater_PWM(heater_output);
      char msg2[64];
      snprintf(msg2, sizeof(msg2), "Heater Output: %.1f%%\r\n", heater_output);
      UART_PutString(msg2);
    }
    else
    {
      UART_PutString("System Paused... Press ESC to resume\r\n");
    }

    HAL_Delay(500);
  }
}

float Read_Temperature(void)
{
  uint32_t adc_value = 0;
  float voltage = 0.0f;
  float temperature = 0.0f;

  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    adc_value = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  voltage = (3.3f * adc_value) / 4095.0f;
  temperature = voltage / 0.01f; // LM35 outputs 10mV per degree Celsius

  return temperature;
}

void Set_Heater_PWM(float heater_percent)
{
  if(heater_percent > 100) heater_percent = 100;
  if(heater_percent < 0) heater_percent = 0;

  uint32_t pulse = (uint32_t)((heater_percent * (htim3.Init.Period + 1)) / 100.0f);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
}

void UART_PutString(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  // TODO: Add your clock setup code here
}

/**
  * @brief ADC1 Initialization Function
  */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

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
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_6;  // Make sure your sensor is connected to this channel
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_391CYCLES;  // Single-ended default

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  */
void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;  // Start with PWM off
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  */
void MX_USART1_UART_Init(void)
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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Example: configure GPIOB PIN 3 as output (replace as needed)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  // User can add error handling here
  while(1)
  {
    // Stay here
  }
}
