/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with hysteresis PID temperature control
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   // For atof()

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define REF_TEMP 30.0f

float currentTemp = 0.0f;   // Actual temperature input from user
uint8_t uart_rx;
char input_buffer[10];
uint8_t input_index = 0;
uint8_t input_ready = 0;

// PID parameters (tuned for smooth fast response with small overshoot)
float Kp = 20.0f;
float Ki = 2.0f;
float Kd = 10.0f;

float integral = 0.0f;
float prev_error = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM3_Init(void);
void UART_PutString(char *str);
void Set_Heater_PWM(float heater_percent);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();

  // Start PWM at 0% duty cycle
  Set_Heater_PWM(0);

  UART_PutString("Temperature Control System\r\n");
  UART_PutString("Reference temperature set to 30.00 C\r\n");
  UART_PutString("Enter actual temperature (deg C): ");

  while (1)
  {
    if(HAL_UART_Receive(&huart1, &uart_rx, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      if(uart_rx == 27)  // ESC pressed
      {
        // Reset input buffer
        input_index = 0;
        input_ready = 0;
        UART_PutString("\r\nReset! Enter actual temperature (deg C): ");
      }
      else if(uart_rx == '\r' || uart_rx == '\n')
      {
        if(input_index > 0) // Only process if buffer not empty
        {
          input_buffer[input_index] = '\0';
          input_ready = 1;
        }
      }
      else
      {
        if(input_index < sizeof(input_buffer)-1)
        {
          input_buffer[input_index++] = uart_rx;
          HAL_UART_Transmit(&huart1, &uart_rx, 1, 10); // Echo back
        }
      }
    }

    if(input_ready)
    {
      // Parse actual temperature entered by user
      currentTemp = atof(input_buffer);

      // Sanity checks
      if(currentTemp < 0) currentTemp = 0;
      if(currentTemp > 100) currentTemp = 100;

      input_index = 0;
      input_ready = 0;

      char msg[64];
      snprintf(msg, sizeof(msg), "\r\nActual temperature set to %.2f C\r\n", currentTemp);
      UART_PutString(msg);
      UART_PutString("Starting temperature regulation...\r\n");

      // Control loop variables
      float heater_output = 0.0f;
      float upper_threshold = 30.0f;
      float lower_threshold = 29.8f;

      while(1)
      {
        if(currentTemp >= upper_threshold)
        {
          // Heater off to allow cooling
          heater_output = 0.0f;
          integral = 0.0f;  // Reset integral to avoid windup
          prev_error = 0.0f;
        }
        else if(currentTemp <= lower_threshold)
        {
          // Heater ON with PID control for fast warm-up
          float error = REF_TEMP - currentTemp;

          integral += error * 0.5f;  // 0.5 = loop delay in seconds
          // Anti-windup clamping
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
          // Between thresholds: heater off to avoid overshoot
          heater_output = 0.0f;
          integral = 0.0f;
          prev_error = 0.0f;
        }

        Set_Heater_PWM(heater_output);

        // Simulate temperature change with safe rates
        if(heater_output > 0)
          currentTemp += 0.1f * (heater_output / 100.0f);  // heating rate
        else
          currentTemp -= 0.05f;  // cooling rate

        // Clamp temperature within bounds
        if(currentTemp < 0) currentTemp = 0;
        if(currentTemp > 150) currentTemp = 150;

        char msg[64];
        snprintf(msg, sizeof(msg), "Current Temp: %.2f C, Heater: %.1f%%\r\n", currentTemp, heater_output);
        UART_PutString(msg);

        // Check for ESC to reset input
        if(HAL_UART_Receive(&huart1, &uart_rx, 1, 10) == HAL_OK)
        {
          if(uart_rx == 27) // ESC pressed
          {
            Set_Heater_PWM(0);
            UART_PutString("\r\nReset! Enter actual temperature (deg C): ");
            break; // exit regulation loop to ask input again
          }
        }

        HAL_Delay(500);  // Control loop delay 500ms
      }
    }
  }
}

void Set_Heater_PWM(float heater_percent)
{
  if(heater_percent > 100) heater_percent = 100;
  if(heater_percent < 0) heater_percent = 0;

  uint32_t pulse = (uint32_t)((heater_percent * (htim3.Init.Period + 1)) / 100.0f);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
}

void UART_PutString(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/* SystemClock_Config, MX_TIM3_Init, MX_USART1_UART_Init, MX_GPIO_Init and Error_Handler
   functions remain unchanged as in your original code */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

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

void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
