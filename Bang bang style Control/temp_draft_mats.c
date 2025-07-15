/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define REFERENCE_TEMP 30.0f
#define TEMP_STEP 0.01f

UART_HandleTypeDef huart1;

float actual_temp = 0.0f;
char rx_buffer[10];
uint8_t rx_data;
/* USER CODE END PTD */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */

void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

int UART_Receive_Line(char *buffer, int max_len)
{
    int i = 0;
    while (1)
    {
        if (HAL_UART_Receive(&huart1, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            if (rx_data == '\r' || rx_data == '\n')
            {
                buffer[i] = '\0';
                return i;
            }
            else if (i < max_len - 1)
            {
                buffer[i++] = rx_data;
                HAL_UART_Transmit(&huart1, &rx_data, 1, HAL_MAX_DELAY); // Echo
            }
        }
    }
}

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

    char msg[100];
    float reference_temp = REFERENCE_TEMP;

    UART_Print("=== Temperature Control System ===\r\n");
    UART_Print("Press ESC anytime to re-enter temperature.\r\n");

    while (1)
    {
        UART_Print("Enter actual temperature (in degree C): ");
        UART_Receive_Line(rx_buffer, sizeof(rx_buffer));
        actual_temp = atof(rx_buffer);

        snprintf(msg, sizeof(msg), "Reference temperature is set to %.2f C\r\n", reference_temp);
        UART_Print(msg);
        UART_Print("Starting temperature control...\r\n");

        while (1)
        {
            if (actual_temp < reference_temp)
            {
                UART_Print("Heater is ON. Temperature increasing...\r\n");
                while (actual_temp < reference_temp)
                {
                    actual_temp += TEMP_STEP;
                    snprintf(msg, sizeof(msg), "Temperature: %.2f C\r\n", actual_temp);
                    UART_Print(msg);

                    if (HAL_UART_Receive(&huart1, &rx_data, 1, 100) == HAL_OK)
                    {
                        if (rx_data == 27)
                        {
                            UART_Print("ESC pressed. Enter new temperature: ");
                            UART_Receive_Line(rx_buffer, sizeof(rx_buffer));
                            actual_temp = atof(rx_buffer);
                            break;
                        }
                        else if (rx_data == '\r' || rx_data == '\n')
                        {
                            int len = 0;
                            while (HAL_UART_Receive(&huart1, (uint8_t *)&rx_buffer[len], 1, 10) == HAL_OK)
                            {
                                if (rx_buffer[len] == '\r' || rx_buffer[len] == '\n')
                                {
                                    rx_buffer[len] = '\0';
                                    break;
                                }
                                len++;
                            }
                            float new_temp = atof(rx_buffer);
                            snprintf(msg, sizeof(msg), "New temperature input: %.2f C\r\n", new_temp);
                            UART_Print(msg);
                            actual_temp = new_temp;
                            break;
                        }
                    }
                    HAL_Delay(100);
                }
            }
            else if (actual_temp > reference_temp)
            {
                UART_Print("Heater is OFF. Temperature decreasing...\r\n");
                while (actual_temp > reference_temp)
                {
                    actual_temp -= TEMP_STEP;
                    snprintf(msg, sizeof(msg), "Temperature: %.2f C\r\n", actual_temp);
                    UART_Print(msg);

                    if (HAL_UART_Receive(&huart1, &rx_data, 1, 100) == HAL_OK)
                    {
                        if (rx_data == 27)
                        {
                            UART_Print("ESC pressed. Enter new temperature: ");
                            UART_Receive_Line(rx_buffer, sizeof(rx_buffer));
                            actual_temp = atof(rx_buffer);
                            break;
                        }
                        else if (rx_data == '\r' || rx_data == '\n')
                        {
                            int len = 0;
                            while (HAL_UART_Receive(&huart1, (uint8_t *)&rx_buffer[len], 1, 10) == HAL_OK)
                            {
                                if (rx_buffer[len] == '\r' || rx_buffer[len] == '\n')
                                {
                                    rx_buffer[len] = '\0';
                                    break;
                                }
                                len++;
                            }
                            float new_temp = atof(rx_buffer);
                            snprintf(msg, sizeof(msg), "New temperature input: %.2f C\r\n", new_temp);
                            UART_Print(msg);
                            actual_temp = new_temp;
                            break;
                        }
                    }
                    HAL_Delay(100);
                }
            }
            else
            {
                snprintf(msg, sizeof(msg), "Temperature stabilized at %.2f C\r\n", actual_temp);
                UART_Print(msg);
                HAL_Delay(1000);
            }
        }
    }
}

/**
  * @brief System Clock Configuration
  */
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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_PCLK3;
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

/**
  * @brief USART1 Initialization Function
  */
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
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
