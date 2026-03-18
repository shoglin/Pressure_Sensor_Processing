/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
extern "C" {
#include "main.h"
}
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

// #include "hydrv_clock.hpp"
#include <cstring>

#include <chrono>
#include <ctime>

extern "C" {
#include <sys/time.h>

int _gettimeofday(struct timeval *tv, void *tz) {
  if (tv) {
    // Get time from RTC or use system tick
    tv->tv_sec = HAL_GetTick() / 1000;
    tv->tv_usec = (HAL_GetTick() % 1000) * 1000;
  }
  return 0;
}
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_LENGTH 5

int depth_mm = 0;

class Memory {
public:
  hydrolib::ReturnCode Read(void *buffer, unsigned address, unsigned length) {
    if (length + address > BUFFER_LENGTH) {
      return hydrolib::ReturnCode::FAIL;
    }
    memcpy(buffer, buffer_ + address, length);
    return hydrolib::ReturnCode::OK;
  }

  hydrolib::ReturnCode Write(const void *buffer, unsigned address,
                             unsigned length) {
    if (address + length > BUFFER_LENGTH) {
      return hydrolib::ReturnCode::FAIL;
    }
    memcpy(buffer_ + address, buffer, length);
    return hydrolib::ReturnCode::OK;
  }

  uint32_t Size() { return BUFFER_LENGTH; }

private:
  uint8_t buffer_[BUFFER_LENGTH] = {};
};

constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 9,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<128, 128>
    uart1(hydrv::UART::UARTLow::USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrv::GPIO::GPIOLow rx_pin2(hydrv::GPIO::GPIOLow::GPIOA_port, 3,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin2(hydrv::GPIO::GPIOLow::GPIOA_port, 2,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<128, 128>
    uart2(hydrv::UART::UARTLow::USART2_115200_LOW, rx_pin2, tx_pin2, 7);

char log_format[] = "[%s] [%l] %m\n\r";

constinit hydrolib::logger::LogDistributor<hydrv::UART::UART<128, 128>>
    distributor(log_format, uart2);

constinit hydrolib::logger::Logger<
    hydrolib::logger::LogDistributor<hydrv::UART::UART<128, 128>>>
    logger("SerialProtocol", 1, distributor);

hydrolib::bus::datalink::StreamManager manager(1, uart1, logger);

hydrolib::bus::datalink::Stream stream(manager, 2);

Memory memory;

hydrolib::bus::application::Slave slave(stream, memory, logger);

hydrolib::bus::application::Master master(stream, logger);

void tx_SendPressureData(int value) {
  uint8_t depth_tx[4] = {0};
  depth_tx[0] = (value >> 0) & 0xFF; // LSB
  depth_tx[1] = (value >> 8) & 0xFF;
  depth_tx[2] = (value >> 16) & 0xFF;
  depth_tx[3] = (value >> 24) & 0xFF; // MSB
  memory.Write(depth_tx, 0, 4);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGHT_OF_THE_ARRAY 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Макросы для расчетов сырого давления
#define VOLTAGE_WITHOUT_ATM_(adc_for_pressure_middle, adc_z)                   \
  ((adc_for_pressure_middle - adc_z) * 16575 / 2048)
#define VOLTAGE_CONV_PASCALES_(adc_conv_volt)                                  \
  (adc_conv_volt * 6895 / 4 * 37 / 9)
// #define VOLTAGE_CONV_PASCALES_(adc_conv_volt) (adc_conv_volt*6895*37/(4*9))
#define PASCALES_CONV_DEPTH_MM_(voltage_conv_pascales)                         \
  (voltage_conv_pascales / (10 * 981))
// Старый вариант
/* #define VOLTAGE_WITHOUT_ATM_(adc_for_pressure_middle, adc_z)
((adc_for_pressure_middle-adc_z)*33150/4096) #define
VOLTAGE_CONV_PASCALES_(adc_conv_volt) (adc_conv_volt*3*6895*37/(4*27)) #define
PASCALES_CONV_DEPTH_MM_(voltage_conv_pascales)
(voltage_conv_pascales*100/(1000*981)) #define
DEPTH_MM_WITH_TERMOCOMP_(depth_mm, temperature) (depth_mm - ) */
// Макросы для расчётов температуры
#define ADC_CONV_VOLTAGE_(adc_for_temperature_middle)                          \
  (adc_for_temperature_middle * 33120 / 4096)
#define VOLTAGE_CONV_TEMPERATURE_(adc_conv_voltage_for_temperature)            \
  ((adc_conv_voltage_for_temperature - 5000) * 10)
// сотые доли градуса. Из мкв в мв (умножить на 1000), поделить на 10мв/г,
// умножить на 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern "C" {
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
}
/* USER CODE BEGIN PV */
int adc_for_pressure = 0;
int adc_for_temperature = 0;
uint32_t adc[LENGHT_OF_THE_ARRAY * 2] = {0};
int adc_for_pressure_middle = 0;
int adc_for_temperature_middle = 0;
int adc_z = 0;
int adc_z_flag = 0;
int adc_flag = 0;
int half_adc_flag = 0;
int tx_flag = 0;
int tx_err = 0;
// uint8_t UART_depth[5] = {0};
int tx_count = 0;
char trans_str[100] = {
    0,
};
uint8_t UART_depth[6] = {
    0,
};

// Преобразования по шагам для сырого давления
int adc_conv_volt_for_pressure = 0;
int voltage_conv_pascales = 0;

// Преобразования по шагам для температуры
int adc_conv_voltage_for_temperature = 0;
int temperature = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(1000);
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, adc, LENGHT_OF_THE_ARRAY * 2);
  uart1.Init();
  uart2.Init();
  /* USER CODE END 2 */
  while (1) {
    if (adc_flag) {
      adc_for_pressure_middle = 0;
      adc_for_temperature_middle = 0;
      if (!half_adc_flag) {
        for (int counter = 0; counter < LENGHT_OF_THE_ARRAY; counter++) {
          if (counter % 2 == 0) {
            adc_for_pressure_middle += adc[counter];
          } else {
            adc_for_temperature_middle += adc[counter];
          }
        }
      } else {
        for (int counter = LENGHT_OF_THE_ARRAY;
             counter < LENGHT_OF_THE_ARRAY * 2; counter++) {
          if (counter % 2 == 0) {
            adc_for_pressure_middle += adc[counter];
          } else {
            adc_for_temperature_middle += adc[counter];
          }
        }
      }
      adc_for_pressure_middle /= LENGHT_OF_THE_ARRAY / 2;
      adc_for_temperature_middle /= LENGHT_OF_THE_ARRAY / 2;
      if (!adc_z_flag) {
        adc_z = adc_for_pressure_middle;
        adc_z_flag = 1;
      }

      // Считаем сырое давление
      adc_conv_volt_for_pressure =
          VOLTAGE_WITHOUT_ATM_(adc_for_pressure_middle, adc_z);
      voltage_conv_pascales =
          VOLTAGE_CONV_PASCALES_(adc_conv_volt_for_pressure);
      depth_mm = PASCALES_CONV_DEPTH_MM_(voltage_conv_pascales);

      // Считаем температуру
      adc_conv_voltage_for_temperature =
          ADC_CONV_VOLTAGE_(adc_for_temperature_middle);
      temperature = VOLTAGE_CONV_TEMPERATURE_(adc_conv_voltage_for_temperature);
      if (temperature < -40000)
        temperature = -40000;
      if (temperature > 125000)
        temperature = 125000;

      // Заканчиваем преобразования и выставляем флаг прерывания в "0"
      adc_flag = 0;
    }

    /* USER CODE END WHILE */
    tx_SendPressureData(depth_mm);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
extern "C" {
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  half_adc_flag = 1;
  adc_flag = 1;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  adc_flag = 1;
  half_adc_flag = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
  if (UartHandle == &huart1) {
    tx_flag = 0;
  }
}

void USART2_IRQHandler(void) { uart2.IRQCallback(); }
void USART1_IRQHandler(void) { uart1.IRQCallback(); }

/* USER CODE END 4 */
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
