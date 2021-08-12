/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string, fmt, argp)) // build string
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void ei_printf(const char *format, ...) {
    va_list myargs;
    va_start(myargs, format);
    vprint(format, myargs);
    va_end(myargs);
}

using namespace ei;

// paste the raw features here
static float features[] = {
	0.2054, 0.2111, 0.2054, 0.2071, 0.2062, 0.2006, 0.2062, 0.1990, 0.1974, 0.2030, 0.2014, 0.1950, 0.2006, 0.2022, 0.2071, 0.2062, 0.2111, 0.2095, 0.2071, 0.2127, 0.2183, 0.2095, 0.2071, 0.2079, 0.2046, 0.2030, 0.1990, 0.1998, 0.1982, 0.2014, 0.2046, 0.1982, 0.2062, 0.2071, 0.2071, 0.2111, 0.2111, 0.2175, 0.2079, 0.2062, 0.2054, 0.2071, 0.1998, 0.1926, 0.2014, 0.1950, 0.1974, 0.1958, 0.1990, 0.2006, 0.2054, 0.2079, 0.2087, 0.2095, 0.2151, 0.2071, 0.2135, 0.2087, 0.2038, 0.2054, 0.2046, 0.2014, 0.1950, 0.1998, 0.1950, 0.2006, 0.1998, 0.2054, 0.2030, 0.2062, 0.2038, 0.2103, 0.2135, 0.2199, 0.2087, 0.2062, 0.2062, 0.2014, 0.2022, 0.1917, 0.1966, 0.2006, 0.2014, 0.2014, 0.1966, 0.2071, 0.2062, 0.2062, 0.2062, 0.2079, 0.2151, 0.2062, 0.2046, 0.2071, 0.2071, 0.2022, 0.1998, 0.2038, 0.1934, 0.1909, 0.2022, 0.1966, 0.2006, 0.2062, 0.2062, 0.2087, 0.2038, 0.2199, 0.2143, 0.2071, 0.2022, 0.2054, 0.2062, 0.1950, 0.2014, 0.1990, 0.1966, 0.1885, 0.2103, 0.2014, 0.2071, 0.2103, 0.2103, 0.2046, 0.2103, 0.2119, 0.2167, 0.2071, 0.2046, 0.2119, 0.2038, 0.2022, 0.1998, 0.1990, 0.1998, 0.2022, 0.2022, 0.2006, 0.2014, 0.2071, 0.2087, 0.2087, 0.2079, 0.2183, 0.2103, 0.2151, 0.2022, 0.2127, 0.2046, 0.1998, 0.2030, 0.2062, 0.1934, 0.1950, 0.1990, 0.1982, 0.2022, 0.2071, 0.2046, 0.1998, 0.2095, 0.2095, 0.2062, 0.2062, 0.2079, 0.2111, 0.2030, 0.2014, 0.1901, 0.1990, 0.2006, 0.1974, 0.2006, 0.1982, 0.1990, 0.2022, 0.2054, 0.2119, 0.2151, 0.2111, 0.2127, 0.2038, 0.2079, 0.2038, 0.2062, 0.2054, 0.1990, 0.1934, 0.1966, 0.1974, 0.2022, 0.2062, 0.2062, 0.1998, 0.2079, 0.2103, 0.2135, 0.2087, 0.2183, 0.1966, 0.2030, 0.2062, 0.1990, 0.2087, 0.2006, 0.2006, 0.2071, 0.1974, 0.2062, 0.1966, 0.2014, 0.2071, 0.2087, 0.2062, 0.2087, 0.2199, 0.2119, 0.2111, 0.2062, 0.1966, 0.2030, 0.2046, 0.2030, 0.2006, 0.1966, 0.1990, 0.2111, 0.1990, 0.2014, 0.2087, 0.2119, 0.2103, 0.2071, 0.2208, 0.2103, 0.2103, 0.2095, 0.2030, 0.2038, 0.2071, 0.2030, 0.2022, 0.1966, 0.2006, 0.2103, 0.2006, 0.2046, 0.2111, 0.2119, 0.2079, 0.2054, 0.2119, 0.2127, 0.2087, 0.2079, 0.1966, 0.2071, 0.2038, 0.2006, 0.1950, 0.1926, 0.1950, 0.2014, 0.1982, 0.2071, 0.2030, 0.2054, 0.2199, 0.2079, 0.2062, 0.2014, 0.2095, 0.2111, 0.2062, 0.2038, 0.2062, 0.1998, 0.1990, 0.1974, 0.1942, 0.1998, 0.2006, 0.2079, 0.2022, 0.2111, 0.2151, 0.2151, 0.2135, 0.2119, 0.2079, 0.2038, 0.2022, 0.1974, 0.2054, 0.1909, 0.2062, 0.1982, 0.2030, 0.2014, 0.2046, 0.2006, 0.2006, 0.2030, 0.2135, 0.2199, 0.2079, 0.2006, 0.2046, 0.2062, 0.2038, 0.2014, 0.2062, 0.1990, 0.2095, 0.1909, 0.2030, 0.2079, 0.2014, 0.2071, 0.2038, 0.2071, 0.2111, 0.2087, 0.2030, 0.2038, 0.2071, 0.2119, 0.2103, 0.2022, 0.2062, 0.1917, 0.1990, 0.1958, 0.2046, 0.2006, 0.2022, 0.2006, 0.2304, 0.2208, 0.2135, 0.2103, 0.2111, 0.2079, 0.2046, 0.2046, 0.1966, 0.2022, 0.1950, 0.2030, 0.1861, 0.2038, 0.2062, 0.2006, 0.2071, 0.2071, 0.2006, 0.2022, 0.2087, 0.2014, 0.1990, 0.2135, 0.2071, 0.2022, 0.2038, 0.2038, 0.2062, 0.2006, 0.2022, 0.1934, 0.1990, 0.2038, 0.2095, 0.2103, 0.2038, 0.2151, 0.2159, 0.2095, 0.2127, 0.2087, 0.2046, 0.2022, 0.2030, 0.2022, 0.2030, 0.1982, 0.2014, 0.2006, 0.2006, 0.2046, 0.2111, 0.2087, 0.2071, 0.2079, 0.2103, 0.2135, 0.2071, 0.2095, 0.2071, 0.1990, 0.2062, 0.2030, 0.2062, 0.1950, 0.2062, 0.1869, 0.1885, 0.2046, 0.2095, 0.2022, 0.2071, 0.2079, 0.2079, 0.1998, 0.2071, 0.2071, 0.2062, 0.2022, 0.2022, 0.1958, 0.1990, 0.1990, 0.1974, 0.2006, 0.1974, 0.1998, 0.2087, 0.2103, 0.2062, 0.2054, 0.2175, 0.2071, 0.2071, 0.2062, 0.2038, 0.2095, 0.2014, 0.1974, 0.1982, 0.1926, 0.2022, 0.1934, 0.2030, 0.2183, 0.2087, 0.2022, 0.2022, 0.2167, 0.2087, 0.2167, 0.1950, 0.2135, 0.2038, 0.2062, 0.2030, 0.2014, 0.1990, 0.2030, 0.1877, 0.1998, 0.2062, 0.2038, 0.2054, 0.2079, 0.2006, 0.2062, 0.2062, 0.2030, 0.2030, 0.2103, 0.2054, 0.2062, 0.2030, 0.1966, 0.1942, 0.1901, 0.1998, 0.1990, 0.2022, 0.2062, 0.2038, 0.2062, 0.2167, 0.2111, 0.2103, 0.2022, 0.2079, 0.2159, 0.2054, 0.2062, 0.2006, 0.1966, 0.1974, 0.2087, 0.1982, 0.1909, 0.1998, 0.2014, 0.2079, 0.2006, 0.2103, 0.2143, 0.2191, 0.2151, 0.2119, 0.2071, 0.2030, 0.2006, 0.2046, 0.1942, 0.1958, 0.2014, 0.2022, 0.2038, 0.2062, 0.2038, 0.2087, 0.2046, 0.2071, 0.2087, 0.2030, 0.2079, 0.2087, 0.1982, 0.2095, 0.1998, 0.1998, 0.1966, 0.1917, 0.1990, 0.1966, 0.1966, 0.2038, 0.2030, 0.2087, 0.2022, 0.2135, 0.2167, 0.2127, 0.2175, 0.2135, 0.2062, 0.2054, 0.2022, 0.2014, 0.2006, 0.2062, 0.1901, 0.1990, 0.1990, 0.1990, 0.2046, 0.2159, 0.2038, 0.2127, 0.2143, 0.2119, 0.2038, 0.2119, 0.2087, 0.2038, 0.2079, 0.1998, 0.1934, 0.1901, 0.1966, 0.2038, 0.1982, 0.2046, 0.2022, 0.2038, 0.1942, 0.2119, 0.2151, 0.2135, 0.2119, 0.2103, 0.2071, 0.2079, 0.2030, 0.2014, 0.1974, 0.1966, 0.1966, 0.1998, 0.2014, 0.2062, 0.2030, 0.2103, 0.2054, 0.2119, 0.2143, 0.2143, 0.2087, 0.2079, 0.2046, 0.2014, 0.2014, 0.1974, 0.1942, 0.2006, 0.1974, 0.1990, 0.2014, 0.2038, 0.2030, 0.2014, 0.2119, 0.2143, 0.2022, 0.2240, 0.2111, 0.2079, 0.2022, 0.2038, 0.2006, 0.1974, 0.2014, 0.1974, 0.1998, 0.1998, 0.2022, 0.2054, 0.2022, 0.2119, 0.2175, 0.2095, 0.2111, 0.2014, 0.2111, 0.2095, 0.2030, 0.2062, 0.1990, 0.2006, 0.2014, 0.2038, 0.2006, 0.1966, 0.2022, 0.2062, 0.2087, 0.2014, 0.2087, 0.2183, 0.2111, 0.2095, 0.2111, 0.2054, 0.2006, 0.2038, 0.2014, 0.1982, 0.1990, 0.2046, 0.1942, 0.1982, 0.2022, 0.2046, 0.2079, 0.2208, 0.2071, 0.2062, 0.2175, 0.2062, 0.2062, 0.2046, 0.2103, 0.2038, 0.2006, 0.2038, 0.1998, 0.1998, 0.1901, 0.2046, 0.2038, 0.2079, 0.2095, 0.2135, 0.2127, 0.2151, 0.2119, 0.2159, 0.2006, 0.2095, 0.1998, 0.2038, 0.1974, 0.1950, 0.2038, 0.1934, 0.2006, 0.2046, 0.2038, 0.2087, 0.2079, 0.2135, 0.2143, 0.2103, 0.2159, 0.2054, 0.2127, 0.2062, 0.1982, 0.2062, 0.1982, 0.1974, 0.1966, 0.1926, 0.1998, 0.2030, 0.2022, 0.2111, 0.2038, 0.2062, 0.2014, 0.2071, 0.2151, 0.2103, 0.2062, 0.2062, 0.2079, 0.2022, 0.1990, 0.2014, 0.2038, 0.1990, 0.2006, 0.2022, 0.2038, 0.2006, 0.2022, 0.2095, 0.2127, 0.2143, 0.1861, 0.2103, 0.2079, 0.2038, 0.2087, 0.2006, 0.2014, 0.2038, 0.1901, 0.1982, 0.1998, 0.2030, 0.2095, 0.2038, 0.2103, 0.2087, 0.2183, 0.2062, 0.2183, 0.2079, 0.2046, 0.2022, 0.1998, 0.2030, 0.1950, 0.2030, 0.1990, 0.2022, 0.1974, 0.1966, 0.2038, 0.2087, 0.2079, 0.2079, 0.2022, 0.2135, 0.2071, 0.2103, 0.2062, 0.2038, 0.2022, 0.2038, 0.1934, 0.1934, 0.1917, 0.2022, 0.1934, 0.1974, 0.2062, 0.2046, 0.2030, 0.2079, 0.2046, 0.2191, 0.2135, 0.2103, 0.2087, 0.2054, 0.2022, 0.2062, 0.2022, 0.1998, 0.2014, 0.1942, 0.1974, 0.2030, 0.2111, 0.2062, 0.2071, 0.2103, 0.2103, 0.2175, 0.2030, 0.1998, 0.2062, 0.1990, 0.1998, 0.2022, 0.2006, 0.2022, 0.2030, 0.2062, 0.1982, 0.2030, 0.2022, 0.2046, 0.2062, 0.2030, 0.2159, 0.2038, 0.2127, 0.2087, 0.2006, 0.2062, 0.2062, 0.1958, 0.1934, 0.1974, 0.1982, 0.2006, 0.2030, 0.2022, 0.2022, 0.2062, 0.2054, 0.2135, 0.2167, 0.2199, 0.2143, 0.2022, 0.2087, 0.2087, 0.1998, 0.2006, 0.1917, 0.1885, 0.1998, 0.2062, 0.2006, 0.2014, 0.2079, 0.2103, 0.2087, 0.2095, 0.2159, 0.2022, 0.2095, 0.2095, 0.2022, 0.2062, 0.2038, 0.2038, 0.1990, 0.1990, 0.1990, 0.2022, 0.2014, 0.2062, 0.2054, 0.2030, 0.2135, 0.2127, 0.2119, 0.2014, 0.2095, 0.2071, 0.2030, 0.2006, 0.1990, 0.2030, 0.2046, 0.2014, 0.1934, 0.2006, 0.2046, 0.2006, 0.2062, 0.2062, 0.2062, 0.2151, 0.2062
};

int get_feature_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ADC :
//internal ADC
// 								Analog 			 -------> 			PA0
//external ADC
//							AD7606 pin							STM32L431 pin
//								+5V				 ------->			+5V
//								GND				 ------->			GND
// 								SER 			 -------> 			PC3
// 								CO-A 			 -------> 			PC5
// 								CO-B 			 -------> 			PC6
// 								REST 			 -------> 			PC7
// 								BUSY 			 -------> 			PA1
// 								CS_N 			 -------> 			PB12
// 								RD/SC 	 	     -------> 			PB13
// 						  	    DB7    		     -------> 			PB14

void AD7606_StartConvst(void)
{
	HAL_GPIO_WritePin(CO_A_GPIO_Port, CO_A_Pin, GPIO_PIN_RESET); //	CO_A_L;
	HAL_GPIO_WritePin(CO_B_GPIO_Port, CO_B_Pin, GPIO_PIN_RESET); //	CO_B_L;
	HAL_Delay(1);
	HAL_GPIO_WritePin(CO_A_GPIO_Port, CO_A_Pin, GPIO_PIN_SET);   //	CO_A_H;
	HAL_GPIO_WritePin(CO_B_GPIO_Port, CO_B_Pin, GPIO_PIN_SET);   //	CO_B_H;
	HAL_Delay(1);
}
void AD7606_RESET(void)
{
	HAL_GPIO_WritePin(REST_GPIO_Port, REST_Pin, GPIO_PIN_RESET); //REST_L;
	HAL_Delay(1);
	HAL_GPIO_WritePin(REST_GPIO_Port, REST_Pin, GPIO_PIN_SET);   //REST_H;
	HAL_Delay(1);
	HAL_GPIO_WritePin(REST_GPIO_Port, REST_Pin, GPIO_PIN_RESET); //REST_L;
}
void AD7606_Init(void)
{
//	MX_SPI2_Init();
//	GPIO_AD7606_Configuration();
	HAL_GPIO_WritePin(CO_A_GPIO_Port, CO_A_Pin, GPIO_PIN_SET);   //	CO_A_H;
	HAL_GPIO_WritePin(CO_B_GPIO_Port, CO_B_Pin, GPIO_PIN_SET);   //	CO_B_H;
	HAL_Delay(1);
	HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, GPIO_PIN_SET);     // SER_H;

	AD7606_RESET();
	HAL_Delay(1);
	AD7606_StartConvst();
}
void AD7606_ReadData(uint16_t * DB_data)
{
	HAL_SPI_Receive(&hspi2, (uint8_t *)DB_data, 8, 1000);
}
#define ADC_DMA_BUF_LEN 5000
uint32_t s_adc_dma_buf[ADC_DMA_BUF_LEN]={0};
uint32_t s_adc_dma_buf_temp[ADC_DMA_BUF_LEN]={0};
uint8_t adc_dma_flag = 0;
uint8_t printf_running = 0;
uint16_t adc_dmc_count = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t ADCvalue;
  float Voltage;
  float Rest;
  uint8_t dis_buf[40];
  uint16_t DB_data[8] = {0};
  int count=895;
  int i,j;
//  #define INTERNAL_ADC_ROLLING
#define INTERNAL_ADC_DMA
//  #define EXTERNAL_ADC
//  #define INFERENCE_ON_EDGE

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  signal_t signal;
  signal.total_length = sizeof(features) / sizeof(features[0]);
  signal.get_data = &get_feature_data;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
// internal adc module init and start
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)s_adc_dma_buf,ADC_DMA_BUF_LEN);
// external adc chip   init and start
//  AD7606_Init();
// compile information and welcome!!
  printf("%d\n", __LINE__);
  printf("%s\n", __TIME__);
  printf("%s\n", __DATE__);
  printf("%s\n", __FILE__);
  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// 5-6s printf complete 5000sample  uartport:115200 8n1
	if(adc_dma_flag==1)
	{
		adc_dma_flag = 0;
		for(i=0;i<ADC_DMA_BUF_LEN;i++)
		{
			ADCvalue = s_adc_dma_buf[i];
			Voltage=ADCvalue*3.3/4096;   //2^12=4096
			printf("%f\t\r\n",Voltage);
// for future, inference on edge device.
//			features[count] = Voltage;
//			count++;
		}
	}

// inference on edge device.
    if(count==895)
    {
#ifdef INFERENCE_ON_EDGE  //inference neural network on powerstrip board, train neural network in cloud/edge server.
		ei_impulse_result_t result = { 0 };
		EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
		HAL_Delay(3000);
#endif
		count=0;
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// 50khz, 5kbyte, 100ms,
	// every 6s printf once
	adc_dmc_count++;
	if(adc_dmc_count==60)
	{
		adc_dmc_count=0;
		memcpy(s_adc_dma_buf_temp, s_adc_dma_buf, ADC_DMA_BUF_LEN);
		adc_dma_flag = 1;
	}

	if (hadc == (&hadc1))
	{
		static uint8_t ledState=0;
		if (ledState == 0)
			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		else
			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		ledState = !ledState;
	}
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
