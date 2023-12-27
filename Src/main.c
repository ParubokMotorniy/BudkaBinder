/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "AC_Dimmer.h"
#include "lcd5110.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Dimmer_1 0
#define MCP9808_ADDRESS 0x18

#define successMessageSize 10
#define errorMessageSize 8
#define maxMessageSize 11
#define uResponseLineSize 8
#define sResponseLineSize 9
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
typedef enum state_t {
 CheckingWeight,
 CheckingTemperature,
 WarmingUp,
 CoolingDown
} state_t;

const uint8_t absolutePetInsideWeightThresholdLow = 2; //can not be set lower than this
const uint8_t absolutePetInsideWeightThresholdUpper = 40; //defined by characteristics of tenso sensors
const int8_t absoluteTemperatureLowBound = -15; //can not be set lower than this
const int8_t absoluteTemperatureUpperBound = 45; //can not be set lower than this

volatile uint8_t petInsideWeightThreshold = 12; //kilograms
volatile int8_t temperatureLowBound = 14; //celsius
volatile int8_t temperatureUpperBound = 20; //celsius

volatile int8_t setTemperature = 0; //celsius
volatile uint16_t temperatureStep = 5;
volatile uint16_t secondsElapsed = 0;
volatile uint16_t temperatureCheckInterval = 0; //seconds
volatile uint16_t maxVoltageValue = 256; //0 to 256
volatile uint16_t maxRugTemperature = 40; //celsius

uint8_t stateMessage[errorMessageSize] = {85, 80, 71, 79, 73, 78, 10, 13};
uint8_t successMessage[successMessageSize] = "SUCCESS!\r\n";
uint8_t errorMessage[errorMessageSize] = "ERROR!\r\n";
uint8_t receive_buff[maxMessageSize];

uint8_t lowerBoundMessage[sResponseLineSize] = {76, 87, 82, 61, 0, 0, 0, 10, 13};
uint8_t upperBoundMessage[uResponseLineSize] = {85, 80, 82, 61, 0, 0, 10, 13};
uint8_t setTempMessage[uResponseLineSize] = {84, 77, 80, 61, 0, 0, 10, 13};
uint8_t loadMessage[uResponseLineSize] = {76, 79, 68, 61, 0, 0, 10, 13};
uint8_t weightBoundMessage[uResponseLineSize] = {87, 71, 84, 61, 0, 0, 10, 13};
uint8_t presenceMessage[uResponseLineSize] = {80, 82, 83, 61, 48, 0, 10, 13};
uint8_t curTempMessage[sResponseLineSize] = {67, 85, 82, 61, 0, 0, 0, 10, 13};

uint8_t ledArrLength = 4;
uint16_t leds[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

volatile int32_t currentWeight = 0;
volatile int16_t currentTemperature = 0;

volatile int32_t hx711_value = 14;
volatile int32_t hx711_value1 = 88;

const int16_t roughPlankWeight = 0;
const int16_t weightConversionCoefficient = 1;
const int16_t supportCoefficient = 2;

void TransitState(state_t newState);

void SwitchStateLEDs(uint16_t ledToEnable)
{
	for(int i = 0; i < ledArrLength; i++)
	{
		if(leds[i] == ledToEnable)
		{
			HAL_GPIO_WritePin(GPIOD, leds[i], GPIO_PIN_SET);
		}else
		{
			HAL_GPIO_WritePin(GPIOD, leds[i], GPIO_PIN_RESET);
		}
	}
}

static uint8_t GAIN;
void delay_us (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	while(__HAL_TIM_GET_COUNTER(&htim10) < us);
}
void hx711_powerUp(void) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
}
void hx711_setGain(uint8_t gain) {
	if(gain < 64) GAIN = 2;
	else if(gain < 128) GAIN = 3;
	else GAIN = 1;
}
void hx711_init(void) {
	hx711_setGain(128);
	hx711_powerUp();
}
int32_t hx711_get_value(void) {
		uint32_t data = 0;
		uint8_t dout;
		int32_t filler;
		int32_t ret_value;
		for (uint8_t i = 0; i < 24; i++) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			delay_us(1);
			dout = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
			data = data << 1;
			if (dout) {
				data++;
			}
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			delay_us(1);
		}
		for( int i = 0; i < GAIN; i ++ ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			delay_us(1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			delay_us(1);
		}
		if( data & 0x800000 )
			filler = 0xFF000000;
		else
			filler = 0x00000000;

		ret_value = filler + data;
		return ret_value;
	}
uint8_t hx711_is_ready(void) {
	return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == GPIO_PIN_RESET;
}
//////////////////////////////////////////////////////////////////////
static uint8_t GAIN1;
void delay_us1(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim11, 0);
	while(__HAL_TIM_GET_COUNTER(&htim11) < us);
}
void hx711_powerUp1(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
}
void hx711_setGain1(uint8_t gain) {
	if(gain < 64) GAIN1 = 2;
	else if(gain < 128) GAIN1 = 3;
	else GAIN1 = 1;
}
void hx711_init1(void) {
	hx711_setGain1(128);
	hx711_powerUp1();
}
int32_t hx711_get_value1(void) {
		uint32_t data = 0;
		uint8_t dout;
		int32_t filler;
		int32_t ret_value;
		for (uint8_t i = 0; i < 24; i++) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			delay_us1(1);
			dout = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
			data = data << 1;
			if (dout) {
				data++;
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			delay_us1(1);
		}
		for( int i = 0; i < GAIN1; i ++ ) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			delay_us1(1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			delay_us1(1);
		}
		if( data & 0x800000 )
			filler = 0xFF000000;
		else
			filler = 0x00000000;

		ret_value = filler + data;
		return ret_value;
	}
uint8_t hx711_is_ready1(void) {
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET;
}
int32_t GetCurrentWeight()
{
	hx711_value += 15;// hx711_get_value();

	hx711_value1 += 25; //hx711_get_value1();

	int32_t weight = (hx711_value + hx711_value1 - roughPlankWeight) * supportCoefficient;
	weight /= 1000;
	weight *= weightConversionCoefficient;

	//return 15;
	//currentWeight = weight;
	//currentWeight += 15;
	return weight;
}

void WEIGHT_init()
{
    hx711_init();
    hx711_init1();
}

int16_t GetCurrentTemperature()
{
	//TODO: temperature measuring code here
//	int temp_flag = 0;
//	uint8_t buffer[2];
//	HAL_I2C_Mem_Read(&hi2c1, MCP9808_ADDRESS << 1, 0x05, 1, buffer, 2, HAL_MAX_DELAY);
//
//	int16_t temperature = ((buffer[0] << 8) | buffer[1]) & 0xFFF;
//	float temp_celsius = (float)temperature / 16.0;
//	sprintf(buffer, "Temperature: %.2f degrees \r\n", temp_celsius);
		//printf( "Temperature: %.2f degrees Celsius \r\n", temp_celsius);

	return 20;
}

bool SetCurrentTemperature(uint8_t newTemperature)
{
	if(newTemperature > temperatureUpperBound || newTemperature < temperatureLowBound){return false;}
	setTemperature = newTemperature;

	Dimm_value(Dimmer_1, (setTemperature * maxVoltageValue) / maxRugTemperature);
	return true;

}

bool SetTemperatureLowerBound(uint8_t newBound)
{
	if(newBound < absoluteTemperatureLowBound || newBound >= temperatureUpperBound){return false;}

	temperatureLowBound = newBound;
	return true;
}

bool SetTemperatureUpperBound(uint8_t newBound)
{
	if(newBound > absoluteTemperatureUpperBound || newBound <= temperatureLowBound){return false;}

	temperatureUpperBound = newBound;
	return true;
}

bool SetWeightThreshold(uint8_t newThreshold)
{
	if(newThreshold > absolutePetInsideWeightThresholdUpper || newThreshold < absolutePetInsideWeightThresholdLow){return false;}

	petInsideWeightThreshold = newThreshold;
	return 1;
}

void WeightCheckingRoutine()
{
	if(currentWeight < petInsideWeightThreshold){return;}

	TransitState(CheckingTemperature);
}

void SwitchTemperatureRelay(uint8_t relayValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, relayValue);
}

bool PetLeft()
{
	return (currentWeight < petInsideWeightThreshold);
}

void TemperatureCheckingRoutine()
{
	if(PetLeft())
	{
		TransitState(CheckingWeight);
		return;
	}

	if(currentTemperature < temperatureLowBound)
	{
		TransitState(WarmingUp);
	} else if(currentTemperature > temperatureUpperBound)
	{
		TransitState(CoolingDown);
	}
}

void WarmingUpRoutine()
{
	if(PetLeft())
	{
		TransitState(CheckingWeight);
	}
}

void CoolingDownRoutine()
{
	if(PetLeft())
	{
		TransitState(CheckingWeight);
	}
}

volatile state_t budkaState;

void (*stateFunction)(void) = WeightCheckingRoutine;


void TransitState(state_t newState)
{
	switch(newState)
	{
		case CheckingWeight:
			SwitchTemperatureRelay(0);
			budkaState = CheckingWeight;
			stateFunction = WeightCheckingRoutine;
			SwitchStateLEDs(GPIO_PIN_12);
			break;
		case CheckingTemperature:
			SwitchTemperatureRelay(1);
			budkaState = CheckingTemperature;
			stateFunction = TemperatureCheckingRoutine;
			SwitchStateLEDs(GPIO_PIN_13);
			break;
		case WarmingUp:
			secondsElapsed = 0;
			budkaState = WarmingUp;
			stateFunction = WarmingUpRoutine;
			SwitchStateLEDs(GPIO_PIN_14);
			break;
		case CoolingDown:
			secondsElapsed = 0;
			budkaState = CoolingDown;
			stateFunction = CoolingDownRoutine;
			SwitchStateLEDs(GPIO_PIN_15);
			break;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Dimmer_init_begin();
	Dimmer_pin_assign(Dimmer_1, GPIO_PIN_1);
	Dimmer_init_end();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  	//LCD_init();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start(&htim10);
    HAL_TIM_Base_Start(&htim11);
    HAL_UART_Receive_IT(&huart6, receive_buff, maxMessageSize);

    WEIGHT_init();
	//LCD5110_refresh(&lcd1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __disable_irq();
	  currentTemperature = GetCurrentTemperature();
	  //GetCurrentWeight();
	  currentWeight = GetCurrentWeight();
	  __enable_irq();
	  stateFunction();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		 HAL_UART_Transmit(&huart6, stateMessage, errorMessageSize, 10);
		secondsElapsed++;
		if(secondsElapsed < temperatureCheckInterval){return;}

		if(budkaState == WarmingUp)
		{
			if(currentTemperature < temperatureLowBound)
			{
				SetCurrentTemperature(setTemperature + temperatureStep);
			}else
			{
				TransitState(CheckingTemperature);
			}
		}else if(budkaState == CoolingDown)
		{
			if(currentTemperature > temperatureUpperBound)
			{
				SetCurrentTemperature(setTemperature - temperatureStep);
			}else
			{
				TransitState(CheckingTemperature);
			}
		}
	}else if(htim->Instance == TIM1)
	{
		HandleTimerInterrupt();
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		HandleDimmerInterrupt();
	}
}

/**
  * @brief  Returns -127 if interpretation results in failure
  */
int8_t InterpretNumberFromBuffer()
{
	int8_t multiplier = 1;
	int8_t result = 0;

	if(receive_buff[8] == '-')
	{
		multiplier = -1;
	} else if(receive_buff[8] != '+')
	{
		return -127;
	}

	if(receive_buff[9] > '9' || receive_buff[10] > '9' || receive_buff[9] < '0' || receive_buff[10] < '0' ){return -127;}

	result += (receive_buff[9]-'0') * 10;
	result += receive_buff[10] - '0';
	result *= multiplier;

	return result;
}

bool ProcessUserQuery()
{
	if(receive_buff[0] == 'G' && receive_buff[1] == 'E' && receive_buff[2] == 'T')
	{
		lowerBoundMessage[sResponseLineSize - 3] = abs(temperatureLowBound)%10 + '0';
		lowerBoundMessage[sResponseLineSize - 4] = abs(temperatureLowBound)/10 + '0';
		lowerBoundMessage[sResponseLineSize - 5] = temperatureLowBound > 0? '+' : '-';
		HAL_UART_Transmit(&huart6, lowerBoundMessage, sResponseLineSize, 10);

		upperBoundMessage[uResponseLineSize - 3] = temperatureUpperBound%10 + '0';
		upperBoundMessage[uResponseLineSize - 4] = temperatureUpperBound/10 + '0';
		HAL_UART_Transmit(&huart6, upperBoundMessage, uResponseLineSize, 10);

		setTempMessage[uResponseLineSize - 3] = setTemperature%10 + '0';
		setTempMessage[uResponseLineSize - 4] = setTemperature/10 + '0';
		HAL_UART_Transmit(&huart6, setTempMessage, uResponseLineSize, 10);

		curTempMessage[sResponseLineSize - 3] = abs(currentTemperature)%10 + '0';
		curTempMessage[sResponseLineSize - 4] = abs(currentTemperature)/10 + '0';
		curTempMessage[sResponseLineSize - 5] = currentTemperature > 0? '+' : '-';
		HAL_UART_Transmit(&huart6, curTempMessage, sResponseLineSize, 10);

		loadMessage[uResponseLineSize - 3] = abs(currentWeight)%10 + '0';
		loadMessage[uResponseLineSize - 4] = (abs(currentWeight) / 10)%10 + '0';
		loadMessage[uResponseLineSize - 5] = abs(currentWeight)/100 + '0';
		HAL_UART_Transmit(&huart6, loadMessage, uResponseLineSize, 10);

		weightBoundMessage[uResponseLineSize - 3] = petInsideWeightThreshold%10 + '0';
		weightBoundMessage[uResponseLineSize - 4] = petInsideWeightThreshold/10 + '0';
		HAL_UART_Transmit(&huart6, weightBoundMessage, uResponseLineSize, 10);

		presenceMessage[uResponseLineSize - 3] = !PetLeft() + '0';
		HAL_UART_Transmit(&huart6, presenceMessage, uResponseLineSize, 10);

		return true;
	} else if(receive_buff[0] == 'S' && receive_buff[1] == 'E' && receive_buff[2] == 'T' && receive_buff[3] == '+' && receive_buff[7] == '=')
	{
		int8_t valueToSet =  InterpretNumberFromBuffer();
		if(valueToSet == -127)
		{
			return false;
		}

		if(receive_buff[4] == 'L' && receive_buff[5] == 'W' && receive_buff[6] == 'R')
		{
			return SetTemperatureLowerBound(valueToSet);
		} else if(receive_buff[4] == 'U' && receive_buff[5] == 'P' && receive_buff[6] == 'R')
		{
			return SetTemperatureUpperBound(valueToSet);
		}else if(receive_buff[4] == 'W' && receive_buff[5] == 'G' && receive_buff[6] == 'T')
		{
			return SetWeightThreshold(valueToSet);
		} else if(receive_buff[4] == 'T' && receive_buff[5] == 'M' && receive_buff[6] == 'P')
		{
			return SetCurrentTemperature(valueToSet);
		}
	}
	return false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(ProcessUserQuery())
	{
		HAL_UART_Transmit_DMA(&huart6, successMessage, successMessageSize);
	} else
	{
		HAL_UART_Transmit_DMA(&huart6, errorMessage, errorMessageSize);
	}
  HAL_UART_Receive_IT(&huart6, receive_buff, maxMessageSize);
  //HAL_UART_Receive_IT(&huart1, receive_buff, 1);
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
