/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS18B20.h"
#include "fatfs.h"
#include "File_Handling_RTOS.h"
#include "SSD1306.h"
#include "Text_Font.h"
#include "BMP280.h"
#include "DS1307.h"
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
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DATA_MONITOREO 		"ESTE_11.TXT"

void myprintf(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

float temperature_1 = 0.0;

char data_temp[18];

char horario[40];

volatile uint16_t idx_t = 0;

volatile uint32_t start_timer = 0;
volatile uint32_t counter_time = 0;

state_t estados_abs;

float pressure, temp_bmp, humidity;

uint8_t date = 0;
uint8_t month = 0;
uint16_t year = 0;
uint8_t dow = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t second = 0;
int8_t zone_hr = 0;
uint8_t zone_min = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	FRESULT res; //Result after operations
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	for(uint8_t i = 0; i<6;i++)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port,USER_LED_Pin);
		HAL_Delay(500);
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

	myprintf("\r\n ABS MEDICION TEMPERATURA - ESTE 11\r\n");
	res = Mount_SD("/");
	if(res != FR_OK)
	{
		myprintf("f_mount error (%i)\r\n", res);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		while(1);
	}
	if(Update_File(DATA_MONITOREO,"MEDICION ESTE 11!\r\n") != FR_OK)
	{
		Create_File(DATA_MONITOREO);
		Update_File(DATA_MONITOREO,"MEDICION ESTE 11 !\r\n");
	}
	Create_File(DATA_MONITOREO);
	Unmount_SD("/");

	myprintf("\r\n SD CARD OK! \r\n");

	//DS18B20_TIM_START();

	estados_abs = OFF;

	/* SSD1306 */
	/*ssd1306_t SSD1306;
	SSD1306.address = SSD1306_ADDR<<1;
	SSD1306.width= SSD1306_WIDTH ;
	SSD1306.height = SSD1306_HEIGHT;
	SSD1306.color = SSD1306_WHITE;
	SSD1306.contrast = 0xCF;
	SSD1306.left = SSD1306_LEFT;
	SSD1306.right = SSD1306_RIGHT;
	SSD1306.center = SSD1306_CENTER;
	SSD1306.Font.inverted = false;
	SSD1306_I2C_Init (&SSD1306);
	HAL_Delay(100);
	SSD1306_I2C_ClearDisplay(&SSD1306);
	SSD1306_I2C_Update(&SSD1306);
	SSD1306_I2C_SetFont( &SSD1306, BIG_FONTS);
	SSD1306_I2C_Cadena( &SSD1306,0,0,"WELS XD");
	SSD1306_I2C_SetFont(&SSD1306, TINY_FONTS);
	SSD1306_I2C_Cadena( &SSD1306, 0,32,"TEMPERATUR");
	SSD1306_I2C_Update( &SSD1306);
	HAL_Delay(100);
	SSD1306_I2C_ClearDisplay(&SSD1306);*/

	/* BMP280 */
	bmp280_t bmp;
	bmp.comm_mode = BMP280_MODE_I2C;
	bmp.hi2c = &hi2c1;
	bmp.address = 0x76;
	if(bmp280_init(&bmp) == HAL_OK)
	{
		myprintf("OK TODO BIEN ");
	}
	HAL_Delay(500);
	int32_t raw_temp, raw_pressure;
	float temp, pressure;
	/* DS1307 */
	DS1307_Init(&hi2c1);
	/* To test leap year correction. */

	DS1307_SetDayOfWeek(4);
	DS1307_SetTimeZone(-5, 00);
	DS1307_SetDate(22);
	DS1307_SetMonth(05);
	DS1307_SetYear(2025);
	DS1307_SetHour(9);
	DS1307_SetMinute(26);
	DS1307_SetSecond(01);
	date = DS1307_GetDate();
	month = DS1307_GetMonth();
	year = DS1307_GetYear();
	hour = DS1307_GetHour();
	minute = DS1307_GetMinute();
	second = DS1307_GetSecond();
	/*
	 * SIN PULSADOR
	 * */
	HAL_TIM_Base_Start_IT(&htim2);
	estados_abs = WAIT;
	myprintf("\r\n START TIMER \r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		switch(estados_abs)
		{
		case WORK:
			if(HAL_OK == bmp280_read_raw(&bmp,&raw_temp,&raw_pressure))
			{
				temp = bmp280_compensate_temperature(&bmp,raw_temp);
				pressure = bmp280_compensate_pressure(&bmp, raw_pressure);
				myprintf("Pressure: %.2f Pa, Temperature: %.2f C\r\n",pressure,temp);
				sprintf(data_temp,"%.2f, %.2f",temp,pressure);
				estados_abs = HOUR;
			}
			else
			{
				myprintf("Temperature/pressure reading failed\r\n");
				HAL_Delay(2000);
				estados_abs = OFF;
			}
			break;
		case HOUR:
			date = DS1307_GetDate();
			month = DS1307_GetMonth();
			year = DS1307_GetYear();
			hour = DS1307_GetHour();
			minute = DS1307_GetMinute();
			second = DS1307_GetSecond();
			sprintf(horario,"%04d-%02d-%02d, %02d:%02d:%02d,",
					year, month, date, hour, minute, second);
			myprintf("%04d-%02d-%02d %02d:%02d:%02d  \r\n",
					year, month, date, hour, minute, second);
			estados_abs = SAVE;
			break;
		case SAVE:
			char data_save[100];
			sprintf(data_save," %s ,%s\r\n",horario, data_temp);
			idx_t = idx_t + 1;
			res = Mount_SD("/");
			if(res != FR_OK)
			{
				myprintf("f_mount error (%i)\r\n", res);
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
				while(1);
			}
			res = Update_File(DATA_MONITOREO,data_save);
			if(res != FR_OK)
			{
				myprintf("f_mount error (%i)\r\n", res);
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
				while(1);
			}
			Unmount_SD("/");
			HAL_Delay(2000);
			estados_abs = WAIT;
			break;
		case WAIT:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
			HAL_Delay(500);
			break;
		default:
			break;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == Button_Pin)
	{
		if(start_timer == 0)
		{
			HAL_TIM_Base_Start_IT(&htim2);
			estados_abs = WAIT;
			myprintf("\r\n START TIMER \r\n\r\n");
			start_timer = 1;
		}
		else
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			start_timer = 0;
			estados_abs = OFF;
			myprintf("\r\n OFF TIMER \r\n\r\n");
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	counter_time++;
	if(counter_time == TOTAL_COUNT)
	{
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		counter_time = 0;
		estados_abs = WORK;
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
