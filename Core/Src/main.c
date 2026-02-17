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
//#define SD_CARD_OK	1
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

/* Software RTC - default: 17/02/2026 00:00:00 */
volatile datetime_t sw_rtc = {0, 0, 0, 17, 2, 2026};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DATA_MONITOREO 		"ESTE_11.TXT"

/* ---------- Software RTC ---------- */
void SW_RTC_Increment(void)
{
	const uint8_t days_in_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

	sw_rtc.sec++;
	if(sw_rtc.sec >= 60) {
		sw_rtc.sec = 0;
		sw_rtc.min++;
		if(sw_rtc.min >= 60) {
			sw_rtc.min = 0;
			sw_rtc.hour++;
			if(sw_rtc.hour >= 24) {
				sw_rtc.hour = 0;
				sw_rtc.day++;
				uint8_t max_day = days_in_month[sw_rtc.month - 1];
				if(sw_rtc.month == 2) {
					uint16_t y = sw_rtc.year;
					if((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))
						max_day = 29;
				}
				if(sw_rtc.day > max_day) {
					sw_rtc.day = 1;
					sw_rtc.month++;
					if(sw_rtc.month > 12) {
						sw_rtc.month = 1;
						sw_rtc.year++;
					}
				}
			}
		}
	}
}

/* Inicializa sw_rtc con la fecha/hora de compilacion
 * __DATE__ = "Mmm DD YYYY"  ej: "Feb 17 2026"
 * __TIME__ = "HH:MM:SS"     ej: "14:30:00"      */
void SW_RTC_Init_CompileTime(void)
{
	static const char mon_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	static const char comp_date[] = __DATE__;
	static const char comp_time[] = __TIME__;

	/* Mes */
	char mon_str[4] = {comp_date[0], comp_date[1], comp_date[2], '\0'};
	const char *found = strstr(mon_names, mon_str);
	sw_rtc.month = (found != NULL) ? (uint8_t)((found - mon_names) / 3 + 1) : 1;

	/* Dia (puede tener espacio inicial para dias de 1 digito) */
	sw_rtc.day = (comp_date[4] == ' ')
					? (uint8_t)(comp_date[5] - '0')
							: (uint8_t)((comp_date[4] - '0') * 10 + (comp_date[5] - '0'));

	/* Año */
	sw_rtc.year = (uint16_t)(
			(comp_date[7] - '0') * 1000 +
			(comp_date[8] - '0') * 100  +
			(comp_date[9] - '0') * 10   +
			(comp_date[10]- '0'));

	/* Hora */
	sw_rtc.hour = (uint8_t)((comp_time[0] - '0') * 10 + (comp_time[1] - '0'));
	sw_rtc.min  = (uint8_t)((comp_time[3] - '0') * 10 + (comp_time[4] - '0'));
	sw_rtc.sec  = (uint8_t)((comp_time[6] - '0') * 10 + (comp_time[7] - '0'));

	myprintf("RTC init: %04d-%02d-%02d %02d:%02d:%02d (compile time)\r\n",
			sw_rtc.year, sw_rtc.month, sw_rtc.day,
			sw_rtc.hour, sw_rtc.min,  sw_rtc.sec);
}
/* ---------------------------------- */

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
#ifdef SD_CARD_OK
	FRESULT res; //Result after operations
#endif
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

	myprintf("\r\nABS MEDICION TEMPERATURA - ESTE 11\r\n");

	/* SD Card - crear/verificar archivo de log */
#ifdef SD_CARD_OK
	/*res = Mount_SD("/");
	if(res != FR_OK)
	{
		myprintf("SD mount error (%i) - sin SD card\r\n", res);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Si el archivo no existe lo crea con encabezado CSV 
		if(Update_File(DATA_MONITOREO, "") != FR_OK)
		{
			Create_File(DATA_MONITOREO);
			Update_File(DATA_MONITOREO, "Timestamp,Temp_C,Pressure_Pa\r\n");
		}
		Unmount_SD("/");
		myprintf("SD card OK\r\n");
	}*/
#endif

	/* Inicializar RTC con fecha/hora de compilacion */
	SW_RTC_Init_CompileTime();
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
		myprintf("SENSOR BMP280 OK! ");
	}
	HAL_Delay(500);
	int32_t raw_temp, raw_pressure;
	float temp, pressure;
	HAL_TIM_Base_Start_IT(&htim2);
	estados_abs = WAIT;
	myprintf("\r\nINCIO MEDICION \r\n\r\n");
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
			sprintf(horario, "%04d-%02d-%02d %02d:%02d:%02d",
					sw_rtc.year, sw_rtc.month, sw_rtc.day,
					sw_rtc.hour, sw_rtc.min,  sw_rtc.sec);
#ifdef SD_CARD_OK
			estados_abs = SAVE;
#else
			estados_abs = PRINT;
#endif
			break;
		case PRINT:
			char data_printf[80];
			sprintf(data_printf, "%s,%.2f,%.2f\r\n", horario, temp, pressure);
			myprintf("[%u] %s", idx_t + 1, data_printf);
			estados_abs = WAIT;
			break;
		case SAVE:
		{
			char data_save[80];
			sprintf(data_save, "%s,%.2f,%.2f\r\n", horario, temp, pressure);
			myprintf("[%u] %s", idx_t + 1, data_save);
#ifdef SD_CARD_OK
			res = Mount_SD("/");
			if(res == FR_OK)
			{
				res = Update_File(DATA_MONITOREO, data_save);
				if(res == FR_OK)
				{
					idx_t++;
				}
				else
				{
					myprintf("SD write error (%i)\r\n", res);
				}
				Unmount_SD("/");
			}
			else
			{
				myprintf("SD mount error (%i)\r\n", res);
			}
#endif
			estados_abs = WAIT;
			break;
		}
		case WAIT:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
			HAL_Delay(1000);
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
	SW_RTC_Increment(); /* 1 tick = 1 segundo */
	counter_time++;
	if(counter_time == TOTAL_COUNT)
	{
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
