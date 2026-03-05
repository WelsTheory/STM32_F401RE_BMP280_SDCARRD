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
#include <stdbool.h>
#include "DS18B20.h"
#include "fatfs.h"
#include "File_Handling_RTOS.h"
#include "SSD1306.h"
#include "Text_Font.h"
#include "BMP280.h"
// #include "DS1307.h"  // Ya no se usa DS1307, se usa registro de compilación
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SD_CARD_OK	1
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

/* DS1307 RTC - Hardware I2C RTC - YA NO SE USA */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END FP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DATA_MONITOREO 		"NORTE_11.TXT"

/* ========== DS1307 RTC - DESHABILITADO - Se usa registro de compilación ========== */
/*
// Configuracion DS1307 - Descomentar para FORZAR configuracion del RTC en cada inicio
// Usar solo si necesitas actualizar la hora manualmente
// #define FORCE_DS1307_CONFIG

// ---------- DS1307 RTC Functions ----------
// Funcion para forzar escritura directa del DS1307 (bypass Clock Halt)
void DS1307_ForceWrite(uint16_t year, uint8_t month, uint8_t day,
                       uint8_t hour, uint8_t minute, uint8_t second)
{
	myprintf("Forzando escritura directa al DS1307 (bypass Clock Halt)...\r\n");

	// Escribir directamente cada registro con valores en BCD
	uint8_t data[8];
	data[0] = 0x00;  // Dirección de inicio (SECOND)
	data[1] = DS1307_EncodeBCD(second) & 0x7F;  // SECOND con CH=0 (bit 7 = 0)
	data[2] = DS1307_EncodeBCD(minute);         // MINUTE
	data[3] = DS1307_EncodeBCD(hour);           // HOUR (24h mode)
	data[4] = 0x01;                             // DOW (día de semana, 1=Domingo)
	data[5] = DS1307_EncodeBCD(day);            // DATE
	data[6] = DS1307_EncodeBCD(month);          // MONTH
	data[7] = DS1307_EncodeBCD(year - 2000);    // YEAR (00-99)

	// Escribir todos los registros de tiempo en una sola transacción
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS1307_I2C_ADDR << 1, data, 8, 1000);

	if(status == HAL_OK)
	{
		myprintf("Escritura I2C OK\r\n");
		HAL_Delay(50);

		// Verificar
		myprintf("Verificando escritura...\r\n");
		uint8_t sec_reg = DS1307_GetRegByte(DS1307_REG_SECOND);
		myprintf("  SECOND reg = 0x%02X (CH bit=%d)\r\n", sec_reg, (sec_reg >> 7) & 1);
	}
	else
	{
		myprintf("ERROR escritura I2C: %d\r\n", status);
	}
}

// Funcion de debug: Lee y muestra registros raw del DS1307
void DS1307_DebugRegisters(void)
{
	myprintf("DS1307 Registros RAW (hex):\r\n");
	myprintf("  SEC  [0x00]: 0x%02X (BCD)\r\n", DS1307_GetRegByte(DS1307_REG_SECOND));
	myprintf("  MIN  [0x01]: 0x%02X (BCD)\r\n", DS1307_GetRegByte(DS1307_REG_MINUTE));
	myprintf("  HOUR [0x02]: 0x%02X (BCD)\r\n", DS1307_GetRegByte(DS1307_REG_HOUR));
	myprintf("  DOW  [0x03]: 0x%02X\r\n", DS1307_GetRegByte(DS1307_REG_DOW));
	myprintf("  DATE [0x04]: 0x%02X (BCD)\r\n", DS1307_GetRegByte(DS1307_REG_DATE));
	myprintf("  MON  [0x05]: 0x%02X (BCD)\r\n", DS1307_GetRegByte(DS1307_REG_MONTH));
	myprintf("  YEAR [0x06]: 0x%02X (BCD, 00-99 = 2000-2099)\r\n", DS1307_GetRegByte(DS1307_REG_YEAR));
	myprintf("  CTRL [0x07]: 0x%02X\r\n", DS1307_GetRegByte(DS1307_REG_CONTROL));

	// Decodificacion BCD de valores importantes
	myprintf("Decodificado: %02d:%02d:%02d  %02d/%02d/20%02d\r\n",
			DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_HOUR) & 0x3F),
			DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_MINUTE)),
			DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_SECOND) & 0x7F),
			DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_DATE)),
			DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_MONTH)),
			DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_YEAR)));
}

// Funcion opcional para configurar la hora del DS1307 con la fecha/hora de compilacion
// Descomenta la llamada a esta funcion si necesitas actualizar el RTC
// __DATE__ = "Mmm DD YYYY"  ej: "Feb 17 2026"
// __TIME__ = "HH:MM:SS"     ej: "14:30:00"
void DS1307_SetCompileTime(void)
{
	static const char mon_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	static const char comp_date[] = __DATE__;
	static const char comp_time[] = __TIME__;

	uint8_t month, day, hour, minute, second;
	uint16_t year;

	// Mes
	char mon_str[4] = {comp_date[0], comp_date[1], comp_date[2], '\0'};
	const char *found = strstr(mon_names, mon_str);
	month = (found != NULL) ? (uint8_t)((found - mon_names) / 3 + 1) : 1;

	// Dia (puede tener espacio inicial para dias de 1 digito)
	day = (comp_date[4] == ' ')
			? (uint8_t)(comp_date[5] - '0')
			: (uint8_t)((comp_date[4] - '0') * 10 + (comp_date[5] - '0'));

	// Año
	year = (uint16_t)(
			(comp_date[7] - '0') * 1000 +
			(comp_date[8] - '0') * 100  +
			(comp_date[9] - '0') * 10   +
			(comp_date[10]- '0'));

	// Hora
	hour   = (uint8_t)((comp_time[0] - '0') * 10 + (comp_time[1] - '0'));
	minute = (uint8_t)((comp_time[3] - '0') * 10 + (comp_time[4] - '0'));
	second = (uint8_t)((comp_time[6] - '0') * 10 + (comp_time[7] - '0'));

	// Configurar el DS1307
	myprintf("Escribiendo: Año=%d, Mes=%d, Dia=%d, Hora=%d:%d:%d\r\n",
			year, month, day, hour, minute, second);

	// Primero asegurar que el reloj esté detenido para escritura segura
	DS1307_SetClockHalt(1);
	HAL_Delay(10);

	// Escribir todos los registros
	DS1307_SetYear(year);
	DS1307_SetMonth(month);
	DS1307_SetDate(day);
	DS1307_SetHour(hour);
	DS1307_SetMinute(minute);
	DS1307_SetSecond(second);

	// Iniciar el reloj
	DS1307_SetClockHalt(0);
	HAL_Delay(50);

	// Verificar lo que realmente se escribio
	myprintf("DS1307 RTC configurado: %04d-%02d-%02d %02d:%02d:%02d\r\n",
			year, month, day, hour, minute, second);
	myprintf("Verificando lectura inmediata...\r\n");
	DS1307_DebugRegisters();

	// Verificar que el reloj esté corriendo
	if(DS1307_GetClockHalt() != 0)
	{
		myprintf("ERROR: El reloj sigue detenido despues de configurar!\r\n");
	}
	else
	{
		myprintf("OK: Reloj corriendo correctamente.\r\n");
	}
}
// ----------------------------------
*/
/* ========== FIN DS1307 DESHABILITADO ========== */

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

	myprintf("\r\nABS MEDICION TEMPERATURA \r\n");

	/* SD Card - crear/verificar archivo de log */
#ifdef SD_CARD_OK
	res = Mount_SD("/");
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
	}

#endif

	/* ========== DS1307 RTC - DESHABILITADO ========== */
	/*
	// Inicializar DS1307 RTC
	myprintf("\r\n--- Inicializando DS1307 RTC ---\r\n");

	// Escaneo de bus I2C para diagnóstico
	myprintf("Escaneando bus I2C1...\r\n");
	uint8_t devices_found = 0;
	for(uint8_t addr = 1; addr < 128; addr++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
		{
			myprintf("  Dispositivo encontrado en 0x%02X\r\n", addr);
			devices_found++;
		}
	}
	myprintf("Dispositivos I2C encontrados: %d\r\n\r\n", devices_found);

	// Test de comunicacion I2C con DS1307
	if(HAL_I2C_IsDeviceReady(&hi2c1, DS1307_I2C_ADDR << 1, 3, 1000) == HAL_OK)
	{
		myprintf("DS1307 detectado en I2C (0x%02X)\r\n", DS1307_I2C_ADDR);

		// Verificar Clock Halt ANTES de Init
		uint8_t halt_before = DS1307_GetClockHalt();
		myprintf("Clock Halt Bit (antes de Init): %d %s\r\n", halt_before, halt_before ? "(DETENIDO)" : "(CORRIENDO)");

		// Inicializar y forzar inicio del reloj
		DS1307_Init(&hi2c1);
		HAL_Delay(10);

		// Verificar Clock Halt DESPUES de Init
		uint8_t halt_after = DS1307_GetClockHalt();
		myprintf("Clock Halt Bit (despues de Init): %d %s\r\n", halt_after, halt_after ? "(DETENIDO)" : "(CORRIENDO)");

		// Si sigue detenido, forzar limpieza del bit
		if(halt_after != 0)
		{
			myprintf("ADVERTENCIA: Reloj sigue detenido. Forzando inicio...\r\n");
			DS1307_SetClockHalt(0);
			HAL_Delay(10);
			uint8_t halt_final = DS1307_GetClockHalt();
			myprintf("Clock Halt Bit (despues de forzar): %d %s\r\n", halt_final, halt_final ? "(AUN DETENIDO - PROBLEMA HW)" : "(CORRIENDO OK)");

			if(halt_final != 0)
			{
				myprintf("\r\n*** PROBLEMA DE HARDWARE DETECTADO ***\r\n");
				myprintf("El DS1307 no responde a comandos de escritura.\r\n");
				myprintf("Posibles causas:\r\n");
				myprintf("  1. Falta bateria CR2032 en el DS1307\r\n");
				myprintf("  2. Resistencias pull-up faltantes en SDA/SCL (necesita 4.7k ohm)\r\n");
				myprintf("  3. Conexiones incorrectas o cables sueltos\r\n");
				myprintf("  4. DS1307 defectuoso\r\n");
				myprintf("  5. Conflicto con BMP280 en el mismo bus I2C\r\n");
				myprintf("\r\nVerifica las conexiones:\r\n");
				myprintf("  DS1307 VCC  -> 5V o 3.3V\r\n");
				myprintf("  DS1307 GND  -> GND\r\n");
				myprintf("  DS1307 SDA  -> PB9 (con pull-up 4.7k a VCC)\r\n");
				myprintf("  DS1307 SCL  -> PB8 (con pull-up 4.7k a VCC)\r\n");
				myprintf("  DS1307 BAT  -> Bateria CR2032 (+)\r\n");
				myprintf("  DS1307 GND  -> Bateria CR2032 (-)\r\n");
				myprintf("***********************************\r\n\r\n");
			}
		}

		// Leer fecha/hora actual
		uint16_t year = DS1307_GetYear();
		uint8_t month = DS1307_GetMonth();
		uint8_t day = DS1307_GetDate();
		uint8_t hour = DS1307_GetHour();
		uint8_t minute = DS1307_GetMinute();
		uint8_t second = DS1307_GetSecond();

		myprintf("Fecha/Hora leida: %04d-%02d-%02d %02d:%02d:%02d\r\n",
				year, month, day, hour, minute, second);

		// Mostrar registros raw para debug
		DS1307_DebugRegisters();

#ifdef FORCE_DS1307_CONFIG
		// Configuracion forzada habilitada
		myprintf("FORCE_DS1307_CONFIG activo. Configurando RTC...\r\n");
		DS1307_SetCompileTime();
		HAL_Delay(100);
		myprintf("RTC configurado. Nueva fecha/hora: %04d-%02d-%02d %02d:%02d:%02d\r\n",
				DS1307_GetYear(), DS1307_GetMonth(), DS1307_GetDate(),
				DS1307_GetHour(), DS1307_GetMinute(), DS1307_GetSecond());
#else
		// Detectar si el RTC necesita configuracion inicial
		// Si el año es inválido (2000-2024 no tiene sentido, o fuera de rango), configurar
		// Detectar año corrupto: 2080 indica problema de lectura
		bool year_corrupted = (year == 2080 || year == 2000);

		if(year < 2025 || year > 2099 || month == 0 || month > 12 || day == 0 || day > 31 || year_corrupted)
		{
			myprintf("ADVERTENCIA: RTC con valores invalidos (año=%d). Configurando...\r\n", year);

			// Limpiar registro de control (puede tener basura)
			DS1307_SetRegByte(DS1307_REG_CONTROL, 0x00);
			HAL_Delay(10);

			// Intento 1: Configurar con funciones normales
			myprintf("Intento 1: Configuracion normal...\r\n");
			DS1307_SetCompileTime();
			HAL_Delay(100);

			// Leer de nuevo para verificar
			uint16_t new_year = DS1307_GetYear();
			uint8_t new_month = DS1307_GetMonth();
			uint8_t new_day = DS1307_GetDate();
			uint8_t new_hour = DS1307_GetHour();
			uint8_t new_min = DS1307_GetMinute();
			uint8_t new_sec = DS1307_GetSecond();

			myprintf("RTC configurado. Nueva fecha/hora: %04d-%02d-%02d %02d:%02d:%02d\r\n",
					new_year, new_month, new_day, new_hour, new_min, new_sec);

			// Si sigue fallando, intentar escritura forzada
			if(new_year < 2025 || new_year > 2099 || DS1307_GetClockHalt() != 0)
			{
				myprintf("FALLO: Configuracion normal no funciono. Intentando escritura forzada...\r\n");

				// Obtener fecha/hora de compilación nuevamente
				static const char mon_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
				static const char comp_date[] = __DATE__;
				static const char comp_time[] = __TIME__;

				char mon_str[4] = {comp_date[0], comp_date[1], comp_date[2], '\0'};
				const char *found = strstr(mon_names, mon_str);
				uint8_t m = (found != NULL) ? (uint8_t)((found - mon_names) / 3 + 1) : 3;
				uint8_t d = (comp_date[4] == ' ') ? (uint8_t)(comp_date[5] - '0')
						: (uint8_t)((comp_date[4] - '0') * 10 + (comp_date[5] - '0'));
				uint16_t y = (uint16_t)((comp_date[7] - '0') * 1000 + (comp_date[8] - '0') * 100
						+ (comp_date[9] - '0') * 10 + (comp_date[10]- '0'));
				uint8_t h = (uint8_t)((comp_time[0] - '0') * 10 + (comp_time[1] - '0'));
				uint8_t min = (uint8_t)((comp_time[3] - '0') * 10 + (comp_time[4] - '0'));
				uint8_t s = (uint8_t)((comp_time[6] - '0') * 10 + (comp_time[7] - '0'));

				DS1307_ForceWrite(y, m, d, h, min, s);

				HAL_Delay(100);
				myprintf("Verificacion final: %04d-%02d-%02d %02d:%02d:%02d\r\n",
						DS1307_GetYear(), DS1307_GetMonth(), DS1307_GetDate(),
						DS1307_GetHour(), DS1307_GetMinute(), DS1307_GetSecond());
			}
		}
		else
		{
			myprintf("DS1307 RTC OK!\r\n");
		}
#endif
	}
	else
	{
		myprintf("ERROR: DS1307 NO detectado en I2C!\r\n");
		myprintf("Verificar conexiones: SDA, SCL, VCC, GND\r\n");
		myprintf("Direccion I2C esperada: 0x%02X (0x%02X shifted)\r\n",
				DS1307_I2C_ADDR, DS1307_I2C_ADDR << 1);
	}
	myprintf("--- Fin inicializacion DS1307 ---\r\n\r\n");
	*/

	/* Usar registro de compilacion en lugar de DS1307 */
	myprintf("\r\n--- Usando registro de compilacion ---\r\n");
	myprintf("Fecha de compilacion: %s\r\n", __DATE__);
	myprintf("Hora de compilacion: %s\r\n", __TIME__);
	myprintf("--- Fin registro de compilacion ---\r\n\r\n");

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
			/* Usar fecha y hora de compilacion (ya no se usa DS1307) */
			sprintf(horario, "%s, %s", __DATE__, __TIME__);
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
	/* Timer tick cada 1 segundo - Se usa registro de compilacion */
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
