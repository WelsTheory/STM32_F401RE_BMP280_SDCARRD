/*
 * DS18B20.c
 *
 *  Created on: Apr 7, 2025
 *      Author: wlimonchi
 */

#include "DS18B20.h"

extern TIM_HandleTypeDef htim1;

void DS18B20_TIM_START(void)
{
	HAL_TIM_Base_Start(&htim1);
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void set_pin_output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void set_pin_input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void set_data_pin(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, PinState);
}

void toggle_data_pin(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

uint8_t read_data_pin(void)
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void start_sensor(void)
{
	set_pin_output();
	set_data_pin(0);
	delay_us(480);
	set_pin_input();
	delay_us(80);
	read_data_pin();
	delay_us(400);
}

void writeData(uint8_t data)
{
	set_pin_output();
	for (uint8_t i = 0; i < 8; i++)
	{
		if (data & (1 << i))
		{
			set_pin_output();
			set_data_pin(0);
			delay_us(1);
			set_pin_input();
			delay_us(60);
			continue;
		}
		set_pin_output();
		set_data_pin(0);
		delay_us(60);
		set_pin_input();
	}
}

uint8_t read_data(void)
{
	uint8_t value = 0;
	set_pin_input();
	for (uint8_t i = 0; i < 8; i++)
	{
		set_pin_output();
		set_data_pin(0);
		delay_us(2);
		set_pin_input();
		if (read_data_pin())
		{
			value |= 1 << i;
		}
		delay_us(60);
	}
	return value;
}

/*
Read the current temperature from the sensor.
This functions blocks for around 800ms as it waits for the conversion time!
@return Temperature in degrees Celsius
 */
float read_temp_celsius(void)
{
	start_sensor();
	HAL_Delay(1);
	writeData(0xCC);
	writeData(0x44);
	HAL_Delay(800);
	start_sensor();
	writeData(0xCC);
	writeData(0xBE);
	uint8_t temp1 = read_data();
	uint8_t temp2 = read_data();
	uint16_t temp_com = (temp2 << 8) | temp1;
	return (float)(temp_com / 16.0);
}

/*
Read the current temperature from the sensor.
This functions blocks for around 800ms as it waits for the conversion time!
@return Temperature in degrees Fahrenheit
 */
float read_temp_fahrenheit(void)
{
	return read_temp_celsius() * 1.8 + 32.0;
}
