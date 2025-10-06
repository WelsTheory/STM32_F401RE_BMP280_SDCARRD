/*
 * DS18B20.h
 *
 *  Created on: Apr 7, 2025
 *      Author: wlimonchi
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "main.h" // include processor files - each processor file is guarded.
#include <stdint.h>
#include <stdbool.h>

void DS18B20_TIM_START(void);

void delay_us (uint16_t us);

float read_temp_celsius(void);
float read_temp_fahrenheit(void);

void set_data_pin(GPIO_PinState PinState);
void toggle_data_pin(void);

void set_pin_output(void);
void set_pin_input(void);

uint8_t read_data_pin(void);

void start_sensor(void);

void writeData(uint8_t data);
uint8_t read_data(void);


#endif /* INC_DS18B20_H_ */
