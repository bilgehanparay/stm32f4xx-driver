/*
 * 001led_toggle.c
 *
 *  Created on: May 8, 2022
 *      Author: bilgehan
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(uint32_t delay){
	while(delay > 0){
		delay--;
	}
}

int main(void){
	GPIO_Handle_t GpioLED;

	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLED);
	while(1){
		GPIO_TogglePin(GPIOD, 12);
		delay(500000);
	}
	return 0;
}
