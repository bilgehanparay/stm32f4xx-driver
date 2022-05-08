/*
 * 001led_toggle.c
 *
 *  Created on: May 8, 2022
 *      Author: bilgehan
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#define HIGH 1
#define BTN_PRESSED HIGH
void delay(uint32_t delay){
	while(delay > 0){
		delay--;
	}
}

int main(void){
	GPIO_Handle_t GpioLED, GPIOBtn;
	uint8_t val;

	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLED);

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1){
		val = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0);
		if(val == BTN_PRESSED){
			delay(250000);
			GPIO_TogglePin(GPIOD, 12);
		}

	}
	return 0;
}
