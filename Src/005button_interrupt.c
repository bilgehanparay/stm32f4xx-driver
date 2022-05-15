/*
 * 001led_toggle.c
 *
 *  Created on: May 8, 2022
 *      Author: bilgehan
 */

#include "string.h"
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
	memset(&GpioLED, 0, sizeof(GpioLED)); // init. to zero
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));
	uint8_t val;

	// Led configuration
	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLED);

	// Button Configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOBtn);

	// IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	return 0;
}

void EXTI0_IRQHandler(void){
	delay(500000);
	GPIO_IRQHandling(GPIO_PIN_NO_0); // clears the Pending register bit
	GPIO_TogglePin(GPIOD, GPIO_PIN_NO_12);
}








