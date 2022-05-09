/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 7, 2022
 *      Author: bilgehan
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx; /*Hold the base address to the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * GPIO pin possible modes
 * */
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4 // rising trigger
#define GPIO_MODE_IT_RT 5 // falling trigger
#define GPIO_MODE_IT_RFT 6// rising and falling register

/*
 * GPIO pin possible output types
 * */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/*
 * GPIO pin possible output speed modes
 * */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

/*
 * GPIO pin pull-up/pull down macros
 * */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

/**
 * GPIO pin numbers
 * */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/*******************************************
 * APIs supported by this driver
 *******************************************/
/*
 * Perip. Clock Settings
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi);

/*
 * Port Init/Deinit functions
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read&Write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFomInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Port IRQ configuration and ISR handling
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

/**
 * GPIO Driver API Requirements
 * 1. GPIO Initialization
 * 2. Enable/Disable GPIO Port Clock
 * 3. Read from a GPIO pin
 * 4. Write to a GPIO pin
 * 5. Configure alternate functionality
 * 6. Interrupt Handling
 * */


// TODO: Enumarate types in the pin config











