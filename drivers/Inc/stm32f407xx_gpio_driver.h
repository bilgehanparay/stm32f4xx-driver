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














