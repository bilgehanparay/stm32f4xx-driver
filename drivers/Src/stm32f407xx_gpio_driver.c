/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 7, 2022
 *      Author: bilgehan
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"


/*******************************************
 * APIs supported by this driver
 *******************************************/
/*************************************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- Enables or disables peripheral clock for the given GPIO Port
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_EN();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_DI();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_DI();
		}
	}
}

/*************************************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- Initialize the GPIO Port
 *
 * @param[in]			- Handle to hold base address and GPIO configuration
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0; // temp register

	// Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	// Configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// non interrupt mode
		// look to GPIO Mode Register in the reference manual
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit first
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0; // clear temp
	}else{
		// interrupt mode
		/*Pin must be in input configuration*/
		/*Configure the edge trigger*/
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// Configure the FTSR
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			// clear the RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			// clear the RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			// clear the RTSR bit
			EXTI->RTSR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		}
		/*Configure the GPIO Port Selection SYSCFG_EXTICR*/
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		/*enable interrupt delivery from peripheral to the processor*/
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		/*identify the IRQ number on which the processor accepts the interrupt from that pin*/
		/*Configure the IRQ priority for the identified IRQ number(Processor side)*/
		/*Enable interrupt reception on that IRQ number(Processor side)*/
		/*Implement IRQ handler*/

	}
		// Configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit first
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;
		// configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit first
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp = 0;
		// configure the optype
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bit first
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;
		// configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
			uint32_t temp1, temp2;
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
			pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2);
		}
}

/*************************************************************************
 * @fn					- GPIO_IDenit
 *
 * @brief				- De-Initialize(Reset) the GPIO Port
 *
 * @param[in]			- Handle to hold base address
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_PCLK_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_PCLK_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_PCLK_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_PCLK_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_PCLK_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_PCLK_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_PCLK_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_PCLK_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_PCLK_RESET();
	}else if(pGPIOx == GPIOJ){
		GPIOJ_PCLK_RESET();
	}else if(pGPIOx == GPIOK){
		GPIOK_PCLK_RESET();
	}
}

/*************************************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Read from given pin from given GPIO Port
 *
 * @param[in]			- HBase address of GPIO Port
 * @param[in]			- pin number
 *
 * @return				- the value of the pin 0 or 1
 *
 * @Note				- none
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*************************************************************************
 * @fn					- GPIO_ReadFomInputPort
 *
 * @brief				- Read from given GPIO Port(all pins)
 *
 * @param[in]			- Base address of GPIO Port
 *
 * @return				- the value of the port-16 pins
 *
 * @Note				- none
 * */
uint16_t GPIO_ReadFomInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*************************************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- Write to given pin from given GPIO Port
 *
 * @param[in]			- Base address of GPIO Port,
 * @param[in]			- pin number
 * @param[in]			- value to write to pin, GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*************************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- Write to given GPIO Port
 *
 * @param[in]			- Base address of GPIO Port,
 * @param[in]			- value to write to port
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/*************************************************************************
 * @fn					- GPIO_TogglePin
 *
 * @brief				-Toggle the GPIO Pin
 *
 * @param[in]			- Base address of GPIO Port,
 * @param[in]			- pin number
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*************************************************************************
 * @fn					- GPIO_IRQConfig
 *
 * @brief				- Configure the IRQ
 *
 * @param[in]			- IRQ number to be configured
 * qparam[in]			- ENABLE or DISABLE
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
// Refer to NVIC registers in the processor(not MCU!)

	/*1. Enable/Disable interrupts in NVIC*/
	if(EnOrDi == ENABLE){
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%32) );
		}
	}else{
		if(IRQNumber <= 31){
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber);
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber);
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	/*Set IRQ priority*/
	// Find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx*4) |= (IRQPriority << shift_amount);
}
/*************************************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- IRQ handler for given pin number
 *
 * @param[in]			- GPIO pin number
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_IRQHandling(uint8_t PinNumber){
	// Clear EXT PR Register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}













