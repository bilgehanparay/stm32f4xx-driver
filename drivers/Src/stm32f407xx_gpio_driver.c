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

}

/*************************************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Read from given pin from given GPIO Port
 *
 * @param[in]			- HBase address of GPIO Port
 * @param[in]			- pin number
 *
 * @return				- the value of the pin
 *
 * @Note				- none
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

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

}

/*************************************************************************
 * @fn					- GPIO_IRQConfig
 *
 * @brief				- Configure the IRQ
 *
 * @param[in]			- IRQ number to be configured
 * @param[in]			- IRQ priority
 * qparam[in]			- ENABLE or DISABLE
 *
 * @return				- none
 *
 * @Note				- none
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

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

}













