/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: May 22, 2022
 *      Author: bilgehan
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"


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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}

/**
 *
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	// First configure SPI_CR1 register
	uint32_t tempreg = 0;


	//1. Configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidi mode bit must be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode bit must be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXOnly bit must be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_TXONLY){
		// TODO:
	}

	// 3. Configure spi serial clock
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*
 *
 * */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

/*
 * Send data over the SPI
 * This is a blocking call!
 **/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		//1. wait until TXE is set
		// while(!(pSPIx->SR & (1 << 1)));
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16 bit DFF
			// load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len = Len-2;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName){
	if(pSPIx->SR & flagName)
		return FLAG_SET;
	return FLAG_RESET;
}




























