/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: May 22, 2022
 *      Author: bilgehan
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

/*
 * Configuration structure for
 * SPIx peripheral
 * */
typedef struct{
	uint8_t SPI_DeviceMode;	/*SPI slave or master*/
	uint8_t SPI_BusConfig;  /**/
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/**
 * @SPI_DeviceMode
 * */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 1
/**
 *@SPI_BusConfig
 **/
#define SPI_BUS_CONFIG_FD	1
#define SPI_BUS_CONFIG_HD	2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	4

/**
 * @SPI_SclkSpeed
 * */
#define SPI_SCLK_SPEED_DIV2 0
#define SPI_SCLK_SPEED_DIV4 1
#define SPI_SCLK_SPEED_DIV8 2
#define SPI_SCLK_SPEED_DIV16 3
#define SPI_SCLK_SPEED_DIV32 4
#define SPI_SCLK_SPEED_DIV64 5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV256 7
/*
 *@SPI_CPHA
 **/
#define SPI_CPHA_LOW 0
#define SPI_CPHA_HIGH 1

/*
 *@SPI_DFF
 **/
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

/**
 *@SPI_CPOL
 **/
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/**
 *@SPI_SSM
 **/
#define SPI_SSM_EN 1
#define SPI_SSM_DI 0


/**
 * @SPI_CR
 * */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE     	6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/**
 *@SPI_SR
 **/
#define SPI_SR_TXE 		1
#define SPI_SR_RXNE 	0
#define SPI_SR_BSY 		7
#define SPI_SR_CHSIDE 	2
#define SPI_SR_UDR 		3
#define SPI_SR_CRC_ERR 	4
#define SPI_SR_MODF 	5
#define SPI_SR_OVR 		6
#define SPI_SR_FRE 		8


/**
 * SPI related flag definitions
 * */
#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/*
 * Perip. Clock Settings
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pGPIOx, uint8_t enOrDi);

/*
 * SPI Init/Deinit functions
 * */
void SPI_Init(SPI_Handle_t *pSPIHandlex);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * Data Send&Receive Functions
 * */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);


/**
 * IRQ Configuration and ISR Handling
 * */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRqPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/**
 * Other peripheral controls
 * */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * Generic functions
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
#endif /* INC_STM32F4XX_SPI_DRIVER_H_*/












