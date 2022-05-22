/*
 * stm32f407xx.h
 *
 *  Created on: May 6, 2022
 *      Author: bilgehan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


/*********************** Processor Specific Details ******************/
#define NO_PR_BITS_IMPLEMENTED	4
/*
 *ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t *)0xE000E10c)
/*
 *ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0			((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2		    ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t *)0xE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 * */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)

/*********************** Processor Specific Details ******************/
/*Generic Macros*/
#define __vo volatile
#define ENABLE	1
#define DISABLE	0
#define SET		ENABLE
#define RESET	DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET   RESET
/**
 * Base addresses of flash and SRAM memories
 * */
#define FLASH_BASEADDR		0x08000000U /**/
#define SRAM1_BASEADDR		0x20000000U /**/
#define SRAM2_BASEADDR		0x20001C00U /**/
#define ROM					0x1FFF0000U /**/
#define SRAM				SRAM1_BASEADDR /**/

/**
 * Base addresses of bus domains
 * */
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/**
 * Base addresses of AHB1
 * */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE+0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE+0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE+0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE+0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE+0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE+0x2000)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE+0x2400)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE+0x2800)
#define RCC_BASEADDR		(AHB1PERIPH_BASE+0x3800)

/**
 * Base addresses of AHB2
 * */

/**
 * Base addresses of APB1
 * */
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)
#define SPI3I2S3_BASEADDR	(APB2PERIPH_BASE + 0x3C00)
#define SPI2I2S3_BASEADDR	(APB2PERIPH_BASE + 0x3800)

/**
 * Base addresses of APB2
 * */
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)
#define SPI5_BASEADDR		(APB2PERIPH_BASE + 0x5000)
#define SPI6_BASEADDR		(APB2PERIPH_BASE + 0x5400)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400)


/********Peripheral definition structures*****************/
/**
 * Note: Registers of a peripheral are specific to MCU,
 * Check for Reference Manual of the MCU for each register
 * and address
 * */

typedef struct{
	__vo uint32_t MODER; 	/*GPIO Port Mode Register Offset 0x00*/
	__vo uint32_t OTYPER;	/*GPIO Port Type Register Offset 0x04*/
	__vo uint32_t OSPEEDR;	/*Offset 0x08*/
	__vo uint32_t PUPDR;	/*Offset 0xOC*/
	__vo uint32_t IDR;		/*Offset 0x10*/
	__vo uint32_t ODR;		/*Offset 0x14*/
	__vo uint32_t BSRR;		/*Offset 0x18*/
	__vo uint32_t LCKR;		/*Offset 0x1C*/
	__vo uint32_t AFR[2];	/*Offset 0x20/0x24*/
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR;			/*Offset 0x00*/
	__vo uint32_t PLLCFGR;		/**/
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2;
	__vo uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t RESERVED6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED8;
	__vo uint32_t RESERVED9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED10;
	__vo uint32_t RESERVED11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/**
 * Peripheral definition structure for EXTI
 * */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition for SPI
 * RM Table 129: SPI Register map and reset values
 * */
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2I2S3_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3I2S3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5		((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6		((SPI_RegDef_t*)SPI6_BASEADDR)

/**
 * Clock Enable macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()	(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()	(RCC->AHB1ENR |= (1 << 10))


/**
 * clock Enable macros for I2C peripherals
 * */
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))

/**
 * clock Enable&disable macros for SPI peripherals
 * */
#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= (1 << 13))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 13))
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15))

/**
 * Clock disable macros for GPIO peripherals
 * */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 10))

/*
 * Reset GPIO clock registers
 * */
#define GPIOA_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 0)); (RCC->AHB1ENR &= ~(1 << 0)); }while(0)
#define GPIOB_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 1)); (RCC->AHB1ENR &= ~(1 << 1)); }while(0)
#define GPIOC_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 2)); (RCC->AHB1ENR &= ~(1 << 2)); }while(0)
#define GPIOD_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 3)); (RCC->AHB1ENR &= ~(1 << 3)); }while(0)
#define GPIOE_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 4)); (RCC->AHB1ENR &= ~(1 << 4)); }while(0)
#define GPIOF_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 5)); (RCC->AHB1ENR &= ~(1 << 5)); }while(0)
#define GPIOG_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 6)); (RCC->AHB1ENR &= ~(1 << 6)); }while(0)
#define GPIOH_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 7)); (RCC->AHB1ENR &= ~(1 << 7)); }while(0)
#define GPIOI_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 8)); (RCC->AHB1ENR &= ~(1 << 8)); }while(0)
#define GPIOJ_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 9)); (RCC->AHB1ENR &= ~(1 << 9)); }while(0)
#define GPIOK_PCLK_RESET()	do{ (RCC->AHB1ENR |= (1 << 10)); (RCC->AHB1ENR &= ~(1 << 10)); }while(0)
#endif /* INC_STM32F407XX_H_ */

/*
 * Clock Enable macros for SYSCFG peripheral
 * */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
//TODO: define base address of other devices on this bus
/*
 * Returns port code for given GPIOx base address
 * */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0:\
								  (x == GPIOB) ? 1:\
								  (x == GPIOC) ? 2:\
								  (x == GPIOD) ? 3:\
								  (x == GPIOE) ? 4:\
								  (x == GPIOF) ? 5:\
								  (x == GPIOG) ? 6:\
								  (x == GPIOH) ? 7:0)


/*
 * IRQ number of STM32f407x MCU
 * Refer to Vector Table (table 61) of RM
 * */
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI5_10 40

/**
 * Macros for IRQ priority levels
 * */
#define NVIC_IRQ_PRIO0 0
#define NVIC_IRQ_PRIO1 1
#define NVIC_IRQ_PRIO2 2
#define NVIC_IRQ_PRIO3 3
#define NVIC_IRQ_PRIO4 4
#define NVIC_IRQ_PRIO5 5
#define NVIC_IRQ_PRIO6 6
#define NVIC_IRQ_PRIO7 7
#define NVIC_IRQ_PRIO8 8
#define NVIC_IRQ_PRIO9 9
#define NVIC_IRQ_PRIO10 10
#define NVIC_IRQ_PRIO11 11
#define NVIC_IRQ_PRIO12 12
#define NVIC_IRQ_PRI113 13
#define NVIC_IRQ_PRIO14 14
#define NVIC_IRQ_PRIO15 15

/*************************************************************
 * Bit position definitions of SPI peripheral
 *************************************************************/

/*
 * CR1 bit positions, Reference Manual section 28.5.1
 * */
#define SPI_CR1_CPHA      0
#define SPI_CR1_CPOL      1
#define SPI_CR1_BR        3
#define SPI_CR1_SSM       9
#define SPI_CR1_RXONLY   10
#define SPI_CR1_DFF	     11
#define SPI_CR1_BIDIOE   14
#define SPI_CR1_BIDIMODE 15





#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
























