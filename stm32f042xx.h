/*
 * stm32f042xx.h
 *
 *  Created on: May 25, 2023
 *      Author: kneaj
 */

#ifndef STM32F042XX_H_
#define STM32F042XX_H_

#define __I     volatile const       /*!< Defines 'read only' permissions */
#define __O     volatile             /*!< Defines 'write only' permissions */
#define __IO    volatile             /*!< Defines 'read / write' permissions */


#define APB				0x40000000UL
#define AHB1				0x40020000UL
#define AHB2				0x48000000UL

/* APB Peripheral Offsets */
#define TIM2_OFFSET			0x0UL
#define TIM3_OFFSET			0x400UL
#define TIM14_OFFSET			0x2000UL
#define RTC_OFFSET			0x2800UL
#define WWDG_OFFSET			0x2C00UL
#define IWDG_OFFSET			0x3000UL
#define SPI2_OFFSET			0x3800UL
#define USART2_OFFSET			0x4400UL
#define I2C1_OFFSET			0x5400UL
#define USB_OFFSET			0x5C00UL
#define USB_CAN_RAM_OFFSET		0x6000UL
#define BXCAN_OFFSET			0x6400UL
#define CRS_OFFSET			0x6C00UL
#define PWR_OFFSET			0x7000UL
#define CEC_OFFSET			0x7800UL
#define SYSCFG_OFFSET			0x10000UL
#define EXTI_OFFSET			0x10400UL
#define ADC_OFFSET			0x12400UL
#define TIM1_OFFSET			0x12C00UL
#define SPI1_I2S1_OFFSET		0x13000UL
#define USART1_OFFSET			0x13800UL
#define TIM16_OFFSET			0x14400UL
#define TIM17_OFFSET			0x14800UL
#define DBGMCU_OFFSET			0x15800UL

/* AHB1 Peripheral Offsets */
#define DMA_OFFSET			0x0UL
#define RCC_OFFSET			0x1000UL
#define FMI_OFFSET			0x2000UL
#define CRC_OFFSET			0x3000UL
#define TSC_OFFSET			0x4000UL

/* AHB2 Peripheral Offsets */
#define GPIOA_OFFSET			0x0UL
#define GPIOB_OFFSET			0x400UL
#define GPIOC_OFFSET			0x800UL
#define GPIOF_OFFSET			0x1400UL



typedef struct
{
	__IO uint32_t RCC_CR;
	__IO uint32_t RCC_CFGR;
	__IO uint32_t RCC_CIR;
	__IO uint32_t RCC_APB2RSTR;
	__IO uint32_t RCC_APB1RSTR;
	__IO uint32_t RCC_AHBENR;
	__IO uint32_t RCC_APB2ENR;
	__IO uint32_t RCC_APB1ENR;
	__IO uint32_t RCC_BDCR;
	__IO uint32_t RCC_CSR;
	__IO uint32_t RCC_AHBRSTR;
	__IO uint32_t RCC_CFGR2;
	__IO uint32_t RCC_CFGR3;
	__IO uint32_t RCC_CR2;
} RCC_t;


typedef struct
{
	__IO uint32_t MODER;
	__IO uint32_t OTYPER;
	__IO uint32_t OSPEEDR;
	__IO uint32_t PUPDR;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t LCKR;
	__IO uint32_t AFRL;
	__IO uint32_t AFRH;
	__IO uint32_t BRR;
} GPIO_t;



/* AHB1 Peripheral Base Addresses */
#define RCC_BASE			(AHB1 + RCC_OFFSET)

/* AHB2 Peripheral Base Addresses */
#define GPIOA_BASE			(AHB2 + GPIOA_OFFSET)
#define GPIOB_BASE			(AHB2 + GPIOB_OFFSET)
#define GPIOC_BASE			(AHB2 + GPIOC_OFFSET)
#define GPIOF_BASE			(AHB2 + GPIOF_OFFSET)


/* Peripheral Structure Overlay Pointers */
#define RCC				((RCC_t *) RCC_BASE)
#define GPIOA				((GPIO_t *) GPIOA_BASE)
#define GPIOB				((GPIO_t *) GPIOB_BASE)
#define GPIOC				((GPIO_t *) GPIOC_BASE)
#define GPIOF				((GPIO_t *) GPIOF_BASE)



#endif /* STM32F042XX_H_ */
