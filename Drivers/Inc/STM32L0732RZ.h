/*
 * STM32L0732RZ.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Test
 */


#include <stdint.h>
#include <stddef.h>
#ifndef INC_STM32L0732RZ_H_
#define INC_STM32L0732RZ_H_

#define __vo					volatile //Volatile assignment shortcut
#define __weak					__attribute__((weak))

/********************************************Start: Processor Specific Details****************
 *
 * ARM Cortex M0+ Processor NVIC ISER, ICER and Priority register Addresses
 */
#define NVIC_ISER				((__vo uint32_t*)0xE000E100)
#define NVIC_ICER				((__vo uint32_t*)0xE000E180)
#define	NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			2
/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U 		//Base address of Flash Memory
#define SRAM_BASEADDR			0x20000000U			//Base address of SRAM Memory
#define ROM_BASEADDR			0x1FF00000U			//Base address of ROM (SYSTEM MEMORY) Memory
#define SRAM 					SRAM_BASE			//Only 1 type of SRAM

#define PERIPH_BASEADDR				0x40000000U 	//Base address of the peripherals base
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR //Base address of the APB1 peripheral base, same as whole peripherals section
#define APB2PERIPH_BASEADDR			0x40010000U 	//Base address of the APB2 peripheral base
#define AHBPERIPH_BASEADDR			0x40020000U 	//Base address of the AHB peripheral base
#define IOPORTPH_BASEADDR			0x50000000U 	//Base address of the IOPORT peripheral base


/*
 * IRQ(Interrupt request) For STM32L073RZ
 */


#define IRQ_NO_EXTI0_1			5		//GPIO 0 and 1 pin IRQ definitions
#define IRQ_NO_EXTI2_3			6		//GPIO 2 and 3 pin IRQ definitions
#define IRQ_NO_EXTI4_15			7		//GPIO 4 to 15 pin IRQ definitions

#define IRQ_NO_SPI1				25		//GPIO 0 and 1 pin IRQ definitions
#define IRQ_NO_SPI2				26		//GPIO 0 and 1 pin IRQ definitions

#define IRQ_NO_EXTI17_19_20		2
#define IRQ_NO_EXTI21_22		12
#define IRQ_NO_EXTI29			13
#define IRQ_NO_EXTI23			23
#define IRQ_NO_EXTI25			27
#define IRQ_NO_EXTI26			28
#define IRQ_NO_EXTI28			29
#define IRQ_NO_EXTI18			31

/*
 * IRQ(Priority numbers) For STM32L073RZ
 */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3


/*
 * Base addresses of peripherals which are hanging on IOPORT bus
 */

#define GPIOA_BASE				(IOPORTPH_BASEADDR + 0x0000) //GPIOA to H excluding F&H
#define GPIOB_BASE				(IOPORTPH_BASEADDR + 0x0400)
#define GPIOC_BASE				(IOPORTPH_BASEADDR + 0x0800)
#define GPIOD_BASE				(IOPORTPH_BASEADDR + 0x0C00)
#define GPIOE_BASE				(IOPORTPH_BASEADDR + 0x1000)
#define GPIOH_BASE				(IOPORTPH_BASEADDR + 0x1C00)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define LCD_BASE				(APB1PERIPH_BASEADDR + 0x2400) //Peripherals in APB1
#define LPUSART1_BASE			(APB1PERIPH_BASEADDR + 0x4800)
#define RTC_BASE				(APB1PERIPH_BASEADDR + 0x2800)
#define USBFS_BASE				(APB1PERIPH_BASEADDR + 0x5C00)

#define I2C1_BASE				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASE				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASE				(APB1PERIPH_BASEADDR + 0x7800)

#define SPI2_BASE				(APB1PERIPH_BASEADDR + 0x3800)

#define USART2_BASE				(APB1PERIPH_BASEADDR + 0x4400)
#define USART4_BASE				(APB1PERIPH_BASEADDR + 0x4C00)
#define USART5_BASE				(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASE				(APB2PERIPH_BASEADDR + 0x0400) //Peripherals in APB2
#define SPI1_BASE				(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASE				(APB2PERIPH_BASEADDR + 0x0000)
#define USART1_BASE				(APB2PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals which are hanging on AHB bus
 */

#define RCC_BASE				(AHBPERIPH_BASEADDR + 0x1000)

/******************peripheral register definition structures****************************/

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;			//GPIO port mode register. Address offset 0x00
	__vo uint32_t OTYPER;			//GPIO port output type register. Address offset 0x04, etc..
	__vo uint32_t OSPEEDR;			//GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;				//GPIO port input data register
	__vo uint32_t ODR;				//GPIO port output data register
	__vo uint32_t BSRR;				//GPIO port bit set/reset register
	__vo uint32_t LCKR;				//GPIO port configuration lock register
	__vo uint32_t AFR[2];			//GPIO alternate function low/high register.Address offset 0x20 for AFRL and 0x24 for AFRH
	__vo uint32_t BRR;				//GPIO port bit reset register
 //__vo means volatile as GPIO registers can change often
} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;				//Clock control register. Address offset 0x00
	__vo uint32_t ICSCR;			//Internal clock sources calibration register. Address offset 0x04, etc..
	__vo uint32_t CRRCR;			//Clock recovery RC register
	__vo uint32_t CFGR;				//Clock configuration register
	__vo uint32_t CIER;				//Clock interrupt enable register
	__vo uint32_t CIFR;				//Clock interrupt flag register
	__vo uint32_t CICR;				//Clock interrupt clear register
	__vo uint32_t IOPRSTR;			//GPIO reset register
	__vo uint32_t AHBRSTR;			//AHB peripheral reset register
	__vo uint32_t APB2RSTR;			//APB2 peripheral reset register
	__vo uint32_t APB1RSTR;			//APB1 peripheral reset register
	__vo uint32_t IOPENR;			//GPIO clock enable register
	__vo uint32_t AHBENR;			//AHB peripheral clock enable register
	__vo uint32_t APB2ENR;			//APB2 peripheral clock enable register
	__vo uint32_t APB1ENR;			//APB1 peripheral clock enable register
	__vo uint32_t IOPSMEN;			//GPIO clock enable in Sleep mode register
	__vo uint32_t AHBSMENR;			//AHB peripheral clock enable in Sleep mode register
	__vo uint32_t APB2SMENR;		//APB2 peripheral clock enable in Sleep mode register
	__vo uint32_t APB1SMENR;		//APB1 peripheral clock enable in Sleep mode register
	__vo uint32_t CCIPR;			//Clock configuration register
	__vo uint32_t CSR;				//Control/status register
 //__vo means volatile as GPIO registers can change often
} RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;				//Interrupt mask register. Address offset 0x00
	__vo uint32_t EMR;				//Event mask register. Address offset 0x04, etc..
	__vo uint32_t RTSR;				//Rising trigger selection register
	__vo uint32_t FTSR;				//Falling trigger selection register
	__vo uint32_t SWIER;			//Software interrupt event register
	__vo uint32_t PR;				//Pending register
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t CFGR1;			//Memory remap register.
	__vo uint32_t CFGR2;			//Event mask register.
	__vo uint32_t EXTICR[4];		//External interrupt configuration registers 1 to 4
	__vo uint32_t COMP1_CTRL;		//Comparator 1 control
	__vo uint32_t COMP2_CTRL;		//Comparator 2 control
	__vo uint32_t CFGR3;		 		//Reference control and status register

} SYSCFG_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */


#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE)

#define RCC						((RCC_RegDef_t*)RCC_BASE)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASE)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASE)

#define SPI1					((SPI_RegDef_t*)SPI1_BASE)
#define SPI2					((SPI_RegDef_t*)SPI2_BASE)
/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			(RCC-> IOPENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC-> IOPENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC-> IOPENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC-> IOPENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC-> IOPENR |= (1 << 4))
#define GPIOH_PCLK_EN()			(RCC-> IOPENR |= (1 << 7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			(RCC-> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC-> APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC-> APB1ENR |= (1 << 30))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC-> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC-> APB1ENR |= (1 << 14))

/*
 * Clock Enable Macros for U(S)ART peripherals
 */
#define USART1_PCLK_EN()			(RCC-> APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()			(RCC-> APB1ENR |= (1 << 17))
#define USART4_PCLK_EN()			(RCC-> APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()			(RCC-> APB1ENR |= (1 << 20))
#define LPUART1EN_PCLK_EN()			(RCC-> APB1ENR |= (1 << 18))

/*
 * Clock Enable Macros for SYSCF peripherals
 */
#define SYSCFG_PCLK_EN()			(RCC-> APB2ENR |= (1 << 0))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			(RCC-> IOPENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC-> IOPENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC-> IOPENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC-> IOPENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC-> IOPENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()			(RCC-> IOPENR &= ~(1 << 7))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 30))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC-> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 14))

/*
 * Clock Disable Macros for U(S)ART peripherals
 */
#define USART1_PCLK_DI()			(RCC-> APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 17))
#define USART4_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 20))
#define LPUART1EN_PCLK_DI()			(RCC-> APB1ENR &= ~(1 << 18))

/*
 * Clock Disable Macros for SYSCF peripherals
 */
#define SYSCFG_PCLK_DI()			(RCC-> APB2ENR &= (1 << 0))


/*
 * Macros to reset GPIOx peripherals
 */
//first set then reset the port
//to perform both statements, using a technique do...while...condition zero loop to execute 2 statements with a single macro
//while(0) allows the execution of the loop only once
#define GPIOA_REG_RST()				do{(RCC->IOPRSTR |= (1 << 0));	(RCC->IOPRSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RST()				do{(RCC->IOPRSTR |= (1 << 1));	(RCC->IOPRSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RST()				do{(RCC->IOPRSTR |= (1 << 2));	(RCC->IOPRSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RST()				do{(RCC->IOPRSTR |= (1 << 3));	(RCC->IOPRSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RST()				do{(RCC->IOPRSTR |= (1 << 4));	(RCC->IOPRSTR &= ~(1 << 4));}while(0)
#define GPIOH_REG_RST()				do{(RCC->IOPRSTR |= (1 << 7));	(RCC->IOPRSTR &= ~(1 << 7));}while(0)

/*
 * Returns port code for a given GPIOx base address
 */


//If x is equal to GPIOA then return 0, else other lines are being processed
// C conditional operators - google it
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOH)?5:0)

/*
 * Generic macros
 */

#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/*
 * ********************Bit position definitions of SPI peripheral registers****************
 */

#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSB_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RX_ONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_OE			14
#define SPI_CR1_BIDI_MODE		15

#define SPI_CR2_RXMAEN			0
#define SPI_CR2_TXMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRC_ERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


//GPIO_RegDef_t *pGPIOA = GPIOA;
/*
* The document called Cortex™-M0 Devices Generic User Guide
* describe the interrupts of M0 processor.
* IRQ0 to IRQ31
* Interrupt Set-Enable Register (ISER) enables interrupt starting at the address of 0xE000E100
* Interrupt Clear-enable Register (ICER) clears interrupt starting at the address of 0xE000E180
* Does the difference of 0x0080 - or 128 in DEC, means that
* there should be 4 ISER registers with 32 bits each?
*/

/********************************************SPI Configuration****************
 *
 * Configuration structure for SPIx peripheral
 */

typedef struct
{
	__vo uint32_t CR1; 		//SPI control register 1
	__vo uint32_t CR2;		//SPI control register 2
	__vo uint32_t SR;		//SPI status register
	__vo uint32_t DR;		//SPI data register
	__vo uint32_t CRCPR;	//SPI CRC polynomial register
	__vo uint32_t RXCRCR;	//SPI RX CRC register
	__vo uint32_t TXCRCR;	//SPI TX CRC register
	__vo uint32_t I2SCFGR;	//SPI_I2S configuration register
	__vo uint32_t I2SSPR;	//SPI_I2S pre-scaler register
}SPI_RegDef_t;

/*
 * Macros to reset SPIx peripherals
 */
//first set then reset the port
//to perform both statements, using a technique do...while...condition zero loop to execute 2 statements with a single macro
//while(0) allows the execution of the loop only once
#define SPI1_REG_RST()				do{(RCC->APB2RSTR |= (1 << 12));	(RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RST()				do{(RCC->APB1RSTR |= (1 << 14));	(RCC->APB1RSTR &= ~(1 << 14));}while(0)

#include "STM32L0732RZ_gpio_driver.h"
#include "STM32L0732RZ_SPI_driver.h"


#endif /* INC_STM32L0732RZ_H_ */

