/*
 * STM32L0732RZ_gpio_driver.c
 *
 *  Created on: Sep 1, 2023
 *      Author: Test
 */

#include "STM32L0732RZ_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/**********************************************************************************************
 *@fn			- GPIO_PeriClockControl
 *
 *@brief		- This function enables or disables peripheral clock for given GPIO port
 *
 *@param[in]	- base address of the GPIO peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_gpio_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable)
{
	if(EnableOrDisable == ENABLE)
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}

		else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}

		else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}

		else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}

		else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}
		else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}
	}
	else
	{
		if(pGPIOx == GPIOA)
					{
						GPIOA_PCLK_DI();
					}

				else if(pGPIOx == GPIOB)
					{
						GPIOB_PCLK_DI();
					}

				else if(pGPIOx == GPIOC)
					{
						GPIOC_PCLK_DI();
					}

				else if(pGPIOx == GPIOD)
					{
						GPIOD_PCLK_DI();
					}

				else if(pGPIOx == GPIOE)
					{
						GPIOE_PCLK_DI();
					}
				else if(pGPIOx == GPIOH)
					{
						GPIOH_PCLK_DI();
					}
	}
}
/*
 * Init and de-init
 */
/**********************************************************************************************
 *@fn			- GPIO_Init
 *
 *@brief		- This function initialized the given GPIO port
 *
 *@param[in]	- base address of the GPIO peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_gpio_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t temp; //temp register

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIO_Handle->pGPIOx,ENABLE);

	//1. Configure the mode of the GPIO pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) //if the GPIO_PinMode is less or equal to GPIO_MODE_ANALOG (3), it is not an interrupt
	{
		//each pin takes 2 bit fields, that's why the shifted bit is multiplied by two. Pin number multiplied by two to get access to pins first bit
		temp =(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));

		pGPIO_Handle->pGPIOx->MODER &= ~(0x3 <<(2 * pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber)); //reseting the value before writing into register

		//writing into the register. By using |= (AND) allows us to change the register value without changing the rest of the bits
		pGPIO_Handle->pGPIOx->MODER |= temp; //using handle variable to access base address of the register and then storing a value into it store temp value to the MODER register at the base address of the peripheral

	}else //interrupt
	{
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) //Falling edge interrupt
		{
			//1. Configure the Falling Trigger Selection register (FTSR)
			EXTI->FTSR |= (1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) //Rising edge interrupt
		{
			//1. Configure the Rising Trigger Selection register (RTSR)
			EXTI->RTSR |= (1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) //Rising-Falling edge interrupt
		{
			//1. Configure the both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR (external interrupt config)

		//a. Dividing the required pin number by 4 and store the value to temp1
		//b. The remainder would tell us which of the EXTICR[1-4] registers to configure
		//c. Reffering to the required pin, apply more function (%) by the factor of 4
		//c. More gives the residual of 2 numbers
		//d. Residual would vary from 0 to 3, so it would allow us
		//d. to choose the correct configuration in the register bits by multiply residual by 4
		//e. Apply settings in respect to the required port[A-E,H]

		uint8_t temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4; //variable to define in which register out of 4 configuration should take place
		uint8_t temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4; //variable to define in which sets of 4 bits within 16 bits should be modified
		uint8_t	portcode = GPIO_BASEADDR_TO_CODE(pGPIO_Handle->pGPIOx);  //Base address of the peripheral returns the portcode macro
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2*4);				 //Each port has a set of codes that has to be written into a register

		//3. Enable the Exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0; //reseting temp value

	//2. Configure the speed
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed <<(2 *pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //2 bits per each of the pin
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber); //reseting the value before writing into register
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp; //Accessing base address of the Output SPEED register and storing a temp value in it
	temp = 0;

	//3. Configure the Pull-up/pull-down (PUPD) settings

	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 *pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //2 bits per each of the pin
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber); //reseting the value before writing into register
	pGPIO_Handle->pGPIOx->PUPDR |= temp; //Accessing base address of the Pull-up pull-down register and storing a temp value in it
	temp = 0;

	//4. Configure the Output type

	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //1 bit per each of the pin so multiplication by 2 is removed
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber); //reseting the value before writing into register
	pGPIO_Handle->pGPIOx->OTYPER |= temp; //Accessing base address of the output register and storing a temp value in it
	temp = 0;


	//5. Configure the alternative functionality

	//There are two registers: Alternate function high and low registers.
	//First the written value we have to divide by 8. If it is more than 1, then we will use an Alternate function High register, and low if it's less than 1
	//Then we have to shift registers by multiplying it by 4, because each pin has 4 bit sections
	if(pGPIO_Handle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFNCT) //if pin mode is selected as Alternative function mode
	{
		uint8_t temp1;
		uint8_t temp2;

		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8; //ALTFNCT High or Low
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8; //How many shifts to be done
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //reseting the value before writing into register
		pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}


}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
				{
					GPIOA_REG_RST();
				}

			else if(pGPIOx == GPIOB)
				{
					GPIOB_REG_RST();
				}

			else if(pGPIOx == GPIOC)
				{
					GPIOC_REG_RST();
				}

			else if(pGPIOx == GPIOD)
				{
					GPIOD_REG_RST();
				}

			else if(pGPIOx == GPIOE)
				{
					GPIOE_REG_RST();
				}
			else if(pGPIOx == GPIOH)
				{
					GPIOH_REG_RST();
				}
}
/*
 * Data read and write
 */
/**********************************************************************************************
 *@fn			- GPIO_ReadFromInputPin
 *
 *@brief		- This function returns value of an input pin
 *
 *@param[in]	- base address of the GPIO peripheral
 *@param[in]	- Pin number
 *
 *return		- value of the pin either 0 or 1
 *
 *@note			- To pull an information of the specific pin, we would shift the selected pin
 *@note			to the very first bit position, mask all of the values behind him
 *@note			and return only the very first bit
 *@note
 *********************************************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
	{
	uint8_t value;
	//the IDR value is being shifted by the amount mentioned in PinNumber input
	//to get the value in the least significant bit
	//and rest of the bits are masked by setting them to 0
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
	}


/**********************************************************************************************
 *@fn			- GPIO_ReadFromInputPort
 *
 *@brief		- This function returns value of an input port
 *
 *@param[in]	- base address of the GPIO peripheral
 *
 *return		- value of the port in 16bit
 *
 *@note
 *********************************************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
	{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
	}

/**********************************************************************************************
 *@fn			- GPIO_WriteToOuputtPin
 *
 *@brief		- This function allows to output a value in a pin
 *
 *@param[in]	- base address of the GPIO peripheral
 *@param[in]	- Pin number
 *@param[in]	- Value either 1 or 0
 *
 *return		-
 *
 *@note
 *********************************************************************************************/

void GPIO_WriteToOuputtPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);//clearing bit position corresponding to the pin number
	}
}
/**********************************************************************************************
 *@fn			- GPIO_WriteToOutputPort
 *
 *@brief		- This function allows to output a value in a port
 *
 *@param[in]	- base address of the GPIO peripheral
 *@param[in]	- Value of a 16 bit
 *
 *return		-
 *
 *@note
 *********************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
		//Write a value to the output data register at the bit field corresponding to the port
		pGPIOx->ODR = Value;
}

/**********************************************************************************************
 *@fn			- GPIO_ToggleOutputPin
 *
 *@brief		- This function allows to change the value of a pin
 *
 *@param[in]	- base address of the GPIO peripheral
 *@param[in]	- Pin number
 *
 *return		-
 *
 *@note
**********************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber); // ^= is an XOR operator
}

/*
 * IRQ Configuration and ISR Handling
 */
/**********************************************************************************************
 *@fn			- GPIO_IRQIInterruptConfig
 *
 *@brief		- This function allows to Configure the interrupt on the GPIO pins
 *
 *@param[in]	- Interrupt request number
 *@param[in]	- Enable or disable
 *
 *return		-
 *
 *@note
**********************************************************************************************/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		//Cortex-M0+ Generic User Guide says that there are only 0-31 ISER bits
		*NVIC_ISER |= ( 1 << IRQNumber);
	}
	else
	{
		//Cortex-M0+ Generic User Guide says that there are only 0-31 ICER bits
		*NVIC_ICER |= ( 1 << IRQNumber);
	}
}


/**********************************************************************************************
 *@fn			- GPIO_IRQPriorityConfig
 *
 *@brief		- This function allows to Configure the interrupt on the GPIO pins
 *
 *@param[in]	- Interrupt request number
 *@param[in]	- Interrupt request Priority level
 *
 *return		-
 *
 *@note
**********************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. Lets find out the IPR register
	uint8_t iprx = IRQNumber /4;
	uint8_t iprx_section = IRQNumber %4;

	//Writing into a required register by
	//Shifting the base address by the IRQ number, and assigning the
	// Priority value to the iprx_sextion bit multiplied by 8
	//SO first we find out in which one out of 8 IPRx registers has to be modified
	//Then we find out in which one out of 4 PRI_x sections of the IPRO we have to write
	// Multiplication by 8 stands because each of the PRI has 8 bits, so we shift it by 8 every time

	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );

	//Out of 8 bits in each PRIx, only 6&7 bits are usable, rest are reserved
	//So when we write priority levels we have to shift it by 6
	// NO_PR_BITS_IMPLEMENTED is set to 2 as only 2 bits are available for priority settings
	// 0 Priority (Highest), 1 , 2 and 3 (Lowest)
}


/**********************************************************************************************
*@fn			- GPIO_IRQHandling
*
*@brief		- This function define the Pin for an interrupt
*
*@param[in]	- Pin number
*
*return		-
*
*@noteGPIO and SPI interrupt handling is different
	* When interrupt happens in the GPIO, EXTI block detects
	* if its a Falling (FT) or Rising edge (RT)
	* Then Interrupt would be send over EXTI line to the IRQ number (NVIC)
	* When NVIC receives the interrupt Pending Register (PR) then if
	* Processor is not executing any Interrupt service routine (ISR)
	* Then the fixed Vector address related to the IRQ number will be accessed
	* Then the processor reads the value which is stored in that address (ISR)
	* My responsibility for keeping the address of ISR function at specified location
	* (Fixed vector address)
	* Finally, the processor clears the Pending Register bit and it will execute ISR
	*
	* If there is an outgoing ISR which is being processed, and another one triggers,
	* The priority levels will be compared and the one with higher priority will be executed first
	*
	* Even if the interrupt reception is disabled at certain IRQ number, the
	* interrupt can still get registered, just ISR wont get executed
	*
	* Action step in handling interrupts:
	* 1. Implement ISR function
	* 2. Store the address of your iSR at the vector address location corresponding to the IRQ
	* number for which we have written the ISR
	*
	* ISRs for various IRQs are being implemented in the startup routine.
	* ISRs are attributed as as a .weak ISRs.
	* My responsibility to overwrite the ISRs in the application
**********************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR register corresponding to the PIN number
	if(EXTI->PR &= (1 << PinNumber)) // If the PR bit position corresponding to the bit on the right side is set - then the interrupt is pended
	{
		//Clear that pending register bit
		EXTI->PR |= (1 << PinNumber);
	}
}

