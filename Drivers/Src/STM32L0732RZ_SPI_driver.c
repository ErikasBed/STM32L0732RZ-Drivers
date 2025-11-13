/*
 * STM32L0732RZ_SPI_driver.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Test
 */

#include "STM32L0732RZ_SPI_driver.h"
#include "STM32L0732RZ.h"

//using word static, to indicate that these functions are private to this .c file, and should not be used elsewhere
static void spi_txe_handle_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_rxne_handle_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_ovr_err_handle_interrupt_handle(SPI_Handle_t *pSPI_Handle);

/*
 * Peripheral Clock setup
 */
/**********************************************************************************************
 *@fn			- SPI_PeriClockControl
 *
 *@brief		- This function enables or disables peripheral clock for given SPI port
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableOrDisable)
{
	if(EnableOrDisable == ENABLE)
		{
			if(pSPIx == SPI1)
				{
					SPI1_PCLK_EN();
				}

			else if(pSPIx == SPI2)
				{
					SPI2_PCLK_EN();
				}

	else
		{
			if(pSPIx == SPI1)
				{
				SPI1_PCLK_DI();
				}

			else if(pSPIx == SPI2)
				{
				SPI2_PCLK_DI();
				}
		}
		}
}
/*
 * Init and de-init
 */
/**********************************************************************************************
 *@fn			- SPI_Init
 *
 *@brief		- This function initialized the given SPI port
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	uint32_t tempreg = 0;

	//Peripheral clock enable
	SPI_PeriClockControl(pSPI_Handle->pSPIx,ENABLE);

	//1. Configure device mode
	tempreg |= pSPI_Handle->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR;		// if master, then the second bit will get shifted to the value of 1
	//2. Configure the bus config
	if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDI_MODE);									// 15th bit is turned off

	}else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDI_MODE);									// 15th bit is on

	}else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be clear
		tempreg &= ~( 1 << SPI_CR1_BIDI_MODE);
		//RXONLY bit should be set
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}
	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPI_Handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPI_Handle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPHA;

	tempreg |= pSPI_Handle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPI_Handle->pSPIx->CR1 = tempreg; //setting the value stored in tempreg to the CR1 register of SPIx
}

/**********************************************************************************************
 *@fn			- SPI_DeInit
 *
 *@brief		- This function initialized the given SPI port
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RST();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RST();
	}
}
/**********************************************************************************************
 *@fn			- SPI_GetFlagStatus
 *
 *@brief		- This function returns the requested flag status
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Flag name
 *
 *return		- True or false
 *
 *@note			-
 *********************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)	//we are testing a bit position of the flag. If its matching
	{
		return FLAG_SET;
	}
	return FLAG_RESET;			//if condition does not match, it returns flag reset (0)
}

/*
 * Data Send Data
 */
/**********************************************************************************************
 *@fn			- SPI_SendData
 *
 *@brief		- This function sends data based on the SPI settings
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Tx Buffer pointer
 *@param[in]	- Data length
 *
 *return		- none
 *
 *@note			- This is a blocking call (it wont return untill all of the data been sent
 *@note
 *********************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	/*
	 * 1. First checking If len == 0 is true, all bites were transfered so Exit the function.
	 * Ff the len == 0, if False - there are more bits to send, so wait till Tx buffer is empty
	 * If we dont wait until TX buffer is empty and keep on passing on bits, data will be lost/corrupted
	 *
	 * 2. Buffer status can be verified via SPI_SR status register bit 1 TXE (Transmit buffer empty)
	 * if its 1 buffer is empty, 0 is full
	 *
	 * 3.1 Check DFF parameter. If it is 0, it is a 8 bit data transmission, then load 1 byte
	 * of data into DR (Data register) and increment the buffer address in order to point to the next data byte
	 *
	 * 3.2 Decrement the Len variable by 1
	 *
	 * 4.1 If DFF == 1, it is a 16 bit data transmission, then load 2 bytes to DR (Data register),
	 * and increment the buffer address twice in order to point to the next data byte
	 *
	 * 4.2 Decrement the Len variable by 2
	 *
	 * TX and RX buffers can be accessed through data register SPI_DR
	 * A write to the data register will write into the Tx buffer and a read
	 * from the data register will return the value held in the Rx buffer
	 *
	 * To read RX buffer, we have to first read SPI_SR_RXNE flag, which says if it is empty (0) or not empty (1)
	 * Same with TXE buffer, before sending
	 */

	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET ); // If the 1st bit of SR register is not 0, we have to wait

		//2. Check the DFF
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer); //Referencing buffer as a 16 bit data
			Len--; //Len is decreased by 2 bc two bytes (16bit) is being sent over the line
			Len--;
			(uint16_t*)pTxBuffer++; // Incrementing the pointed by two to shift the next data to be written
		}
		else
		{
			//8 bit DFF
			//1. Load the data into the DR
			pSPIx->DR = *pTxBuffer; //Referencing buffer as a 8 bit data
			Len--;
			pTxBuffer++;
		}
	}
}
/**********************************************************************************************
 *@fn			- SPI_ReceiveData
 *
 *@brief		- This function receives data based on the SPI settings
 *
 *@param[in]	- Rx Buffer pointer
 *@param[in]	- Data length
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	/*
	 * 1. First checking If Len == 0 is true, all bites were transfered so Exit the function.
	 * If the Len == 0, is False - there are more bits to receive, so wait till Rx buffer is empty
	 * If we dont wait until RX buffer is empty and keep on passing on bits, data will be lost/corrupted
	 *
	 * 2. Buffer status can be verified via SPI_SR status register bit 1 RXNE (Receive buffer empty)
	 * if its 1 buffer is empty, 0 is full
	 *
	 * 3.1 Check DFF parameter. If it is 0, it is a 8 bit data reception, then load 1 byte
	 * of data into DR (Data register) and increment the buffer address in order to point to the next data byte
	 *
	 * 3.2 Decrement the Len variable by 1
	 *
	 * 4.1 If DFF == 1, it is a 16 bit data reception, then load 2 bytes to DR (Data register),
	 * and increment the buffer address twice in order to point to the next data byte
	 *
	 * 4.2 Decrement the Len variable by 2
	 *
	 * TX and RX buffers can be accessed through data register SPI_DR
	 * A write to the data register will write into the Tx buffer and a read
	 * from the data register will return the value held in the Rx buffer
	 *
	 * To read RX buffer, we have to first read SPI_SR_RXNE flag, which says if it is empty (0) or not empty (1)
	 * Same with TXE buffer, before sending
	 */
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET ); // If the 1st bit of SR register is not 0, we have to wait

		//2. Check the DFF
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. Load the data into the DR
			*((uint16_t*)pRxBuffer) = pSPIx->DR; //Referencing buffer as a 16 bit data
			Len--; //Len is decreased by 2 bc two bytes (16bit) is being sent over the line
			Len--;
			(uint16_t*)pRxBuffer++; // Incrementing the pointed by two to shift the next data to be written
		}
		else
		{
			//8 bit DFF
			//1. Load the data into the DR
			*pRxBuffer = pSPIx->DR  ; //Referencing buffer as a 8 bit data
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPI_Handle->TxState;

	if(state != SPI_BUSY_IN_TX) //first verify if TX status is yet not too busy
	{
		/*
		 * Structure of sending data using IT mode
		 * 1. Save the Tx buffer address and Len information in some global variables
		 * 2. Mark the SPI state as busy in transmission so that no other code can
		 * take over same SPI peripheral until transmission is over
		 * 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		 * 4. Data transmission will be handled by the ISR code (will implement later)
		 */

		//1. Save the Tx buffer address and Len information in some global variables
		pSPI_Handle->pTxBuffer = pTxBuffer;
		pSPI_Handle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can
		//take over same SPI peripheral until transmission is over
		pSPI_Handle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
// NOT A BLOCKING CALL, aka NO WHILE LOOPS THAT MIGHT GET STUCC
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPI_Handle->RxState;

	if(state != SPI_BUSY_IN_RX) //first verify if RX status is yet not too busy
	{
		//1. Save the Rx buffer address and Len information in some global variables
		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can
		//take over same SPI peripheral until transmission is over
		pSPI_Handle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;

}

/*
 * Enabling or disabling SPI comms
 */
/**********************************************************************************************
 *@fn			- SPI_PeripheralControl
 *
 *@brief		- This function enables or disables SPI communication
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx-> CR1 |= (1 << SPI_CR1_SPE); //Enabling the SPI communication
	}
	else
	{
		pSPIx-> CR1 &= ~(1 << SPI_CR1_SPE); //Disabling the SPI communication
	}
}

/*
 * SSI bit control
 */
/**********************************************************************************************
 *@fn			- SPI_SSIConfig
 *
 *@brief		- This function enables or disables Software controlled slave (SSI)
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx-> CR1 |= (1 << SPI_CR1_SSI); //Enabling the SPI communication
		}
		else
		{
			pSPIx-> CR1 &= ~(1 << SPI_CR1_SSI); //Disabling the SPI communication
		}
}
/*
 * SSOE bit control
 */
/**********************************************************************************************
 *@fn			- SPI_SSOEConfig
 *
 *@brief		- This function enables or disables SS control as a master
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx-> CR2 |= (1 << SPI_CR2_SSOE); //Enabling the SPI communication
		}
		else
		{
			pSPIx-> CR2 &= ~(1 << SPI_CR2_SSOE); //Disabling the SPI communication
		}
}
/*
 * IRQ Configuration and ISR Handling
 */
/**********************************************************************************************
 *@fn			- SPI_IRQITConfig
 *
 *@brief		- This function enables or disables peripheral clock for given SPI port
 *
 *@param[in]	- base address of the SPI peripheral
 *@param[in]	- Enable or disable Macros defined in the STM32L0732RZ_SPI_driver.h file
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 *@fn			- SPI_IRQHandling
 *
 *@brief		- This function allows to Configure the interrupt on the SPI pins
 *
 *@param[in]	- Interrupt request number
 *@param[in]	- Enable or disable
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle)
{
	/*
	 * Whenever the control comes inside of ISR, it means that SPI peripheral triggered the interrupt
	 * It could have happened due to 6 possible reasons (TXE, RXNE, MODF, OVR, CRCERR, FRE)
	 * First we have to decode why interrupt happened and which event triggered it
	 * It is done by checking the SR register
	 * If interrupt caused by RXNE - handle an RXNE event
	 * If interrupt caused by TXE - handle an TXE event
	 * If interrupt caused by Error flag - handle error event
	 *
	 * Handle TX interrupt routine:
	 * 1. First check if DFF register if it is 8 or 16 bit mode
	 * 2.1 If it is 8 bits - Write one byte to SPI data register
	 * 2.2 Decrease the LEN by 1
	 * 3.1 If it is 16 bits - Write two byte to SPI data register
	 * 3.2 Decrease the LEN by 2
	 * 4. Check if Len is equal to 0
	 * 5. If it is - transmission is over and close the SPI TX
	 * 6. If no - transmission is not over, so wait till another TXE interrupt and then send next bytes till Len = 0
	 *
	 * PS: CRC is not being implemented in this course :/
	 * PSS: No need to worry abt MODF fault as well bc we are not using the device in multi master implication
	 * PSSS: Same with TI Frame format FRE, so only Overrun error will be handled besides TXE RXNE
	 *
	 * Overrun error is when RX buffer is full, and new data comes in but it cannot be saved as buffer is full, thus it is lost
	 */


	uint8_t temp1, temp2;
	//1. First check for TXE
	temp1 = pSPI_Handle->pSPIx->SR &( 1 << SPI_SR_TXE); // if TXE flag is set, then temp1 = 1, if reset temp1 = 0.
	temp2 = pSPI_Handle->pSPIx->CR2 &( 1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) //if both temp1 and temp2 are true, we handle TXE
	{
		//handle TXE
		spi_txe_handle_interrupt_handle(pSPI_Handle);
	}
	//2. Check for RXNE
		temp1 = pSPI_Handle->pSPIx->SR &( 1 << SPI_SR_RXNE);
		temp2 = pSPI_Handle->pSPIx->CR2 &( 1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_handle_interrupt_handle(pSPI_Handle);
	}

	//3. Check for OVR flag
	temp1 = pSPI_Handle->pSPIx->SR &( 1 << SPI_SR_OVR); //
	temp2 = pSPI_Handle->pSPIx->CR2 &( 1 << SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{
		//handle OVR error
		spi_ovr_err_handle_interrupt_handle(pSPI_Handle);
	}

}
/**********************************************************************************************
 *@fn			- SPI_IRQPriorityConfig
 *
 *@brief		- This function allows to Configure the interrupt on the GPIO pins
 *
 *@param[in]	- Interrupt request number
 *@param[in]	- Interrupt request Priority level
 *
 *return		- none
 *
 *@note			-
 *********************************************************************************************/

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
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

/*
 * During SPI communication, Interrupts can be generated by the following events:
 * 1. Transmit TX buffer ready to be loaded
 * 2. Data received in Rx buffer
 * 3. Master mode fault (in single master case, must avoid this error happening) (Prevented by taking care of NSS pin)
 * 4. Overrun error ()
 */
/*
 * Some helper function implementations
 */

static void spi_txe_handle_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{ //Similar code as in SPI_SendData


	//1. Check the DFF
	if( (pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//1. Load the data into the DR
		pSPI_Handle->pSPIx->DR = *((uint16_t*)pSPI_Handle->pTxBuffer); //Referencing buffer as a 16 bit data
		pSPI_Handle->TxLen--; //Len is decreased by 2 bc two bytes (16bit) is being sent over the line
		pSPI_Handle->TxLen--;
		(uint16_t*)pSPI_Handle->pTxBuffer++; // Incrementing the pointed by two to shift the next data to be written
	}
	else
	{
		//8 bit DFF
		//1. Load the data into the DR
		pSPI_Handle->pSPIx->DR = *pSPI_Handle->pTxBuffer; //Referencing buffer as a 8 bit data
		pSPI_Handle->TxLen--;
		pSPI_Handle->pTxBuffer++;
	}
	if(! pSPI_Handle->TxLen) //if TxLen is 0
	{
		// TxLen is 0, close the SPI comms
		//Inform the application that TX is over

		//Deactivate TXE bit. This prevents interrupts from setting up TXE flag
		SPI_CloseTransmission(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_TX_CMPLT); //Informing the application that its all good now
	}
}


static void spi_rxne_handle_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	//1. Check the DFF
	if( (pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//1. Load the data into the DR
		*((uint16_t*)pSPI_Handle->pRxBuffer) = pSPI_Handle->pSPIx->DR ; //Referencing buffer as a 16 bit data
		pSPI_Handle->RxLen--; //Len is decreased by 2 bc two bytes (16bit) is being sent over the line
		pSPI_Handle->RxLen--;
		pSPI_Handle->pRxBuffer--; // Decrementing the pointed by two to shift the next data to be written
		pSPI_Handle->pRxBuffer--;
	}
	else
	{
		//8 bit DFF
		//1. Load the data into the DR
		*(pSPI_Handle->pRxBuffer) = (uint8_t) pSPI_Handle->pSPIx->DR ; //Referencing buffer as a 8 bit data
		pSPI_Handle->RxLen--;
		pSPI_Handle->pRxBuffer--;
	}
	if(! pSPI_Handle->TxLen) //if TxLen is 0
	{
		// TxLen is 0, close the SPI comms
		//Inform the application that TX is over

		//Deactivate TXE bit. This prevents interrupts from setting up TXE flag
		SPI_CloseReception(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_RX_CMPLT); //Informing the application that its all good now
	}
}

static void spi_ovr_err_handle_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	//If the SPI peripheral is busy in transmission, and at that time if OVR error happens, the if statement below wont get executed
	//but application will get informed of OVR error event
	//Then application will call a SPI_ClearOVRFlag function
	uint8_t temp;
	// 1. Clear the OVR flag
	if(pSPI_Handle->TxState != SPI_BUSY_IN_TX) //if its not a busy flag, we can delete whatever is inside the register
	{
		temp = pSPI_Handle->pSPIx->DR;
		temp = pSPI_Handle->pSPIx->SR;
	}
	(void)temp;
	// 2. Inform the application
	//If the Busy flag is set, then data wont be cleared, thus Application even call back has to process it in order not to loose it
	SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) //application may call this function to clear OVR flag
{
	uint8_t temp;
	temp =pSPIx->DR;
	temp =pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPI_Handle->pTxBuffer = NULL;
	pSPI_Handle->TxLen = 0;
	pSPI_Handle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPI_Handle->pRxBuffer = NULL;
	pSPI_Handle->RxLen = 0;
	pSPI_Handle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv)
{
	//This is a weak implementation. The application may override this function
}
