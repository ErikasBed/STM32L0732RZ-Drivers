/*
 * STM32L0732RZ_SPI_driver.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Test
 */

#ifndef INC_STM32L0732RZ_SPI_DRIVER_H_
#define INC_STM32L0732RZ_SPI_DRIVER_H_
#include "STM32L0732RZ.h"
/*
 * Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
* Handle structure for SPIx peripheral
*/

typedef struct
{
	SPI_RegDef_t	*pSPIx; 	//Basic address of SPIx peripheral
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer; //To store the applications Tx buffer address
	uint8_t			*pRxBuffer;	//To store the applications Rx buffer address
	uint32_t		TxLen; 		//To store Tx len
	uint32_t		RxLen;		//To store Rx len
	uint8_t			TxState; 	//To store Tx state
	uint8_t			RxState;	//To store Rx state
}SPI_Handle_t;

/*
 * @SPI_DeviceMode			//bit 2 [MSTR] of the SPI_CR1 register
 */
#define SPI_DEVICE_MODE_SLAVE	0
#define SPI_DEVICE_MODE_MASTER	1

/*
 * SPI_BusConfig			//bits 14 [BIDIOE] and 15 [BIDIMODE] of the SPI_CR1 register
 */
#define SPI_BUS_CONFIG_FD				1	//FUll duplex mode
#define SPI_BUS_CONFIG_HD				2	//Half duplex mode
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3	//Simplex mode Reception only

/*
 * @SPI_SclkSpeed			// bits 3 to 5 [BR[2:0]] of the SPI_CR1 register
 */
#define SPI_SCLK_SPEED_DIV2					0	//Fspi clock divided by 2
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7	//Fspi clock divided by 256

/*
 * @SPI_DFF					// bit 11 [DFF] of the SPI_CR1 register
 */
#define SPI_DFF_8							0	//Data frame format, either 8 or 16bits (default)
#define SPI_DFF_16							1

/*
 * @SPI_CPOL				// bit 1 [CPOL] of the SPI_CR1 register
 */
#define SPI_CPOL_LOW						0	//Clock polarity
#define SPI_CPOL_HIGH						1

/*
 * @SPI_CPHA				// bit 0 [CPHA] of the SPI_CR1 register
 */
#define SPI_CPHA_LOW						0	//Clock phase
#define SPI_CPHA_HIGH						1

/*
 * @SPI_SSM					// bit 9 [SSM] of the SPI_CR1 register
 */
#define SPI_SSM_DI							0	//Hardware slave management enabled or
#define SPI_SSM_EN							1	//Software slave management enabled

/*
 * SPI Interrupt states
 */

#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/*
 * Possible SPI application events
 */

#define SPI_EVENT_TX_CMPLT 					1	//SPI event transmission complete
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4


/*
 * SPI related status flags definitions
 */

#define SPI_TXE_FLAG 		(1 << SPI_SR_TXE) //this flag is giving the masking info TXE field in the Status register
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/**********************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about APIs check the function definitions
 *********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableorDisable);
/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);		//Arguments: SPI 1 or 2, pointer of the Transmission buffer and Size of the data transfer
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle,uint8_t *pTxBuffer, uint32_t Len);		//Interrupt based sending/receiving data
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi); //SS pin control when master

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx); //application may call this function to clear OVR flag
void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle);
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv);


#endif /* INC_STM32L0732RZ_SPI_DRIVER_H_ */
