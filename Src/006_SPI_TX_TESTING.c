/*
 * 006_SPI_TX_TESTING.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Erikas
 */
#include "STM32L0732RZ.h"
#include <string.h>
/*
 * SPI1_MOSI 	PA7
 * SPI1_MISO 	PA6
 * SPI1_SCK 	PA5
 * SPI1_NSS		PA4
 * Alternate functionality mode 0
 *
 * SPI2_MOSI	PB15
 * SPI2_MISO	PB14
 * SPI2_SCK		PB13
 * SPI2_NSS		PB12
 * Alternate functionality mode 0
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFNCT;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // Generates SCLK of 1 MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;				// Software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}
int main(void)
{
	char user_data[] = "Hello World";
	SPI2_GPIOInits();

	SPI2_Inits(); //All SPI settings has to be done when SPI is disabled

	//Sets NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	//send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Confirm SPI is not busy before closing the SPI comms, so no data would be lost in a middle of the transmission
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );
	//After sending the data we disable the SPI2 peripheral

	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;
}
