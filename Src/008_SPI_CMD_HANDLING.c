/*
 * 008_SPI_CMD_HANDLING.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Erikas
 */
#include "STM32L0732RZ.h"
#include <string.h>
#include <stdio.h>
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

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON	1
#define LED_OFF	0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //SS pin has to be pulled-up, otherwise it won't pull down the Enable pin as it will always stay high.
	GPIO_Init(&SPIPins);
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
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;				// Software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	//Setting up various parameters for Port C Pin 13 as it is wired to the user Button on the Nucleo board
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //External pull up resistor is already on the board

	GPIO_Init(&GpioBtn); 					// Initializing the Port

	//Setting up various parameters for Port C Pin 13 as it is wired to the user Button on the Nucleo board
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //External pull up resistor is already on the board


	GPIO_PeriClockControl(GPIOC, ENABLE); 	// Enabling the peripheral clock
	GPIO_Init(&GpioBtn); 					// Initializing the Port
}
void delay(void)
{
	for(uint32_t i=0; i < 50000; i++);

}

uint8_t SPI_VerifyResponse(uint8_t AckByte)
{
	if(AckByte == (uint8_t)0xF5)
		{
			//ack
			return 1;
		}
		return 0;
}

int main(void)
{
	/*
	 * First master sends a command once button is pressed. Once received command, slaves checks if the command is supported.
	 * If supported, slave will send a response a ACK byte of 0xF5 or if not supported - NACK byte of 0xA5
	 * If ACK is received by a master, it will send 1 or more command arguments, so slave would take an action according to the command
	 * If NACK is received by a master, it will display an error message
	 *
	 * Commands:
	 *
	 * 1.CMD_LED_CTR  [Pin Number, Value(On or off)] (PIN13 inbuild LED, but we use the PIN :()
	 * 2.CMD_SENSOR_READ [Analog_Pin_Number]
	 * 3.CMD_LED_READ [Pin_Number]
	 * 4.CMD_PRINT [Lenght, message(Len)]
	 * 5.CMD_ID_READ
	 */

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;


	GPIO_ButtonInit();
	SPI2_GPIOInits();

	SPI2_Inits(); //All SPI settings has to be done when SPI is disabled

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);
	while(1){

		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//1. CMD_LED_CTRL <pin no(1)>	<value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t AckByte;
		uint8_t args[2]; //arguments 1 and 2


		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off the RXNE

		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,AckByte,1);

		if(SPI_VerifyResponse(AckByte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2, args, 2);

			//Dummy read
			SPI_ReceiveData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL Executed \n");
		}

		//END of COMMAND_LED_CTRL

		//2. CMD_SENSOR_READ <analog pin number(1)>

		//wait until the button is pressed

		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		commandcode = COMMAND_SENSOR_READ;
		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);


		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&AckByte,1);

		if( SPI_VerifyResponse(AckByte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&AckByte,1);

		if( SPI_VerifyResponse(AckByte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}


		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&AckByte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(AckByte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed \n");

		}


		//5. CMD_ID_READ
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&AckByte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(AckByte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//Confirm SPI is not busy before closing the SPI comms, so no data would be lost in a middle of the transmission
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//After sending the data we disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
