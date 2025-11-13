/*
 * 002_LED_BUTTON.c
 *
 *  Created on: Sep 4, 2023
 *      Author: Test
 */

#include "STM32L0732RZ.h"
#include "STM32L0732RZ_gpio_driver.h"

#define LOW 0
#define	BTN_PRESSED LOW
extern void initialise_monitor_handles(void);

void delay(void)
{
	for(uint32_t i=0; i < 50000; i++);

}
int main(void)
{
	initialise_monitor_handles();
	GPIO_Handle_t GpioLed, GpioBtn;

	//Setting up various parameters for Port A Pin 5 as it is wired to the user LED on the Nucleo board
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE); 	// Enabling the peripheral clock
	GPIO_Init(&GpioLed); 					// Initializing the Port

	//Setting up various parameters for Port C Pin 13 as it is wired to the user Button on the Nucleo board
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //External pull up resistor is already on the board


	GPIO_PeriClockControl(GPIOC, ENABLE); 	// Enabling the peripheral clock
	GPIO_Init(&GpioBtn); 					// Initializing the Port

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}


	}
	return 0;
}

