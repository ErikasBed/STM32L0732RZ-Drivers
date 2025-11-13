/*
 * 001_LED_TOGGLE.c
 *
 *  Created on: Sep 3, 2023
 *      Author: 37060
 */
#include "STM32L0732RZ.h"
#include "STM32L0732RZ_gpio_driver.h"

extern void initialise_monitor_handles(void);

void delay(void)
{
	for(uint32_t i=0; i < 50000; i++);

}
int main(void)
{
	initialise_monitor_handles();
	GPIO_Handle_t GpioLed;

	//Setting up various parameters for Port A Pin 5 as it is wired to the user LED on the Nucleo board
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOA, ENABLE); 	// Enabling the peripheral clock
	GPIO_Init(&GpioLed); 					// Initializing the Port

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}

