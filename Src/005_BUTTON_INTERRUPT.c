/*
 * 005_BUTTON_INTERRUPT.c
 *
 *  Created on: Sep 7, 2023
 *      Author: Erikas
 */

#include "STM32L0732RZ.h"
#include "STM32L0732RZ_gpio_driver.h"
#include <string.h>

#define LOW 0
#define	BTN_PRESSED LOW
extern void initialise_monitor_handles(void);

void delay(void)
{
	for(uint32_t i=0; i < 50000; i++);

}
int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	//In case if we dont define all GPIO_PinConfig parameters, we have to use such an notation,
	//Otherwise the wrong fiels might get filled in with wrong data
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioLed));

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
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_IT_FT; //falling edge interrupt
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD; //External pull up resistor is already on the board


	GPIO_PeriClockControl(GPIOA, ENABLE); 	// Enabling the peripheral clock
	GPIO_Init(&GpioBtn); 					// Initializing the Port
	GpioBtn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_IN; //falling edge interrupt
	GPIO_Init(&GpioBtn);

	GPIO_WriteToOuputtPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET); //setting pin to low actually

	//IRQ configurations for the PIN
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI4_15,NVIC_IRQ_PRI3);	//Priority config, unecessary here as only 1 type of interrupt is being used
	GPIO_IRQITConfig(IRQ_NO_EXTI0_1, ENABLE);				//interrupt on pin 13

	while(1);

}

void EXTI0_1_IRQHandler(void)
{
	//This part of the code should work when the GpioBtn is set to LOW
	//But it works only if I manually set GPIOA>MODER>MODE1 as 0x00
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_1);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}

