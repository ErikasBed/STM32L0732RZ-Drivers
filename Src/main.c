/*
 * main.c
 *
 *  Created on: Sep 1, 2023
 *      Author: Test
 */

#include "STM32L0732RZ.h"
#include "STM32L0732RZ_gpio_driver.h"

int main(void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	GPIO_IRQHandling(0);
}

