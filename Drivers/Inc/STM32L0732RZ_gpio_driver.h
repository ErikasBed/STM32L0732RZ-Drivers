/*
 * STM32L0732RZ_gpio_driver.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Test
 */

#include "STM32L0732RZ.h"

#ifndef INC_STM32L0732RZ_GPIO_DRIVER_H_
#define INC_STM32L0732RZ_GPIO_DRIVER_H_

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< Possible values from @GPIO_PIN_NUMBER >*/
	uint8_t GPIO_PinMode;			/*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< Possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< Possible values from @GPIO_PIN_PULL_UP_DOWN >*/
	uint8_t GPIO_PinOPType;			/*!< Possible values from @GPIO_PIN_OUTPUT_TYPE >*/
	uint8_t GPIO_PinAltFunMode;		/*!< Possible values from @GPIO_PIN_ALT_FNCT >*/
}GPIO_PinConfig_t;

typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; /*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig; /*!< This holds GPIO pin configuration settings>*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_Numbers
 * GPIO PIN numbers
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO PIN possible modes
 */

#define GPIO_MODE_IN 		0	//Input mode [Non-Interrupt mode]
#define GPIO_MODE_OUT 		1	//Output mode [Non-Interrupt mode]
#define GPIO_MODE_ALTFNCT 	2	//Alternative Function [Non-Interrupt mode]
#define GPIO_MODE_ANALOG 	3	//Analog mode [Non-Interrupt mode]
#define GPIO_MODE_IT_FT		4 	//Input falling edge [Interrupt mode]
#define GPIO_MODE_IT_RT		5	//Input rising edge	[Interrupt mode]
#define GPIO_MODE_IT_RFT	6	//Input rising edge/falling edge trigger [Interrupt mode]

/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO PIN possible output types
 */
#define GPIO_OP_TYPE_PP		0	//Push-pull
#define GPIO_OP_TYPE_OD		1	//Open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO PIN possible output speeds
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGHEST	3

/*
 * @GPIO_PIN_PULL_UP_DOWN
 * GPIO PIN pull-up pull-down configuration macros
 */

#define GPIO_NO_PUPD		0	//No pull-up or pull-down
#define GPIO_PIN_PU			1	//Pull-up
#define GPIO_PIN_PD			2	//Pull-Down

/*
 * GPIO PIN pull-up pull-down configuration macros
 */

/*
 * GPIO PIN pull-up pull-down configuration macros
 */


/**********************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about APIs check the function definitions
 *********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableorDisable);
/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOuputtPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);



#endif /* INC_STM32L0732RZ_GPIO_DRIVER_H_ */
