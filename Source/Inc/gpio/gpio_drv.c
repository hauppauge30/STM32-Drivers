/*
 * gpio.c
 *
 *  Created on: May 28, 2024
 *      Author: Ostafie Razvan
 */
#include "std_types.h"
#include "memmap_gpio.h"

void vDoConfigurePin(gpio_ports Port,gpio_pins Pin,eModesForGPIOx_MODER u2Mode)
{
    // Calculate the position of the pin in the MODER register
    uint32 position = Pin * 2;

	switch(Port)
	{
		case PORTA:
		    // Clear the existing mode for the specified pin
			AHB1_GPIOA->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOA->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTB:
			// Clear the existing mode for the specified pin
			AHB1_GPIOB->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOB->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTC:
			// Clear the existing mode for the specified pin
			AHB1_GPIOC->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOC->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTD:
			// Clear the existing mode for the specified pin
			AHB1_GPIOD->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOD->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTE:
			// Clear the existing mode for the specified pin
			AHB1_GPIOE->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOE->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTF:
			// Clear the existing mode for the specified pin
			AHB1_GPIOF->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOF->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTG:
			// Clear the existing mode for the specified pin
			AHB1_GPIOG->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOG->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
		case PORTH:
			// Clear the existing mode for the specified pin
			AHB1_GPIOH->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOH->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
	}
}

void vDoSetPin(gpio_ports Port,gpio_pins Pin,bool pin_mode)
{
    // Calculate the position of the pin in the MODER register
    uint32 position = Pin;

	switch(Port)
	{
		case PORTA:
		    // Clear the existing state for the specified pin
			AHB1_GPIOA->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOA->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTB:
		    // Clear the existing state for the specified pin
			AHB1_GPIOB->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOB->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTC:
		    // Clear the existing state for the specified pin
			AHB1_GPIOC->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOC->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTD:
		    // Clear the existing state for the specified pin
			AHB1_GPIOD->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOD->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTE:
		    // Clear the existing state for the specified pin
			AHB1_GPIOE->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOE->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTF:
		    // Clear the existing state for the specified pin
			AHB1_GPIOF->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOF->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTG:
		    // Clear the existing state for the specified pin
			AHB1_GPIOG->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOG->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
		case PORTH:
		    // Clear the existing state for the specified pin
			AHB1_GPIOH->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOH->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
	}
}


