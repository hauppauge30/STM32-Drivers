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


/*
 * Pin Speed Configuration:
 *
 * Low Speed:
 *   Used for pins controlling LEDs or buttons where response times are not critical and energy consumption needs to be minimized.
 *
 * Medium Speed:
 *   Suitable for slow communications and control interfaces that do not require high speeds.
 *
 * High Speed:
 *   Used for fast communication interfaces such as SPI, I2C, or UART.
 *
 * Very High Speed:
 *   Used for high-performance applications such as display drivers or high-speed communications.
 */

void vDoSelectPinSpeed(gpio_ports Port,gpio_pins Pin,eModesForGPIOx_OSPEEDR pin_speed)
{
    // Calculate the position of the pin in the MODER register
    uint32 position = Pin * 2;

	switch(Port)
	{
		case PORTA:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOA->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOA->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTB:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOB->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOB->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTC:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOC->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOC->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTD:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOD->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOD->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTE:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOE->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOE->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTF:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOF->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOF->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTG:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOG->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOG->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
		case PORTH:
		    // Clear the existing speed for the specified pin
			AHB1_GPIOH->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
			// Set the new pin speed for the specified pin
			AHB1_GPIOH->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
	}
}


/*
 * Output push-pull (reset state)
 *
 * When the pin is configured for logic level "1", it supplies a positive voltage (VCC) to the connected load (e.g., an LED).
 * At logic level "0", it can pull the pin down to ground level (GND).
 */

/*
 * Mode of Operation: In open-drain mode, the pin can only "pull down" the electrical signal,
 * meaning it can connect the pin to ground (GND).
 *
 * Behavior: When the pin is configured for a logic level "1", it is "inactive" and does not supply any voltage.
 * At a logic level "0", it connects the pin to ground, allowing the load to sink current to GND.
 */

void vDoCfgOutTypePins(gpio_ports Port,gpio_pins Pin,eModesForPIOx_OTYPER output_type)
{
    // Calculate the position of the pin in the MODER register
    uint32 position = Pin * 1;

	switch(Port)
	{
		case PORTA:
		    // Clear the existing state for the specified pin
			AHB1_GPIOA->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOA->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTB:
		    // Clear the existing state for the specified pin
			AHB1_GPIOB->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOB->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTC:
		    // Clear the existing state for the specified pin
			AHB1_GPIOC->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOC->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTD:
		    // Clear the existing state for the specified pin
			AHB1_GPIOD->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOD->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTE:
		    // Clear the existing state for the specified pin
			AHB1_GPIOE->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOE->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTF:
		    // Clear the existing state for the specified pin
			AHB1_GPIOF->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOF->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTG:
		    // Clear the existing state for the specified pin
			AHB1_GPIOG->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOG->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
		case PORTH:
		    // Clear the existing state for the specified pin
			AHB1_GPIOH->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
			// Set the new mode for the specified pin
			AHB1_GPIOH->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
	}
}

/*
 * Pull-Down Configuration:
 *
 * In a pull-down configuration, a resistor (called a pull-down resistor) is connected between the GPIO pin and ground (GND).
 * This resistor keeps the signal on the pin at a logical "0" level when the pin is not connected to anything or is not driven high.
 *
 * Behavior:
 * - Inactive Pin: When the GPIO pin is not connected to anything or not driven high, the pull-down resistor pulls it down to ground (GND),
 *   ensuring the pin is at a logical "0" level.
 * - Active Pin: If the pin is driven and connected to a voltage source (VCC), it will be at a logical "1" level. However, when this
 *   voltage source is disconnected, the pull-down resistor will again pull the pin to a logical "0" level.
 *
 * Usage:
 * Pull-down resistors are used to prevent the GPIO pin from "floating." Without a pull-down resistor, the pin might not have a defined
 * logical level when not connected to anything, which could cause unexpected behavior in the circuit.
 */


