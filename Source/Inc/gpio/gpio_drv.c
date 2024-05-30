/*
 * gpio.c
 *
 *  Created on: May 28, 2024
 *      Author: Ostafie Razvan
 */
#include "std_types.h"
#include "memmap_gpio.h"

#define AHB1_GPIOA ((Registers_GPIO *)(GPIOA_BASE))
#define AHB1_GPIOB ((Registers_GPIO *)(GPIOB_BASE))
#define AHB1_GPIOC ((Registers_GPIO *)(GPIOC_BASE))
#define AHB1_GPIOD ((Registers_GPIO *)(GPIOD_BASE))
#define AHB1_GPIOE ((Registers_GPIO *)(GPIOE_BASE))
#define AHB1_GPIOF ((Registers_GPIO *)(GPIOF_BASE))
#define AHB1_GPIOG ((Registers_GPIO *)(GPIOG_BASE))
#define AHB1_GPIOH ((Registers_GPIO *)(GPIOH_BASE))
#define AHB1_GPIOI ((Registers_GPIO *)(GPIOI_BASE))


void vDoConfigurePin(gpio_ports Port, gpio_pins Pin, eModesForGPIOx_MODER u2Mode)
{
    // Calculate the position of the pin in the MODER register
	uint32 position = Pin * 2;

    switch (Port)
    {
        case PORTA:
            // Clear the existing mode for the specified pin
            AHB1_GPIOA->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOA->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break; // Exit the switch after each case
        case PORTB:
            // Clear the existing mode for the specified pin
            AHB1_GPIOB->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOB->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        case PORTC:
            // Clear the existing mode for the specified pin
            AHB1_GPIOC->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOC->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        case PORTD:
            // Clear the existing mode for the specified pin
            AHB1_GPIOD->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOD->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        case PORTE:
            // Clear the existing mode for the specified pin
            AHB1_GPIOE->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOE->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        case PORTF:
            // Clear the existing mode for the specified pin
            AHB1_GPIOF->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOF->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        case PORTG:
            // Clear the existing mode for the specified pin
            AHB1_GPIOG->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOG->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        case PORTH:
            // Clear the existing mode for the specified pin
            AHB1_GPIOH->GPIOx_MODER.GPIOx_MODER_Reg &= ~(0x3 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOH->GPIOx_MODER.GPIOx_MODER_Reg |= (u2Mode) << position;
            break;
        default:
            // Invalid port, do nothing or handle the error accordingly
            break;
    }
}


void vDoSetPin(gpio_ports Port, gpio_pins Pin, bool pin_mode)
{
    // Calculate the position of the pin in the ODR register
	uint32 position = Pin;

    switch (Port)
    {
        case PORTA:
            // Clear the existing state for the specified pin
            AHB1_GPIOA->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOA->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break; // Exit the switch after each case
        case PORTB:
            // Clear the existing state for the specified pin
            AHB1_GPIOB->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOB->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        case PORTC:
            // Clear the existing state for the specified pin
            AHB1_GPIOC->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOC->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        case PORTD:
            // Clear the existing state for the specified pin
            AHB1_GPIOD->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOD->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        case PORTE:
            // Clear the existing state for the specified pin
            AHB1_GPIOE->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOE->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        case PORTF:
            // Clear the existing state for the specified pin
            AHB1_GPIOF->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOF->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        case PORTG:
            // Clear the existing state for the specified pin
            AHB1_GPIOG->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOG->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        case PORTH:
            // Clear the existing state for the specified pin
            AHB1_GPIOH->GPIOx_ODR.u32GPIOx_ODR_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOH->GPIOx_ODR.u32GPIOx_ODR_Reg |= (pin_mode << position);
            break;
        default:
            // Invalid port, do nothing or handle the error accordingly
            break;
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

void vDoSelectPinSpeed(gpio_ports Port, gpio_pins Pin, eModesForGPIOx_OSPEEDR pin_speed)
{
    // Calculate the position of the pin in the OSPEEDR register
    uint32 position = Pin * 2;

    switch (Port)
    {
        case PORTA:
            // Clear the existing speed for the specified pin
            AHB1_GPIOA->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOA->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break; // Exit the switch after each case
        case PORTB:
            // Clear the existing speed for the specified pin
            AHB1_GPIOB->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOB->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        case PORTC:
            // Clear the existing speed for the specified pin
            AHB1_GPIOC->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOC->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        case PORTD:
            // Clear the existing speed for the specified pin
            AHB1_GPIOD->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOD->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        case PORTE:
            // Clear the existing speed for the specified pin
            AHB1_GPIOE->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOE->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        case PORTF:
            // Clear the existing speed for the specified pin
            AHB1_GPIOF->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOF->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        case PORTG:
            // Clear the existing speed for the specified pin
            AHB1_GPIOG->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOG->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        case PORTH:
            // Clear the existing speed for the specified pin
            AHB1_GPIOH->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg &= ~(0x03 << position);
            // Set the new pin speed for the specified pin
            AHB1_GPIOH->GPIOx_OSPEEDR.GPIOx_OSPEEDR_Reg |= (pin_speed << position);
            break;
        default:
            // Invalid port, do nothing or handle the error accordingly
            break;
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

void vDoCfgOutTypePins(gpio_ports Port, gpio_pins Pin, eModesForPIOx_OTYPER output_type)
{
    // Calculate the position of the pin in the OTYPER register
	uint32 position = Pin;

    switch (Port)
    {
        case PORTA:
            // Clear the existing state for the specified pin
            AHB1_GPIOA->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOA->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break; // Exit the switch after each case
        case PORTB:
            // Clear the existing state for the specified pin
            AHB1_GPIOB->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOB->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        case PORTC:
            // Clear the existing state for the specified pin
            AHB1_GPIOC->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOC->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        case PORTD:
            // Clear the existing state for the specified pin
            AHB1_GPIOD->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOD->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        case PORTE:
            // Clear the existing state for the specified pin
            AHB1_GPIOE->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOE->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        case PORTF:
            // Clear the existing state for the specified pin
            AHB1_GPIOF->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOF->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        case PORTG:
            // Clear the existing state for the specified pin
            AHB1_GPIOG->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOG->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        case PORTH:
            // Clear the existing state for the specified pin
            AHB1_GPIOH->GPIOx_OTYPER.GPIOx_OTYPER_Reg &= ~(0x01 << position);
            // Set the new mode for the specified pin
            AHB1_GPIOH->GPIOx_OTYPER.GPIOx_OTYPER_Reg |= (output_type << position);
            break;
        default:
            // Invalid port, do nothing or handle the error accordingly
            break;
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

/*
 * Pull-Up Resistor:
 *
 * A pull-up resistor is connected between a GPIO pin and the supply voltage (VCC). It ensures that the pin is at a defined
 * logic level "1" when it is not actively driven by another source. This prevents the pin from floating, which could result
 * in unpredictable behavior. Pull-up resistors are commonly used with input pins to guarantee a default high state.
 *
 * For example, when a button is connected to an input pin with a pull-up resistor, the pin reads as logic level "1"
 * when the button is not pressed. When the button is pressed, the pin is connected to ground (GND), and it reads as logic level "0".
*/

/*
 * Available configurations:
 * 	eNoPullUpOrPullDown
 *	ePullUp
 *	ePullDown
 */
void vDoCfgInputOutputTypePins(gpio_ports Port, gpio_pins Pin, eModesForGPIOx_PUPDR io_type)
{
    // Calculate the position of the pin in the MODER register
	uint32 position = Pin * 2;

    switch (Port)
    {
        case PORTA:
            // Clear the existing value for the specified pin
            AHB1_GPIOA->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOA->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break; // Important: Exit the switch after each case
        case PORTB:
            // Clear the existing value for the specified pin
            AHB1_GPIOB->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOB->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        case PORTC:
            // Clear the existing value for the specified pin
            AHB1_GPIOC->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOC->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        case PORTD:
            // Clear the existing value for the specified pin
            AHB1_GPIOD->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOD->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        case PORTE:
            // Clear the existing value for the specified pin
            AHB1_GPIOE->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOE->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        case PORTF:
            // Clear the existing value for the specified pin
            AHB1_GPIOF->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOF->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        case PORTG:
            // Clear the existing value for the specified pin
            AHB1_GPIOG->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOG->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        case PORTH:
            // Clear the existing value for the specified pin
            AHB1_GPIOH->GPIOx_PUPDR.GPIOx_PUPDR_Reg &= ~(0x03 << position);
            // Set the new value of config pullup/pulldown for the specified pin
            AHB1_GPIOH->GPIOx_PUPDR.GPIOx_PUPDR_Reg |= (io_type << position);
            break;
        default:
            // Invalid port, do nothing or handle the error accordingly
            break;
    }
}


volatile bool bReadInputPin(gpio_ports Port, gpio_pins Pin)
{
	uint32 position = Pin;
    volatile uint32 *reg;

    switch (Port)
    {
        case PORTA:
            reg = &(AHB1_GPIOA->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTB:
        	reg = &(AHB1_GPIOB->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTC:
        	reg = &(AHB1_GPIOC->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTD:
        	reg = &(AHB1_GPIOD->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTE:
        	reg = &(AHB1_GPIOE->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTF:
        	reg = &(AHB1_GPIOF->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTG:
        	reg = &(AHB1_GPIOG->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        case PORTH:
        	reg = &(AHB1_GPIOH->GPIOx_IDR.u32GPIOx_IDR_Reg);
            break;
        default:
            return 0; // Invalid port
    }

    // Verificăm dacă bitul corespunzător pinului este setat
    return (*reg & (1 << position)) != 0;
}



/*
În contextul STM32 și a altor microcontrolere, atomicitatea se referă la capacitatea unei operații de a fi
executată complet și fără întreruperi de alte operații. O operație atomică este una care se realizează ca
o singură unitate indivizibilă, fără a permite intervenția sau modificarea de către alte operații între
începutul și sfârșitul ei.

În practică, atomicitatea este crucială atunci când lucrăm cu dispozitive periferice și stări de pin,
deoarece vrem să ne asigurăm că operațiile noastre sunt finalizate complet și fără niciun fel de interferențe.
De exemplu, atunci când setăm sau resetăm starea unui pin GPIO pentru a controla un LED sau un alt dispozitiv,
vrem să ne asigurăm că acest lucru se întâmplă într-o singură mișcare, fără să existe posibilitatea ca starea
să fie alterată între operațiile individuale care ar putea fi necesare pentru a realiza această acțiune.

Atomicitatea este adesea obținută prin intermediul unor mecanisme hardware sau software specifice,
cum ar fi instrucțiunile atomice ale procesorului sau accesul protejat la resurse partajate.
În cazul STM32, unele registre și operații oferă această caracteristică, permițând dezvoltatorilor
să creeze aplicații sigure și robuste fără a fi nevoie să se îngrijoreze de condițiile de cursă în
timpul manipulării datelor și a stărilor de pin.
 */
void vDoSetStateOutputPin(gpio_ports Port, gpio_pins Pin, bool state_of_pin)
{
	uint32 position = Pin;

    volatile uint32 *reg;
    switch (Port)
    {
        case PORTA:
            reg = &(AHB1_GPIOA->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTB:
            reg = &(AHB1_GPIOB->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTC:
            reg = &(AHB1_GPIOC->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTD:
            reg = &(AHB1_GPIOD->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTE:
            reg = &(AHB1_GPIOE->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTF:
            reg = &(AHB1_GPIOF->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTG:
            reg = &(AHB1_GPIOG->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        case PORTH:
            reg = &(AHB1_GPIOH->GPIOx_BSRR.u32GPIOx_BSRR_Reg);
            break;
        default:
            return; // Invalid port
    }

    if (state_of_pin == HIGH)
    {
        *reg = (1 << position); // Set pin
    }
    else
    {
        *reg = (1 << (position + 16)); // Reset pin
    }
}

