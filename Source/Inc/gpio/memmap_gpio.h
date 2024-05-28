/*
 * memmap_gpio.h
 *
 *  Created on: May 28, 2024
 *      Author: admin
 */
#ifndef GPIO_MEMMAP_GPIO_H_
#define GPIO_MEMMAP_GPIO_H_

#include "std_types.h"

#define AHB1_BUS_BASE    	(0x40020000UL)

/* RCC is a pointer to RCC_Registers structure
 * RCC address is the base address of RCC module in STM32F103C8T6
 * */
#define AHB1  ((RCC_Registers *)(0x40020000UL)


typedef union
{
	uint8 MODER0:2;
	uint8 MODER1:2;
	uint8 MODER2:2;
	uint8 MODER3:2;
	uint8 MODER4:2;
	uint8 MODER5:2;
	uint8 MODER6:2;
	uint8 MODER7:2;
	uint8 MODER8:2;
	uint8 MODER9:2;
	uint8 MODER10:2;
	uint8 MODER11:2;
	uint8 MODER12:2;
	uint8 MODER13:2;
	uint8 MODER14:2;
	uint8 MODER15:2;
	uin32 GPIOx_MODER_Reg;
}GPIOx_MODER_reg;

/* Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15)
These bits are written by software to configure the I/O direction mode.
00: Input (reset state)
01: General purpose output mode
10: Alternate function mode
11: Analog mode
*/
typedef enum
{
	eInput,
	eGeneralPurposeOutput,
	eAlternateFunction,
	eAnalogMode
}eModesForGPIOx_MODER;


typedef union
{
	bool OT0:1;
	bool OT1:1;
	bool OT2:1;
	bool OT3:1;
	bool OT4:1;
	bool OT5:1;
	bool OT6:1;
	bool OT7:1;
	bool OT8:1;
	bool OT9:1;
	bool OT10:1;
	bool OT11:1;
	bool OT12:1;
	bool OT13:1;
	bool OT14:1;
	bool OT15:1;
	const uint16 reserved;
	uint32 GPIOx_OTYPER_Reg;
}GPIOx_OTYPER_reg;

/*
Bits 31:16 Reserved, must be kept at reset value.
Bits 15:0 OTy: Port x configuration bits (y = 0..15)
These bits are written by software to configure the output type of the I/O port.
0: Output push-pull (reset state)
1: Output open-drain
*/
typedef enum
{
	eOutputPushPull,
	eOutputOpenDrain
}eModesForPIOx_OTYPER;



typedef struct AHB1_GPIO_R
{
	GPIOx_MODER_reg GPIOx_MODER;
	GPIOx_OTYPER_reg GPIOx_OTYPER;


};



#endif /* GPIO_MEMMAP_GPIO_H_ */
