/*
 * memmap_gpio.h
 *
 *  Created on: May 28, 2024
 *      Author: Ostafie Razvan
 */
#ifndef GPIO_MEMMAP_GPIO_H_
#define GPIO_MEMMAP_GPIO_H_

#include "std_types.h"

/* First setup the BUS AHB1_BASE */
#define AHB1_BASE (0x40020000UL)

/* Configure the offset for every PORT */
#define GPIOA_OFFSET (0x0000UL)
#define GPIOB_OFFSET (0x0400UL)
#define GPIOC_OFFSET (0x0800UL)
#define GPIOD_OFFSET (0x0C00UL)
#define GPIOE_OFFSET (0x1000UL)
#define GPIOF_OFFSET (0x1400UL)
#define GPIOG_OFFSET (0x1800UL)
#define GPIOH_OFFSET (0x1C00UL)
#define GPIOI_OFFSET (0x2000UL)

/* Configure the base for every port */

#define GPIOB_BASE (AHB1_BASE + GPIOB_OFFSET)
#define GPIOC_BASE (AHB1_BASE + GPIOC_OFFSET)
#define GPIOD_BASE (AHB1_BASE + GPIOD_OFFSET)
#define GPIOE_BASE (AHB1_BASE + GPIOE_OFFSET)
#define GPIOF_BASE (AHB1_BASE + GPIOF_OFFSET)
#define GPIOG_BASE (AHB1_BASE + GPIOG_OFFSET)
#define GPIOH_BASE (AHB1_BASE + GPIOH_OFFSET)
#define GPIOI_BASE (AHB1_BASE + GPIOI_OFFSET)


#define GPIOA_MODER   (*(volatile unsigned int *)GPIOA_BASE + 0x00UL)
#define GPIOA_BASE (AHB1_BASE + GPIOA_OFFSET)
/* First setup the BUS AHB1_BASE */
#define AHB1_BASE (0x40020000UL)
/* Configure the offset for every PORT */
#define GPIOA_OFFSET (0x0000UL)

typedef union
{
	struct {
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
	}GPIOx_MODER_Bit_Set;
	uint32 GPIOx_MODER_Reg;
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
	struct {
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
	}GPIOx_OTYPER_Bit_Set;
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

typedef union
{
	struct {
		uint8 OSPEEDR0:2;
		uint8 OSPEEDR1:2;
		uint8 OSPEEDR2:2;
		uint8 OSPEEDR3:2;
		uint8 OSPEEDR4:2;
		uint8 OSPEEDR5:2;
		uint8 OSPEEDR6:2;
		uint8 OSPEEDR7:2;
		uint8 OSPEEDR8:2;
		uint8 OSPEEDR9:2;
		uint8 OSPEEDR10:2;
		uint8 OSPEEDR11:2;
		uint8 OSPEEDR12:2;
		uint8 OSPEEDR13:2;
		uint8 OSPEEDR14:2;
		uint8 OSPEEDR15:2;
	}GPIOx_OSPEEDR_Bit_Set;
	uint32 GPIOx_OSPEEDR_Reg;
}GPIOx_OSPEEDR_reg;


/*
 Bits 2y:2y+1 OSPEEDRy[1:0]: Port x configuration bits (y = 0..15)
These bits are written by software to configure the I/O output speed.
00: Low speed
01: Medium speed
10: Fast speed
11: High speed
 */

typedef enum
{
	eOSpeedLowSpeed,
	eOSpeedMediumSpeed,
	eOSpeedFastSpeed,
	eOSpeedHighSpeed
}eModesForGPIOx_OSPEEDR;

typedef struct
{
	struct {
		uint8  PUPDR0:2;
		uint8  PUPDR1:2;
		uint8  PUPDR2:2;
		uint8  PUPDR3:2;
		uint8  PUPDR4:2;
		uint8  PUPDR5:2;
		uint8  PUPDR6:2;
		uint8  PUPDR7:2;
		uint8  PUPDR8:2;
		uint8  PUPDR9:2;
		uint8  PUPDR10:2;
		uint8  PUPDR11:2;
		uint8  PUPDR12:2;
		uint8  PUPDR13:2;
		uint8  PUPDR14:2;
		uint8  PUPDR15:2;
	}GPIOx_PUPDR_Bit_Set;
	uint32 GPIOx_PUPDR_Reg;
}GPIOx_PUPDR_reg;

/*
Bits 2y:2y+1 PUPDRy[1:0]: Port x configuration bits (y = 0..15)
These bits are written by software to configure the I/O pull-up or pull-down
00: No pull-up, pull-down
01: Pull-up
10: Pull-down
11: Reserved
*/
typedef enum
{
	eNoPullUpOrPullDown,
	ePullUp,
	ePullDown,
	eReserved
}eModesForGPIOx_PUPDR;


typedef union
{
	struct{
		bool IDR0:1;
		bool IDR1:1;
		bool IDR2:1;
		bool IDR3:1;
		bool IDR4:1;
		bool IDR5:1;
		bool IDR6:1;
		bool IDR7:1;
		bool IDR8:1;
		bool IDR9:1;
		bool IDR10:1;
		bool IDR11:1;
		bool IDR12:1;
		bool IDR13:1;
		bool IDR14:1;
		bool IDR15:1;
		uint16 reserved;
	}GPIOx_IDR_Bit_Set;
	uint32 u32GPIOx_IDR_Reg;
}GPIOx_IDR_reg;



typedef union
{
	struct{
		bool  ODR0:1;
		bool  ODR1:1;
		bool  ODR2:1;
		bool  ODR3:1;
		bool  ODR4:1;
		bool  ODR5:1;
		bool  ODR6:1;
		bool  ODR7:1;
		bool  ODR8:1;
		bool  ODR9:1;
		bool  ODR10:1;
		bool  ODR11:1;
		bool  ODR12:1;
		bool  ODR13:1;
		bool  ODR14:1;
		bool  ODR15:1;
		uint16 reserved;
	}GPIOx_ODR_Bit_Set;
	uint32 u32GPIOx_ODR_Reg;
}GPIOx_ODR_reg;

typedef union
{
	struct{
		bool BS0:1;
		bool BS1:1;
		bool BS2:1;
		bool BS3:1;
		bool BS4:1;
		bool BS5:1;
		bool BS6:1;
		bool BS7:1;
		bool BS8:1;
		bool BS9:1;
		bool BS10:1;
		bool BS11:1;
		bool BS12:1;
		bool BS13:1;
		bool BS14:1;
		bool BS15:1;
		bool BR0:1;
		bool BR1:1;
		bool BR2:1;
		bool BR3:1;
		bool BR4:1;
		bool BR5:1;
		bool BR6:1;
		bool BR7:1;
		bool BR8:1;
		bool BR9:1;
		bool BR10:1;
		bool BR11:1;
		bool BR12:1;
		bool BR13:1;
		bool BR14:1;
		bool BR15:1;
	}GPIOx_BSRR_Bit_Set;
	volatile uint32 u32GPIOx_BSRR_Reg;
}GPIOx_BSRR_reg;


typedef union
{
	struct{
		bool LCK_K0: 1;
		bool LCK_K1: 1;
		bool LCK_K2: 1;
		bool LCK_K3: 1;
		bool LCK_K4: 1;
		bool LCK_K5: 1;
		bool LCK_K6: 1;
		bool LCK_K7: 1;
		bool LCK_K8: 1;
		bool LCK_K9: 1;
		bool LCK_K10: 1;
		bool LCK_K11: 1;
		bool LCK_K12: 1;
		bool LCK_K13: 1;
		bool LCK_K14: 1;
		bool LCK_K15: 1;
		bool LCK_K16: 1;
		uint16 reserved:15;
	}GPIOx_LCKR_Bit_Set;

	uint32 u32GPIOx_LCKR_Reg;
}GPIOx_LCKR_reg;


/* TODO: This need to be redesigned in the future */
typedef union
{
	struct{
		struct {
			bool AFRL0_bit0:1;
			bool AFRL0_bit1:1;
			bool AFRL0_bit2:1;
			bool AFRL0_bit3:1;
		}AFRL0;
		struct {
			bool AFRL1_bit0:1;
			bool AFRL1_bit1:1;
			bool AFRL1_bit2:1;
			bool AFRL1_bit3:1;
		}AFRL1;
		struct {
			bool AFRL2_bit0:1;
			bool AFRL2_bit1:1;
			bool AFRL2_bit2:1;
			bool AFRL2_bit3:1;
		}AFRL2;
		struct {
			bool AFRL3_bit0:1;
			bool AFRL3_bit1:1;
			bool AFRL3_bit2:1;
			bool AFRL3_bit3:1;
		}AFRL3;
		struct {
			bool AFRL4_bit0:1;
			bool AFRL4_bit1:1;
			bool AFRL4_bit2:1;
			bool AFRL4_bit3:1;
		}AFRL4;
		struct {
			bool AFRL5_bit0:1;
			bool AFRL5_bit1:1;
			bool AFRL5_bit2:1;
			bool AFRL5_bit3:1;
		}AFRL5;
		struct {
			bool AFRL6_bit0:1;
			bool AFRL6_bit1:1;
			bool AFRL6_bit2:1;
			bool AFRL6_bit3:1;
		}AFRL6;
		struct {
			bool AFRL7_bit0:1;
			bool AFRL7_bit1:1;
			bool AFRL7_bit2:1;
			bool AFRL7_bit3:1;
		}AFRL7;
	}GPIOx_AFRL_Bit_Set;

	uint32 u32GPIOx_AFRL_Reg;
}GPIOx_AFRL_reg;

typedef union
{
	struct {
		uint8 AFRH8:4;
		uint8 AFRH9:4;
		uint8 AFRH10:4;
		uint8 AFRH11:4;
		uint8 AFRH12:4;
		uint8 AFRH13:4;
		uint8 AFRH14:4;
		uint8 AFRH15:4;
	}GPIOx_AFRH_Bit_Set;
	uint32 u32GPIOx_AFRH_Reg;
}GPIOx_AFRH_reg;


typedef struct
{
	/* 0x40020000 */
	volatile GPIOx_MODER_reg   GPIOx_MODER;   /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x00*/
	/* 0x40020004 */
	volatile GPIOx_OTYPER_reg  GPIOx_OTYPER;  /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x04*/
	/* 0x40020008 */
	volatile GPIOx_OSPEEDR_reg GPIOx_OSPEEDR; /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x08*/
	/* 0x4002000C */
	volatile GPIOx_PUPDR_reg   GPIOx_PUPDR;   /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x0C*/
	/* 0x40020010 */
	volatile GPIOx_IDR_reg	  GPIOx_IDR;     /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x10*/
	/* 0x40020014 */
	volatile GPIOx_ODR_reg	  GPIOx_ODR;     /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x14*/
	/* 0x40020018 */
	volatile GPIOx_BSRR_reg	  GPIOx_BSRR;    /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x18*/
	/* 0x4002001C */
	volatile GPIOx_LCKR_reg    GPIOx_LCKR;    /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x1C*/
	/* 0x40020020 */
	volatile GPIOx_AFRL_reg    GPIOx_AFRL;	 /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x20*/
	/* 0x40020024 */
	volatile GPIOx_AFRH_reg    GPIOx_AFRH;    /* START ADRESS 0x40020000 + Datasheet offset for this register: 0x24*/
}Registers_GPIO;


typedef enum
{
	PA1,
	PA2,
	PA3,
	PA4,
	PA5,
	PA6,
	PA7,
	PA8,
	PA9,
	PA10,
	PA11,
	PA12,
	PA13,
	PA14,
	PA15,
}gpio_pins;

typedef enum
{
	PORTA,
	PORTB,
	PORTC,
	PORTD,
	PORTE,
	PORTF,
	PORTG,
	PORTH
}gpio_ports;



#endif /* GPIO_MEMMAP_GPIO_H_ */
