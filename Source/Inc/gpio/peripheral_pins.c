/*
 * Table 29. ST morpho connector on NUCLEO-F401RE, NUCLEO-F411RE, NUCLEO-F446RE
 *
 * +--------------------+--------------------+--------------------+--------------------+
 * | CN7 odd pins       | CN7 even pins      | CN10 odd pins      | CN10 even pins     |
 * +--------------------+--------------------+--------------------+--------------------+
 * | Pin | Name         | Pin | Name         | Pin | Name         | Pin | Name         |
 * |-----+--------------|-----+--------------|-----+--------------|-----+--------------|
 * |  1  | PC10         |  2  | PC11         |  1  | PC9          |  2  | PC8          |
 * |  3  | PC12         |  4  | PD2          |  3  | PB8          |  4  | PC6          |
 * |  5  | VDD          |  6  | E5V          |  5  | PB9          |  6  | PB5          |
 * |  7  | BOOT0(1)     |  8  | GND          |  7  | AVDD         |  8  | U5V(2)       |
 * |  9  | -            | 10  | -            |  9  | GND          | 10  | -            |
 * | 11  | -            | 12  | IOREF        | 11  | PA5          | 12  | PA12         |
 * | 13  | PA13(3)      | 14  | RESET        | 13  | PA6          | 14  | PA11         |
 * | 15  | PA14(3)      | 16  | +3.3V        | 15  | PA7          | 16  | PB12         |
 * | 17  | PA15         | 18  | +5V          | 17  | PB6          | 18  | -            |
 * | 19  | GND          | 20  | GND          | 19  | PC7          | 20  | GND          |
 * | 21  | PB7          | 22  | GND          | 21  | PA9          | 22  | PB2          |
 * | 23  | PC13         | 24  | VIN          | 23  | PA8          | 24  | PB1          |
 * | 25  | PC14         | 26  | -            | 25  | PB10         | 26  | PB15         |
 * | 27  | PC15         | 28  | PA0          | 27  | PB4          | 28  | PB14         |
 * | 29  | PH0          | 30  | PA1          | 29  | PB5          | 30  | PB13         |
 * | 31  | PH1          | 32  | PA4          | 31  | PB3          | 32  | AGND         |
 * | 33  | VBAT         | 34  | PB0          | 33  | PA10         | 34  | PC4          |
 * | 35  | PC2          | 36  | PC1 or PB9(4)| 35  | PA2          | 36  | -            |
 * | 37  | PC3          | 38  | PC0 or PB8(4)| 37  | PA3          | 38  | -            |
 * +--------------------+--------------------+--------------------+--------------------+
 *
 * Notes:
 * (1) BOOT0 is not available on NUCLEO-F446RE.
 * (2) U5V is connected to CN7 pin 16 on NUCLEO-F446RE.
 * (3) PA13 and PA14 are used for SWD debugging and should not be used as GPIOs.
 * (4) PC1 or PB9 and PC0 or PB8 depend on the specific board variant.
 */

#include "std_types.h"
#include "memmap_gpio.h"
#include "gpio_drv.h"
#include "peripheral_pins.h"

void vDoSetStateUserLed(bool pin_state)
{
	/* To be safe it's only boolean */
	//pin_state = pin_state != 0;
	vDoConfigurePin(PORTA,PA5, eGeneralPurposeOutput);
	vDoSelectPinSpeed(PORTA,PA5, eOSpeedLowSpeed);
	vDoCfgOutTypePins(PORTA,PA5, eOutputPushPull);
	vDoCfgInputOutputTypePins(PORTA,PA5,eNoPullUpOrPullDown);

	if(pin_state != 0)
	{
		vDoSetPin(PORTA,PA5);
	}
	else
	{
		vDoResetPin(PORTA,PA5);
	}
}



