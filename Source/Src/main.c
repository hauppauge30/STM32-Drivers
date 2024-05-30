
#include "../Inc/gpio/peripheral_pins.h"
#define AHB1_BASE (0x40020000UL)
#define GPIOA_OFFSET (0x0000UL)
#define GPIOA_BASE (AHB1_BASE + GPIOA_OFFSET)

#define MODE_R_OFFSET (0x00UL)
#define GPIOA_MODE_R   (*(volatile unsigned int *)(GPIOA_BASE + MODE_R_OFFSET))

#define OD_R_OFFSET (0x14UL)
#define GPIOA_OD_R  (*(volatile unsigned int *)(GPIOA_BASE + OD_R_OFFSET))

#define PIN5 (1U<<5)
#define LED_PIN PIN5

#define RCC_OFFSET (0x3800UL)
#define RCC_BASE  (AHB1_BASE+RCC_OFFSET)

#define AHB1_R_OFFSET (0x30UL)
#define RCC_AHB1EN_R (*(volatile unsigned int *)(RCC_BASE + AHB1_R_OFFSET))


/* First setup the BUS AHB1_BASE */

/* Configure the offset for every PORT */
#define GPIOA_OFFSET (0x0000UL)
#define GPIOAEN (1U<<0)

int main()
{
	RCC_AHB1EN_R |=GPIOAEN;
	GPIOA_MODE_R |= (1U<<10);
	GPIOA_MODE_R &=~(1U<<11);
	while(1)
	{
		GPIOA_OD_R |=LED_PIN;
	}



}
