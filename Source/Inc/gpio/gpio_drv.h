/*
 * gpio_drv.h
 *
 *  Created on: May 28, 2024
 *      Author: admin
 */

#ifndef GPIO_GPIO_DRV_H_
#define GPIO_GPIO_DRV_H_

void vDoConfigurePin(gpio_ports Port,gpio_pins Pin,eModesForGPIOx_MODER u2Mode);
void vDoSetPin(gpio_ports Port,gpio_pins Pin,bool pin_mode);

#endif /* GPIO_GPIO_DRV_H_ */
