/*
 * gpio_drv.h
 *
 *  Created on: May 28, 2024
 *      Author: admin
 */

#ifndef GPIO_GPIO_DRV_H_
#define GPIO_GPIO_DRV_H_

void vDoConfigurePin(gpio_ports Port,gpio_pins Pin,eModesForGPIOx_MODER u2Mode);
void vDoSetPin(gpio_ports Port, gpio_pins Pin);
void vDoResetPin(gpio_ports Port, gpio_pins Pin);
void vDoSelectPinSpeed(gpio_ports Port,gpio_pins Pin,eModesForGPIOx_OSPEEDR pin_speed);
void vDoCfgOutTypePins(gpio_ports Port,gpio_pins Pin,eModesForPIOx_OTYPER output_type);
void vDoCfgInputOutputTypePins(gpio_ports Port,gpio_pins Pin,eModesForGPIOx_PUPDR io_type);
volatile bool bReadInputPin(gpio_ports Port,gpio_pins Pin);
void vDoSetStateOutputPin(gpio_ports Port, gpio_pins Pin, bool state_of_pin);

#endif /* GPIO_GPIO_DRV_H_ */
