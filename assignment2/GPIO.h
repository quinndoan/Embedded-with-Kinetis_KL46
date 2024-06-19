/*
 * GPIO.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Nitro Tiger
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "Port.h"



typedef struct {
    PORT_Type *port;
    FGPIO_Type *fgpio;
    uint8_t pin_number;
    uint8_t direction;    // 0 as input, 1 as output
    uint8_t initialValue; // Initial value
} GPIO_Config_Type;

void GPIO_Init(GPIO_Config_Type * GPIO_Config);
void GPIO_SetPin(FGPIO_Type * FGPIOx, uint8_t pin_number);
uint8_t GPIO_ReadPin(FGPIO_Type * FGPIOx, uint8_t pin_number);
void GPIO_TogglePin(FGPIO_Type * FGPIOx, uint8_t pin_number);

#endif /* GPIO_H_ */


