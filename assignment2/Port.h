/*
 * Port.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Nitro Tiger
 */

#ifndef PORT_H_
#define PORT_H_

#include<stdint.h>
#include "MKL46Z4.h"


typedef enum {
    PORT_PULL_DISABLE = 0U,
    PORT_PULL_DOWN = 2U,
    PORT_PULL_UP = 3U
} PortPull_t;

typedef enum {
    PORT_MUX_ANALOG = 0U,
    PORT_MUX_GPIO = 1U
} PortMux_t;

typedef enum {
    PORT_INTERRUPT_DISABLE = 0X0U,
    PORT_INTERRUPT_LOGIC_ZERO = 0X8U,
    PORT_INTERRUPT_FALLING_EDGE = 0X9U
} PortInterrupt_t;

typedef void (*CallbackType)(uint8_t);

typedef struct {
    PORT_Type *PORTx;
    uint8_t pin_number;
    PortMux_t Mux;
    PortPull_t Pull;
    PortInterrupt_t IntType;
    CallbackType Callback;
} PortConfig_Type;

void PORT_Init(PortConfig_Type * PORT_Config);
void PORT_EXTI_Config(PortConfig_Type * PORT_Config);
void PORT_EXTI_ClearFlag(PortConfig_Type * PORT_Config);
void ButtonHandler(uint8_t pin_number);
void InterruptHandler(uint8_t pin_number);


#endif /* PORT_H_ */


