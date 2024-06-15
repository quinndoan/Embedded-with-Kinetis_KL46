/*
 * Port.h
 *
 *  Created on: Jun 11, 2024
 *      Author: Nitro Tiger
 */

#ifndef PORT_H_
#define PORT_H_

#include<stdint.h>
#include "MKL46Z4.h"

typedef struct {
	uint32_t PCR[32];

}Port_type;

typedef enum{
	PORT_PULL_DISABLE = 0U,
	PORT_PULL_DOWN = 2U,
	PORT_PULL_UP = 3U
}PortPull_t;

typedef enum{
	PORT_MUX_ANALOG = 0U,
	PORT_MUX_GPIO = 1U
}PortMux_t;

typedef enum{
	PORT_INTERRUPT_DISABLE = 0X0U,
	PORT_INTERRUPT_LOGIC_ZERO = 0X8U,
	PORT_INTERRUPT_FALLING_EDGE = 0X9U  // Why have 0x?
}PortInterrupt_t;

typedef void (*CallbackType)(uint8_t);

typedef struct{
	PortMux_t   Mux;
	PortPull_t   Pull;
	PortInterrupt_t  IntType;
	CallbackType Callback;
}PortConfig_t;

#endif /* PORT_H_ */