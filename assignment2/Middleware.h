/*
 * Middleware.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Nitro Tiger
 */

#ifndef MIDDLEWARE_H_
#define MIDDLEWARE_H_

#include "GPIO.h"
#include "Port.h"

void LED_Init();
void BTN_Init();
uint8_t Read_BTN(uint8_t button);
void ButtonHandler(uint8_t pin_number);

#endif /* MIDDLEWARE_H_ */
