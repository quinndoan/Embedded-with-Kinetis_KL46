/*
 * Middleware.c
 *
 *  Created on: Jun 19, 2024
 *      Author: Nitro Tiger
 */


#include "Middleware.h"


void LED_Init() {
    GPIO_Config_Type green_led_config = {
        .port = PORTD,
        .fgpio = (FGPIO_Type*)GPIOD,
        .pin_number = 5,
        .direction = 1,
        .initialValue = 0
    };
    GPIO_Init(&green_led_config);

    GPIO_Config_Type red_led_config = {
        .port = PORTE,
        .fgpio = (FGPIO_Type*)GPIOE,
        .pin_number = 29,
        .direction = 1,
        .initialValue = 0
    };
    GPIO_Init(&red_led_config);
}

void BTN_Init() {
    GPIO_Config_Type button_sw1_config = {
        .port = PORTC,
        .fgpio = (FGPIO_Type*)GPIOC,
        .pin_number = 3,
        .direction = 0,
        .initialValue = 0
    };
    GPIO_Init(&button_sw1_config);

    GPIO_Config_Type button_sw2_config = {
        .port = PORTC,
        .fgpio = (FGPIO_Type*)GPIOC,
        .pin_number = 12,
        .direction = 0,
        .initialValue = 0
    };
    GPIO_Init(&button_sw2_config);
}

uint8_t Read_BTN(uint8_t button) {
    if (button == 1) {
        return GPIO_ReadPin((FGPIO_Type*)GPIOC, 3);
    } else if (button == 2) {
        return GPIO_ReadPin((FGPIO_Type*)GPIOC, 12);
    }
    return 0;
}

void ButtonHandler(uint8_t pin_number) {
    if (pin_number == 12) {
        GPIO_TogglePin((FGPIO_Type*)GPIOD, 5); // Toggle green LED
        PORT_EXTI_ClearFlag(&(PortConfig_Type){
            .PORTx = PORTC,
            .pin_number = 12
        });
    } else if (pin_number == 3) {
        PORT_EXTI_ClearFlag(&(PortConfig_Type){
            .PORTx = PORTC,
            .pin_number = 3
        });
    }
}
