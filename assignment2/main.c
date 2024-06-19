/*
 * main.c
 *
 *  Created on: Jun 19, 2024
 *      Author: Nitro Tiger
 */


#include "Middleware.h"

int main() {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    LED_Init();
    BTN_Init();

    PortConfig_Type button_sw2_interrupt_config = {
        .PORTx = PORTC,
        .pin_number = 12,
        .Mux = PORT_MUX_GPIO,
        .Pull = PORT_PULL_UP,
        .IntType = PORT_INTERRUPT_FALLING_EDGE,
        .Callback = ButtonHandler
    };
    PORT_EXTI_Config(&button_sw2_interrupt_config);

    NVIC->ISER[0] = (1U << 31);

    while (1) {
        // Main loop
    }
}
