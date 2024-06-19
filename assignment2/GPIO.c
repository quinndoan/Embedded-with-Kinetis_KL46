/*
 * GPIO.c
 *
 *  Created on: Jun 19, 2024
 *      Author: Nitro Tiger
 */

#include "GPIO.h"
#include "stdint.h"

void GPIO_Init(GPIO_Config_Type * GPIO_Config) {
    // Enable Port for Clock
    if (GPIO_Config->port == PORTA) {
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    } else if (GPIO_Config->port == PORTB) {
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    } else if (GPIO_Config->port == PORTC) {
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    } else if (GPIO_Config->port == PORTD) {
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    } else if (GPIO_Config->port == PORTE) {
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    }

    // Set MUX as GPIO
    GPIO_Config->port->PCR[GPIO_Config->pin_number] = PORT_PCR_MUX(1);

    // Set GPIO_Pin
    if (GPIO_Config->direction == 1) {
        GPIO_Config->fgpio->PDDR |= (1U << GPIO_Config->pin_number); // Set as output

        if (GPIO_Config->initialValue) {
            GPIO_Config->fgpio->PSOR = (1U << GPIO_Config->pin_number); // Set pin
        } else {
            GPIO_Config->fgpio->PCOR = (1U << GPIO_Config->pin_number); // Clear pin
        }
    } else {
        GPIO_Config->fgpio->PDDR &= ~(1U << GPIO_Config->pin_number); // Set as input
    }
}

void GPIO_SetPin(FGPIO_Type * FGPIOx, uint8_t pin_number) {
    if (pin_number < 32) {
        FGPIOx->PSOR = (1U << pin_number); // Use PSOR to set pin
    } else {
        // Error
    }
}

uint8_t GPIO_ReadPin(FGPIO_Type * FGPIOx, uint8_t pin_number) {
    if (pin_number < 32) {
        // Read from PDIR
        return (FGPIOx->PDIR & (1U << pin_number)) ? 1 : 0;
    } else {
        // error
    }
}

void GPIO_TogglePin(FGPIO_Type * FGPIOx, uint8_t pin_number) {
    if (pin_number < 32) {
        // Toggle using PTOR
        FGPIOx->PTOR = (1U << pin_number);
    } else {
        // Error
    }
}


