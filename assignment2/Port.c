#include "Port.h"
#include "GPIO.h"

#define NULL ((void*)0)

static CallbackType PortCD_Callback= NULL;

PortConfig_Type pUserConfig = {
    .Mux = PORT_MUX_GPIO,
    .Pull = PORT_PULL_DISABLE
};

PortConfig_Type PortBTNConfig = {
    .Mux = PORT_MUX_GPIO,
    .Pull = PORT_PULL_UP,
    .IntType = PORT_INTERRUPT_FALLING_EDGE,
    .Callback = &ButtonHandler
};

void PORT_Init(PortConfig_Type * PORT_Config) {
    // Set up Pull
    if (PORT_Config->Pull != PORT_PULL_DISABLE) {
        PORT_Config->PORTx->PCR[PORT_Config->pin_number] |= PORT_PCR_PE_MASK; // Enable pull
        if (PORT_Config->Pull == PORT_PULL_UP) {
            PORT_Config->PORTx->PCR[PORT_Config->pin_number] |= PORT_PCR_PS_MASK; // Pull-up
        } else {
            PORT_Config->PORTx->PCR[PORT_Config->pin_number] &= ~PORT_PCR_PS_MASK; // Pull-down
        }
    } else {
        PORT_Config->PORTx->PCR[PORT_Config->pin_number] &= ~PORT_PCR_PE_MASK; // Disable pull
    }
}

void PORT_EXTI_Config(PortConfig_Type * PORT_Config) {
    // Set interrupt for one Port
    PORT_Config->PORTx->PCR[PORT_Config->pin_number] &= ~PORT_PCR_IRQC_MASK;
    PORT_Config->PORTx->PCR[PORT_Config->pin_number] |= PORT_PCR_IRQC(PORT_Config->IntType);
    PortCD_Callback = PORT_Config->Callback;
}

void PORT_EXTI_ClearFlag(PortConfig_Type * PORT_Config) {
    if (PORT_Config->pin_number < 32) {
        PORT_Config->PORTx->ISFR = (1U << PORT_Config->pin_number);  // Clear interrupt flag
    } else {
        // Handle error if necessary
    }
}


void InterruptHandler(uint8_t pin_number) {
    if (PortCD_Callback != NULL) {
        PortCD_Callback(pin_number);
    }
}

void PORTC_PORTD_IRQHandler(void) {
    if (PORTC->ISFR & (1U << 12)) {
        InterruptHandler(12);
        PORTC->ISFR = (1U << 12);  // Clear interrupt flag
    } else if (PORTC->ISFR & (1U << 3)) {
        InterruptHandler(3);
        PORTC->ISFR = (1U << 3);  // Clear interrupt flag
    }
}

