
#include "stdint.h"
#include "MKL46Z4.h"
// không dùng bằng mà dùng |=
void delay(void){
    volatile unsigned int i;
    for (i=0; i<200000; i++){
        __asm("NOP");  // no operation
    }
}

void initLedGreen(void){
    // Set PTC12 as GPIO
    PORTC->PCR[12] = PORT_PCR_MUX(1);       // Configure GPIO
    GPIOC->PDDR |= (1U<<12);                // Set as Output
    GPIOC->PCOR |= (1U<<12);                 // Turn off Green LED
}

void initLedRed(void){
    // Set PTC3 as GPIO
    PORTC->PCR[3] = PORT_PCR_MUX(1);        // Configure GPIO
    GPIOC->PDDR |= (1U<<3);                 // Set as Output
    GPIOC->PCOR |= (1U<<3);                  // Turn off Red LED
}

void initButtonForSW1(void){
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // Pull Enable, Pull Select
    GPIOC->PDDR &= ~(1U<<3);                 // Set as Input
}

void initButtonForSW2(void){
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // Pull Enable, Pull Select
    GPIOC->PDDR &= ~(1U<<12);                // Set as Input
}

void toggle_led_Green(){
    GPIOC->PTOR |= (1U<<12);                  // Toggle Green LED
}

void blinkRed(void){
    for (int i=0; i<3; i++){                 // Blink Red LED 3 times
        GPIOC->PTOR |= (1U<<3);
        delay();
        GPIOC->PTOR |= (1U<<3);
        delay();
    }
}

void blinkBoth(void){
    for (int i=0; i<3; i++){
        GPIOC->PTOR |= (1U<<12);
        GPIOC->PTOR |= (1U<<3);	// Toggle both LEDs
        delay();
     //   GPIOC->PTOR = (1U<<12) | (1U<<3);    // Toggle both LEDs
        GPIOC->PTOR |= (1U<<12);
        GPIOC->PTOR |= (1U<<3);
        delay();
    }
}

void requirement1(void){
    toggle_led_Green();                      // Toggle Green LED
    blinkRed();                              // Blink Red LED 3 times
}

void requirement2(void){
    blinkBoth();                             // Blink both LEDs
    for (int i=0; i<3; i++){                 // Blink Green LED 3 times
        toggle_led_Green();
        delay();
        toggle_led_Green();
        delay();
    }
}

int main(){
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    initLedGreen();
    initLedRed();
    initButtonForSW1();
    initButtonForSW2();
    while (1){
        if (!(GPIOC->PDIR & (1U<<3))){       // Check if SW1 is pressed
            requirement1();
            while(!(GPIOC->PDIR & (1U<<3))); // Wait for SW1 to be released
        }
        if (!(GPIOC->PDIR & (1U<<12))){      // Check if SW2 is pressed
            requirement2();
            while(!(GPIOC->PDIR & (1U<<12))); // Wait for SW2 to be released
        }
    }
    return 0;
}