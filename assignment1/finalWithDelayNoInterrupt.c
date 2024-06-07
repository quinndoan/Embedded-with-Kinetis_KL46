
#include "stdint.h"
#include "MKL46Z4.h"
void delay(void){
    volatile unsigned int i;
    for (i=0; i<200000; i++){
        __asm("NOP");  // no operation
    }
}

void initLedGreen(void){
	// Enable Clock for Port
		//SIM->SCGC5 |= (1U<<12);
		SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

		// Set PTD5 as GPIO
		PORTD->PCR[5] &= ~PORT_PCR_MUX_MASK;// since MUX (10-8) alternative 1 as GPIO
		PORTD->PCR[5] |= PORT_PCR_MUX(1U);
		//Set Port as Output
		GPIOD->PDDR |= (1U<<5);

		// turn_off default
		GPIOD->PCOR |= (1U<<5);
}

void initLedRed(void){
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
		PORTE->PCR[29] &= ~PORT_PCR_MUX_MASK; // since MUX (10-8) alternative 1 as GPIO
		PORTE->PCR[29] |= PORT_PCR_MUX(1U);
		//Set Port as Output
		GPIOE->PDDR |= (1U<<29);
		// turn off
		GPIOE->PCOR |= (1U<<29);
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
    GPIOD->PTOR |= (1U<<5);                  // Toggle Green LED
}

void turn_off_led(){
	GPIOD->PDOR |= (1U<<5);
	GPIOE->PDOR |= (1U<<29);
}

void blinkRed(void){
    for (int i=0; i<3; i++){                 // Blink Red LED 3 times
        GPIOE->PTOR |= (1U<<29);
        delay();
        GPIOE->PTOR |= (1U<<29);
        delay();
    }
}

void blinkBoth(void){
    for (int i=0; i<3; i++){
        GPIOD->PTOR |= (1U<<5);
        GPIOE->PTOR |= (1U<<29);	// Toggle both LEDs
        delay();
     //   GPIOC->PTOR = (1U<<12) | (1U<<3);    // Toggle both LEDs
        GPIOD->PTOR |= (1U<<5);
        GPIOE->PTOR |= (1U<<29);
        delay();
    }
}

void requirement1(void){
    toggle_led_Green();                      // Toggle Green LED
    blinkRed(); // Blink Red LED 3 times
    delay();
    delay();
    turn_off_led();
}

void requirement2(void){
    blinkBoth();                             // Blink both LEDs
}

int main(){
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    initLedGreen();
    initLedRed();
    initButtonForSW1();
    initButtonForSW2();
    turn_off_led();
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