#include "stdint.h"
// sử dụng Port Set và Clear để on off led
#define SIM_SCGC5 (*(volatile uint32_t*)0x40048038)
#define PORTD_PCR5  (*(volatile uint32_t *)0x4004C014)
#define GPIO_PDDR (*(volatile uint32_t *)0x400FF0D4)// Port Data Direction Register
#define GPIO_PSOR (*(volatile uint32_t *)0x400FF0C4)// SET INPUT
#define GPIO_PCOR (*(volatile uint32_t *)0x400FF0C8)// CLEAR INPUT

void SystemCoreClockUpdate (void);

void initLed(void){
	// Enable Clock for Port
	SIM_SCGC5 |= (1U<<12);

	// Set PTD5 as GPIO
	PORTD_PCR5 = 0x00000100;// since MUX (10-8) alternative 1 as GPIO

	//Set Port as Output
	GPIO_PDDR |= (1U<<5);

}

void turn_on_led(){
	GPIO_PSOR |= (1U<<5);
}
void turn_off_led(){
	GPIO_PCOR |= (1U<<5);
}

void main(){
	initLed();
	turn_on_led();
	turn_off_led();
}
