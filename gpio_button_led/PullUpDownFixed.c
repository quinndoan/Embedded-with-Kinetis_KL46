#include "stdint.h"
#include "MKL46Z4.h"

void delay(void){
	volatile unsigned int i;
	for (i=0;i<200000;i++){
		__asm("NOP");  // nonsense operation
	}
}

void initLed(void){
	// Enable Clock for Port
	//SIM->SCGC5 |= (1U<<12);
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	// Set PTD5 as GPIO
	PORTD->PCR[5] &= ~PORT_PCR_MUX_MASK;// since MUX (10-8) alternative 1 as GPIO
	PORTD->PCR[5] |= PORT_PCR_MUX(1U);
	//Set Port as Output
	GPIOD->PDDR |= (1U<<5);

}
void initButtonForLed(void){
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

	// Configure PC12 (Button) and the GPIO -> input (LED configure as output)
	PORTC->PCR[12] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[12] |= PORT_PCR_MUX(1U);

	// Configure as Input
	GPIOC->PDDR &= ~(1U<<12);   // PDDR: Port Data Direction, define output or input

	// Configure pull
	PORTC->PCR[12] |= PORT_PCR_PE_MASK; // check again if MAKS or SHIFT
	PORTC->PCR[12] |= PORT_PCR_PS_MASK;

}

void turn_on_led(){
	//GPIO_PSOR = (1U<<5);		//cách hai, nhanh hơn trong xử lý
	GPIOD->PDOR &= ~(1U<<5);

}
void turn_off_led(){
	//GPIO_PCOR = (1U<<5);
	GPIOD->PDOR |= (1U<<5);
}

void toggle_led(){
	GPIOD->PTOR |= (1U<<5);
}


int main(){
	initLed();
	initButtonForLed();
	turn_on_led();
	//turn_off_led();
	while (1){
		if (((GPIOC->PDIR)&(1U<<12)) ==0){
			
			// Toggle Led
			GPIOD->PTOR = (1U<<5);
		}
	}
}
