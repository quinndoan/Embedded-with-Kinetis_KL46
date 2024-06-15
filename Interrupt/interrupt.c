#include<stdint.h>
#include "MKL46Z4.h"
#include<stdio.h>
#include"Port.h"
#include<string.h>

static CallBackType PortCD_CallBack =NULL;
void delay(void){
    volatile unsigned int i;
    for (i=0; i<200000; i++){
        __asm("NOP");  // no operation
    }
}

void initLedGreen(void){
	// Enable Clock for Port
		//SIM->SCGC5 |= (1U<<12);
		SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;	// Enable Port D

		// Set PTD5 as GPIO
		PORTD->PCR[5] &= ~PORT_PCR_MUX_MASK;// since MUX (10-8) alternative 1 as GPIO
		PORTD->PCR[5] |= PORT_PCR_MUX(1U);
		//Set Port as Output
		GPIOD->PDDR |= (1U<<5);

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


typedef struct{
	PortMux_t   Mux;
	PortPull_t   Pull;
	PortInterrupt_t  IntType;

}PortConfig_t;

PortConfig_t pUserConfig={
		.Mux = PORT_MUX_GPIO,
		.Pull = PORT_PULL_DISABLE
};

PortConfig_t PortBTNConfig={
		.Mux = PORT_MUX_GPIO,
		.Pull = PORT_PULL_UP,
		.IntType = PORT_INTERRUPT_FALLING_EDGE,
		.Callback = &ButtonHandler
};

void PortPin_Init(Port_type* PORTx, uint8_t Pin, PortConfig_t *pUserConfig){
	// Interrupt configure for C
	// Write into IRQC field

	// Enter exception number

	PORTx->PCR[Pin]  &= ~PORT_PCR_IRQC_MASK;
	if (0U != pUserConfig->IntType){
		PORTx->PCR[Pin] |= PORT_PCR_IRQC(pUserConfig->IntType);
		if (PORTx== PORTA || PORTx == PORTB){
			// NVIC  exception number set
			NVIC->ISER[0] = (1U<<30);

		}else if (PORTC == PORTx || PORTD == PORTx){
			PortCD_CallBack = pUserConfig->CallBack;
			NVIC->ISER[0] = (1U<<31);
		}
		else{
			// doing nothing
		}
	}
}

void PORTC_PORTD_IRQHandler(void){

	if (0U != (PORTC->PCR[12] & PORT_PCR_ISF_MASK)){
		// toggle Led
			GPIOD->PTOR = (1U<<5);
			// Clear Interrupt Status Flag -> avoid pending forever
			PORTC->PCR[12] |= PORT_PCR_ISF(1U);	// Clear to write
	}
	else if (0U != (PORTC->PCR[3] & PORT_PCR_ISF_MASK)){
		PORTC->PCR[3] |= PORT_PCR_ISF(1U);
	}

}

volatile int a=10;
void DMA0_IRQHandler(void){

	NVIC->ISPR[0] = (1U<<30);
	while(a==10);
	(void)a;
}



int main(){
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    initLedGreen();
    initButtonForSW1();
    initButtonForSW2();

    PORTC->PCR[12] |= PORT_PCR_IRQC(0b1010);
    NVIC->ISER[0] = (1U<<0);

    // set priority

    NVIC->IP[7] = 0x40400000;

    NVIC->ISER[0] = (1U<<30);

    //set pending 16
    NVIC->ISPR[0] = (1U<<0);



}