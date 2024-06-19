#include "MKL46Z4.h"
#include "stdint.h"
// Hardware Trigger compare to Software Trigger for ADC
uint16_t ADCData;

void ADC_Init(){

	// Clock for ADC
	SIM->SCGC5 |= SIM_SCGC5_PORTE(1U);
	SIM->SCGC6 |= SIM_SCGC6_ADC0(1U);


	// Port Mux control
	PORTE->PCR[22] &= PORT_PCR_MUX_MASK;
	//Clock Source
	ADC0->CFG1 &= ~ADC_CFG1_ADICLK_MASK;

	//SW Trigger
	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;    // softwareTrigger

	// Mode + Resolution
	ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
	// Resolution
	ADC0->CFG1 |= ~ADC_CFG1_MODE(3U);

	// Single or
	ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;

	// voltage reference(3V)
	ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;

}

uint16_t ADC_ReadPoll(void){
	// start trigger conversion
	ADC0->SC1[0] = ((ADC0->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(3U));

	//
	// End of Conversion
	while (0U == (ADC0->SC1[0] & ADC_SC1_ADCH_MASK));
	return ADC0->R[0];

}
void ADC_ReadInterrupt(void){
	// Enable ADC interrupt, NVIC
	ADC0->SC1[0] |= ADC_SC1_AIEN(1U);
	NVIC->ISER[0] = (1U<<15);
	// Trigger
	ADC0->SC1[0] = ((ADC0->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(3U));

}
void ADC0_IRQHandler(void){
	// Read Data Register
	ADCData = ADC0->R[0];
}

void PIT_Init(void){
	// Clock for PIT (SCGC)
	SIM->SCGC6 |= SIM_SCGC6_PIT(1U);
	// Enable PIT Module
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	//PIT->MCR &= PIT_MCR_FRZ_MASK;

	// Add load for PIT
	PIT->CHANNEL->LDVAL  = 1000;

	// Enable Timer and Interrupt using PIT(NVIC)
	PIT->CHANNEL->TCTRL |= (1U<< PIT_TCTRL_TEN_SHIFT);
	PIT->CHANNEL->TCTRL |= (1U<<PIT_TCTRL_TIE_SHIFT);

}

void ADC_PIT_Init(void){
	//Clock for ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0(1U);
	// Enable Hardware Trigger
	ADC0->SC2 |= (1U<< ADC_SC2_ADTRG_SHIFT);

	// Configure ADC using PIT, interconnection
	SIM->SOPT7 = (SIM->SOPT7 & ~SIM_SOPT7_ADC0ALTTRGEN_MASK) | SIM_SOPT7_ADC0PRETRGSEL(0b0110);

	// Mode and Resolution
	ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
		// Resolution
	ADC0->CFG1 |= ~ADC_CFG1_MODE(3U);

}