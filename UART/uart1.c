#include "MKL46Z4.h"
#include "stdint.h"

void UART_Init(){
	// Clock and Port (Tx and Rx for MUX)
	SIM->SCGC4 |= SIM_SCGC4_UART0(1U);
	// PORT PTA1-UART0-Rx/ PTA2 -UART0-Tx

	PORTA->PCR[1] |= PORT_PCR_MUX(2U);		// choose as UART
	PORTA->PCR[2] |= PORT_PCR_MUX(2U);

	// Pull up
	PORTA->PCR[1] |= PORT_PCR_PE(1U) | PORT_PCR_PS(1U);
	PORTA->PCR[2] |= PORT_PCR_PE(1U) | PORT_PCR_PS(1U);

	// Disable Tx and Rx, so can write on register
	UART0->C2 &= ~UART0_C2_TE_MASK;
	UART0->C2 &= ~UART0_C2_RE_MASK;

	// Baudrate (Clock and 2 DIV)

		// FCRDIV
	MCG_SC &= ~MCG_SC_FCRDIV_MASK; // dua ve bang 0, CHIA CHO 1
		// Choose Fast Internal
	MCG_C2 |= MCG_C2_IRCS(1U);


		// Choose MUX
	MCG_C1 &= ~MCG_C1_CLKS_MASK;
	MCG_C1 |= MCG_C1_CLKS(1U);

		// Set up de Clock Gate dong, ENABLE MCGIRCLK
	MCG_C1 &= ~MCG_C1_IRCLKEN_MASK;
	MCG_C1 |= MCG_C1_IRCLKEN(1U);
		// Cau hinh SIM
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(3U);

		// Viet vao Baud Rate Low
	UART_BDL &= ~UART_BDH_SBR_MASK;
	UART_BDL |= UART_BDL_SBR(0B01101000);

		// Chon Sampling
	UART0_C4 |= UART0_C4_OSR(0B00011);

	UART0_C5 |= UART0_C5_BOTHEDGE(1U);  // setup both edge

	// Frame
		// Data Length (8/9/10) -8
	UART0->C4 &= ~UART0_C4_M10_MASK;
	UART0_C1 &= ~UART0_C1_M_MASK;		 // dat ve default 0
		// Parity (None)
	UART0_C1 &= ~UART0_C1_PE_MASK;
		// Number of stop bits (1 or 2) -1
	UART0_BDH &= ~UART0_BDH_SBNS_MASK;
		// Other ( default- don't use)

	// Enable Tx và Rx
	UART0->C2 |= UART0_C2_TE(1U);
	UART0->C2 |= UART0_C2_RE(1U);
}

void UART_SendChar(char c){
	// Wait TDRE =1
	while ((UART0->S1 & UART0_S1_TDRE_MASK)==0);
	// Buffer =c
	UART0->D = c;
	while ((UART0->S1 & UART0_S1_TC_MASK)==0);
	UART0->S1 &= ~UART0_C2_TE_MASK;

}

void UART_TransmitString(uint8_t *ptr, uint8_t length){
	uint8_t i=0;
	for (i=0;i<length;i++){
		while ((UART0->S1 & UART0_S1_TDRE_MASK)==0){
			UART0->D = ptr[i];}

	}
	while ((UART0->S1 & UART0_S1_TC_MASK)==0);
}

char UART_Receive(void){
	while (((UART0->S1 & UART_S1_RDRF_MASK)==0));
	return UART0->D;
}