void UART0_Innit(UART0_Innit_t *UART0_Info){
	//Enable Clock for UART0
	SIM->SCGC4 |= (SIM_SCGC4_UART0(1u));

	//Choose clock source for UART0
	SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(3u);			//MCGIRCLK
	MCG->C1 |= MCG_C1_IRCLKEN(1u);					//Clock_Gate
	MCG->C2 |= MCG_C2_IRCS(1u);						//Fast internal reference clock selected.
	MCG->SC &= ~(MCG_SC_FCRDIV_MASK);				//FRDIV = 1
	//PORT MUX UART0
	Port_Innit(&UART0_PIN_TX);
	Port_Innit(&UART0_PIN_RX);

	// disable TE, RE to write UART registers
	UART0->C2 &= ~ UART0_C2_TE_MASK;
	UART0->C2 &= ~UART0_C2_RE_MASK;

	//Number of stop bits
	UART0->BDH |= (UART_BDH_SBNS(UART0_Info->UART0_Stop_Bit));

	//Baud Rate Divisor
	uint16_t SBR = 4000000/(UART0_Info->Baud_Rate * 16);
	UART0->BDH &= ~(UART_BDH_SBR_MASK);
	UART0->BDH |= (UART_BDH_SBR(SBR >> 8));

	UART0->BDL &= ~(UART0_BDL_SBR_MASK);
	UART0->BDL |= (UART0_BDL_SBR(SBR));

	//Data Length
	UART0->C1 |= (UART0_C1_M(UART0_Info->Data_Length));

	//Parity
	UART0->C1 |= (UART0_Info->Parity);

	//Interrupt
	if(UART0_Info->UART0_Interrupt == Interrupt){
		if(UART0_Info->Direction == Receiver){
			UART0->C2 |= (UART0_C2_RIE(1u));
		}
		else{
			UART0->C2 |= (UART0_C2_TIE(1u));
		}
		NVIC->ISER[0] |= (1u << 12);
	}

	//Transmit order
	UART0->S2 |= (UART0_Info->Transmit_Order);

	//Sampling Ratio
	UART0->C4 &= ~(UART0_C4_OSR_MASK);
	UART0->C4 |= (UART0_C4_OSR(15u));

	//Direction
	if(UART0_Info->Direction == Receiver){
		UART0->C2 |= (UART0_C2_RE(1u));
	}
	else if(UART0_Info->Direction == Transmitter){
		UART0->C2 |= (UART0_C2_TE(1u));
	}
}

void UART0_SendChar(uint8_t Char){
	//Enable Transmitter
	UART0->C2 |= (UART0_C2_TE(1u));
	while((UART0->S1 & (1 << 7)) == 0);

	UART0->D = Char;

	while((UART0->S1 & (1 << 6)) == 0);

	//Disable Transmitter
	UART0->C2 &= ~(UART0_C2_TE_MASK);
}

void UART0_SendStr(uint8_t *ptr, uint8_t len){
	//Enable Transmitter
	uint8_t i = 0;
	UART0->C2 |= (UART0_C2_TE(1u));
	for( i = 0; i < len; i++){
		while((UART0->S1 & (1 << 7)) == 0);
		UART0->D = ptr[i];
		while((UART0->S1 & (1 << 6)) == 0);
	}
	//Disable Transmitter
	UART0->C2 &= ~(UART0_C2_TE_MASK);
}

void UART_Data_To_PC_Innit(){
	UART0_Innit_t UART0_Info;
	UART0_Info.UART0_Stop_Bit = 1u;
	UART0_Info.Baud_Rate = 9600;
	UART0_Info.Data_Length = _8_bits;
	UART0_Info.Parity = No_Parity;
	UART0_Info.UART0_Interrupt = No_Interrupt;
	UART0_Info.Direction = Transmitter;
	UART0_Info.Transmit_Order = LSB;
	UART0_Innit(&UART0_Info);
}
