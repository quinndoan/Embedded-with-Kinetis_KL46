#include<stdint.h>
#include<MKL46Z4.h>
void SystemCoreClockUpdate(void);

typedef enum
{
	PORT_PULL_DISABLE = 0U,
	PORT_PULL_DOWN  = 2U,
	PORT_PULL_UP    = 3U,
} PortPull_t;

typedef enum
{
	PORT_MUX_ANALOG  = 0U,
	PORT_MUX_GPIO    = 1U,
}PortMux_t ;

typedef enum
{
	Button_1  = 1,
	Button_2 = 2,
	Non     = 3,
} Status_t ;
typedef struct
{
	PortMux_t  mux;
	PortPull_t  pull;
} PortConfig_t;

PortConfig_t PortLEDConfig = {
		.mux = PORT_MUX_GPIO,
		.pull = PORT_PULL_DISABLE
};

PortConfig_t PortBTNConfig = {
		.mux = PORT_MUX_GPIO,
		.pull = PORT_PULL_UP
};


void delay(uint32_t TimeDelay)
{
	uint32_t i;
	for (i=0; i < TimeDelay * 2000; i++)
	{
		__asm("nop");
	}
}
void PORTPin_Init(PORT_Type *  PORTx, uint8_t Pin, PortConfig_t * PUserConfig)
{
	// MUX control: configure Pin
	PORTx->PCR[Pin] &= ~ PORT_PCR_MUX_MASK;
	PORTx->PCR[Pin] |= PORT_PCR_MUX(PUserConfig->mux);

	if ( PUserConfig->pull == 2)
	{
		PORTx->PCR[Pin] |= PORT_PCR_PE(1U);
		PORTx->PCR[Pin] &= ~ PORT_PCR_PS_MASK;
	}
	else if (PUserConfig->pull == 3)
	{
		PORTx->PCR[Pin] |= PORT_PCR_PE(1U);
		PORTx->PCR[Pin] |= PORT_PCR_PS(1U);
	}
	else
	{
		PORTx->PCR[Pin] &= ~PORT_PCR_PE_MASK;
	}
}

void Systick_Init100ms()
{
	// Clock Source  - processor
	SysTick ->CTRL |= (1U << SysTick_CTRL_CLKSOURCE_Pos);
	// Interrupt?
	SysTick->CTRL |= (1U << SysTick_CTRL_TICKINT_Pos);
	//Reload Value
	SysTick->LOAD = 10000000;

// enable in runtime SysTick
// SysTick->CTRL |= (1U << SysTick_CTRL_ENABLE_Pos);
}



void SysTick_Handler(void)
{
// toggle Led
	GPIOD->PTOR = (1U << 5);
// ADC
//
}


uint16_t ADCData;
void ADC0_IRQHandler(void)
{
	// Read Data Register
	ADCData = ADC0 ->R[0];
	// handle Data
	//control Led
	// Transmit to PC
}
void ADC_unit()
{
	// clock for ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0(1U);
	// Clock for Port E
	SIM->SCGC5 |= SIM_SCGC5_PORTE(1U);
	// Port MUX Control (PTE22 - ADC_SE3)
	PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;
	// enable clock source
	ADC0->CFG1 &= ~ ADC_CFG1_ADICLK_MASK;
	// HW or SW trigger
	//ADC0 -> SC2 &= ~ ADC_SC2_ADTRG_MASK; // sw trigger
	ADC0 -> SC2 |=  ADC_SC2_ADTRG(1U); //hw trigger
	// voltage reference
	ADC0->SC2 &= ~ ADC_SC2_REFSEL_MASK;
// Conversion control ( mode( (single or differential) and resolution) + single or continues convert)
	// Mode(Single or differential   + resolution(16bit )
ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;// SC1[0] = SC1[A]
	ADC0 ->CFG1 &= ~ ADC_CFG1_MODE_MASK;
	ADC0 ->CFG1 |= ADC_CFG1_MODE(3U);
	// (Single or continues convert)
	ADC0 -> SC3 &= ~ADC_SC3_ADCO_MASK;
}
//
uint16_t ADC_Readpoll()
{
	// trigger Start conversion( SC1n[ADCH])
	ADC0 -> SC1[0] = (ADC0 ->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(3U);
	// wait ECO =1
	while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0U);// write in SC1[A] or read R[A] to clear CoCo
	// Read Data Register
	return ADC0 -> R[0];
}

void ADC_ReadInter()
{
	// enable ADC Interrupt/ NVIC
	ADC0->SC1[0] |= ADC_SC1_AIEN(1U);
	NVIC ->ISER[0] |= (1U << 15);

	// Trigger Start Conversion
	ADC0 -> SC1[0] = (ADC0 ->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(3U);
}

// PIT
void PIT_Init()
{
	// enable Clock
	SIM->SCGC6 |= SIM_SCGC6_PIT(1U);
	// turn on PIT - MCR[MDIS]
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	// reload value
	PIT->CHANNEL[0].LDVAL = 0x100000;
}

void PIT_StartTimer()
{
	// enable Timer TCTRL[TEN]
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN(1U);
}

// UART
void UART_Init()
{
	// Clock
	SIM->SCGC4 |= SIM_SCGC4_UART0(1U);
//	SIM->SOPT2 =(SIM->SOPT2 & ~SIM_SOPT2_UART0SRC_MASK) | SIM_SOPT2_UART0SRC(3U);//UART0 Clock Source Select is MCGIRCLK clock
	// Port TXpin, RXpin, MUX= UART, Pullup
	//PTA1: UART0-RX, PTA2:UART0-TX
	// confugure PTA1, PTA2 are UART
	SIM->SCGC5 |= SIM_SCGC5_PORTA(1U);
	PORTA->PCR[1] = (PORTA->PCR[1] & ~PORT_PCR_MUX_MASK)| PORT_PCR_MUX(2U);
	PORTA->PCR[2] = (PORTA->PCR[2] & ~PORT_PCR_MUX_MASK)| PORT_PCR_MUX(2U);
	// confugure PTA1, PTA2 are Pullup
	PORTA->PCR[1] |= PORT_PCR_PE(1U);
	PORTA->PCR[1] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[2] |= PORT_PCR_PE(1U);
	PORTA->PCR[2] &= ~PORT_PCR_PS_MASK;
	// disable TE, RE to write UART registers
	UART0->C2 &= ~ UART0_C2_TE_MASK;
	UART0->C2 &= ~UART0_C2_RE_MASK;
	// confugure PTA1: UART0-RX, PTA2:UART0-TX

	// Baudrate( Clock, 2 dividers)
	MCG->C2 |= MCG_C2_IRCS(1U); // Fast internal reference clock selected.
	MCG->SC = (MCG->SC & ~MCG_SC_FCRDIV_MASK) | MCG_SC_FCRDIV(0U);
	MCG->C1 |= MCG_C1_IRCLKEN(1U);//Internal Reference Clock Enable
//	MCG->C1 =(MCG->C1 & ~MCG_C1_CLKS_MASK) | MCG_C1_CLKS(1U) ;// Internal reference clock is selected.

	UART0->BDH &= ~UART0_BDH_SBR_MASK;
	UART0->BDL =(UART0->BDL & ~UART0_BDL_SBR_MASK) | UART0_BDL_SBR(26U);// divider SBR
	UART0->C4 =(UART0->C4 &  ~ UART0_C4_OSR_MASK) | UART0_C4_OSR(15U);// divider OSR
	SIM->SOPT2 =(SIM->SOPT2 & ~SIM_SOPT2_UART0SRC_MASK) | SIM_SOPT2_UART0SRC(3U);//UART0 Clock Source Select is MCGIRCLK clock
// Frame
	// Data Length( 8/9/10) -8
	UART0->C1 &= ~UART0_C1_M_MASK;
	UART0->C4 &= ~UART0_C4_M10_MASK;
	//Parity( Non, odd, even)- non
	UART0->C1 &= ~ UART0_C1_PE_MASK;
	// Number of stop Bits(1/2) - 1

	// other ( LSB frist,Invert, Loopback...)
	// LSB
	UART0->S2 &= ~UART0_S2_MSBF_MASK;
	// No invert
	UART0->S2 &= ~ UART0_S2_RXINV_MASK; // no invert receive
	UART0->C3 &= ~ UART0_C3_TXINV_MASK; // no invert data transmit

}

void UART_SendString(uint8_t *pStr, uint8_t len )
{
	// Enable Transmitter(TE)
	UART0->C2 |= UART0_C2_TE_MASK;
	uint8_t idx = 0 ;
	for(idx = 0; idx < len; idx++)
	{
		// wait TDRE = 1 // thanh ghi rong
		while ((UART0->S1 & UART0_S1_TDRE_MASK) == 0);
		UART0->D = pStr[idx];
		// Wait TC = 1
		while ((UART0->S1 & UART0_S1_TC_MASK) == 0);
	}

	// disable Transmitter(TE)
	UART0->C2 &= ~UART0_C2_TE_MASK;
}


uint8_t UART_GetChar()
{
	// enable Receiver(RE)
	UART0->C2 |= UART0_C2_RE_MASK;
	// Wait RDRF =1
	while ((UART0->S1 & UART0_S1_RDRF_MASK) == 0);
	// c = RxBuffer
	return UART0->D;
	// Disable Receiver
	UART0->C2 &= ~UART0_C2_RE_MASK;
}

void main()
{
	uint8_t d;
	SystemCoreClock;
// enable clock for Port C, D
	SIM->SCGC5 |= SIM_SCGC5_PORTC(1U);
	SIM ->SCGC5 |= SIM_SCGC5_PORTD(1U);
	SIM ->SCGC5 |= SIM_SCGC5_PORTE(1U);
// configure LEDs
	//GREEN LED
	PORTPin_Init(PORTD, 5U, &PortLEDConfig);
	//red LED
	PORTPin_Init(PORTE, 29, &PortLEDConfig);

	GPIOD ->PDDR |= (1U << 5); // green Led
	GPIOE ->PDDR |= (1U << 29); // red Led
	UART_Init();
	UART_SendString("ABC", 3 );
	while(1)
	{

	}
}
