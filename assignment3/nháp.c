#include <stdint.h>
#include <MKL46Z4.h>

uint16_t ADCData;

void UART_Initialize();
void UART_SendChar(char c);
void UART_TransmitString(uint8_t *ptr, uint8_t length);
void UART_TransmitValue(uint16_t value);
char UART_ReceiveChar(void);

void LED_Initialize() {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    PORTD->PCR[5] = PORT_PCR_MUX(1U);
    PORTE->PCR[29] = PORT_PCR_MUX(1U);

    GPIOD->PDDR |= (1U << 5);
    GPIOE->PDDR |= (1U << 29);

    GPIOD->PSOR |= (1U << 5); // Red LED off
    GPIOE->PSOR |= (1U << 29); // Green LED off
}

void ADC_Init()
{
    // Clock for ADC
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // PORT_MUX_CONTROL
    PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[22] |= PORT_PCR_MUX(0);

    // Clock Source
    ADC0->CFG1 &= ~ADC_CFG1_ADICLK_MASK;

    // HW Trigger
    ADC0->SC2 |= ADC_SC2_ADTRG_MASK;

    // Mode and 16 bit
    ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
    ADC0->CFG1 |= ADC_CFG1_MODE(3U);

    // Single or continuous
    ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;

    // Voltage reference
    ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
}


void ADC_ReadInt()
{
    // Enable ADC Interrupt and NVIC
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;
    NVIC->ISER[0] |= (1U << 15);
    GPIOD->PTOR = (1U << 5);
    // Trigger Start Conversion
    ADC0->SC1[0] = (ADC0->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(3U);
}


void ADC0_IRQHandler(void)
{
    // Read Data Register
    ADCData = ADC0->R[0];


    // Send the ADCData value via UART
    UART_TransmitValue(ADCData);

    // Clear COCO by reading ADC0->R[0]
}


void PIT_Init()
{
    // Enable Clock
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

    // Turn on PIT - MCR
    PIT->MCR &= ~PIT_MCR_MDIS_MASK;

    // Reload Value
    PIT->CHANNEL[0].LDVAL = 0x100000;
}

void PIT_StartTimer()
{
    // Enable Timer
	GPIOE->PTOR = (1U << 29);
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;

}

void UART_Initialize(){
    // Enable clock for UART0 and Port A
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // Configure PTA1 as UART0_RX and PTA2 as UART0_TX
    PORTA->PCR[1] = PORT_PCR_MUX(2U);  // Set PTA1 to UART0_RX
    PORTA->PCR[2] = PORT_PCR_MUX(2U);  // Set PTA2 to UART0_TX

    // Enable pull-up resistors on PTA1 and PTA2
    PORTA->PCR[1] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[2] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    // Disable UART0 transmitter and receiver before configuration
    UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);

    // Configure MCGIRCLK as UART0 clock source
    MCG->C1 |= MCG_C1_IRCLKEN_MASK; // Enable MCGIRCLK
    MCG->C2 |= MCG_C2_IRCS_MASK;    // Select fast internal reference clock
    MCG->SC &= ~MCG_SC_FCRDIV_MASK; // Set FCRDIV to 0 (divide by 1)

    // Select MCGIRCLK as UART0 clock source
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(3U);

    // Set baud rate (assuming a baud rate of 9600)
    uint16_t sbr = 26; // SBR value for 9600 baud with 4 MHz clock
    UART0->BDH = (sbr >> 8) & UART0_BDH_SBR_MASK;
    UART0->BDL = sbr & UART0_BDL_SBR_MASK;

    // Configure oversampling ratio to 16
    UART0->C4 = (UART0->C4 & ~UART0_C4_OSR_MASK) | UART0_C4_OSR(15);

    // Enable both edge sampling
    UART0->C5 |= UART0_C5_BOTHEDGE_MASK;

    // Configure 8-bit data, no parity, 1 stop bit
    UART0->C1 &= ~(UART0_C1_M_MASK | UART0_C1_PE_MASK);
    UART0->BDH &= ~UART0_BDH_SBNS_MASK;

    // Enable UART0 transmitter and receiver
    UART0->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;
}

void UART_SendChar(char c){
    // Wait for transmit data register empty flag
    while (!(UART0->S1 & UART0_S1_TDRE_MASK));
    // Send character
    UART0->D = c;
    // Wait for transmission complete flag
    while (!(UART0->S1 & UART0_S1_TC_MASK));
}

void UART_TransmitString(uint8_t *ptr, uint8_t length){
    for (uint8_t i = 0; i < length; i++){
        UART_SendChar(ptr[i]);
    }
}

void UART_TransmitValue(uint16_t value){
    char buffer[10];
    int length = sprintf(buffer, "%u\r\n", value);
    UART_TransmitString((uint8_t *)buffer, length);
}

char UART_ReceiveChar(void){
    // Wait for receive data register full flag
    while (!(UART0->S1 & UART0_S1_RDRF_MASK));
    // Read received character
    return UART0->D;
}


void main()
{
	SystemCoreClockUpdate();
    ADC_Init();
    PIT_Init();
    UART_Initialize();
    LED_Initialize();


    // Interconnect PIT_ADC
    SIM->SOPT7 &= ~SIM_SOPT7_ADC0TRGSEL_MASK;
    SIM->SOPT7 |= SIM_SOPT7_ADC0TRGSEL(4U); // Trigger source is PIT
    SIM->SOPT7 &= ~SIM_SOPT7_ADC0ALTTRGEN_MASK;
    SIM->SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN(1U);


    while(1) {
    	 // Select ADC Channel
    	    ADC_ReadInt();

    	    // Start Timer
    	    PIT_StartTimer();

        // Main loop
    }
}