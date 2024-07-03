#include "MKL46Z4.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"

void UART_Initialize();
void UART_SendChar(char c);
void UART_TransmitString(uint8_t *ptr, uint8_t length);
char UART_ReceiveChar(void);
void ADC_Initialize();
void PIT_Initialize();
void LED_Initialize();
void SystemCoreClockUpdate(void);
void ADC0_IRQHandler(void);
void PIT_IRQHandler(void);

uint16_t ADCData;

int main(void) {
    SystemCoreClockUpdate();
    UART_Initialize();
    ADC_Initialize();
    PIT_Initialize();
    LED_Initialize();


    return 0;
}

void UART_Initialize() {
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PORTA->PCR[1] = PORT_PCR_MUX(2U);
    PORTA->PCR[2] = PORT_PCR_MUX(2U);

    PORTA->PCR[1] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[2] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);

    MCG->C1 |= MCG_C1_IRCLKEN_MASK;
    MCG->C2 |= MCG_C2_IRCS_MASK;
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;

    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(3U);

    uint16_t sbr = 26;
    UART0->BDH = (sbr >> 8) & UART0_BDH_SBR_MASK;
    UART0->BDL = sbr & UART0_BDL_SBR_MASK;

    UART0->C4 = (UART0->C4 & ~UART0_C4_OSR_MASK) | UART0_C4_OSR(15);
    UART0->C5 |= UART0_C5_BOTHEDGE_MASK;

    UART0->C1 &= ~(UART0_C1_M_MASK | UART0_C1_PE_MASK);
    UART0->BDH &= ~UART0_BDH_SBNS_MASK;

    UART0->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;
}

void UART_SendChar(char c) {
    while (!(UART0->S1 & UART0_S1_TDRE_MASK));
    UART0->D = c;
    while (!(UART0->S1 & UART0_S1_TC_MASK));
}

void UART_TransmitString(uint8_t *ptr, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        UART_SendChar(ptr[i]);
    }
}

char UART_ReceiveChar(void) {
    while (!(UART0->S1 & UART0_S1_RDRF_MASK));
    return UART0->D;
}

void ADC_Initialize() {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  // Enable ADC0 clock
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; // Enable PORTE clock

    // Configure PORTE pin 22 as ADC input
    PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;

    // Configure ADC0
    ADC0->CFG1 = ADC_CFG1_ADICLK(0) |
                 ADC_CFG1_MODE(3) |      // 1-bit conversion
                 ADC_CFG1_ADIV(1);       // Clock divide by 2 (resulting in 1 MHz)

    ADC0->CFG2 = 0;                      // Default settings

    ADC0->SC2 = ADC_SC2_REFSEL(0) |      // Default voltage reference (VREFH and VREFL)
                ADC_SC2_ADTRG(0);        // Software Trigger

    ADC0->SC3 = 0;                       // Default settings, single conversion, no averaging

    // Enable ADC interrupts
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK; // Enable interrupts without selecting any channel yet
    NVIC_EnableIRQ(ADC0_IRQn);           // Enable ADC0 interrupt in NVIC
}


void PIT_Initialize() {
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR &= ~PIT_MCR_MDIS_MASK;

    PIT->CHANNEL[0].LDVAL = 0xB71B00;
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
    NVIC_EnableIRQ(PIT_IRQn);

    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void LED_Initialize() {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    PORTD->PCR[5] = PORT_PCR_MUX(1U);
    PORTE->PCR[29] = PORT_PCR_MUX(1U);

    GPIOD->PDDR |= (1U << 5);
    GPIOE->PDDR |= (1U << 29);

    GPIOD->PSOR |= (1U << 5); // Red LED off
    GPIOE->PSOR |= (1U << 29); // Green LED off
}

void ADC0_IRQHandler(void) {
    if (ADC0->SC1[0] & ADC_SC1_COCO_MASK) {
        ADCData = ADC0->R[0];

        // Toggle Red LED
        GPIOD->PTOR = (1U << 5);

        // Prepare and send ADC value
        char message[20];
        sprintf(message, "ADC: %d\r\n", ADCData);
        UART_TransmitString((uint8_t *)message, strlen(message));
    }
}

void PIT_IRQHandler(void) {
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        // Clear interrupt flag
        PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;

        // Toggle Green LED
        GPIOE->PTOR = (1U << 29);

        // Start ADC conversion
        ADC0->SC1[0] = ADC_SC1_ADCH(22); // Correct ADC channel
    }
}