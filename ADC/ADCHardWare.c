#include <stdint.h>
#include <MKL46Z4.h>

// bản chính xác, đã check debug, giúp thay đổi ADCR[0] và COCO, AIEN

uint16_t ADCData;
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

    // Trigger Start Conversion
    ADC0->SC1[0] = (ADC0->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(3U);
}


void ADC0_IRQHandler(void)
{
    // Read Data Register
    ADCData = ADC0->R[0];

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
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;

}

void main()
{
    ADC_Init();
    PIT_Init();

    // Interconnect PIT_ADC
    SIM->SOPT7 &= ~SIM_SOPT7_ADC0TRGSEL_MASK;
    SIM->SOPT7 |= SIM_SOPT7_ADC0TRGSEL(4U); // Trigger source is PIT
    SIM->SOPT7 &= ~SIM_SOPT7_ADC0ALTTRGEN_MASK;
    SIM->SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN(1U);

    // Select ADC Channel
    ADC_ReadInt();

    // Start Timer
    PIT_StartTimer();

    while(1) {
        // Main loop
    }
}