#include "MKL46Z4.h"
#include "stdint.h"

// dùng UART Interrupt để đánh dấu mỗi lần truyền tin, và gửi nó vào buffer
// Đã check truyền gửi đúng thông qua Send() và Receive()
#define BUFFER_SIZE 128

volatile uint8_t rxBuffer[BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile uint8_t txIndex = 0;

void UART_Initialize();
void UART_SendChar(char c);
void UART_TransmitString(uint8_t *ptr, uint8_t length);
char UART_ReceiveChar(void);
void UART0_IRQHandler(void);

int main(void) {
    UART_Initialize();

    uint8_t Message[] = "Today maybe a good day\r\n";
    UART_TransmitString(Message, sizeof(Message) - 1);

    // Main loop
    while (1) {
        // Check if data is received
        if (rxIndex != txIndex) {
            // Send back received data
            UART_SendChar(rxBuffer[txIndex]);
            txIndex = (txIndex + 1) % BUFFER_SIZE;
        }
    }

    return 0;
}

void UART_Initialize() {
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
    UART0->BDH = (sbr >> 8) & UART0_BDH_SBR_MASK; // dich 8 bit de chi lay ra phan cao nhat
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

    // Enable UART0 receive interrupt
    UART0->C2 |= UART0_C2_RIE_MASK;

    // Enable UART0 interrupt in NVIC
    NVIC_EnableIRQ(UART0_IRQn);
}

void UART_SendChar(char c) {
    // Wait for transmit data register empty flag
    while (!(UART0->S1 & UART0_S1_TDRE_MASK));
    // Send character
    UART0->D = c;
    // Wait for transmission complete flag
    while (!(UART0->S1 & UART0_S1_TC_MASK));
}

void UART_TransmitString(uint8_t *ptr, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        UART_SendChar(ptr[i]);
    }
}

char UART_ReceiveChar(void) {
    // Wait for receive data register full flag
    while (!(UART0->S1 & UART0_S1_RDRF_MASK));
    // Read received character
    return UART0->D;
}

void UART0_IRQHandler(void) {
    // Check if receive data register full flag is set
    if (UART0->S1 & UART0_S1_RDRF_MASK) {
        // Read received character and store in buffer
        rxBuffer[rxIndex] = UART0->D;
        rxIndex = (rxIndex + 1) % BUFFER_SIZE;
        // Handle buffer overflow (optional)
        if (rxIndex == txIndex) {
            // Buffer overflow, handle as needed
        }
    }
}

