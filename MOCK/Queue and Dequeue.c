#include "MKL46Z4.h"
#include "stdint.h"
#include "stdbool.h"
#include <string.h>

// file for Queue and Dequeue Only

#define BUFFER_SIZE 256
#define QUEUE_SIZE 15
//#define FLASH_START_ADDRESS 0x00020000

volatile uint8_t rxBuffer[BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile bool newData = false;

typedef struct {
    uint8_t data[BUFFER_SIZE];
    uint16_t length;
} QueueItem;

typedef struct {
    QueueItem items[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} Queue;

Queue srecQueue = {{0}, 0, 0, 0};

void UART_Initialize();
void UART_SendChar(char c);
void UART_TransmitString(uint8_t *ptr, uint8_t length);
void UART0_IRQHandler(void);
void Queue_Enqueue(Queue *q, uint8_t *data, uint16_t length);
bool Queue_Dequeue(Queue *q, QueueItem *item);
void Flash_Write(uint32_t address, uint8_t *data, uint32_t length);
void Process_SREC(QueueItem *item);

int main(void) {
    UART_Initialize();

    uint8_t Message[] = "Ready to receive data...\r\n";
    UART_TransmitString(Message, sizeof(Message) - 1);

    // Main loop
    while (1) {
        if (newData) {
            newData = false;
            // Add received data to queue
            Queue_Enqueue(&srecQueue, (uint8_t *)rxBuffer, rxIndex);
            rxIndex = 0; // Reset buffer index
        }

        QueueItem item;
        if (Queue_Dequeue(&srecQueue, &item)) {
            // Send dequeued item back to PC
            UART_TransmitString(item.data, item.length);
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

void UART0_IRQHandler(void) {
    // Check if receive data register full flag is set
    if (UART0->S1 & UART0_S1_RDRF_MASK) {
        // Read received character and store in buffer
        rxBuffer[rxIndex++] = UART0->D;
        // Handle buffer overflow
        if (rxIndex >= BUFFER_SIZE || rxBuffer[rxIndex - 1] == '\n') {
            newData = true;
        }
    }
}

void Queue_Enqueue(Queue *q, uint8_t *data, uint16_t length) {
    if (q->count < QUEUE_SIZE) {
        q->items[q->head].length = length;
        for (uint16_t i = 0; i < length; i++) {
            q->items[q->head].data[i] = data[i];
        }
        q->head = (q->head + 1) % QUEUE_SIZE;
        q->count++;
    }
}

bool Queue_Dequeue(Queue *q, QueueItem *item) {
    if (q->count == 0) {
        return false;
    } else {
        *item = q->items[q->tail];
        q->tail = (q->tail + 1) % QUEUE_SIZE;
        q->count--;
        return true;
    }
}

void Flash_Write(uint32_t address, uint8_t *data, uint32_t length) {
    // Implement flash write logic here
    // Note: Make sure the address is valid and not overlapping with program code
}

void Process_SREC(QueueItem *item) {
    // Implement SREC parsing and processing logic here
    // Example: Check checksum, parse data, and write to Flash
  //  Flash_Write(FLASH_START_ADDRESS, item->data, item->length);
}
