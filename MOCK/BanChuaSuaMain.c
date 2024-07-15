#include "MKL46Z4.h"
#include "stdint.h"
#include "stdbool.h"
#include <string.h>

#define BUFFER_SIZE 256
#define QUEUE_SIZE 15
#define HEX_INVALID 255
#define SYNTAX_ERROR_FLASH 1
#define SUCCESS 0

volatile uint8_t rxBuffer[BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile bool newData = false;

typedef enum {
    SREC_TRUE = 0x0,
    CHECK_SUM_ERROR = 0x1,
    SYNTAX_ERROR = 0x2,
    BYTE_COUNT_ERROR = 0x3
} SrecStatus_t;

typedef struct Srec {
    uint8_t recordType;
    uint8_t sizeOfAddress;
    uint32_t ByteCount;
    uint32_t sizeOfData;
    uint32_t checkSum;
    uint32_t numberAddress; // dùng cho TH address 4 bytes, chuyển sẵn Address sang từ String sang Hex
    char *Address;
    char *Data;
    uint8_t checksum;
} Srec;

typedef struct {
    char data[BUFFER_SIZE];
    uint16_t length;
} QueueItem;

typedef struct {
    QueueItem items[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} Queue;

Queue srecQueue = {{0}, 0, 0, 0};

uint8_t StrtoHexFlash(const char *str, uint8_t len, uint32_t *pDecimal) {
    *pDecimal = 0;
    for (uint32_t idx = 0; idx < len; idx++) {
        uint8_t hex = char_to_hex(str[idx]);
        if (hex == HEX_INVALID) {
            return SYNTAX_ERROR_FLASH;
        }
        *pDecimal = (*pDecimal << 4) | hex;
    }
    return SUCCESS;
}

void Program_LongWord_Command(uint32_t Address, uint32_t Data) {
    //Wait previous command to finished
    while ((FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0);

    //Check Error and Protection flags
    if (((FTFA->FSTAT & FTFA_FSTAT_ACCERR_MASK) & FTFA_FSTAT_FPVIOL_MASK) != 0) {
        FTFA->FSTAT = 0x30;
    }

    //Command
    FTFA->FCCOB0 = 0x06;

    //Address
    FTFA->FCCOB3 = (uint8_t)(Address & 0xFF);
    FTFA->FCCOB2 = (uint8_t)((Address & 0xFF00) >> 8);
    FTFA->FCCOB1 = (uint8_t)((Address & 0xFF0000) >> 16);

    //Data
    uint32_t *ptr = (uint32_t *)0x40020008;
    *ptr = Data;

    //Clear CCIF
    FTFA->FSTAT = 0x80;
}

void Erase_Flash_Sector(uint32_t Address) {
    //Wait previous command to finished
    while ((FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0);

    //Check Error and Protection flags
    if (((FTFA->FSTAT & FTFA_FSTAT_ACCERR_MASK) & FTFA_FSTAT_FPVIOL_MASK) != 0) {
        FTFA->FSTAT = 0x30;
    }

    //Command
    FTFA->FCCOB0 = 0x09;

    //Address
    FTFA->FCCOB3 = (uint8_t)(Address & 0xFF);
    FTFA->FCCOB2 = (uint8_t)((Address & 0xFF00) >> 8);
    FTFA->FCCOB1 = (uint8_t)((Address & 0xFF0000) >> 16);

    //Clear CCIF
    FTFA->FSTAT = 0x80;
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

void Queue_Enqueue(Queue *q, char *data, uint16_t length) {
    if (q->count < QUEUE_SIZE) {
        q->items[q->head].length = length;
        memcpy(q->items[q->head].data, data, length);
        q->items[q->head].data[length] = '\0'; // Null terminate for safety
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

uint8_t char_to_hex(uint8_t c) {
    if ('0' <= c && c <= '9') {
        return c - '0';
    } else if ('A' <= c && c <= 'F') {
        return c - 'A' + 10;
    } else {
        return HEX_INVALID;
    }
}

SrecStatus_t StrtoHex(const char *str, uint8_t len, uint32_t *pDecimal) {
    if (pDecimal == NULL) {
        return BYTE_COUNT_ERROR;
    }

    *pDecimal = 0;
    for (uint32_t idx = 0; idx < len; idx++) {
        uint8_t hex = char_to_hex(str[idx]);
        if (hex == HEX_INVALID) {
            return SYNTAX_ERROR;
        }
        *pDecimal = (*pDecimal << 4) | hex;
    }

    return SREC_TRUE;
}

uint8_t calculate_checksum(const char *line, uint32_t sizeForLine) {
    uint8_t checksum = 0;
    for (uint32_t i = 2; i < sizeForLine - 1; i += 2) {
        uint32_t convertTwoByte;
        StrtoHex(&line[i], 2, &convertTwoByte);
        checksum += convertTwoByte;
    }

    return ~checksum & 0xFF;
}

SrecStatus_t ReadCheckLine(const char *line, Srec *line1) {
    if (line1 == NULL) {
        return BYTE_COUNT_ERROR;
    }

    if (line[0] != 'S') {
        return SYNTAX_ERROR;
    }

    uint8_t a2 = line[1];
    uint8_t a2Number = char_to_hex(a2);
    if (a2Number == HEX_INVALID) {
        return SYNTAX_ERROR;
    }

    switch (a2Number) {
        case 1:
        case 5:
        case 9:
            line1->sizeOfAddress = 4;
            break;
        case 2:
        case 6:
        case 8:
            line1->sizeOfAddress = 6;
            break;
        default:
            line1->sizeOfAddress = 8;
    }
    line1->recordType = a2Number;

    uint32_t byteCount;
    SrecStatus_t byteCountStatus = StrtoHex(line + 2, 2, &byteCount);
    if (byteCountStatus != SREC_TRUE) {
        return byteCountStatus;
    }
    line1->ByteCount = byteCount;

    uint8_t lengthAddress = line1->sizeOfAddress;
    line1->Address = (char *)malloc(lengthAddress + 1);
    if (line1->Address == NULL) {
        return BYTE_COUNT_ERROR;
    }
    memcpy(line1->Address, &line[4], lengthAddress);
    line1->Address[lengthAddress] = '\0';

    uint32_t pAddress;
    SrecStatus_t addressStatus = StrtoHex(line1->Address, lengthAddress, &pAddress);
    if (addressStatus != SREC_TRUE) {
        free(line1->Address);
        return addressStatus;
    }
    line1->numberAddress = pAddress;    

    uint32_t numberDataByte = (line1->ByteCount) - 1 - (line1->sizeOfAddress / 2);
    line1->sizeOfData = numberDataByte * 2;
    line1->Data = (char *)malloc(line1->sizeOfData + 1);
    if (line1->Data == NULL) {
        free(line1->Address);
        return BYTE_COUNT_ERROR;
    }
    memcpy(line1->Data, &line[4 + lengthAddress], line1->sizeOfData);
    line1->Data[line1->sizeOfData] = '\0';    

    for (uint32_t j = 0; j < line1->sizeOfData; j++) {
        if (char_to_hex(line1->Data[j]) == HEX_INVALID) {
            free(line1->Address);
            free(line1->Data);
            return BYTE_COUNT_ERROR;
        }
    }

    uint32_t checkSum;
    SrecStatus_t checkSumStatus = StrtoHex(line + 4 + lengthAddress + line1->sizeOfData, 2, &checkSum);
    if (checkSumStatus != SREC_TRUE) {
        free(line1->Address);
        free(line1->Data);
        return checkSumStatus;
    }
    line1->checkSum = checkSum;

    uint32_t sizeForLine = line1->ByteCount * 2 + 2;
    char lineForCaculate[sizeForLine + 1];
    strncpy(lineForCaculate, line, sizeForLine);
    lineForCaculate[sizeForLine] = '\0';
    uint8_t checkSumCaculate = calculate_checksum(lineForCaculate, sizeForLine);

    if (checkSumCaculate != line1->checkSum) {
        free(line1->Address);
        free(line1->Data);
        return CHECK_SUM_ERROR;
    }

    return SREC_TRUE;
}

Srec* parse_srec(const char *line) {
    Srec *srec = (Srec *)malloc(sizeof(Srec));
    if (srec == NULL) {
        return NULL;
    }

    SrecStatus_t status = ReadCheckLine(line, srec);
    if (status != SREC_TRUE) {
        const char *errorMessage;
        switch (status) {
            case CHECK_SUM_ERROR:
                errorMessage = "Checksum error\r\n";
                break;
            case SYNTAX_ERROR:
                errorMessage = "Syntax error\r\n";
                break;
            case BYTE_COUNT_ERROR:
                errorMessage = "Byte count error\r\n";
                break;
            default:
                errorMessage = "Unknown error\r\n";
                break;
        }
        UART_TransmitString((uint8_t *)errorMessage, strlen(errorMessage));
        free(srec);
        return NULL;
    }
    if ((srec->recordType != 1) & (srec->recordType != 2) & (srec->recordType != 3)) {
        free(srec);
        return NULL;
    }

    return srec;
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

void Process_SREC(char *line) {
    Srec *srec = parse_srec(line);
    if (srec != NULL) {
        
            Program_Flash(srec->numberAddress, srec->Data, srec->sizeOfData);
        }
        free(srec->Address);
        free(srec->Data);
        free(srec);
}


void Program_Flash(uint32_t startAddress, const char *data, uint32_t length) {
    uint32_t address = startAddress;
    uint32_t dataWord;
    uint32_t fullWords = length / 8;
    uint32_t remainingBytes = length % 8;
    char buffer[9] = {0};

    for (uint32_t i = 0; i < fullWords * 8; i += 8) {
        uint8_t status = StrtoHexFlash(&data[i], 8, &dataWord);
        if (status == SUCCESS) {
            Program_LongWord_Command(address, dataWord);
            address += 4;
        } else {
            // Handle error (e.g., invalid hex string)
        }
    }

    // Handle remaining bytes
    if (remainingBytes > 0) {
        // Copy remaining bytes to buffer and pad with '0'
        for (uint32_t i = 0; i < remainingBytes; i++) {
            buffer[i] = data[fullWords * 8 + i];
        }
        for (uint32_t i = remainingBytes; i < 8; i++) {
            buffer[i] = '0';
        }
        buffer[8] = '\0';

        uint8_t status = StrtoHexFlash(buffer, 8, &dataWord);
        if (status == SUCCESS) {
            Program_LongWord_Command(address, dataWord);
        } else {
            // Handle error (e.g., invalid hex string)
        }
    }
}


int main(void) {
    UART_Initialize();

    uint8_t Message[] = "Ready to receive data...\r\n";
    UART_TransmitString(Message, sizeof(Message) - 1);
    Erase_Flash_Sector(0x2800);

    // Simulate Queue with test data
    const char *testData = "S113280000600020D528000043290000F92D0000B5\nS113281000000000000000000000000000000000B4\n";

    // Tách các dòng dữ liệu và đẩy từng dòng vào hàng đợi
    const char *line = testData;
    while (*line) {
        const char *nextLine = strchr(line, '\n');
        if (nextLine) {
            Queue_Enqueue(&srecQueue, line, nextLine - line);
            line = nextLine + 1;
        } else {
            Queue_Enqueue(&srecQueue, line, strlen(line));
            break;
        }
    }

    // Main loop
    while (1) {
        QueueItem item;
        if (Queue_Dequeue(&srecQueue, &item)) {
            // Process each line in dequeued item
           // Process_SREC(item.data);
        	UART_TransmitString(item.data, strlen(item.data));
        } else {
            break; // Exit the loop when the queue is empty
        }
    }

    return 0;
}
