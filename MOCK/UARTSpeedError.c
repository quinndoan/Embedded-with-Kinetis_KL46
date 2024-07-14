#include "MKL46Z4.h"
#include "stdint.h"
#include "stdbool.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// bị đè dữ liệu khi UART chạy quá nhanh so với tốc độ truyền

#define BUFFER_SIZE 256
#define QUEUE_SIZE 30
#define HEX_INVALID 255

volatile uint8_t rxBuffer[BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile bool newLine = false;

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
    char *Address;
    char *Data;
    uint8_t checksum;
}Srec;

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

void UART_Initialize();
void UART_SendChar(char c);
void UART_TransmitString(uint8_t *ptr, uint8_t length);
void UART0_IRQHandler(void);
void Queue_Enqueue(Queue *q, char *data, uint16_t length);
bool Queue_Dequeue(Queue *q, QueueItem *item);
void SendProcessedData(struct Srec *srecLine);
void Process_SREC(Queue *q);

void Init_Button_SW3();

uint8_t char_to_hex(uint8_t c);
SrecStatus_t StrtoHex(const char *str, uint8_t len, uint32_t *pDecimal);
uint8_t calculate_checksum(const char *line, uint32_t sizeForLine);
SrecStatus_t ReadCheckLine(const char *line, struct Srec *line1);
struct Srec* parse_srec(const char *line);

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

    // Enable UART0 receive interrupt
    UART0->C2 |= UART0_C2_RIE_MASK;

    // Enable UART0 interrupt in NVIC
    NVIC_EnableIRQ(UART0_IRQn);
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

//void UART0_IRQHandler(void) {
//    if (UART0->S1 & UART0_S1_RDRF_MASK) {
//        uint8_t receivedChar = UART0->D;
//        rxBuffer[rxIndex++] = receivedChar;
//        if (rxIndex >= BUFFER_SIZE || receivedChar == '\n') { // phải sửa đoạn này phần End Of File nữa
//            newLine = true;
//        }
//    }
//}


//void UART0_IRQHandler(void) {
//    if (UART0->S1 & UART0_S1_RDRF_MASK) {
//        uint8_t receivedChar = UART0->D;
//        rxBuffer[rxIndex++] = receivedChar;
//        if (rxIndex >= BUFFER_SIZE || receivedChar == '\n' || receivedChar == '\r') {
//            newLine = true;
//            rxBuffer[rxIndex] = '\0'; // Null terminate for safety
//        }
//    }
//}

void UART0_IRQHandler(void) {
    if (UART0->S1 & UART0_S1_RDRF_MASK) {
        uint8_t receivedChar = UART0->D;
        if (receivedChar != '\r') { // Ignore carriage return
            rxBuffer[rxIndex++] = receivedChar;
            if (rxIndex >= BUFFER_SIZE || receivedChar == '\n') {
                newLine = true;
                rxBuffer[rxIndex - 1] = '\0'; // Replace '\n' with null terminator
            }
        }
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
        case 3:
        case 7:
            line1->sizeOfAddress = 8;
            break;
        default:
            return SYNTAX_ERROR;
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
    if ((srec->recordType != 1) & (srec->recordType != 2) & (srec->recordType != 3)){
        free(srec);
        return NULL;
    }

    return srec;
}

void SendProcessedData(Srec *srecLine) {
    char buffer[BUFFER_SIZE];
    int length;

    // Format the SREC data into a readable string
    length = snprintf(buffer, BUFFER_SIZE,
                      "Record Type: S%d\n"
                      "Byte Count: %lu\n"
                      "Address: %s\n"
                      "Data: %s\n"
                      "Checksum: %02X\n\n",
                      srecLine->recordType,
                      srecLine->ByteCount,
                      srecLine->Address,
                      srecLine->Data,
                      srecLine->checkSum);

    // Transmit the formatted string to the PC
    UART_TransmitString((uint8_t *)buffer, length);
}

void Process_SREC(Queue *q) {
    QueueItem item;
    while (Queue_Dequeue(q, &item)) {
        Srec *parsedSrec = parse_srec(item.data);
        if (parsedSrec != NULL) {
            SendProcessedData(parsedSrec);
            free(parsedSrec->Address);
            free(parsedSrec->Data);
            free(parsedSrec);
        }
    }
}

void test_srec_processing() {
    const char *test_srec_lines[] = {
        "S1130000285F245F2212226A000424290008237C2A",
        "S113A05059A2000059A2000059A2000059A2000010",
        "S113A0C080B500F00EF800F02DF81B4801688022DE",
        "S5030004F8",
        "S113A1D00548014603B4684680F3098800BF00BF00"
    };

    for (int i = 0; i < sizeof(test_srec_lines) / sizeof(test_srec_lines[0]); i++) {
        Queue_Enqueue(&srecQueue, (char *)test_srec_lines[i], strlen(test_srec_lines[i]));
    }

    Process_SREC(&srecQueue);
}


int main(){
	UART_Initialize();

	//test_srec_processing();

		 uint8_t Message[] = "Send the SREC file baby\r\n";
		    UART_TransmitString(Message, sizeof(Message) - 1);

		    while (1) {
		        if (newLine) {
		            newLine = false;
		            if (rxIndex > 0) { // Ensure there's something to process
						Queue_Enqueue(&srecQueue, (char *)rxBuffer, rxIndex);
						rxIndex = 0; // Reset index for the next line
					}

		        Process_SREC(&srecQueue);
		    }
		    }

		    return 0;

}