/*
 * MOCK_Driver.c
 *
 *  Created on: Jul 11, 2024
 *      Author: Admin
 */
#include <MOCK_Driver.h>

static volatile Clear_t Clear_Flag_PORTA;
static volatile Clear_t Clear_Flag_PORTC;
static volatile Clear_t Clear_Flag_PORTD;
static uint32_t count = 0;


extern volatile uint8_t MODE;

extern volatile uint8_t rxBuffer[BUFFER_SIZE];
extern volatile uint8_t rxIndex;
extern volatile bool newLine;
extern volatile Queue srecQueue;


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

void UART0_IRQHandler(void) {
    if (UART0->S1 & UART0_S1_RDRF_MASK) {
        uint8_t receivedChar = UART0->D;
        rxBuffer[rxIndex++] = receivedChar;
        if (rxIndex >= BUFFER_SIZE || receivedChar == '\n') { // phải sửa đoạn này phần End Of File nữa
            newLine = true;
        }
    }
}

void Queue_Enqueue(Queue *q, char *data, uint16_t length) {
    if (q->count < QUEUE_SIZE) {
        q->items[q->head].length = length;
        strncpy(q->items[q->head].data, data, length);
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
    uint8_t result;

    if ('0' <= c && c <= '9') {
        result = c - '0';
    } else if ('A' <= c && c <= 'F') {
        result = c - 'A' + 10;
    } else {
        result = HEX_INVALID;
    }

    return result;
}

SrecStatus_t StrtoHex(const char *str, uint8_t len, uint32_t *pDecimal) {
    SrecStatus_t result = SREC_TRUE;

    if (pDecimal == NULL) {
        return BYTE_COUNT_ERROR;
    }

    *pDecimal = 0;
	uint32_t idx;
    for (idx = 0; idx < len; idx++) {
        if (char_to_hex(str[idx]) == HEX_INVALID) {
            result = SYNTAX_ERROR;
            break;
        }
        *pDecimal += (char_to_hex(str[idx])) * pow(16, len - 1 - idx);
    }

    return result;
}

uint8_t calculate_checksum(const char *line, uint32_t sizeForLine) {
    uint8_t checksum = 0;
    uint32_t i;
    for (i = 2; i < sizeForLine - 1; i += 2) {
        uint32_t convertTwoByte;
        StrtoHex(&line[i], 2, &convertTwoByte);

        checksum += convertTwoByte;
    }

    checksum = 0xFF - (checksum & 0xFF);

    return checksum;
}

SrecStatus_t ReadCheckLine(const char *line, Srec *line1) {
    if (line1 == NULL) {
        return BYTE_COUNT_ERROR;
    }

    uint8_t a1 = line[0];
    if (a1 != 'S') {
        return SYNTAX_ERROR;
    }

    uint8_t a2 = line[1];
    uint8_t a2Number = char_to_hex(a2);

    if (a2Number == HEX_INVALID) {
        return SYNTAX_ERROR;
    }

    if (0 <= a2Number && a2Number <= 9 && a2Number != 4) {
        if (a2Number == 1 || a2Number == 5 || a2Number == 9) {
            line1->sizeOfAddress = 4;
        } else if (a2Number == 2 || a2Number == 6 || a2Number == 8) {
            line1->sizeOfAddress = 6;
        } else {
            line1->sizeOfAddress = 8;
        }
    } else {
        return SYNTAX_ERROR;
    }

    char a3[3];
    memcpy(a3, &line[2], 2);
    a3[2] = '\0';
    uint32_t byteCount;
    SrecStatus_t byteCountStatus = StrtoHex(a3, 2, &byteCount);
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

    uint32_t j;
    for (j = 0; j < line1->sizeOfData; j++) {
        if (char_to_hex(line1->Data[j]) == HEX_INVALID) {
            free(line1->Address);
            free(line1->Data);
            return BYTE_COUNT_ERROR;
        }
    }

    char a6[3];
    strncpy(a6, &line[4 + lengthAddress + line1->sizeOfData], 2);
    a6[2] = '\0';
    uint32_t checkSum;
    SrecStatus_t checkSumStatus = StrtoHex(a6, 2, &checkSum);
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


void Process_SREC(Queue *q) {
    QueueItem item;
    if (Queue_Dequeue(q, &item)) {
        struct Srec *parsedSrec = parse_srec(item.data);
        if (parsedSrec != NULL) {
            SendProcessedData(parsedSrec);
            free(parsedSrec->Address);
            free(parsedSrec->Data);
            free(parsedSrec);
        }
    }
}

struct Srec* parse_srec(const char *line) {
    struct Srec *srec = (struct Srec *)malloc(sizeof(struct Srec));
    if (srec == NULL) {
        return NULL;
    }

    SrecStatus_t status = ReadCheckLine(line, srec);
    if (status != SREC_TRUE) {
	 switch (status) {
				case CHECK_SUM_ERROR:
					uint8_t Message1[] = "Checksum error\r\n";
					UART_TransmitString(Message1, sizeof(Message1) - 1);
					break;
				case SYNTAX_ERROR:
					uint8_t Message2[] = "Syntax error\r\n";
					UART_TransmitString(Message2, sizeof(Message2) - 1);
					break;
				case BYTE_COUNT_ERROR:
					uint8_t Message3[] = "ByteCountError\r\n";
					UART_TransmitString(Message3, sizeof(Message3) - 1);
					break;
				default:
					uint8_t Message4[] = "UnknownError\r\n";
					UART_TransmitString(Message4, sizeof(Message4) - 1);
					break;
			}
        free(srec);
        return NULL;
    }

    srec->Address = (char *)malloc(srec->sizeOfAddress + 1);
    if (srec->Address == NULL) {
        free(srec);
        return NULL;
    }
    strncpy(srec->Address, &line[4], srec->sizeOfAddress);
    srec->Address[srec->sizeOfAddress] = '\0';

    srec->sizeOfData = srec->ByteCount - (srec->sizeOfAddress / 2) - 1;
    srec->Data = (char *)malloc(srec->sizeOfData * 2 + 1);
    if (srec->Data == NULL) {
        free(srec->Address);
        free(srec);
        return NULL;
    }
    strncpy(srec->Data, &line[4 + srec->sizeOfAddress], srec->sizeOfData * 2);
    srec->Data[srec->sizeOfData * 2] = '\0';

    return srec;
}

void SendProcessedData(struct Srec *srecLine) {
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

void Program_LongWord_Command(uint32_t Address, uint32_t Data){
	//Wait previous command to finished
	while((FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0);

	//Check Error and Protection flags
	if(((FTFA->FSTAT & FTFA_FSTAT_ACCERR_MASK) & FTFA_FSTAT_FPVIOL_MASK) != 0){
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

void Erase_Flash_Sector(uint32_t Address){
	//Wait previous command to finished
	while((FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0);

	//Check Error and Protection flags
	if(((FTFA->FSTAT & FTFA_FSTAT_ACCERR_MASK) & FTFA_FSTAT_FPVIOL_MASK) != 0){
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
