#define BUFFER_SIZE 256
#define QUEUE_SIZE 30
#define HEX_INVALID 255

volatile uint8_t rxBuffer[BUFFER_SIZE];
volatile uint8_t procBuffer[BUFFER_SIZE]; // Processing buffer
volatile uint8_t rxIndex = 0;
volatile uint8_t procIndex = 0; // Processing buffer index
volatile bool newLine = false;
volatile bool procReady = false; // Indicates if the processing buffer is ready

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

void UART_Initialize() {
    // Initialization code here
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
        if (receivedChar != '\r') { // Ignore carriage return
            rxBuffer[rxIndex++] = receivedChar;
            if (rxIndex >= BUFFER_SIZE || receivedChar == '\n') {
                newLine = true;
                memcpy((char *)procBuffer, (char *)rxBuffer, rxIndex);
                procIndex = rxIndex;
                rxIndex = 0;
                procReady = true;
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

void Process_SREC(Queue *q) {
    QueueItem item;
    while (Queue_Dequeue(q, &item)) {
        Srec *parsedSrec = parse_srec(item.data);
        if (parsedSrec != NULL) {
            SendProcessedData(parsedSrec);
            free(parsedSrec->Address);
            free(parsedSrec->Data);
            free(parsedSrec);
        } else {
            uint8_t Message1[] = "SREC Error\r\n";
            UART_TransmitString(Message1, sizeof(Message1) - 1);
        }
    }
}

int main() {
    UART_Initialize();

    uint8_t Message[] = "Send the SREC file baby\r\n";
    UART_TransmitString(Message, sizeof(Message) - 1);

    while (1) {
        if (procReady) {
            procReady = false;
            if (procIndex > 0) { // Ensure there's something to process
                Queue_Enqueue(&srecQueue, (char *)procBuffer, procIndex);
                procIndex = 0; // Reset index for the next line
            }
        }

        Process_SREC(&srecQueue);
    }

    return 0;
}

while (1) {
        if (newLine) {
            newLine = false;
            if (rxIndex > 0) { // Ensure there's something to process
                memcpy((char *)procBuffer, (char *)rxBuffer, rxIndex);
                procIndex = rxIndex;
                rxIndex = 0; // Reset index for the next line
                procReady = true;
            }
        }

        if (procReady) {
            procReady = false;
            Queue_Enqueue(&srecQueue, (char *)procBuffer, procIndex);
            procIndex = 0; // Reset index for the next line
        }
}