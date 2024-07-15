void Process_SREC(Queue *q) {
    QueueItem item;
    while (Queue_Dequeue(q, &item)) {
        Srec *parsedSrec = parse_srec(item.data);
        if (parsedSrec != NULL) {
            // Extract address and data from the SREC structure
            parsedSrec->numberAddress là địa chỉ
            
            dùng một vòng for để ghi 4 byte một vào địa chỉ
            for {
            uint32_t data;
            StrtoHex(parsedSrec->Data, parsedSrec->sizeOfData, &data);
            
            // Program the data into the flash
            Program_LongWord_Command(address, data);}

            free(parsedSrec->Address);
            free(parsedSrec->Data);
            free(parsedSrec);
        }
    }

    void Program_LongWord_Command(uint32_t Address, uint32_t Data);

    void Erase_Flash_Sector(uint32_t Address);

    ra main: xóa trc 2800;
    Erase_Flash_Sector(address);