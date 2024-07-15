
#include "MKL46Z4.h"
#include "stdint.h"
#include "stdbool.h"
#define HEX_INVALID 255
#define SYNTAX_ERROR 1
#define SUCCESS 0

void Erase_Flash_Sector(uint32_t Address) {
    // Wait for previous command to finish
    while ((FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0);

    // Check Error and Protection flags
    if ((FTFA->FSTAT & (FTFA_FSTAT_ACCERR_MASK | FTFA_FSTAT_FPVIOL_MASK)) != 0) {
        FTFA->FSTAT = FTFA_FSTAT_ACCERR_MASK | FTFA_FSTAT_FPVIOL_MASK; // Clear flags
    }

    // Command
    FTFA->FCCOB0 = 0x09;

    // Address
    FTFA->FCCOB3 = (uint8_t)(Address & 0xFF);
    FTFA->FCCOB2 = (uint8_t)((Address & 0xFF00) >> 8);
    FTFA->FCCOB1 = (uint8_t)((Address & 0xFF0000) >> 16);

    // Clear CCIF
    FTFA->FSTAT = FTFA_FSTAT_CCIF_MASK;
}

void Program_LongWord_Command(uint32_t Address, uint32_t Data) {
    // Wait for previous command to finish
    while ((FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0);

    // Check Error and Protection flags
    if ((FTFA->FSTAT & (FTFA_FSTAT_ACCERR_MASK | FTFA_FSTAT_FPVIOL_MASK)) != 0) {
        FTFA->FSTAT = FTFA_FSTAT_ACCERR_MASK | FTFA_FSTAT_FPVIOL_MASK; // Clear flags
    }

    // Command
    FTFA->FCCOB0 = 0x06;

    // Address
    FTFA->FCCOB3 = (uint8_t)(Address & 0xFF);
    FTFA->FCCOB2 = (uint8_t)((Address & 0xFF00) >> 8);
    FTFA->FCCOB1 = (uint8_t)((Address & 0xFF0000) >> 16);

    // Data
    uint32_t *ptr = (uint32_t *)0x40020008;
    *ptr = Data;

    // Clear CCIF
    FTFA->FSTAT = FTFA_FSTAT_CCIF_MASK;
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

uint8_t StrtoHex(const char *str, uint8_t len, uint32_t *pDecimal) {
    *pDecimal = 0;
    for (uint32_t idx = 0; idx < len; idx++) {
        uint8_t hex = char_to_hex(str[idx]);
        if (hex == HEX_INVALID) {
            return SYNTAX_ERROR;
        }
        *pDecimal = (*pDecimal << 4) | hex;
    }
    return SUCCESS;
}

void Program_Flash(uint32_t startAddress, const char *data, uint32_t length) {
    uint32_t address = startAddress;
    uint32_t dataWord;
    uint32_t fullWords = length / 8;
    uint32_t remainingBytes = length % 8;
    char buffer[9] = {0};

    for (uint32_t i = 0; i < fullWords * 8; i += 8) {
        uint8_t status = StrtoHex(&data[i], 8, &dataWord);
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

        uint8_t status = StrtoHex(buffer, 8, &dataWord);
        if (status == SUCCESS) {
            Program_LongWord_Command(address, dataWord);
        } else {
            // Handle error (e.g., invalid hex string)
        }
    }
}

int main() {
    Erase_Flash_Sector(0x2800);
    const char *data = "A00012348976";
    Program_Flash(0x2800, data, 12);

    return 0;
}