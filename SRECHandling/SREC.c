#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include<math.h>
#include "sub.h"
#include <stdlib.h>

#define HEX_INVALID 255

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
    uint32_t length = sizeForLine;

    if (length < 4) {
        printf("Error: Line length is insufficient for checksum calculation\n");
        return 0;
    }
    uint32_t i;
    for (i = 2; i < length - 1; i += 2) {
        uint32_t convertTwoByte;
        StrtoHex(&line[i], 2, &convertTwoByte);

        // Add the value to checksum
        checksum += convertTwoByte;
      //  checksum += hexValue;
    }

    // Calculate the final checksum value
    checksum = 0xFF - (checksum & 0xFF);

    return checksum;
}


ReadResult *ReadCheckLine(const char *line) {
    ReadResult *line1 = (ReadResult *)malloc(sizeof(ReadResult));
    if (line1 == NULL) {
        printf("Memory allocation failed");
        exit(EXIT_FAILURE);
    }

    uint8_t a1 = line[0];
    if (a1 == 'S') {
        line1->type = SREC_TRUE;  // true
    } else {
        line1->type = SYNTAX_ERROR;
    }

    uint8_t a2 = line[1];
    uint8_t a2Number = char_to_hex(a2);
    if (a2Number == HEX_INVALID) {
        line1->type = SYNTAX_ERROR; // syntax error
    } else if (0 <= a2Number && a2Number <= 9 && a2Number != 4) {   // xoa phan hoac cua hexa
        line1->type = SREC_TRUE;  // true
        if (a2Number == 1 || a2Number == 5 || a2Number == 9) {
            line1->sizeOfAddress = 4;  // doc 4 ky tu cho Address
        } else if (a2Number == 2 || a2Number == 6 || a2Number == 8) {
            line1->sizeOfAddress = 6;  // doc 6 ky tu cho Address
        } else {
            line1->sizeOfAddress = 8;  // doc 8 ky tu cho Address
        }
    }

    // Handle ByteCount
    char a3[3];
    strncpy(a3, &line[2], 2);
    a3[2] = '\0';
    uint32_t byteCount;
    SrecStatus_t byteCountStatus = StrtoHex(a3, 2, &byteCount);
    if (byteCountStatus != SREC_TRUE) {
        line1->type = byteCountStatus;
    }
    line1->ByteCount = byteCount;

    // Handle Address
    uint8_t lengthAddress = line1->sizeOfAddress + 1;
    char a4[lengthAddress];
    strncpy(a4, &line[4], lengthAddress - 1);
    a4[lengthAddress - 1] = '\0';
    // Check if the address field has the right hex type
    uint32_t pAddress;
    line1->type = StrtoHex(a4, lengthAddress - 1, &pAddress);
    // Handle Data
    uint32_t numberDataByte = (line1->ByteCount) - 1 - (line1->sizeOfAddress / 2);
    line1->sizeOfData = numberDataByte * 2;             // number of characters for Data
    uint32_t sizeForA5 = line1->sizeOfData + 1;

    char a5[sizeForA5];
    strncpy(a5, &line[4 + lengthAddress - 1], sizeForA5 - 1);
    a5[sizeForA5 - 1] = '\0';
    // check the data length
    uint32_t i;
    for (i = 0; i < line1->sizeOfData; i++) {
        if (char_to_hex(a5[i]) == HEX_INVALID) {
            line1->type = BYTE_COUNT_ERROR;
        }
    }

    // Handle CheckSum
    // Check the end of line or end of file after reading two characters for checksum
    char a6[3];
    strncpy(a6, &line[4 + lengthAddress - 1 + sizeForA5 - 1], 2);
    a6[2] = '\0';
    uint32_t checkSum;
    SrecStatus_t checkSumStatus = StrtoHex(a6, 2, &checkSum);
    line1->type = checkSumStatus;
    line1->checkSum = checkSum;

    if (line1->type == SREC_TRUE) {
        // check the value of CheckSum
        uint32_t sizeForLine = line1->ByteCount * 2 + 2;
        char lineForCaculate[sizeForLine + 1];
        strncpy(lineForCaculate, line, sizeForLine);
        lineForCaculate[sizeForLine] = '\0';
        uint8_t checkSumCaculate = calculate_checksum(lineForCaculate, sizeForLine);
    
       if (checkSumCaculate != line1->checkSum) {
            line1->type = CHECK_SUM_ERROR;
        }
    }

    return line1;
}

void printResult(const ReadResult *line) {
    if (line->type == SREC_TRUE){
    	printf("TRUE\n");
	}
	else {
		printf("FALSE\n");
		switch(line->type){
			case(1):
				printf("CHECK_SUM_ERROR\n");
				break;
			case(2):
				printf("SYNTAX_ERROR\n");
				break;
			case(3):
				printf("BYTE_COUNT_ERROR\n");
				break;
	}
	}
}