int main() {
    FILE *fileptr = fopen("SREC.txt", "r");
    if (fileptr == NULL) {
        printf("Can not open file");
        exit(EXIT_FAILURE);
    }
    uint32_t countLine=1;

    char line[256];
    while (fgets(line, sizeof(line), fileptr)) {
        ReadResult *result = ReadCheckLine(line);
        printf("Line %d:\n",countLine);
        printResult(result);
        countLine++;
        free(result);
    }

    fclose(fileptr);
    return 0;
}

