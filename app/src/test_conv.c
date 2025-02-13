#include "test_conv.h"


// Addresses for Convolution
#define BASE_ADDR 0x08800000

#define INPUT_ADDR      0x08800000
#define OUTPUT_ADDR     0x08800020

#define READ_CHECK_ADDR   0x0880008D

// Addresses for writing data
#define OUT_WRITE_ADDR   0x90000000L

int test_conv_dma() {
    puts("Starting test");
    uint16_t in_kernel[8] = {0x0000, 0x3C00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}; // {0, 1, 0, 0, 0, 0, 0, 0} in FP16
    int8_t in_arr[8] = {1, 2, 3, 4, 5, 6, 7, 8};                                                            

    puts("Setting values of MMIO registers");
    set_conv_params(conv_len, 1, in_kernel);

    puts("Starting Convolution");
    start_conv();

    printf("read from address: %x\n", conv_data);

    printf("Start DMA Write (C)\r\n");
    write_conv_dma(0, conv_len, conv_data);
    printf("start read DMA\r\n");
    read_conv_dma(0, conv_len, OUT_WRITE_ADDR);
    
    printf("\nRead written to memory address \n");
    for (int i = 0; i < conv_len; i++) {
        uint64_t current_out = reg_read16(OUT_WRITE_ADDR + 2*i);         // the OUTPUT is a queue, each read gives one FP16
        printf("[%d] 0x%x\r\n", i, current_out);
    }



    printf("\n\n");
    printf("TEST DONE \r\n");
    return 0;
}

int test_conv_dma_P() {
    puts("Starting test");
    uint16_t in_kernel[8] = {0x0000, 0x3C00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}; // {0, 1, 0, 0, 0, 0, 0, 0} in FP16
    int8_t in_arr[8] = {1, 2, 3, 4, 5, 6, 7, 8};                                                            

    puts("Setting values of MMIO registers");
    set_conv_params(conv_len, 1, in_kernel);

    puts("Starting Convolution");
    start_conv();

    printf("read from address: %x\n", conv_data);

    printf("Start DMA Read (P)\r\n");
    read_conv_dma_p(4, conv_len, OUT_WRITE_ADDR);
    printf("Start DMA Write (C)\r\n");
    write_conv_dma(0, conv_len, conv_data);
    
    printf("\nRead written to memory address \n");
    for (int i = 0; i < conv_len; i++) {
        uint64_t current_out = reg_read16(OUT_WRITE_ADDR + 2*i);         // the OUTPUT is a queue, each read gives one FP16
        printf("[%d] 0x%x\r\n", i, current_out);
    }



    printf("\n\n");
    printf("TEST DONE \r\n");
    return 0;
}


int test_conv_basic() {
    puts("Starting test");
    uint16_t in_kernel[8] = {0x0000, 0x3C00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}; // {0, 1, 0, 0, 0, 0, 0, 0} in FP16
    int8_t in_arr[8] = {1, 2, 3, 4, 5, 6, 7, 8};                                                            

    puts("Setting values of MMIO registers");
    set_conv_params(conv_len, 1, in_kernel);

    puts("Starting Convolution");
    start_conv();

    puts("Setting values of data inputs");
    // reg_write64(INPUT_ADDR, *((uint64_t*) in_arr));             // 64 bits: 8 INT8s
    for (int i=0; i<conv_len/8; i++) {
        reg_write64(INPUT_ADDR, *((uint64_t*) conv_data) + i);             // 64 bits: 8 INT8s

    }
    



    puts("Waiting for convolution to complete");
    sleep(1);
    
    uint16_t test_out[8];
    printf("\n\nTest Output (FP16 binary): ");
    for (int i = 0; i < conv_len/4; i++) {
        uint64_t current_out = reg_read64(OUTPUT_ADDR);         // the OUTPUT is a queue, each read gives one FP16
        // each output is 4 stacked FP16 so do bit shifting to seperate the results
        for (int j = 0; j < 4; j++) {
            test_out[i*4 + j] = (current_out >> (j*16)) & 0xFFFF;
            printf("0x%x\n", test_out[i*4 + j]);
        }
    }
    printf("\n\n");


    printf("TEST DONE \r\n");
    return 0;

}