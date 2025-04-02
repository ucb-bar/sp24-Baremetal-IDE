#include "../mmio.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "data/fft_data.h"
#include "data/conv_data.h"
#include "data/I2S_data.h"

#include "utils/DMA_utils.h"
#include "utils/I2S_utils.h"
#include "utils/fft_utils.h"
#include "utils/conv_utils.h"

// Addresses for writing data
#define DMA_ADDR1 0x87000000L // address for FFT
#define DMA_ADDR2 0x88000000L // address for Conv

#define DMA_ADDR3 0x87000000L // Address for I2S L
#define DMA_ADDR4 0x88000000L // Address for I2S R

int main(void) {
    puts("\n[STARTING TEST]\n\n");

    // Setup Conv stuff
    uint16_t in_dilation[1] = {1};
    uint16_t in_kernel[8] = {0x0000, 0x3C00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}; // {0, 1, 0, 0, 0, 0, 0, 0} in FP16

    puts("Setup params for Convolution");
    set_conv_params(conv_len, in_dilation[0], in_kernel);
    start_conv();

    // Setup FFT stuff
    puts("Setup params for FFT");
    reset_fft();

    // Setup FFT stuff
    puts("Setup I2S");
    // set_I2S_params(0, 0, 0, 3, 3, 1, 0);
    // set_I2S_clkdiv(0, 2);
    // set_I2S_watermark(0, 7, 7);

    
    // Start DMA
    enable_Crack();
    printf("Starting blocks\n");

    // for I2S
    // write_I2S_tx_DMA(0, 3, I2S_len/2, I2S_data0, 1, 10);
    // write_I2S_tx_DMA(0, 4, I2S_len/2, I2S_data1, 0, 10);

    // read_I2S_rx_DMA(0, 5, I2S_len/2, DMA_ADDR3, 1, 10);
    // read_I2S_rx_DMA(0, 6, I2S_len/2, DMA_ADDR4, 0, 10);
    // set_I2S_en(0, 1, 1);

    // for fft
    write_fft_dma(1, fft_len, fft_data);

    // for conv
    write_conv_dma(0, conv_len, conv_data);
    read_conv_dma(0, conv_len, DMA_ADDR2);

    
    
    // check if FFT is complete cause its blocking (bad)
    printf("[Blocks are cooking]\n");
    while(fft_busy() || fft_count_left()){
        printf("pain:%d, %d \n", fft_busy(), fft_count_left());
    }; // This is needed since fft is blocking and is not a very good block
    read_fft_dma(1, fft_len, DMA_ADDR1);



    // Check blocks output
    printf("\nTest Output (FFT): \n");
    uint32_t poll;
    // for(int i=0; i<fft_len; i++) { // We only print the first couple points
    for(int i=0; i<30; i++) {
        poll = reg_read32(DMA_ADDR1 + i*8);
        uint32_t real = poll & 0xFFFF;
        uint32_t imag = (poll >> 16);
        printf("[%d]real: (%hd), imag: (%hd)\n", i, real, imag);
    }

    printf("\nTest Output (Conv): \n");
    // for (int i = 0; i < conv_len; i++) {
    for (int i = 0; i < 30; i++) {
        uint64_t current_out = reg_read16(DMA_ADDR2 + 2*i);
        printf("[%d] 0x%"PRIx16" \n", i, current_out);
    }
    

    // printf("\nTest Output (I2S): \n");
    // printf("    Left Channel: \n");
    // for(int i=0; i<10; i++) {
    //     poll = reg_read32(DMA_ADDR3 + i*4);
    //     printf("[%d](%x)\n", i, poll);
    // }
    // printf("    Right Channel: \n");
    // for(int i=0; i<10; i++) {
    //     poll = reg_read32(DMA_ADDR4 + i*4);
    //     printf("[%d](%x)\n", i, poll);
    // }

    printf("\n[DONE TEST]\n\n");

}





