#include "../mmio.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "data/fft_data.h"
#include "data/conv_data.h"

#include "utils/fft_utils.h"
#include "utils/conv_utils.h"

#define DMA_ADDR1 0x87000000L

int main(void) {
    printf("\n[STARTING TEST]\n\n");

    reset_fft();
    enable_Crack();

    write_fft_dma(0, 256, (uint64_t*) fft_data);
    
    while(fft_busy() || fft_count_left()){
        printf("pain:%d, %d \n", fft_busy(), fft_count_left());
    }; // This is needed since fft is blocking and is not a very good block

    read_fft_real_dma(0, 256, DMA_ADDR1);
    printf("[DONE] Waiting Write\n");

    uint32_t poll, real, imag;
    // for(int i=0; i<512; i++) {
    //     poll = reg_read32(DMA_ADDR1 + i*8);
    //     real = poll & 0xFFFF; 
    //     imag = (poll >> 16);
    //     printf("[%d]real: (%hd), imag: (%hd)\n", i, real, imag);
    // }
    for(int i=0; i<256; i++) {
        poll = reg_read16(DMA_ADDR1 + i*4);
        printf("[%d]real: (%hd)\n", i, poll);
    }
    
    printf("[DONE] Test\n");

}
