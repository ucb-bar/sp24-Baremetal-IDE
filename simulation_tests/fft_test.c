#include "../mmio.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "data/fft_data.h"
#include "data/fft_expected_data.h" // Include the expected FFT data file

#include "utils/fft_utils.h"
#include "utils/conv_utils.h"

#define NUM_POINTS 256
// #define NUM_TESTS 1
#define MAX_DIFF 3

#define DMA_ADDR1 0x87000000L

int main(void) {
    printf("\n[STARTING TEST]\n\n");
    printf("\n[NUMBER OF TESTS] %d\n\n", NUM_TESTS);
    int error_cnt = 0;

    for (int i = 0; i < NUM_TESTS; i++) {
      reset_fft();
      enable_Crack();

      write_fft_dma(0, NUM_POINTS, (uint32_t*)fft_data[i]);

      while(fft_busy() || fft_count_left()){
        printf("pain:%d, %d \n", fft_busy(), fft_count_left());
      }; // This is needed since fft is blocking and is not a very good block

      uint32_t poll;

      for(int j=0; j<NUM_POINTS; j++) {
        poll = read_fft();
        int16_t poll_real = (int16_t) poll;
        int16_t expected_real = (int16_t) fft_expected_data[i][j];
        if (poll_real - expected_real < -MAX_DIFF || poll_real - expected_real > MAX_DIFF) {
          printf("[FAIL, test=%d, idx=%d] Expected %lx, received %lx]\n", i, j, fft_expected_data[i][j], poll);
          error_cnt++;
        }
      //printf("Actual: %d, Expected: %d \n", poll, fft_expected_data[i][j]);

        if (poll_real != 0 && poll_real > MAX_DIFF) { // Show the peak, account for noise
          printf("[Test: %d], Peak at Index: %d, Actual: %d, Expected: %d \n", i, j, poll_real, fft_expected_data[i][j]);
        }

      }
      printf("[DONE] Test %d\n", i);
  }

  if (error_cnt == 0) {
      printf("[SUCCESS] All FFT outputs match the expected data.\n");
  } else {
      printf("[FAILURE] There were %d errors in FFT outputs.\n", error_cnt);
  }

}
