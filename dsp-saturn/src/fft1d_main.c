// See LICENSE for license details.

//**************************************************************************
// fft2 benchmark
//--------------------------------------------------------------------------
//

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "riscv.h"
#include "chip_config.h"
//--------------------------------------------------------------------------
// Input/Reference Data
#include "encoding.h"
#include "dataset1.h"

#define TWO_PI = 2 * M_PI

//--------------------------------------------------------------------------
// Main

extern volatile uint64_t tohost;
extern volatile uint64_t fromhost;

void app_init() {
  // torch::executor::runtime_init();
}

void app_main() {

  #if PREALLOCATE
    for (size_t i = 0; i < DATA_SIZE-1; i++) {
      volatile float tmp;
      tmp = input_Xr[i];
      tmp = input_Xi[i];
      tmp = input_Wr[i];
      tmp = input_Wi[i];
    }
  #endif

  // Do the FFT
  //setStats(1);
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");
  //printf("start cycle ")
  fft2(input_Xr, input_Xi, input_Wr, input_Wi, DATA_SIZE, LOG2_DATA_SIZE);
  //setStats(0);
  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  printf("mcycle = %lu\r\n", end_time - start_time);
  printf("minstret = %lu\r\n", end_instructions - start_instructions);

  #define VERIFY
  #ifdef VERIFY
  #define FFT_MAX_ERROR (10e-8f)
    // Check the results
    {
      size_t i;
      for (i = 0; i < DATA_SIZE; i++) {
        float rdiff, idiff, err;
        rdiff = input_Xr[i] - verify_Xr[i];
        idiff = input_Xi[i] - verify_Xi[i];

        err = (rdiff * rdiff) + (idiff * idiff);
        if (err > FFT_MAX_ERROR) {
          return (i + 1);
        }
      }
    }
  #endif
  return 0;

  // while(1) {
  //   printf("Hello world from hart : %d\r\n", mhartid);
  // }
}

int main( int argc, char* argv[] )
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Configure the system clock */
  /* Configure the system clock */
  
  /* USER CODE BEGIN SysInit */
  UART_InitType UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART_init_config);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */  
  /* USER CODE BEGIN Init */
  app_init();
  /* USER CODE END Init */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    app_main();
    return 0;
  }
  /* USER CODE END WHILE */

 }

/*
 * Main function for secondary harts
 * 
 * Multi-threaded programs should provide their own implementation.
 */
void __attribute__((weak, noreturn)) __main(void) {
  while (1) {
   asm volatile ("wfi");
  }
}

// Naive implementation using little vectorization to get something going
void fft1d(float* samples, size_t N, float* result_real, float* result_imag) {
  if (N == 1) {
    return samples;
  }

  // float even[N / 2];
  // float odd[N / 2];
  float* even = (float*)malloc((N / 2) * sizeof(float));
  float* odd = (float*)malloc((N / 2) * sizeof(float));


  int evenIndex = 0;
  int oddIndex = 0;
  for (int i = 0; i < N; i += 1) {
    if ((i % 2) == 0) {
      even[evenIndex] = samples[i];
      evenIndex += 1;
    } else {
      odd[oddIndex] = samples[i];
      oddIndex += 1;
    }
  }

  float *real_twiddle;
  float *imag_twiddle;
  fft1d(even, N / 2, result_real, result_imag);
  fft1d(odd, N / 2, result_real, result_imag);

  // float real_twiddles[N];
  // float imag_twiddles[N];
  for (int i = 0; i < N; i += 1) {
    computeTwiddle(N, i, real_twiddle, imag_twiddle);
    // real_twiddles[i] = *real_twiddle;
    // imag_twiddles[i] = *imag_twiddle;
    result_real[i] = even[i % (N / 2)] + (*real_twiddle) * odd[i % (N / 2)];
    result_imag[i] = even[i % (N / 2)] + (*imag_twiddle) * odd[i % (N / 2)];
  }

        // do {
        //   size_t vl;
        //   __asm__ __volatile__ (
        //     "vsetvli %0, %1, e32, m4, ta, ma" "\n\t"
        //     : "=r" (vl)
        //     : "r" (N));

        //   __asm__ __volatile__ (
        //     "vle32.v v8,  %0" "\n\t" // samples
        //     "vle32.v v12, %1" "\n\t" // result_real
        //     "vle32.v v16, %2" "\n\t" // result_imag
        //     // "vle32.v v20, %3" "\n\t" // bi

        //     "vfsub.vv  v24, v8,  v12" "\n\t"  // ar - br
        //     "vle32.v   v4,  %5"       "\n\t"  // Wi
        //     "vfsub.vv  v28, v16, v20" "\n\t"  // ai - bi
        //     "vle32.v   v0,  %4"       "\n\t"  // Wr
        //     "vfadd.vv  v16, v16, v20" "\n\t"  // ai' = ai + bi
        //     "vfmul.vv  v20, v24, v4"  "\n\t"  // Wi * (ar - br)
        //     "vfmul.vv  v4,  v28, v4"  "\n\t"  // Wi * (ai - bi)
        //     "vfadd.vv  v8,  v8,  v12" "\n\t"  // ar' = ar + br

        //     "vse32.v v16, %2" "\n\t" // ai'

        //     "vfmadd.vv v28, v0,  v20" "\n\t"  // bi' = Wr * (ai - bi) + Wi * (ar - br)
        //     "vfmsub.vv v24, v0,  v4"  "\n\t"  // br' = Wr * (ar - br) - Wi * (ai - bi)

        //     "vse32.v v8,  %0" "\n\t" // ar'
        //     "vse32.v v28, %3" "\n\t" // bi'
        //     "vse32.v v24, %1" "\n\t" // br'

        //     :
        //     : "A" (*samples), "A" (ack ack ack), "A" (*Xi0), "A" (*Xi1), "A" (*wr), "A" (*wi));

        //   n -= vl;
        //   wr += vl;
        //   wi += vl;
        //   Xr0 += vl;
        //   Xi0 += vl;
        //   Xr1 += vl;
        //   Xi1 += vl;

        // } while (n > 0);

        // do {
        //   // # Full 1D FFT Implementation in RISC-V Vector Assembly
        //   vsetvli t0, a0, e32   // Set vector length with element width of 32 bits

        //   mv t1, a1             // t1 points to real_input
        //   mv t2, a2             // t2 points to real_output
        //   mv t3, a3             // t3 points to imag_output

        //   // Compute log2(N) to determine the number of stages
        //   li t11, 0             // Initialize t11 to store log2(N)
        //   mv t12, a0            // Copy N to t12
        //   log2_calc:
        //       srli t12, t12, 1  // Right shift by 1 (divide by 2)
        //       addi t11, t11, 1  // Increment log2 counter
        //       bnez t12, log2_calc // Repeat until t12 becomes 0

        //   // Stage 1: Load real input into vectors
        //   vle32.v v0, (t1)      // Load input array into v0

        //   // # Initialize twiddle factor base addresses
        //   // la t4, twiddle_real    # Base address of real twiddle factors
        //   // la t5, twiddle_imag    # Base address of imaginary twiddle factors

        //   // # Recursive FFT computation loop
        //   // # Outer loop for log2(N) stages
        //   li t6, 0              // Stage counter (t6)
        //   li t7, 1              // Initial stride (t7 = 2^stage)
        //   fft_outer_loop:
        //       bge t6, t11, fft_done // Exit loop when stage >= log2(N)

        //       // Inner loop for butterfly computations in the current stage
        //       mv t8, t1          // Reset input pointer for this stage
        //       mv t9, t2          // Reset output pointer for this stage
        //       li t10, 0          // Butterfly index counter
        //   fft_inner_loop:
        //       // Load twiddle factors - bruh these are not computed
        //       vle32.v v1, (t4)   // Load real twiddle factors into v1
        //       vle32.v v2, (t5)   // Load imaginary twiddle factors into v2

        //       // Load even and odd elements for butterfly
        //       vle32.v v3, (t8)   // Load even-indexed elements into v3
        //       addi t8, t8, t7    // Advance pointer by stride for odd elements
        //       vle32.v v4, (t8)   // Load odd-indexed elements into v4

        //       // Compute twiddle factor multiplication for odd elements
        //       vmul.vv v5, v4, v1 // Real part: odd * twiddle_real
        //       vmul.vv v6, v4, v2 // Imaginary part: odd * twiddle_imag

        //       // Perform butterfly computation
        //       vadd.vv v7, v3, v5 // Real addition: even + real_twiddle_result
        //       vsub.vv v8, v3, v5 // Real subtraction: even - real_twiddle_result
        //       vadd.vv v9, v6, v2 // Imaginary addition: odd + imag_twiddle_result
        //       vsub.vv v10, v6, v2 // Imaginary subtraction: odd - imag_twiddle_result

        //       // Store results
        //       vse32.v v7, (t9)   // Store real addition result
        //       addi t9, t9, t7    // Advance pointer
        //       vse32.v v8, (t9)   // Store real subtraction result
        //       vse32.v v9, (t3)   // Store imaginary addition result
        //       addi t3, t3, t7    // Advance pointer
        //       vse32.v v10, (t3)  // Store imaginary subtraction result

        //       // Increment butterfly counter
        //       addi t10, t10, 1
        //       blt t10, t7, fft_inner_loop // Repeat inner loop until done

        //       // Increment stage counter
        //       addi t6, t6, 1
        //       slli t7, t7, 1      // Double the stride for the next stage
        //       j fft_outer_loop

        //   fft_done:
        //   // End of FFT computation

        // }

}

void computeTwiddle(int N, int k, double *real, double *imag) {
    // Calculate the angle for the twiddle factor
    double angle = -(TWO_PI) * k / N;
    
    // Compute the real and imaginary parts of the twiddle factor
    *real = cos(angle);  // Real part: cos(2 * pi * k / N)
    *imag = sin(angle);  // Imaginary part: -sin(2 * pi * k / N)
}
