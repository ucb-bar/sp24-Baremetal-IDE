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

#include "riscv.h"
#include "chip_config.h"
//--------------------------------------------------------------------------
// Input/Reference Data
#include "encoding.h"
#include "dataset1.h"

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


void fft2(float Xr[], float Xi[], const float Wr[], const float Wi[], size_t N, size_t M)
{
  {
    size_t N1, N2;
    float *end;

    end = Xr + N;

    for (N1 = N; N1 > 4;) {
      float *Xr0;
      float *Xi0;
      float *Xr1;
      float *Xi1;
      const float *wr;
      const float *wi;

      N2 = N1 / 2;

      Xr0 = Xr; // Lower half
      Xi0 = Xi;
      Xr1 = Xr + N2; // Upper half
      Xi1 = Xi + N2;

      // Iterate over butterfly groups
      do {
        size_t n;

        n = N2;
        wr = Wr;
        wi = Wi;
        // Iterate over butterflies in group
        do {
          size_t vl;
          __asm__ __volatile__ (
            "vsetvli %0, %1, e32, m4, ta, ma" "\n\t"
            : "=r" (vl)
            : "r" (n));

          __asm__ __volatile__ (
            "vle32.v v8,  %0" "\n\t" // ar
            "vle32.v v12, %1" "\n\t" // br
            "vle32.v v16, %2" "\n\t" // ai
            "vle32.v v20, %3" "\n\t" // bi

            "vfsub.vv  v24, v8,  v12" "\n\t"  // ar - br
            "vle32.v   v4,  %5"       "\n\t"  // Wi
            "vfsub.vv  v28, v16, v20" "\n\t"  // ai - bi
            "vle32.v   v0,  %4"       "\n\t"  // Wr
            "vfadd.vv  v16, v16, v20" "\n\t"  // ai' = ai + bi
            "vfmul.vv  v20, v24, v4"  "\n\t"  // Wi * (ar - br)
            "vfmul.vv  v4,  v28, v4"  "\n\t"  // Wi * (ai - bi)
            "vfadd.vv  v8,  v8,  v12" "\n\t"  // ar' = ar + br

            "vse32.v v16, %2" "\n\t" // ai'

            "vfmadd.vv v28, v0,  v20" "\n\t"  // bi' = Wr * (ai - bi) + Wi * (ar - br)
            "vfmsub.vv v24, v0,  v4"  "\n\t"  // br' = Wr * (ar - br) - Wi * (ai - bi)

            "vse32.v v8,  %0" "\n\t" // ar'
            "vse32.v v28, %3" "\n\t" // bi'
            "vse32.v v24, %1" "\n\t" // br'

            :
            : "A" (*Xr0), "A" (*Xr1), "A" (*Xi0), "A" (*Xi1), "A" (*wr), "A" (*wi));

          n -= vl;
          wr += vl;
          wi += vl;
          Xr0 += vl;
          Xi0 += vl;
          Xr1 += vl;
          Xi1 += vl;

        } while (n > 0);

        Xr0 = Xr1;
        Xi0 = Xi1;
        Xr1 += N2;
        Xi1 += N2;

      } while (Xr1 < end);

      Wr = wr;
      Wi = wi;
      N1 = N2;
    }
  }

  {
    float *xr;
    float *xi;
    size_t n;

    /* Stage M-2 */
    xr = Xr;
    xi = Xi;
    n = N / 4;
    do {
      size_t vl;
      __asm__ __volatile__ (
        "vsetvli %0, %1, e32, m2, ta, ma" "\n\t"
        : "=r" (vl)
        : "r" (n));

      __asm__ __volatile__ (
        "vlseg4e32.v v0, %0" "\n\t"
        "vlseg4e32.v v8, %1" "\n\t"

        "vfadd.vv v16, v0, v4" "\n\t"   // xr[0] + xr[2]
        "vfadd.vv v18, v2, v6" "\n\t"   // xr[1] + xr[3]
        "vfsub.vv v20, v0, v4" "\n\t"   // xr[0] - xr[2]
        "vfsub.vv v22, v10, v14" "\n\t" // xi[1] - xi[3]

        "vfsub.vv v30, v6, v2" "\n\t"   // xr[3] - xr[1]
        "vfadd.vv v24, v8, v12" "\n\t"  // xi[0] + xi[2]
        "vfadd.vv v26, v10, v14" "\n\t" // xi[1] + xi[3]
        "vfsub.vv v28, v8, v12" "\n\t"  // xi[0] - xi[2]

        "vsseg4e32.v v16, %0" "\n\t"
        "vsseg4e32.v v24, %1" "\n\t"
        :
        : "A" (*xr), "A" (*xi));

        n -= vl;
        xr += 4 * vl;
        xi += 4 * vl;
    } while (n > 0);

    /* Stage M-1 */
    xr = Xr;
    xi = Xi;
    n = N / 2;
    do {
      size_t vl;
      __asm__ __volatile__ (
        "vsetvli %0, %1, e32, m4, ta, ma" "\n\t"
        : "=r" (vl)
        : "r" (n));

      __asm__ __volatile__ (
        "vlseg2e32.v v0, %0" "\n\t"
        "vlseg2e32.v v8, %1" "\n\t"

        "vfadd.vv v16, v0, v4" "\n\t"  // xr[0] + xr[1]
        "vfsub.vv v20, v0, v4" "\n\t"  // xr[0] - xr[1]
        "vfadd.vv v24, v8, v12" "\n\t" // xi[0] + xi[1]
        "vfsub.vv v28, v8, v12" "\n\t" // xi[0] - xi[1]

        "vsseg2e32.v v16, %0" "\n\t"
        "vsseg2e32.v v24, %1" "\n\t"
        :
        : "A" (*xr), "A" (*xi));

      n -= vl;
      xr += 2 * vl;
      xi += 2 * vl;
    } while (n > 0);
  }

  /* Bit-reversal unscrambler */
  {
    size_t i, j, b;
    size_t N1, N2;
    N1 = N - 1;
    N2 = N >> 1;
    for (i = 0, j = 0; i < N1; i++) {
      if (i < j) {
        float z;
        z = Xr[j];
        Xr[j] = Xr[i];
        Xr[i] = z;

        z = Xi[j];
        Xi[j] = Xi[i];
        Xi[i] = z;
      }
      b = ~i & (i + 1);
      b = N2 / b;
      j ^= N1 & ~(b - 1);
    }
  }
}

// #define NUM_COUNTERS 2
// static uintptr_t counters[NUM_COUNTERS];
// static char* counter_names[NUM_COUNTERS];

// void setStats(int enable)
// {
//   int i = 0;
// #define READ_CTR(name) do { \
//     while (i >= NUM_COUNTERS) ; \
//     uintptr_t csr = read_csr(name); \
//     if (!enable) { csr -= counters[i]; counter_names[i] = #name; } \
//     counters[i++] = csr; \
//   } while (0)

//   READ_CTR(mcycle);
//   READ_CTR(minstret);

// #undef READ_CTR
// }