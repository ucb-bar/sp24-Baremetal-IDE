/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main FFT functionality testing body with some stress tests
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "main.h"
#include "chip_config.h"
#include "hal_DMA.h"
#include "hal_fft.h"
#include "kiss_fft.h"

// Struct for benchmarking
typedef struct {
  uint64_t cycles;
  bool correct;
} fft_result_t;

// #define LOGPATH "./fft_log.txt" // TODO abandoned efforts to output into log files
#define DMA_ADDR1 0x87000000L // DMA base address
#define INPUT_ADDR1 0x08000000U // Where to save data - scratchpad is 0x08000000U
#define NUM_POINTS 128 // Should always be 128 for DSP24, == FFT length
#define DMA_NUM 0 // Tested with 0 and 1
#define MAX_DIFF 5 // Might work down to 2-3
#define RM_IMAG 1 // Remove imaginary values for easier output parsing

#include "../goldenmodel/fft_data_128len_131c.h"
#include "../goldenmodel/fft_data_128len_twinkle.h"
#include "../goldenmodel/fft_expected_data_128len_131c.h"
#include "../goldenmodel/fft_expected_data_128len_twinkle.h"
#include "tone_samples.h" // TODO still debugging..

/*
 * PICK YOUR INPUT OPTION BELOW (AND COMPARISON IF APPLICABLE)
 */

/* INPUT OPTIONS */
#define INPUT_DATA fft_data_twinkle
// #define INPUT_DATA fft_data_131c
// #define INPUT_DATA B3_samples_hex_128 // TODO Still needs work - different format.. 

/* COMPARISON OUTPUT OPTIONS */
#define OUTPUT_DATA fft_expected_data_twinkle
// #define OUTPUT_DATA fft_expected_data_131c

#ifdef OUTPUT_DATA
bool compare_output = true;
#else 
bool compare_output = false;
#define OUTPUT_DATA fft_expected_data_131c // throwaday data 
#endif 

#ifndef NUM_TESTS
#define NUM_TESTS 14 // 1 // will also be overwritten if in data file
#endif

/*
 * END OF USER OPTIONS (SORRY FOR USING DEFINES FOR THIS, CRINGE I KNOW)
 */


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN PUC */

void app_init() {
  // torch::executor::runtime_init();
}

/*
 * THE MAIN FFT DMA TEST
 * It works.
*/

int run_fft_dma_bmark(int i, int iterations = 100, bool busycycles = false) {
  int error_cnt = 0;
  fft_result_t result;
  memset(&result, 0, sizeof(result));

  reset_fft();
  reset_DMA(); // reset is at DMA base address 
  // enable_Crack(); // bad idea to enable for initial tests 

  /* WRITE INPUT DATA */
  for(int x=0; x < iterations; x++) {
    start_roi(); // start collecting shmoo data
    uint64_t start_time = READ_CSR("mcycle");

    write_fft_dma(DMA_NUM, NUM_POINTS, (uint32_t*) INPUT_DATA[i]); 

    if (busycycles) { // might want to wait until fft_busy() explicitly
      uint64_t start_time = READ_CSR("mcycle");
    }
    while(fft_busy() || fft_count_left()){
      // continue; // not sure why this was added
      printf("[Blocking] pain:%d, %d \r\n", fft_busy(), fft_count_left());
    }; // This is needed since fft is blocking and is not a very good block
    if (busycycles) {
      uint64_t end_time = READ_CSR("mcycle");
    }

    /* READ & COMPARE OUTPUT DATA */

    // while (*(volatile char*) (DMA_BASE+0x1) != 0);
    read_fft_real_dma(DMA_NUM, NUM_POINTS, DMA_ADDR1);
    // while (*(volatile char*) (DMA_BASE+0x1) != 0);

    if (!busycycles) {
      uint64_t end_time = READ_CSR("mcycle");
    }
    end_roi(); // stop collecting shmoo data

    uint32_t poll_dmafft, real_dmafft, imag_dmafft;
    uint32_t poll_real_max_dmafft = 0;
    uint32_t idx_max_dmafft = 0;
    for(int j=0; j < NUM_POINTS; j++) {
      poll_dmafft = reg_read32(DMA_ADDR1 + j*4); // DO NOT DO j*8, even if other people do!
      int16_t poll_real_dmafft = poll_dmafft & 0xFFFF; // same effect as "(int16_t) poll"
      int16_t poll_imag_dmafft = poll_dmafft >> 16; // untested - not needed for audio type inputs
      
      if (compare_output) {
        int16_t expected_real_dmafft = (int16_t) OUTPUT_DATA[i][j];
        if (poll_real_dmafft - expected_real_dmafft < -MAX_DIFF || poll_real_dmafft - expected_real_dmafft > MAX_DIFF) {
          error_cnt++;
        }
      } 
    }
  }
  
  /* RESULTS & CLEANUP */

  result.cycles = end_time - start_time;
  result.correct = error_cnt == 0;
  xmit_payload_packet(&result, 9); // pointer to payload and size of payload
  return error_cnt;
}

/*
 * THE MAIN KISS FFT ON CPU DMA TEST
 * It does not match Numpy expectations.
*/

int run_cpu_fft_bmark(int i, int iterations = 100, bool busycycles = false) {
  /* SETUP */
  int error_cnt = 0;
  fft_result_t result;
  memset(&result, 0, sizeof(result));
  
  // Allocates memory for FFT + parameters but not buffers
  // Return value is a contiguous block of memory, can be free()d
  kiss_fft_cfg cfg = kiss_fft_alloc(NUM_POINTS , 0, 0, 0);
  // Allocate memory for the input data buffer
  kiss_fft_cpx* fftbuf = (kiss_fft_cpx*) malloc(NUM_POINTS * sizeof(kiss_fft_cpx));
  // Allocate memory for the output data buffer
  kiss_fft_cpx* fftoutbuf = (kiss_fft_cpx*) malloc(NUM_POINTS * sizeof(kiss_fft_cpx));
  // Load data into input buffer
  for(int j = 0; j < NUM_POINTS; j += 1) {
      // kiss_fft_cpx is struct with kiss_fft_scalar real, imaginary of chosen type (see FIXED_POINT)
      fftbuf[j].r = (int16_t) INPUT_DATA[i][j]; 
      fftbuf[j].i = 0; // (int16_t) (INPUT_DATA[0][i] >> 16); 
      // printf("[DATA DEBUG..] %x\r\n",  INPUT_DATA[0][i]); // This should match the input vector line by line
  }

  /* DO THE FFT TRANFORMATION */
  start_roi(); 
  uint64_t start_time = READ_CSR("mcycle");
  for(int x=0; x < iterations; x++) {
    // actually kiss_fft_stride -> kf_work -> openmp -> magic, trust me bro
    kiss_fft(cfg, fftbuf, fftoutbuf);
  }
  uint64_t end_time = READ_CSR("mcycle");
  end_roi(); 

  /* RESULTS */

  int index = 0;
  int max = 0; // Note the type: if Hz stuck at 0, max and buffer might be mismatched types
  for (int j = 0; j < NUM_POINTS; j++) { 
    if (abs(fftoutbuf[j].r) > max) {
      max = abs(fftoutbuf[j].r);
      index = j;
    }
    if (compare_output) {
      int16_t expected_real_fft = (int16_t) OUTPUT_DATA[i][j];

      if (fftoutbuf[j].r - expected_real_fft < -MAX_DIFF || fftoutbuf[j].r - expected_real_fft > MAX_DIFF) {
        error_cnt++;
      }
    }
  }
  
  /* RESULTS & CLEANUP */
  
  free(cfg);
  free(fftbuf);
  free(fftoutbuf);
  kiss_fft_cleanup();
  result.correct = error_cnt < 5;
  result.cycles = end_time - start_time;
  xmit_payload_packet(&result, 9); // pointer to payload and size of payload
  return error_cnt;
}

/*
 * BENCHMARKING WRAPPERS
*/

void run_fft_dma_bmark_sequence(int iterations = 100, bool busycycles = false) {
  for (int i = 0; i < NUM_TESTS; i++) {
    run_fft_dma_bmark(i, iterations, busycycles);
  }
}

void run_cpu_fft_bmark_sequence(int iterations = 100, bool busycycles = false) {
  for (int i = 0; i < NUM_TESTS; i++) {
    run_cpu_fft_bmark(i, iterations, busycycles);
  }
}

/*
 * RUNNING FROM APP MAIN IS RECOMMENDED
 * Comment out what you're not running.
*/

/* USER CODE END PUC */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
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
    test_info t = init_test(UART1);
    int seed = *((int*) &t.payload);
    switch (t.testid) {
      case 0:
        run_fft_dma_bmark_sequence(10, false);
        break;
      case 1:
        run_cpu_fft_bmark_sequence(10, false);
        break;
      default:
        break;
    }
    clean_test(t);
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

// Then clone bmarks, add folder there
// Then make a local CMake
// Then add to the bmarks cmake
// Then need to make Python file ( can steal existing as ref then bug ethan )

// They dynamically change the baud rate for communications for example for getting data to stay in a readable format over UART
// But can also just delete the print statements to avoid slowing stuff down / garbled output