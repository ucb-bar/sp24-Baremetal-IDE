/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "main.h"
#include "chip_config.h"
#include "hal_DMA.h"
#include "hal_fft.h"

#include "dataset2.h" // original test shazam used - not sure what it corresponds to
#include "../goldenmodel/fft_data_128len_131c.h"
#include "../goldenmodel/fft_data_128len_twinkle.h"
#include "../goldenmodel/fft_expected_data_128len_131c.h"
#include "../goldenmodel/fft_expected_data_128len_twinkle.h"

// #define LOGPATH "./fft_log.txt"
#define DMA_ADDR1 0x87000000L // DMA base address
#define INPUT_ADDR1 0x08000000U // Where to save data - scratchpad is 0x08000000U
#define INPUT_DATA fft_data_twinkle
#define OUTPUT_DATA fft_expected_data_twinkle
#define NUM_POINTS 128
#define DMA_NUM 0 // Tested with 0 and 1
#define MAX_DIFF 5
#ifndef NUM_TESTS
#define NUM_TESTS 14 // 14 // will also be overwritten if in data file
#endif
#define RM_IMAG 0 // Remove imaginary values for easier output parsing


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

// void log_printf(FILE *log_file, const char *message) {
//   // Print the message to the console
//   printf("%s", message);
//   // Write the same message to the log file
//   fprintf(log_file, "%s", message);
// }

void app_main() {
  
  /* LOG FILE SETUP */

  // Open the log file in write mode
  // FILE *log_file = fopen(LOGPATH, "w");
  // if (log_file == NULL) {
  //     perror("Failed to open log file.");
  //     return;
  // }

  /* TEST SETUP */

  printf("\r\n[STARTING TEST]\r\n");
  printf("\n[NUMBER OF TESTS: %d]\r\n", NUM_TESTS);

  int error_cnt = 0;
  uint64_t mhartid = READ_CSR("mhartid");

  for (int i = 0; i < NUM_TESTS; i++) {
    // printf("[TEST: %d] [FFT] vs [NUMPY]\r\n", i);
    reset_fft();
    // enable_Crack(); // bad idea to enable for initial tests 
    uint64_t start_time = READ_CSR("mcycle");
    uint64_t start_instructions = READ_CSR("minstret");

    /* WRITE INPUT DATA */

    write_fft_dma(DMA_NUM, NUM_POINTS, (uint32_t*) INPUT_DATA[i]); 

    while(fft_busy() || fft_count_left()){
      // continue; // not sure why this was added
      printf("pain:%d, %d \r\n", fft_busy(), fft_count_left());
    }; // This is needed since fft is blocking and is not a very good block

    /* READ & COMPARE OUTPUT DATA */

    /* Making use of DMA */

    printf("[TEST: %d] [DMA-FFT] vs [NUMPY]\r\n", i);

    // while (*(volatile char*) (DMA_BASE+0x1) != 0);

    read_fft_real_dma(DMA_NUM, NUM_POINTS, DMA_ADDR1);

    // while (*(volatile char*) (DMA_BASE+0x1) != 0);

    uint32_t poll, real, imag;
    uint32_t poll_real_max = 0;
    uint32_t idx_max = 0;
    // for(int i=0; i<512; i++) {
    //     poll = reg_read32(DMA_ADDR1 + i*8);
    //     real = poll & 0xFFFF; 
    //     imag = (poll >> 16);
    //     printf("[%d]real: (%hd), imag: (%hd)\r\n", i, real, imag);
    // }
    for(int j=0; j < NUM_POINTS; j++) {
      poll = reg_read16(DMA_ADDR1 + j*4);
      int16_t poll_real = (int16_t) poll;
      int16_t expected_real = (int16_t) OUTPUT_DATA[i][j];
      if (poll_real - expected_real < -MAX_DIFF || poll_real - expected_real > MAX_DIFF) {
        printf("[FAIL @ test=%d, idx=%d] [DMA-FFT] Actual: %lx, [NUMPY] Expected: %lx]\n", i, j, poll_real, expected_real);
        error_cnt++;
      }
      printf("[idx=%d] Actual: %d, Expected: %d \r\n", j, poll_real, expected_real);

      if (poll_real > poll_real_max) {
        poll_real_max = poll_real;
        idx_max = j;
      }
    }

    // uint32_t poll_dma;
    // int index_dma = 0;
    // float max_dma = 0;
    // for (int i = 0; i < NUM_POINTS; i++) {
    //   /* DMA Check */
    //   poll_dma = reg_read32(INPUT_ADDR1 + i*8);
    //   uint32_t real_dma = poll_dma & 0xFFFF;
    //   uint32_t imag_dma = (poll_dma >> 16);
    //   if (fabs(real_dma) > max_dma) {
    //     max_dma = fabs(real_dma);
    //     index_dma = i;
    //   }
    //   if (RM_IMAG) {
    //     printf("[%d] [DMA] Real: (%hd)\r\n", i, real_dma);
    //   } else {
    //     printf("[%d] [DMA] Imag: (%hd), Real: (%hd)\r\n", i, imag_dma, real_dma);
    //   }
    // }

    /* Not making use of DMA */

    // printf("[TEST: %d] [RAW-FFT] vs [NUMPY]\r\n", i);

    // uint32_t poll;
    // uint32_t poll_real_max = 0;
    // uint32_t idx_max = 0;

    // for(int j=0; j<NUM_POINTS; j++) {
    //   poll = read_fft();
    //   int16_t poll_real = (int16_t) poll;
    //   int16_t expected_real = (int16_t) OUTPUT_DATA[i][j];

    //   if (poll_real - expected_real < -MAX_DIFF || poll_real - expected_real > MAX_DIFF) {
    //     printf("[FAIL @ test=%d, idx=%d] [FFT] Actual: %lx, [NUMPY] Expected: %lx]\n", i, j, poll_real, expected_real);
    //     error_cnt++;
    //   }

    //   printf("[idx=%d] Actual: %d, Expected: %d \r\n", j, poll_real, expected_real);

    //   if (poll_real > poll_real_max) {
    //     poll_real_max = poll_real;
    //     idx_max = j;
    //   }
      
    // }

    /* RESULTS & CLEANUP */

    uint64_t end_time = READ_CSR("mcycle");
    uint64_t end_instructions = READ_CSR("minstret");
    printf("[TEST: %d] Peak at Index %d: Actual: %d, Expected: %d \r\n", i, idx_max, poll_real_max, OUTPUT_DATA[i][idx_max]);
    printf("ERRORS FOUND: %d\r\n", error_cnt);
    printf("mcycle = %lu\r\n", end_time - start_time);
    printf("minstret = %lu\r\n", end_instructions - start_instructions);

  }
  printf("[DONE WITH ALL TESTS]\r\n");

  // Close the log file
  // fclose(log_file);

}
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