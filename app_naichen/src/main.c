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
#include "main.h"
#include "fft_data.h"
#include "chip_config.h"

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



// Addresses for writing data
#define DMA_ADDR1 0x87000000L // address for FFT

void app_main() {
  uint64_t mhartid = READ_CSR("mhartid");
  printf("sadly unalive myself from hart : %d\r\n", mhartid);

  puts("\n[STARTING TEST]\n\n");
    // Setup FFT stuff
    puts("Setup params for FFT");
    reset_fft();

    // Start DMA
    enable_Crack();
    printf("Starting blocks\n");
    write_fft_dma(1, fft_len, fft_data);
    
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
    printf("\n[DONE TEST]\n\n");
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