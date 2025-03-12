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
#include <stdio.h>
#include <stdlib.h>
#include "chip_config.h"
#include "hal_DMA.h"
#include "hal_fft.h"
#include "meep.h"
#include "tone_samples.h"

#include "kiss_fft.h"
#define DMA_ADDR1 0x87000000L


// WAV file header structure
struct WAVHeader {
    char chunkID[4];   // Should be "RIFF"
    uint32_t chunkSize;
    char format[4];    // Should be "WAVEID"
    char junk[72];
    char subchunk1ID[4]; // Should be "fmt "
    uint32_t subchunk1Size;
    uint16_t audioFormat; // Usually 1 for PCM
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char subchunk2ID[4]; // Should be "data"
    uint32_t subchunk2Size; // Size of audio data
};

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

void app_main(uint32_t* data) {
  uint64_t mhartid = READ_CSR("mhartid");

    printf("\n[STARTING TEST]\n\n");

    // reset_fft();
    // reset_DMA();
    // // enable_Crack();

    // write_fft_dma(1, 128, data);
    // uint64_t start_time = READ_CSR("mcycle");
    // uint64_t start_instructions = READ_CSR("minstret");

    // while(fft_busy() || fft_count_left()) {
    //   continue;
    //     printf("pain:%d, %d \n", fft_busy(), fft_count_left());
    // }; // This is needed since fft is blocking and is not a very good block

    // read_fft_dma(1, 128, 0x08000000U);

    // uint64_t end_time = READ_CSR("mcycle");
    // uint64_t end_instructions = READ_CSR("minstret");

    // printf("[DONE] Waiting Write\n");
    // printf("mcycle = %lu\r\n", end_time - start_time);
    // printf("minstret = %lu\r\n", end_instructions - start_instructions);

    
    // for (int i = 0; i < 128; i++) {
    //   printf("Imag: %d  Real: %d\r\n", (*((uint32_t*) (0x08000000U + 4*i)) >> 16), (*((uint32_t*) (0x08000000U + 4*i)) && 0xFFFF));
    // }

    printf("Starting CPU FFT\r\n");

    uint64_t start_time = READ_CSR("mcycle");
    uint64_t start_instructions = READ_CSR("minstret");

    kiss_fft_cfg cfg = kiss_fft_alloc(512 , 0, 0, 0);
    int nfft = 512;

    kiss_fft_cpx* fftbuf = (kiss_fft_cpx*) malloc(nfft * sizeof(kiss_fft_cpx) );
    kiss_fft_cpx* fftoutbuf = (kiss_fft_cpx*) malloc(nfft * sizeof(kiss_fft_cpx) );

    for(int i = 0; i < nfft; i += 1) {
        fftbuf[i].r = B3_samples_512[i];
        fftbuf[i].i = 0;
    }

    kiss_fft(cfg, fftbuf, fftoutbuf);

    uint64_t end_time = READ_CSR("mcycle");
    uint64_t end_instructions = READ_CSR("minstret");

    printf("mcycle = %lu\r\n", end_time - start_time);
    printf("minstret = %lu\r\n", end_instructions - start_instructions);

    printf("Finished CPU FFT\r\n");
    int index = 0;
    float max = 0;
    for (int i = 0; i < nfft; i++) {
      if (fabs(fftoutbuf[i].i) > max) {
        max = fabs(fftoutbuf[i].i);
        index = i;
      }
      printf("Imag: %f  Real: %f\r\n", fftoutbuf[i].i, fftoutbuf[i].r);
    }

    printf("Resulting frequency is about %f\r\n", (880.0) * index / nfft);

    free(cfg);
    free(fftbuf);
    free(fftoutbuf);
    kiss_fft_cleanup();


    
    
    // uint32_t poll, real, imag;
    // for(int i=0; i<512; i++) {
    //     poll = reg_read32(DMA_ADDR1 + i*8);
    //     real = poll & 0xFFFF; 
    //     imag = (poll >> 16);
    //     printf("[%d]real: (%hd), imag: (%hd)\n", i, real, imag);
    // }
    // for(int i=0; i<256; i++) {
    //     poll = reg_read16(DMA_ADDR1 + i*4);
    //     printf("[%d]real: (%hd)\n", i, poll);
    // }
    
    printf("[DONE] Test\n");

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

  // if (argc != 2) {
  //       fprintf(stderr, "Usage: %s <wav_file>\n", argv[0]);
  //       return 1;
  //   }

  //   FILE *fp = fopen("twinkle_twinkle_as.wav", "rb");
  //   if (fp == NULL) {
  //       perror("Error opening file");
  //       return 1;
  //   }

  UART_InitType UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART_init_config);

  struct WAVHeader* header = &HEADER;
  //fread(&header, sizeof(header), 1, fp);

  // Check if it's a valid WAV file
  if (strncmp(header->chunkID, "RIFF", 4) != 0 ||
      strncmp(header->format, "WAVE", 4) != 0) {
      fprintf(stderr, "Invalid WAV file\n");
      
      return 1;
  }

  printf("Channels: %u\r\n", header->numChannels);
  printf("Sample Rate: %u\r\n", header->sampleRate);
  printf("Bits per Sample: %u\r\n", header->bitsPerSample);

  // Read audio data
  uint32_t *data = header + sizeof(struct WAVHeader);
  printf("header size: %u\r\n", header->subchunk2Size);
  

  // Process the audio data here

  
  
  /* USER CODE BEGIN SysInit */
  // UART_InitType UART_init_config;
  // UART_init_config.baudrate = 115200;
  // UART_init_config.mode = UART_MODE_TX_RX;
  // UART_init_config.stopbits = UART_STOPBITS_2;
  // uart_init(UART0, &UART_init_config);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */  
  /* USER CODE BEGIN Init */
  app_init();
  /* USER CODE END Init */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    app_main(B3_samples_512);
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