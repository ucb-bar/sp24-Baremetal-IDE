/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main FFT comparison & Shazam program body
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
#include <stdbool.h>
#include "chip_config.h"
#include "hal_DMA.h"
#include "hal_fft.h"
#include "kiss_fft.h"

/* Constant definitions */
#define DMA_ADDR1 0x87000000L
#define INPUT_ADDR1 0x08000000U // Where to save data - scratchpad is 0x08000000U
#define NFFT 128 // FFT length for all
#define RM_IMAG 0 // Remove imaginary values for easier output parsing
#define FIXED_POINT 16 // For kiss_fft: 32 is int32_t, 16 is int16_t, undefined is float

/* Test data */
#include "meep.h" // .wav HEADER[] file
#include "tone_samples.h" // Input data samples 
#define SAMPLING_FREQ 880.0 // In Hz for current test
#define SAMPLE_CHOICE B3_samples_128 // From the headers 
#define SAMPLE_CHOICE_NAME "B3_samples_128"

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

/* START DMA FFT TEST */
void run_dma_fft_test(uint32_t* data, bool print) {
  printf("Starting DMA FFT on input: %s\r\n", SAMPLE_CHOICE_NAME);

  /* SETUP */
  reset_fft();
  reset_DMA(); // reset is at DMA base address 
  // DO NOT enable crack - unreliable, causes race conditions, or may deadlock, depending on where you look
  disable_Crack();
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");

  /* WRITE DATA */
  write_fft_dma(1, NFFT, data); // does reg_write32, set_DMAC, start_DMA
  // This is needed since fft is blocking and is not a very good block
  while(fft_busy() || fft_count_left()) {
    continue;
      printf("[Blocking] pain:%d, %d \r\n", fft_busy(), fft_count_left());
  };

  /* READ DATA */
  read_fft_dma(1, NFFT, INPUT_ADDR1); // does set_DMAC, start_DMA
  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  /* RESULTS */
  printf("[DONE] DMA FFT Transformation Complete\r\n");
  printf("@ mcycle = %lu\r\n", end_time - start_time);
  printf("@ minstret = %lu\r\n", end_instructions - start_instructions);
  
  if (print) {
    /* Old way of reading, reg_read32() is safer */
    // for (int i = 0; i < NFFT; i++) {
    //   printf("Imag: %d  Real: %d\r\n", 
    //     (*((uint32_t*) (INPUT_ADDR1 + 4*i)) >> 16), 
    //     (*((uint32_t*) (INPUT_ADDR1 + 4*i)) && 0xFFFF));
    // }

    uint32_t poll;
    int index = 0;
    float max = 0; // should be fine even if values are int
    for(int i=0; i<NFFT; i++) {
      poll = reg_read32(INPUT_ADDR1 + i*8); 
      uint16_t imag = poll >> 16; // Imaginary is top 16 bits 
      uint16_t real = poll & 0xFFFF; // Real is bottom 16 bits - this masks the top
      if (fabs(real) > max) { // abs is int, fabs is double (8 bytes)
        max = fabs(real);
        index = i;
      }
      if (RM_IMAG) {
        printf("[%d] [DMA] Real: (%hd)\r\n", i, real);
      } else {
        printf("[%d] [DMA] Imag: (%hd), Real: (%hd)\r\n", i, imag, real); // %hd - short int in decimal form
      }
      printf("[DMA] Resulting frequency is about %f\r\n", (SAMPLING_FREQ) * index / NFFT);
    }
  }
}

/* START CPU FFT TEST */
void run_cpu_fft_test(uint32_t* data, bool print) {
  printf("Starting CPU FFT on input: %s\r\n", SAMPLE_CHOICE_NAME);

  /* SETUP */
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");
  // Allocates memory for FFT + parameters but not buffers
  // Return value is a contiguous block of memory, can be free()d
  kiss_fft_cfg cfg = kiss_fft_alloc(NFFT , 0, 0, 0);
  // Allocate memory for the input data buffer
  kiss_fft_cpx* fftbuf = (kiss_fft_cpx*) malloc(NFFT * sizeof(kiss_fft_cpx));
  // Allocate memory for the output data buffer
  kiss_fft_cpx* fftoutbuf = (kiss_fft_cpx*) malloc(NFFT * sizeof(kiss_fft_cpx));
  // Load data into input buffer
  for(int i = 0; i < NFFT; i += 1) {
      // kiss_fft_cpx is struct with kiss_fft_scalar real, imaginary of chosen type (see FIXED_POINT)
      fftbuf[i].r = data[i]; 
      fftbuf[i].i = 0;
  }

  /* DO THE FFT TRANFORMATION */
  // actually kiss_fft_stride -> kf_work -> openmp -> magic, trust me bro
  kiss_fft(cfg, fftbuf, fftoutbuf);
  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  /* RESULTS */
  printf("[DONE] CPU FFT Transformation Complete\r\n");
  printf("@ mcycle = %lu\r\n", end_time - start_time);
  printf("@ minstret = %lu\r\n", end_instructions - start_instructions);

  if (print) {
    int index = 0;
    float max = 0; // should be fine even if values are int
    for (int i = 0; i < NFFT; i++) { 
      if (fabs(fftoutbuf[i].r) > max) { // original code uses .i and I don't know why
        max = fabs(fftoutbuf[i].r);  // original code uses .i and I don't know why
        index = i;
      }
      /* Original defaults to float */
      // printf("[%d] [CPU] Imag: (%f)  Real: (%f)\r\n", i, fftoutbuf[i].i, fftoutbuf[i].r); 
      /* For uint16_t */
      printf("[%d] [CPU] Imag: (%hd)  Real: (%hd)\r\n", i, fftoutbuf[i].i, fftoutbuf[i].r);
    }
    printf("Resulting frequency is about %f\r\n", (SAMPLING_FREQ) * index / NFFT);
  }

  /* CLEANUP */
  free(cfg);
  free(fftbuf);
  free(fftoutbuf);
  kiss_fft_cleanup();
}

/* Untested in new function form - moved code (that was working) from main 
  Don't forget to free the KISS FFT memory after done, but not before.
*/
void run_line_by_line_compare(kiss_fft_cfg cfg, kiss_fft_cpx* fftbuf, kiss_fft_cpx* fftoutbuf) {
  /* OLD LINE BY LINE COMPARISON BELOW */
  printf("[Start CPU vs DMA Comparison]\r\n");

  uint32_t poll_dma;
  int index_dma = 0;
  float max_dma = 0;

  int index_cpu = 0;
  float max_cpu = 0;

  for (int i = 0; i < NFFT; i++) {
    /* DMA Check */
    poll_dma = reg_read32(INPUT_ADDR1 + i*8);
    uint32_t real_dma = poll_dma & 0xFFFF;
    uint32_t imag_dma = (poll_dma >> 16);
    if (fabs(real_dma) > max_dma) {
      max_dma = fabs(real_dma);
      index_dma = i;
    }
    if (RM_IMAG) {
      printf("[%d] [DMA] Real: (%hd)\r\n", i, real_dma);
    } else {
      printf("[%d] [DMA] Imag: (%hd), Real: (%hd)\r\n", i, imag_dma, real_dma);
    }

    /* Kiss FFT CPU Check */
    if (fabs(fftoutbuf[i].i) > max_cpu) {
      max_cpu = fabs(fftoutbuf[i].i);
      index_cpu = i;
    }
    if (RM_IMAG) {
      printf("[%d] [CPU] Real: (%f)\r\n", i, fftoutbuf[i].r);
    } else {
      printf("[%d] [CPU] Imag: (%f),  Real: (%f)\r\n", i, fftoutbuf[i].i, fftoutbuf[i].r);
    }

  }

  printf("[CPU] Resulting frequency is about %f\r\n", (SAMPLING_FREQ) * index_cpu / NFFT);
  printf("[DMA] Resulting frequency is about %f\r\n", (SAMPLING_FREQ) * index_dma / NFFT);

  /* Old code - can  delete if above works */
  // uint32_t poll;
  // int index = 0;
  // float max = 0;
  // // for (int i = 0; i < NFFT; i++) {
  // for (int i = 0; i < 10; i++) { // only testing first few values
  //   poll = reg_read32(INPUT_ADDR1 + i*8);
  //   uint32_t real = poll & 0xFFFF;
  //   uint32_t imag = (poll >> 16);
  //   printf("[%d] [CPU] Imag: (%hd), Real: (%hd)\r\n", i, imag, real);

  //   if (fabs(fftoutbuf[i].i) > max) {
  //     max = fabs(fftoutbuf[i].i);
  //     index = i;
  //   }
  //   printf("[%d] [DMA] Imag: (%f),  Real: (%f)\r\n", i, fftoutbuf[i].i, fftoutbuf[i].r);

  // }
  // printf("Resulting frequency is about %f\r\n", (SAMPLING_FREQ) * index / nfft);

  printf("[End CPU vs DMA Comparison]\r\n");

  /* Even older code - larger length examples */

  // uint32_t poll, real, imag;
  // for(int i=0; i<512; i++) {
  //     poll = reg_read32(DMA_ADDR1 + i*8);
  //     real = poll & 0xFFFF; 
  //     imag = (poll >> 16);
  //     printf("[%d]real: (%hd), imag: (%hd)\r\n", i, real, imag);
  // }
  // for(int i=0; i<256; i++) {
  //     poll = reg_read16(DMA_ADDR1 + i*4);
  //     printf("[%d]real: (%hd)\r\n", i, poll);
  // }
}

void run_dma_cpu_comparison(uint32_t* data) {
    uint64_t mhartid = READ_CSR("mhartid");

    printf("\r\n[STARTING TEST]\r\n");

    run_dma_fft_test(data, true);

    reset_fft();
    reset_DMA();

    run_cpu_fft_test(data, true);
    
    printf("[DONE TEST]\r\n");
}
/* USER CODE END PUC */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  /* MCU Configuration--------------------------------------------------------*/

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

  /* Start Shazam Demo */

  struct WAVHeader* header = &HEADER;
  //fread(&header, sizeof(header), 1, fp);

  // Check if it's a valid WAV file
  if (strncmp(header->chunkID, "RIFF", 4) != 0 ||
      strncmp(header->format, "WAVE", 4) != 0) {
      fprintf(stderr, "Invalid WAV file\r\n");
      
      return 1;
  }

  printf("Channels: %u\r\n", header->numChannels);
  printf("Sample Rate: %u\r\n", header->sampleRate);
  printf("Bits per Sample: %u\r\n", header->bitsPerSample);

  // Read audio data
  uint32_t *data = header + sizeof(struct WAVHeader);
  printf("header size: %u\r\n", header->subchunk2Size);
  

  // Process the audio data here

  /* End Shazam Demo */

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
    run_dma_cpu_comparison(SAMPLE_CHOICE);
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