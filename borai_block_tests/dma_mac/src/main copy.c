/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Jasmine Angle (angle@berkeley.edu)
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
#include "chip_config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * BENCHMARK_ONLY
 * 
 * If defined, correctness outputs will be disabled for benchmarking purposes.
 * This allows you to write once for both a test and a benchmark at the same
 * time.
 * If this is commented, the results of benchmarking should be ignored, as the
 * UART printing statements will add artificial delay to the test.
 */
// #define BENCHMARK_ONLY

/**
 * Number of rows and columns stored within the DMA matrix.
 * 
 * Matrix M1
 * o---> DMA_NUM_COLS --o
 * | ------ v2_1 ------ |
 * | ------ v2_2 ------ |
 * | ------ v2_. ------ |
 * v
 * DMA_NUM_ROWS
 * 
 * Vector v1 (size DMA_NUM_COLS)
 */
#define DMA_NUM_ROWS 16
#define DMA_NUM_COLS 64

#define DST_INTENDED_ARRAY ((int16_t*) (SCRATCH_BASE + 0x2000U))

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


/**
 * dma_verify_MAC
 * 
 * Runs and verifies the output of a Matrix-Vector Multiplication run on the DMA
 * MAC module. This function is not pure, though prints out results to UART0,
 * assuming a valid UART channel has been initialized prior to execution.
 * 
 * Inputs:
 *  DMA_Type *DMAX: Pointer to a DMA_Type structure, already implemented as DMA0-3.
 *  void *src: Pointer to the first element of the matrix of int8_t values.
 *  void *operand: Pointer to the first element of the operand vector's int8_t
 *                 values.
 *  uint64_t src_stride: Source stride between rows (in bytes).
 *  uint32_t count: Number of rows in the source matrix (must be <32).
 *  int16_t *dst: Destination array to hold output values.
 *  char *test_name: Name for the test to print to UART0.
 */
void dma_verify_MAC(DMA_Type *DMAX, void *src, int8_t *operand, uint64_t src_stride, uint32_t count, int16_t *dst, char *test_name) {
  uint64_t mhartid = READ_CSR("mhartid");
  printf("--------------- [HART ID %lu] Starting test \"%s\" ---------------\r\n", mhartid, test_name);

  printf("Clearing destination array...");
  for (size_t i = 0; i < count; i++) {
    dst[i] = 0x0000U;
  }
  printf("Done!\r\n");

  // Start matrix-vector multiplication. Blocking operation.
  uint64_t start_time = READ_CSR("mcycle");
  dma_init_MAC(DMAX, src, operand, src_stride, count);
  uint64_t end_time = READ_CSR("mcycle");
  DMA_Status status = dma_get_MAC_result(DMAX, dst, count);
  if (status != DMA_OK) {
    printf("ERROR: DMA returned error status code %d\r\n", status);
  } else{
    printf("DMA OK (Finished in %lu cycles)\r\n", end_time - start_time);
  }

  printf("VERIFY: --- DMA Verify MAC Results Utility ---\r\n");
  printf("VERIFY: DMA Stride:\t%lu\r\n", DMAX->SRCSTRIDE);
  printf("VERIFY: Destination Address:\t%p\r\n", dst);
  printf("VERIFY: Source Address:\t%p (has offset of %lu bytes)\r\n", src, ((uint64_t)src % 0x80));
  printf("VERIFY: Row Count:\t%u\r\n", count);
  printf("VERIFY: Operand Buffer Input Dump (Int8):");
  
  for (size_t i = 0; i < 64; i++) {
    if (i % 8 == 0) {
      printf("\r\n%p:", (&operand) + i);
    }
    printf("\t%d", operand[i]);
  }

  printf("\r\nVERIFY: DMA Operand Register Dump (Int8):");
  for (size_t i = 0; i < 64; i++) {
    if (i % 8 == 0) {
      printf("\r\n%p:", (&DMAX->OPERAND_REG) + i);
    }
    printf("\t%d", (int16_t)DMAX->OPERAND_REG[i]);
  }

  printf("\r\n");

  printf("Allocating space for intended compuation result array in Scratchpad Bank...");
  for (size_t i = 0; i < count; i++) {
    DST_INTENDED_ARRAY[i] = 0x0000U;
  }
  printf("Done!\r\n");

  int16_t *intended = DST_INTENDED_ARRAY;

  uint64_t start_time_naive = READ_CSR("mcycle");
  for (size_t i = 0; i < count; i++) {
    for (size_t j = 0; j < 64; j++) {
      size_t in = src_stride * i;
      int8_t *ptr = (int8_t*) src + in + j;
      intended[i] += operand[j] * (*ptr);
    }
  }
  uint64_t end_time_naive = READ_CSR("mcycle");
  printf("VERIFY: Naive run completed in %lu cycles.\r\n", end_time_naive - start_time_naive);

  for (size_t i = 0; i < count; i++) {
    if (intended[i] == DMAX->DEST_REG[i]) {
      printf("VERIFY: [CORRECT :D]");
    } else {
      printf("VERIFY: [INVALID D:]");
    }
    printf("\t(Row %lu)\t\tExpected\t%d\tGot\t%d\r\n", i, intended[i], DMAX->DEST_REG[i]);
  }

  
  printf("Intended Result Dump (Int8):");
  for (size_t i = 0; i < count; i++) {
    if (i % 8 == 0) {
      printf("\r\n%p:", (intended) + i);
    }
    printf("\t%#x", (uint16_t)intended[i]);
  }
  printf("\r\nDMA Destination Register Dump (Int8, Full Register):");
  for (size_t i = 0; i < 32; i++) {
    if (i % 8 == 0) {
      printf("\r\n%p:", &DMAX->DEST_REG + i);
    }
    printf("\t%d", (int16_t)DMAX->DEST_REG[i]);
  }

  printf("\r\nDMA Destination Register Dump (Int8 Hexadecimal, Full Register):");
  for (size_t i = 0; i < 32; i++) {
    if (i % 8 == 0) {
      printf("\r\n%p:", &DMAX->DEST_REG + i);
    }
    printf("\t%#04x", (uint16_t)DMAX->DEST_REG[i]);
  }
  printf("\r\nVERIFY: !-- End DMA Verify MAC Results Utility --!\r\n");
  // free(intended);
  printf("--------------- [HART ID %lu] Finished test \"%s\" ---------------\r\n", mhartid, test_name);
}


int8_t *alloc_ones_matrix(size_t rows, size_t cols, int8_t *result) {
  if (result == NULL) {
    result = malloc(sizeof(int8_t) * rows * cols);
  }
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      result[i*cols+j] = 0x01 + (i % 2);
    }
  }
  return result;
}

int8_t *alloc_zero_matrix(size_t rows, size_t cols, int8_t *result) {
  if (result == NULL) {
    result = malloc(sizeof(int8_t) * rows * cols);
  }
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      result[i*cols+j] = 0x00;
    }
  }
  return result;
}

#define CACHELINE         128
#define REGION_SIZE_LINES 16
#define STRIDE (0.75 * CACHELINE)

__attribute__((aligned(CACHELINE))) int8_t mem1[REGION_SIZE_LINES][CACHELINE / sizeof(int8_t)];

int16_t saturate(int32_t x) {
    int16_t max = (1 << 15) - 1;
    int16_t min = -(1 << 15);
    if (x > max) {
        return max;
    } else if (x < min) {
        return min;
    } else {
        return x;
    }
}

void run_old_test(int16_t *dst) {
  int offset = 42;
  
  puts("Initializing memory\r");
  for (size_t i = 0; i < REGION_SIZE_LINES; i++) {
      for (size_t j = 0; j < CACHELINE/sizeof(int8_t); j++) {
          mem1[i][j] = 1;
      }
  }
  
  int8_t operandReg[sizeof(DMA1->OPERAND_REG)/sizeof(uint8_t)];
  for (size_t i = 0; i < sizeof(operandReg)/sizeof(uint8_t); i++) {
      operandReg[i] = 1;
  }

  _Static_assert(sizeof(operandReg) == 64, "opreg size");

  // _Static_assert(STRIDE % CACHELINE == 0, "stride not aligned");
  uint32_t count = REGION_SIZE_LINES * CACHELINE / STRIDE;
  int16_t expected[sizeof(DMA1->DEST_REG)/sizeof(int16_t)];
  for (size_t i = 0; i < count; i++) {
      int32_t sum = 0;
      for (size_t j = 0; j < sizeof(operandReg)/sizeof(uint8_t) - offset; j++) {
          sum += (int16_t)operandReg[j] * (int16_t)mem1[(int) (i*STRIDE / CACHELINE)][(int) (i*STRIDE) % CACHELINE + j + offset];
      }
      for (size_t j = sizeof(operandReg)/sizeof(uint8_t) - offset; j < sizeof(operandReg)/sizeof(uint8_t); j++) {
          sum += (int16_t)operandReg[j] * (int16_t)mem1[(int) (i*STRIDE / CACHELINE) + 1][(int) (i*STRIDE) % CACHELINE + j - (sizeof(operandReg)/sizeof(uint8_t) - offset)];
      }
      expected[i] = saturate(sum);
  }

  for (size_t i = 0; i < REGION_SIZE_LINES; i++) {
        for (size_t j = 0; j < CACHELINE/sizeof(int8_t); j++) {
            printf("\t%4" PRIx16, mem1[i][j]);
        }
        printf("\n");
    }


  for (size_t i = 0; i < REGION_SIZE_LINES; i++) {
    printf("\t%4" PRIx16 "\r\n", expected[i]);
  }
  dma_verify_MAC(DMA0, &mem1[0][offset], &operandReg, STRIDE, count, dst, "Rickety old test");

  for (size_t i = 0; i < REGION_SIZE_LINES; i++) {
    printf("\t%016" PRIx64 "\r\n", DMA1->DEST_REG[i]);
  }

}


void app_init() {
  // torch::executor::runtime_init();
}

void app_main() {
  // Clears the entire screen and moves character pointer to top left
  printf("%c%c%c%c%c[0;0H",0x1B,0x5B,0x32,0x4A,0x1B);

  int16_t *dst = (int16_t*) SCRATCH_BASE;
  // int8_t v1_1[VECTOR_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
  // int8_t v2_1[VECTOR_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1};

  int8_t rows = 16;
  int8_t cols = 2;
  run_old_test(dst);

  // Tests with a simple ones matrix and shifts for every 16-byte offset.
  int8_t *operand_0 = alloc_ones_matrix(1, cols, 0x86000000);
  for (int i = 0; i < (DMA_NUM_COLS - cols); i++) {
    operand_0[cols+i] = 0x00;
  }
  for (int i = 1; i < 2; i++) {
    for (int j = 0; j < 129; j++) {
      int8_t *src_0 = alloc_ones_matrix(rows, cols, 0x85000000 + (i << 4));
      dma_verify_MAC(DMA0, src_0, operand_0, j, rows, dst, "All ones offset test");
      alloc_zero_matrix(rows, cols, src_0);
    }
    
    //alloc_zero_matrix(1, cols, operand_0);
  }

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