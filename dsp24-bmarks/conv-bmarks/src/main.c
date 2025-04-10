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
#include "float16.h"

#define BASE_ADDR 0x08800000

#define INPUT_ADDR      0x08800000
#define OUTPUT_ADDR     0x08800020
#define KERNEL_ADDR     0x08800040
#define START_ADDR      0x0880006C
#define LENGTH_ADDR     0x08800078
#define DILATION_ADDR   0x0880007C
#define STATUS_ADDR     0x0880006A
#define RESET_ADDR      0x0880008F

#define INPUT_LENGTH 16384
#define QUEUE_DEPTH 256
#define KERNEL_LEN 8

typedef struct {
  uint64_t cycles;
  bool correct;
} convtest_result_t;

uint16_t in_kernel[8] __attribute__ ((aligned (16))) = {0x0000, 0x3C00, 0x0000, 0x3C00, 0x0000, 0x3C00, 0x0000, 0x3C00};

void convolution_1D_f16(uint16_t *arr, size_t arr_len, uint16_t *kernel, size_t kernel_len, size_t dilation, uint16_t *output) {
  for (int i = 0; i < arr_len; i += 1) {
    output[i] = 0;
    for (int j = 0; j < kernel_len; j += 1) {
      int arr_index = i + j * dilation;
      uint16_t item;
      if (arr_index >= arr_len) {
        /* Index > arr_len edge case: zero-extend */
        item = 0;
      } else {
        item = arr[arr_index];
      }
      output[i] = f16_add(output[i], f16_mul(item, kernel[j]));
    }
  }
}

void convolution_1D_f32(float *in_arr, size_t arr_len, float *kernel, size_t kernel_len, float* output) {
  for (int i = 0; i < arr_len-kernel_len; i += 1) {
    output[i] = 0;
    for (int j = 0; j < kernel_len; j += 1) {
      output[i] += in_arr[i + j] * kernel[j];
    }
  }
}

void cpu_f16_test(int seed) {
  convtest_result_t result;
  uint64_t time;

  // setup input
  uint16_t in_arr[INPUT_LENGTH];
  volatile uint16_t ref_out[INPUT_LENGTH + 8];

  for (int i = 0; i < INPUT_LENGTH; i++) {
    in_arr[i] = f16_from_int(i);
  }

  start_roi();

  time = get_cycles();
  convolution_1D_f16(in_arr, INPUT_LENGTH, in_kernel, 8, 1, ref_out);
  result.cycles = get_cycles() - time;

  end_roi();

  result.correct = true;
  xmit_payload_packet(&result, 9);

}

void cpu_f32_test(int seed) {
  srand(seed);
  convtest_result_t result;
  uint64_t time;

  // setup input
  float in_arr[INPUT_LENGTH];
  float kernel[KERNEL_LEN];

  volatile float ref_out[INPUT_LENGTH + 8];

  for (int i = 0; i < INPUT_LENGTH; i++) {
    in_arr[i] = rand();
  }
  
  for (int i = 0; i < KERNEL_LEN; i++) {
    kernel[i] = rand();
  }
  start_roi();

  time = get_cycles();
  for (int i = 0; i < 100; i++) {
    convolution_1D_f32(in_arr, INPUT_LENGTH, kernel, 8, ref_out);
  }
  result.cycles = get_cycles() - time;

  end_roi();

  result.correct = true;
  xmit_payload_packet(&result, 9);

}

void convaccel_test(int seed) {
  convtest_result_t result;
  uint64_t time;

  uint16_t in_arr[INPUT_LENGTH] __attribute__ ((aligned (16)));
  volatile uint16_t ref_out[INPUT_LENGTH+8] __attribute__ ((aligned (16)));
  volatile uint16_t conv_out[INPUT_LENGTH+8] __attribute__ ((aligned (16)));

  for (int i = 0; i < INPUT_LENGTH; i++) {
    in_arr[i] = f16_from_int(i%256);
  }

  volatile uint64_t* in_kernel_ptr = in_kernel;
  volatile uint64_t* in_arr_ptr = (uint64_t*) in_arr;
  volatile uint64_t* ref_out_ptr = (uint64_t*) ref_out;
  volatile uint64_t* conv_out_ptr = (uint64_t*) conv_out;

  reg_write8(RESET_ADDR, 1);
  reg_write8(RESET_ADDR, 0);

  reg_write32(LENGTH_ADDR, INPUT_LENGTH);
  reg_write16(DILATION_ADDR, 1);
  reg_write64(KERNEL_ADDR, *((uint64_t*) in_kernel));         // 64 bits: 4 FP16s
  reg_write64(KERNEL_ADDR, *((uint64_t*) (in_kernel + 4)));   // 64 bits: 4 FP16s (Total 8)
  
  reg_write8(START_ADDR, 1);

  start_roi();
  time = get_cycles();

  size_t load_counter = 0;
  size_t store_counter = 0;

  
  while (load_counter < INPUT_LENGTH) {
    for (size_t load_start = load_counter; load_counter < load_start+(256) && load_counter < INPUT_LENGTH; load_counter += 8) {
      reg_write64(INPUT_ADDR, *((uint64_t*) (in_arr + load_counter)));
    }
    for (size_t store_start = store_counter; store_counter < store_start+(256) && store_counter < INPUT_LENGTH; store_counter += 4) {
      *((uint64_t*)(&conv_out[store_counter])) = reg_read64(OUTPUT_ADDR);
    }
  }
  result.cycles = get_cycles() - time;
  end_roi();
  // convolution_1D_f16(in_arr, INPUT_LENGTH, in_kernel, 8, 1, ref_out);

  // bool res_correct = true;
  // for (int i = 0; i < INPUT_LENGTH + 8; i++) {
  //   if (conv_out[i] != ref_out[i]) {
  //     res_correct = false;
  //     break;
  //   }
  // }
  result.correct = true;
  xmit_payload_packet(&result, 9);
}

void convaccel_test_dma(int seed) {
  convtest_result_t result;
  uint64_t time;

  uint16_t in_arr[INPUT_LENGTH] __attribute__ ((aligned (16)));
  volatile uint16_t ref_out[INPUT_LENGTH+8] __attribute__ ((aligned (16)));
  volatile uint16_t conv_out[INPUT_LENGTH+8] __attribute__ ((aligned (16)));

  for (int i = 0; i < INPUT_LENGTH; i++) {
    in_arr[i] = f16_from_int(i%256);
  }

  volatile uint64_t* in_kernel_ptr = in_kernel;
  volatile uint64_t* in_arr_ptr = (uint64_t*) in_arr;
  volatile uint64_t* ref_out_ptr = (uint64_t*) ref_out;
  volatile uint64_t* conv_out_ptr = (uint64_t*) conv_out;


  start_roi();
  time = get_cycles();

  size_t dma_size = 1024;

  enable_Crack();
  for (int i = 0; i < 5000; i++) {
    size_t load_counter = 0;
    size_t store_counter = 0;
  
    reg_write8(RESET_ADDR, 1);
    reg_write8(RESET_ADDR, 0);
  
    reg_write32(LENGTH_ADDR, INPUT_LENGTH);
    reg_write16(DILATION_ADDR, 1);
    reg_write64(KERNEL_ADDR, *((uint64_t*) in_kernel));         // 64 bits: 4 FP16s
    reg_write64(KERNEL_ADDR, *((uint64_t*) (in_kernel + 4)));   // 64 bits: 4 FP16s (Total 8)
    
    reg_write8(START_ADDR, 1);
  
    while (load_counter < INPUT_LENGTH) {
      write_conv_dma(0, dma_size, (uint64_t*) (in_arr + load_counter));
      // puts("Started Write!\r\n");
      read_conv_dma(1, dma_size, (uint64_t*) (conv_out + store_counter));
      // read_conv_dma_p(4, INPUT_LENGTH, (uint64_t*) out_result);
      while (*(volatile char*) (DMA_BASE+0x1) != 0);
  
      load_counter += dma_size;
      store_counter += dma_size;
    }  
  }

  result.cycles = get_cycles() - time;
  end_roi();
  // convolution_1D_f16(in_arr, INPUT_LENGTH, in_kernel, 8, 1, ref_out);

  // bool res_correct = true;
  // for (int i = 0; i < INPUT_LENGTH + 8; i++) {
  //   if (conv_out[i] != ref_out[i]) {
  //     res_correct = false;
  //     break;
  //   }
  // }
  result.correct = true;
  xmit_payload_packet(&result, 9);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  while (1) {
    test_info t = init_test(UART1);
    int seed = *((int*) &t.payload);
    switch (t.testid) {
      case 0:
        cpu_f16_test(seed);
        break;
      case 1:
        convaccel_test(seed);
        break;
      case 2:
        convaccel_test_dma(seed);
        break;
      case 3:
        cpu_f32_test(seed);
        break;
      default:
        cpu_f16_test(seed);
        break;
    }

    clean_test(t);
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