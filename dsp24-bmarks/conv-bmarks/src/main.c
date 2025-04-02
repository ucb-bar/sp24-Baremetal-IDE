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

#define INPUT_LENGTH 1024
#define QUEUE_DEPTH 256

typedef struct {
  uint64_t cycles;
  bool correct;
} convtest_result_t;

uint16_t in_kernel[8] = {0xBC00, 0x4400, 0x3C00, 0xBC00, 0x4400, 0xBC00, 0x4400, 0x4000};

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

void cpu_f16_test(int seed) {
  convtest_result_t result;
  uint64_t time;

  // setup input
  uint16_t in_arr[INPUT_LENGTH/2];
  volatile uint16_t ref_out[INPUT_LENGTH];

  for (int i = 0; i < INPUT_LENGTH; i++) {
    in_arr[i] = f16_from_int(i);
  }

  start_roi();

  time = get_cycles();
  convolution_1D_f16(in_arr, INPUT_LENGTH, in_kernel, 8, 0, ref_out);
  result.cycles = get_cycles() - time;

  end_roi();

  result.correct = true;
  xmit_payload_packet(&result, 9);

}

void convaccel_test(int seed) {
  convtest_result_t result;
  uint64_t time;

  uint16_t in_arr[INPUT_LENGTH/2];
  volatile uint16_t ref_out[INPUT_LENGTH/2];
  volatile uint16_t conv_out[INPUT_LENGTH/2];

  for (int i = 0; i < INPUT_LENGTH; i++) {
    in_arr[i] = f16_from_int(i);
  }

  uint64_t* in_kernel_ptr = in_kernel;

  CONVACCEL->LENGTH = INPUT_LENGTH;
  CONVACCEL->KERNEL = in_kernel_ptr[0];
  CONVACCEL->KERNEL = in_kernel_ptr[1];
  CONVACCEL->START = 1;
  
  time = get_cycles();
  int transfer_count = INPUT_LENGTH/4;
  for (int i = 0; i < transfer_count; i += QUEUE_DEPTH) {
    for (int j = i; (j < i + QUEUE_DEPTH) && (j < transfer_count); j++) {
      CONVACCEL->DATA_ENQUEUE = in_kernel_ptr[j];
    }
    for (int j = i; (j < i + QUEUE_DEPTH) && (j < transfer_count); j++) {
      conv_out[j] = CONVACCEL->RESULT_DEQUEUE;
    }
  }
  result.cycles = get_cycles() - time;

  convolution_1D_f16(in_arr, INPUT_LENGTH, in_kernel, 8, 0, ref_out);

  bool res_correct = true;
  for (int i = 0; i < INPUT_LENGTH/2; i++) {
    if (conv_out[i] != ref_out[i]) {
      res_correct = false;
      break;
    }
  }
  result.correct = res_correct;
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