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


void conv_acc(uint8_t *input_audio, size_t audio_len, uint16_t *input_kernel, size_t kernel_len, size_t dilation, uint16_t *output_audio);

void conv_test() {
  size_t audio_len = story_bearly_wav_len;
  size_t kernel_len = 8;
  size_t dilation = 1;

  uint8_t* input_audio_ = audio;
  for (int i = 0; i < audio_len; i++) {
    input_audio_[i] = input_audio_[i] / 2;
  }

  uint16_t output_audio_cpu[16];
  uint16_t output_audio_acc[16];
  uint16_t input_kernel[8] = {0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000};

  uint8_t* input_audio;
  for (int stride = 0; stride < audio_len; stride += 16) {
    input_audio = input_audio_ + stride;

    software_conv(input_audio, 16, input_kernel, kernel_len, dilation, output_audio_cpu);
    conv_acc(input_audio, 16, input_kernel, kernel_len, dilation, output_audio_acc);

  // for(int i = 0; i < audio_len; i++) {
  //   if(output_audio_cpu[i] != output_audio_acc[i]) {
  //     printf("\r\nmismatch at index %d\r\n", i);
  //   }
  // }

  // printf("\r\nFinished");
  // printf("\r\noutput_audio_acc:\r\n");
  // for(int i = 0; i < audio_len; i++) {
  //   printf("0x%x - %d\r\n", output_audio_acc[i], f16_int(output_audio_acc[i]));
  // }
    for(int i = 0; i < 16; i++) {
        printf("%d ", f16_int(output_audio_acc[i]));
    }
    printf("\r\n");
  }
}

void conv_acc(uint8_t *input_audio, size_t audio_len, uint16_t *input_kernel, size_t kernel_len, size_t dilation, uint16_t *output_audio) {
  reg_write64(CONV_BASE, *((uint64_t*) (input_audio)));
  reg_write64(CONV_BASE, *((uint64_t*) (input_audio + 8)));
  set_conv_params(audio_len >= 16 ? 16 : audio_len, dilation, input_kernel);
  start_conv();
  asm volatile("fence");

  for (int i = 0; i < 4; i++) {
    uint64_t current_out = reg_read64(CONV_OUTPUT_ADDR);
    uint16_t* unpacked_out = (uint16_t*) &current_out;
    for (int j = 0; j < 4; j++) {
      output_audio[i*4 + j] = unpacked_out[j];
    }
  }

  reg_write8(CONV_START_ADDR, 0);
  reg_write8(RESET_ADDR, 1);
  asm volatile("fence");

  uint64_t start_index = 1;
  for(uint64_t i = 9; i < audio_len - 2 * kernel_len; i++) {

    uint8_t input_audio_temp[16];
    for(int j = 0; j < 16; j++) {
      input_audio_temp[j] = input_audio[start_index + j];
    }

    reg_write64(CONV_BASE, *((uint64_t*) (input_audio_temp)));
    reg_write64(CONV_BASE, *((uint64_t*) (input_audio_temp + 8)));
    set_conv_params(16, dilation, input_kernel);
    start_conv();
    asm volatile("fence");

    // uint64_t current_out = reg_read64(CONV_OUTPUT_ADDR);
    // uint16_t* unpacked_out = (uint16_t*) &current_out;
    // output_audio[i] = unpacked_out[8];

    uint16_t output_audio_temp[16];
    for (int x = 0; x < 4; x++) {
      uint64_t current_out = reg_read64(CONV_OUTPUT_ADDR);
      uint16_t* unpacked_out = (uint16_t*) &current_out;
      for (int y = 0; y < 4; y++) {
        output_audio_temp[x*4 + y] = unpacked_out[y];
      }
    }

    output_audio[i] = output_audio_temp[8];
      
    reg_write8(CONV_START_ADDR, 0);
    reg_write8(RESET_ADDR, 1);
    asm volatile("fence");

    start_index += 1;
  }
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