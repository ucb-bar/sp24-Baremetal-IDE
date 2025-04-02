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
#include "chip_config.h"
#include <inttypes.h>
#include "util.h"

/*
| 15-14 |   13   |   12   |    11    |    10    |
  RSVD    RX_FL     TX_FL   RX_USE_F   TX_USE_F   
|  9-8  |  7-6  |   5-4   |    3    |   2   |   1   |
  RX_BD   TX_BD   WS_SIZE   DAC_EN    RX_EN   TX_EN
*/

#define CHANNEL 0
#include "btstory.h"
uint64_t* audio = (uint64_t*) story_bearly_wav;

I2S_PARAMS I2S_DEFAULT = {
    .channel     = 0,
    .tx_en       = 1,
    .rx_en       = 1,
    .bitdepth_tx = 3,
    .bitdepth_rx = 3,
    .clkgen      = 1,
    .dacen       = 0,
    .ws_len      = 3,
    .clkdiv      = 7,
    .tx_fp       = 0,
    .rx_fp       = 0,
    .tx_force_left = 0,
    .rx_force_left = 0
};

uint16_t kernel[8] = {
  0x0000, 0x3C00, 0x0000, 0x3C00, 0x0000, 0x3C00, 0x0000, 0x3C00
};

void app_init() {
  uint64_t mhartid = READ_CSR("mhartid");

  printf("(BEGIN) On hart: %d", mhartid);

  //set_I2S_params(CHANNEL, true, true, 3, 3, true, 0, 2);
  //set_I2S_en(0, true, true);
  //set_I2S_clkdiv(CHANNEL, 7);

  printf("I2S params initializing");

  set_I2S_params(&I2S_DEFAULT);

  printf("Init done");
}


void compare_dma() {
  printf("(TEST 1) Normal start");
  uint64_t normal_start = get_cycles();
  size_t counter;
  double pi;
  for (counter = 0; counter < 32; counter++)
    write_I2S_tx(CHANNEL, true, audio[counter]);
  //pi = compute_pi(10);

  for (; counter < 64; counter++)
    write_I2S_tx(CHANNEL, true, audio[counter]);
  //pi = compute_pi(10);

  for (; counter < 96; counter++)
    write_I2S_tx(CHANNEL, true, audio[counter]);
  //pi = compute_pi(10);

  for (; counter < 128; counter++)
    write_I2S_tx(CHANNEL, true, audio[counter]);
  //pi = compute_pi(10);

  uint64_t normal_end = get_cycles();

  printf("(TEST 1) Normal end");
  printf("Normal cycle count: %d", normal_end - normal_start);
 //printf("DEBUG: pi %f", pi);

  printf("(TEST 1) DMA start");
  uint64_t accel_start = get_cycles();
  
  write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 0 , true, 100000);
  pi = compute_pi(10);
  write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 32, true, 100000);
  pi = compute_pi(10);
  write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 64, true, 100000);
  pi = compute_pi(10);
  write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 96, true, 100000);
  pi = compute_pi(10);

  uint64_t accel_end = get_cycles();

  printf("(TEST 1) DMA end");
  printf("DMA cycle count: %d", accel_end - accel_start);
  //printf("DEBUG: pi %f", pi);
}

#define INPUT_ADDR      0x08800000
#define OUTPUT_ADDR     0x08800020
#define KERNEL_ADDR     0x08800040
#define START_ADDR      0x0880006C
#define LENGTH_ADDR     0x08800078
#define DILATION_ADDR   0x0880007C
#define INPUT_TYPE_ADDR 0x0880008E
#define RESET_ADDR      0x0880008E

void app_main() {

    printf("\r\nStarting test");
    reg_write8(RESET_ADDR, 1);

    // https://bwrcrepo.eecs.berkeley.edu/ee290c_ee194_intech22/sp24-chips/-/wikis/digital/dsp24/Programming-Interfaces#convolution-accelerator
    // reg_write8(INPUT_TYPE_ADDR, 0);
    uint8_t* in_arr = (uint8_t*) audio;
    uint32_t in_len[1] = {story_bearly_wav_len};
    uint16_t in_dilation[1] = {1};
    uint16_t in_kernel[8] = {0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000}; // {2, 0, 0, 0, 0, 0, 0, 0} in FP16              

    printf("\r\nSetting values of MMIO registers");
    set_conv_params(story_bearly_wav_len, 1, ((uint64_t*) in_kernel));                  
    write_conv_dma(0, story_bearly_wav_len / 4, in_arr);

    uint64_t cpu_start_cycles = READ_CSR("mcycle");
    //printf("\r\nStarting Convolution");
    asm volatile ("fence");
    start_conv();

    uint64_t cpu_end_cycles = READ_CSR("mcycle");

    asm volatile ("fence");

    //printf("\r\nWaiting for convolution to complete");
    
    //printf("\r\nInput (INT8): ");
    //for (int i = 0; i < story_bearly_wav_len; i++) {
    //    printf("%u ", in_arr[i]);
    //}

    uint16_t test_out[story_bearly_wav_len];
    read_conv_dma(1, story_bearly_wav_len / 8, ((uint64_t*) test_out));
    //printf("\r\nTest Output (INT8): ");
    asm volatile ("fence");

    uint8_t true_out[story_bearly_wav_len];

    asm volatile ("fence");
    for (int i = 0; i < story_bearly_wav_len / 8; i++) {
        uint8_t out = (uint8_t) f16_int(test_out[i]);
        true_out[i] = out;
    }

    for (int i = 0; i < story_bearly_wav_len / 8; i++) {
        printf("%d ", true_out[i]);
    }
    
    fflush(stdout);
}

void app_main_test() {

    printf("\r\nStarting test");
    reg_write8(RESET_ADDR, 1);

    // https://bwrcrepo.eecs.berkeley.edu/ee290c_ee194_intech22/sp24-chips/-/wikis/digital/dsp24/Programming-Interfaces#convolution-accelerator
    // reg_write8(INPUT_TYPE_ADDR, 0);
    int8_t in_arr[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint32_t in_len[1] = {16};
    uint16_t in_dilation[1] = {1};
    uint16_t in_kernel[8] = {0x4000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}; // {2, 0, 0, 0, 0, 0, 0, 0} in FP16              

    printf("\r\nSetting values of MMIO registers");
    set_conv_params(16, 1, ((uint64_t*) in_kernel));                  
    write_conv_dma(0, 16, in_arr);

    uint64_t cpu_start_cycles = READ_CSR("mcycle");
    printf("\r\nStarting Convolution");
    asm volatile ("fence");
    start_conv();

    uint64_t cpu_end_cycles = READ_CSR("mcycle");

    asm volatile ("fence");

    printf("\r\nWaiting for convolution to complete");
    
    printf("\r\nInput (INT8): ");
    for (int i = 0; i < 16; i++) {
        printf("%d ", in_arr[i]);
    }

    uint16_t test_out[32];
    read_conv_dma(0, 16, ((uint64_t*) test_out));
    printf("\r\nTest Output (FP16 binary): ");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("0x%"PRIx64" ", test_out[i*4 + j]);
        }
    }

    uint16_t ref_out[32];
    for (int i = 0; i < 16; i++) {
        ref_out[i] = f16_from_int((int32_t) (in_arr[i] * 2));
    }
    printf("\r\nReference Output (FP16 binary): ");
    for (int i = 0; i < 16; i++) {
        printf("%#x ", ref_out[i]);
    }
    printf("\r\n");

    if (memcmp(test_out, ref_out, 16) == 0) {
        printf("\r\n[TEST PASSED]: Test Output matches Reference Output.");
        printf("\r\nmcycle: %llu", cpu_end_cycles - cpu_start_cycles);
    } else {
        printf("\r\n[TEST FAILED]: Test Output does not match Reference Output.");
        printf("\r\nmcycle: %llu", cpu_end_cycles - cpu_start_cycles);
    }
    printf("\r\n\r\n");

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

volatile uint16_t* i2s_config = (uint16_t*) 0x10042000;
volatile uint16_t* i2s_clkdiv = (uint16_t*) 0x10042014;
volatile uint64_t* i2s_lenqueue = (uint64_t*) 0x10042020;
volatile uint64_t* i2s_renqueue = (uint64_t*) 0x10042028;
volatile uint8_t* i2s_status = (uint8_t*) 0x10042008;

void manual_setting_test(void)
{
	*i2s_clkdiv = 7;
    asm volatile("fence");
	*i2s_config = 0x00F3;
    asm volatile("fence");

	for (int i = 0; i < story_bearly_wav_len; i++) {
		*i2s_lenqueue = audio[i];
        //while ((*i2s_status) & 0b10) {
        //    asm volatile("nop");
        //}

        //while ((*i2s_status) & 0b10) {
        //    asm volatile("nop");
        //}
	}

    printf("Second time for debugging potentially\r\n");

    for (int i = 0; i < story_bearly_wav_len; i++) {
		*i2s_lenqueue = audio[i];
        //while ((*i2s_status) & 0b10) {
        //    asm volatile("nop");
        //}

        //while ((*i2s_status) & 0b10) {
        //    asm volatile("nop");
        //}
	}

    asm volatile("fence");
    //for (int j = 0; j < story_bearly_wav_len; j += 32) {
    //    write_I2S_tx_DMA(CHANNEL, j % 2, 32, story_bearly_wav + j , true, 100000);
    //    
    //}
}

void app_test(void)
{
	printf("Audio test start\r\n");

	for (int i = 0; i < story_bearly_wav_len; i++) {
		*i2s_lenqueue = audio[i];
        // Debugging attempts
        //while ((*i2s_status) & 0b10) {
        //    asm volatile("nop");
        //}
        
        //while ((*i2s_status) & 0b10) {
        //    asm volatile("nop");
        //}
	}

    printf("Audio test end\r\n");

    for (int j = 0; j < story_bearly_wav_len; j += 32) {
        write_I2S_tx_DMA(CHANNEL, 0, 32, story_bearly_wav + j , true, 10);
        printf("Audio test DMA attempt\r\n");
    }
    
    asm volatile("fence");
}


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
  app_test(); // manual_setting_test();
  //conv_test(); //compare_dma();
  //compare_dma();
  //simple_test();
  //app_main_test();
  return 0;
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