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
#include "rocketcore.h"
#include <inttypes.h>

/*
| 15-14 |   13   |   12   |    11    |    10    |
  RSVD    RX_FL     TX_FL   RX_USE_F   TX_USE_F   
|  9-8  |  7-6  |   5-4   |    3    |   2   |   1   |
  RX_BD   TX_BD   WS_SIZE   DAC_EN    RX_EN   TX_EN
*/

#define CHANNEL 0

I2S_PARAMS I2S_DEFAULT = {
    .channel     = 0,
    .tx_en       = 1,
    .rx_en       = 1,
    .bitdepth_tx = 3,
    .bitdepth_rx = 3,
    .clkgen      = 1,
    .dacen       = 0,
    .ws_len      = 3,
    .clkdiv      = 176,
    .tx_fp       = 0,
    .rx_fp       = 0,
    .tx_force_left = 0,
    .rx_force_left = 0
};

uint64_t target_frequency = 500000000l;


void app_init() {
  configure_pll(PLL, target_frequency/50000000, 0);
  set_all_clocks(CLOCK_SELECTOR, 1);

  uint64_t mhartid = READ_CSR("mhartid");

  printf("(BEGIN) On hart: %d", mhartid);

  printf("I2S params initializing");

  set_I2S_params(&I2S_DEFAULT);

  printf("Init done");
}


// void compare_dma() {
//   printf("(TEST 1) Normal start");
//   uint64_t normal_start = get_cycles();
//   size_t counter;
//   double pi;
//   for (counter = 0; counter < 32; counter++)
//     write_I2S_tx(CHANNEL, true, audio[counter]);
//   //pi = compute_pi(10);

//   for (; counter < 64; counter++)
//     write_I2S_tx(CHANNEL, true, audio[counter]);
//   //pi = compute_pi(10);

//   for (; counter < 96; counter++)
//     write_I2S_tx(CHANNEL, true, audio[counter]);
//   //pi = compute_pi(10);

//   for (; counter < 128; counter++)
//     write_I2S_tx(CHANNEL, true, audio[counter]);
//   //pi = compute_pi(10);

//   uint64_t normal_end = get_cycles();

//   printf("(TEST 1) Normal end");
//   printf("Normal cycle count: %d", normal_end - normal_start);
//  //printf("DEBUG: pi %f", pi);

//   printf("(TEST 1) DMA start");
//   uint64_t accel_start = get_cycles();
  
//   write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 0 , true, 100000);
//   pi = compute_pi(10);
//   write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 32, true, 100000);
//   pi = compute_pi(10);
//   write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 64, true, 100000);
//   pi = compute_pi(10);
//   write_I2S_tx_DMA(CHANNEL, 8, 32, audio + 96, true, 100000);
//   pi = compute_pi(10);

//   uint64_t accel_end = get_cycles();

//   printf("(TEST 1) DMA end");
//   printf("DMA cycle count: %d", accel_end - accel_start);
//   //printf("DEBUG: pi %f", pi);
// }

volatile uint16_t* i2s_config = (uint16_t*) 0x10042000;
volatile uint16_t* i2s_clkdiv = (uint16_t*) 0x10042014;
volatile uint64_t* i2s_lenqueue = (uint64_t*) 0x10042020;
volatile uint64_t* i2s_renqueue = (uint64_t*) 0x10042028;
volatile uint8_t* i2s_status = (uint8_t*) 0x10042008;

void manual_setting_test(void) {

  uint64_t counter = 0;
  uint64_t playback = 0;
  uint64_t recording_length = 5; //In Seconds
  uint64_t recording_cycle_length = (recording_length * 44100 / 4);
  uint64_t recorded_audio_left[recording_cycle_length];
  uint64_t recorded_audio_right[recording_cycle_length];
  uint64_t playback_audio[recording_cycle_length];
  // For reference: uint64_t target_frequency = 500000000l;
  while (1) {
    uint64_t normal_start = get_cycles();
    while (counter < recording_cycle_length) {
      recorded_audio_left[counter] = read_I2S_rx(CHANNEL, true);
      recorded_audio_right[counter] = read_I2S_rx(CHANNEL, false);
      counter++;
    }
    for (int i = 0; i < recording_cycle_length - 100; i++) {
      playback_audio[i] = recorded_audio_left[i] / 2 + recorded_audio_right[i + 100] / 2;
    }
    for (int i = recording_cycle_length - 100; i < recording_cycle_length; i++) {
      playback_audio[i] = recorded_audio_left[i];
    }
    int delay = recording_cycle_length / 20;
      for (int i = delay; i < recording_cycle_length; i++) {
        playback_audio[i] = playback_audio[i] /2 + playback_audio[i - delay] / 2;
      }

    
      
    while (playback < recording_cycle_length) {
      write_I2S_tx(CHANNEL, true, playback_audio[playback]);
      playback++;
    }
    uint64_t normal_end = get_cycles();
    printf("%d\n", normal_start - normal_end);
    counter = 0;
    playback = 0;
    //for (int i = 0; i < story_bearly_wav_len / 4; i++)
    //  write_I2S_tx(CHANNEL, true, audio[i]);
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
  //app_test(); // manual_setting_test();
  manual_setting_test();
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