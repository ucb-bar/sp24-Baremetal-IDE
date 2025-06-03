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
#include "hal_I2S.h"
#include "hal_DMA.h"
#include "hal_mmio.h"

#define CHANNEL 0
#include "btstory.h"

uint64_t* audio = (uint64_t*) story_bearly_wav;
uint64_t target_frequency = 500000000l;

void app_init() {
    configure_pll(PLL, target_frequency / 50000000, 0);
    set_all_clocks(CLOCK_SELECTOR, 1);

    uint64_t mhartid = READ_CSR("mhartid");
    printf("(BEGIN) On hart: %d\r\n", mhartid);

    printf("I2S params initializing\r\n");

    set_I2S_params(CHANNEL, 1, 1, 3, 3, 1, 0, 3);
    set_I2S_clkdiv(CHANNEL, 176);
    set_I2S_fp(CHANNEL, 0, 0);
    set_I2S_force_left(CHANNEL, 0, 0);
    set_I2S_watermark(CHANNEL, 4, 4);

    printf("Init done\r\n");
}

void manual_setting_test(void) {
    uint64_t counter = 0;
    uint64_t playback = 0;
    uint64_t recording_length = 5; // seconds
    uint64_t recording_cycle_length = (recording_length * 44100 / 4);
    uint64_t recorded_audio[recording_cycle_length];

    while (1) {
        printf("Recording!\r\n");
        while (counter < recording_cycle_length) {
            recorded_audio[counter] = read_I2S_rx(CHANNEL, 1);
            counter++;
        }

        printf("Playing!\r\n");
        while (playback < counter) {
            write_I2S_tx(CHANNEL, 1, recorded_audio[playback]);
            playback += 2;
        }

        counter = 0;
        playback = 0;

        for (int i = 0; i < story_bearly_wav_len / 4; i++) {
            write_I2S_tx(CHANNEL, 1, audio[i]);
        }

        asm volatile("fence");
    }
}

void app_test(void) {
    printf("Audio test start\r\n");

    for (int i = 0; i < story_bearly_wav_len; i++) {
        write_I2S_tx(CHANNEL, 1, audio[i]);
    }

    printf("Audio test end\r\n");

    for (int j = 0; j < story_bearly_wav_len; j += 32) {
        write_I2S_tx_DMA(CHANNEL, 0, 32, story_bearly_wav + j, 1, 10);
        printf("Audio test DMA attempt\r\n");
    }

    asm volatile("fence");
}

int main(int argc, char **argv) {
    UART_InitType UART_init_config;
    UART_init_config.baudrate = 115200;
    UART_init_config.mode = UART_MODE_TX_RX;
    UART_init_config.stopbits = UART_STOPBITS_2;
    uart_init(UART0, &UART_init_config);

    app_init();
    manual_setting_test();
    return 0;
}

void __attribute__((weak, noreturn)) __main(void) {
    while (1) {
        asm volatile ("wfi");
    }
}