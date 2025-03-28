#include "libbmark.h"
#include "pll.h"
#include "gpio.h"
#include "chip_config.h"
#include <stdbool.h>

#define BMARK_GPIO_PIN GPIO_PIN_1

long chip_freq;
long chip_mtime_freq;

bool first_iteration = true;
const char header_ack = 0x06;

UART_Type *debug_uart;

test_info init_test(UART_Type *UARTx) {
  int packet_size;
  char header_start;
  test_info t;

  debug_uart = UARTx;

  GPIO_InitType gpio_init_config;
  gpio_init_config.mode = GPIO_MODE_OUTPUT;
  gpio_init_config.pull = GPIO_PULL_NONE;
  gpio_init_config.drive_strength = GPIO_DS_STRONG;

  gpio_init(GPIOC, &gpio_init_config, BMARK_GPIO_PIN);
  gpio_write_pin(GPIOC, BMARK_GPIO_PIN, 1);
  

  if (first_iteration) {
    CLOCK_SELECTOR->UNCORE = 0;
    CLOCK_SELECTOR->TILE0 = 0;
    CLOCK_SELECTOR->TILE1 = 0;
    CLOCK_SELECTOR->TILE2 = 0;
    CLOCK_SELECTOR->TILE3 = 0;
    CLOCK_SELECTOR->CLKTAP = 0; //Ensure we are using the non PLL clock

    UART_InitType UART_init_config;
    UART_init_config.baudrate = 115200;
    UART_init_config.mode = UART_MODE_TX_RX;
    UART_init_config.stopbits = UART_STOPBITS_2;
    uart_init(debug_uart, &UART_init_config);

    // Wake up all the secondary HARTs
    for (int i = 1; i < 4; i++) {
      CLINT->MSIP[i] = 1;
    }
    first_iteration = false;
  }

  // Enable UART Receive and Transmit without setting baudrate

  while (1) {
    // Waits for SOH (Start of Header) 0x01 or UART ENQ (Enquiry) 0x05
    uart_receive(debug_uart, &header_start, 1, 0);
    
    if (header_start == 0x05) {
      // If ENQ (Enquiry), send back Acknowledge (0x06) to say we are valid.
      uart_transmit(debug_uart, &header_ack, 1, 0);
    } else if (header_start == 0x01) {
      // If SOH (Start of Header), prepare to read a header.
      break;
    }
  }

  uart_receive(debug_uart, &packet_size, 4, 0);
  uart_receive(debug_uart, &chip_freq, 8, 0);
  uart_receive(debug_uart, &(t.testid), 1, 0);

  chip_mtime_freq = chip_freq / 1000;
  if (packet_size > 8) {
    t.payload_buffer = malloc(packet_size);
    uart_receive(debug_uart, t.payload_buffer, packet_size, 0);
  } else if (packet_size > 0) {
    uart_receive(debug_uart, &t.payload, packet_size, 0);
    t.payload_buffer = NULL;
  } else {
    t.payload_buffer = NULL;
  }

  int clkmult = chip_freq / 50000000;
  int uart_divisor = (chip_freq / 115200) - 1;

  CLOCK_SELECTOR->UNCORE = 0;
  CLOCK_SELECTOR->TILE0 = 0;
  CLOCK_SELECTOR->TILE1 = 0;
  CLOCK_SELECTOR->TILE2 = 0;
  CLOCK_SELECTOR->TILE3 = 0;
  CLOCK_SELECTOR->CLKTAP = 0;
  PLL->PLLEN = 0;
  PLL->MDIV_RATIO = 1;
  PLL->RATIO = clkmult;
  PLL->FRACTION = 0;
  PLL->ZDIV0_RATIO = 1;
  PLL->ZDIV1_RATIO = 1;
  PLL->LDO_ENABLE = 1;
  PLL->PLLEN = 1;
  PLL->POWERGOOD_VNN = 1;
  PLL->PLLFWEN_B = 1;
  CLOCK_SELECTOR->UNCORE = 1;
  CLOCK_SELECTOR->TILE0 = 1;
  CLOCK_SELECTOR->TILE1 = 1;
  CLOCK_SELECTOR->TILE2 = 1;
  CLOCK_SELECTOR->TILE3 = 1;
  CLOCK_SELECTOR->CLKTAP = 1;

  debug_uart->DIV = uart_divisor;

  return t;
}

void start_roi() {
  char start_char = 7;
  uart_transmit(debug_uart, &start_char, 1, 0);
  gpio_write_pin(GPIOC, BMARK_GPIO_PIN, 0);
}

void end_roi() {
  gpio_write_pin(GPIOC, BMARK_GPIO_PIN, 1);
  char end_char = 23;
  uart_transmit(debug_uart, &end_char, 1, 0);
}

void xmit_payload_packet(void* data, size_t size) {
  uart_transmit(debug_uart, &size, 4, 0);
  if (size != 0 && data != NULL) {
    uart_transmit(debug_uart, data, size, 0);
  }
}

void clean_test(test_info t) {
  if (t.payload_buffer != NULL) {
    free(t.payload_buffer);
  }
}
