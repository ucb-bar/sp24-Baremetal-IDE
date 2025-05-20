#include <stdint.h>
#include "UART_TSI.h"
#include "chip_config.h"

#define  CMD_READ  0x00
#define  CMD_WRITE  0x01

#define UART_TSI_RX GPIO_NUM_15
#define UART_TSI_TX GPIO_NUM_11

#define UART UART_NUM_1

void uart_tsi_init(void) {
  printf("Start UART TSI Setup\n");

  UART_InitType UART1_init_config;
  // UART1_init_config.baudrate = 921600;
  UART1_init_config.baudrate = 115200;
  UART1_init_config.mode = UART_MODE_TX_RX;
  UART1_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART1, &UART1_init_config);

  const int uart_buffer_size = (1024 * 2);

  printf("UART TSI Setup done\n");
}

int uart_tsi_read(uint64_t address) {
  uint8_t req[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  memcpy(req + 4, &address, 8);
  uart_transmit(UART1, req, 20, 100000000000000);

  uint8_t ret[4];
  uart_receive(UART1, ret, 4, 100000000000000);

  int received_value;
  memcpy(&received_value, ret, sizeof(float));

  return received_value;
}

void uart_tsi_write(uint64_t address, uint32_t data) {
  uint8_t write[24] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  memcpy(write + 4, &address, 8);
  memcpy(write + 20, &data, 4);
  uart_transmit(UART1, write, 24, 100000000000000);
}