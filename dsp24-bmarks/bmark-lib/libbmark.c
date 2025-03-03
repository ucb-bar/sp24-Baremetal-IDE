#include "libbmark.h"

long chip_freq;
long chip_mtime_freq;
UART_Type *debug_uart;

test_info init_test(UART_Type *UARTx) {
  int packet_size;
  test_info t;

  debug_uart = UARTx;
  // Enable UART Receive and Transmit without setting baudrate
  SET_BITS(debug_uart->TXCTRL, UART_TXCTRL_TXEN_MSK);
  SET_BITS(debug_uart->RXCTRL, UART_RXCTRL_RXEN_MSK);

  uart_receive(debug_uart, &packet_size, 4, 0);
  uart_receive(debug_uart, &chip_freq, 8, 0);
  uart_receive(debug_uart, &(t.testid), 1, 0);

  chip_mtime_freq = chip_freq / 1000;
  if (packet_size > 8) {
    t.payload_buffer = malloc(packet_size);
    uart_receive(debug_uart, t.payload_buffer, packet_size, 0);
  } else if (packet_size > 0) {
    uart_receive(debug_uart, &t.payload, packet_size, 0);
  } else {
    t.payload_buffer = NULL;
  }
  return t;
}

void start_roi() {
  char start_char = 7;
  uart_transmit(debug_uart, &start_char, 1, 0);
}

void end_roi() {
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
