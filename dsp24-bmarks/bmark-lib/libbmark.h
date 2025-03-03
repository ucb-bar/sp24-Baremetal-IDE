#ifndef __LIBBMARK_H
#define __LIBBMARK_H

#include "uart.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

extern long chip_freq;
extern long chip_mtime_freq;
extern UART_Type *debug_uart;

typedef struct {
    void* payload_buffer;
    char testid;
} test_info;

test_info init_test(UART_Type *UARTx);

void start_roi();

void end_roi();

void xmit_payload_packet(void* data, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __LIBBMARK_H */