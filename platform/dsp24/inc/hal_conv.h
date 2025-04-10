// CONV_utils.h
#ifndef CONV_UTILS_h
#define CONV_UTILS_h


#ifdef __cplusplus
extern "C" {
#endif

#include  "hal_mmio.h"
#include "chip_config.h"


// Addresses for Convolution
#define CONV_BASE 0x08800000U
#define CONV_OUTPUT_ADDR     CONV_BASE + 0x20
#define CONV_KERNEL_ADDR     CONV_BASE + 0x40
#define CONV_START_ADDR      CONV_BASE + 0x6C
#define CONV_LENGTH_ADDR     CONV_BASE + 0x78
#define CONV_DILATION_ADDR   CONV_BASE + 0x7C
#define CONV_ISFLOAT_ADDR    CONV_BASE + 0x8E

#define READ_CHECK_ADDR   CONV_BASE + 0x8D

typedef struct __attribute__((packed)) {
    __IO uint64_t DATA_ENQUEUE;
    uint32_t RESERVED0[6];
    __IO uint64_t RESULT_DEQUEUE;
    uint32_t RESERVED1[6];
    __IO uint64_t KERNEL;
    uint8_t RESERVED2[34];
    __IO uint8_t STATUS;
    uint8_t RESERVED3[1];
    __IO uint8_t START;
    __IO uint8_t CLEAR;
    uint8_t RESERVED4[10];
    __IO uint32_t LENGTH;
    __IO uint16_t DILATION;
    uint8_t RESERVED5[14];
    __IO uint8_t ENQUEUE_REQ;
    __IO uint8_t DEQUEUE_REQ;
    __IO uint8_t USE_FLOAT;
    __IO uint8_t RESET;
} ConvAccel_Type;

int set_conv_params(int len, uint16_t in_dilation, uint16_t* in_kernel);

void write_conv_dma(int dma_num, int length, uint64_t* data);

void read_conv_dma(int dma_num, int length, uint64_t* write_addr);

void read_conv_dma_p(int dma_num, int length, uint64_t* write_addr);

void start_conv();

#ifdef __cplusplus
}
#endif

#endif