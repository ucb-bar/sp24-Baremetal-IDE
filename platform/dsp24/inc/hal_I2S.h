// I2S_utils.h
#ifndef __I2S_H
#define __I2S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chip_config.h"

#define I2S_STATUS                  I2S_BASE + 0x08

#define I2S_WATERMARK_RX_L          I2S_BASE + 0x102
#define I2S_WATERMARK_RX_R          I2S_BASE + 0x103
#define I2S_WATERMARK_TX_L          I2S_BASE + 0x100
#define I2S_WATERMARK_TX_R          I2S_BASE + 0x101

#define I2S_TX_WATERMARK            I2S_BASE + 0x0C
#define I2S_RX_WATERMARK            I2S_BASE + 0x10

#define I2S_CLKDIV                  I2S_BASE + 0x14

#define I2S_TX_L                    I2S_BASE + 0x20
#define I2S_TX_R                    I2S_BASE + 0x28
#define I2S_RX_L                    I2S_BASE + 0x60
#define I2S_RX_R                    I2S_BASE + 0x68

//#define I2S_FP_MODE                 I2S_BASE + 0xB8

typedef struct __attribute__((packed)) {
  __IO uint16_t CONFIG[4];         /* 0x0-0x7*/
  __I  uint8_t  STATUS[4];         /* 0x8-0xB*/
  __IO uint8_t  TX_WATERMARK[4];   /* 0xC-0xF*/
  __IO uint8_t  RX_WATERMARK[4];   /* 0x10-0x13*/
  __IO uint16_t CLK_DIV[4];        /* 0x14-0x1B*/
  uint32_t RESERVED0;              /* 0x1C-0x1F*/
  __O uint64_t TX_LENQUEUE0;       /* 0x20-0x27*/
  __O uint64_t TX_RENQUEUE0;       /* 0x28-0x2F*/
  __O uint64_t TX_LENQUEUE1;       /* 0x30-0x37*/
  __O uint64_t TX_RENQUEUE1;       /* 0x38-0x3F*/
  __O uint64_t TX_LENQUEUE2;       /* 0x40-0x47*/
  __O uint64_t TX_RENQUEUE2;       /* 0x48-0x4F*/
  __O uint64_t TX_LENQUEUE3;       /* 0x50-0x57*/
  __O uint64_t TX_RENQUEUE3;       /* 0x58-0x5F*/
  __I uint64_t RX_LDEQUEUE0;       /* 0x60-0x67*/
  __I uint64_t RX_RDEQUEUE0;       /* 0x68-0x6F*/
  __I uint64_t RX_LDEQUEUE1;       /* 0x70-0x77*/
  __I uint64_t RX_RDEQUEUE1;       /* 0x78-0x7F*/
  __I uint64_t RX_LDEQUEUE2;       /* 0x80-0x87*/
  __I uint64_t RX_RDEQUEUE2;       /* 0x88-0x8F*/
  __I uint64_t RX_LDEQUEUE3;       /* 0x90-0x97*/
  __I uint64_t RX_RDEQUEUE3;       /* 0x98-0x9F*/
} I2S_Type;

void set_I2S_params(int channel, int tx_en, int rx_en, int bitdepth_tx, int bitdepth_rx, int clkgen, int dacen, int ws_len);

void set_I2S_clkdiv(int channel, int clkdiv);

void set_I2S_watermark(int channel, int watermark_tx, int watermark_rx);

void set_I2S_en(int channel, int tx_en, int rx_en);

uint64_t read_I2S_tx(int channel, int left);

void write_I2S_rx(int channel, int left, uint64_t data);

uint64_t write_I2S_tx_DMA(int channel, int dma_num, int length, uint64_t* read_addr, int left, int poll);

uint64_t read_I2S_rx_DMA(int channel, int dma_num, int length, uint64_t* write_addr, int left, int poll);

void set_I2S_fp(int channel, int tx_fp, int rx_fp);

void set_I2S_force_left(int channel, int tx_force_left, int rx_force_left);


#ifdef __cplusplus
}
#endif

#endif