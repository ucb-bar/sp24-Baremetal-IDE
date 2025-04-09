// I2S_utils.h
#ifndef I2S_UTILS_h
#define I2S_UTILS_h

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_DMA.h"
#include  "hal_mmio.h"
#include "chip_config.h"

#define I2S_BASE                             0x10042000U
#define I2S_CONFIG(channel)                  I2S_BASE + 0x2 * channel
#define I2S_STATUS(channel)                  I2S_BASE + 0x08 + channel

#define I2S_WATERMARK_RX_L(channel)          I2S_BASE + 0x102 + 0x4 * channel
#define I2S_WATERMARK_RX_R(channel)          I2S_BASE + 0x103 + 0x4 * channel
#define I2S_WATERMARK_TX_L(channel)          I2S_BASE + 0x100 + 0x4 * channel
#define I2S_WATERMARK_TX_R(channel)          I2S_BASE + 0x101 + 0x4 * channel

#define I2S_TX_WATERMARK(channel)            I2S_BASE + 0x0C + channel
#define I2S_RX_WATERMARK(channel)            I2S_BASE + 0x10 + channel

#define I2S_CLKDIV(channel)                  I2S_BASE + 0x14 + 0x2 * channel

#define I2S_TX_L(channel)                    I2S_BASE + 0x20 + 0x10 * channel
#define I2S_TX_R(channel)                    I2S_BASE + 0x28 + 0x10 * channel
#define I2S_RX_L(channel)                    I2S_BASE + 0x60 + 0x10 * channel
#define I2S_RX_R(channel)                    I2S_BASE + 0x68 + 0x10 * channel

//#define I2S_FP_MODE                 I2S_BASE + 0xB8

typedef struct I2S_PARAMS_struct {
    int channel;
    int tx_en;
    int rx_en;
    int bitdepth_tx;
    int bitdepth_rx;
    int clkgen;
    int dacen;
    int ws_len;
    int clkdiv;
    int tx_fp;
    int rx_fp;
    int tx_force_left;
    int rx_force_left;
} I2S_PARAMS;

/* Don't use this one. My eyes hurt trying to figure out which param corresponds to which. */
void set_I2S_params_manual(int channel, int tx_en, int rx_en, int bitdepth_tx, int bitdepth_rx, int clkgen, int dacen, int ws_len);

void set_I2S_clkdiv(int channel, int clkdiv);

void set_I2S_params(I2S_PARAMS* params);

void set_I2S_watermark(int channel, int watermark_tx, int watermark_rx);

void set_I2S_en(int channel, int tx_en, int rx_en);

uint64_t read_I2S_rx(int channel, int left);

void write_I2S_tx(int channel, int left, uint64_t data);

uint64_t write_I2S_tx_DMA(int channel, int dma_num, int length, uint64_t* read_addr, int left, int poll);

uint64_t read_I2S_rx_DMA(int channel, int dma_num, int length, uint64_t* write_addr, int left, int poll);

void set_I2S_fp(int channel, int tx_fp, int rx_fp);

void set_I2S_force_left(int channel, int tx_force_left, int rx_force_left);


#ifdef __cplusplus
}
#endif

#endif