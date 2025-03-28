/**
 * @file ll_dma.h
 * @author Jasmine Angle / angle@berkeley.edu
 * @brief 
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __LL_DMA_H
#define __LL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_ERR_POS                        (0U)
#define DMA_ERR_MSK                        (0x1U << DMA_ERR_POS)
#define DMA_COMPL_POS                      (1U)
#define DMA_COMPL_MSK                      (0x1U << DMA_COMPL_POS)
#define DMA_INPROG_POS                     (2U)
#define DMA_INPROG_MSK                     (0x1U << DMA_INPROG_POS)
#define DMA_BADMODE_POS                    (3U)
#define DMA_BADMODE_MSK                    (0x1U << DMA_BADMODE_POS)
#define DMA_CNTERR_POS                     (4U)
#define DMA_CNTERR_MSK                     (0x1U << DMA_CNTERR_POS)
#define DMA_DENYR_POS                      (5U)
#define DMA_DENYR_MSK                      (0x1U << DMA_DENYR_POS)
#define DMA_CORRUPTR_POS                   (6U)
#define DMA_CORRUPTR_MSK                   (0x1U << DMA_CORRUPTR_POS)
#define DMA_DENYW_POS                      (7U)
#define DMA_DENYW_MSK                      (0x1U << DMA_DENYW_POS)

typedef struct {
  __I uint64_t STATUS;    // 0x00
  __IO uint8_t MODE;            // 0x08
  __IO void* SRC_ADDR;          // 0x10
  __IO void* DEST_ADDR;         // 0x18
  __IO uint64_t SRCSTRIDE;      // 0x20
  __IO uint32_t COUNT;          // 0x28
  uint64_t RESERVED0[2];        // 0x30
  __IO int8_t OPERAND_REG[64]; // 0x40  
  __I int16_t DEST_REG[32];    // 0x80
  // uint64_t RESERVED1[24];
} DMA_Type;

#ifdef __cplusplus
}
#endif

#endif /* __LL_DMA_H */
