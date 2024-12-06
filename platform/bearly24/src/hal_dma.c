#include "hal_dma.h"

DMA_Status get_status(DMA_Type* DMAX) {
  if (READ_BITS(DMAX->STATUS, DMA_BADMODE_MSK))
    return DMA_BADMODE;
  else if (READ_BITS(DMAX->STATUS, DMA_CNTERR_MSK))
    return DMA_CNTERR;
  else if (READ_BITS(DMAX->STATUS, DMA_DENYR_MSK))
    return DMA_DENYR;
  else if (READ_BITS(DMAX->STATUS, DMA_CORRUPTR_MSK))
    return DMA_CORRUPTR;
  else if (READ_BITS(DMAX->STATUS, DMA_DENYW_MSK))
    return DMA_DENYW;
  return DMA_OK;
}

void dma_init_memcpy(DMA_Type* DMAX, void* src, void* dst, uint64_t src_stride, uint32_t count) {
  while (dma_operation_inprogress_and_not_error(DMAX));
  
  DMAX->SRC_ADDR = (uint64_t) src;
  DMAX->DEST_ADDR = (uint64_t) dst;
  DMAX->SRCSTRIDE = src_stride;
  DMAX->MODE = MODE_COPY;
  DMAX->COUNT = count;
}

void dma_init_MAC(DMA_Type* DMAX, void* src, int8_t* operand, uint64_t src_stride, uint32_t count) {
  while (dma_operation_inprogress_and_not_error(DMAX));

  // int64_t* op = (int64_t *) operand;
  // int64_t* reg = (int64_t *) DMAX->OPERAND_REG;
  // for (size_t i = 0; i < 8; i++)  // 8*8byte-wide elements in operand = 64 total cols
  //   reg[i] = op[i];
  
  // 8*8byte-wide elements in operand = 64 total cols
  memcpy(DMAX->OPERAND_REG, operand, 64);
  DMAX->SRC_ADDR = (uint64_t) src;
  DMAX->SRCSTRIDE = src_stride;
  DMAX->MODE = MODE_MAC;
  DMAX->COUNT = count;

}

DMA_Status dma_await_result(DMA_Type* DMAX) {
  while (dma_operation_inprogress_and_not_error(DMAX));
  if (dma_operation_complete(DMAX))
    return DMA_OK;
  else
    return get_status(DMAX);
}

DMA_Status dma_get_MAC_result(DMA_Type* DMAX, int16_t* dst, uint32_t count) {
  while (dma_operation_inprogress_and_not_error(DMAX));
  if (count > 32)
    count = 32;
  
  if (dma_operation_complete(DMAX)){
    // Copy count * 2^4 (count * 16 byte vals)
    memcpy(dst, DMAX->DEST_REG, count << 4);
    // int64_t* op = (int64_t *) operand;
    // int64_t* reg = (int64_t *) DMAX->OPERAND_REG;
    // for (size_t i = 0; i < count; i++)
    //   dst[i] = ((int16_t *) DMAX->DEST_REG)[i];
    return DMA_OK;
  }
  else {
    for (size_t i = 0; i < count; i++)
        dst[i] = -1;
    return get_status(DMAX);
  }
}