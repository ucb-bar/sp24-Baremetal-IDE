#ifndef __HARDWARE_CONF_H_
#define __HARDWARE_CONF_H_

#ifdef __cplusplus
extern "C" {
#endif


// #include "bearly24.h"

/**
 * This section controls which peripheral device is included in the application program.
 * To save the memory space, the unused peripheral device can be commented out.
 */

// Hardware Enable //

/**
 * Enables the Quantized Transformer V_DOTPROD function.
 */
// #define ENABLE_QT_DOTPROD

/**
 * Enables the DMA MatVec functionality to speed up matmul computations through DMA0.
 */
// #define ENABLE_DMA_MATVEC
#define DMA_NUM_ROWS 16
#define DMA_NUM_COLS 64

/**
 * Enables compatibility and acceleration using the DSP'24 Saturn-V vector-enabled cores.
 */
#define ENABLE_SATURNV_VEC


// Accelerator library inclusions //
#include "chip_config.h"
#include "riscv.h"

#ifdef ENABLE_QT_DOTPROD
#include "hal_qt.h"
#endif

#ifdef ENABLE_DMA_MATVEC
#include "hal_dma.h"
#endif

#ifdef ENABLE_SATURNV_VEC

#endif

#include "ll_pll.h"

#ifdef __cplusplus
}
#endif

#endif  /* __HAL_CONF_H */
