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
#include "chip_config.h"
#include "riscv.h"
#include "hal_dma.h"

#ifdef __cplusplus
}
#endif

#endif  /* __HAL_CONF_H */