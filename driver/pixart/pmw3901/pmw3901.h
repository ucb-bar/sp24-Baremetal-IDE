#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include "spi.h"

// Default SPI Chip Select ID (adjust if needed, e.g., 0, 1, 2)
#define PMW3901_CS_ID           0 

// Registers
#define PMW3901_REG_PRODUCT_ID  0x00
#define PMW3901_REG_REVISION_ID 0x01
#define PMW3901_REG_MOTION      0x02
#define PMW3901_REG_DELTA_X_L   0x03
#define PMW3901_REG_DELTA_X_H   0x04
#define PMW3901_REG_DELTA_Y_L   0x05
#define PMW3901_REG_DELTA_Y_H   0x06
#define PMW3901_REG_SQUAL       0x07

// Chip ID
#define PMW3901_PRODUCT_ID_VAL  0x49

/**
 * @brief Structure to hold motion data
 */
typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal; // Surface Quality (0-255, higher is better)
    uint8_t motion_occurred; // 1 if motion, 0 otherwise
} PMW3901_Data;

/**
 * @brief Initialize the PMW3901 Sensor
 * @param spi Pointer to SPI peripheral (e.g., SPI0)
 * @return 0 on success, -1 on failure
 */
int pmw3901_init(SPI_Type *spi);

/**
 * @brief Read Optical Flow Data
 * @param spi Pointer to SPI peripheral
 * @param data Pointer to structure to store X/Y delta
 * @return 0 on success, -1 on failure
 */
int pmw3901_read_motion(SPI_Type *spi, PMW3901_Data *data);

#endif // PMW3901_H