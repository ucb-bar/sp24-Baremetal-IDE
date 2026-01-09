#ifndef ICM_42688_H
#define ICM_42688_H

#include <stdint.h>
#include "i2c.h"
#include "clint.h"

// I2C Address
#define ICM_42688_ADDR          0x69 

// Registers
#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_DRIVE_CONFIG    0x13
#define ICM_REG_INT_CONFIG      0x14
#define ICM_REG_FIFO_CONFIG     0x16
#define ICM_REG_TEMP_DATA1      0x1D
#define ICM_REG_ACCEL_DATA_X1   0x1F
#define ICM_REG_GYRO_DATA_X1    0x25
#define ICM_REG_INT_STATUS      0x2D
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_WHO_AM_I        0x75

// Configuration Bits
#define ICM_WHO_AM_I_VAL        0x47
#define ICM_PWR_TEMP_ON         (0 << 5)
#define ICM_PWR_GYRO_MODE_LN    (3 << 2) // Low Noise Mode
#define ICM_PWR_ACCEL_MODE_LN   (3 << 0) // Low Noise Mode

// Data Structure
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} ICM42688_Data;

/**
 * @brief Initialize the ICM-42688 Sensor
 * @param i2c Pointer to I2C peripheral (e.g., I2C0)
 * @return 0 on success, -1 on failure
 */
int icm42688_init(I2C_Type *i2c, CLINT_Type *clint);

/**
 * @brief Read Accelerometer and Gyroscope data
 * @param i2c Pointer to I2C peripheral
 * @param data Pointer to data structure to fill
 * @return 0 on success, -1 on failure
 */
int icm42688_read_all(I2C_Type *i2c, ICM42688_Data *data, CLINT_Type *clint);

#endif // ICM_42688_H