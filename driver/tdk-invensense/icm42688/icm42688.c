#include "icm42688.h"
#include <stdio.h>

#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 10000 // Arbitrary timeout cycles
#endif

static int icm_write_reg(I2C_Type *i2c, uint8_t reg, uint8_t value, CLINT_Type *clint) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    
    Status status = i2c_master_transmit(i2c, clint, ICM_42688_ADDR, buf, 2, I2C_TIMEOUT);
    return (status == OK) ? 0 : -1;
}

static int icm_read_regs(I2C_Type *i2c, uint8_t start_reg, uint8_t *buf, uint32_t len, CLINT_Type *clint) {
    // 1. Write Register Address
    Status status = i2c_master_transmit(i2c, clint, ICM_42688_ADDR, &start_reg, 1, I2C_TIMEOUT);
    if (status != OK) return -1;

    // 2. Read Data
    status = i2c_master_receive(i2c, clint, ICM_42688_ADDR, buf, len, I2C_TIMEOUT);
    return (status == OK) ? 0 : -1;
}

int icm42688_init(I2C_Type *i2c, CLINT_Type *clint) {
    uint8_t who_am_i = 0;
    
    // 1. Check Connection
    if (icm_read_regs(i2c, ICM_REG_WHO_AM_I, &who_am_i, 1, clint) != 0) {
        printf("ICM Read WHO_AM_I Failed.\n");
        return -1;
    }
    
    if (who_am_i != ICM_WHO_AM_I_VAL) {
        printf("ICM ID Mismatch: 0x%02X\n", who_am_i);
        return -2;
    }

    // 2. Reset Device (Optional, skipping to avoid complex delays for now)
    
    // 3. Enable Sensors (Gyro & Accel in Low Noise Mode)
    // Bits 3:2 = Gyro Mode (11 = Low Noise)
    // Bits 1:0 = Accel Mode (11 = Low Noise)
    uint8_t pwr_mgmt = ICM_PWR_GYRO_MODE_LN | ICM_PWR_ACCEL_MODE_LN;
    if (icm_write_reg(i2c, ICM_REG_PWR_MGMT0, pwr_mgmt, clint) != 0) {
        return -3;
    }

    // 4. Configure Full Scale (Optional - using defaults)
    // Accel Config0 (0x50): Default is +/- 16g usually or needs setup
    // Gyro Config0 (0x4F): Default is +/- 2000dps usually
    
    return 0; // Success
}

int icm42688_read_all(I2C_Type *i2c, ICM42688_Data *data, CLINT_Type *clint) {
    uint8_t raw_data[14]; // Temp (2) + Accel (6) + Gyro (6)
    
    // Start reading from TEMP_DATA1 (0x1D) to capture everything in one burst
    // Map: Temp(2) -> Accel(6) -> Gyro(6)
    // Note: Check specific datasheet map. 
    // Standard: Temp(1D,1E), Accel(1F-24), Gyro(25-2A)
    
    if (icm_read_regs(i2c, ICM_REG_TEMP_DATA1, raw_data, 14, clint) != 0) {
        return -1;
    }

    // Parse Data (Big Endian)
    data->temp    = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->accel_x = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->accel_y = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    data->accel_z = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    data->gyro_x  = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_y  = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    data->gyro_z  = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return 0;
}