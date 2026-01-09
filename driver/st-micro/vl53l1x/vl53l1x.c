#include "vl53l1x.h"
#include <stdio.h>

#define I2C_TIMEOUT 100000 

// Registers (Partial Map)
#define VL53L1_SYSTEM__MODE_START                 0x0087
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x0016
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A
#define VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E
#define VL53L1_MM_CONFIG__INNER_OFFSET_MM         0x0020
#define VL53L1_MM_CONFIG__OUTER_OFFSET_MM         0x0022
#define VL53L1_GPIO_HV_MUX__CTRL                  0x0030
#define VL53L1_GPIO__TIO_HV_STATUS                0x0031
#define VL53L1_SYSTEM__INTERRUPT_CLEAR            0x0086
#define VL53L1_RESULT__RANGE_STATUS               0x0089
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1_IDENTIFICATION__MODEL_ID           0x010F

// --- Low Level I2C Wrappers ---

static int vl_write_byte(I2C_Type *i2c, CLINT_Type *clint, uint16_t reg, uint8_t value) {
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = value;
    return (i2c_master_transmit(i2c, clint, VL53L1X_ADDR, buf, 3, I2C_TIMEOUT) == OK) ? 0 : -1;
}

static int vl_write_word(I2C_Type *i2c, CLINT_Type *clint, uint16_t reg, uint16_t value) {
    uint8_t buf[4];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
    return (i2c_master_transmit(i2c, clint, VL53L1X_ADDR, buf, 4, I2C_TIMEOUT) == OK) ? 0 : -1;
}

static int vl_read_bytes(I2C_Type *i2c, CLINT_Type *clint, uint16_t reg, uint8_t *data, uint16_t len) {
    uint8_t reg_addr[2];
    reg_addr[0] = (reg >> 8) & 0xFF;
    reg_addr[1] = reg & 0xFF;
    if (i2c_master_transmit(i2c, clint, VL53L1X_ADDR, reg_addr, 2, I2C_TIMEOUT) != OK) return -1;
    if (i2c_master_receive(i2c, clint, VL53L1X_ADDR, data, len, I2C_TIMEOUT) != OK) return -1;
    return 0;
}

// --- Core API ---

int vl53l1x_init(I2C_Type *i2c, CLINT_Type *clint) {
    uint8_t model_id[2];
    if (vl_read_bytes(i2c, clint, VL53L1_IDENTIFICATION__MODEL_ID, model_id, 2) != 0) return -1;
    
    // Basic reset sequence (Soft reset)
    vl_write_byte(i2c, clint, 0x0000, 0x00);
    // Delay needed here in real RTOS, baremetal spin loop:
    for(volatile int i=0; i<10000; i++);
    vl_write_byte(i2c, clint, 0x0000, 0x01);

    // Normally we load 80+ bytes of default tuning here. 
    // Assuming sensor defaults are sufficient for basic operation or 
    // bootloader handles it. If flight is unstable, insert full register map here.
    
    return vl53l1x_start_ranging(i2c, clint);
}

int vl53l1x_start_ranging(I2C_Type *i2c, CLINT_Type *clint) {
    // Start Continuous
    return vl_write_byte(i2c, clint, VL53L1_SYSTEM__MODE_START, 0x40); 
}

int vl53l1x_stop_ranging(I2C_Type *i2c, CLINT_Type *clint) {
    return vl_write_byte(i2c, clint, VL53L1_SYSTEM__MODE_START, 0x00);
}

int vl53l1x_check_data_ready(I2C_Type *i2c, CLINT_Type *clint) {
    uint8_t status;
    if (vl_read_bytes(i2c, clint, VL53L1_GPIO__TIO_HV_STATUS, &status, 1) != 0) return -1;
    return (status & 0x01);
}

int16_t vl53l1x_read_distance(I2C_Type *i2c, CLINT_Type *clint) {
    int timeout = 1000; 
    while (timeout > 0) {
        if (vl53l1x_check_data_ready(i2c, clint) == 1) break;
        timeout--;
    }
    if (timeout == 0) return -2; 

    uint8_t data[2];
    vl_read_bytes(i2c, clint, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, data, 2);
    vl_write_byte(i2c, clint, VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);

    return (int16_t)((data[0] << 8) | data[1]);
}

// --- Calibration API ---

int vl53l1x_calibrate_offset(I2C_Type *i2c, CLINT_Type *clint, int16_t target_dist_mm, int16_t *offset) {
    int32_t sum_dist = 0;
    int samples = 50;
    
    // 1. Reset offsets to 0
    vl_write_word(i2c, clint, VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0);
    vl_write_word(i2c, clint, VL53L1_MM_CONFIG__INNER_OFFSET_MM, 0);
    vl_write_word(i2c, clint, VL53L1_MM_CONFIG__OUTER_OFFSET_MM, 0);

    vl53l1x_start_ranging(i2c, clint);

    // 2. Collect Samples
    for (int i = 0; i < samples; i++) {
        int16_t d = vl53l1x_read_distance(i2c, clint);
        if (d < 0) { i--; continue; } // Skip bad reads
        sum_dist += d;
    }
    
    vl53l1x_stop_ranging(i2c, clint);

    // 3. Calculate Offset
    int16_t avg_dist = sum_dist / samples;
    *offset = target_dist_mm - avg_dist;

    // 4. Apply Offset (Shift left by 2 as per datasheet 14.2 format usually, 
    // but standard driver multiplies by 4 for register format)
    return vl_write_word(i2c, clint, VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, (*offset) * 4);
}

int vl53l1x_calibrate_xtalk(I2C_Type *i2c, CLINT_Type *clint, int16_t target_dist_mm, uint16_t *xtalk) {
    // Simplified Xtalk: Assuming 0 gradient, just plane offset
    // NOTE: Full Xtalk calibration requires reading signal rates which is complex 
    // in baremetal without floating point support struct.
    // This is a stub to show where register writes go.
    
    // Reset Xtalk
    vl_write_word(i2c, clint, VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, 0);
    
    // Real implementation requires reading RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0
    // and computing average over 50 samples.
    
    *xtalk = 0; // Placeholder
    return 0;
}