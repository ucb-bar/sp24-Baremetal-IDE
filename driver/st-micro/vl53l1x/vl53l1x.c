#include "vl53l1x.h"
#include <stdio.h>

#define I2C_TIMEOUT 100000 

// Registers (Partial Map)
#define VL53L1_SOFT_RESET                           0x0000
#define VL53L1_SYSTEM__MODE_START                 0x0087
#define VL53L1_FIRMWARE__SYSTEM_STATUS              0x00E5
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

// --- Critical Default Tuning Data (From ST ULD API) ---
static const uint8_t vl53l1x_default_configuration[] = {
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x01, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (0x01: active high) */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity */
    0x00, /* 0x32 : not documented */
    0x02, /* 0x33 : not documented */
    0x08, /* 0x34 : not documented */
    0x00, /* 0x35 : not documented */
    0x08, /* 0x36 : not documented */
    0x10, /* 0x37 : not documented */
    0x01, /* 0x38 : not documented */
    0x01, /* 0x39 : not documented */
    0x00, /* 0x3a : not documented */
    0x00, /* 0x3b : not documented */
    0x00, /* 0x3c : not documented */
    0x00, /* 0x3d : not documented */
    0xff, /* 0x3e : not documented */
    0x00, /* 0x3f : not documented */
    0x0F, /* 0x40 : not documented */
    0x00, /* 0x41 : not documented */
    0x00, /* 0x42 : not documented */
    0x00, /* 0x43 : not documented */
    0x00, /* 0x44 : not documented */
    0x00, /* 0x45 : not documented */
    0x20, /* 0x46 : interrupt configuration 0->level 1->pulse */
    0x0b, /* 0x47 : not documented */
    0x00, /* 0x48 : not documented */
    0x00, /* 0x49 : not documented */
    0x02, /* 0x4a : not documented */
    0x0a, /* 0x4b : not documented */
    0x21, /* 0x4c : not documented */
    0x00, /* 0x4d : not documented */
    0x00, /* 0x4e : not documented */
    0x05, /* 0x4f : not documented */
    0x00, /* 0x50 : not documented */
    0x00, /* 0x51 : not documented */
    0x00, /* 0x52 : not documented */
    0x00, /* 0x53 : not documented */
    0xc8, /* 0x54 : not documented */
    0x00, /* 0x55 : not documented */
    0x00, /* 0x56 : not documented */
    0x38, /* 0x57 : not documented */
    0xff, /* 0x58 : not documented */
    0x01, /* 0x59 : not documented */
    0x00, /* 0x5a : not documented */
    0x08, /* 0x5b : not documented */
    0x00, /* 0x5c : not documented */
    0x00, /* 0x5d : not documented */
    0x01, /* 0x5e : not documented */
    0xcc, /* 0x5f : not documented */
    0x0f, /* 0x60 : not documented */
    0x01, /* 0x61 : not documented */
    0xf1, /* 0x62 : not documented */
    0x0d, /* 0x63 : not documented */
    0x01, /* 0x64 : Sigma threshold MSB (default 14 mm x 4) */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (default 0.4 mcps) */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not documented */
    0xb8, /* 0x69 : not documented */
    0x00, /* 0x6a : not documented */
    0x00, /* 0x6b : not documented */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not documented */
    0x00, /* 0x71 : not documented */
    0x00, /* 0x72 : distance mode (0=short, 1=long) */
    0x00, /* 0x73 : not documented */
    0x00, /* 0x74 : not documented */
    0x00, /* 0x75 : not documented */
    0x00, /* 0x76 : not documented */
    0x01, /* 0x77 : not documented */
    0x0f, /* 0x78 : not documented */
    0x0d, /* 0x79 : not documented */
    0x0e, /* 0x7a : not documented */
    0x0e, /* 0x7b : not documented */
    0x00, /* 0x7c : not documented */
    0x00, /* 0x7d : not documented */
    0x02, /* 0x7e : not documented */
    0xc7, /* 0x7f : ROI center */
    0xff, /* 0x80 : XY ROI (For 16x16, use 0x0F) */
    0x9B, /* 0x81 : ROI center */
    0x00, /* 0x82 : not documented */
    0x00, /* 0x83 : not documented */
    0x00, /* 0x84 : not documented */
    0x01, /* 0x85 : not documented */
    0x00, /* 0x86 : clear interrupt, 0x01=clear */
    0x00  /* 0x87 : ranging, 0x00=stop, 0x40=start */
};

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

// --- Internal Helper ---
static void vl_delay_ms(int ms) {
    // Crude blocking delay for baremetal
    // Adjust loop count based on your clock speed
    for(volatile int i=0; i< (ms * 10000); i++); 
}

// --- Core API ---

int vl53l1x_init(I2C_Type *i2c, CLINT_Type *clint) {
    uint8_t model_id[2];
    if (vl_read_bytes(i2c, clint, VL53L1_IDENTIFICATION__MODEL_ID, model_id, 2) != 0) return -1;
    
    // Basic reset sequence (Soft reset)
    vl_write_byte(i2c, clint, VL53L1_SOFT_RESET, 0x00);
    vl_delay_ms(1);
    vl_write_byte(i2c, clint, VL53L1_SOFT_RESET, 0x01);
    vl_delay_ms(1);

    // Normally we load 80+ bytes of default tuning here. 
    // Assuming sensor defaults are sufficient for basic operation or 
    // bootloader handles it. If flight is unstable, insert full register map here.
    int timeout = 100;
    uint8_t status = 0;
    while(timeout > 0) {
        vl_read_bytes(i2c, clint, VL53L1_FIRMWARE__SYSTEM_STATUS, &status, 1);
        // 0x03 means boot complete and ready
        if ((status & 0x03) == 0x03) break; 
        vl_delay_ms(1);
        timeout--;
    }

    if (timeout == 0) printf("WARNING: ToF Boot Timeout (Status 0x%02X)\n", status);
    
    // 4. Load Default Configuration (CRITICAL STEP)
    // Writing 87 bytes starting from register 0x2D
    for (uint16_t i = 0; i < sizeof(vl53l1x_default_configuration); i++) {
        vl_write_byte(i2c, clint, 0x2D + i, vl53l1x_default_configuration[i]);
    }

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
        vl_delay_ms(1);
        timeout--;
    }
    if (timeout == 0) return -2; 

    // 2. Check Range Status
    // 0: Valid, 1: Sigma Fail, 2: Signal Fail, 4: Out of bounds, 7: Wrap around
    uint8_t range_status;
    vl_read_bytes(i2c, clint, VL53L1_RESULT__RANGE_STATUS, &range_status, 1);

    uint8_t data[2];
    vl_read_bytes(i2c, clint, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, data, 2);
    int16_t distance = (int16_t)((data[0] << 8) | data[1]);

    // 3. Clear Data Ready Interrupt
    vl_write_byte(i2c, clint, VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);

    // Filter invalid ranges based on status
    // Status 9 is "Range Valid" in some docs, but usually 0 is valid.
    // If status is 2 (Signal Fail) or 4 (Out of bounds), usually dist is garbage or 0.
    if (range_status != 0 && range_status != 9) {
        // Optional debug print to see why it fails
        // printf("TOF Warn: Status %d, Dist %d\n", range_status, distance);
        if (range_status == 1 || range_status == 2) return -3; // Signal failure
        if (range_status == 4) return -4; // Out of bounds
    }

    return distance;
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