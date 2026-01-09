#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>
#include "i2c.h"
#include "clint.h"

// Default I2C Address (7-bit)
#define VL53L1X_ADDR            0x29

// Distance Modes
#define VL53L1X_DISTANCEMODE_SHORT  1
#define VL53L1X_DISTANCEMODE_LONG   2

/**
 * @brief Initialize the VL53L1X ToF Sensor
 * Loads default tuning settings and boots the device.
 */
int vl53l1x_init(I2C_Type *i2c, CLINT_Type *clint);

/**
 * @brief Set Distance Mode
 * @param mode VL53L1X_DISTANCEMODE_SHORT (1.3m max) or LONG (4m max)
 */
int vl53l1x_set_distance_mode(I2C_Type *i2c, CLINT_Type *clint, uint16_t mode);

/**
 * @brief Set Measurement Timing Budget
 * @param budget_ms Budget in milliseconds (e.g., 20ms, 33ms, 140ms)
 * Lower budget = faster, Higher budget = more accurate/longer range.
 */
int vl53l1x_set_timing_budget(I2C_Type *i2c, CLINT_Type *clint, uint16_t budget_ms);

/**
 * @brief Start Continuous Ranging
 * @param inter_measurement_ms Time between measurements in ms (must be >= budget)
 */
int vl53l1x_start_ranging(I2C_Type *i2c, CLINT_Type *clint);

/**
 * @brief Stop Ranging
 */
int vl53l1x_stop_ranging(I2C_Type *i2c, CLINT_Type *clint);

/**
 * @brief Check if New Data is Ready
 * @return 1 if ready, 0 if not, negative on error
 */
int vl53l1x_check_data_ready(I2C_Type *i2c, CLINT_Type *clint);

/**
 * @brief Read Distance
 * Blocking call (waits for data ready).
 * @return Distance in mm, or -1 on error.
 */
int16_t vl53l1x_read_distance(I2C_Type *i2c, CLINT_Type *clint);

/**
 * @brief Calibrate Offset
 * Place a target (grey 17%) at exactly 140mm.
 * @param target_dist_mm True distance to target (e.g., 140)
 * @return 0 on success.
 */
int vl53l1x_calibrate_offset(I2C_Type *i2c, CLINT_Type *clint, int16_t target_dist_mm, int16_t *offset);

/**
 * @brief Calibrate Crosstalk (Xtalk)
 * Place a target at a known distance.
 */
int vl53l1x_calibrate_xtalk(I2C_Type *i2c, CLINT_Type *clint, int16_t target_dist_mm, uint16_t *xtalk);

#endif // VL53L1X_H