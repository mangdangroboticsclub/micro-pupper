/**
 * @file qmi8658a.h
 * @brief QMI8658A IMU Driver - Portable Component
 * 
 * Generic driver for QMI8658A 6-axis IMU sensor.
 * All configuration (pins, ranges, etc.) is passed at runtime.
 * No project-specific hardcoded values.
 */

#ifndef QMI8658A_H
#define QMI8658A_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

// ═══════════════════════════════════════════════════════
// QMI8658A CONFIGURATION
// ═══════════════════════════════════════════════════════

#define QMI8658A_I2C_ADDR_DEFAULT   0x6A        // Default I2C address
#define QMI8658A_I2C_ADDR_ALT       0x6B        // Alternate I2C address
#define QMI8658A_CHIP_ID            0x05        // Expected CHIP_ID register value

// ═══════════════════════════════════════════════════════
// QMI8658A REGISTERS
// ═══════════════════════════════════════════════════════

#define QMI8658A_REG_CHIP_ID        0x00
#define QMI8658A_REG_REVISION_ID    0x01
#define QMI8658A_REG_CTRL1          0x02        // SPI Interface and Sensor Enable
#define QMI8658A_REG_CTRL2          0x03        // Accelerometer Settings
#define QMI8658A_REG_CTRL3          0x04        // Gyroscope Settings
#define QMI8658A_REG_CTRL5          0x06        // LPF Settings
#define QMI8658A_REG_CTRL7          0x08        // Enable Sensors
#define QMI8658A_REG_CTRL8          0x09        // Motion Detection
#define QMI8658A_REG_CTRL9          0x0A        // Host Commands

#define QMI8658A_REG_STATUSINT      0x2D        // Sensor Data Availability
#define QMI8658A_REG_STATUS0        0x2E        // Output Data Overrun & Availability
#define QMI8658A_REG_STATUS1        0x2F        // Motion, Pedometer, Tap, etc.

#define QMI8658A_REG_TEMP_L         0x33        // Temperature low byte
#define QMI8658A_REG_TEMP_H         0x34        // Temperature high byte
#define QMI8658A_REG_AX_L           0x35        // Accelerometer X low byte
#define QMI8658A_REG_AX_H           0x36        // Accelerometer X high byte
#define QMI8658A_REG_AY_L           0x37        // Accelerometer Y low byte
#define QMI8658A_REG_AY_H           0x38        // Accelerometer Y high byte
#define QMI8658A_REG_AZ_L           0x39        // Accelerometer Z low byte
#define QMI8658A_REG_AZ_H           0x3A        // Accelerometer Z high byte
#define QMI8658A_REG_GX_L           0x3B        // Gyroscope X low byte
#define QMI8658A_REG_GX_H           0x3C        // Gyroscope X high byte
#define QMI8658A_REG_GY_L           0x3D        // Gyroscope Y low byte
#define QMI8658A_REG_GY_H           0x3E        // Gyroscope Y high byte
#define QMI8658A_REG_GZ_L           0x3F        // Gyroscope Z low byte
#define QMI8658A_REG_GZ_H           0x40        // Gyroscope Z high byte

// ═══════════════════════════════════════════════════════
// ACCELEROMETER CONFIGURATION
// ═══════════════════════════════════════════════════════

// Accelerometer Full Scale (bits 6:4 of CTRL2)
typedef enum {
    QMI8658A_ACCEL_RANGE_2G  = (0x00 << 4),     // ±2g
    QMI8658A_ACCEL_RANGE_4G  = (0x01 << 4),     // ±4g
    QMI8658A_ACCEL_RANGE_8G  = (0x02 << 4),     // ±8g
    QMI8658A_ACCEL_RANGE_16G = (0x03 << 4),     // ±16g
} qmi8658a_accel_range_t;

// Accelerometer ODR (bits 3:0 of CTRL2)
typedef enum {
    QMI8658A_ACCEL_ODR_8000   = 0x00,           // 7174.4 Hz
    QMI8658A_ACCEL_ODR_4000   = 0x01,           // 3587.2 Hz
    QMI8658A_ACCEL_ODR_2000   = 0x02,           // 1793.6 Hz
    QMI8658A_ACCEL_ODR_1000   = 0x03,           // 896.8 Hz
    QMI8658A_ACCEL_ODR_500    = 0x04,           // 448.4 Hz
    QMI8658A_ACCEL_ODR_250    = 0x05,           // 224.2 Hz
    QMI8658A_ACCEL_ODR_125    = 0x06,           // 112.1 Hz
    QMI8658A_ACCEL_ODR_62_5   = 0x07,           // 56.05 Hz
    QMI8658A_ACCEL_ODR_31     = 0x08,           // 28.025 Hz
    QMI8658A_ACCEL_ODR_128_LP = 0x0C,           // 128 Hz (Low Power)
    QMI8658A_ACCEL_ODR_21_LP  = 0x0D,           // 21 Hz (Low Power)
    QMI8658A_ACCEL_ODR_11_LP  = 0x0E,           // 11 Hz (Low Power)
    QMI8658A_ACCEL_ODR_3_LP   = 0x0F,           // 3 Hz (Low Power)
} qmi8658a_accel_odr_t;

// ═══════════════════════════════════════════════════════
// GYROSCOPE CONFIGURATION
// ═══════════════════════════════════════════════════════

// Gyroscope Full Scale (bits 6:4 of CTRL3)
typedef enum {
    QMI8658A_GYRO_RANGE_16   = (0x00 << 4),     // ±16 dps
    QMI8658A_GYRO_RANGE_32   = (0x01 << 4),     // ±32 dps
    QMI8658A_GYRO_RANGE_64   = (0x02 << 4),     // ±64 dps
    QMI8658A_GYRO_RANGE_128  = (0x03 << 4),     // ±128 dps
    QMI8658A_GYRO_RANGE_256  = (0x04 << 4),     // ±256 dps
    QMI8658A_GYRO_RANGE_512  = (0x05 << 4),     // ±512 dps
    QMI8658A_GYRO_RANGE_1024 = (0x06 << 4),     // ±1024 dps
    QMI8658A_GYRO_RANGE_2048 = (0x07 << 4),     // ±2048 dps
} qmi8658a_gyro_range_t;

// Gyroscope ODR (bits 3:0 of CTRL3)
typedef enum {
    QMI8658A_GYRO_ODR_8000 = 0x00,              // 7174.4 Hz
    QMI8658A_GYRO_ODR_4000 = 0x01,              // 3587.2 Hz
    QMI8658A_GYRO_ODR_2000 = 0x02,              // 1793.6 Hz
    QMI8658A_GYRO_ODR_1000 = 0x03,              // 896.8 Hz
    QMI8658A_GYRO_ODR_500  = 0x04,              // 448.4 Hz
    QMI8658A_GYRO_ODR_250  = 0x05,              // 224.2 Hz
    QMI8658A_GYRO_ODR_125  = 0x06,              // 112.1 Hz
    QMI8658A_GYRO_ODR_62_5 = 0x07,              // 56.05 Hz
    QMI8658A_GYRO_ODR_31   = 0x08,              // 28.025 Hz
} qmi8658a_gyro_odr_t;

// ═══════════════════════════════════════════════════════
// DATA STRUCTURES
// ═══════════════════════════════════════════════════════

/**
 * @brief Raw 16-bit sensor readings
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} qmi8658a_raw_data_t;

/**
 * @brief Scaled IMU sensor readings
 */
typedef struct {
    float accel_x;          // m/s²
    float accel_y;          // m/s²
    float accel_z;          // m/s²
    float gyro_x;           // °/s
    float gyro_y;           // °/s
    float gyro_z;           // °/s
    float accel_magnitude;  // Total acceleration
} qmi8658a_data_t;

/**
 * @brief IMU configuration structure (passed by user)
 */
typedef struct {
    // I2C configuration
    i2c_port_t i2c_num;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq_hz;
    uint8_t i2c_addr;
    
    // Sensor configuration
    qmi8658a_accel_range_t accel_range;
    qmi8658a_accel_odr_t accel_odr;
    qmi8658a_gyro_range_t gyro_range;
    qmi8658a_gyro_odr_t gyro_odr;
} qmi8658a_config_t;

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the QMI8658A IMU
 * @param config Pointer to configuration (required, no defaults)
 * @return true if initialization successful
 */
bool qmi8658a_init(const qmi8658a_config_t *config);

/**
 * @brief Check if IMU is responsive
 * @return true if IMU responds with correct CHIP_ID
 */
bool qmi8658a_check_device(void);

/**
 * @brief Read raw sensor data
 * @param out_data Pointer to raw data structure
 * @return true if read successful
 */
bool qmi8658a_read_raw(qmi8658a_raw_data_t *out_data);

/**
 * @brief Read scaled sensor data
 * @param out_data Pointer to scaled data structure
 * @return true if read successful
 */
bool qmi8658a_read(qmi8658a_data_t *out_data);

/**
 * @brief Print diagnostic information
 */
void qmi8658a_debug_status(void);

/**
 * @brief Get the current configuration
 * @return Pointer to the active configuration
 */
const qmi8658a_config_t* qmi8658a_get_config(void);

#endif // QMI8658A_H
