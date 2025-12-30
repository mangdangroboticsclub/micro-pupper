/**
 * @file gyro_balance.h
 * @brief Gyroscope-based balance/stabilization system
 * 
 * Keeps robot legs facing ground using gyro Y axis feedback.
 * Can be toggled on/off by rotating the robot on the X axis.
 */

#ifndef GYRO_BALANCE_H
#define GYRO_BALANCE_H

#include <stdbool.h>
#include "qmi8658a.h"

// ═══════════════════════════════════════════════════════
// BALANCE CONFIGURATION
// ═══════════════════════════════════════════════════════

// Enable/disable gyro stabilization at startup
#define GYRO_BALANCE_ENABLED_DEFAULT        false

// Maximum leg angle adjustment from neutral (degrees)
#define GYRO_BALANCE_MAX_CORRECTION         90.0f

// Gyro deadzone - ignore small rotations (degrees/second)
#define GYRO_BALANCE_DEADZONE               0.5f

// Proportional gain for gyro response
#define GYRO_BALANCE_GAIN                   1.6f

// Low-pass filter coefficient (0.0-1.0)
#define GYRO_BALANCE_SMOOTHING              0.3f

// Update rate for stabilization (ms)
#define GYRO_BALANCE_UPDATE_INTERVAL_MS     50

// ═══════════════════════════════════════════════════════
// DYNAMIC SPEED CONFIGURATION
// ═══════════════════════════════════════════════════════

#define GYRO_BALANCE_SPEED_MIN              150
#define GYRO_BALANCE_SPEED_MAX              2000
#define GYRO_BALANCE_SPEED_THRESHOLD        10.0f
#define GYRO_BALANCE_SPEED_CURVE            1.2f

// ═══════════════════════════════════════════════════════
// TOGGLE GESTURE CONFIGURATION (X-axis rotation)
// ═══════════════════════════════════════════════════════

// Minimum gyro X rate to detect rotation gesture (dps)
// From logs: idle is ~6 dps, gesture peaks at 250-350 dps
#define GYRO_BALANCE_TOGGLE_THRESHOLD       150.0f

// Time window to complete the gesture (ms)
// Must rotate one way, then the other within this time
#define GYRO_BALANCE_TOGGLE_WINDOW_MS       1000

// Cooldown between toggles (ms)
#define GYRO_BALANCE_TOGGLE_COOLDOWN_MS     1500

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the gyro balance system
 */
void gyro_balance_init(void);

/**
 * @brief Process IMU data for balance and toggle detection
 * @param data Current IMU sensor data
 */
void gyro_balance_process(const qmi8658a_data_t *data);

/**
 * @brief Enable or disable gyro balance
 * @param enable true to enable, false to disable
 */
void gyro_balance_enable(bool enable);

/**
 * @brief Check if gyro balance is currently enabled
 * @return true if enabled
 */
bool gyro_balance_is_enabled(void);

#endif // GYRO_BALANCE_H
