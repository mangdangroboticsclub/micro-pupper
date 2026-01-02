/**
 * @file pressure_balance.h
 * @brief Pressure-based crouch/stance control for back legs
 * 
 * When pressure is detected on BL or BR servos, the robot crouches down.
 * When pressure is released, it smoothly returns to stance.
 */

#ifndef PRESSURE_BALANCE_H
#define PRESSURE_BALANCE_H

#include <stdbool.h>
#include <stdint.h>

// ═══════════════════════════════════════════════════════
// PRESSURE BALANCE CONFIGURATION
// ═══════════════════════════════════════════════════════

// Enable/disable at startup
#define PRESSURE_BALANCE_ENABLED_DEFAULT    true

// Pressure threshold (position error in degrees)
#define PRESSURE_BALANCE_THRESHOLD          0.25f

// How much to move down per update cycle when pressure detected (degrees)
#define PRESSURE_BALANCE_DOWN_STEP          2.0f

// Maximum crouch angle from stance (degrees)
#define PRESSURE_BALANCE_MAX_CROUCH         40.0f

// Return to stance speed when released
#define PRESSURE_BALANCE_RETURN_STEP        3.0f

// Update interval (ms)
#define PRESSURE_BALANCE_UPDATE_MS          30

// Servo speed for movements
#define PRESSURE_BALANCE_SPEED_MIN          100
#define PRESSURE_BALANCE_SPEED_MAX          800

// Smoothing factor (0.0 = very smooth, 1.0 = instant)
#define PRESSURE_BALANCE_SMOOTHING          0.3f

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize pressure balance system
 */
void pressure_balance_init(void);

/**
 * @brief Update pressure balance - call this frequently
 * 
 * Reads BL and BR servo positions, detects pressure,
 * and adjusts all leg angles accordingly.
 */
void pressure_balance_update(void);

/**
 * @brief Enable or disable pressure balance
 * @param enable true to enable, false to disable
 */
void pressure_balance_enable(bool enable);

/**
 * @brief Check if pressure balance is enabled
 * @return true if enabled
 */
bool pressure_balance_is_enabled(void);

/**
 * @brief Check if robot is currently crouched due to pressure
 * @return true if crouched
 */
bool pressure_balance_is_crouched(void);

/**
 * @brief Get current crouch offset (degrees from stance)
 * @return Current offset in degrees
 */
float pressure_balance_get_offset(void);

#endif // PRESSURE_BALANCE_H