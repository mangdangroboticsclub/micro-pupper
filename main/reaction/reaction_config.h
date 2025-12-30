/**
 * @file reaction_config.h
 * @brief Configuration for user interaction reactions
 * 
 * Defines thresholds and parameters for detecting user pushes
 * and triggering appropriate animations.
 */

#ifndef REACTION_CONFIG_H
#define REACTION_CONFIG_H

#include <stdbool.h>
#include "qmi8658a.h"

// ═══════════════════════════════════════════════════════
// DELTA-BASED PUSH DETECTION
// ═══════════════════════════════════════════════════════
// Detects sudden changes in acceleration (impulse) rather than
// sustained thresholds. A push causes a large delta between readings.

// Minimum delta (change) in accel_x to detect a push (m/s² per sample)
// This is the difference between current and previous reading
#define REACTION_DELTA_THRESHOLD        7.0f    // m/s² change

// Minimum absolute acceleration to consider (filters out noise deltas)
#define REACTION_MIN_ACCEL              3.0f    // m/s²

// Minimum time between reactions (debounce)
#define REACTION_COOLDOWN_MS            2000    // 2 seconds

// Animation timing adjustment (added to each keyframe delay)
// Increase if animation is too fast, decrease if too slow
#define REACTION_TIMING_OFFSET_MS       100     // milliseconds

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the reaction system
 */
void reaction_init(void);

/**
 * @brief Process IMU data and trigger reactions if thresholds met
 * @param data Current IMU sensor data
 */
void reaction_process_imu(const qmi8658a_data_t *data);

#endif // REACTION_CONFIG_H
