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
#define REACTION_DELTA_THRESHOLD        9000.0f    // m/s² change

// Minimum absolute acceleration to consider (filters out noise deltas)
#define REACTION_MIN_ACCEL              1.0f    // m/s²

// Minimum time between reactions (debounce)
#define REACTION_COOLDOWN_MS            1600    // 1.6 seconds

// Animation timing adjustment (added to each keyframe delay)
// Increase if animation is too fast, decrease if too slow
#define REACTION_TIMING_OFFSET_MS       0     // milliseconds

// ═══════════════════════════════════════════════════════
// PRESSURE THRESHOLDS
// ═══════════════════════════════════════════════════════

// Back legs: crouch threshold
#define REACTION_BACK_PRESSURE_THRESHOLD    0.08f    // degrees

// Front legs: walk trigger threshold
#define REACTION_FRONT_PRESSURE_THRESHOLD   0.15f    // degrees

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

/**
 * @brief Pressure detection and reaction task (100Hz)
 */
void reaction_pressure_task(void *arg);

/**
 * @brief Notify reaction system that walk has completed
 *        (starts settle timer to avoid false pressure triggers)
 */
void reaction_notify_walk_complete(void);

#endif // REACTION_CONFIG_H