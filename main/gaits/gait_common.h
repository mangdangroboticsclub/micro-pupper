/**
 * @file gait_common.h
 * @brief Common definitions for all gait algorithms
 * 
 * Shared types and constants used across trot, walk, creep, and crawl gaits.
 * 
 * Uses dog_config.h for servo IDs and hardware configuration.
 * Angle reversal for right-side servos is handled automatically by dog_config.
 */

#ifndef GAIT_COMMON_H
#define GAIT_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "dog_config.h"

// ═══════════════════════════════════════════════════════
// SERVO ID DEFINITIONS (from dog_config.h)
// ═══════════════════════════════════════════════════════

#define SERVO_FRONT_RIGHT   DOG_SERVO_FR
#define SERVO_FRONT_LEFT    DOG_SERVO_FL
#define SERVO_BACK_RIGHT    DOG_SERVO_BR
#define SERVO_BACK_LEFT     DOG_SERVO_BL

// ═══════════════════════════════════════════════════════
// COMMON TYPES
// ═══════════════════════════════════════════════════════

/**
 * @brief Direction of walking (common to all gaits)
 */
typedef enum {
    GAIT_DIRECTION_FORWARD,
    GAIT_DIRECTION_BACKWARD,
    GAIT_DIRECTION_TURN_LEFT,
    GAIT_DIRECTION_TURN_RIGHT,
    GAIT_DIRECTION_STOP
} gait_direction_t;

/**
 * @brief Base configuration shared by all gaits
 */
typedef struct {
    float stance_angle_fr;      ///< Front-right neutral angle (degrees)
    float stance_angle_fl;      ///< Front-left neutral angle (degrees)
    float stance_angle_br;      ///< Back-right neutral angle (degrees)
    float stance_angle_bl;      ///< Back-left neutral angle (degrees)
    float swing_amplitude;      ///< Maximum angle deviation from stance (degrees)
    uint16_t step_duration_ms;  ///< Duration of each step phase (milliseconds)
    uint16_t servo_speed;       ///< Servo movement speed (0-4095)
} gait_config_t;

/**
 * @brief Default stance configuration
 * Uses unified angles from dog_config - reversal is automatic
 */
#define GAIT_DEFAULT_STANCE() {                     \
    .stance_angle_fr = DOG_STANCE_FRONT,            \
    .stance_angle_fl = DOG_STANCE_FRONT,            \
    .stance_angle_br = DOG_STANCE_BACK,             \
    .stance_angle_bl = DOG_STANCE_BACK,             \
    .swing_amplitude = DOG_SWING_AMPLITUDE,         \
    .step_duration_ms = 250,                        \
    .servo_speed = DOG_SPEED_FAST                   \
}

#endif // GAIT_COMMON_H
