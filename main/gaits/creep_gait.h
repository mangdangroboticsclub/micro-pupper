/**
 * @file creep_gait.h
 * @brief Creep Gait Algorithm for Quadruped Robot
 * 
 * Implements a slow, stable 4-beat creep gait with body shift for stability:
 *   1. Shift body weight away from the leg about to move
 *   2. Lift and move one leg forward
 *   3. Place leg down
 *   4. Repeat for next leg in sequence: FL -> BR -> FR -> BL
 * 
 * This is the most stable gait - maximizes static stability margin.
 * Used for rough terrain or when carrying heavy loads.
 * Always maintains a stable tripod (3 legs on ground).
 */

#ifndef CREEP_GAIT_H
#define CREEP_GAIT_H

#include "gait_common.h"

// ═══════════════════════════════════════════════════════
// CREEP GAIT CONFIGURATION
// ═══════════════════════════════════════════════════════

typedef gait_config_t creep_gait_config_t;

/**
 * @brief Default configuration for creep gait
 * Slower and more deliberate than walk
 */
#define CREEP_GAIT_DEFAULT_CONFIG() { \
    .stance_angle_fr = 270.0f,        \
    .stance_angle_fl = 90.0f,         \
    .stance_angle_br = 90.0f,         \
    .stance_angle_bl = 270.0f,        \
    .swing_amplitude = 20.0f,         \
    .step_duration_ms = 400,          \
    .servo_speed = 600                \
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the creep gait controller
 * @param config Pointer to gait configuration, or NULL for defaults
 * @return true if all servos responded successfully
 */
bool creep_gait_init(const creep_gait_config_t *config);

/**
 * @brief Update the gait configuration
 */
void creep_gait_set_config(const creep_gait_config_t *config);

/**
 * @brief Get the current gait configuration
 */
void creep_gait_get_config(creep_gait_config_t *config);

// ═══════════════════════════════════════════════════════
// GAIT CONTROL
// ═══════════════════════════════════════════════════════

/**
 * @brief Start the creep gait
 * @param direction Initial walking direction
 * @return true if task was created successfully
 */
bool creep_gait_start(gait_direction_t direction);

/**
 * @brief Stop the creep gait and return to stance
 */
void creep_gait_stop(void);

/**
 * @brief Set the walking direction
 */
void creep_gait_set_direction(gait_direction_t direction);

/**
 * @brief Get the current walking direction
 */
gait_direction_t creep_gait_get_direction(void);

/**
 * @brief Check if the gait is currently running
 */
bool creep_gait_is_running(void);

/**
 * @brief Move all servos to stance position
 */
void creep_gait_goto_stance(void);

/**
 * @brief Execute a single creep step
 */
void creep_gait_step(gait_direction_t direction);

#endif // CREEP_GAIT_H
