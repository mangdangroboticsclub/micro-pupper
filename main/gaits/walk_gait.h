/**
 * @file walk_gait.h
 * @brief Horse Walk Gait Algorithm for Quadruped Robot
 * 
 * Implements a 4-beat walk gait where each leg moves independently in sequence:
 *   Phase 1: Front-Left (2)
 *   Phase 2: Back-Right (3)
 *   Phase 3: Front-Right (1)
 *   Phase 4: Back-Left (4)
 * 
 * This is the most stable but slowest gait, as 3 legs are always on the ground.
 * Natural horse walking pattern - smooth and energy efficient.
 */

#ifndef WALK_GAIT_H
#define WALK_GAIT_H

#include "gait_common.h"

// ═══════════════════════════════════════════════════════
// WALK GAIT CONFIGURATION
// ═══════════════════════════════════════════════════════

typedef gait_config_t walk_gait_config_t;

/**
 * @brief Default configuration for walk gait
 */
#define WALK_GAIT_DEFAULT_CONFIG() { \
    .stance_angle_fr = 270.0f,       \
    .stance_angle_fl = 90.0f,        \
    .stance_angle_br = 90.0f,        \
    .stance_angle_bl = 270.0f,       \
    .swing_amplitude = 25.0f,        \
    .step_duration_ms = 300,         \
    .servo_speed = 800               \
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the walk gait controller
 * @param config Pointer to gait configuration, or NULL for defaults
 * @return true if all servos responded successfully
 */
bool walk_gait_init(const walk_gait_config_t *config);

/**
 * @brief Update the gait configuration
 */
void walk_gait_set_config(const walk_gait_config_t *config);

/**
 * @brief Get the current gait configuration
 */
void walk_gait_get_config(walk_gait_config_t *config);

// ═══════════════════════════════════════════════════════
// GAIT CONTROL
// ═══════════════════════════════════════════════════════

/**
 * @brief Start the walk gait
 * @param direction Initial walking direction
 * @return true if task was created successfully
 */
bool walk_gait_start(gait_direction_t direction);

/**
 * @brief Stop the walk gait and return to stance
 */
void walk_gait_stop(void);

/**
 * @brief Set the walking direction
 */
void walk_gait_set_direction(gait_direction_t direction);

/**
 * @brief Get the current walking direction
 */
gait_direction_t walk_gait_get_direction(void);

/**
 * @brief Check if the gait is currently running
 */
bool walk_gait_is_running(void);

/**
 * @brief Move all servos to stance position
 */
void walk_gait_goto_stance(void);

/**
 * @brief Execute a single walk step
 */
void walk_gait_step(gait_direction_t direction);

#endif // WALK_GAIT_H
