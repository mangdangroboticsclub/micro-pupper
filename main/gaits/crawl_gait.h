/**
 * @file crawl_gait.h
 * @brief Crawl Gait Algorithm for Quadruped Robot
 * 
 * Implements a wave-like crawl gait where legs move in a ripple pattern:
 *   Phase 1: Back-Left (4) swings forward
 *   Phase 2: Back-Right (3) swings forward  
 *   Phase 3: Front-Left (2) swings forward
 *   Phase 4: Front-Right (1) swings forward
 * 
 * Legs move in a wave from back to front on each side.
 * Provides good traction and stability at low speeds.
 * More fluid motion than creep, but slower than trot.
 */

#ifndef CRAWL_GAIT_H
#define CRAWL_GAIT_H

#include "gait_common.h"

// ═══════════════════════════════════════════════════════
// CRAWL GAIT CONFIGURATION
// ═══════════════════════════════════════════════════════

typedef gait_config_t crawl_gait_config_t;

/**
 * @brief Default configuration for crawl gait
 * Uses unified angles - reversal is handled by dog_config
 */
#define CRAWL_GAIT_DEFAULT_CONFIG() {       \
    .stance_angle_fr = DOG_STANCE_FRONT,    \
    .stance_angle_fl = DOG_STANCE_FRONT,    \
    .stance_angle_br = DOG_STANCE_BACK,     \
    .stance_angle_bl = DOG_STANCE_BACK,     \
    .swing_amplitude = DOG_SWING_AMPLITUDE, \
    .step_duration_ms = 250,                \
    .servo_speed = DOG_SPEED_MEDIUM         \
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the crawl gait controller
 * @param config Pointer to gait configuration, or NULL for defaults
 * @return true if all servos responded successfully
 */
bool crawl_gait_init(const crawl_gait_config_t *config);

/**
 * @brief Update the gait configuration
 */
void crawl_gait_set_config(const crawl_gait_config_t *config);

/**
 * @brief Get the current gait configuration
 */
void crawl_gait_get_config(crawl_gait_config_t *config);

// ═══════════════════════════════════════════════════════
// GAIT CONTROL
// ═══════════════════════════════════════════════════════

/**
 * @brief Start the crawl gait
 * @param direction Initial walking direction
 * @return true if task was created successfully
 */
bool crawl_gait_start(gait_direction_t direction);

/**
 * @brief Stop the crawl gait and return to stance
 */
void crawl_gait_stop(void);

/**
 * @brief Set the walking direction
 */
void crawl_gait_set_direction(gait_direction_t direction);

/**
 * @brief Get the current walking direction
 */
gait_direction_t crawl_gait_get_direction(void);

/**
 * @brief Check if the gait is currently running
 */
bool crawl_gait_is_running(void);

/**
 * @brief Move all servos to stance position
 */
void crawl_gait_goto_stance(void);

/**
 * @brief Execute a single crawl step
 */
void crawl_gait_step(gait_direction_t direction);

#endif // CRAWL_GAIT_H
