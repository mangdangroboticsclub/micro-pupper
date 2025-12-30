/**
 * @file trot_gait.h
 * @brief Trot Gait Walking Algorithm for Quadruped Robot
 * 
 * Implements a trot gait where diagonal leg pairs move together:
 * - Phase A: Front-Right (1) + Back-Left (4) move forward
 * - Phase B: Front-Left (2) + Back-Right (3) move forward
 * 
 * Servo ID Layout:
 *   Front: [1] Right  [2] Left
 *   Back:  [3] Right  [4] Left
 * 
 * Positive angle direction (clockwise) means:
 *   - Right legs move backward
 *   - Left legs move forward
 */

#ifndef TROT_GAIT_H
#define TROT_GAIT_H

#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════
// SERVO ID DEFINITIONS
// ═══════════════════════════════════════════════════════

#define SERVO_FRONT_RIGHT   1
#define SERVO_FRONT_LEFT    2
#define SERVO_BACK_RIGHT    3
#define SERVO_BACK_LEFT     4

// ═══════════════════════════════════════════════════════
// GAIT CONFIGURATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Configuration parameters for the trot gait
 */
typedef struct {
    float stance_angle_fr;   ///< Front-right neutral angle (degrees)
    float stance_angle_fl;   ///< Front-left neutral angle (degrees)
    float stance_angle_br;   ///< Back-right neutral angle (degrees)
    float stance_angle_bl;   ///< Back-left neutral angle (degrees)
    float swing_amplitude;   ///< Maximum angle deviation from stance (degrees)
    uint16_t step_duration_ms;  ///< Duration of each step phase (milliseconds)
    uint16_t servo_speed;    ///< Servo movement speed (0-4095)
} trot_gait_config_t;

/**
 * @brief Direction of walking
 */
typedef enum {
    TROT_DIRECTION_FORWARD,
    TROT_DIRECTION_BACKWARD,
    TROT_DIRECTION_STOP
} trot_direction_t;

/**
 * @brief Default configuration for the trot gait
 * Stance angles: FR=270°, FL=90°, BR=90°, BL=270°
 */
#define TROT_GAIT_DEFAULT_CONFIG() { \
    .stance_angle_fr = 270.0f,       \
    .stance_angle_fl = 90.0f,        \
    .stance_angle_br = 90.0f,        \
    .stance_angle_bl = 270.0f,       \
    .swing_amplitude = 30.0f,        \
    .step_duration_ms = 200,         \
    .servo_speed = 1500              \
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the trot gait controller
 * 
 * Must be called after servo driver initialization.
 * Sets all servos to stance position.
 * 
 * @param config Pointer to gait configuration, or NULL for defaults
 * @return true if all servos responded successfully
 */
bool trot_gait_init(const trot_gait_config_t *config);

/**
 * @brief Update the gait configuration
 * 
 * @param config Pointer to new configuration
 */
void trot_gait_set_config(const trot_gait_config_t *config);

/**
 * @brief Get the current gait configuration
 * 
 * @param config Pointer to store configuration
 */
void trot_gait_get_config(trot_gait_config_t *config);

// ═══════════════════════════════════════════════════════
// GAIT CONTROL
// ═══════════════════════════════════════════════════════

/**
 * @brief Start the trot gait walking task
 * 
 * Creates a FreeRTOS task that continuously executes the gait.
 * 
 * @param direction Initial walking direction
 * @return true if task was created successfully
 */
bool trot_gait_start(trot_direction_t direction);

/**
 * @brief Stop the trot gait and return to stance
 */
void trot_gait_stop(void);

/**
 * @brief Set the walking direction
 * 
 * Can be called while gait is running.
 * 
 * @param direction New walking direction
 */
void trot_gait_set_direction(trot_direction_t direction);

/**
 * @brief Get the current walking direction
 * 
 * @return Current direction
 */
trot_direction_t trot_gait_get_direction(void);

/**
 * @brief Check if the gait is currently running
 * 
 * @return true if gait task is active
 */
bool trot_gait_is_running(void);

// ═══════════════════════════════════════════════════════
// DIRECT CONTROL (for manual stepping)
// ═══════════════════════════════════════════════════════

/**
 * @brief Move all servos to stance position
 */
void trot_gait_goto_stance(void);

/**
 * @brief Execute a single trot step
 * 
 * Useful for testing or manual control without the background task.
 * 
 * @param direction Direction for this step
 */
void trot_gait_step(trot_direction_t direction);

#endif // TROT_GAIT_H
