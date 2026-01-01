/**
 * @file walk_forward_reaction.h
 * @brief Walk forward animation with pressure/IMU detection
 */

#ifndef WALK_FORWARD_REACTION_H
#define WALK_FORWARD_REACTION_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Play walk forward animation for fixed number of cycles
 * @param cycles Number of complete walk cycles
 */
void walk_forward_play(uint8_t cycles);

/**
 * @brief Walk forward continuously while push conditions are met
 * 
 * Walks while front pressure + IMU forward tilt/push detected
 * 
 * @param get_imu_data Callback to get IMU data (pitch degrees, forward accel G)
 *                     Can be NULL to ignore IMU
 */
void walk_forward_play_while_pushed(void (*get_imu_data)(float *pitch, float *accel_y));

/**
 * @brief Walk forward while front legs have pressure (simple version)
 * 
 * No IMU needed - just walks while front servos detect pressure
 */
void walk_forward_play_while_front_pressure(void);

/**
 * @brief Check if forward walk conditions are met
 * 
 * @param pitch_deg Current pitch in degrees
 * @param accel_y Forward acceleration in G
 * @return true if should walk forward
 */
bool walk_forward_check_conditions(float pitch_deg, float accel_y);

/**
 * @brief Check only pressure-based forward condition
 * @return true if front legs have pressure
 */
bool walk_forward_check_pressure_only(void);

#endif // WALK_FORWARD_REACTION_H