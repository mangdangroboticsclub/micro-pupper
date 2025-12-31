/**
 * @file servo_pressure.h
 * @brief Servo load-based pressure detection
 */

#ifndef SERVO_PRESSURE_H
#define SERVO_PRESSURE_H

#include <stdint.h>
#include <stdbool.h>


/**
 * @brief Pressure status for all servos
 */
typedef struct {
    bool front_pressure;        // Servo 1 or 2 has pressure
    bool back_pressure;         // Servo 3 or 4 has pressure
    bool left_pressure;         // Servo 2 or 4 has pressure
    bool right_pressure;        // Servo 1 or 3 has pressure
    
    int16_t servo_load[4];      // Load values for each servo
    bool servo_pressure[4];     // Pressure status for each servo
} servo_pressure_status_t;

/**
 * @brief Initialize pressure detection
 */
bool servo_pressure_init(void);

/**
 * @brief Update pressure readings from all servos
 * @return true if any pressure status changed
 */
bool servo_pressure_update(void);

/**
 * @brief Check if front legs have pressure
 */
bool servo_pressure_check_front(void);

/**
 * @brief Check if back legs have pressure
 */
bool servo_pressure_check_back(void);

/**
 * @brief Check if left legs have pressure
 */
bool servo_pressure_check_left(void);

/**
 * @brief Check if right legs have pressure
 */
bool servo_pressure_check_right(void);

/**
 * @brief Get detailed pressure status
 */
void servo_pressure_get_status(servo_pressure_status_t *status);

/**
 * @brief Print current pressure status to console
 */
void servo_pressure_print_status(void);

#endif // SERVO_PRESSURE_H