/**
 * @file sts3032_servo.h
 * @brief STS3032 Servo Control API
 * 
 * High-level API for controlling STS3032 serial bus servos.
 * Provides easy-to-use functions for position control, torque control,
 * and servo management.
 */

#ifndef STS3032_SERVO_H
#define STS3032_SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "sts3032_protocol.h"

// ═══════════════════════════════════════════════════════
// SPEED PRESETS
// ═══════════════════════════════════════════════════════

typedef enum {
    SPEED_VERY_SLOW = 50,
    SPEED_SLOW      = 150,
    SPEED_MEDIUM    = 500,
    SPEED_FAST      = 1500,
    SPEED_VERY_FAST = 3000,
    SPEED_MAX       = 4095
} sts_servo_speed_t;

// ═══════════════════════════════════════════════════════
// CONVERSION FUNCTIONS
// ═══════════════════════════════════════════════════════

uint16_t sts_angle_to_position(float angle);
float sts_position_to_angle(uint16_t position);

// ═══════════════════════════════════════════════════════
// BASIC SERVO FUNCTIONS
// ═══════════════════════════════════════════════════════

bool sts_servo_ping(uint8_t id);
int sts_servo_scan_bus(uint8_t start_id, uint8_t end_id);
void sts_servo_enable_torque(uint8_t id, bool enable);

// ═══════════════════════════════════════════════════════
// POSITION CONTROL
// ═══════════════════════════════════════════════════════

void sts_servo_set_angle(uint8_t id, float angle, uint16_t speed);
void sts_servo_set_position(uint8_t id, uint16_t position, uint16_t speed);
bool sts_servo_get_position(uint8_t id, uint16_t *position);
bool sts_servo_get_angle(uint8_t id, float *angle);
bool sts_servo_get_speed(uint8_t id, uint16_t *speed);

// ═══════════════════════════════════════════════════════
// ID MANAGEMENT
// ═══════════════════════════════════════════════════════

bool sts_servo_change_id(uint8_t old_id, uint8_t new_id);
bool sts_servo_read_id(uint8_t query_id, uint8_t *current_id);
void sts_servo_broadcast_reset_id(void);

#endif // STS3032_SERVO_H
