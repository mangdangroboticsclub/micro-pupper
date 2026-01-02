/**
 * @file sts3032_servo.h
 * @brief STS3032 Servo Driver - Combined Header for HeySanta
 *
 * Low-level protocol and high-level API for STS3032 serial bus servos.
 * This is a self-contained header for the HeySanta board.
 */

#ifndef STS3032_SERVO_H
#define STS3032_SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ═══════════════════════════════════════════════════════
// PROTOCOL CONSTANTS
// ═══════════════════════════════════════════════════════

#define STS_FRAME_HEADER    0xFF
#define STS_BROADCAST_ID    0xFE

// STS Commands
#define STS_PING            0x01
#define STS_READ            0x02
#define STS_WRITE           0x03

// Memory Addresses (EEPROM)
#define STS_ID              0x05
#define STS_BAUD_RATE_ADDR  0x06

// Memory Addresses (RAM)
#define STS_TORQUE_ENABLE       0x28
#define STS_GOAL_POSITION_L     0x2A
#define STS_GOAL_POSITION_H     0x2B
#define STS_GOAL_TIME_L         0x2C
#define STS_GOAL_TIME_H         0x2D
#define STS_GOAL_SPEED_L        0x2E
#define STS_GOAL_SPEED_H        0x2F
#define STS_PRESENT_POSITION_L  0x38
#define STS_PRESENT_POSITION_H  0x39
#define STS_PRESENT_SPEED_L     0x3A
#define STS_PRESENT_SPEED_H     0x3B

// ═══════════════════════════════════════════════════════
// HARDWARE CONFIGURATION
// ═══════════════════════════════════════════════════════

typedef struct {
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t txen_pin;
    uint32_t baud_rate;
} sts_protocol_config_t;

// ═══════════════════════════════════════════════════════
// PROTOCOL FUNCTIONS
// ═══════════════════════════════════════════════════════

esp_err_t sts_protocol_init(const sts_protocol_config_t *config);
void sts_protocol_deinit(void);

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
bool sts_servo_get_load(uint8_t id, int16_t *load);

#ifdef __cplusplus
}
#endif

#endif // STS3032_SERVO_H
