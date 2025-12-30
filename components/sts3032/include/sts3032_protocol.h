/**
 * @file sts3032_protocol.h
 * @brief STS3032 Servo Protocol Layer
 *
 * Low-level protocol implementation for STS3032 serial bus servos.
 * Handles packet construction, checksums, and basic communication.
 */

#ifndef STS3032_PROTOCOL_H
#define STS3032_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

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
#define STS_ID              0x05  // Servo ID address
#define STS_BAUD_RATE_ADDR  0x06  // Baud rate address

// Memory Addresses (RAM)
#define STS_TORQUE_ENABLE   0x28
#define STS_GOAL_POSITION_L 0x2A
#define STS_GOAL_POSITION_H 0x2B
#define STS_GOAL_TIME_L     0x2C
#define STS_GOAL_TIME_H     0x2D
#define STS_GOAL_SPEED_L    0x2E
#define STS_GOAL_SPEED_H    0x2F
#define STS_PRESENT_POSITION_L 0x38
#define STS_PRESENT_POSITION_H 0x39
#define STS_PRESENT_SPEED_L    0x3A
#define STS_PRESENT_SPEED_H    0x3B

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
uint8_t sts_checksum(uint8_t *buf, int len);
void sts_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, int param_len);
bool sts_read_response(uint8_t *response, int max_len, int *out_len);
void sts_write_register(uint8_t id, uint8_t address, uint8_t *data, int len);
bool sts_read_register(uint8_t id, uint8_t address, int len, uint8_t *data);

#endif // STS3032_PROTOCOL_H
