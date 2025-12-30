/**
 * @file dog_config.h
 * @brief Dog Hardware Configuration
 * 
 * Centralizes all hardware-specific configuration for the quadruped robot:
 *   - UART and GPIO pin definitions
 *   - Servo ID assignments
 *   - Angle reversal for right-side servos (360 - angle)
 *   - Stance and swing angle definitions
 * 
 * Servo Layout (viewed from above):
 *   Front: [FL=2]  [FR=1]
 *   Back:  [BL=4]  [BR=3]
 * 
 * Angle Convention:
 *   - Left side servos use angles directly
 *   - Right side servos are reversed: actual_angle = 360 - desired_angle
 *   - This makes front and back leg programming identical
 */

#ifndef DOG_CONFIG_H
#define DOG_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "qmi8658a.h"

// ═══════════════════════════════════════════════════════
// UART CONFIGURATION
// ═══════════════════════════════════════════════════════

#define DOG_SERVO_UART_NUM      UART_NUM_1
#define DOG_SERVO_TX_PIN        GPIO_NUM_10
#define DOG_SERVO_RX_PIN        GPIO_NUM_11
#define DOG_SERVO_TXEN_PIN      GPIO_NUM_3
#define DOG_SERVO_BAUD_RATE     1000000

// ═══════════════════════════════════════════════════════
// SERVO ID DEFINITIONS
// ═══════════════════════════════════════════════════════

#define DOG_SERVO_FR            1   // Front Right
#define DOG_SERVO_FL            2   // Front Left
#define DOG_SERVO_BR            3   // Back Right
#define DOG_SERVO_BL            4   // Back Left

#define DOG_SERVO_COUNT         4

// ═══════════════════════════════════════════════════════
// ANGLE REVERSAL FOR RIGHT SIDE
// ═══════════════════════════════════════════════════════
// Right-side servos are mounted mirrored, so we reverse angles
// to make programming consistent between left and right sides.

/**
 * @brief Reverse angle for right-side servo (360 - angle)
 * @param angle Input angle in degrees
 * @return Reversed angle
 */
#define DOG_REVERSE_ANGLE(angle)    (360.0f - (angle))

/**
 * @brief Check if servo ID is on the right side
 */
#define DOG_IS_RIGHT_SIDE(id)       ((id) == DOG_SERVO_FR || (id) == DOG_SERVO_BR)

/**
 * @brief Check if servo ID is a front leg
 */
#define DOG_IS_FRONT_LEG(id)        ((id) == DOG_SERVO_FR || (id) == DOG_SERVO_FL)

// ═══════════════════════════════════════════════════════
// STANCE ANGLES (Neutral Position)
// ═══════════════════════════════════════════════════════
// Define angles from the LEFT side perspective, right side will be reversed.

// Front legs neutral: 90 degrees
#define DOG_STANCE_FRONT            90.0f

// Back legs neutral: 270 degrees
#define DOG_STANCE_BACK             270.0f

// ═══════════════════════════════════════════════════════
// SWING AMPLITUDE
// ═══════════════════════════════════════════════════════

#define DOG_SWING_AMPLITUDE         25.0f   // Degrees from stance

// ═══════════════════════════════════════════════════════
// SPEED PRESETS
// ═══════════════════════════════════════════════════════

#define DOG_SPEED_SLOW              300
#define DOG_SPEED_MEDIUM            700
#define DOG_SPEED_FAST              1500
#define DOG_SPEED_VERY_FAST         3000
#define DOG_SPEED_MAX               4095

// ═══════════════════════════════════════════════════════
// DYNAMIC STANCE SPEED (prevents shaking)
// ═══════════════════════════════════════════════════════
// Speed scales based on distance to target to prevent oscillation

#define DOG_STANCE_SPEED_MIN        100     // Speed for small corrections
#define DOG_STANCE_SPEED_MAX        1500    // Speed for large movements
#define DOG_STANCE_SPEED_THRESHOLD  30.0f   // Angle delta for max speed (degrees)
#define DOG_STANCE_SPEED_CURVE      1.5f    // Power curve (>1 = bias toward slow)

// ═══════════════════════════════════════════════════════
// IMU CONFIGURATION (QMI8658A)
// ═══════════════════════════════════════════════════════

#define DOG_IMU_I2C_NUM             I2C_NUM_0
#define DOG_IMU_SDA_PIN             GPIO_NUM_1
#define DOG_IMU_SCL_PIN             GPIO_NUM_2
#define DOG_IMU_I2C_FREQ_HZ         400000
#define DOG_IMU_I2C_ADDR            0x6A

// IMU sensor configuration
#define DOG_IMU_ACCEL_RANGE         QMI8658A_ACCEL_RANGE_8G
#define DOG_IMU_ACCEL_ODR           QMI8658A_ACCEL_ODR_500
#define DOG_IMU_GYRO_RANGE          QMI8658A_GYRO_RANGE_512
#define DOG_IMU_GYRO_ODR            QMI8658A_GYRO_ODR_500

// IMU logging configuration (change threshold before logging)
#define DOG_IMU_ACCEL_CHANGE_THRESHOLD  1.5f    // m/s² change before logging
#define DOG_IMU_GYRO_CHANGE_THRESHOLD   5.0f    // dps change before logging

/**
 * @brief Default IMU configuration for the dog
 */
#define DOG_IMU_DEFAULT_CONFIG() {                      \
    .i2c_num = DOG_IMU_I2C_NUM,                         \
    .sda_pin = DOG_IMU_SDA_PIN,                         \
    .scl_pin = DOG_IMU_SCL_PIN,                         \
    .i2c_freq_hz = DOG_IMU_I2C_FREQ_HZ,                 \
    .i2c_addr = DOG_IMU_I2C_ADDR,                       \
    .accel_range = DOG_IMU_ACCEL_RANGE,                 \
    .accel_odr = DOG_IMU_ACCEL_ODR,                     \
    .gyro_range = DOG_IMU_GYRO_RANGE,                   \
    .gyro_odr = DOG_IMU_GYRO_ODR                        \
}

// ═══════════════════════════════════════════════════════
// DOG CONFIGURATION STRUCTURE
// ═══════════════════════════════════════════════════════

/**
 * @brief Complete dog hardware configuration
 */
typedef struct {
    // UART configuration
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t txen_pin;
    uint32_t baud_rate;
    
    // Stance angles (before reversal applied)
    float stance_front;         // Front legs neutral angle
    float stance_back;          // Back legs neutral angle
    float swing_amplitude;      // Max deviation from stance
    
    // Default servo speed
    uint16_t default_speed;
} dog_config_t;

/**
 * @brief Default dog configuration
 */
#define DOG_DEFAULT_CONFIG() {                  \
    .uart_num = DOG_SERVO_UART_NUM,             \
    .tx_pin = DOG_SERVO_TX_PIN,                 \
    .rx_pin = DOG_SERVO_RX_PIN,                 \
    .txen_pin = DOG_SERVO_TXEN_PIN,             \
    .baud_rate = DOG_SERVO_BAUD_RATE,           \
    .stance_front = DOG_STANCE_FRONT,           \
    .stance_back = DOG_STANCE_BACK,             \
    .swing_amplitude = DOG_SWING_AMPLITUDE,     \
    .default_speed = DOG_SPEED_FAST             \
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the dog hardware
 * @param config Pointer to configuration, or NULL for defaults
 * @return true if initialization successful
 */
bool dog_init(const dog_config_t *config);

/**
 * @brief Get the current dog configuration
 * @return Pointer to the active configuration
 */
const dog_config_t* dog_get_config(void);

// ═══════════════════════════════════════════════════════
// SERVO CONTROL WITH AUTOMATIC REVERSAL
// ═══════════════════════════════════════════════════════

/**
 * @brief Move a servo to an angle (auto-reverses for right side)
 * @param servo_id Servo ID (1-4)
 * @param angle Desired angle in degrees (from left-side perspective)
 * @param speed Movement speed (0-4095)
 */
void dog_servo_move(uint8_t servo_id, float angle, uint16_t speed);

/**
 * @brief Move all servos at once (auto-reverses right side)
 * @param angle_fr Front-right angle (left perspective, will be reversed)
 * @param angle_fl Front-left angle
 * @param angle_br Back-right angle (left perspective, will be reversed)
 * @param angle_bl Back-left angle
 * @param speed Movement speed
 */
void dog_servo_move_all(float angle_fr, float angle_fl, 
                        float angle_br, float angle_bl, uint16_t speed);

/**
 * @brief Move all servos to stance position
 */
void dog_goto_stance(void);

/**
 * @brief Move all servos to stance with dynamic speed
 * 
 * Uses slower speed for small corrections to prevent shaking,
 * faster speed for large movements.
 */
void dog_goto_stance_smooth(void);

/**
 * @brief Get the stance angle for a specific servo
 * @param servo_id Servo ID (1-4)
 * @return Stance angle (already reversed for right side)
 */
float dog_get_stance_angle(uint8_t servo_id);

/**
 * @brief Get swing forward angle for a servo
 * @param servo_id Servo ID (1-4)
 * @return Forward swing angle (already reversed for right side)
 */
float dog_get_swing_forward_angle(uint8_t servo_id);

/**
 * @brief Get push back angle for a servo
 * @param servo_id Servo ID (1-4)
 * @return Push back angle (already reversed for right side)
 */
float dog_get_push_back_angle(uint8_t servo_id);

/**
 * @brief Check if all servos are responding
 * @return true if all 4 servos respond to ping
 */
bool dog_check_servos(void);

/**
 * @brief Enable/disable torque on all servos
 * @param enable true to enable torque
 */
void dog_set_torque(bool enable);

// ═══════════════════════════════════════════════════════
// IMU FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the IMU with dog-specific configuration
 * @return true if initialization successful
 */
bool dog_imu_init(void);

/**
 * @brief Read IMU data
 * @param out_data Pointer to output structure
 * @return true if read successful
 */
bool dog_imu_read(qmi8658a_data_t *out_data);

/**
 * @brief Start the IMU monitoring task with smart logging
 * Only logs when values change significantly
 */
void dog_imu_task_start(void);

#endif // DOG_CONFIG_H
