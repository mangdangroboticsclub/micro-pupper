/**
 * @file ble_servo.h
 * @brief Minimal BLE Servo Controller
 * 
 * Simple BLE interface for servo control - designed for gait animation iteration.
 * Receives servo commands via Web Bluetooth and executes them.
 * 
 * Commands (JSON):
 *   {"s":[fr,fl,br,bl,speed,delay]}  - Move all 4 servos (unified timing)
 *   {"m":[[fr,fl,br,bl,speed,delay], ...]}  - Sequence of unified moves
 *   {"l":[[fr,fr_spd,fr_dly],[fl,fl_spd,fl_dly],[br,br_spd,br_dly],[bl,bl_spd,bl_dly]]}
 *        - Per-leg move with individual speed/delay per leg
 *   {"L":[<leg1>,<leg2>,...]}  - Sequence of per-leg moves
 *   {"p":1}  - Ping (returns {"p":1})
 *   {"r":1}  - Return to stance
 */

#ifndef BLE_SERVO_H
#define BLE_SERVO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ═══════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════

#define BLE_SERVO_DEVICE_NAME   "MicroPupper"

// ═══════════════════════════════════════════════════════
// CALLBACKS
// ═══════════════════════════════════════════════════════

/**
 * @brief Callback for servo movement command
 * @param fr Front-right angle (unified, auto-reversed for right side)
 * @param fl Front-left angle  
 * @param br Back-right angle (unified, auto-reversed for right side)
 * @param bl Back-left angle
 * @param speed Servo speed (0-4095)
 * @param delay_ms Delay after this move before next command
 */
typedef void (*ble_servo_move_cb_t)(float fr, float fl, float br, float bl, 
                                     uint16_t speed, uint16_t delay_ms);

/**
 * @brief Per-leg move parameters
 */
typedef struct {
    float angle;        ///< Target angle
    uint16_t speed;     ///< Servo speed (0-4095)
    uint16_t delay_ms;  ///< Delay after moving this leg
} ble_leg_move_t;

/**
 * @brief Callback for per-leg movement command
 * Each leg moves with its own speed and delay, allowing offset timing between legs.
 * @param fr Front-right leg parameters
 * @param fl Front-left leg parameters
 * @param br Back-right leg parameters
 * @param bl Back-left leg parameters
 */
typedef void (*ble_servo_leg_move_cb_t)(ble_leg_move_t fr, ble_leg_move_t fl,
                                         ble_leg_move_t br, ble_leg_move_t bl);

/**
 * @brief Callback for return to stance
 */
typedef void (*ble_servo_stance_cb_t)(void);

/**
 * @brief Callback for connection state changes
 */
typedef void (*ble_servo_connect_cb_t)(bool connected);

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize and start BLE servo controller
 * @param move_cb Callback when servo move command received (unified timing)
 * @param leg_move_cb Callback for per-leg move command (individual timing, can be NULL)
 * @param stance_cb Callback to return to stance
 * @param connect_cb Callback for connection state (optional, can be NULL)
 * @return true if successful
 */
bool ble_servo_init(ble_servo_move_cb_t move_cb,
                    ble_servo_leg_move_cb_t leg_move_cb,
                    ble_servo_stance_cb_t stance_cb,
                    ble_servo_connect_cb_t connect_cb);

/**
 * @brief Check if BLE is connected
 */
bool ble_servo_is_connected(void);

/**
 * @brief Send current servo positions back to client (for feedback)
 */
bool ble_servo_send_state(float fr, float fl, float br, float bl);

/**
 * @brief Send a simple response message
 */
bool ble_servo_send_response(const char* msg);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVO_H
