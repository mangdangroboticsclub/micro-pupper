/**
 * @file gyro_balance.c
 * @brief Gyroscope-based balance/stabilization implementation
 * 
 * Uses gyro Y axis to keep legs facing ground.
 * Toggle feature: rotate robot on X axis (like a barrel roll) to enable/disable.
 */

#include "gyro_balance.h"
#include "dog_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "GYRO_BAL";

// ═══════════════════════════════════════════════════════
// BALANCE STATE
// ═══════════════════════════════════════════════════════

static bool balance_enabled = GYRO_BALANCE_ENABLED_DEFAULT;
static bool initialized = false;
static float gyro_filtered_y = 0.0f;
static float accumulated_angle = 0.0f;
static float prev_accumulated_angle = 0.0f;
static TickType_t last_balance_time = 0;

// ═══════════════════════════════════════════════════════
// TOGGLE GESTURE STATE
// ═══════════════════════════════════════════════════════

typedef enum {
    TOGGLE_IDLE,
    TOGGLE_FIRST_ROTATION,      // Detected first direction
    TOGGLE_WAITING_REVERSE      // Waiting for opposite direction
} toggle_state_t;

static toggle_state_t toggle_state = TOGGLE_IDLE;
static int8_t first_rotation_dir = 0;   // +1 or -1
static TickType_t toggle_gesture_start = 0;
static TickType_t last_toggle_time = 0;

// ═══════════════════════════════════════════════════════
// TOGGLE GESTURE DETECTION
// ═══════════════════════════════════════════════════════

/**
 * @brief Detect X-axis rotation gesture to toggle balance
 * 
 * Gesture: Rotate robot on X axis one way, then back.
 * This creates a distinctive pattern in gyro_x readings.
 */
static void detect_toggle_gesture(const qmi8658a_data_t *data)
{
    TickType_t now = xTaskGetTickCount();
    
    // Check cooldown
    if (pdTICKS_TO_MS(now - last_toggle_time) < GYRO_BALANCE_TOGGLE_COOLDOWN_MS) {
        return;
    }
    
    float gyro_x = data->gyro_x;
    bool above_threshold = fabsf(gyro_x) >= GYRO_BALANCE_TOGGLE_THRESHOLD;
    int8_t current_dir = (gyro_x > 0) ? 1 : -1;
    
    switch (toggle_state) {
        case TOGGLE_IDLE:
            if (above_threshold) {
                // Start of gesture - first rotation detected
                toggle_state = TOGGLE_FIRST_ROTATION;
                first_rotation_dir = current_dir;
                toggle_gesture_start = now;
                ESP_LOGD(TAG, "Toggle gesture started (dir: %+d)", first_rotation_dir);
            }
            break;
            
        case TOGGLE_FIRST_ROTATION:
            // Check timeout
            if (pdTICKS_TO_MS(now - toggle_gesture_start) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                toggle_state = TOGGLE_IDLE;
                ESP_LOGD(TAG, "Toggle gesture timed out");
                break;
            }
            
            // If rotation stopped, wait for reverse
            if (!above_threshold) {
                toggle_state = TOGGLE_WAITING_REVERSE;
            }
            break;
            
        case TOGGLE_WAITING_REVERSE:
            // Check timeout
            if (pdTICKS_TO_MS(now - toggle_gesture_start) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                toggle_state = TOGGLE_IDLE;
                ESP_LOGD(TAG, "Toggle gesture timed out waiting for reverse");
                break;
            }
            
            if (above_threshold) {
                if (current_dir != first_rotation_dir) {
                    // Reverse rotation detected - toggle!
                    bool new_state = !balance_enabled;
                    gyro_balance_enable(new_state);
                    last_toggle_time = now;
                    ESP_LOGI(TAG, "Toggle gesture complete! Balance: %s", 
                             new_state ? "ON" : "OFF");
                }
                toggle_state = TOGGLE_IDLE;
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════
// BALANCE CONTROL
// ═══════════════════════════════════════════════════════

/**
 * @brief Apply gyro-based stabilization to keep legs facing ground
 */
static void apply_balance(const qmi8658a_data_t *data)
{
    if (!balance_enabled) {
        return;
    }
    
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_balance_time);
    
    // Rate limit updates
    if (elapsed_ms < GYRO_BALANCE_UPDATE_INTERVAL_MS) {
        return;
    }
    last_balance_time = now;
    
    // Get gyro Y reading (pitch rate in degrees/second)
    float gyro_y = data->gyro_y;
    
    // Apply deadzone
    if (fabsf(gyro_y) < GYRO_BALANCE_DEADZONE) {
        gyro_y = 0.0f;
    }
    
    // Low-pass filter for smoothing
    gyro_filtered_y = (GYRO_BALANCE_SMOOTHING * gyro_y) + 
                      ((1.0f - GYRO_BALANCE_SMOOTHING) * gyro_filtered_y);
    
    // Calculate correction angle
    float correction = gyro_filtered_y * GYRO_BALANCE_GAIN;
    
    // Integrate to accumulate angle (with decay to prevent drift)
    accumulated_angle = accumulated_angle * 0.98f + correction * 0.02f;
    
    // Clamp correction to maximum allowed
    if (accumulated_angle > GYRO_BALANCE_MAX_CORRECTION) {
        accumulated_angle = GYRO_BALANCE_MAX_CORRECTION;
    } else if (accumulated_angle < -GYRO_BALANCE_MAX_CORRECTION) {
        accumulated_angle = -GYRO_BALANCE_MAX_CORRECTION;
    }
    
    // Apply correction to all legs
    float front_correction = accumulated_angle;
    float back_correction = accumulated_angle;
    
    float angle_fl = DOG_STANCE_FRONT + front_correction;
    float angle_fr = DOG_STANCE_FRONT + front_correction;
    float angle_bl = DOG_STANCE_BACK + back_correction;
    float angle_br = DOG_STANCE_BACK + back_correction;
    
    // Calculate dynamic speed based on angle change
    float angle_delta = fabsf(accumulated_angle - prev_accumulated_angle);
    prev_accumulated_angle = accumulated_angle;
    
    float speed_ratio = angle_delta / GYRO_BALANCE_SPEED_THRESHOLD;
    if (speed_ratio > 1.0f) {
        speed_ratio = 1.0f;
    }
    
    // Apply power curve
    speed_ratio = powf(speed_ratio, GYRO_BALANCE_SPEED_CURVE);
    
    uint16_t dynamic_speed = (uint16_t)(GYRO_BALANCE_SPEED_MIN + 
                              speed_ratio * (GYRO_BALANCE_SPEED_MAX - GYRO_BALANCE_SPEED_MIN));
    
    // Move servos
    dog_servo_move_all(angle_fr, angle_fl, angle_br, angle_bl, dynamic_speed);
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void gyro_balance_init(void)
{
    ESP_LOGI(TAG, "Gyro balance system initialized");
    ESP_LOGI(TAG, "Default: %s, Max correction: %.1f°, Gain: %.2f",
             GYRO_BALANCE_ENABLED_DEFAULT ? "ENABLED" : "DISABLED",
             GYRO_BALANCE_MAX_CORRECTION, GYRO_BALANCE_GAIN);
    ESP_LOGI(TAG, "Toggle gesture: rotate X-axis > %.0f dps both directions within %d ms",
             GYRO_BALANCE_TOGGLE_THRESHOLD, GYRO_BALANCE_TOGGLE_WINDOW_MS);
    
    gyro_filtered_y = 0.0f;
    accumulated_angle = 0.0f;
    prev_accumulated_angle = 0.0f;
    last_balance_time = xTaskGetTickCount();
    toggle_state = TOGGLE_IDLE;
    last_toggle_time = 0;
    
    initialized = true;
}

void gyro_balance_process(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    // Always check for toggle gesture
    detect_toggle_gesture(data);
    
    // Apply balance if enabled
    apply_balance(data);
}

void gyro_balance_enable(bool enable)
{
    if (enable && !balance_enabled) {
        // Reset state when enabling
        gyro_filtered_y = 0.0f;
        accumulated_angle = 0.0f;
        prev_accumulated_angle = 0.0f;
        last_balance_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "Gyro balance ENABLED");
    } else if (!enable && balance_enabled) {
        // Return to stance smoothly when disabling
        dog_goto_stance_smooth();
        ESP_LOGI(TAG, "Gyro balance DISABLED - returning to stance");
    }
    balance_enabled = enable;
}

bool gyro_balance_is_enabled(void)
{
    return balance_enabled;
}
