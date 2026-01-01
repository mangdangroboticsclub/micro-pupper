/**
 * @file gyro_balance.c
 * @brief Gyroscope-based balance/stabilization implementation
 * 
 * Uses Mahony filter to fuse accelerometer and gyroscope data for
 * stable angle estimation. Keeps legs facing ground based on pitch angle.
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
// MAHONY FILTER CONFIGURATION
// ═══════════════════════════════════════════════════════

#define MAHONY_KP                   1.0f    // Reduced - less accelerometer noise
#define MAHONY_KI                   0.003f  // Reduced - slower bias correction

#define ACCEL_MAGNITUDE_MIN         0.9f    // Tighter window = less noise
#define ACCEL_MAGNITUDE_MAX         1.1f

// ═══════════════════════════════════════════════════════
// ANTI-JITTER CONFIGURATION
// ═══════════════════════════════════════════════════════

// Chase smoothing
#define CHASE_SPEED                 0.4f   // Slower = smoother (was 0.4)
#define CHASE_DEADBAND              1.0f    // Larger = less micro-movements (was 0.3)

// Pitch angle filtering (low-pass on the angle itself)
#define PITCH_SMOOTHING             0.2f   // 0.0 = raw, 1.0 = frozen. Try 0.8-0.9

// Minimum correction change to actually send to servos
#define SERVO_UPDATE_THRESHOLD      0.1f    // degrees - ignore tiny changes

// Hysteresis - once moving, keep moving until truly stable
#define HYSTERESIS_THRESHOLD        0.2f   // degrees

// ═══════════════════════════════════════════════════════
// BALANCE STATE
// ═══════════════════════════════════════════════════════

static bool balance_enabled = GYRO_BALANCE_ENABLED_DEFAULT;
static bool initialized = false;
static TickType_t last_balance_time = 0;
static TickType_t last_update_time = 0;

// Mahony filter state - quaternion representation
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Integral error for Mahony filter (gyro bias estimation)
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// Estimated Euler angles (in degrees)
static float pitch_angle_raw = 0.0f;    // Direct from quaternion
static float pitch_angle = 0.0f;        // Smoothed version we actually use
static float roll_angle = 0.0f;

// Smooth chase state
static float target_correction = 0.0f;
static float current_correction = 0.0f;
static float last_sent_correction = 0.0f;  // What we last sent to servos
static bool is_moving = false;              // Hysteresis state

// ═══════════════════════════════════════════════════════
// TOGGLE GESTURE STATE
// ═══════════════════════════════════════════════════════

typedef enum {
    TOGGLE_IDLE,
    TOGGLE_FIRST_ROTATION,
    TOGGLE_WAITING_REVERSE
} toggle_state_t;

static toggle_state_t toggle_state = TOGGLE_IDLE;
static int8_t first_rotation_dir = 0;
static TickType_t toggle_gesture_start = 0;
static TickType_t last_toggle_time = 0;

// ═══════════════════════════════════════════════════════
// MAHONY FILTER IMPLEMENTATION
// ═══════════════════════════════════════════════════════

static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // Convert gyro from deg/s to rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    
    float accel_magnitude = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;
    
    if (accel_magnitude > ACCEL_MAGNITUDE_MIN && accel_magnitude < ACCEL_MAGNITUDE_MAX) {
        recipNorm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;
        
        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);
        
        if (MAHONY_KI > 0.0f) {
            // Clamp integral to prevent windup
            integralFBx += MAHONY_KI * halfex * dt;
            integralFBy += MAHONY_KI * halfey * dt;
            integralFBz += MAHONY_KI * halfez * dt;
            
            // Anti-windup clamp
            const float MAX_INTEGRAL = 0.5f;
            if (integralFBx > MAX_INTEGRAL) integralFBx = MAX_INTEGRAL;
            if (integralFBx < -MAX_INTEGRAL) integralFBx = -MAX_INTEGRAL;
            if (integralFBy > MAX_INTEGRAL) integralFBy = MAX_INTEGRAL;
            if (integralFBy < -MAX_INTEGRAL) integralFBy = -MAX_INTEGRAL;
            if (integralFBz > MAX_INTEGRAL) integralFBz = MAX_INTEGRAL;
            if (integralFBz < -MAX_INTEGRAL) integralFBz = -MAX_INTEGRAL;
            
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        
        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;
    }
    
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += (qa*gx + qc*gz - q3*gy);
    q2 += (qa*gy - qb*gz + q3*gx);
    q3 += (qa*gz + qb*gy - qc*gx);
    
    recipNorm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

static void quaternion_to_euler(void)
{
    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
    roll_angle = atan2f(sinr_cosp, cosr_cosp) * 57.2958f;
    
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f) {
        pitch_angle_raw = copysignf(90.0f, sinp);
    } else {
        pitch_angle_raw = asinf(sinp) * 57.2958f;
    }
    
    // ═══════════════════════════════════════════════════
    // LOW-PASS FILTER on pitch angle itself
    // ═══════════════════════════════════════════════════
    pitch_angle = pitch_angle * PITCH_SMOOTHING + pitch_angle_raw * (1.0f - PITCH_SMOOTHING);
}

// ═══════════════════════════════════════════════════════
// TOGGLE GESTURE DETECTION
// ═══════════════════════════════════════════════════════

static void detect_toggle_gesture(const qmi8658a_data_t *data)
{
    TickType_t now = xTaskGetTickCount();
    
    if (pdTICKS_TO_MS(now - last_toggle_time) < GYRO_BALANCE_TOGGLE_COOLDOWN_MS) {
        return;
    }
    
    float gyro_x = data->gyro_x;
    bool above_threshold = fabsf(gyro_x) >= GYRO_BALANCE_TOGGLE_THRESHOLD;
    int8_t current_dir = (gyro_x > 0) ? 1 : -1;
    
    switch (toggle_state) {
        case TOGGLE_IDLE:
            if (above_threshold) {
                toggle_state = TOGGLE_FIRST_ROTATION;
                first_rotation_dir = current_dir;
                toggle_gesture_start = now;
                ESP_LOGD(TAG, "Toggle gesture started (dir: %+d)", first_rotation_dir);
            }
            break;
            
        case TOGGLE_FIRST_ROTATION:
            if (pdTICKS_TO_MS(now - toggle_gesture_start) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                toggle_state = TOGGLE_IDLE;
                ESP_LOGD(TAG, "Toggle gesture timed out");
                break;
            }
            if (!above_threshold) {
                toggle_state = TOGGLE_WAITING_REVERSE;
            }
            break;
            
        case TOGGLE_WAITING_REVERSE:
            if (pdTICKS_TO_MS(now - toggle_gesture_start) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                toggle_state = TOGGLE_IDLE;
                ESP_LOGD(TAG, "Toggle gesture timed out waiting for reverse");
                break;
            }
            if (above_threshold) {
                if (current_dir != first_rotation_dir) {
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

static void apply_balance(const qmi8658a_data_t *data)
{
    TickType_t now = xTaskGetTickCount();
    
    float dt = pdTICKS_TO_MS(now - last_update_time) / 1000.0f;
    last_update_time = now;
    
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.02f;
    }
    
    mahony_update(data->gyro_x, data->gyro_y, data->gyro_z,
                  data->accel_x, data->accel_y, data->accel_z,
                  dt);
    quaternion_to_euler();
    
    if (!balance_enabled) {
        return;
    }
    
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_balance_time);
    if (elapsed_ms < GYRO_BALANCE_UPDATE_INTERVAL_MS) {
        return;
    }
    last_balance_time = now;
    
    // ═══════════════════════════════════════════════════
    // STEP 1: Calculate target correction
    // ═══════════════════════════════════════════════════
    
    float effective_pitch = pitch_angle;
    if (fabsf(effective_pitch) < GYRO_BALANCE_DEADZONE) {
        effective_pitch = 0.0f;
    }
    
    target_correction = effective_pitch * GYRO_BALANCE_GAIN;
    
    if (target_correction > GYRO_BALANCE_MAX_CORRECTION) {
        target_correction = GYRO_BALANCE_MAX_CORRECTION;
    } else if (target_correction < -GYRO_BALANCE_MAX_CORRECTION) {
        target_correction = -GYRO_BALANCE_MAX_CORRECTION;
    }
    
    // ═══════════════════════════════════════════════════
    // STEP 2: Chase with hysteresis
    // ═══════════════════════════════════════════════════
    
    float diff = target_correction - current_correction;
    float abs_diff = fabsf(diff);
    
    // Hysteresis: use different thresholds for starting vs stopping movement
    float threshold = is_moving ? HYSTERESIS_THRESHOLD : CHASE_DEADBAND;
    
    if (abs_diff > threshold) {
        current_correction += diff * CHASE_SPEED;
        is_moving = true;
    } else {
        is_moving = false;
    }
    
    // ═══════════════════════════════════════════════════
    // STEP 3: Only send to servos if change is significant
    // ═══════════════════════════════════════════════════
    
    float servo_diff = fabsf(current_correction - last_sent_correction);
    
    if (servo_diff < SERVO_UPDATE_THRESHOLD) {
        // Skip this update - change too small
        return;
    }
    
    last_sent_correction = current_correction;
    
    // ═══════════════════════════════════════════════════
    // STEP 4: Apply to servos
    // ═══════════════════════════════════════════════════
    
    float angle_fl = DOG_STANCE_FRONT + current_correction;
    float angle_fr = DOG_STANCE_FRONT + current_correction;
    float angle_bl = DOG_STANCE_BACK + current_correction;
    float angle_br = DOG_STANCE_BACK + current_correction;
    
    // Use a fixed moderate speed for smoother movement
    // Dynamic speed can cause jitter when oscillating
    uint16_t servo_speed = (GYRO_BALANCE_SPEED_MIN + GYRO_BALANCE_SPEED_MAX) / 2;
    
    dog_servo_move_all(angle_fr, angle_fl, angle_br, angle_bl, servo_speed);
    
    // Debug logging
    static int log_counter = 0;
    if (++log_counter >= 25) {
        ESP_LOGD(TAG, "Pitch: %+5.1f° (raw:%+5.1f°) Target: %+5.1f° Current: %+5.1f°", 
                 pitch_angle, pitch_angle_raw, target_correction, current_correction);
        log_counter = 0;
    }
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void gyro_balance_init(void)
{
    ESP_LOGI(TAG, "Gyro balance system initialized (Mahony filter)");
    ESP_LOGI(TAG, "Default: %s, Max correction: %.1f°, Gain: %.2f",
             GYRO_BALANCE_ENABLED_DEFAULT ? "ENABLED" : "DISABLED",
             GYRO_BALANCE_MAX_CORRECTION, GYRO_BALANCE_GAIN);
    ESP_LOGI(TAG, "Mahony Kp: %.2f, Ki: %.4f", MAHONY_KP, MAHONY_KI);
    ESP_LOGI(TAG, "Chase: %.2f, Deadband: %.1f°, Pitch smooth: %.2f", 
             CHASE_SPEED, CHASE_DEADBAND, PITCH_SMOOTHING);
    ESP_LOGI(TAG, "Servo threshold: %.1f°, Hysteresis: %.2f°",
             SERVO_UPDATE_THRESHOLD, HYSTERESIS_THRESHOLD);
    
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    pitch_angle_raw = 0.0f;
    pitch_angle = 0.0f;
    roll_angle = 0.0f;
    target_correction = 0.0f;
    current_correction = 0.0f;
    last_sent_correction = 0.0f;
    is_moving = false;
    
    last_balance_time = xTaskGetTickCount();
    last_update_time = xTaskGetTickCount();
    toggle_state = TOGGLE_IDLE;
    last_toggle_time = 0;
    
    initialized = true;
}

void gyro_balance_process(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    detect_toggle_gesture(data);
    apply_balance(data);
}

void gyro_balance_enable(bool enable)
{
    if (enable && !balance_enabled) {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        pitch_angle_raw = 0.0f;
        pitch_angle = 0.0f;
        roll_angle = 0.0f;
        target_correction = 0.0f;
        current_correction = 0.0f;
        last_sent_correction = 0.0f;
        is_moving = false;
        last_balance_time = xTaskGetTickCount();
        last_update_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "Gyro balance ENABLED");
    } else if (!enable && balance_enabled) {
        dog_goto_stance_smooth();
        ESP_LOGI(TAG, "Gyro balance DISABLED - returning to stance");
    }
    balance_enabled = enable;
}

bool gyro_balance_is_enabled(void)
{
    return balance_enabled;
}

float gyro_balance_get_pitch(void)
{
    return pitch_angle;
}

float gyro_balance_get_roll(void)
{
    return roll_angle;
}