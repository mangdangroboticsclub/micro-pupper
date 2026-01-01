/**
 * @file servo_pressure.c
 * @brief Servo position-based pressure detection
 */

#include "servo_pressure.h"
#include "sts3032_driver.h"
#include "dog_config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "SERVO_PRESSURE";

// ═══════════════════════════════════════════════════════
// PRESSURE THRESHOLDS
// ═══════════════════════════════════════════════════════

#define POSITION_ERROR_THRESHOLD  0.2f  // Degrees of error indicates pressure
#define PRESSURE_SAMPLES          1     // Number of consecutive samples needed

// ═══════════════════════════════════════════════════════
// TARGET POSITIONS (From LEFT-side perspective)
// ═══════════════════════════════════════════════════════

static const float TARGET_ANGLES[4] = {
    90.0f,   // FR (ID1) - Right front (will be reversed to 270 on servo)
    90.0f,   // FL (ID2) - Left front
    270.0f,  // BR (ID3) - Right back (will be reversed to 90 on servo)
    270.0f   // BL (ID4) - Left back
};

// ═══════════════════════════════════════════════════════
// PRESSURE DETECTION STATE
// ═══════════════════════════════════════════════════════

typedef struct {
    float target_angle;      // Physical target angle (left perspective)
    float current_angle;     // Physical current angle (left perspective)
    float position_error;
    uint8_t pressure_count;
    bool has_pressure;
    bool read_success;
} servo_pressure_state_t;

static servo_pressure_state_t g_servo_state[4] = {0};

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

static bool read_servo_position(uint8_t servo_id, float *physical_angle)
{
    float raw_angle = 0;
    
    if (!sts_servo_get_angle(servo_id, &raw_angle)) {
        return false;
    }
    
    // Convert raw servo angle to physical angle (left-side perspective)
    // Right-side servos (FR=1, BR=3) are reversed
    if (DOG_IS_RIGHT_SIDE(servo_id)) {
        *physical_angle = 360.0f - raw_angle;
    } else {
        *physical_angle = raw_angle;
    }
    
    return true;
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool servo_pressure_init(void)
{
    ESP_LOGI(TAG, "Servo pressure detection initialized");
    ESP_LOGI(TAG, "Target positions (left perspective): FR=%.0f° FL=%.0f° BR=%.0f° BL=%.0f°",
             TARGET_ANGLES[0], TARGET_ANGLES[1], TARGET_ANGLES[2], TARGET_ANGLES[3]);
    ESP_LOGI(TAG, "Position error threshold: %.1f degrees", POSITION_ERROR_THRESHOLD);
    ESP_LOGI(TAG, "Samples needed: %d", PRESSURE_SAMPLES);
    
    for (uint8_t i = 0; i < 4; i++) {
        g_servo_state[i].target_angle = TARGET_ANGLES[i];
        g_servo_state[i].current_angle = 0;
        g_servo_state[i].position_error = 0;
        g_servo_state[i].pressure_count = 0;
        g_servo_state[i].has_pressure = false;
        g_servo_state[i].read_success = false;
    }
    
    ESP_LOGI(TAG, "Waiting 2 seconds for servos to settle...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Clear any initial false positives
    for (uint8_t i = 0; i < 4; i++) {
        float angle = 0;
        if (read_servo_position(i + 1, &angle)) {
            g_servo_state[i].current_angle = angle;
        }
    }
    
    ESP_LOGI(TAG, "Pressure detection ready - monitoring continuously");
    
    return true;
}

bool servo_pressure_update(void)
{
    bool any_change = false;
    static servo_pressure_state_t last_logged[4] = {0};
    static bool first_log = true;
    
    // Declare servo names at function scope
    const char *servo_names[] = {"FR", "FL", "BR", "BL"};
    
    for (uint8_t i = 0; i < 4; i++) {
        float current_angle = 0;
        uint8_t servo_id = i + 1;
        
        bool read_ok = read_servo_position(servo_id, &current_angle);
        g_servo_state[i].read_success = read_ok;
        
        if (!read_ok) {
            continue;
        }
        
        g_servo_state[i].current_angle = current_angle;
        
        // Calculate position error (how far from target)
        float error = fabs(current_angle - g_servo_state[i].target_angle);
        
        // Handle angle wrap-around (e.g., 359° vs 1°)
        if (error > 180.0f) {
            error = 360.0f - error;
        }
        
        g_servo_state[i].position_error = error;
        
        // Check if position changed AT ALL (like IMU - print every change)
        float angle_change = fabs(current_angle - last_logged[i].current_angle);
        if (angle_change > 180.0f) {
            angle_change = 360.0f - angle_change;
        }
        
        bool should_log = first_log || (angle_change > 0.3f); // Log if position changes by >0.3°
        
        if (should_log) {
            ESP_LOGI(TAG, "%s: Target=%5.1f° Current=%5.1f° Error=%4.1f° %s", 
                     servo_names[i],
                     g_servo_state[i].target_angle,
                     current_angle,
                     error,
                     error > POSITION_ERROR_THRESHOLD ? "⚠️  PRESSURE" : "");
            
            last_logged[i] = g_servo_state[i];
        }
        
        // If servo is pushed away from target, detect pressure
        if (error > POSITION_ERROR_THRESHOLD) {
            g_servo_state[i].pressure_count++;
            
            if (g_servo_state[i].pressure_count >= PRESSURE_SAMPLES) {
                if (!g_servo_state[i].has_pressure) {
                    ESP_LOGI(TAG, "🚨 PRESSURE DETECTED on %s!", servo_names[i]);
                    any_change = true;
                }
                g_servo_state[i].has_pressure = true;
            }
        } else {
            // Reset pressure detection
            if (g_servo_state[i].has_pressure) {
                ESP_LOGI(TAG, "✅ Pressure RELEASED on %s", servo_names[i]);
                any_change = true;
            }
            g_servo_state[i].pressure_count = 0;
            g_servo_state[i].has_pressure = false;
        }
    }
    
    first_log = false;
    
    return any_change;
}

bool servo_pressure_check_front(void)
{
    return g_servo_state[0].has_pressure || g_servo_state[1].has_pressure;
}

bool servo_pressure_check_back(void)
{
    return g_servo_state[2].has_pressure || g_servo_state[3].has_pressure;
}

bool servo_pressure_check_left(void)
{
    return g_servo_state[1].has_pressure || g_servo_state[3].has_pressure;
}

bool servo_pressure_check_right(void)
{
    return g_servo_state[0].has_pressure || g_servo_state[2].has_pressure;
}

void servo_pressure_get_status(servo_pressure_status_t *status)
{
    if (!status) return;
    
    status->front_pressure = servo_pressure_check_front();
    status->back_pressure = servo_pressure_check_back();
    status->left_pressure = servo_pressure_check_left();
    status->right_pressure = servo_pressure_check_right();
    
    for (uint8_t i = 0; i < 4; i++) {
        status->servo_load[i] = (int16_t)(g_servo_state[i].position_error * 10);
        status->servo_pressure[i] = g_servo_state[i].has_pressure;
    }
}

void servo_pressure_print_status(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    
    const char *servo_names[] = {"FR", "FL", "BR", "BL"};
    
    for (uint8_t i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "%s (ID%d): Target=%5.1f° Current=%5.1f° Error=%4.1f° %s", 
                 servo_names[i],
                 i + 1,
                 g_servo_state[i].target_angle,
                 g_servo_state[i].current_angle,
                 g_servo_state[i].position_error,
                 g_servo_state[i].has_pressure ? "⚠️  PRESSURE" : "✓ OK");
    }
    
    ESP_LOGI(TAG, "───────────────────────────────────────");
    ESP_LOGI(TAG, "Regions: Front=%s  Back=%s  Left=%s  Right=%s",
             servo_pressure_check_front() ? "PRESS" : "OK",
             servo_pressure_check_back() ? "PRESS" : "OK",
             servo_pressure_check_left() ? "PRESS" : "OK",
             servo_pressure_check_right() ? "PRESS" : "OK");
    ESP_LOGI(TAG, "═══════════════════════════════════════");
}