#include "pressure_balance.h"
#include "dog_config.h"
#include "sts3032_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "PRES_BAL";

// ═══════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════

#define PRESSURE_THRESHOLD          0.0f    // Degrees of error to trigger
#define PRESSURE_CROUCH_AMOUNT     35.0f    // How much back legs go down
#define PRESSURE_CROUCH_SPEED       500     // Speed to crouch down
#define PRESSURE_RETURN_SPEED       500     // Speed to return to stance
#define PRESSURE_UPDATE_MS           25     // Update interval
#define PRESSURE_DEADZONE           0.2f    // Ignore tiny errors

// Timing
#define PRESSURE_HOLD_MS            400     // Hold crouch for this long
#define PRESSURE_RETURN_WAIT_MS     400     // Wait for servos to finish returning
#define PRESSURE_COOLDOWN_MS       2000     // Long cooldown to prevent re-trigger

// ═══════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════

typedef enum {
    STATE_INIT,             // Initial settling
    STATE_MONITORING,       // Watching for pressure
    STATE_CROUCHING,        // Moving down
    STATE_HOLDING,          // Holding crouch position
    STATE_RETURNING,        // Moving back up
    STATE_COOLDOWN          // Waiting before next trigger
} pressure_state_t;

static pressure_state_t s_state = STATE_INIT;
static TickType_t s_state_start_time = 0;

static bool s_enabled = true;
static bool s_initialized = false;
static TickType_t s_last_update = 0;

// Use fixed baseline (the commanded stance positions)
static const float BASELINE_BL = DOG_STANCE_BACK;   // 270.0
static const float BASELINE_BR = DOG_STANCE_BACK;   // 270.0

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

static const char* state_name(pressure_state_t state)
{
    switch (state) {
        case STATE_INIT:       return "INIT";
        case STATE_MONITORING: return "MONITORING";
        case STATE_CROUCHING:  return "CROUCHING";
        case STATE_HOLDING:    return "HOLDING";
        case STATE_RETURNING:  return "RETURNING";
        case STATE_COOLDOWN:   return "COOLDOWN";
        default: return "UNKNOWN";
    }
}

static void change_state(pressure_state_t new_state)
{
    if (s_state != new_state) {
        ESP_LOGI(TAG, "State: %s -> %s", state_name(s_state), state_name(new_state));
        s_state = new_state;
        s_state_start_time = xTaskGetTickCount();
    }
}

static uint32_t time_in_state_ms(void)
{
    return pdTICKS_TO_MS(xTaskGetTickCount() - s_state_start_time);
}

/**
 * @brief Read servo position as physical angle
 */
static bool read_physical_angle(uint8_t servo_id, float *angle_out)
{
    float raw_angle = 0;
    
    if (!sts_servo_get_angle(servo_id, &raw_angle)) {
        return false;
    }
    
    if (DOG_IS_RIGHT_SIDE(servo_id)) {
        *angle_out = 360.0f - raw_angle;
    } else {
        *angle_out = raw_angle;
    }
    
    return true;
}

/**
 * @brief Get pressure error from back legs (compared to fixed baseline)
 */
static float get_pressure_error(void)
{
    float bl = 0, br = 0;
    
    if (!read_physical_angle(DOG_SERVO_BL, &bl) || 
        !read_physical_angle(DOG_SERVO_BR, &br)) {
        return 0.0f;
    }
    
    // Calculate deviation from FIXED baseline (commanded stance)
    float error_bl = fabsf(bl - BASELINE_BL);
    float error_br = fabsf(br - BASELINE_BR);
    
    // Use max error
    float max_error = (error_bl > error_br) ? error_bl : error_br;
    
    return max_error;
}

/**
 * @brief Move ONLY back legs down (front legs stay at stance)
 */
static void go_crouch_back_only(void)
{
    // Back legs go down (lower angle = legs push back/down)
    float angle_bl = DOG_STANCE_BACK - PRESSURE_CROUCH_AMOUNT;
    float angle_br = DOG_STANCE_BACK - PRESSURE_CROUCH_AMOUNT;
    
    ESP_LOGI(TAG, "⬇️ BACK DOWN: BL=%.0f° BR=%.0f° (speed=%d)", 
             angle_bl, angle_br, PRESSURE_CROUCH_SPEED);
    
    // Only move back legs - convert to raw angles for right side
    dog_servo_move(DOG_SERVO_BL, angle_bl, PRESSURE_CROUCH_SPEED);
    dog_servo_move(DOG_SERVO_BR, angle_br, PRESSURE_CROUCH_SPEED);
}

/**
 * @brief Return back legs to stance position
 */
static void go_stance_back_only(void)
{
    ESP_LOGI(TAG, "⬆️ BACK UP: BL=%.0f° BR=%.0f° (speed=%d)", 
             DOG_STANCE_BACK, DOG_STANCE_BACK, PRESSURE_RETURN_SPEED);
    
    // Only move back legs back to stance
    dog_servo_move(DOG_SERVO_BL, DOG_STANCE_BACK, PRESSURE_RETURN_SPEED);
    dog_servo_move(DOG_SERVO_BR, DOG_STANCE_BACK, PRESSURE_RETURN_SPEED);
}

// ═══════════════════════════════════════════════════════
// MAIN UPDATE
// ═══════════════════════════════════════════════════════

static void process_state_machine(void)
{
    uint32_t time_ms = time_in_state_ms();
    
    switch (s_state) {
        case STATE_INIT:
            // Wait for initial settling
            if (time_ms > 1000) {
                ESP_LOGI(TAG, "✓ Initialized, baseline: BL=%.0f° BR=%.0f°", 
                         BASELINE_BL, BASELINE_BR);
                change_state(STATE_MONITORING);
            }
            break;
            
        case STATE_MONITORING: {
            // Get pressure error directly (no filtering to avoid residual)
            float error = get_pressure_error();
            
            // Apply deadzone
            if (error < PRESSURE_DEADZONE) {
                error = 0.0f;
            }
            
            // Log occasionally
            static TickType_t last_log = 0;
            if (pdTICKS_TO_MS(xTaskGetTickCount() - last_log) > 1000) {
                ESP_LOGI(TAG, "Monitoring - Error: %.2f° (threshold: %.2f°)", 
                         error, PRESSURE_THRESHOLD);
                last_log = xTaskGetTickCount();
            }
            
            // Check for pressure (only if above threshold)
            if (error > PRESSURE_THRESHOLD) {
                ESP_LOGI(TAG, "🔴 PRESSURE! Error: %.2f°", error);
                go_crouch_back_only();
                change_state(STATE_CROUCHING);
            }
            break;
        }
        
        case STATE_CROUCHING:
            // Wait for crouch movement to complete
            if (time_ms > 350) {
                change_state(STATE_HOLDING);
            }
            break;
            
        case STATE_HOLDING:
            // Hold the crouch for a moment
            if (time_ms > PRESSURE_HOLD_MS) {
                go_stance_back_only();
                change_state(STATE_RETURNING);
            }
            break;
            
        case STATE_RETURNING:
            // Wait for return movement to complete
            if (time_ms > PRESSURE_RETURN_WAIT_MS) {
                ESP_LOGI(TAG, "⏳ Cooldown %dms...", PRESSURE_COOLDOWN_MS);
                change_state(STATE_COOLDOWN);
            }
            break;
            
        case STATE_COOLDOWN:
            // Long cooldown - don't check pressure at all during this time
            if (time_ms > PRESSURE_COOLDOWN_MS) {
                ESP_LOGI(TAG, "✓ Ready");
                change_state(STATE_MONITORING);
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void pressure_balance_init(void)
{
    ESP_LOGI(TAG, "════════════════════════════════════════");
    ESP_LOGI(TAG, "Pressure Balance System (Back Legs Only)");
    ESP_LOGI(TAG, "  Threshold: %.2f°", PRESSURE_THRESHOLD);
    ESP_LOGI(TAG, "  Back crouch: %.1f° @ speed %d", PRESSURE_CROUCH_AMOUNT, PRESSURE_CROUCH_SPEED);
    ESP_LOGI(TAG, "  Return: speed %d", PRESSURE_RETURN_SPEED);
    ESP_LOGI(TAG, "  Hold: %dms, Cooldown: %dms", PRESSURE_HOLD_MS, PRESSURE_COOLDOWN_MS);
    ESP_LOGI(TAG, "════════════════════════════════════════");
    
    s_last_update = xTaskGetTickCount();
    s_state = STATE_INIT;
    s_state_start_time = xTaskGetTickCount();
    s_initialized = true;
    s_enabled = PRESSURE_BALANCE_ENABLED_DEFAULT;
}

void pressure_balance_update(void)
{
    if (!s_initialized || !s_enabled) {
        return;
    }
    
    TickType_t now = xTaskGetTickCount();
    
    // Rate limit updates
    if (pdTICKS_TO_MS(now - s_last_update) < PRESSURE_UPDATE_MS) {
        return;
    }
    s_last_update = now;
    
    process_state_machine();
}

void pressure_balance_enable(bool enable)
{
    if (enable && !s_enabled) {
        s_state = STATE_COOLDOWN;  // Start with cooldown
        s_state_start_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "Pressure balance ENABLED");
    } else if (!enable && s_enabled) {
        go_stance_back_only();
        ESP_LOGI(TAG, "Pressure balance DISABLED");
    }
    s_enabled = enable;
}

bool pressure_balance_is_enabled(void)
{
    return s_enabled;
}

bool pressure_balance_is_crouched(void)
{
    return (s_state == STATE_CROUCHING || s_state == STATE_HOLDING);
}

float pressure_balance_get_offset(void)
{
    if (s_state == STATE_CROUCHING || s_state == STATE_HOLDING) {
        return PRESSURE_CROUCH_AMOUNT;
    }
    return 0.0f;
}