

#include "reaction_config.h"
#include "gyro_balance.h"
#include "walk_forward_reaction.h"
#include "servo/servo_pressure.h"
#include "dog_config.h"
#include "sts3032_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "REACTION";

// ═══════════════════════════════════════════════════════
// PRESSURE BALANCE CONFIGURATION
// ═══════════════════════════════════════════════════════

#define CROUCH_AMOUNT           45.0f   // How much back legs go down
#define CROUCH_SPEED            700     // Speed to crouch down
#define RETURN_SPEED            700     // Speed to return to stance
#define CROUCH_HOLD_MS          400     // Hold crouch for this long
#define CROUCH_RETURN_WAIT_MS   400     // Wait for servos to finish returning
#define CROUCH_COOLDOWN_MS      1500    // Cooldown after crouch sequence

// ═══════════════════════════════════════════════════════
// POST-MOVEMENT SETTLE TIME
// ═══════════════════════════════════════════════════════

#define POST_WALK_SETTLE_MS     1000    // Wait after walk before checking pressure

// ═══════════════════════════════════════════════════════
// DELTA-BASED DETECTION
// ═══════════════════════════════════════════════════════

#define MIN_PRESSURE_DELTA      0.12f   // Minimum delta to react (ignore tiny movements)
#define DELTA_DOMINANCE_MARGIN  0.09f   // One side must be this much bigger to "win"
#define PRESSURE_CONFIRM_COUNT  1       // Consecutive readings needed

// ═══════════════════════════════════════════════════════
// CROUCH STATE MACHINE
// ═══════════════════════════════════════════════════════

typedef enum {
    CROUCH_STATE_IDLE,       // Not crouching, monitoring
    CROUCH_STATE_CROUCHING,  // Moving down
    CROUCH_STATE_HOLDING,    // Holding crouch position
    CROUCH_STATE_RETURNING,  // Moving back up
    CROUCH_STATE_COOLDOWN    // Waiting before next trigger
} crouch_state_t;

static crouch_state_t s_crouch_state = CROUCH_STATE_IDLE;
static TickType_t s_crouch_state_start = 0;

// ═══════════════════════════════════════════════════════
// STATE TRACKING
// ═══════════════════════════════════════════════════════

static TickType_t last_walk_reaction_time = 0;
static TickType_t last_walk_complete_time = 0;
static bool initialized = false;

// Previous acceleration reading for delta calculation
static float prev_accel_x = 0.0f;
static bool has_prev_reading = false;

// Track if we already handled this pressure event
static bool pressure_handled = false;

// Consecutive reading counters
static int front_dominant_count = 0;
static int back_dominant_count = 0;

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

static const char* crouch_state_name(crouch_state_t state)
{
    switch (state) {
        case CROUCH_STATE_IDLE:      return "IDLE";
        case CROUCH_STATE_CROUCHING: return "CROUCHING";
        case CROUCH_STATE_HOLDING:   return "HOLDING";
        case CROUCH_STATE_RETURNING: return "RETURNING";
        case CROUCH_STATE_COOLDOWN:  return "COOLDOWN";
        default:                     return "UNKNOWN";
    }
}

static void change_crouch_state(crouch_state_t new_state)
{
    if (s_crouch_state != new_state) {
        ESP_LOGI(TAG, "Crouch: %s -> %s",
                 crouch_state_name(s_crouch_state),
                 crouch_state_name(new_state));
        s_crouch_state = new_state;
        s_crouch_state_start = xTaskGetTickCount();
    }
}

static uint32_t crouch_time_in_state_ms(void)
{
    return pdTICKS_TO_MS(xTaskGetTickCount() - s_crouch_state_start);
}

static bool is_walk_cooldown_expired(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_walk_reaction_time);
    return elapsed_ms >= REACTION_COOLDOWN_MS;
}

static void update_walk_reaction_time(void)
{
    last_walk_reaction_time = xTaskGetTickCount();
}

static bool is_settled_after_walk(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_walk_complete_time);
    return elapsed_ms >= POST_WALK_SETTLE_MS;
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
 * @brief Get delta from stance for front legs (max of FL, FR)
 */
static float get_front_delta(void)
{
    float fl = 0, fr = 0;
    
    if (!read_physical_angle(DOG_SERVO_FL, &fl) ||
        !read_physical_angle(DOG_SERVO_FR, &fr)) {
        return 0.0f;
    }
    
    float delta_fl = fabsf(fl - DOG_STANCE_FRONT);
    float delta_fr = fabsf(fr - DOG_STANCE_FRONT);
    
    return (delta_fl > delta_fr) ? delta_fl : delta_fr;
}

/**
 * @brief Get delta from stance for back legs (max of BL, BR)
 */
static float get_back_delta(void)
{
    float bl = 0, br = 0;
    
    if (!read_physical_angle(DOG_SERVO_BL, &bl) ||
        !read_physical_angle(DOG_SERVO_BR, &br)) {
        return 0.0f;
    }
    
    float delta_bl = fabsf(bl - DOG_STANCE_BACK);
    float delta_br = fabsf(br - DOG_STANCE_BACK);
    
    return (delta_bl > delta_br) ? delta_bl : delta_br;
}

/**
 * @brief Determine which end is being pushed more
 * @return 1 = front dominant, -1 = back dominant, 0 = no clear winner
 */
static int get_pressure_direction(float front_delta, float back_delta)
{
    float max_delta = (front_delta > back_delta) ? front_delta : back_delta;
    
    // Must have minimum movement to trigger anything
    if (max_delta < MIN_PRESSURE_DELTA) {
        return 0;
    }
    
    // Check which side dominates with margin
    if (front_delta > back_delta + DELTA_DOMINANCE_MARGIN) {
        return 1;   // Front wins → walk forward
    }
    
    if (back_delta > front_delta + DELTA_DOMINANCE_MARGIN) {
        return -1;  // Back wins → crouch
    }
    
    return 0;  // Too close to call
}

/**
 * @brief Move ONLY back legs down
 */
static void crouch_back_legs(void)
{
    float angle_bl = DOG_STANCE_BACK - CROUCH_AMOUNT;
    float angle_br = DOG_STANCE_BACK - CROUCH_AMOUNT;
    
    ESP_LOGI(TAG, "⬇️ BACK DOWN: BL=%.0f° BR=%.0f°", angle_bl, angle_br);
    
    dog_servo_move(DOG_SERVO_BL, angle_bl, CROUCH_SPEED);
    dog_servo_move(DOG_SERVO_BR, angle_br, CROUCH_SPEED);
}

/**
 * @brief Return back legs to stance position
 */
static void stance_back_legs(void)
{
    ESP_LOGI(TAG, "⬆️ BACK UP: BL=%.0f° BR=%.0f°", DOG_STANCE_BACK, DOG_STANCE_BACK);
    
    dog_servo_move(DOG_SERVO_BL, DOG_STANCE_BACK, RETURN_SPEED);
    dog_servo_move(DOG_SERVO_BR, DOG_STANCE_BACK, RETURN_SPEED);
}

/**
 * @brief Check if crouching is in progress
 */
static bool is_crouching_active(void)
{
    return s_crouch_state != CROUCH_STATE_IDLE;
}

// ═══════════════════════════════════════════════════════
// CROUCH STATE MACHINE
// ═══════════════════════════════════════════════════════

static void process_crouch_state_machine(bool trigger_crouch)
{
    uint32_t time_ms = crouch_time_in_state_ms();
    
    switch (s_crouch_state) {
        case CROUCH_STATE_IDLE:
            if (trigger_crouch) {
                crouch_back_legs();
                change_crouch_state(CROUCH_STATE_CROUCHING);
            }
            break;
            
        case CROUCH_STATE_CROUCHING:
            if (time_ms > 350) {
                change_crouch_state(CROUCH_STATE_HOLDING);
            }
            break;
            
        case CROUCH_STATE_HOLDING:
            if (time_ms > CROUCH_HOLD_MS) {
                stance_back_legs();
                change_crouch_state(CROUCH_STATE_RETURNING);
            }
            break;
            
        case CROUCH_STATE_RETURNING:
            if (time_ms > CROUCH_RETURN_WAIT_MS) {
                ESP_LOGI(TAG, "⏳ Crouch cooldown %dms...", CROUCH_COOLDOWN_MS);
                change_crouch_state(CROUCH_STATE_COOLDOWN);
            }
            break;
            
        case CROUCH_STATE_COOLDOWN:
            if (time_ms > CROUCH_COOLDOWN_MS) {
                ESP_LOGI(TAG, "✓ Ready");
                change_crouch_state(CROUCH_STATE_IDLE);
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void reaction_notify_walk_complete(void)
{
    last_walk_complete_time = xTaskGetTickCount();
    ESP_LOGI(TAG, "Walk complete, settling for %dms...", POST_WALK_SETTLE_MS);
}

void reaction_init(void)
{
    ESP_LOGI(TAG, "════════════════════════════════════════");
    ESP_LOGI(TAG, "Reaction System (Delta Comparison Mode)");
    ESP_LOGI(TAG, "  Compare front vs back delta from stance");
    ESP_LOGI(TAG, "  Min delta to react: %.2f°", MIN_PRESSURE_DELTA);
    ESP_LOGI(TAG, "  Dominance margin: %.2f°", DELTA_DOMINANCE_MARGIN);
    ESP_LOGI(TAG, "  Confirm count: %d readings", PRESSURE_CONFIRM_COUNT);
    ESP_LOGI(TAG, "  Front > Back → Walk forward");
    ESP_LOGI(TAG, "  Back > Front → Crouch down");
    ESP_LOGI(TAG, "  IMU push → Walk 3 cycles");
    ESP_LOGI(TAG, "════════════════════════════════════════");
    
    prev_accel_x = 0.0f;
    has_prev_reading = false;
    pressure_handled = false;
    s_crouch_state = CROUCH_STATE_IDLE;
    s_crouch_state_start = xTaskGetTickCount();
    last_walk_complete_time = 0;
    front_dominant_count = 0;
    back_dominant_count = 0;
    
    gyro_balance_init();
    
    initialized = true;
}

void reaction_process_imu(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    gyro_balance_process(data);
    
    if (gyro_balance_is_enabled()) {
        return;
    }
    
    if (is_crouching_active()) {
        return;
    }
    
    if (!is_walk_cooldown_expired()) {
        return;
    }
    
    float current_accel_x = data->accel_x;
    
    if (!has_prev_reading) {
        prev_accel_x = current_accel_x;
        has_prev_reading = true;
        return;
    }
    
    float delta = current_accel_x - prev_accel_x;
    prev_accel_x = current_accel_x;
    
    // Front push via IMU
    if (delta >= REACTION_DELTA_THRESHOLD && current_accel_x >= REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "🏃 IMU front push! (delta: +%.2f, accel: %.2f) → 3 cycles",
                 delta, current_accel_x);
        update_walk_reaction_time();
        pressure_handled = true;
        walk_forward_play(3);
        return;
    }
    
    // Back push via IMU
    if (delta <= -REACTION_DELTA_THRESHOLD && current_accel_x <= -REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "⬅️ IMU back push! (delta: %.2f, accel: %.2f) → 3 cycles",
                 delta, current_accel_x);
        update_walk_reaction_time();
        pressure_handled = true;
        walk_forward_play(3);
        return;
    }
}

// ═══════════════════════════════════════════════════════
// PRESSURE DETECTION TASK (100Hz)
// ═══════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════
// PRESSURE DETECTION TASK (100Hz)
// ═══════════════════════════════════════════════════════

void reaction_pressure_task(void *arg)
{
    ESP_LOGI(TAG, "Pressure task started (100Hz)");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Pressure monitoring active");
    
    while (1) {
        if (!initialized) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        if (gyro_balance_is_enabled()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        if (!is_settled_after_walk()) {
            // Reset state when settling after walk
            front_dominant_count = 0;
            back_dominant_count = 0;
            pressure_handled = false;  // <-- Reset here too!
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Get deltas from stance
        float front_delta = get_front_delta();
        float back_delta = get_back_delta();
        
        // Determine direction
        int direction = get_pressure_direction(front_delta, back_delta);
        
        // Log occasionally
        static TickType_t last_log = 0;
        if (pdTICKS_TO_MS(xTaskGetTickCount() - last_log) > 2000) {
            const char *dir_str = (direction == 1) ? "FRONT" : 
                                  (direction == -1) ? "BACK" : "none";
            ESP_LOGI(TAG, "Delta: Front=%.2f° Back=%.2f° → %s | State:%s | handled=%d cnt=%d",
                     front_delta, back_delta, dir_str,
                     crouch_state_name(s_crouch_state),
                     pressure_handled, front_dominant_count);
            last_log = xTaskGetTickCount();
        }
        
        // ═══════════════════════════════════════════════════
        // PROCESS PRESSURE DIRECTION
        // ═══════════════════════════════════════════════════
        
        bool trigger_crouch = false;
        
        if (s_crouch_state == CROUCH_STATE_IDLE) {
            // Count consecutive readings for each direction
            if (direction == 1) {
                front_dominant_count++;
                back_dominant_count = 0;
            } else if (direction == -1) {
                back_dominant_count++;
                front_dominant_count = 0;
            } else {
                front_dominant_count = 0;
                back_dominant_count = 0;
                pressure_handled = false;
            }
            
            // Check for confirmed triggers (only if not already handled)
            if (!pressure_handled) {
                if (front_dominant_count >= PRESSURE_CONFIRM_COUNT) {
                    ESP_LOGI(TAG, "🚶 FRONT wins! (F=%.2f° > B=%.2f°) → Walk",
                             front_delta, back_delta);
                    pressure_handled = true;
                    front_dominant_count = 0;
                    update_walk_reaction_time();
                    walk_forward_play(1);
                }
                else if (back_dominant_count >= PRESSURE_CONFIRM_COUNT) {
                    ESP_LOGI(TAG, "🔴 BACK wins! (B=%.2f° > F=%.2f°) → Crouch",
                             back_delta, front_delta);
                    pressure_handled = true;
                    back_dominant_count = 0;
                    trigger_crouch = true;
                }
            }
        }
        
        process_crouch_state_machine(trigger_crouch);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}