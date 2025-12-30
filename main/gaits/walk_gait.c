/**
 * @file walk_gait.c
 * @brief Horse Walk Gait Implementation
 * 
 * 4-beat walk gait - each leg moves independently in sequence:
 *   Phase 0: Front-Left (2) swings forward
 *   Phase 1: Back-Right (3) swings forward
 *   Phase 2: Front-Right (1) swings forward
 *   Phase 3: Back-Left (4) swings forward
 * 
 * This creates a natural horse walking pattern.
 * Always 3 legs on the ground for maximum stability.
 */

#include "walk_gait.h"
#include "sts3032_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "WALK_GAIT";

// ═══════════════════════════════════════════════════════
// INTERNAL STATE
// ═══════════════════════════════════════════════════════

static walk_gait_config_t s_config;
static gait_direction_t s_direction = GAIT_DIRECTION_STOP;
static TaskHandle_t s_gait_task_handle = NULL;
static volatile bool s_running = false;
static uint8_t s_phase = 0;

// Leg sequence for walk gait: FL, BR, FR, BL (diagonal pairs alternating sides)
static const uint8_t WALK_SEQUENCE[4] = {
    SERVO_FRONT_LEFT,   // Phase 0
    SERVO_BACK_RIGHT,   // Phase 1
    SERVO_FRONT_RIGHT,  // Phase 2
    SERVO_BACK_LEFT     // Phase 3
};

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

static float get_stance_angle(uint8_t servo_id)
{
    switch (servo_id) {
        case SERVO_FRONT_RIGHT: return s_config.stance_angle_fr;
        case SERVO_FRONT_LEFT:  return s_config.stance_angle_fl;
        case SERVO_BACK_RIGHT:  return s_config.stance_angle_br;
        case SERVO_BACK_LEFT:   return s_config.stance_angle_bl;
        default: return 180.0f;
    }
}

/**
 * @brief Calculate swing angle for a leg moving forward
 * 
 * Right legs (1,3): CW (+) = backward, so forward swing = stance - amplitude
 * Left legs (2,4):  CW (+) = forward, so forward swing = stance + amplitude
 */
static float calc_swing_forward(uint8_t servo_id)
{
    float stance = get_stance_angle(servo_id);
    float amp = s_config.swing_amplitude;
    
    if (servo_id == SERVO_FRONT_RIGHT || servo_id == SERVO_BACK_RIGHT) {
        return stance - amp;  // Right legs: CCW for forward
    } else {
        return stance + amp;  // Left legs: CW for forward
    }
}

/**
 * @brief Calculate push-back angle for a leg
 */
static float calc_push_back(uint8_t servo_id)
{
    float stance = get_stance_angle(servo_id);
    float amp = s_config.swing_amplitude;
    
    if (servo_id == SERVO_FRONT_RIGHT || servo_id == SERVO_BACK_RIGHT) {
        return stance + amp;  // Right legs: CW for backward
    } else {
        return stance - amp;  // Left legs: CCW for backward
    }
}

static void move_servo(uint8_t id, float angle)
{
    sts_servo_set_angle(id, angle, s_config.servo_speed);
}

static void goto_stance_internal(void)
{
    move_servo(SERVO_FRONT_RIGHT, s_config.stance_angle_fr);
    move_servo(SERVO_FRONT_LEFT, s_config.stance_angle_fl);
    move_servo(SERVO_BACK_RIGHT, s_config.stance_angle_br);
    move_servo(SERVO_BACK_LEFT, s_config.stance_angle_bl);
    
    ESP_LOGI(TAG, "All servos to stance");
}

// ═══════════════════════════════════════════════════════
// GAIT EXECUTION
// ═══════════════════════════════════════════════════════

/**
 * @brief Execute one phase of walk gait
 * 
 * Only one leg moves at a time (the "swing" leg).
 * Other legs hold their positions (3 in stance/push).
 */
static void execute_phase(uint8_t phase, gait_direction_t direction)
{
    if (direction == GAIT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    bool forward = (direction == GAIT_DIRECTION_FORWARD);
    uint8_t swing_leg = WALK_SEQUENCE[phase];
    
    // Set all legs to their appropriate positions
    for (int i = 0; i < 4; i++) {
        uint8_t leg = WALK_SEQUENCE[i];
        float angle;
        
        if (i == phase) {
            // This leg is swinging
            angle = forward ? calc_swing_forward(leg) : calc_push_back(leg);
        } else {
            // Other legs push back (or forward if reversing)
            angle = forward ? calc_push_back(leg) : calc_swing_forward(leg);
            // Reduce amplitude for supporting legs to maintain stability
            float stance = get_stance_angle(leg);
            angle = stance + (angle - stance) * 0.5f;
        }
        
        move_servo(leg, angle);
    }
    
    ESP_LOGD(TAG, "Walk phase %d: swing leg %d", phase, swing_leg);
}

static void gait_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Walk gait task started");
    
    while (s_running) {
        gait_direction_t dir = s_direction;
        
        if (dir == GAIT_DIRECTION_STOP) {
            goto_stance_internal();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        execute_phase(s_phase, dir);
        vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms));
        s_phase = (s_phase + 1) % 4;
    }
    
    goto_stance_internal();
    ESP_LOGI(TAG, "Walk gait task stopped");
    s_gait_task_handle = NULL;
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool walk_gait_init(const walk_gait_config_t *config)
{
    ESP_LOGI(TAG, "Initializing walk gait controller");
    
    if (config != NULL) {
        s_config = *config;
    } else {
        walk_gait_config_t default_config = WALK_GAIT_DEFAULT_CONFIG();
        s_config = default_config;
    }
    
    ESP_LOGI(TAG, "Config: FR=%.0f° FL=%.0f° BR=%.0f° BL=%.0f° amp=%.1f°",
             s_config.stance_angle_fr, s_config.stance_angle_fl,
             s_config.stance_angle_br, s_config.stance_angle_bl,
             s_config.swing_amplitude);
    
    bool all_ok = true;
    for (uint8_t id = 1; id <= 4; id++) {
        if (!sts_servo_ping(id)) {
            ESP_LOGE(TAG, "Servo ID %d not responding", id);
            all_ok = false;
        } else {
            sts_servo_enable_torque(id, true);
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    goto_stance_internal();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    s_direction = GAIT_DIRECTION_STOP;
    s_phase = 0;
    
    return all_ok;
}

void walk_gait_set_config(const walk_gait_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
    }
}

void walk_gait_get_config(walk_gait_config_t *config)
{
    if (config != NULL) {
        *config = s_config;
    }
}

bool walk_gait_start(gait_direction_t direction)
{
    if (s_gait_task_handle != NULL) {
        s_direction = direction;
        return true;
    }
    
    s_direction = direction;
    s_running = true;
    s_phase = 0;
    
    BaseType_t ret = xTaskCreate(gait_task, "walk_gait", 4096, NULL, 5, &s_gait_task_handle);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gait task");
        s_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Walk gait started");
    return true;
}

void walk_gait_stop(void)
{
    if (!s_running) return;
    
    ESP_LOGI(TAG, "Stopping walk gait");
    s_running = false;
    
    while (s_gait_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void walk_gait_set_direction(gait_direction_t direction)
{
    s_direction = direction;
}

gait_direction_t walk_gait_get_direction(void)
{
    return s_direction;
}

bool walk_gait_is_running(void)
{
    return s_running;
}

void walk_gait_goto_stance(void)
{
    goto_stance_internal();
}

void walk_gait_step(gait_direction_t direction)
{
    if (direction == GAIT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    execute_phase(s_phase, direction);
    vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms));
    s_phase = (s_phase + 1) % 4;
}
