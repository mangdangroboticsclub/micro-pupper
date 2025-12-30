/**
 * @file creep_gait.c
 * @brief Creep Gait Implementation
 * 
 * Slow, maximally stable 4-beat gait with weight shifting.
 * Sequence: FL -> BR -> FR -> BL
 * 
 * Each step has sub-phases:
 *   1. Shift weight (other legs adjust to stabilize)
 *   2. Swing leg moves forward
 *   3. Leg plants, weight normalizes
 * 
 * This provides the highest stability margin of all gaits.
 */

#include "creep_gait.h"
#include "sts3032_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CREEP_GAIT";

// ═══════════════════════════════════════════════════════
// INTERNAL STATE
// ═══════════════════════════════════════════════════════

static creep_gait_config_t s_config;
static gait_direction_t s_direction = GAIT_DIRECTION_STOP;
static TaskHandle_t s_gait_task_handle = NULL;
static volatile bool s_running = false;
static uint8_t s_phase = 0;

// Leg sequence for creep: FL, BR, FR, BL
static const uint8_t CREEP_SEQUENCE[4] = {
    SERVO_FRONT_LEFT,
    SERVO_BACK_RIGHT,
    SERVO_FRONT_RIGHT,
    SERVO_BACK_LEFT
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

static float calc_swing_forward(uint8_t servo_id)
{
    float stance = get_stance_angle(servo_id);
    float amp = s_config.swing_amplitude;
    
    if (servo_id == SERVO_FRONT_RIGHT || servo_id == SERVO_BACK_RIGHT) {
        return stance - amp;
    } else {
        return stance + amp;
    }
}

static float calc_push_back(uint8_t servo_id)
{
    float stance = get_stance_angle(servo_id);
    float amp = s_config.swing_amplitude;
    
    if (servo_id == SERVO_FRONT_RIGHT || servo_id == SERVO_BACK_RIGHT) {
        return stance + amp;
    } else {
        return stance - amp;
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
 * @brief Execute creep gait with weight shift
 * 
 * For each swinging leg, the opposite legs shift slightly to maintain
 * the center of gravity over the support triangle.
 */
static void execute_phase(uint8_t phase, gait_direction_t direction)
{
    if (direction == GAIT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    bool forward = (direction == GAIT_DIRECTION_FORWARD);
    uint8_t swing_leg = CREEP_SEQUENCE[phase];
    
    // Sub-phase 1: Pre-position supporting legs (weight shift preparation)
    // Supporting legs move slightly back to create stable tripod
    for (int i = 0; i < 4; i++) {
        uint8_t leg = CREEP_SEQUENCE[i];
        if (leg != swing_leg) {
            float stance = get_stance_angle(leg);
            float support_offset = s_config.swing_amplitude * 0.3f;
            
            // Move supporting legs toward push-back position
            float angle;
            if (forward) {
                angle = (leg == SERVO_FRONT_RIGHT || leg == SERVO_BACK_RIGHT) 
                    ? stance + support_offset : stance - support_offset;
            } else {
                angle = (leg == SERVO_FRONT_RIGHT || leg == SERVO_BACK_RIGHT) 
                    ? stance - support_offset : stance + support_offset;
            }
            move_servo(leg, angle);
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms / 3));
    
    // Sub-phase 2: Swing leg moves forward
    float swing_angle = forward ? calc_swing_forward(swing_leg) : calc_push_back(swing_leg);
    move_servo(swing_leg, swing_angle);
    
    vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms / 3));
    
    // Sub-phase 3: Swing leg plants, return to neutral stance slightly
    // All legs settle toward stance
    for (int i = 0; i < 4; i++) {
        uint8_t leg = CREEP_SEQUENCE[i];
        float stance = get_stance_angle(leg);
        float current_offset = s_config.swing_amplitude * 0.4f;
        
        float angle;
        if (leg == swing_leg) {
            // Swinging leg stays extended
            angle = swing_angle;
        } else {
            // Supporting legs hold moderate push position
            if (forward) {
                angle = (leg == SERVO_FRONT_RIGHT || leg == SERVO_BACK_RIGHT) 
                    ? stance + current_offset : stance - current_offset;
            } else {
                angle = (leg == SERVO_FRONT_RIGHT || leg == SERVO_BACK_RIGHT) 
                    ? stance - current_offset : stance + current_offset;
            }
        }
        move_servo(leg, angle);
    }
    
    vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms / 3));
    
    ESP_LOGD(TAG, "Creep phase %d: swing leg %d", phase, swing_leg);
}

static void gait_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Creep gait task started");
    
    while (s_running) {
        gait_direction_t dir = s_direction;
        
        if (dir == GAIT_DIRECTION_STOP) {
            goto_stance_internal();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        execute_phase(s_phase, dir);
        s_phase = (s_phase + 1) % 4;
    }
    
    goto_stance_internal();
    ESP_LOGI(TAG, "Creep gait task stopped");
    s_gait_task_handle = NULL;
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool creep_gait_init(const creep_gait_config_t *config)
{
    ESP_LOGI(TAG, "Initializing creep gait controller");
    
    if (config != NULL) {
        s_config = *config;
    } else {
        creep_gait_config_t default_config = CREEP_GAIT_DEFAULT_CONFIG();
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

void creep_gait_set_config(const creep_gait_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
    }
}

void creep_gait_get_config(creep_gait_config_t *config)
{
    if (config != NULL) {
        *config = s_config;
    }
}

bool creep_gait_start(gait_direction_t direction)
{
    if (s_gait_task_handle != NULL) {
        s_direction = direction;
        return true;
    }
    
    s_direction = direction;
    s_running = true;
    s_phase = 0;
    
    BaseType_t ret = xTaskCreate(gait_task, "creep_gait", 4096, NULL, 5, &s_gait_task_handle);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gait task");
        s_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Creep gait started");
    return true;
}

void creep_gait_stop(void)
{
    if (!s_running) return;
    
    ESP_LOGI(TAG, "Stopping creep gait");
    s_running = false;
    
    while (s_gait_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void creep_gait_set_direction(gait_direction_t direction)
{
    s_direction = direction;
}

gait_direction_t creep_gait_get_direction(void)
{
    return s_direction;
}

bool creep_gait_is_running(void)
{
    return s_running;
}

void creep_gait_goto_stance(void)
{
    goto_stance_internal();
}

void creep_gait_step(gait_direction_t direction)
{
    if (direction == GAIT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    execute_phase(s_phase, direction);
    s_phase = (s_phase + 1) % 4;
}
