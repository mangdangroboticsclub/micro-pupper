/**
 * @file trot_gait.c
 * @brief Trot Gait Walking Algorithm Implementation
 * 
 * Trot gait moves diagonal leg pairs in synchrony:
 *   - Phase A: Front-Right + Back-Left swing forward
 *   - Phase B: Front-Left + Back-Right swing forward
 * 
 * Servo arrangement and positive angle meaning:
 *   ID 1 - Front Right - Clockwise (+) = leg moves backward
 *   ID 2 - Front Left  - Clockwise (+) = leg moves forward  
 *   ID 3 - Back Right  - Clockwise (+) = leg moves backward
 *   ID 4 - Back Left   - Clockwise (+) = leg moves forward
 */

#include "trot_gait.h"
#include "sts3032_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TROT_GAIT";

// ═══════════════════════════════════════════════════════
// INTERNAL STATE
// ═══════════════════════════════════════════════════════

static trot_gait_config_t s_config;
static trot_direction_t s_direction = TROT_DIRECTION_STOP;
static TaskHandle_t s_gait_task_handle = NULL;
static volatile bool s_running = false;

// Current phase of the gait (0 or 1)
static uint8_t s_phase = 0;

// ═══════════════════════════════════════════════════════
// ANGLE CALCULATION HELPERS
// ═══════════════════════════════════════════════════════

/**
 * @brief Calculate servo angles for diagonal pair A (FR + BL)
 * 
 * For FORWARD motion:
 *   - Front Right (1): CW (+) = leg backward, so swing forward = stance - amplitude
 *   - Back Left (4):   CW (+) = leg forward, so swing forward = stance + amplitude
 * 
 * @param forward_swing true if this pair is swinging forward (the "swing" phase)
 * @param angle_fr Output angle for front-right servo
 * @param angle_bl Output angle for back-left servo
 */
static void calc_pair_a_angles(bool forward_swing, float *angle_fr, float *angle_bl)
{
    float offset = s_config.swing_amplitude;
    
    if (forward_swing) {
        // Pair A swinging forward
        // FR: CW = backward, so forward = CCW = negative offset
        // BL: CW = forward, so forward = CW = positive offset
        *angle_fr = s_config.stance_angle_fr - offset;
        *angle_bl = s_config.stance_angle_bl + offset;
    } else {
        // Pair A pushing back (stance phase)
        // FR: push back = CW = positive offset
        // BL: push back = CCW = negative offset
        *angle_fr = s_config.stance_angle_fr + offset;
        *angle_bl = s_config.stance_angle_bl - offset;
    }
}

/**
 * @brief Calculate servo angles for diagonal pair B (FL + BR)
 * 
 * For FORWARD motion:
 *   - Front Left (2):  CW (+) = leg forward, so swing forward = stance + amplitude
 *   - Back Right (3):  CW (+) = leg backward, so swing forward = stance - amplitude
 * 
 * @param forward_swing true if this pair is swinging forward
 * @param angle_fl Output angle for front-left servo
 * @param angle_br Output angle for back-right servo
 */
static void calc_pair_b_angles(bool forward_swing, float *angle_fl, float *angle_br)
{
    float offset = s_config.swing_amplitude;
    
    if (forward_swing) {
        // Pair B swinging forward
        // FL: CW = forward, so forward = CW = positive offset
        // BR: CW = backward, so forward = CCW = negative offset
        *angle_fl = s_config.stance_angle_fl + offset;
        *angle_br = s_config.stance_angle_br - offset;
    } else {
        // Pair B pushing back (stance phase)
        // FL: push back = CCW = negative offset
        // BR: push back = CW = positive offset
        *angle_fl = s_config.stance_angle_fl - offset;
        *angle_br = s_config.stance_angle_br + offset;
    }
}

// ═══════════════════════════════════════════════════════
// SERVO CONTROL HELPERS
// ═══════════════════════════════════════════════════════

/**
 * @brief Move a servo to the specified angle
 */
static void move_servo(uint8_t id, float angle)
{
    sts_servo_set_angle(id, angle, s_config.servo_speed);
}

/**
 * @brief Move all servos to stance position
 */
static void goto_stance_internal(void)
{
    move_servo(SERVO_FRONT_RIGHT, s_config.stance_angle_fr);
    move_servo(SERVO_FRONT_LEFT, s_config.stance_angle_fl);
    move_servo(SERVO_BACK_RIGHT, s_config.stance_angle_br);
    move_servo(SERVO_BACK_LEFT, s_config.stance_angle_bl);
    
    ESP_LOGI(TAG, "Servos to stance: FR=%.0f° FL=%.0f° BR=%.0f° BL=%.0f°",
             s_config.stance_angle_fr, s_config.stance_angle_fl,
             s_config.stance_angle_br, s_config.stance_angle_bl);
}

// ═══════════════════════════════════════════════════════
// GAIT EXECUTION
// ═══════════════════════════════════════════════════════

/**
 * @brief Execute one phase of the trot gait
 * 
 * In trot gait:
 *   Phase 0: Pair A (FR+BL) swings forward, Pair B (FL+BR) pushes back
 *   Phase 1: Pair B (FL+BR) swings forward, Pair A (FR+BL) pushes back
 * 
 * For backward motion, the swing/push is reversed.
 */
static void execute_phase(uint8_t phase, trot_direction_t direction)
{
    float angle_fr, angle_fl, angle_br, angle_bl;
    
    if (direction == TROT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    bool forward = (direction == TROT_DIRECTION_FORWARD);
    
    if (phase == 0) {
        // Phase 0: Pair A swings, Pair B pushes
        calc_pair_a_angles(forward, &angle_fr, &angle_bl);
        calc_pair_b_angles(!forward, &angle_fl, &angle_br);
    } else {
        // Phase 1: Pair B swings, Pair A pushes
        calc_pair_a_angles(!forward, &angle_fr, &angle_bl);
        calc_pair_b_angles(forward, &angle_fl, &angle_br);
    }
    
    // Move all servos simultaneously
    move_servo(SERVO_FRONT_RIGHT, angle_fr);
    move_servo(SERVO_FRONT_LEFT, angle_fl);
    move_servo(SERVO_BACK_RIGHT, angle_br);
    move_servo(SERVO_BACK_LEFT, angle_bl);
    
    ESP_LOGD(TAG, "Phase %d: FR=%.1f° FL=%.1f° BR=%.1f° BL=%.1f°",
             phase, angle_fr, angle_fl, angle_br, angle_bl);
}

/**
 * @brief FreeRTOS task that runs the gait continuously
 */
static void gait_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Trot gait task started");
    
    while (s_running) {
        trot_direction_t dir = s_direction;
        
        if (dir == TROT_DIRECTION_STOP) {
            // When stopped, go to stance and wait
            goto_stance_internal();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Execute current phase
        execute_phase(s_phase, dir);
        
        // Wait for step duration
        vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms));
        
        // Advance to next phase
        s_phase = (s_phase + 1) % 2;
    }
    
    // Return to stance before exiting
    goto_stance_internal();
    
    ESP_LOGI(TAG, "Trot gait task stopped");
    s_gait_task_handle = NULL;
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════
// PUBLIC API - INITIALIZATION
// ═══════════════════════════════════════════════════════

bool trot_gait_init(const trot_gait_config_t *config)
{
    ESP_LOGI(TAG, "Initializing trot gait controller");
    
    // Apply configuration
    if (config != NULL) {
        s_config = *config;
    } else {
        trot_gait_config_t default_config = TROT_GAIT_DEFAULT_CONFIG();
        s_config = default_config;
    }
    
    ESP_LOGI(TAG, "Config: FR=%.0f° FL=%.0f° BR=%.0f° BL=%.0f° amp=%.1f° step=%dms",
             s_config.stance_angle_fr, s_config.stance_angle_fl,
             s_config.stance_angle_br, s_config.stance_angle_bl,
             s_config.swing_amplitude, s_config.step_duration_ms);
    
    // Verify all servos are responding
    bool all_ok = true;
    for (uint8_t id = 1; id <= 4; id++) {
        if (!sts_servo_ping(id)) {
            ESP_LOGE(TAG, "Servo ID %d not responding", id);
            all_ok = false;
        } else {
            // Enable torque
            sts_servo_enable_torque(id, true);
            ESP_LOGI(TAG, "Servo ID %d ready", id);
        }
    }
    
    if (!all_ok) {
        ESP_LOGW(TAG, "Some servos not responding, continuing anyway");
    }
    
    // Move to stance position
    vTaskDelay(pdMS_TO_TICKS(100));
    goto_stance_internal();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    s_direction = TROT_DIRECTION_STOP;
    s_phase = 0;
    
    ESP_LOGI(TAG, "Trot gait controller initialized");
    return all_ok;
}

void trot_gait_set_config(const trot_gait_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
        ESP_LOGI(TAG, "Config updated: FR=%.0f° FL=%.0f° BR=%.0f° BL=%.0f° amp=%.1f° step=%dms",
                 s_config.stance_angle_fr, s_config.stance_angle_fl,
                 s_config.stance_angle_br, s_config.stance_angle_bl,
                 s_config.swing_amplitude, s_config.step_duration_ms);
    }
}

void trot_gait_get_config(trot_gait_config_t *config)
{
    if (config != NULL) {
        *config = s_config;
    }
}

// ═══════════════════════════════════════════════════════
// PUBLIC API - GAIT CONTROL
// ═══════════════════════════════════════════════════════

bool trot_gait_start(trot_direction_t direction)
{
    if (s_gait_task_handle != NULL) {
        ESP_LOGW(TAG, "Gait already running, updating direction");
        s_direction = direction;
        return true;
    }
    
    s_direction = direction;
    s_running = true;
    s_phase = 0;
    
    BaseType_t ret = xTaskCreate(
        gait_task,
        "trot_gait",
        4096,
        NULL,
        5,
        &s_gait_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gait task");
        s_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Trot gait started, direction=%d", direction);
    return true;
}

void trot_gait_stop(void)
{
    if (!s_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping trot gait");
    s_running = false;
    
    // Wait for task to finish
    while (s_gait_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Trot gait stopped");
}

void trot_gait_set_direction(trot_direction_t direction)
{
    s_direction = direction;
    ESP_LOGI(TAG, "Direction set to %d", direction);
}

trot_direction_t trot_gait_get_direction(void)
{
    return s_direction;
}

bool trot_gait_is_running(void)
{
    return s_running;
}

// ═══════════════════════════════════════════════════════
// PUBLIC API - DIRECT CONTROL
// ═══════════════════════════════════════════════════════

void trot_gait_goto_stance(void)
{
    goto_stance_internal();
}

void trot_gait_step(trot_direction_t direction)
{
    if (direction == TROT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    execute_phase(s_phase, direction);
    vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms));
    s_phase = (s_phase + 1) % 2;
}
