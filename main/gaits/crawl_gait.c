/**
 * @file crawl_gait.c
 * @brief Crawl Gait Implementation
 * 
 * Alternating crawl gait - legs alternate sides for straight forward motion:
 *   Phase 0: Back-Left (4) swings forward
 *   Phase 1: Front-Right (1) swings forward  (opposite side!)
 *   Phase 2: Back-Right (3) swings forward
 *   Phase 3: Front-Left (2) swings forward   (opposite side!)
 * 
 * By alternating sides, push forces are balanced for straight-line motion.
 * 
 * Uses dog_config for automatic right-side angle reversal.
 */

#include "crawl_gait.h"
#include "dog_config.h"
#include "sts3032_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CRAWL_GAIT";

// ═══════════════════════════════════════════════════════
// INTERNAL STATE
// ═══════════════════════════════════════════════════════

static crawl_gait_config_t s_config;
static gait_direction_t s_direction = GAIT_DIRECTION_STOP;
static TaskHandle_t s_gait_task_handle = NULL;
static volatile bool s_running = false;
static uint8_t s_phase = 0;

// Different leg sequences for different movement directions
// Forward: alternating sides for balanced straight motion
static const uint8_t SEQ_FORWARD[4] = {
    SERVO_BACK_LEFT,    // Phase 0: Back left
    SERVO_FRONT_RIGHT,  // Phase 1: Front right (alternate side)
    SERVO_BACK_RIGHT,   // Phase 2: Back right
    SERVO_FRONT_LEFT    // Phase 3: Front left (alternate side)
};

// Turn Right: same-side consecutive (left side pushes more)
static const uint8_t SEQ_TURN_RIGHT[4] = {
    SERVO_BACK_LEFT,    // Phase 0
    SERVO_BACK_RIGHT,   // Phase 1
    SERVO_FRONT_LEFT,   // Phase 2
    SERVO_FRONT_RIGHT   // Phase 3
};

// Turn Left: same-side consecutive (right side pushes more) - mirror of turn right
static const uint8_t SEQ_TURN_LEFT[4] = {
    SERVO_BACK_RIGHT,   // Phase 0
    SERVO_BACK_LEFT,    // Phase 1
    SERVO_FRONT_RIGHT,  // Phase 2
    SERVO_FRONT_LEFT    // Phase 3
};

// Track leg positions for wave effect (0.0 = back, 1.0 = forward)
static float s_leg_positions[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * @brief Get the appropriate leg sequence for the current direction
 */
static const uint8_t* get_sequence_for_direction(gait_direction_t direction)
{
    switch (direction) {
        case GAIT_DIRECTION_TURN_LEFT:
            return SEQ_TURN_LEFT;
        case GAIT_DIRECTION_TURN_RIGHT:
            return SEQ_TURN_RIGHT;
        case GAIT_DIRECTION_FORWARD:
        case GAIT_DIRECTION_BACKWARD:
        default:
            return SEQ_FORWARD;
    }
}

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

static int get_leg_index(uint8_t servo_id, const uint8_t* sequence)
{
    for (int i = 0; i < 4; i++) {
        if (sequence[i] == servo_id) return i;
    }
    return 0;
}

/**
 * @brief Calculate angle based on position in swing cycle
 * @param servo_id The servo to calculate for
 * @param position 0.0 = full back, 1.0 = full forward
 * 
 * Uses unified angles - dog_servo_move handles reversal automatically
 */
static float calc_angle_from_position(uint8_t servo_id, float position)
{
    float stance = get_stance_angle(servo_id);
    float amp = s_config.swing_amplitude;
    
    // Position: -1.0 = back, 0.0 = stance, +1.0 = forward
    float offset = (position * 2.0f - 1.0f) * amp;
    
    // Use unified angle convention: positive offset = forward for all legs
    // The dog_servo_move function handles right-side reversal automatically
    return stance + offset;
}

/**
 * @brief Move servo with speed based on direction
 * Forward motion uses fast speed, turning uses medium speed
 */
static void move_servo_for_direction(uint8_t id, float angle, gait_direction_t direction)
{
    uint16_t speed;
    
    if (direction == GAIT_DIRECTION_TURN_LEFT || direction == GAIT_DIRECTION_TURN_RIGHT) {
        speed = DOG_SPEED_MEDIUM;  // Slower for turning
    } else {
        speed = DOG_SPEED_MAX;  // Fast for forward/backward
    }
    
    // Use dog_servo_move for automatic right-side reversal
    dog_servo_move(id, angle, speed);
}

static void goto_stance_internal(void)
{
    // Use dog_servo_move for automatic right-side reversal
    dog_servo_move(SERVO_FRONT_RIGHT, s_config.stance_angle_fr, s_config.servo_speed);
    dog_servo_move(SERVO_FRONT_LEFT, s_config.stance_angle_fl, s_config.servo_speed);
    dog_servo_move(SERVO_BACK_RIGHT, s_config.stance_angle_br, s_config.servo_speed);
    dog_servo_move(SERVO_BACK_LEFT, s_config.stance_angle_bl, s_config.servo_speed);
    
    for (int i = 0; i < 4; i++) {
        s_leg_positions[i] = 0.5f;  // Reset to neutral
    }
    
    ESP_LOGI(TAG, "All servos to stance");
}

// ═══════════════════════════════════════════════════════
// GAIT EXECUTION
// ═══════════════════════════════════════════════════════

/**
 * @brief Execute crawl gait phase
 * 
 * Creates a wave motion where each leg is at a different phase offset.
 * When one leg swings forward, adjacent legs are in different positions
 * creating a smooth wave pattern.
 * 
 * Different sequences are used for different directions:
 *   - Forward/Backward: alternating sides for straight motion
 *   - Turn Left/Right: same-side consecutive for rotation
 */
static void execute_phase(uint8_t phase, gait_direction_t direction)
{
    if (direction == GAIT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    // Determine if we're moving forward (includes turning, which uses forward leg motion)
    bool move_forward = (direction != GAIT_DIRECTION_BACKWARD);
    
    // Get the appropriate sequence for this direction
    const uint8_t* sequence = get_sequence_for_direction(direction);
    
    // The current leg in the sequence swings forward
    uint8_t swing_leg = sequence[phase];
    
    // Update leg positions for wave effect
    for (int i = 0; i < 4; i++) {
        uint8_t leg = sequence[i];
        int idx = get_leg_index(leg, sequence);
        
        if (i == phase) {
            // This leg swings forward
            s_leg_positions[idx] = move_forward ? 1.0f : 0.0f;
        } else {
            // Other legs move toward push-back gradually
            // Calculate how many phases since this leg swung
            int phases_since = (phase - i + 4) % 4;
            
            if (phases_since > 0) {
                // Leg is pushing back proportionally
                float push_amount = (float)phases_since / 4.0f;
                s_leg_positions[idx] = move_forward ? (1.0f - push_amount * 0.8f) : (push_amount * 0.8f);
            }
        }
        
        float angle = calc_angle_from_position(leg, s_leg_positions[idx]);
        move_servo_for_direction(leg, angle, direction);
    }
    
    ESP_LOGD(TAG, "Crawl phase %d: swing leg %d, positions: [%.1f, %.1f, %.1f, %.1f]",
             phase, swing_leg,
             s_leg_positions[0], s_leg_positions[1], 
             s_leg_positions[2], s_leg_positions[3]);
}

static void gait_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Crawl gait task started");
    
    // Initialize positions for smooth start
    for (int i = 0; i < 4; i++) {
        s_leg_positions[i] = 0.5f;
    }
    
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
    ESP_LOGI(TAG, "Crawl gait task stopped");
    s_gait_task_handle = NULL;
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool crawl_gait_init(const crawl_gait_config_t *config)
{
    ESP_LOGI(TAG, "Initializing crawl gait controller");
    
    if (config != NULL) {
        s_config = *config;
    } else {
        crawl_gait_config_t default_config = CRAWL_GAIT_DEFAULT_CONFIG();
        s_config = default_config;
    }
    
    ESP_LOGI(TAG, "Config: FR=%.0f° FL=%.0f° BR=%.0f° BL=%.0f° amp=%.1f°",
             s_config.stance_angle_fr, s_config.stance_angle_fl,
             s_config.stance_angle_br, s_config.stance_angle_bl,
             s_config.swing_amplitude);
    
    // Check servos using dog_config
    bool all_ok = dog_check_servos();
    if (!all_ok) {
        ESP_LOGW(TAG, "Some servos not responding");
    }
    
    // Enable torque
    dog_set_torque(true);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    goto_stance_internal();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    s_direction = GAIT_DIRECTION_STOP;
    s_phase = 0;
    
    return all_ok;
}

void crawl_gait_set_config(const crawl_gait_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
    }
}

void crawl_gait_get_config(crawl_gait_config_t *config)
{
    if (config != NULL) {
        *config = s_config;
    }
}

bool crawl_gait_start(gait_direction_t direction)
{
    if (s_gait_task_handle != NULL) {
        s_direction = direction;
        return true;
    }
    
    s_direction = direction;
    s_running = true;
    s_phase = 0;
    
    BaseType_t ret = xTaskCreate(gait_task, "crawl_gait", 4096, NULL, 5, &s_gait_task_handle);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gait task");
        s_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Crawl gait started");
    return true;
}

void crawl_gait_stop(void)
{
    if (!s_running) return;
    
    ESP_LOGI(TAG, "Stopping crawl gait");
    s_running = false;
    
    while (s_gait_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void crawl_gait_set_direction(gait_direction_t direction)
{
    s_direction = direction;
}

gait_direction_t crawl_gait_get_direction(void)
{
    return s_direction;
}

bool crawl_gait_is_running(void)
{
    return s_running;
}

void crawl_gait_goto_stance(void)
{
    goto_stance_internal();
}

void crawl_gait_step(gait_direction_t direction)
{
    if (direction == GAIT_DIRECTION_STOP) {
        goto_stance_internal();
        return;
    }
    
    execute_phase(s_phase, direction);
    vTaskDelay(pdMS_TO_TICKS(s_config.step_duration_ms));
    s_phase = (s_phase + 1) % 4;
}
