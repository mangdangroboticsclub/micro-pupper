/**
 * @file walk_forward_reaction.c
 * @brief Walk forward animation implementation
 * 
 * 6-keyframe animation that plays in response to detecting
 * a forward push on the accelerometer OR pressure on front legs.
 * 
 * Logic:
 *   - Pressure only (FL or FR) → Walk 1 cycle
 *   - Pressure + IMU push detected → Walk 3 cycles
 */

#include "walk_forward_reaction.h"
#include "dog_config.h"
#include "reaction/reaction_config.h"
#include "servo/servo_pressure.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "WALK_FORWARD";

// ═══════════════════════════════════════════════════════
// FORWARD DETECTION THRESHOLDS
// ═══════════════════════════════════════════════════════

#define FORWARD_PITCH_THRESHOLD   5.0f    // Degrees of forward tilt to trigger
#define FORWARD_ACCEL_THRESHOLD   0.15f   // G-force threshold for forward push
#define CONTINUOUS_CHECK_INTERVAL 50      // ms between condition checks during walk
#define MAX_WALK_CYCLES           50      // Safety limit

// ═══════════════════════════════════════════════════════
// ANIMATION KEYFRAMES
// ═══════════════════════════════════════════════════════

typedef struct {
    float fr;        // Front-right angle
    float fl;        // Front-left angle
    float br;        // Back-right angle
    float bl;        // Back-left angle
    uint16_t speed;  // Movement speed
    uint16_t delay;  // Delay after move (ms)
} keyframe_t;

static const keyframe_t walk_forward_keyframes[] = {
    // Keyframe 1: Lift FR and BL
    { .fr = 55,  .fl = 110, .br = 290, .bl = 240, .speed = 1600, .delay = 150 },
    // Keyframe 2: Swing FR and BL forward
    { .fr = 95,  .fl = 80,  .br = 260, .bl = 285, .speed = 1050, .delay = 150 },
    // Keyframe 3: Neutral
    { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 1300, .delay = 75  },
    // Keyframe 4: Lift FL and BR
    { .fr = 110, .fl = 55,  .br = 240, .bl = 290, .speed = 1600, .delay = 150 },
    // Keyframe 5: Swing FL and BR forward
    { .fr = 80,  .fl = 95,  .br = 285, .bl = 260, .speed = 950,  .delay = 150 },
    // Keyframe 6: Neutral
    { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 700,  .delay = 75  },
};

#define KEYFRAME_COUNT (sizeof(walk_forward_keyframes) / sizeof(keyframe_t))

// ═══════════════════════════════════════════════════════
// INTERNAL HELPERS
// ═══════════════════════════════════════════════════════

/**
 * @brief Play a single walk cycle (all keyframes once)
 */
static void play_one_cycle(void)
{
    for (size_t i = 0; i < KEYFRAME_COUNT; i++) {
        const keyframe_t *kf = &walk_forward_keyframes[i];
        
        dog_servo_move_all(kf->fr, kf->fl, kf->br, kf->bl, kf->speed);
        
        uint32_t total_delay = kf->delay + REACTION_TIMING_OFFSET_MS;
        if (total_delay > 0) {
            vTaskDelay(pdMS_TO_TICKS(total_delay));
        }
    }
}

// ═══════════════════════════════════════════════════════
// FORWARD CONDITION DETECTION
// ═══════════════════════════════════════════════════════

bool walk_forward_check_conditions(float pitch_deg, float accel_y)
{
    bool front_pressure = servo_pressure_check_front();
    bool forward_tilt = (pitch_deg > FORWARD_PITCH_THRESHOLD);
    bool forward_push = (accel_y > FORWARD_ACCEL_THRESHOLD);
    
    bool should_walk = front_pressure && (forward_tilt || forward_push);
    
    if (should_walk) {
        ESP_LOGD(TAG, "Forward conditions met: pressure=%d, pitch=%.1f°, accel_y=%.2fg",
                 front_pressure, pitch_deg, accel_y);
    }
    
    return should_walk;
}

bool walk_forward_check_pressure_only(void)
{
    return servo_pressure_check_front();
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void walk_forward_play(uint8_t cycles)
{
    if (cycles == 0) {
        return;
    }
    
    ESP_LOGI(TAG, "🚶 Walking forward: %d cycle(s)", cycles);
    
    for (uint8_t cycle = 0; cycle < cycles; cycle++) {
        ESP_LOGD(TAG, "Cycle %d/%d", cycle + 1, cycles);
        play_one_cycle();
    }
    
    ESP_LOGI(TAG, "Walk complete, returning to stance");
    dog_goto_stance();
}

void walk_forward_play_while_pushed(void (*get_imu_data)(float *pitch, float *accel_y))
{
    ESP_LOGI(TAG, "Starting continuous walk (while pushed)");
    
    float pitch = 0.0f;
    float accel_y = 0.0f;
    uint8_t cycles_played = 0;
    
    // Update pressure state
    servo_pressure_update();
    
    // Get initial IMU data
    if (get_imu_data) {
        get_imu_data(&pitch, &accel_y);
    }
    
    // Check if we should start
    bool front_pressure = servo_pressure_check_front();
    if (!front_pressure) {
        ESP_LOGI(TAG, "No front pressure, not walking");
        return;
    }
    
    ESP_LOGI(TAG, "🏃 Front pressure detected! Walking...");
    
    while (cycles_played < MAX_WALK_CYCLES) {
        play_one_cycle();
        cycles_played++;
        
        ESP_LOGD(TAG, "Completed cycle %d", cycles_played);
        
        // Update sensors
        servo_pressure_update();
        if (get_imu_data) {
            get_imu_data(&pitch, &accel_y);
        }
        
        // Check if we should continue
        front_pressure = servo_pressure_check_front();
        bool forward_tilt = (pitch > FORWARD_PITCH_THRESHOLD);
        bool forward_push = (accel_y > FORWARD_ACCEL_THRESHOLD);
        bool should_continue = front_pressure || forward_tilt || forward_push;
        
        if (!should_continue) {
            ESP_LOGI(TAG, "Conditions no longer met after %d cycles", cycles_played);
            break;
        }
    }
    
    if (cycles_played >= MAX_WALK_CYCLES) {
        ESP_LOGW(TAG, "Safety limit reached (%d cycles)", MAX_WALK_CYCLES);
    }
    
    ESP_LOGI(TAG, "Returning to stance");
    dog_goto_stance();
    // Return to stance position after animation completes
    ESP_LOGI(TAG, "Animation complete, returning to stance");
    dog_goto_stance_smooth();
}

void walk_forward_play_while_front_pressure(void)
{
    ESP_LOGI(TAG, "Starting walk (while front pressure)");
    
    uint8_t cycles_played = 0;
    
    // Update and check initial pressure
    servo_pressure_update();
    
    if (!servo_pressure_check_front()) {
        ESP_LOGI(TAG, "No front pressure, not starting");
        return;
    }
    
    ESP_LOGI(TAG, "🏃 Front pressure detected! Walking...");
    
    while (cycles_played < MAX_WALK_CYCLES) {
        play_one_cycle();
        cycles_played++;
        
        // Update and check pressure
        servo_pressure_update();
        
        if (!servo_pressure_check_front()) {
            ESP_LOGI(TAG, "Pressure released after %d cycles", cycles_played);
            break;
        }
    }
    
    if (cycles_played >= MAX_WALK_CYCLES) {
        ESP_LOGW(TAG, "Safety limit reached");
    }
    
    ESP_LOGI(TAG, "Returning to stance");
    dog_goto_stance();
}