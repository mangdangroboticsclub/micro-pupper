/**
 * @file walk_forward_reaction.c
 * @brief Walk forward animation implementation
 * 
 * 6-keyframe animation that plays in response to detecting
 * a forward push on the accelerometer.
 */

#include "walk_forward_reaction.h"
#include "dog_config.h"
#include "reaction/reaction_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WALK_FORWARD";

// ═══════════════════════════════════════════════════════
// ANIMATION KEYFRAMES
// ═══════════════════════════════════════════════════════

typedef struct {
    float fr;           // Front-right angle
    float fl;           // Front-left angle
    float br;           // Back-right angle
    float bl;           // Back-left angle
    uint16_t speed;     // Movement speed
    uint16_t delay;     // Delay after move (ms)
} keyframe_t;

static const keyframe_t walk_forward_keyframes[] = {
    // Keyframe 1
    { .fr = 55,  .fl = 110, .br = 290, .bl = 240, .speed = 1600, .delay = 150 },
    // Keyframe 2
    { .fr = 95,  .fl = 80,  .br = 260, .bl = 285, .speed = 1050, .delay = 150 },
    // Keyframe 3
    { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 1300, .delay = 75  },
    // Keyframe 4
    { .fr = 110, .fl = 55,  .br = 240, .bl = 290, .speed = 1600, .delay = 150 },
    // Keyframe 5
    { .fr = 80,  .fl = 95,  .br = 285, .bl = 260, .speed = 950,  .delay = 150 },
    // Keyframe 6
    { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 700,  .delay = 75  },
};

#define KEYFRAME_COUNT (sizeof(walk_forward_keyframes) / sizeof(keyframe_t))

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void walk_forward_play(uint8_t cycles)
{
    ESP_LOGI(TAG, "Playing walk forward animation (%d cycles, %d keyframes each)",
             cycles, KEYFRAME_COUNT);
    
    for (uint8_t cycle = 0; cycle < cycles; cycle++) {
        ESP_LOGI(TAG, "Cycle %d/%d", cycle + 1, cycles);
        
        for (size_t i = 0; i < KEYFRAME_COUNT; i++) {
            const keyframe_t *kf = &walk_forward_keyframes[i];
            
            // Move all servos to keyframe position
            dog_servo_move_all(kf->fr, kf->fl, kf->br, kf->bl, kf->speed);
            
            // Apply keyframe delay + timing offset
            uint32_t total_delay = kf->delay + REACTION_TIMING_OFFSET_MS;
            if (total_delay > 0) {
                vTaskDelay(pdMS_TO_TICKS(total_delay));
            }
        }
    }
    
    // Return to stance position after animation completes
    ESP_LOGI(TAG, "Animation complete, returning to stance");
    dog_goto_stance();
}
