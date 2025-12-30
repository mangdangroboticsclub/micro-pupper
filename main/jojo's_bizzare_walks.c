/**
 * @file crawl_movements.c
 * @brief Exported Crawl Gait Movement Functions
 * 
 * Stance angles: FR=90 (reversed), FL=90, BR=270 (reversed), BL=270
 * Swing amplitude: 25 degrees
 * 
 * Angle calculations (after right-side reversal):
 *   FR: forward=115, stance=90,  back=65    (reversed right side)
 *   FL: forward=115, stance=90,  back=65
 *   BR: forward=295, stance=270, back=245   (reversed right side)
 *   BL: forward=295, stance=270, back=245
 * 
 * Servo order in arrays: FL, FR, BL, BR
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MOVEMENT";

// Stance (neutral) angles
#define NEUTRAL_FL  90
#define NEUTRAL_FR  90    // 360 - 270 (reversed right side)
#define NEUTRAL_BL  270
#define NEUTRAL_BR  270   // 360 - 90 (reversed right side)

// Forward swing angles (leg moves forward)
#define SWING_FL    115
#define SWING_FR    115   // 360 - 245 (reversed right side)
#define SWING_BL    295
#define SWING_BR    295   // 360 - 65 (reversed right side)

// Push back angles (leg pushes backward)
#define PUSH_FL     65
#define PUSH_FR     65    // 360 - 295 (reversed right side)
#define PUSH_BL     245
#define PUSH_BR     245   // 360 - 115 (reversed right side)

/**
 * @brief Move forward using crawl gait
 * Sequence: BL -> FR -> BR -> FL (alternating sides)
 * @param duration_seconds How long to walk forward
 */
static void MovementGoForward(int duration_seconds) {
    ESP_LOGI(TAG, ">>> GO FORWARD <<<");
    
    // One complete crawl cycle (4 phases)
    // Phase order: BL swings, FR swings, BR swings, FL swings
    int FL_angles[] = {PUSH_FL,  PUSH_FL,  PUSH_FL,  SWING_FL};
    int FR_angles[] = {PUSH_FR,  SWING_FR, PUSH_FR,  PUSH_FR};
    int BL_angles[] = {SWING_BL, PUSH_BL,  PUSH_BL,  PUSH_BL};
    int BR_angles[] = {PUSH_BR,  PUSH_BR,  SWING_BR, PUSH_BR};
    uint16_t delays_ms[] = {250, 250, 250, 250};
    
    int cycles = (duration_seconds * 1000) / 1000;  // 1 cycle = 1 second
    
    for (int c = 0; c < cycles; c++) {
        for (int i = 0; i < 4; i++) {
            servo_write_all(
                FL_angles[i] + osang_[0],
                FR_angles[i] + osang_[1],
                BL_angles[i] + osang_[2],
                BR_angles[i] + osang_[3],
                (uint16_t)SPEED_VERY_FAST
            );
            vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
        }
    }
    
    // Return to stance
    servo_write_all(
        NEUTRAL_FL + osang_[0],
        NEUTRAL_FR + osang_[1],
        NEUTRAL_BL + osang_[2],
        NEUTRAL_BR + osang_[3],
        (uint16_t)SPEED_VERY_FAST
    );
}

/**
 * @brief Turn right using crawl gait
 * Sequence: BL -> BR -> FL -> FR (same-side consecutive)
 * @param duration_seconds How long to turn
 */
static void MovementTurnRight(int duration_seconds) {
    ESP_LOGI(TAG, ">>> TURN RIGHT <<<");
    
    // One complete turn cycle (4 phases)
    // Phase order: BL swings, BR swings, FL swings, FR swings
    int FL_angles[] = {PUSH_FL,  PUSH_FL,  SWING_FL, PUSH_FL};
    int FR_angles[] = {PUSH_FR,  PUSH_FR,  PUSH_FR,  SWING_FR};
    int BL_angles[] = {SWING_BL, PUSH_BL,  PUSH_BL,  PUSH_BL};
    int BR_angles[] = {PUSH_BR,  SWING_BR, PUSH_BR,  PUSH_BR};
    uint16_t delays_ms[] = {250, 250, 250, 250};
    
    int cycles = (duration_seconds * 1000) / 1000;  // 1 cycle = 1 second
    
    for (int c = 0; c < cycles; c++) {
        for (int i = 0; i < 4; i++) {
            servo_write_all(
                FL_angles[i] + osang_[0],
                FR_angles[i] + osang_[1],
                BL_angles[i] + osang_[2],
                BR_angles[i] + osang_[3],
                (uint16_t)SPEED_MEDIUM
            );
            vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
        }
    }
    
    // Return to stance
    servo_write_all(
        NEUTRAL_FL + osang_[0],
        NEUTRAL_FR + osang_[1],
        NEUTRAL_BL + osang_[2],
        NEUTRAL_BR + osang_[3],
        (uint16_t)SPEED_MEDIUM
    );
}

/**
 * @brief Turn left using crawl gait
 * Sequence: BR -> BL -> FR -> FL (mirror of turn right)
 * @param duration_seconds How long to turn
 */
static void MovementTurnLeft(int duration_seconds) {
    ESP_LOGI(TAG, ">>> TURN LEFT <<<");
    
    // One complete turn cycle (4 phases)
    // Phase order: BR swings, BL swings, FR swings, FL swings
    int FL_angles[] = {PUSH_FL,  PUSH_FL,  PUSH_FL,  SWING_FL};
    int FR_angles[] = {PUSH_FR,  PUSH_FR,  SWING_FR, PUSH_FR};
    int BL_angles[] = {PUSH_BL,  SWING_BL, PUSH_BL,  PUSH_BL};
    int BR_angles[] = {SWING_BR, PUSH_BR,  PUSH_BR,  PUSH_BR};
    uint16_t delays_ms[] = {250, 250, 250, 250};
    
    int cycles = (duration_seconds * 1000) / 1000;  // 1 cycle = 1 second
    
    for (int c = 0; c < cycles; c++) {
        for (int i = 0; i < 4; i++) {
            servo_write_all(
                FL_angles[i] + osang_[0],
                FR_angles[i] + osang_[1],
                BL_angles[i] + osang_[2],
                BR_angles[i] + osang_[3],
                (uint16_t)SPEED_MEDIUM
            );
            vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
        }
    }
    
    // Return to stance
    servo_write_all(
        NEUTRAL_FL + osang_[0],
        NEUTRAL_FR + osang_[1],
        NEUTRAL_BL + osang_[2],
        NEUTRAL_BR + osang_[3],
        (uint16_t)SPEED_MEDIUM
    );
}