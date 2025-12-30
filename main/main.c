/**
 * @file main.c
 * @brief Robot Control with Minimal BLE Servo Protocol
 * 
 * Supports two modes:
 *   1. BLE Control Mode - Control servos via Web Bluetooth for gait development
 *   2. Demo Mode - Run automated gait demonstrations
 * 
 * BLE Commands (JSON format):
 *   - Single move:   {"s":[fr,fl,br,bl,speed,delay_ms]}
 *   - Multi move:    {"m":[[fr,fl,br,bl,speed,delay],[...]]}
 *   - Per-leg move:  {"l":[[fr,spd,dly],[fl,spd,dly],[br,spd,dly],[bl,spd,dly]]}
 *   - Per-leg multi: {"L":[...per-leg moves...]}
 *   - Stance:        {"c":"stance"}
 *   - Ping:          {"c":"ping"}
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Dog configuration (pins, servo IDs, angle reversal)
#include "dog_config.h"

// Minimal BLE servo control
#include "ble/ble_servo.h"

// Crawl gait algorithm
#include "gait_common.h"
#include "crawl_gait.h"

static const char *TAG = "ROBOT_MAIN";

// ═══════════════════════════════════════════════════════
// BLE SERVO CALLBACKS
// ═══════════════════════════════════════════════════════

/**
 * @brief Called when BLE receives a servo move command
 * 
 * If any angle is -1, that leg is skipped (used for offset gait single-leg moves).
 * If all angles are valid (>= 0), all legs move together.
 */
static void on_servo_move(float fr, float fl, float br, float bl,
                          uint16_t speed, uint16_t delay_ms) {
    ESP_LOGI(TAG, "Move: FR=%.0f FL=%.0f BR=%.0f BL=%.0f spd=%u dly=%u",
             fr, fl, br, bl, speed, delay_ms);
    
    // Check if this is a selective move (some legs have -1)
    bool fr_move = (fr >= 0);
    bool fl_move = (fl >= 0);
    bool br_move = (br >= 0);
    bool bl_move = (bl >= 0);
    
    if (fr_move && fl_move && br_move && bl_move) {
        // All legs moving - use batch move
        dog_servo_move_all(fr, fl, br, bl, speed);
    } else {
        // Selective leg movement - move only specified legs
        if (fr_move) dog_servo_move(DOG_SERVO_FR, fr, speed);
        if (fl_move) dog_servo_move(DOG_SERVO_FL, fl, speed);
        if (br_move) dog_servo_move(DOG_SERVO_BR, br, speed);
        if (bl_move) dog_servo_move(DOG_SERVO_BL, bl, speed);
    }
    
    // Apply delay if specified
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief Called when BLE receives a per-leg move command
 * 
 * Each leg has its own speed and delay, allowing different timing between legs
 * for more natural gaits. Legs with angle == -1 are skipped (not moved).
 * This supports offset gait mode where only one leg moves at a time.
 */
static void on_leg_move(ble_leg_move_t fr, ble_leg_move_t fl,
                        ble_leg_move_t br, ble_leg_move_t bl) {
    ESP_LOGI(TAG, "Per-leg move: FR=%.0f/%u/%u FL=%.0f/%u/%u BR=%.0f/%u/%u BL=%.0f/%u/%u",
             fr.angle, fr.speed, fr.delay_ms,
             fl.angle, fl.speed, fl.delay_ms,
             br.angle, br.speed, br.delay_ms,
             bl.angle, bl.speed, bl.delay_ms);
    
    // Move each leg with its individual speed
    // Skip legs with angle == -1 (not part of this move in offset gait mode)
    
    if (fr.angle >= 0 && fr.speed > 0) {
        dog_servo_move(DOG_SERVO_FR, fr.angle, fr.speed);
        if (fr.delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(fr.delay_ms));
        }
    }
    
    if (fl.angle >= 0 && fl.speed > 0) {
        dog_servo_move(DOG_SERVO_FL, fl.angle, fl.speed);
        if (fl.delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(fl.delay_ms));
        }
    }
    
    if (br.angle >= 0 && br.speed > 0) {
        dog_servo_move(DOG_SERVO_BR, br.angle, br.speed);
        if (br.delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(br.delay_ms));
        }
    }
    
    if (bl.angle >= 0 && bl.speed > 0) {
        dog_servo_move(DOG_SERVO_BL, bl.angle, bl.speed);
        if (bl.delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(bl.delay_ms));
        }
    }
}

/**
 * @brief Called when BLE receives a stance command
 */
static void on_stance(void) {
    ESP_LOGI(TAG, "Stance command received");
    // Stop any running gait
    if (crawl_gait_is_running()) {
        crawl_gait_stop();
    }
    dog_goto_stance();
}

/**
 * @brief Called on BLE connection state change
 */
static void on_connect(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "=== BLE Client Connected ===");
        // Stop gait and go to stance on new connection
        if (crawl_gait_is_running()) {
            crawl_gait_stop();
        }
        dog_goto_stance();
    } else {
        ESP_LOGI(TAG, "=== BLE Client Disconnected ===");
    }
}

// ═══════════════════════════════════════════════════════
// DEMO MODE (Optional)
// ═══════════════════════════════════════════════════════

#define RUN_DEMO_MODE 0  // Set to 1 to run demo instead of BLE mode

static void run_demo_mode(void) {
    ESP_LOGI(TAG, "Running Demo Mode");
    
    // Initialize crawl gait
    crawl_gait_config_t crawl_config = {
        .stance_angle_fr = DOG_STANCE_FRONT,
        .stance_angle_fl = DOG_STANCE_FRONT,
        .stance_angle_br = DOG_STANCE_BACK,
        .stance_angle_bl = DOG_STANCE_BACK,
        .swing_amplitude = DOG_SWING_AMPLITUDE,
        .step_duration_ms = 250,
        .servo_speed = DOG_SPEED_VERY_FAST,
    };
    crawl_gait_init(&crawl_config);
    
    // Demo: Forward 6s -> Right 6s -> Left 6s -> Forward 6s -> Stop
    ESP_LOGI(TAG, ">>> FORWARD");
    crawl_gait_start(GAIT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    ESP_LOGI(TAG, ">>> TURN RIGHT");
    crawl_gait_set_direction(GAIT_DIRECTION_TURN_RIGHT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    ESP_LOGI(TAG, ">>> TURN LEFT");
    crawl_gait_set_direction(GAIT_DIRECTION_TURN_LEFT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    ESP_LOGI(TAG, ">>> FORWARD");
    crawl_gait_set_direction(GAIT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    crawl_gait_stop();
    ESP_LOGI(TAG, "Demo complete!");
}

// ═══════════════════════════════════════════════════════
// MAIN APPLICATION
// ═══════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      MicroPupper Robot Control        ║");
    ESP_LOGI(TAG, "║      BLE + Web Bluetooth Ready        ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    // ───────────────────────────────────────────────────────
    // STEP 1: Initialize NVS (required for BLE)
    // ───────────────────────────────────────────────────────
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    // ───────────────────────────────────────────────────────
    // STEP 2: Initialize dog hardware
    // ───────────────────────────────────────────────────────
    
    if (!dog_init(NULL)) {
        ESP_LOGW(TAG, "Some servos not responding, but continuing...");
    }
    ESP_LOGI(TAG, "Dog hardware initialized");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Initialize IMU with smart logging
    if (dog_imu_init()) {
        dog_imu_task_start();
        ESP_LOGI(TAG, "IMU initialized with smart logging");
    } else {
        ESP_LOGW(TAG, "IMU initialization failed, continuing without IMU");
    }
    
    // ───────────────────────────────────────────────────────
    // STEP 3: Initialize crawl gait (for text commands)
    // ───────────────────────────────────────────────────────
    
    crawl_gait_config_t crawl_config = {
        .stance_angle_fr = DOG_STANCE_FRONT,
        .stance_angle_fl = DOG_STANCE_FRONT,
        .stance_angle_br = DOG_STANCE_BACK,
        .stance_angle_bl = DOG_STANCE_BACK,
        .swing_amplitude = DOG_SWING_AMPLITUDE,
        .step_duration_ms = 250,
        .servo_speed = DOG_SPEED_VERY_FAST,
    };
    crawl_gait_init(&crawl_config);
    ESP_LOGI(TAG, "Crawl gait initialized");
    
#if RUN_DEMO_MODE
    // ───────────────────────────────────────────────────────
    // Demo Mode
    // ───────────────────────────────────────────────────────
    run_demo_mode();
#else
    // ───────────────────────────────────────────────────────
    // STEP 4: Initialize BLE Servo Control
    // ───────────────────────────────────────────────────────
    
    // Initialize BLE with callbacks
    if (!ble_servo_init(on_servo_move, on_leg_move, on_stance, on_connect)) {
        ESP_LOGE(TAG, "Failed to initialize BLE!");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BLE Ready - Connect via Web Bluetooth ║");
    ESP_LOGI(TAG, "║  Device: MicroPupper                   ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Commands:");
    ESP_LOGI(TAG, "  Move:     {\"s\":[fr,fl,br,bl,speed,delay]}");
    ESP_LOGI(TAG, "  Multi:    {\"m\":[[fr,fl,br,bl,spd,dly],[...]]}");
    ESP_LOGI(TAG, "  Per-leg:  {\"l\":[[fr,spd,dly],[fl,...],[br,...],[bl,...]]}");
    ESP_LOGI(TAG, "  Per-leg+: {\"L\":[per-leg moves...]}");
    ESP_LOGI(TAG, "  Stance:   {\"c\":\"stance\"}");
    ESP_LOGI(TAG, "  Ping:     {\"c\":\"ping\"}");
    ESP_LOGI(TAG, "");
#endif
    
    // ───────────────────────────────────────────────────────
    // Main loop
    // ───────────────────────────────────────────────────────
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (ble_servo_is_connected()) {
            ESP_LOGD(TAG, "BLE connected, waiting for commands...");
        } else {
            ESP_LOGD(TAG, "Waiting for BLE connection...");
        }
    }
}
