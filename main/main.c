#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Dog configuration (pins, servo IDs, angle reversal)
#include "dog_config.h"

// Reaction system (pressure + IMU)
#include "reaction/reaction_config.h"

// Minimal BLE servo control
#include "ble/ble_servo.h"

// Crawl gait algorithm
#include "gait_common.h"
#include "crawl_gait.h"

static const char *TAG = "ROBOT_MAIN";

// ═══════════════════════════════════════════════════════
// MODE SELECTION
// ═══════════════════════════════════════════════════════

#define MODE_BLE        0   // BLE control mode
#define MODE_DEMO       1   // Crawl gait demo
#define MODE_REACTION   2   // Pressure + IMU reaction mode

#define RUN_MODE  MODE_REACTION   // <-- Change this to select mode

// ═══════════════════════════════════════════════════════
// BLE SERVO CALLBACKS
// ═══════════════════════════════════════════════════════

static void on_servo_move(float fr, float fl, float br, float bl,
                          uint16_t speed, uint16_t delay_ms) {
    ESP_LOGI(TAG, "Move: FR=%.0f FL=%.0f BR=%.0f BL=%.0f spd=%u dly=%u",
             fr, fl, br, bl, speed, delay_ms);
    
    bool fr_move = (fr >= 0);
    bool fl_move = (fl >= 0);
    bool br_move = (br >= 0);
    bool bl_move = (bl >= 0);
    
    if (fr_move && fl_move && br_move && bl_move) {
        dog_servo_move_all(fr, fl, br, bl, speed);
    } else {
        if (fr_move) dog_servo_move(DOG_SERVO_FR, fr, speed);
        if (fl_move) dog_servo_move(DOG_SERVO_FL, fl, speed);
        if (br_move) dog_servo_move(DOG_SERVO_BR, br, speed);
        if (bl_move) dog_servo_move(DOG_SERVO_BL, bl, speed);
    }
    
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void on_leg_move(ble_leg_move_t fr, ble_leg_move_t fl,
                        ble_leg_move_t br, ble_leg_move_t bl) {
    ESP_LOGI(TAG, "Per-leg move: FR=%.0f/%u/%u FL=%.0f/%u/%u BR=%.0f/%u/%u BL=%.0f/%u/%u",
             fr.angle, fr.speed, fr.delay_ms,
             fl.angle, fl.speed, fl.delay_ms,
             br.angle, br.speed, br.delay_ms,
             bl.angle, bl.speed, bl.delay_ms);
    
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

static void on_stance(void) {
    ESP_LOGI(TAG, "Stance command received");
    if (crawl_gait_is_running()) {
        crawl_gait_stop();
    }
    dog_goto_stance_smooth();
}

static void on_connect(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "=== BLE Client Connected ===");
        if (crawl_gait_is_running()) {
            crawl_gait_stop();
        }
        dog_goto_stance_smooth();
    } else {
        ESP_LOGI(TAG, "=== BLE Client Disconnected ===");
    }
}

// ═══════════════════════════════════════════════════════
// DEMO MODE
// ═══════════════════════════════════════════════════════

static void run_demo_mode(void) {
    ESP_LOGI(TAG, "Running Demo Mode");
    
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
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    // Initialize dog hardware
    if (!dog_init(NULL)) {
        ESP_LOGW(TAG, "Some servos not responding, but continuing...");
    }
    ESP_LOGI(TAG, "Dog hardware initialized");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Move to stance position
    dog_goto_stance();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize IMU
    bool imu_ok = dog_imu_init();
    if (imu_ok) {
        dog_imu_task_start();
        ESP_LOGI(TAG, "IMU initialized");
    } else {
        ESP_LOGW(TAG, "IMU initialization failed");
    }
    
    // ═══════════════════════════════════════════════════════
    // MODE SELECTION
    // ═══════════════════════════════════════════════════════
    
#if RUN_MODE == MODE_REACTION
    // ───────────────────────────────────────────────────────
    // Reaction Mode: Pressure + IMU triggers walking
    // ───────────────────────────────────────────────────────
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Pressure + IMU Reaction Mode        ║");
    ESP_LOGI(TAG, "║                                       ║");
    ESP_LOGI(TAG, "║   Light push (pressure) → 1 cycle     ║");
    ESP_LOGI(TAG, "║   Hard push (pressure+IMU) → 3 cycles ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Initialize reaction system
    reaction_init();
    
    // Create pressure detection task (100Hz polling)
    xTaskCreate(reaction_pressure_task, "pressure", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Pressure task created (100Hz)");
    
    // The IMU reaction runs via the IMU task callback
    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
#elif RUN_MODE == MODE_DEMO
    // ───────────────────────────────────────────────────────
    // Demo Mode
    // ───────────────────────────────────────────────────────
    run_demo_mode();
    
#else
    // ───────────────────────────────────────────────────────
    // BLE Mode
    // ───────────────────────────────────────────────────────
    
    // Initialize crawl gait (for BLE commands)
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
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (ble_servo_is_connected()) {
            ESP_LOGD(TAG, "BLE connected, waiting for commands...");
        } else {
            ESP_LOGD(TAG, "Waiting for BLE connection...");
        }
    }
#endif
}