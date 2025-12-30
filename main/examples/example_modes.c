/**
 * @file example_modes.c
 * @brief STS3032 Servo Driver - Additional Examples
 * 
 * This file contains optional examples showing different use cases
 * of the STS3032 servo driver.
 * 
 * To use these examples in your main.c:
 * 1. Include "examples/example_modes.h"
 * 2. Call the example function you want to run
 */

#include "example_modes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "../sts3032_protocol.h"
#include "../sts3032_servo.h"
#include "../sts3032_config.h"

static const char *TAG = "EXAMPLE";

// ═══════════════════════════════════════════════════════
// EXAMPLE 1: Servo Configuration
// ═══════════════════════════════════════════════════════
// Use this when setting up new servos or changing IDs

void example_servo_configuration(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EXAMPLE: Servo Configuration");
    ESP_LOGI(TAG, "============================");
    ESP_LOGI(TAG, "");
    
    // Option A: Use the wizard to assign a specific ID
    // Uncomment and change the number for each servo you want to configure
    // sts_config_wizard(2);  // Assign ID 2 to the connected servo
    
    // Option B: Auto-detect all servos on the bus
    // This will scan and display all connected servos
    ESP_LOGI(TAG, "Scanning for all servos on the bus...");
    sts_config_auto_detect();
    
    // Option C: Manually change a servo's ID
    // Uncomment to change a specific servo's ID
    // if (sts_servo_ping(1)) {
    //     ESP_LOGI(TAG, "Changing servo ID from 1 to 3");
    //     sts_servo_change_id(1, 3);
    // }
    
    ESP_LOGI(TAG, "Configuration example complete");
}

// ═══════════════════════════════════════════════════════
// EXAMPLE 2: Single Servo Test
// ═══════════════════════════════════════════════════════
// Simple test of a single servo

void example_single_servo(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EXAMPLE: Single Servo Test");
    ESP_LOGI(TAG, "==========================");
    ESP_LOGI(TAG, "");
    
    // Scan for servos
    ESP_LOGI(TAG, "Scanning for servos (ID 1-10)...");
    int servo_count = sts_servo_scan_bus(1, 10);
    
    if (servo_count == 0) {
        ESP_LOGE(TAG, "No servos found! Check connections.");
        return;
    }
    
    // Enable torque on found servos
    for (uint8_t id = 1; id <= 3; id++) {
        if (sts_servo_ping(id)) {
            sts_servo_enable_torque(id, true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    // Simple movement test
    ESP_LOGI(TAG, "Starting movement test...");
    
    while (1) {
        // Move to 90 degrees
        ESP_LOGI(TAG, "Moving to 90°");
        sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move to 45 degrees
        ESP_LOGI(TAG, "Moving to 45°");
        sts_servo_set_angle(1, 45.0, SPEED_MEDIUM);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move to 135 degrees
        ESP_LOGI(TAG, "Moving to 135°");
        sts_servo_set_angle(1, 135.0, SPEED_MEDIUM);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Read current position
        float angle;
        if (sts_servo_get_angle(1, &angle)) {
            ESP_LOGI(TAG, "Current angle: %.1f°", angle);
        }
    }
}

// ═══════════════════════════════════════════════════════
// EXAMPLE 3: Multi-Servo Control
// ═══════════════════════════════════════════════════════
// Demonstrates controlling multiple servos simultaneously

void example_multi_servo(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EXAMPLE: Multi-Servo Control");
    ESP_LOGI(TAG, "============================");
    ESP_LOGI(TAG, "");
    
    // Define your servo IDs
    uint8_t servo_ids[] = {1, 2, 3};
    int num_servos = sizeof(servo_ids) / sizeof(servo_ids[0]);
    
    // Enable torque on all servos
    ESP_LOGI(TAG, "Connecting to servos...");
    for (int i = 0; i < num_servos; i++) {
        if (sts_servo_ping(servo_ids[i])) {
            ESP_LOGI(TAG, "  Servo ID %d: Online", servo_ids[i]);
            sts_servo_enable_torque(servo_ids[i], true);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ESP_LOGW(TAG, "  Servo ID %d: Not found", servo_ids[i]);
        }
    }
    
    // Coordinated movement demo
    ESP_LOGI(TAG, "Starting coordinated movement patterns...");
    
    while (1) {
        // Pattern 1: All to 90°
        ESP_LOGI(TAG, "Pattern 1: All servos to center (90°)");
        for (int i = 0; i < num_servos; i++) {
            sts_servo_set_angle(servo_ids[i], 90.0, SPEED_MEDIUM);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Pattern 2: Sequential positions
        ESP_LOGI(TAG, "Pattern 2: Sequential positions");
        sts_servo_set_angle(servo_ids[0], 45.0, SPEED_MEDIUM);
        sts_servo_set_angle(servo_ids[1], 90.0, SPEED_MEDIUM);
        sts_servo_set_angle(servo_ids[2], 135.0, SPEED_MEDIUM);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Pattern 3: Wave motion
        ESP_LOGI(TAG, "Pattern 3: Wave motion (up)");
        for (int i = 0; i < num_servos; i++) {
            sts_servo_set_angle(servo_ids[i], 135.0, SPEED_FAST);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ESP_LOGI(TAG, "Pattern 3: Wave motion (down)");
        for (int i = 0; i < num_servos; i++) {
            sts_servo_set_angle(servo_ids[i], 45.0, SPEED_FAST);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
