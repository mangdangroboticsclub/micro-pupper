/**
 * @file sts3032_config.c
 * @brief STS3032 Configuration Utilities Implementation
 */

#include "sts3032_config.h"
#include "sts3032_servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STS_CONFIG";

void sts_config_wizard(uint8_t new_id) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  SERVO ID CONFIGURATION WIZARD        ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "This wizard will assign ID %d to the servo", new_id);
    ESP_LOGI(TAG, "");
    ESP_LOGW(TAG, "⚠️  IMPORTANT STEPS:");
    ESP_LOGW(TAG, "   1. Disconnect ALL servos from bus");
    ESP_LOGW(TAG, "   2. Connect ONLY the servo you want to configure");
    ESP_LOGW(TAG, "   3. Make sure servo is powered");
    ESP_LOGI(TAG, "");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Check if there's a servo at ID 1 (default factory ID)
    ESP_LOGI(TAG, "Checking for servo at ID 1 (factory default)...");
    
    if (sts_servo_ping(1)) {
        ESP_LOGI(TAG, "✓ Found servo at ID 1");
        ESP_LOGI(TAG, "");
        
        if (new_id == 1) {
            ESP_LOGI(TAG, "Servo is already ID 1. No change needed.");
            ESP_LOGI(TAG, "If this servo was previously configured, it's ready to use.");
        } else {
            ESP_LOGI(TAG, "Changing servo from ID 1 to ID %d...", new_id);
            
            if (sts_servo_change_id(1, new_id)) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "✓ SUCCESS! Servo is now ID %d", new_id);
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Next steps:");
                ESP_LOGI(TAG, "  1. Disconnect this servo");
                ESP_LOGI(TAG, "  2. Connect the next servo (if any)");
                ESP_LOGI(TAG, "  3. Power cycle ESP32");
                ESP_LOGI(TAG, "  4. Run wizard again with next ID");
            } else {
                ESP_LOGE(TAG, "❌ Failed to change ID");
            }
        }
    } else {
        // Check if servo is already at the target ID
        ESP_LOGW(TAG, "No servo found at ID 1");
        ESP_LOGI(TAG, "Checking if servo is already at ID %d...", new_id);
        
        if (sts_servo_ping(new_id)) {
            ESP_LOGI(TAG, "✓ Servo is already at ID %d!", new_id);
            ESP_LOGI(TAG, "No configuration needed.");
        } else {
            ESP_LOGW(TAG, "No servo found at ID %d either", new_id);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Troubleshooting:");
            ESP_LOGI(TAG, "  - Make sure servo is connected");
            ESP_LOGI(TAG, "  - Make sure servo is powered");
            ESP_LOGI(TAG, "  - Only ONE servo should be connected");
            ESP_LOGI(TAG, "  - Servo might have a different ID already");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Scanning bus to find servos...");
            sts_servo_scan_bus(1, 10);
        }
    }
    
    ESP_LOGI(TAG, "");
}

void sts_config_auto_detect(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  SERVO AUTO-DETECTION                 ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    int count = sts_servo_scan_bus(1, 20);
    
    if (count == 0) {
        ESP_LOGW(TAG, "No servos detected!");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Troubleshooting:");
        ESP_LOGI(TAG, "  - Check servo power connection");
        ESP_LOGI(TAG, "  - Check UART wiring (TX, RX, TXEN)");
        ESP_LOGI(TAG, "  - Verify baud rate (should be 1000000)");
        ESP_LOGI(TAG, "  - Try scanning a wider range");
    } else if (count == 1) {
        ESP_LOGI(TAG, "Configuration looks good for single servo setup.");
    } else {
        ESP_LOGI(TAG, "Multiple servos detected.");
        ESP_LOGI(TAG, "Make sure each servo has a unique ID.");
    }
    
    ESP_LOGI(TAG, "");
}
