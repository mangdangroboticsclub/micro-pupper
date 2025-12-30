/**
 * @file dog_imu.c
 * @brief Dog-specific IMU implementation with smart logging
 * 
 * Uses the qmi8658a driver with project-specific configuration
 * and implements smart logging that only logs when values change.
 */

#include "dog_config.h"
#include "reaction/reaction_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "DOG_IMU";

// ═══════════════════════════════════════════════════════
// SMART LOGGING STATE
// ═══════════════════════════════════════════════════════

static qmi8658a_data_t g_last_logged_data = {0};
static bool g_first_log = true;

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * @brief Check if data has changed significantly since last log
 */
static bool has_significant_change(const qmi8658a_data_t *current, const qmi8658a_data_t *previous)
{
    float accel_threshold = DOG_IMU_ACCEL_CHANGE_THRESHOLD;
    float gyro_threshold = DOG_IMU_GYRO_CHANGE_THRESHOLD;
    
    // Check accelerometer changes
    if (fabsf(current->accel_x - previous->accel_x) > accel_threshold) return true;
    if (fabsf(current->accel_y - previous->accel_y) > accel_threshold) return true;
    if (fabsf(current->accel_z - previous->accel_z) > accel_threshold) return true;
    
    // Check gyroscope changes
    if (fabsf(current->gyro_x - previous->gyro_x) > gyro_threshold) return true;
    if (fabsf(current->gyro_y - previous->gyro_y) > gyro_threshold) return true;
    if (fabsf(current->gyro_z - previous->gyro_z) > gyro_threshold) return true;
    
    return false;
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool dog_imu_init(void)
{
    qmi8658a_config_t config = DOG_IMU_DEFAULT_CONFIG();
    
    ESP_LOGI(TAG, "Initializing IMU...");
    
    if (!qmi8658a_init(&config)) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        return false;
    }
    
    // Run diagnostics
    qmi8658a_debug_status();
    
    // Initialize reaction system
    reaction_init();
    
    ESP_LOGI(TAG, "IMU initialized successfully");
    return true;
}

bool dog_imu_read(qmi8658a_data_t *out_data)
{
    return qmi8658a_read(out_data);
}

// ═══════════════════════════════════════════════════════
// IMU MONITORING TASK
// ═══════════════════════════════════════════════════════

static void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started (smart logging enabled)");
    ESP_LOGI(TAG, "Log thresholds: Accel=%.2f m/s², Gyro=%.1f dps",
             DOG_IMU_ACCEL_CHANGE_THRESHOLD, DOG_IMU_GYRO_CHANGE_THRESHOLD);
    
    qmi8658a_data_t data;
    
    while (1) {
        if (dog_imu_read(&data)) {
            // Check for user interaction reactions first
            reaction_process_imu(&data);
            
            // Log if first reading or significant change
            if (g_first_log || has_significant_change(&data, &g_last_logged_data)) {
                ESP_LOGI(TAG, "ACCEL: X=%+7.2f Y=%+7.2f Z=%+7.2f m/s² | GYRO: X=%+7.1f Y=%+7.1f Z=%+7.1f dps",
                         data.accel_x, data.accel_y, data.accel_z,
                         data.gyro_x, data.gyro_y, data.gyro_z);
                
                g_last_logged_data = data;
                g_first_log = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Read at 20Hz
    }
}

void dog_imu_task_start(void)
{
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
