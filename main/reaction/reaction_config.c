/**
 * @file reaction_config.c
 * @brief Reaction system implementation
 * 
 * Simple logic:
 *   - IMU push detected → 3 cycles
 *   - Pressure only → 1 cycle
 */

#include "reaction_config.h"
#include "gyro_balance.h"
#include "walk_forward_reaction.h"
#include "servo/servo_pressure.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "REACTION";

// ═══════════════════════════════════════════════════════
// STATE TRACKING
// ═══════════════════════════════════════════════════════

static TickType_t last_reaction_time = 0;
static bool initialized = false;

// Previous acceleration reading for delta calculation
static float prev_accel_x = 0.0f;
static bool has_prev_reading = false;

// Track if we already handled this pressure event
static bool pressure_handled = false;

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

static bool is_cooldown_expired(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_reaction_time);
    return elapsed_ms >= REACTION_COOLDOWN_MS;
}

static void update_reaction_time(void)
{
    last_reaction_time = xTaskGetTickCount();
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void reaction_init(void)
{
    ESP_LOGI(TAG, "Reaction system initialized");
    ESP_LOGI(TAG, "  IMU push → 3 cycles");
    ESP_LOGI(TAG, "  Pressure only → 1 cycle");
    ESP_LOGI(TAG, "  IMU delta threshold: %.1f m/s²", REACTION_DELTA_THRESHOLD);
    ESP_LOGI(TAG, "  Cooldown: %d ms", REACTION_COOLDOWN_MS);
    
    prev_accel_x = 0.0f;
    has_prev_reading = false;
    pressure_handled = false;
    
    // Initialize subsystems
    gyro_balance_init();
    servo_pressure_init();
    
    initialized = true;
}

void reaction_process_imu(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    // Process gyro balance (toggle detection + stabilization)
    gyro_balance_process(data);
    
    // Skip reactions if gyro balance is active
    if (gyro_balance_is_enabled()) {
        return;
    }
    
    // Check cooldown
    if (!is_cooldown_expired()) {
        return;
    }
    
    // ═══════════════════════════════════════════════════════
    // IMU DELTA CALCULATION
    // ═══════════════════════════════════════════════════════
    
    float current_accel_x = data->accel_x;
    
    if (!has_prev_reading) {
        prev_accel_x = current_accel_x;
        has_prev_reading = true;
        return;
    }
    
    float delta = current_accel_x - prev_accel_x;
    prev_accel_x = current_accel_x;
    
    // ═══════════════════════════════════════════════════════
    // IMU PUSH DETECTION (Priority - 3 cycles)
    // ═══════════════════════════════════════════════════════
    
    // Front push via IMU
    if (delta >= REACTION_DELTA_THRESHOLD && current_accel_x >= REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "🏃 IMU front push! (delta: +%.2f, accel: %.2f) → 3 cycles",
                 delta, current_accel_x);
        update_reaction_time();
        pressure_handled = true;  // Prevent pressure trigger right after
        walk_forward_play(3);
        return;
    }
    
    // Back push via IMU
    if (delta <= -REACTION_DELTA_THRESHOLD && current_accel_x <= -REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "⬅️ IMU back push! (delta: %.2f, accel: %.2f) → 3 cycles",
                 delta, current_accel_x);
        update_reaction_time();
        pressure_handled = true;  // Prevent pressure trigger right after
        walk_forward_play(3);
        return;
    }
}

// ═══════════════════════════════════════════════════════
// PRESSURE DETECTION TASK (100Hz)
// ═══════════════════════════════════════════════════════

void reaction_pressure_task(void *arg)
{
    ESP_LOGI(TAG, "Pressure task started (100Hz)");
    
    while (1) {
        // Wait for initialization
        if (!initialized) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Skip if gyro balance active
        if (gyro_balance_is_enabled()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Update pressure readings
        servo_pressure_update();
        
        bool front_pressure = servo_pressure_check_front();
        bool back_pressure = servo_pressure_check_back();
        
        // Reset handled flag when released
        if (!front_pressure && !back_pressure) {
            pressure_handled = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Skip if already handled or in cooldown
        if (pressure_handled || !is_cooldown_expired()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Trigger reaction
        if (front_pressure) {
            ESP_LOGI(TAG, "🚶 Front pressure → 1 cycle");
            pressure_handled = true;
            update_reaction_time();
            walk_forward_play(1);
        } else if (back_pressure) {
            ESP_LOGI(TAG, "🚶 Back pressure → 1 cycle");
            pressure_handled = true;
            update_reaction_time();
            walk_forward_play(1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz polling
    }
}