/**
 * @file reaction_config.c
 * @brief Reaction system implementation
 * 
 * Monitors IMU data and triggers animations when thresholds are met.
 */

#include "reaction_config.h"
#include "gyro_balance.h"
#include "walk_forward_reaction.h"
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

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * @brief Check if enough time has passed since last reaction
 */
static bool is_cooldown_expired(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_reaction_time);
    return elapsed_ms >= REACTION_COOLDOWN_MS;
}

/**
 * @brief Update the last reaction timestamp
 */
static void update_reaction_time(void)
{
    last_reaction_time = xTaskGetTickCount();
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void reaction_init(void)
{
    ESP_LOGI(TAG, "Reaction system initialized (delta-based detection)");
    ESP_LOGI(TAG, "Delta threshold: %.1f m/s², Min accel: %.1f m/s²",
             REACTION_DELTA_THRESHOLD, REACTION_MIN_ACCEL);
    ESP_LOGI(TAG, "Cooldown: %d ms", REACTION_COOLDOWN_MS);
    
    prev_accel_x = 0.0f;
    has_prev_reading = false;
    
    // Initialize the gyro balance subsystem
    gyro_balance_init();
    
    initialized = true;
}

void reaction_process_imu(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    // Process gyro balance (toggle detection + stabilization)
    gyro_balance_process(data);
    
    float current_accel_x = data->accel_x;
    
    // Need a previous reading to calculate delta
    if (!has_prev_reading) {
        prev_accel_x = current_accel_x;
        has_prev_reading = true;
        return;
    }
    
    // Calculate delta (change from previous reading)
    float delta = current_accel_x - prev_accel_x;
    
    // Store current as previous for next iteration
    prev_accel_x = current_accel_x;
    
    // Skip push reactions if gyro balance is active
    if (gyro_balance_is_enabled()) {
        return;
    }
    
    // Check cooldown
    if (!is_cooldown_expired()) {
        return;
    }
    
    // Front push: large positive delta AND current reading is positive
    // (acceleration suddenly increased in +X direction)
    if (delta >= REACTION_DELTA_THRESHOLD && current_accel_x >= REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "Front push detected! (delta: +%.2f, accel: %.2f m/s²)",
                 delta, current_accel_x);
        update_reaction_time();
        walk_forward_play(3);
        return;
    }
    
    // Back push: large negative delta AND current reading is negative
    // (acceleration suddenly increased in -X direction)
    if (delta <= -REACTION_DELTA_THRESHOLD && current_accel_x <= -REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "Back push detected! (delta: %.2f, accel: %.2f m/s²)",
                 delta, current_accel_x);
        update_reaction_time();
        // TODO: Add backward reaction animation here
        ESP_LOGW(TAG, "Back push reaction not yet implemented");
        return;
    }
}
