/**
 * @file sts3032_servo.c
 * @brief STS3032 Servo Control Implementation
 */

#include "sts3032_servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STS_SERVO";

// ═══════════════════════════════════════════════════════
// CONVERSION FUNCTIONS
// ═══════════════════════════════════════════════════════

uint16_t sts_angle_to_position(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 360) angle = 360;
    return (uint16_t)(angle * 4095.0 / 360.0);
}

float sts_position_to_angle(uint16_t position) {
    return (float)position * 360.0 / 4095.0;
}

// ═══════════════════════════════════════════════════════
// BASIC SERVO FUNCTIONS
// ═══════════════════════════════════════════════════════

bool sts_servo_ping(uint8_t id) {
    sts_send_packet(id, STS_PING, NULL, 0);
    
    uint8_t response[32];
    int len;
    
    return sts_read_response(response, 32, &len);
}

int sts_servo_scan_bus(uint8_t start_id, uint8_t end_id) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  SCANNING SERVO BUS                   ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "Scanning IDs %d to %d...", start_id, end_id);
    ESP_LOGI(TAG, "");
    
    int found_count = 0;
    
    for (uint8_t id = start_id; id <= end_id; id++) {
        if (sts_servo_ping(id)) {
            ESP_LOGI(TAG, "✓ Servo ID %d is online!", id);
            found_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Scan complete. Found %d servo(s).", found_count);
    ESP_LOGI(TAG, "");
    
    return found_count;
}

void sts_servo_enable_torque(uint8_t id, bool enable) {
    ESP_LOGI(TAG, "Servo ID %d: Torque %s", id, enable ? "ON" : "OFF");
    
    uint8_t value = enable ? 1 : 0;
    sts_write_register(id, STS_TORQUE_ENABLE, &value, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// ═══════════════════════════════════════════════════════
// POSITION CONTROL
// ═══════════════════════════════════════════════════════

void sts_servo_set_angle(uint8_t id, float angle, uint16_t speed) {
    uint16_t position = sts_angle_to_position(angle);
    
    ESP_LOGI(TAG, "Servo ID %d: Moving to %.1f° (pos=%d) at speed %d",
             id, angle, position, speed);
    
    sts_servo_set_position(id, position, speed);
}

void sts_servo_set_position(uint8_t id, uint16_t position, uint16_t speed) {
    uint8_t params[6];
    
    params[0] = position & 0xFF;
    params[1] = (position >> 8) & 0xFF;
    params[2] = 0x00;  // Time Low (0 = max speed)
    params[3] = 0x00;  // Time High
    params[4] = speed & 0xFF;
    params[5] = (speed >> 8) & 0xFF;
    
    sts_write_register(id, STS_GOAL_POSITION_L, params, 6);
}

bool sts_servo_get_position(uint8_t id, uint16_t *position) {
    uint8_t data[2];
    
    if (sts_read_register(id, STS_PRESENT_POSITION_L, 2, data)) {
        if (position) {
            *position = data[0] | (data[1] << 8);
        }
        return true;
    }
    
    return false;
}

bool sts_servo_get_angle(uint8_t id, float *angle) {
    uint16_t position;
    
    if (sts_servo_get_position(id, &position)) {
        if (angle) {
            *angle = sts_position_to_angle(position);
        }
        return true;
    }
    
    return false;
}

bool sts_servo_get_speed(uint8_t id, uint16_t *speed) {
    uint8_t data[2];
    
    if (sts_read_register(id, STS_PRESENT_SPEED_L, 2, data)) {
        if (speed) {
            *speed = data[0] | (data[1] << 8);
        }
        return true;
    }
    
    return false;
}

// ═══════════════════════════════════════════════════════
// ID MANAGEMENT
// ═══════════════════════════════════════════════════════

bool sts_servo_change_id(uint8_t old_id, uint8_t new_id) {
    if (new_id == 0 || new_id > 253) {
        ESP_LOGE(TAG, "Invalid new ID %d (must be 1-253)", new_id);
        return false;
    }
    
    ESP_LOGW(TAG, "⚠️  Changing servo ID %d → %d", old_id, new_id);
    ESP_LOGW(TAG, "    This writes to EEPROM!");
    
    uint8_t value = new_id;
    sts_write_register(old_id, STS_ID, &value, 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for EEPROM write
    
    // Verify by pinging the new ID
    ESP_LOGI(TAG, "Verifying new ID %d...", new_id);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (sts_servo_ping(new_id)) {
        ESP_LOGI(TAG, "✓ Successfully changed to ID %d!", new_id);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ ID change verification failed");
        return false;
    }
}

bool sts_servo_read_id(uint8_t query_id, uint8_t *current_id) {
    uint8_t id;
    
    if (sts_read_register(query_id, STS_ID, 1, &id)) {
        if (current_id) {
            *current_id = id;
        }
        ESP_LOGI(TAG, "Servo at query ID %d has actual ID: %d", query_id, id);
        return true;
    }
    
    return false;
}

void sts_servo_broadcast_reset_id(void) {
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "🚨 BROADCAST: Resetting ALL servos to ID 1");
    ESP_LOGW(TAG, "   Disconnect all but ONE servo first!");
    ESP_LOGW(TAG, "");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint8_t value = 1;
    sts_write_register(STS_BROADCAST_ID, STS_ID, &value, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Broadcast complete. Servo(s) should now be ID 1");
}
