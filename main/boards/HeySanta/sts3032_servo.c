/**
 * @file sts3032_servo.c
 * @brief STS3032 Servo Driver - Combined Implementation for HeySanta
 */

#include "sts3032_servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "STS3032";

// Hardware configuration (stored after init)
static uart_port_t g_uart_num = UART_NUM_1;
static gpio_num_t g_txen_pin = GPIO_NUM_3;

// ═══════════════════════════════════════════════════════
// INTERNAL PROTOCOL FUNCTIONS
// ═══════════════════════════════════════════════════════

static uint8_t sts_checksum(uint8_t *buf, int len) {
    uint8_t sum = 0;
    for (int i = 2; i < len - 1; i++) {
        sum += buf[i];
    }
    return ~sum;
}

static void sts_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, int param_len) {
    uint8_t packet[64];
    
    packet[0] = STS_FRAME_HEADER;
    packet[1] = STS_FRAME_HEADER;
    packet[2] = id;
    packet[3] = param_len + 2;
    packet[4] = cmd;
    
    if (params && param_len > 0) {
        memcpy(&packet[5], params, param_len);
    }
    
    int total_len = 5 + param_len + 1;
    packet[total_len - 1] = sts_checksum(packet, total_len);
    
    // Enable transmit
    gpio_set_level(g_txen_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Send packet
    uart_flush(g_uart_num);
    uart_write_bytes(g_uart_num, packet, total_len);
    uart_wait_tx_done(g_uart_num, pdMS_TO_TICKS(100));
    
    // Disable transmit (enable receive)
    gpio_set_level(g_txen_pin, 0);
}

static bool sts_read_response(uint8_t *response, int max_len, int *out_len) {
    vTaskDelay(pdMS_TO_TICKS(5));
    
    int len = uart_read_bytes(g_uart_num, response, max_len, pdMS_TO_TICKS(100));
    
    if (len < 6) {
        return false;
    }
    
    if (response[0] != STS_FRAME_HEADER || response[1] != STS_FRAME_HEADER) {
        return false;
    }
    
    uint8_t calc_checksum = sts_checksum(response, len);
    if (calc_checksum != response[len - 1]) {
        ESP_LOGW(TAG, "Checksum mismatch: expected 0x%02X, got 0x%02X", 
                 calc_checksum, response[len - 1]);
        return false;
    }
    
    if (out_len) *out_len = len;
    return true;
}

static void sts_write_register(uint8_t id, uint8_t address, uint8_t *data, int len) {
    uint8_t params[32];
    params[0] = address;
    
    if (data && len > 0) {
        memcpy(&params[1], data, len);
    }
    
    sts_send_packet(id, STS_WRITE, params, len + 1);
}

static bool sts_read_register(uint8_t id, uint8_t address, int len, uint8_t *data) {
    uint8_t params[2];
    params[0] = address;
    params[1] = len;
    
    sts_send_packet(id, STS_READ, params, 2);
    
    uint8_t response[32];
    int resp_len;
    
    if (sts_read_response(response, 32, &resp_len)) {
        if (resp_len >= (5 + len + 1)) {
            if (data) {
                memcpy(data, &response[5], len);
            }
            return true;
        }
    }
    
    return false;
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

esp_err_t sts_protocol_init(const sts_protocol_config_t *config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Store configuration
    g_uart_num = config->uart_num;
    g_txen_pin = config->txen_pin;
    
    ESP_LOGI(TAG, "Initializing STS3032 protocol");
    ESP_LOGI(TAG, "  UART: %d, TX: %d, RX: %d, TXEN: %d, Baud: %lu",
             config->uart_num, config->tx_pin, config->rx_pin, 
             config->txen_pin, config->baud_rate);
    
    // Configure TXEN pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->txen_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(config->txen_pin, 0);
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = (int)config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(config->uart_num, 2048, 2048, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return ret;
    }
    
    ret = uart_param_config(config->uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters");
        uart_driver_delete(config->uart_num);
        return ret;
    }
    
    ret = uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin,
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        uart_driver_delete(config->uart_num);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "STS3032 protocol initialized successfully");
    
    return ESP_OK;
}

void sts_protocol_deinit(void) {
    uart_driver_delete(g_uart_num);
    ESP_LOGI(TAG, "STS3032 protocol deinitialized");
}

// ═══════════════════════════════════════════════════════
// CONVERSION FUNCTIONS
// ═══════════════════════════════════════════════════════

uint16_t sts_angle_to_position(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 360) angle = 360;
    return (uint16_t)(angle * 4095.0f / 360.0f);
}

float sts_position_to_angle(uint16_t position) {
    return (float)position * 360.0f / 4095.0f;
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
    ESP_LOGI(TAG, "Scanning servo bus (IDs %d-%d)...", start_id, end_id);
    
    int found_count = 0;
    
    for (uint8_t id = start_id; id <= end_id; id++) {
        if (sts_servo_ping(id)) {
            ESP_LOGI(TAG, "  ✓ Servo ID %d online", id);
            found_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI(TAG, "Scan complete: %d servo(s) found", found_count);
    return found_count;
}

void sts_servo_enable_torque(uint8_t id, bool enable) {
    ESP_LOGI(TAG, "Servo %d: Torque %s", id, enable ? "ON" : "OFF");
    
    uint8_t value = enable ? 1 : 0;
    sts_write_register(id, STS_TORQUE_ENABLE, &value, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

bool sts_servo_get_load(uint8_t id, int16_t *load) {
    if (!load) return false;
    
    uint8_t response[8];
    if (!sts_read_register(id, 0x28, 2, response)) {
        return false;
    }
    
    *load = (int16_t)(response[0] | (response[1] << 8));
    return true;
}

// ═══════════════════════════════════════════════════════
// POSITION CONTROL
// ═══════════════════════════════════════════════════════

void sts_servo_set_angle(uint8_t id, float angle, uint16_t speed) {
    uint16_t position = sts_angle_to_position(angle);
    ESP_LOGI(TAG, "Servo %d: angle=%.1f° pos=%d speed=%d", id, angle, position, speed);
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
    
    // Small delay to allow the servo to process the command
    // This prevents bus collisions when sending to multiple servos
    // vTaskDelay(pdMS_TO_TICKS(2));
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
    
    ESP_LOGW(TAG, "Changing servo ID %d -> %d (EEPROM write)", old_id, new_id);
    
    uint8_t value = new_id;
    sts_write_register(old_id, STS_ID, &value, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (sts_servo_ping(new_id)) {
        ESP_LOGI(TAG, "✓ ID change successful");
        return true;
    } else {
        ESP_LOGE(TAG, "✗ ID change verification failed");
        return false;
    }
}

bool sts_servo_read_id(uint8_t query_id, uint8_t *current_id) {
    uint8_t id;
    
    if (sts_read_register(query_id, STS_ID, 1, &id)) {
        if (current_id) {
            *current_id = id;
        }
        return true;
    }
    return false;
}

void sts_servo_broadcast_reset_id(void) {
    ESP_LOGW(TAG, "BROADCAST: Resetting ALL servos to ID 1");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint8_t value = 1;
    sts_write_register(STS_BROADCAST_ID, STS_ID, &value, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Broadcast complete");
}
