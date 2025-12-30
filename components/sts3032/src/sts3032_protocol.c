/**
 * @file sts3032_protocol.c
 * @brief STS3032 Servo Protocol Implementation
 */

#include "sts3032_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "STS_PROTOCOL";

// Hardware configuration (stored after init)
static uart_port_t g_uart_num = UART_NUM_1;
static gpio_num_t g_txen_pin = GPIO_NUM_3;

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
    ESP_LOGI(TAG, "  UART: %d", config->uart_num);
    ESP_LOGI(TAG, "  TX Pin: %d", config->tx_pin);
    ESP_LOGI(TAG, "  RX Pin: %d", config->rx_pin);
    ESP_LOGI(TAG, "  TXEN Pin: %d", config->txen_pin);
    ESP_LOGI(TAG, "  Baud Rate: %lu", config->baud_rate);
    
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
        .baud_rate = config->baud_rate,
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
// PROTOCOL FUNCTIONS
// ═══════════════════════════════════════════════════════

uint8_t sts_checksum(uint8_t *buf, int len) {
    uint8_t sum = 0;
    for (int i = 2; i < len - 1; i++) {
        sum += buf[i];
    }
    return ~sum;
}

void sts_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, int param_len) {
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

bool sts_read_response(uint8_t *response, int max_len, int *out_len) {
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

void sts_write_register(uint8_t id, uint8_t address, uint8_t *data, int len) {
    uint8_t params[32];
    params[0] = address;
    
    if (data && len > 0) {
        memcpy(&params[1], data, len);
    }
    
    sts_send_packet(id, STS_WRITE, params, len + 1);
}

bool sts_read_register(uint8_t id, uint8_t address, int len, uint8_t *data) {
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
