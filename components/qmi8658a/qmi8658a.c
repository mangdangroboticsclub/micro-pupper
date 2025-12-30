/**
 * @file qmi8658a.c
 * @brief QMI8658A IMU Driver Implementation
 * 
 * Portable driver - all configuration passed at init time.
 */

#include "qmi8658a.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "QMI8658A";

// ═══════════════════════════════════════════════════════
// STATIC VARIABLES
// ═══════════════════════════════════════════════════════

static qmi8658a_config_t g_config;
static bool g_initialized = false;

#define I2C_TIMEOUT_MS  1000

// ═══════════════════════════════════════════════════════
// I2C COMMUNICATION HELPERS
// ═══════════════════════════════════════════════════════

static esp_err_t write_reg(uint8_t reg_addr, uint8_t value)
{
    uint8_t write_buf[2] = {reg_addr, value};
    return i2c_master_write_to_device(g_config.i2c_num, g_config.i2c_addr, 
                                     write_buf, sizeof(write_buf), 
                                     I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t read_reg(uint8_t reg_addr, uint8_t *out_value)
{
    return i2c_master_write_read_device(g_config.i2c_num, g_config.i2c_addr,
                                       &reg_addr, 1,
                                       out_value, 1,
                                       I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t read_regs(uint8_t reg_addr, uint8_t *out_buf, size_t len)
{
    return i2c_master_write_read_device(g_config.i2c_num, g_config.i2c_addr,
                                       &reg_addr, 1,
                                       out_buf, len,
                                       I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// ═══════════════════════════════════════════════════════
// CONVERSION HELPERS
// ═══════════════════════════════════════════════════════

static float accel_to_ms2(int16_t raw_value)
{
    float sensitivity;
    uint8_t range_bits = (g_config.accel_range >> 4) & 0x07;
    
    switch (range_bits) {
        case 0x00: sensitivity = 16384.0f; break;  // ±2g
        case 0x01: sensitivity = 8192.0f;  break;  // ±4g
        case 0x02: sensitivity = 4096.0f;  break;  // ±8g
        case 0x03: sensitivity = 2048.0f;  break;  // ±16g
        default:   sensitivity = 4096.0f;
    }
    
    return (raw_value / sensitivity) * 9.81f;
}

static float gyro_to_dps(int16_t raw_value)
{
    float sensitivity;
    uint8_t range_bits = (g_config.gyro_range >> 4) & 0x07;
    
    switch (range_bits) {
        case 0x00: sensitivity = 2048.0f; break;   // ±16 dps
        case 0x01: sensitivity = 1024.0f; break;   // ±32 dps
        case 0x02: sensitivity = 512.0f;  break;   // ±64 dps
        case 0x03: sensitivity = 256.0f;  break;   // ±128 dps
        case 0x04: sensitivity = 128.0f;  break;   // ±256 dps
        case 0x05: sensitivity = 64.0f;   break;   // ±512 dps
        case 0x06: sensitivity = 32.0f;   break;   // ±1024 dps
        case 0x07: sensitivity = 16.0f;   break;   // ±2048 dps
        default:   sensitivity = 64.0f;
    }
    
    return raw_value / sensitivity;
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool qmi8658a_init(const qmi8658a_config_t *config)
{
    esp_err_t ret;
    uint8_t chip_id;
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Configuration required");
        return false;
    }
    
    g_config = *config;
    
    ESP_LOGI(TAG, "Initializing on I2C%d (SDA: GPIO%d, SCL: GPIO%d, addr: 0x%02X)",
             g_config.i2c_num, g_config.sda_pin, g_config.scl_pin, g_config.i2c_addr);
    
    // Configure I2C
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = g_config.sda_pin,
        .scl_io_num = g_config.scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = g_config.i2c_freq_hz,
    };
    
    ret = i2c_param_config(g_config.i2c_num, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(g_config.i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Verify chip ID
    ret = read_reg(QMI8658A_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "CHIP_ID: 0x%02X (expected: 0x%02X)", chip_id, QMI8658A_CHIP_ID);
    if (chip_id != QMI8658A_CHIP_ID) {
        ESP_LOGW(TAG, "CHIP_ID mismatch!");
    }
    
    // Configure CTRL1 - Little-Endian + auto-increment
    ret = write_reg(QMI8658A_REG_CTRL1, 0x40);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL1");
        return false;
    }
    
    // Configure CTRL2 - Accelerometer
    uint8_t ctrl2 = (g_config.accel_range & 0x70) | (g_config.accel_odr & 0x0F);
    ret = write_reg(QMI8658A_REG_CTRL2, ctrl2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL2");
        return false;
    }
    
    // Configure CTRL3 - Gyroscope
    uint8_t ctrl3 = (g_config.gyro_range & 0x70) | (g_config.gyro_odr & 0x0F);
    ret = write_reg(QMI8658A_REG_CTRL3, ctrl3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL3");
        return false;
    }
    
    // Enable both sensors (CTRL7: aEN=bit0, gEN=bit1)
    ret = write_reg(QMI8658A_REG_CTRL7, 0x03);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Prime the data registers
    uint8_t status;
    for (int i = 0; i < 5; i++) {
        read_reg(QMI8658A_REG_STATUS0, &status);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    ESP_LOGI(TAG, "Status: 0x%02X", status);
    
    g_initialized = true;
    ESP_LOGI(TAG, "Initialized successfully!");
    return true;
}

bool qmi8658a_check_device(void)
{
    uint8_t chip_id;
    esp_err_t ret = read_reg(QMI8658A_REG_CHIP_ID, &chip_id);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID");
        return false;
    }
    
    if (chip_id != QMI8658A_CHIP_ID) {
        ESP_LOGW(TAG, "CHIP_ID mismatch: 0x%02X", chip_id);
        return false;
    }
    
    ESP_LOGI(TAG, "Device check passed");
    return true;
}

bool qmi8658a_read_raw(qmi8658a_raw_data_t *out_data)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return false;
    }
    
    // Read status first to latch data
    uint8_t status;
    esp_err_t ret = read_reg(QMI8658A_REG_STATUS0, &status);
    if (ret != ESP_OK) {
        return false;
    }
    
    // Read 12 bytes: 6 accel + 6 gyro
    uint8_t data[12];
    ret = read_regs(QMI8658A_REG_AX_L, data, 12);
    if (ret != ESP_OK) {
        return false;
    }
    
    // Combine bytes (little-endian)
    out_data->accel_x = (int16_t)((data[1] << 8) | data[0]);
    out_data->accel_y = (int16_t)((data[3] << 8) | data[2]);
    out_data->accel_z = (int16_t)((data[5] << 8) | data[4]);
    out_data->gyro_x = (int16_t)((data[7] << 8) | data[6]);
    out_data->gyro_y = (int16_t)((data[9] << 8) | data[8]);
    out_data->gyro_z = (int16_t)((data[11] << 8) | data[10]);
    
    return true;
}

bool qmi8658a_read(qmi8658a_data_t *out_data)
{
    qmi8658a_raw_data_t raw;
    
    if (!qmi8658a_read_raw(&raw)) {
        return false;
    }
    
    out_data->accel_x = accel_to_ms2(raw.accel_x);
    out_data->accel_y = accel_to_ms2(raw.accel_y);
    out_data->accel_z = accel_to_ms2(raw.accel_z);
    
    out_data->gyro_x = gyro_to_dps(raw.gyro_x);
    out_data->gyro_y = gyro_to_dps(raw.gyro_y);
    out_data->gyro_z = gyro_to_dps(raw.gyro_z);
    
    out_data->accel_magnitude = sqrtf(
        out_data->accel_x * out_data->accel_x +
        out_data->accel_y * out_data->accel_y +
        out_data->accel_z * out_data->accel_z);
    
    return true;
}

void qmi8658a_debug_status(void)
{
    uint8_t val;
    
    ESP_LOGI(TAG, "=== Diagnostic Status ===");
    
    if (read_reg(QMI8658A_REG_CHIP_ID, &val) == ESP_OK) {
        ESP_LOGI(TAG, "CHIP_ID: 0x%02X %s", val, 
                 (val == QMI8658A_CHIP_ID) ? "✓" : "✗");
    }
    
    if (read_reg(QMI8658A_REG_CTRL1, &val) == ESP_OK) {
        ESP_LOGI(TAG, "CTRL1: 0x%02X (BE=%d, ADDR_AI=%d)", val,
                 (val >> 5) & 1, (val >> 6) & 1);
    }
    
    if (read_reg(QMI8658A_REG_CTRL2, &val) == ESP_OK) {
        ESP_LOGI(TAG, "CTRL2: 0x%02X (aFS=%d, aODR=%d)", val,
                 (val >> 4) & 0x07, val & 0x0F);
    }
    
    if (read_reg(QMI8658A_REG_CTRL3, &val) == ESP_OK) {
        ESP_LOGI(TAG, "CTRL3: 0x%02X (gFS=%d, gODR=%d)", val,
                 (val >> 4) & 0x07, val & 0x0F);
    }
    
    if (read_reg(QMI8658A_REG_CTRL7, &val) == ESP_OK) {
        ESP_LOGI(TAG, "CTRL7: 0x%02X (aEN=%d, gEN=%d)", val,
                 val & 1, (val >> 1) & 1);
    }
    
    // Read raw data
    qmi8658a_raw_data_t raw;
    if (qmi8658a_read_raw(&raw)) {
        ESP_LOGI(TAG, "RAW ACCEL: X=%6d Y=%6d Z=%6d",
                 raw.accel_x, raw.accel_y, raw.accel_z);
        ESP_LOGI(TAG, "RAW GYRO:  X=%6d Y=%6d Z=%6d",
                 raw.gyro_x, raw.gyro_y, raw.gyro_z);
    }
    
    ESP_LOGI(TAG, "=== End Diagnostic ===");
}

const qmi8658a_config_t* qmi8658a_get_config(void)
{
    return &g_config;
}
