#include "wifi_board.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"
#include "mcp_server.h"
#include "audio/codecs/santa_audio_codec.h"
#include "assets/lang_config.h"

// STS3032 Servo Driver (local to HeySanta)
extern "C" {
#include "sts3032_servo.h"
}

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include "esp_lcd_st7735.h"
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "emoji_display.h"
#include <string.h>
#include <cstdlib>
#include <cmath>

#define TAG "HeySanta"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

// ═══════════════════════════════════════════════════════
// SERVO CONFIGURATION (using sts3032 driver)
// ═══════════════════════════════════════════════════════
#define SERVO_UART_NUM      UART_NUM_1
#define SERVO_TX_PIN        GPIO_NUM_10
#define SERVO_RX_PIN        GPIO_NUM_11
#define SERVO_TXEN_PIN      GPIO_NUM_3
#define SERVO_BAUD_RATE     1000000

// Servo IDs (same convention as dog_config)
#define SERVO_FR            1   // Front Right
#define SERVO_FL            2   // Front Left
#define SERVO_BR            3   // Back Right
#define SERVO_BL            4   // Back Left
#define SERVO_COUNT         4

// Stance angles (from left-side perspective)
#define STANCE_FRONT        90.0f   // Front legs neutral
#define STANCE_BACK         270.0f  // Back legs neutral
#define SWING_AMPLITUDE     25.0f   // Max deviation from stance

// Speed presets
#define SPEED_SLOW          300
#define SPEED_MEDIUM        700
#define SPEED_FAST          1500
#define SPEED_VERY_FAST     3000
#define SPEED_MAX           4095

// Dynamic speed for stance (prevents shaking)
#define STANCE_SPEED_MIN        20
#define STANCE_SPEED_MAX        1500
#define STANCE_SPEED_THRESHOLD  40.0f
#define STANCE_SPEED_CURVE      1.4f

// Angle reversal macros for right-side servos
#define REVERSE_ANGLE(angle)    (360.0f - (angle))
#define IS_RIGHT_SIDE(id)       ((id) == SERVO_FR || (id) == SERVO_BR)
#define IS_FRONT_LEG(id)        ((id) == SERVO_FR || (id) == SERVO_FL)

// ═══════════════════════════════════════════════════════
// QMI8658A IMU CONFIG
// ═══════════════════════════════════════════════════════
#define IMU_SDA_PIN         GPIO_NUM_1
#define IMU_SCL_PIN         GPIO_NUM_2
#define IMU_I2C_FREQ_HZ     400000

#define IMU_I2C_ADDR_PRIMARY   0x6A
#define IMU_I2C_ADDR_SECONDARY 0x6B

#define IMU_CHIP_ID         0x05

// QMI8658A Registers
#define QMI_REG_CHIP_ID     0x00
#define QMI_REG_REVISION    0x01
#define QMI_REG_CTRL1       0x02
#define QMI_REG_CTRL2       0x03
#define QMI_REG_CTRL3       0x04
#define QMI_REG_CTRL4       0x05
#define QMI_REG_CTRL5       0x06
#define QMI_REG_CTRL6       0x07
#define QMI_REG_CTRL7       0x08
#define QMI_REG_CTRL8       0x09
#define QMI_REG_CTRL9       0x0A
#define QMI_REG_STATUS0     0x2E
#define QMI_REG_AX_L        0x35

// Gyro Balance Config
#define GYRO_BALANCE_ENABLED_DEFAULT    false
#define GYRO_BALANCE_MAX_CORRECTION     90.0f
#define GYRO_BALANCE_DEADZONE           0.5f
#define GYRO_BALANCE_GAIN               1.6f
#define GYRO_BALANCE_SMOOTHING          0.3f
#define GYRO_BALANCE_DECAY              0.98f
#define GYRO_BALANCE_UPDATE_INTERVAL_MS 50
#define IMU_UPDATE_INTERVAL_MS          50
#define GYRO_BALANCE_SPEED_MIN          150
#define GYRO_BALANCE_SPEED_MAX          2000
#define GYRO_BALANCE_SPEED_THRESHOLD    10.0f
#define GYRO_BALANCE_SPEED_CURVE        1.2f
#define GYRO_BALANCE_TOGGLE_THRESHOLD   150.0f
#define GYRO_BALANCE_TOGGLE_WINDOW_MS   1000
#define GYRO_BALANCE_TOGGLE_COOLDOWN_MS 1500

struct ImuData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float accel_magnitude;
};

// Shared keyframe structure for movement sequences (FR, FL, BR, BL)
struct Keyframe {
    float fr;
    float fl;
    float br;
    float bl;
    uint16_t speed;
    uint16_t delay_ms;
};

// ═══════════════════════════════════════════════════════
// GLOBAL CONVERSATION LOCK
// ═══════════════════════════════════════════════════════
static volatile bool g_conversation_active = false;

void SetConversationActive(bool active) {
    g_conversation_active = active;
    if (active) {
        ESP_LOGI(TAG, "🔒 CONVERSATION ACTIVE - IMU balance paused");
    } else {
        ESP_LOGI(TAG, "🔓 CONVERSATION ENDED - IMU balance resumed");
    }
}

bool IsConversationActive() {
    return g_conversation_active;
}

// ═══════════════════════════════════════════════════════
// IMU CONTROLLER
// ═══════════════════════════════════════════════════════
class ImuController {
private:
    bool initialized_ = false;
    uint8_t detected_address_ = 0;
    i2c_master_dev_handle_t imu_device_ = nullptr;
    
    esp_err_t WriteReg(uint8_t reg, uint8_t value) {
        if (!imu_device_) return ESP_ERR_INVALID_STATE;
        uint8_t buf[2] = {reg, value};
        return i2c_master_transmit(imu_device_, buf, 2, 1000);
    }
    
    esp_err_t ReadReg(uint8_t reg, uint8_t* value) {
        if (!imu_device_) return ESP_ERR_INVALID_STATE;
        return i2c_master_transmit_receive(imu_device_, &reg, 1, value, 1, 1000);
    }
    
    esp_err_t ReadRegs(uint8_t reg, uint8_t* buf, size_t len) {
        if (!imu_device_) return ESP_ERR_INVALID_STATE;
        return i2c_master_transmit_receive(imu_device_, &reg, 1, buf, len, 1000);
    }
    
    float AccelToMs2(int16_t raw) {
        return (raw / 4096.0f) * 9.81f;
    }
    
    float GyroToDps(int16_t raw) {
        return raw / 64.0f;
    }

public:
    bool Initialize(i2c_master_bus_handle_t i2c_bus) {
        ESP_LOGI(TAG, "=== QMI8658A IMU Initialization ===");
        
        if (!i2c_bus) {
            ESP_LOGE(TAG, "I2C bus handle is null");
            return false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        uint8_t addresses[] = {IMU_I2C_ADDR_PRIMARY, IMU_I2C_ADDR_SECONDARY};
        bool found = false;
        
        for (int i = 0; i < 2; i++) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addresses[i],
                .scl_speed_hz = IMU_I2C_FREQ_HZ,
            };
            
            i2c_master_dev_handle_t temp_dev;
            if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &temp_dev) == ESP_OK) {
                uint8_t chip_id;
                uint8_t reg = QMI_REG_CHIP_ID;
                if (i2c_master_transmit_receive(temp_dev, &reg, 1, &chip_id, 1, 1000) == ESP_OK) {
                    if (chip_id == IMU_CHIP_ID) {
                        ESP_LOGI(TAG, "✓ IMU found at address 0x%02X (chip ID: 0x%02X)", addresses[i], chip_id);
                        imu_device_ = temp_dev;
                        detected_address_ = addresses[i];
                        found = true;
                        break;
                    }
                }
                i2c_master_bus_rm_device(temp_dev);
            }
        }
        
        if (!found) {
            ESP_LOGE(TAG, "✗ IMU not found at 0x6A or 0x6B");
            return false;
        }
        
        WriteReg(QMI_REG_CTRL1, 0x80);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        WriteReg(QMI_REG_CTRL1, 0x40);
        WriteReg(QMI_REG_CTRL2, 0x24);
        WriteReg(QMI_REG_CTRL3, 0x54);
        WriteReg(QMI_REG_CTRL7, 0x03);
        WriteReg(QMI_REG_CTRL9, 0x00);
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        uint8_t status;
        if (ReadReg(QMI_REG_STATUS0, &status) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register");
            return false;
        }
        
        ESP_LOGI(TAG, "✓ IMU initialized (status: 0x%02X)", status);
        ESP_LOGI(TAG, "  Config: ±8g accel, ±512dps gyro, 500Hz ODR");
        
        initialized_ = true;
        return true;
    }
    
    bool Read(ImuData* data) {
        if (!initialized_) return false;
        
        uint8_t buf[12];
        if (ReadRegs(QMI_REG_AX_L, buf, 12) != ESP_OK) {
            return false;
        }
        
        int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);
        int16_t raw_gx = (int16_t)((buf[7] << 8) | buf[6]);
        int16_t raw_gy = (int16_t)((buf[9] << 8) | buf[8]);
        int16_t raw_gz = (int16_t)((buf[11] << 8) | buf[10]);
        
        data->accel_x = AccelToMs2(raw_ax);
        data->accel_y = AccelToMs2(raw_ay);
        data->accel_z = AccelToMs2(raw_az);
        data->gyro_x = GyroToDps(raw_gx);
        data->gyro_y = GyroToDps(raw_gy);
        data->gyro_z = GyroToDps(raw_gz);
        data->accel_magnitude = sqrtf(data->accel_x * data->accel_x + 
                                       data->accel_y * data->accel_y + 
                                       data->accel_z * data->accel_z);
        return true;
    }
    
    bool IsInitialized() const { return initialized_; }
    uint8_t GetAddress() const { return detected_address_; }
};

// ═══════════════════════════════════════════════════════
// GYRO BALANCE CONTROLLER
// ═══════════════════════════════════════════════════════
class GyroBalanceController {
private:
    bool enabled_ = false;
    float accumulated_angle_ = 0.0f;
    float last_correction_ = 0.0f;
    uint32_t last_update_time_ = 0;
    
    enum ToggleState { IDLE, FIRST_ROTATION, WAITING_REVERSE };
    ToggleState toggle_state_ = IDLE;
    bool first_rotation_positive_ = false;
    uint32_t first_rotation_time_ = 0;
    uint32_t last_toggle_time_ = 0;
    
    uint16_t CalculateDynamicSpeed(float angle_delta) {
        float abs_delta = fabsf(angle_delta);
        if (abs_delta < 0.5f) return GYRO_BALANCE_SPEED_MIN;
        
        float normalized = abs_delta / GYRO_BALANCE_SPEED_THRESHOLD;
        if (normalized > 1.0f) normalized = 1.0f;
        
        float curved = powf(normalized, GYRO_BALANCE_SPEED_CURVE);
        uint16_t speed = GYRO_BALANCE_SPEED_MIN + 
                        (uint16_t)(curved * (GYRO_BALANCE_SPEED_MAX - GYRO_BALANCE_SPEED_MIN));
        
        if (speed > GYRO_BALANCE_SPEED_MAX) speed = GYRO_BALANCE_SPEED_MAX;
        return speed;
    }
    
public:
    bool ProcessToggle(float gyro_x) {
        return false;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if ((now - last_toggle_time_) < GYRO_BALANCE_TOGGLE_COOLDOWN_MS) {
            return false;
        }
        
        bool toggled = false;
        
        switch (toggle_state_) {
            case IDLE:
                if (fabsf(gyro_x) > GYRO_BALANCE_TOGGLE_THRESHOLD) {
                    toggle_state_ = FIRST_ROTATION;
                    first_rotation_positive_ = (gyro_x > 0);
                    first_rotation_time_ = now;
                }
                break;
                
            case FIRST_ROTATION:
                if ((now - first_rotation_time_) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                    toggle_state_ = IDLE;
                } else if (fabsf(gyro_x) < GYRO_BALANCE_TOGGLE_THRESHOLD * 0.5f) {
                    toggle_state_ = WAITING_REVERSE;
                }
                break;
                
            case WAITING_REVERSE:
                if ((now - first_rotation_time_) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                    toggle_state_ = IDLE;
                } else if (fabsf(gyro_x) > GYRO_BALANCE_TOGGLE_THRESHOLD) {
                    bool current_positive = (gyro_x > 0);
                    if (current_positive != first_rotation_positive_) {
                        enabled_ = !enabled_;
                        toggled = true;
                        last_toggle_time_ = now;
                        toggle_state_ = IDLE;
                        
                        accumulated_angle_ = 0.0f;
                        last_correction_ = 0.0f;
                        
                        ESP_LOGI("GyroBalance", "TOGGLE! Balance mode: %s", 
                                 enabled_ ? "ENABLED" : "DISABLED");
                    } else {
                        toggle_state_ = IDLE;
                    }
                }
                break;
        }
        
        return toggled;
    }
    
    struct BalanceResult {
        float front_offset;
        float back_offset;
        uint16_t speed;
    };
    
    BalanceResult Calculate(float gyro_y, float dt_sec) {
        BalanceResult result = {0.0f, 0.0f, GYRO_BALANCE_SPEED_MIN};
        
        if (!enabled_) return result;
        
        float filtered_gyro = gyro_y;
        if (fabsf(filtered_gyro) < GYRO_BALANCE_DEADZONE) {
            filtered_gyro = 0.0f;
        }
        
        float raw_integration = filtered_gyro * dt_sec;
        accumulated_angle_ += raw_integration * GYRO_BALANCE_SMOOTHING;
        accumulated_angle_ *= GYRO_BALANCE_DECAY;
        
        float correction = accumulated_angle_ * GYRO_BALANCE_GAIN;
        
        if (correction > GYRO_BALANCE_MAX_CORRECTION) correction = GYRO_BALANCE_MAX_CORRECTION;
        if (correction < -GYRO_BALANCE_MAX_CORRECTION) correction = -GYRO_BALANCE_MAX_CORRECTION;
        
        float correction_delta = correction - last_correction_;
        result.speed = CalculateDynamicSpeed(correction_delta);
        last_correction_ = correction;
        
        result.front_offset = correction;
        result.back_offset = -correction;
        
        return result;
    }
    
    void Enable(bool enable) { 
        enabled_ = enable;
        if (!enable) {
            accumulated_angle_ = 0.0f;
            last_correction_ = 0.0f;
        }
    }
    bool IsEnabled() const { return enabled_; }
    void Reset() { 
        accumulated_angle_ = 0.0f; 
        last_correction_ = 0.0f;
    }
};

// ═══════════════════════════════════════════════════════
// SERVO CONTROLLER - Using sts3032 driver library
// ═══════════════════════════════════════════════════════
class ServoController {
private:
    uint16_t default_speed_ = SPEED_MAX;
    bool is_flipped_ = false;
    bool initialized_ = false;

    /**
     * @brief Apply angle reversal for right-side servos
     * Right-side servos are mounted mirrored, so we reverse angles
     */
    static float ApplyReversal(uint8_t servo_id, float angle) {
        if (IS_RIGHT_SIDE(servo_id)) {
            return REVERSE_ANGLE(angle);
        }
        return angle;
    }

    /**
     * @brief Get base stance angle for a servo (before reversal)
     */
    static float GetBaseStance(uint8_t servo_id) {
        if (IS_FRONT_LEG(servo_id)) {
            return STANCE_FRONT;
        } else {
            return STANCE_BACK;
        }
    }

    /**
     * @brief Calculate dynamic speed based on angle delta
     */
    static uint16_t CalculateDynamicSpeed(float angle_delta) {
        float abs_delta = fabsf(angle_delta);
        float speed_ratio = abs_delta / STANCE_SPEED_THRESHOLD;
        
        if (speed_ratio > 1.0f) {
            speed_ratio = 1.0f;
        }
        
        // Apply power curve to bias toward lower speeds
        speed_ratio = powf(speed_ratio, STANCE_SPEED_CURVE);
        
        return (uint16_t)(STANCE_SPEED_MIN + 
                          speed_ratio * (STANCE_SPEED_MAX - STANCE_SPEED_MIN));
    }

    /**
     * @brief Move a single servo with automatic reversal for right side
     */
    void ServoMove(uint8_t servo_id, float angle, uint16_t speed) {
        float actual_angle = ApplyReversal(servo_id, angle);
        sts_servo_set_angle(servo_id, actual_angle, speed);
    }

    /**
     * @brief Move all servos with automatic reversal for right side
     * @param angle_fr Front-right angle (from left perspective, will be reversed)
     * @param angle_fl Front-left angle
     * @param angle_br Back-right angle (from left perspective, will be reversed)
     * @param angle_bl Back-left angle
     * @param speed Movement speed
     */
    void ServoMoveAll(float angle_fr, float angle_fl, float angle_br, float angle_bl, uint16_t speed) {
        // Apply reversal to right-side servos
        float actual_fr = ApplyReversal(SERVO_FR, angle_fr);
        float actual_br = ApplyReversal(SERVO_BR, angle_br);
        
        // Left side uses angles directly
        float actual_fl = angle_fl;
        float actual_bl = angle_bl;
        
        // Move all servos using sts3032 driver
        sts_servo_set_angle(SERVO_FR, actual_fr, speed);
        sts_servo_set_angle(SERVO_FL, actual_fl, speed);
        sts_servo_set_angle(SERVO_BR, actual_br, speed);
        sts_servo_set_angle(SERVO_BL, actual_bl, speed);
    }

public:
    void Initialize() {
        ESP_LOGI(TAG, "Initializing servo controller using sts3032 driver...");
        
        // Initialize the servo protocol using sts3032 driver
        sts_protocol_config_t protocol_config = {
            .uart_num = SERVO_UART_NUM,
            .tx_pin = SERVO_TX_PIN,
            .rx_pin = SERVO_RX_PIN,
            .txen_pin = SERVO_TXEN_PIN,
            .baud_rate = SERVO_BAUD_RATE,
        };
        
        esp_err_t ret = sts_protocol_init(&protocol_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize servo protocol: %s", esp_err_to_name(ret));
            return;
        }
        
        ESP_LOGI(TAG, "Servo protocol initialized");
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check all servos
        ESP_LOGI(TAG, "Checking servos...");
        int found = sts_servo_scan_bus(1, SERVO_COUNT);
        if (found < SERVO_COUNT) {
            ESP_LOGW(TAG, "Only %d of %d servos responding", found, SERVO_COUNT);
        }
        
        // Enable torque on all servos
        ESP_LOGI(TAG, "Enabling servo torque...");
        for (uint8_t id = 1; id <= SERVO_COUNT; id++) {
            sts_servo_enable_torque(id, true);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        
        initialized_ = true;
        MoveInit();
        
        ESP_LOGI(TAG, "Servo controller initialized - Stance: Front=%.0f° Back=%.0f°",
                 STANCE_FRONT, STANCE_BACK);
    }

    void MoveInit() {
        ESP_LOGI(TAG, "Dog: Moving to stance position");
        
        // Use unified angles - reversal happens automatically
        ServoMoveAll(
            STANCE_FRONT,  // FR
            STANCE_FRONT,  // FL
            STANCE_BACK,   // BR
            STANCE_BACK,   // BL
            default_speed_
        );
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    void MoveReset() {
        ESP_LOGI(TAG, "Dog: Resetting to stance (flipped: %s)", is_flipped_ ? "yes" : "no");
        SetConversationActive(true);  // **BLOCK IMU**
        
        if (is_flipped_) {
            // When flipped, front legs use back stance and back legs use front stance
            ServoMoveAll(
                STANCE_BACK,   // FR
                STANCE_BACK,   // FL
                STANCE_FRONT,  // BR
                STANCE_FRONT,  // BL
                default_speed_
            );
        } else {
            // Normal orientation
            ServoMoveAll(
                STANCE_FRONT,  // FR
                STANCE_FRONT,  // FL
                STANCE_BACK,   // BR
                STANCE_BACK,   // BL
                default_speed_
            );
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void ApplyBalance(float front_offset, float back_offset, uint16_t speed) {
        if (IsConversationActive()) return;  // **CHECK CONVERSATION LOCK**
        
        // Apply balance offsets to stance angles
        float front_angle = STANCE_FRONT + front_offset;
        float back_angle = STANCE_BACK - back_offset;
        
        ServoMoveAll(front_angle, front_angle, back_angle, back_angle, speed);
    }

    // ═══════════════════════════════════════════════════════
    // MOVEMENT SEQUENCES
    // ═══════════════════════════════════════════════════════
    void WalkForward(int loops = 3) {
        ESP_LOGI(TAG, "Dog: >>> WALKING FORWARD <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        // Walk cycle implemented as keyframes (FR, FL, BR, BL)

        Keyframe walk_keyframes[] = {
            // Keyframe 1: Lift FR and BL
            { .fr = 55,  .fl = 110, .br = 290, .bl = 240, .speed = 1600, .delay_ms = 250 },
            // Keyframe 2: Swing FR and BL forward
            { .fr = 95,  .fl = 80,  .br = 260, .bl = 285, .speed = 1050, .delay_ms = 250 },
            // Keyframe 3: Neutral
            { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 1300, .delay_ms = 80  },
            // Keyframe 4: Lift FL and BR
            { .fr = 110, .fl = 55,  .br = 240, .bl = 290, .speed = 1600, .delay_ms = 250 },
            // Keyframe 5: Swing FL and BR forward
            { .fr = 80,  .fl = 95,  .br = 285, .bl = 260, .speed = 950,  .delay_ms = 250 },
            // Keyframe 6: Neutral
            { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 700,  .delay_ms = 80  },
        };

        const int kf_count = sizeof(walk_keyframes) / sizeof(walk_keyframes[0]);

        for (int loop = 0; loop < loops; loop++) {
            for (int i = 0; i < kf_count; i++) {
                const Keyframe &kf = walk_keyframes[i];
                ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
                vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
            }
        }
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void DoubleFrontFlip() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform DoubleFrontFlip - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> DOUBLE FRONT FLIP <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 80.0f, .fl = 80.0f, .br = 260.0f, .bl = 260.0f, .speed = 2000, .delay_ms = 164},
            {.fr = 50.0f, .fl = 50.0f, .br = 275.0f, .bl = 275.0f, .speed = 2000, .delay_ms = 304},
            {.fr = 20.0f, .fl = 20.0f, .br = 230.0f, .bl = 230.0f, .speed = 2095, .delay_ms = 304},
            {.fr = 260.0f, .fl = 260.0f, .br = 80.0f, .bl = 80.0f, .speed = 2700, .delay_ms = 450},
            {.fr = 260.0f, .fl = 260.0f, .br = 80.0f, .bl = 80.0f, .speed = 2700, .delay_ms = 164},
            {.fr = 190.0f, .fl = 190.0f, .br = 30.0f, .bl = 30.0f, .speed = 2095, .delay_ms = 304},
            {.fr = 80.0f, .fl = 80.0f, .br = 260.0f, .bl = 260.0f, .speed = 4095, .delay_ms = 304},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void FrontFlip() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform FrontFlip - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> FRONT FLIP <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 80.0f, .fl = 80.0f, .br = 260.0f, .bl = 260.0f, .speed = 200, .delay_ms = 164},
            {.fr = 50.0f, .fl = 50.0f, .br = 275.0f, .bl = 275.0f, .speed = 2000, .delay_ms = 304},
            {.fr = 20.0f, .fl = 20.0f, .br = 230.0f, .bl = 230.0f, .speed = 2095, .delay_ms = 304},
            {.fr = 260.0f, .fl = 260.0f, .br = 80.0f, .bl = 80.0f, .speed = 2700, .delay_ms = 304},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        is_flipped_ = true;
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void Pounce() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform Pounce - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> POUNCE <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 90.0f, .fl = 90.0f, .br = 270.0f, .bl = 270.0f, .speed = 2400, .delay_ms = 207},
            {.fr = 80.0f, .fl = 80.0f, .br = 289.0f, .bl = 289.0f, .speed = 2400, .delay_ms = 313},
            {.fr = 130.0f, .fl = 130.0f, .br = 230.0f, .bl = 230.0f, .speed = 2400, .delay_ms = 200},
            {.fr = 90.0f, .fl = 90.0f, .br = 270.0f, .bl = 270.0f, .speed = 2400, .delay_ms = 260},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void BackFlip() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform BackFlip - dog is already flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> BACK FLIP <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 80.0f, .fl = 80.0f, .br = 260.0f, .bl = 260.0f, .speed = 4095, .delay_ms = 164},
            {.fr = 50.0f, .fl = 50.0f, .br = 275.0f, .bl = 275.0f, .speed = 4095, .delay_ms = 304},
            {.fr = 20.0f, .fl = 20.0f, .br = 230.0f, .bl = 230.0f, .speed = 4095, .delay_ms = 304},
            {.fr = 260.0f, .fl = 260.0f, .br = 80.0f, .bl = 80.0f, .speed = 4095, .delay_ms = 304},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        is_flipped_ = true;
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void BackFlipReverse() {
        if (!is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform BackFlipReverse - dog is not flipped. Use BackFlip first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> BACK FLIP REVERSE <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 260.0f, .fl = 260.0f, .br = 80.0f, .bl = 80.0f, .speed = 4095, .delay_ms = 164},
            {.fr = 310.0f, .fl = 310.0f, .br = 85.0f, .bl = 85.0f, .speed = 4095, .delay_ms = 304},
            {.fr = 280.0f, .fl = 280.0f, .br = 30.0f, .bl = 30.0f, .speed = 4095, .delay_ms = 304},
            {.fr = 80.0f, .fl = 80.0f, .br = 260.0f, .bl = 260.0f, .speed = 4095, .delay_ms = 304},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        is_flipped_ = false;
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void TurnLeftFast() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform TurnLeftFast - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> TURN LEFT (FAST) <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 65.0f, .fl = 65.0f, .br = 245.0f, .bl = 295.0f, .speed = 700, .delay_ms = 300},
            {.fr = 65.0f, .fl = 65.0f, .br = 295.0f, .bl = 245.0f, .speed = 700, .delay_ms = 300},
            {.fr = 65.0f, .fl = 115.0f, .br = 245.0f, .bl = 245.0f, .speed = 700, .delay_ms = 300},
            {.fr = 115.0f, .fl = 65.0f, .br = 245.0f, .bl = 245.0f, .speed = 700, .delay_ms = 300},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void TurnRightFast() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform TurnRightFast - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> TURN RIGHT (FAST) <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        const Keyframe keyframes[] = {
            {.fr = 65.0f, .fl = 65.0f, .br = 295.0f, .bl = 245.0f, .speed = 700, .delay_ms = 300},
            {.fr = 65.0f, .fl = 65.0f, .br = 245.0f, .bl = 295.0f, .speed = 700, .delay_ms = 300},
            {.fr = 115.0f, .fl = 65.0f, .br = 245.0f, .bl = 245.0f, .speed = 700, .delay_ms = 300},
            {.fr = 65.0f, .fl = 115.0f, .br = 245.0f, .bl = 245.0f, .speed = 700, .delay_ms = 300},
        };
        
        for (const auto& kf : keyframes) {
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }
        
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }
    
    void SitAndStand(uint32_t sit_time_ms) {
        ESP_LOGI(TAG, "Dog: >>> SIT & STAND <<< (sit_time_ms=%u)", (unsigned int)sit_time_ms);
        SetConversationActive(true);  // **BLOCK IMU**

        const Keyframe keyframes[] = {
            {.fr = 70.0f,  .fl = 70.0f,  .br = 325.0f, .bl = 325.0f, .speed = 1000, .delay_ms = (uint16_t)sit_time_ms},
            {.fr = 170.0f, .fl = 170.0f, .br = 325.0f, .bl = 325.0f, .speed = 1200, .delay_ms = 400},
            {.fr = 90.0f,  .fl = 90.0f,  .br = 270.0f, .bl = 270.0f, .speed = 1000,  .delay_ms = 300},
        };

        const int kf_count = sizeof(keyframes) / sizeof(keyframes[0]);
        for (int i = 0; i < kf_count; ++i) {
            const Keyframe &kf = keyframes[i];
            ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
            vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
        }

        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    bool IsInitialized() const { return initialized_; }
};

// ═══════════════════════════════════════════════════════
// AUDIO CODEC
// ═══════════════════════════════════════════════════════
class HeySantaCodec : public SantaAudioCodec  {
public:
    HeySantaCodec(i2c_master_bus_handle_t i2c_bus, int input_sample_rate, int output_sample_rate,
    gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din, uint8_t es7210_addr, bool input_reference)
        : SantaAudioCodec(i2c_bus, input_sample_rate, output_sample_rate,
                             mclk,  bclk,  ws,  dout,  din, es7210_addr, input_reference) {}

    virtual void EnableOutput(bool enable) override {
        SantaAudioCodec::EnableOutput(enable);
        SetConversationActive(enable);  // **BLOCK IMU WHEN SPEAKING**
    }
};

// ═══════════════════════════════════════════════════════
// HEYSANTABOARD
// ═══════════════════════════════════════════════════════
class HeySantaBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Button wake_button_;
    anim::EmojiWidget* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    ServoController servo_controller_;
    ImuController imu_controller_;
    GyroBalanceController gyro_balance_;
    TaskHandle_t imu_task_handle_ = nullptr;
    bool imu_task_running_ = false;
    uint32_t last_imu_time_ = 0;

    static void ImuTaskWrapper(void* param) {
        static_cast<HeySantaBoard*>(param)->ImuTask();
    }

    void ImuTask() {
        ESP_LOGI(TAG, "IMU monitoring task started");
        const TickType_t interval = pdMS_TO_TICKS(IMU_UPDATE_INTERVAL_MS);
        ImuData imu_data;
        
        while (imu_task_running_) {
            if (imu_controller_.Read(&imu_data)) {
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                float dt = (now - last_imu_time_) / 1000.0f;
                if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;
                last_imu_time_ = now;
                
                gyro_balance_.ProcessToggle(imu_data.gyro_x);
                
                // **ONLY RUN IF NOT TALKING**
                if (gyro_balance_.IsEnabled() && !IsConversationActive()) {
                    auto result = gyro_balance_.Calculate(imu_data.gyro_y, dt);
                    servo_controller_.ApplyBalance(result.front_offset, result.back_offset, result.speed);
                }
            }
            vTaskDelay(interval);
        }
        
        ESP_LOGI(TAG, "IMU monitoring task stopped");
        vTaskDelete(NULL);
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        mcp_server.AddTool("dog.walk", "Make the robot dog walk forward", PropertyList({Property("loops", kPropertyTypeInteger, 2, 1, 100)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog walk command received");
                int loops = properties["loops"].value<int>();
                servo_controller_.WalkForward(loops);
                return "Dog walked forward";
            });
        
        mcp_server.AddTool("dog.double_front_flip", "Make the robot dog do a double front flip (two complete flips, lands upright)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog double front flip command received");
                servo_controller_.DoubleFrontFlip();
                return "Dog did double front flip";
            });
        
        mcp_server.AddTool("dog.front_flip", "Make the robot dog do a front flip (ends upside down, requires BackFlipReverse to recover)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog front flip command received");
                servo_controller_.FrontFlip();
                return "Dog did front flip and is now upside down";
            });

        mcp_server.AddTool("dog.back_flip", "Make the robot dog do a back flip (ends upside down, requires BackFlipReverse to recover)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog back flip command received");
                servo_controller_.BackFlip();
                return "Dog did back flip and is now upside down";
            });

        mcp_server.AddTool("dog.back_flip_reverse", "Recover from upside-down position after a flip (only works when dog is flipped)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog back flip reverse command received");
                servo_controller_.BackFlipReverse();
                return "Dog recovered from upside-down position";
            });

        mcp_server.AddTool("dog.pounce", "Make the robot dog pounce", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog pounce command received");
                servo_controller_.Pounce();
                return "Dog pounced";
            });

        mcp_server.AddTool("dog.turn_left_fast", "Make the robot dog turn left quickly", PropertyList({Property("loops", kPropertyTypeInteger, 1, 1, 100)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog turn left fast command received");
                int loops = properties["loops"].value<int>();
                for (int i = 0; i < loops; ++i) {
                    servo_controller_.TurnLeftFast();
                }
                return "Dog turned left";
            });

        mcp_server.AddTool("dog.turn_right_fast", "Make the robot dog turn right quickly", PropertyList({Property("loops", kPropertyTypeInteger, 1, 1, 100)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog turn right fast command received");
                int loops = properties["loops"].value<int>();
                for (int i = 0; i < loops; ++i) {
                    servo_controller_.TurnRightFast();
                }
                return "Dog turned right";
            });

        mcp_server.AddTool("dog.reset", "Reset the robot dog to neutral position", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog reset command received");
                servo_controller_.MoveReset();
                return "Dog reset to neutral";
            });

        mcp_server.AddTool("dog.sit_and_stand", "Make the robot dog sit then stand (Sit&Stand)", PropertyList({Property("sit_time_ms", kPropertyTypeInteger, 1000, 0, 60000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog Sit&Stand command received");
                int sit_ms = properties["sit_time_ms"].value<int>();
                if (sit_ms < 0) sit_ms = 0;
                servo_controller_.SitAndStand((uint32_t)sit_ms);
                return "Dog performed Sit&Stand";
            });

        mcp_server.AddTool("dog.balance_enable", "Enable gyro balance mode", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Balance enable command received");
                gyro_balance_.Enable(true);
                return "Gyro balance mode enabled";
            });

        mcp_server.AddTool("dog.balance_disable", "Disable gyro balance mode", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Balance disable command received");
                gyro_balance_.Enable(false);
                servo_controller_.MoveReset();
                return "Gyro balance mode disabled";
            });

        mcp_server.AddTool("dog.balance_status", "Get current gyro balance mode status", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = gyro_balance_.IsEnabled();
                bool imu_ok = imu_controller_.IsInitialized();
                char status[128];
                snprintf(status, sizeof(status), "Balance: %s, IMU: %s (addr: 0x%02X)", 
                         enabled ? "ENABLED" : "DISABLED",
                         imu_ok ? "OK" : "NOT AVAILABLE",
                         imu_controller_.GetAddress());
                return std::string(status);
            });

        mcp_server.AddTool("self.audio.be_quiet", "Make Santa speak more quietly (50% volume)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "BeQuiet command - setting volume to 50%%");
                auto& board = Board::GetInstance();
                auto codec = board.GetAudioCodec();
                if (codec) {
                    codec->SetOutputVolume(50);
                    return "Santa will now speak more quietly (50% volume)";
                } else {
                    return "Audio codec not available";
                }
            });

        mcp_server.AddTool("self.system.quit", "Quit the application and restart Santa", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Quit command - sending exit to server");
                auto& app = Application::GetInstance();
                app.SendSystemCommand("exit");
                return "Ho ho ho! Santa is ending this session. See you soon!";
            });
    }

    void InitializeI2c() {
        ESP_LOGI(TAG, "=== Initializing SINGLE I2C bus on GPIO1/GPIO2 ===");
        
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = IMU_SDA_PIN,
            .scl_io_num = IMU_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        esp_err_t ret = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "✓ I2C bus created on GPIO1/GPIO2 (shared by IMU + codec + camera)");
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        wake_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.PlaySound(Lang::Sounds::P3_ACTIVE);  
            } else if (app.GetDeviceState() == kDeviceStateListening) {
                app.PlaySound(Lang::Sounds::P3_DEACTIVATE); 
            }
            
            app.ToggleChatState();
        });
    }

    void InitializeSt7735Display() {
        ESP_LOGI(TAG, "Initializing ST7735 display...");
        
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = nullptr;
        
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(panel_io, &panel_config, &panel));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
        
        ESP_LOGI(TAG, "✓ Display initialized");
        display_ = new anim::EmojiWidget(panel, panel_io);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_5;
        config.ledc_timer = LEDC_TIMER_1;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

public:
    HeySantaBoard() : boot_button_(BOOT_BUTTON_GPIO), wake_button_(WAKE_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7735Display();
        InitializeButtons();
        // InitializeCamera();
        servo_controller_.Initialize();

         vTaskDelay(pdMS_TO_TICKS(500));
        
        if (imu_controller_.Initialize(i2c_bus_)) {
            imu_task_running_ = true;
            last_imu_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
            xTaskCreate(ImuTaskWrapper, "imu_task", 4096, this, 5, &imu_task_handle_);
            ESP_LOGI(TAG, "✓ IMU task started");
        } else {
            ESP_LOGW(TAG, "✗ IMU initialization failed - balance mode disabled");
        }
        
        InitializeTools();
        GetBacklight()->RestoreBrightness();
        
        ESP_LOGI(TAG, "=== HeySanta board initialized! ===");
    }
    
    ~HeySantaBoard() {
        if (imu_task_running_) {
            imu_task_running_ = false;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        static HeySantaCodec audio_codec(i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, 
            AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(HeySantaBoard);