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
#include "esp_http_server.h"
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
#include <img_converters.h> 

#define TAG "HeySanta"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

// ═══════════════════════════════════════════════════════
// SERVO CONFIGURATION (using sts3032 driver)
// ═══════════════════════════════════════════════════════
#define SERVO_UART_NUM UART_NUM_1
#define SERVO_TX_PIN GPIO_NUM_10
#define SERVO_RX_PIN GPIO_NUM_11
#define SERVO_TXEN_PIN GPIO_NUM_3
#define SERVO_BAUD_RATE 1000000

// Servo IDs (same convention as dog_config)
#define SERVO_FR 1 // Front Right
#define SERVO_FL 2 // Front Left
#define SERVO_BR 3 // Back Right
#define SERVO_BL 4 // Back Left
#define SERVO_COUNT 4

// Stance angles (from left-side perspective)
#define STANCE_FRONT 90.0f // Front legs neutral
#define STANCE_BACK 270.0f // Back legs neutral
#define SWING_AMPLITUDE 25.0f // Max deviation from stance

// Speed presets
#define SPEED_SLOW 300
#define SPEED_MEDIUM 700
#define SPEED_FAST 1500
#define SPEED_VERY_FAST 3000
#define SPEED_MAX 4095

// Dynamic speed for stance (prevents shaking)
#define STANCE_SPEED_MIN 20
#define STANCE_SPEED_MAX 1500
#define STANCE_SPEED_THRESHOLD 40.0f
#define STANCE_SPEED_CURVE 1.4f

// Angle reversal macros for right-side servos
#define REVERSE_ANGLE(angle) (360.0f - (angle))
#define IS_RIGHT_SIDE(id) ((id) == SERVO_FR || (id) == SERVO_BR)
#define IS_FRONT_LEG(id) ((id) == SERVO_FR || (id) == SERVO_FL)

// ═══════════════════════════════════════════════════════
// QMI8658A IMU CONFIG
// ═══════════════════════════════════════════════════════
#define IMU_SDA_PIN GPIO_NUM_1
#define IMU_SCL_PIN GPIO_NUM_2
#define IMU_I2C_FREQ_HZ 400000

#define IMU_I2C_ADDR_PRIMARY 0x6A
#define IMU_I2C_ADDR_SECONDARY 0x6B

#define IMU_CHIP_ID 0x05

// QMI8658A Registers
#define QMI_REG_CHIP_ID 0x00
#define QMI_REG_REVISION 0x01
#define QMI_REG_CTRL1 0x02
#define QMI_REG_CTRL2 0x03
#define QMI_REG_CTRL3 0x04
#define QMI_REG_CTRL4 0x05
#define QMI_REG_CTRL5 0x06
#define QMI_REG_CTRL6 0x07
#define QMI_REG_CTRL7 0x08
#define QMI_REG_CTRL8 0x09
#define QMI_REG_CTRL9 0x0A
#define QMI_REG_STATUS0 0x2E
#define QMI_REG_AX_L 0x35

// ═══════════════════════════════════════════════════════
// GYRO BALANCE CONFIG (Mahony Filter Based - IMPROVED)
// ═══════════════════════════════════════════════════════
#define GYRO_BALANCE_ENABLED_DEFAULT false
#define GYRO_BALANCE_MAX_CORRECTION 65.0f
#define GYRO_BALANCE_DEADZONE 0.7f
#define GYRO_BALANCE_GAIN 2.5f
#define GYRO_BALANCE_UPDATE_INTERVAL_MS 10
#define IMU_UPDATE_INTERVAL_MS 10

// Mahony filter parameters (optimized from gyro_balance.c)
#define MAHONY_KP 1.5f      // Reduced - less accelerometer noise
#define MAHONY_KI 0.003f    // Reduced - slower bias correction
#define ACCEL_MAGNITUDE_MIN 0.9f  // Tighter window = less noise
#define ACCEL_MAGNITUDE_MAX 1.1f

// Anti-jitter configuration (from gyro_balance.c)
#define CHASE_SPEED 0.4f           // Slower = smoother
#define CHASE_DEADBAND 0.7f        // Larger = less micro-movements
#define PITCH_SMOOTHING 0.4f       // Low-pass on angle itself (0.0=raw, 1.0=frozen)
#define SERVO_UPDATE_THRESHOLD 0.4f // degrees - ignore tiny changes
#define HYSTERESIS_THRESHOLD 0.2f  // degrees - keep moving once started

// Dynamic speed for balance
#define GYRO_BALANCE_SPEED_MIN 150
#define GYRO_BALANCE_SPEED_MAX 2000
#define GYRO_BALANCE_SPEED_THRESHOLD 10.0f
#define GYRO_BALANCE_SPEED_CURVE 1.2f

// Toggle gesture configuration
#define GYRO_BALANCE_TOGGLE_THRESHOLD 150.0f
#define GYRO_BALANCE_TOGGLE_WINDOW_MS 1000
#define GYRO_BALANCE_TOGGLE_COOLDOWN_MS 1500

// ═══════════════════════════════════════════════════════
// IMU PUSH DETECTION CONFIG
// ═══════════════════════════════════════════════════════
#define PUSH_REACTION_ENABLED_DEFAULT false
#define REACTION_DELTA_THRESHOLD 3.0f
#define REACTION_MIN_ACCEL 1.0f
#define REACTION_COOLDOWN_MS 1600
#define REACTION_WALK_CYCLES 1

// ═══════════════════════════════════════════════════════
// MOVEMENT TRANSITION CONFIG (NEW - fixes jitter)
// ═══════════════════════════════════════════════════════
#define PRE_MOVEMENT_SETTLE_MS 50    // Delay before starting movement
#define POST_MOVEMENT_SETTLE_MS 300  // Time to let servos settle after movement
#define MOVEMENT_END_WAIT_MS 500     // Wait for servos to reach final position

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
// GLOBAL CONVERSATION LOCK (IMPROVED - fixes jitter)
// ═══════════════════════════════════════════════════════
static volatile bool g_conversation_active = false;
static volatile uint32_t g_conversation_end_time = 0;


// Static handler - outside the class
struct SnapCtx {
    httpd_req_t* req;
    size_t total;
    bool ok;
};

static esp_err_t camera_snapshot_handler(httpd_req_t* req) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "esp_camera_fb_get() returned NULL");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Frame grab failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    SnapCtx ctx = { req, 0, true };

    bool encode_ok = frame2jpg_cb(fb, 80,
        [](void* arg, size_t index, const void* data, size_t len) -> unsigned int {
            auto* c = (SnapCtx*)arg;
            if (httpd_resp_send_chunk(c->req, (const char*)data, len) != ESP_OK) {
                c->ok = false;
                return 0;
            }
            c->total += len;
            return len;
        }, &ctx);

    esp_camera_fb_return(fb);
    httpd_resp_send_chunk(req, nullptr, 0);

    if (!encode_ok || !ctx.ok) {
        ESP_LOGE(TAG, "JPEG encode/send failed (encode_ok=%d send_ok=%d)", encode_ok, ctx.ok);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "📷 Snapshot: %zu bytes sent", ctx.total);
    return ESP_OK;
}
static esp_err_t camera_stream_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    ESP_LOGI(TAG, "📹 Stream client connected");

    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Stream: frame grab failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Convert RGB565 → JPEG
        uint8_t* jpg_buf = nullptr;
        size_t   jpg_len = 0;
        bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        esp_camera_fb_return(fb);

        if (!ok || !jpg_buf) {
            ESP_LOGW(TAG, "Stream: JPEG conversion failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // -- boundary + headers --
        char part_hdr[128];
        int hlen = snprintf(part_hdr, sizeof(part_hdr),
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %zu\r\n\r\n", jpg_len);

        esp_err_t res = httpd_resp_send_chunk(req, part_hdr, hlen);

        // -- JPEG payload --
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);
        }

        // -- trailing CRLF --
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, "\r\n", 2);
        }

        free(jpg_buf);

        if (res != ESP_OK) {
            ESP_LOGI(TAG, "📹 Stream client disconnected");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(33)); // ~30 fps cap
    }

    return ESP_OK;
}
void SetConversationActive(bool active) {
    if (active) {
        g_conversation_active = true;
        ESP_LOGI(TAG, "🔒 CONVERSATION ACTIVE - IMU processing paused");
    } else {
        // Record when conversation ended for settle time
        g_conversation_end_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        g_conversation_active = false;
        ESP_LOGI(TAG, "🔓 CONVERSATION ENDED - IMU will resume after settle time");
    }
}

bool IsConversationActive() {
    if (g_conversation_active) {
        return true;
    }
    // Also return true during settle period after movement
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - g_conversation_end_time) < POST_MOVEMENT_SETTLE_MS) {
        return true;
    }
    return false;
}

bool IsConversationActiveRaw() {
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
// GYRO BALANCE CONTROLLER (Improved from gyro_balance.c)
// ═══════════════════════════════════════════════════════
class GyroBalanceController {
private:
    bool enabled_ = GYRO_BALANCE_ENABLED_DEFAULT;
    bool initialized_ = false;
    uint32_t last_balance_time_ = 0;
    uint32_t last_update_time_ = 0;

    // Mahony filter state - quaternion representation
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;

    // Integral error for Mahony filter (gyro bias estimation)
    float integralFBx_ = 0.0f, integralFBy_ = 0.0f, integralFBz_ = 0.0f;

    // Estimated Euler angles (in degrees)
    float pitch_angle_raw_ = 0.0f;  // Direct from quaternion
    float pitch_angle_ = 0.0f;      // Smoothed version we actually use
    float roll_angle_ = 0.0f;

    // Smooth chase state (from gyro_balance.c)
    float target_correction_ = 0.0f;
    float current_correction_ = 0.0f;
    float last_sent_correction_ = 0.0f;  // What we last sent to servos
    bool is_moving_ = false;  // Hysteresis state

    // Toggle gesture state
    enum ToggleState { TOGGLE_IDLE, TOGGLE_FIRST_ROTATION, TOGGLE_WAITING_REVERSE };
    ToggleState toggle_state_ = TOGGLE_IDLE;
    int8_t first_rotation_dir_ = 0;
    uint32_t toggle_gesture_start_ = 0;
    uint32_t last_toggle_time_ = 0;

    // Fast inverse square root
    static float InvSqrt(float x) {
        union {
            float f;
            uint32_t i;
        } conv;
        
        float halfx = 0.5f * x;
        conv.f = x;
        conv.i = 0x5f3759df - (conv.i >> 1);
        conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
        conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
        return conv.f;
    }

    // Mahony AHRS filter update
    void MahonyUpdate(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float dt) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;
        
        // Convert gyro from deg/s to rad/s
        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;
        
        // Compute feedback only if accelerometer measurement valid
        float accel_magnitude = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;
        
        if (accel_magnitude > ACCEL_MAGNITUDE_MIN && accel_magnitude < ACCEL_MAGNITUDE_MAX) {
            // Normalise accelerometer measurement
            recipNorm = InvSqrt(ax*ax + ay*ay + az*az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            
            // Estimated direction of gravity
            halfvx = q1_*q3_ - q0_*q2_;
            halfvy = q0_*q1_ + q2_*q3_;
            halfvz = q0_*q0_ - 0.5f + q3_*q3_;
            
            // Error is cross product between estimated and measured direction of gravity
            halfex = (ay*halfvz - az*halfvy);
            halfey = (az*halfvx - ax*halfvz);
            halfez = (ax*halfvy - ay*halfvx);
            
            // Compute and apply integral feedback if enabled
            if (MAHONY_KI > 0.0f) {
                integralFBx_ += MAHONY_KI * halfex * dt;
                integralFBy_ += MAHONY_KI * halfey * dt;
                integralFBz_ += MAHONY_KI * halfez * dt;
                
                // Anti-windup clamp
                const float MAX_INTEGRAL = 0.5f;
                integralFBx_ = fmaxf(-MAX_INTEGRAL, fminf(MAX_INTEGRAL, integralFBx_));
                integralFBy_ = fmaxf(-MAX_INTEGRAL, fminf(MAX_INTEGRAL, integralFBy_));
                integralFBz_ = fmaxf(-MAX_INTEGRAL, fminf(MAX_INTEGRAL, integralFBz_));
                
                gx += integralFBx_;
                gy += integralFBy_;
                gz += integralFBz_;
            }
            
            // Apply proportional feedback
            gx += MAHONY_KP * halfex;
            gy += MAHONY_KP * halfey;
            gz += MAHONY_KP * halfez;
        }
        
        // Integrate rate of change of quaternion
        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        qa = q0_;
        qb = q1_;
        qc = q2_;
        q0_ += (-qb*gx - qc*gy - q3_*gz);
        q1_ += (qa*gx + qc*gz - q3_*gy);
        q2_ += (qa*gy - qb*gz + q3_*gx);
        q3_ += (qa*gz + qb*gy - qc*gx);
        
        // Normalise quaternion
        recipNorm = InvSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        q0_ *= recipNorm;
        q1_ *= recipNorm;
        q2_ *= recipNorm;
        q3_ *= recipNorm;
    }

    // Convert quaternion to Euler angles
    void QuaternionToEuler() {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q0_*q1_ + q2_*q3_);
        float cosr_cosp = 1.0f - 2.0f * (q1_*q1_ + q2_*q2_);
        roll_angle_ = atan2f(sinr_cosp, cosr_cosp) * 57.2958f;
        
        // Pitch (y-axis rotation) - RAW value
        float sinp = 2.0f * (q0_*q2_ - q3_*q1_);
        if (fabsf(sinp) >= 1.0f) {
            pitch_angle_raw_ = copysignf(90.0f, sinp);
        } else {
            pitch_angle_raw_ = asinf(sinp) * 57.2958f;
        }
        
        // ═══════════════════════════════════════════════════
        // LOW-PASS FILTER on pitch angle itself (from gyro_balance.c)
        // ═══════════════════════════════════════════════════
        pitch_angle_ = pitch_angle_ * PITCH_SMOOTHING + pitch_angle_raw_ * (1.0f - PITCH_SMOOTHING);
    }

public:
    void Initialize() {
        ESP_LOGI(TAG, "Gyro balance system initialized (Mahony filter)");
        ESP_LOGI(TAG, "  Default: %s, Max correction: %.1f°, Gain: %.2f",
                 GYRO_BALANCE_ENABLED_DEFAULT ? "ENABLED" : "DISABLED",
                 GYRO_BALANCE_MAX_CORRECTION, GYRO_BALANCE_GAIN);
        ESP_LOGI(TAG, "  Mahony Kp: %.2f, Ki: %.4f", MAHONY_KP, MAHONY_KI);
        ESP_LOGI(TAG, "  Chase: %.2f, Deadband: %.1f°, Pitch smooth: %.2f",
                 CHASE_SPEED, CHASE_DEADBAND, PITCH_SMOOTHING);
        ESP_LOGI(TAG, "  Servo threshold: %.1f°, Hysteresis: %.2f°",
                 SERVO_UPDATE_THRESHOLD, HYSTERESIS_THRESHOLD);
        
        Reset();
        initialized_ = true;
    }

    // Detect toggle gesture (rotate on X axis back and forth)
    bool ProcessToggle(float gyro_x) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check cooldown
        if ((now - last_toggle_time_) < GYRO_BALANCE_TOGGLE_COOLDOWN_MS) {
            return false;
        }
        
        bool above_threshold = fabsf(gyro_x) >= GYRO_BALANCE_TOGGLE_THRESHOLD;
        int8_t current_dir = (gyro_x > 0) ? 1 : -1;
        
        switch (toggle_state_) {
            case TOGGLE_IDLE:
                if (above_threshold) {
                    toggle_state_ = TOGGLE_FIRST_ROTATION;
                    first_rotation_dir_ = current_dir;
                    toggle_gesture_start_ = now;
                    ESP_LOGD(TAG, "Toggle gesture started (dir: %+d)", first_rotation_dir_);
                }
                break;
                
            case TOGGLE_FIRST_ROTATION:
                if ((now - toggle_gesture_start_) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                    toggle_state_ = TOGGLE_IDLE;
                    ESP_LOGD(TAG, "Toggle gesture timed out");
                    break;
                }
                if (!above_threshold) {
                    toggle_state_ = TOGGLE_WAITING_REVERSE;
                }
                break;
                
            case TOGGLE_WAITING_REVERSE:
                if ((now - toggle_gesture_start_) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                    toggle_state_ = TOGGLE_IDLE;
                    ESP_LOGD(TAG, "Toggle gesture timed out waiting for reverse");
                    break;
                }
                if (above_threshold) {
                    if (current_dir != first_rotation_dir_) {
                        last_toggle_time_ = now;
                        toggle_state_ = TOGGLE_IDLE;
                        ESP_LOGI(TAG, "Toggle gesture complete!");
                        return true;
                    }
                    toggle_state_ = TOGGLE_IDLE;
                }
                break;
        }
        
        return false;
    }

    struct BalanceResult {
        float front_offset;
        float back_offset;
        uint16_t speed;
        bool should_update;
    };

    // Process IMU data and calculate balance corrections (IMPROVED from gyro_balance.c)
    BalanceResult Process(const ImuData& data, float dt) {
        BalanceResult result = {0.0f, 0.0f, GYRO_BALANCE_SPEED_MIN, false};
        
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Clamp dt to reasonable range
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.02f;
        }
        
        // Always update the Mahony filter for accurate angle tracking
        MahonyUpdate(data.gyro_x, data.gyro_y, data.gyro_z,
                    data.accel_x, data.accel_y, data.accel_z,
                    dt);
        QuaternionToEuler();
        
        if (!enabled_) {
            return result;
        }
        
        // Rate limit balance updates
        if ((now - last_balance_time_) < GYRO_BALANCE_UPDATE_INTERVAL_MS) {
            return result;
        }
        last_balance_time_ = now;
        
        // ═══════════════════════════════════════════════════
        // STEP 1: Calculate target correction
        // ═══════════════════════════════════════════════════
        float effective_pitch = pitch_angle_;
        if (fabsf(effective_pitch) < GYRO_BALANCE_DEADZONE) {
            effective_pitch = 0.0f;
        }
        
        target_correction_ = effective_pitch * GYRO_BALANCE_GAIN;
        
        if (target_correction_ > GYRO_BALANCE_MAX_CORRECTION) {
            target_correction_ = GYRO_BALANCE_MAX_CORRECTION;
        } else if (target_correction_ < -GYRO_BALANCE_MAX_CORRECTION) {
            target_correction_ = -GYRO_BALANCE_MAX_CORRECTION;
        }
        
        // ═══════════════════════════════════════════════════
        // STEP 2: Chase with hysteresis
        // ═══════════════════════════════════════════════════
        float diff = target_correction_ - current_correction_;
        float abs_diff = fabsf(diff);
        
        // Hysteresis: use different thresholds for starting vs stopping movement
        float threshold = is_moving_ ? HYSTERESIS_THRESHOLD : CHASE_DEADBAND;
        
        if (abs_diff > threshold) {
            current_correction_ += diff * CHASE_SPEED;
            is_moving_ = true;
        } else {
            is_moving_ = false;
        }
        
        // ═══════════════════════════════════════════════════
        // STEP 3: Only send to servos if change is significant
        // ═══════════════════════════════════════════════════
        float servo_diff = fabsf(current_correction_ - last_sent_correction_);
        
        if (servo_diff < SERVO_UPDATE_THRESHOLD) {
            // Skip this update - change too small
            return result;
        }
        
        last_sent_correction_ = current_correction_;
        
        // ═══════════════════════════════════════════════════
        // STEP 4: Calculate final output
        // ═══════════════════════════════════════════════════
        result.front_offset = current_correction_;
        result.back_offset = current_correction_;
        
        // Use a fixed moderate speed for smoother movement
        // Dynamic speed can cause jitter when oscillating
        result.speed = (GYRO_BALANCE_SPEED_MIN + GYRO_BALANCE_SPEED_MAX) / 2;
        
        result.should_update = true;
        
        // Debug logging (less frequent)
        static int log_counter = 0;
        if (++log_counter >= 25) {
            ESP_LOGD(TAG, "Pitch: %+5.1f° (raw:%+5.1f°) Target: %+5.1f° Current: %+5.1f°", 
                     pitch_angle_, pitch_angle_raw_, target_correction_, current_correction_);
            log_counter = 0;
        }
        
        return result;
    }

    void Enable(bool enable) { 
        if (enable && !enabled_) {
            Reset();
            ESP_LOGI(TAG, "⚖️ Gyro balance ENABLED");
        } else if (!enable && enabled_) {
            ESP_LOGI(TAG, "⚖️ Gyro balance DISABLED - returning to stance");
        }
        enabled_ = enable;
    }

    bool IsEnabled() const { return enabled_; }

    void Reset() { 
        // Reset quaternion to identity
        q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
        
        // Reset integral terms
        integralFBx_ = 0.0f; integralFBy_ = 0.0f; integralFBz_ = 0.0f;
        
        // Reset angles
        pitch_angle_raw_ = 0.0f;
        pitch_angle_ = 0.0f;
        roll_angle_ = 0.0f;
        
        // Reset output state
        target_correction_ = 0.0f;
        current_correction_ = 0.0f;
        last_sent_correction_ = 0.0f;
        is_moving_ = false;
        
        // Reset timing
        last_balance_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
        last_update_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Reset toggle state
        toggle_state_ = TOGGLE_IDLE;
        last_toggle_time_ = 0;
    }

    // NEW: Sync to stance position after movement completes
    void SyncToStance() {
        // Reset the chase state to current position (no correction needed)
        target_correction_ = 0.0f;
        current_correction_ = 0.0f;
        last_sent_correction_ = 0.0f;
        is_moving_ = false;
        
        ESP_LOGD(TAG, "Balance controller synced to stance position");
    }

    float GetPitch() const { return pitch_angle_; }
    float GetRoll() const { return roll_angle_; }
};

// ═══════════════════════════════════════════════════════
// IMU PUSH REACTION CONTROLLER
// ═══════════════════════════════════════════════════════
class PushReactionController {
private:
    bool enabled_ = PUSH_REACTION_ENABLED_DEFAULT;
    bool has_prev_reading_ = false;
    float prev_accel_x_ = 0.0f;
    uint32_t last_reaction_time_ = 0;

public:
    enum PushDirection {
        PUSH_NONE = 0,
        PUSH_FRONT = 1,
        PUSH_BACK = -1
    };

    void Enable(bool enable) {
        if (enabled_ != enable) {
            enabled_ = enable;
            ESP_LOGI(TAG, "👋 Push reaction %s", enable ? "ENABLED" : "DISABLED");
            if (!enable) {
                Reset();
            }
        }
    }

    bool IsEnabled() const { return enabled_; }

    void Reset() {
        has_prev_reading_ = false;
        prev_accel_x_ = 0.0f;
    }

    bool IsCooldownExpired() const {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        return (now - last_reaction_time_) >= REACTION_COOLDOWN_MS;
    }

    void UpdateReactionTime() {
        last_reaction_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    PushDirection ProcessAccel(float accel_x) {
        if (!enabled_) {
            return PUSH_NONE;
        }
        
        if (!has_prev_reading_) {
            prev_accel_x_ = accel_x;
            has_prev_reading_ = true;
            return PUSH_NONE;
        }
        
        float delta = accel_x - prev_accel_x_;
        prev_accel_x_ = accel_x;
        
        // Check cooldown
        if (!IsCooldownExpired()) {
            return PUSH_NONE;
        }
        
        // Front push detection (positive delta, positive acceleration)
        if (delta >= REACTION_DELTA_THRESHOLD && accel_x >= REACTION_MIN_ACCEL) {
            ESP_LOGI(TAG, "🏃 FRONT PUSH detected! (delta: +%.2f, accel: %.2f)",
                     delta, accel_x);
            return PUSH_FRONT;
        }
        
        // Back push detection (negative delta, negative acceleration)
        if (delta <= -REACTION_DELTA_THRESHOLD && accel_x <= -REACTION_MIN_ACCEL) {
            ESP_LOGI(TAG, "⬅️ BACK PUSH detected! (delta: %.2f, accel: %.2f)",
                     delta, accel_x);
            return PUSH_BACK;
        }
        
        return PUSH_NONE;
    }
};

// ═══════════════════════════════════════════════════════
// SERVO CONTROLLER
// ═══════════════════════════════════════════════════════
class ServoController {
private:
    uint16_t default_speed_ = SPEED_MAX;
    bool is_flipped_ = false;
    bool initialized_ = false;
    volatile bool walk_in_progress_ = false;

    static float ApplyReversal(uint8_t servo_id, float angle) {
        if (IS_RIGHT_SIDE(servo_id)) {
            return REVERSE_ANGLE(angle);
        }
        return angle;
    }

    static float GetBaseStance(uint8_t servo_id) {
        if (IS_FRONT_LEG(servo_id)) {
            return STANCE_FRONT;
        } else {
            return STANCE_BACK;
        }
    }

    static uint16_t CalculateDynamicSpeed(float angle_delta) {
        float abs_delta = fabsf(angle_delta);
        float speed_ratio = abs_delta / STANCE_SPEED_THRESHOLD;
        
        if (speed_ratio > 1.0f) {
            speed_ratio = 1.0f;
        }
        
        speed_ratio = powf(speed_ratio, STANCE_SPEED_CURVE);
        
        return (uint16_t)(STANCE_SPEED_MIN + 
                          speed_ratio * (STANCE_SPEED_MAX - STANCE_SPEED_MIN));
    }

    void ServoMove(uint8_t servo_id, float angle, uint16_t speed) {
        float actual_angle = ApplyReversal(servo_id, angle);
        sts_servo_set_angle(servo_id, actual_angle, speed);
    }

    void ServoMoveAll(float angle_fr, float angle_fl, float angle_br, float angle_bl, uint16_t speed) {
        // ═══════════════════════════════════════════════════
        // CALIBRATION OFFSETS (adjust these to level the robot)
        // Positive = leg moves backward/down, Negative = leg moves forward/up
        // ═══════════════════════════════════════════════════
        angle_fr += 0.0f;   // Front Right offset
        angle_fl += 0.0f;   // Front Left offset
        angle_br += 0.0f;   // Back Right offset
        angle_bl += 0.0f;   // Back Left offset
        
        float actual_fr = ApplyReversal(SERVO_FR, angle_fr);
        float actual_br = ApplyReversal(SERVO_BR, angle_br);
        float actual_fl = angle_fl;
        float actual_bl = angle_bl;
        
        sts_servo_set_angle(SERVO_FR, actual_fr, speed);
        sts_servo_set_angle(SERVO_FL, actual_fl, speed);
        sts_servo_set_angle(SERVO_BR, actual_br, speed);
        sts_servo_set_angle(SERVO_BL, actual_bl, speed);
    }

    void PrintServoAngles() {
        float angle_fr, angle_fl, angle_br, angle_bl;
        
        sts_servo_get_angle(SERVO_FR, &angle_fr);
        sts_servo_get_angle(SERVO_FL, &angle_fl);
        sts_servo_get_angle(SERVO_BR, &angle_br);
        sts_servo_get_angle(SERVO_BL, &angle_bl);
        
        ESP_LOGI(TAG, "Servo angles - FR: %.1f°  FL: %.1f°  BR: %.1f°  BL: %.1f°", 
            angle_fr, angle_fl, angle_br, angle_bl);
    }

    // Helper to prepare for movement - waits for any pending servo commands
    void PrepareForMovement() {
        vTaskDelay(pdMS_TO_TICKS(PRE_MOVEMENT_SETTLE_MS));
    }

    // Helper to finalize movement - waits for servos to reach position
    void FinalizeMovement() {
        vTaskDelay(pdMS_TO_TICKS(MOVEMENT_END_WAIT_MS));
    }

public:
    void Initialize() {
        ESP_LOGI(TAG, "Initializing servo controller using sts3032 driver...");
        
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
        
        ESP_LOGI(TAG, "Checking servos...");
        int found = sts_servo_scan_bus(1, SERVO_COUNT);
        if (found < SERVO_COUNT) {
            ESP_LOGW(TAG, "Only %d of %d servos responding", found, SERVO_COUNT);
        }
        
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
        
        ServoMoveAll(
            STANCE_FRONT,
            STANCE_FRONT,
            STANCE_BACK,
            STANCE_BACK,
            default_speed_
        );
        
        vTaskDelay(pdMS_TO_TICKS(500));
        PrintServoAngles();
    }

    void MoveReset() {
        ESP_LOGI(TAG, "Dog: Resetting to stance (flipped: %s)", is_flipped_ ? "yes" : "no");
        
        if (is_flipped_) {
            ServoMoveAll(
                STANCE_BACK,
                STANCE_BACK,
                STANCE_FRONT,
                STANCE_FRONT,
                default_speed_
            );
        } else {
            ServoMoveAll(
                STANCE_FRONT,
                STANCE_FRONT,
                STANCE_BACK,
                STANCE_BACK,
                default_speed_
            );
        }
        
        // Wait for servos to actually reach position
        FinalizeMovement();
    }

    void GotoStanceSmooth() {
        ESP_LOGI(TAG, "Dog: Smooth return to stance");
        
        ServoMoveAll(
            STANCE_FRONT,
            STANCE_FRONT,
            STANCE_BACK,
            STANCE_BACK,
            (GYRO_BALANCE_SPEED_MIN + GYRO_BALANCE_SPEED_MAX) / 2
        );
    }

    void ApplyBalance(float front_offset, float back_offset, uint16_t speed) {
        if (IsConversationActive()) return;
        if (walk_in_progress_) return;
        
        float angle_fl = STANCE_FRONT + front_offset;
        float angle_fr = STANCE_FRONT + front_offset;
        float angle_bl = STANCE_BACK + back_offset;
        float angle_br = STANCE_BACK + back_offset;
        
        ServoMoveAll(angle_fr, angle_fl, angle_br, angle_bl, speed);
    }

    bool IsWalkInProgress() const { return walk_in_progress_; }

    void WalkForward(int loops = 3) {
        ESP_LOGI(TAG, "Dog: >>> WALKING FORWARD (%d loops) <<<", loops);
        SetConversationActive(true);
        walk_in_progress_ = true;
        
        PrepareForMovement();
        
        Keyframe walk_keyframes[] = {
            { .fr = 55,  .fl = 110, .br = 290, .bl = 240, .speed = 1600, .delay_ms = 250 },
            { .fr = 95,  .fl = 80,  .br = 260, .bl = 285, .speed = 1050, .delay_ms = 250 },
            { .fr = 90,  .fl = 90,  .br = 270, .bl = 270, .speed = 1300, .delay_ms = 80  },
            { .fr = 110, .fl = 55,  .br = 240, .bl = 290, .speed = 1600, .delay_ms = 250 },
            { .fr = 80,  .fl = 95,  .br = 285, .bl = 260, .speed = 950,  .delay_ms = 250 },
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
        
        walk_in_progress_ = false;
        MoveReset();
        SetConversationActive(false);
    }

    void DoubleFrontFlip() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform DoubleFrontFlip - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> DOUBLE FRONT FLIP <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void FrontFlip() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform FrontFlip - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> FRONT FLIP <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void Pounce() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform Pounce - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> POUNCE <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void PounceBack(int loops = 1) {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform PounceBack - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> POUNCE BACK (%d loops) <<<", loops);
        SetConversationActive(true);
        PrepareForMovement();
        
        const Keyframe keyframes[] = {
            {.fr = 90.0f, .fl = 90.0f, .br = 270.0f, .bl = 270.0f, .speed = 20000, .delay_ms = 207},
            {.fr = 68.0f, .fl = 68.0f, .br = 280.0f, .bl = 280.0f, .speed = 2000, .delay_ms = 313},
            {.fr = 90.0f, .fl = 90.0f, .br = 180.0f, .bl = 180.0f, .speed = 3300, .delay_ms = 150},
            {.fr = 90.0f, .fl = 90.0f, .br = 270.0f, .bl = 270.0f, .speed = 2000, .delay_ms = 300},
            {.fr = 90.0f, .fl = 90.0f, .br = 270.0f, .bl = 270.0f, .speed = 20000, .delay_ms = 207},
            {.fr = 68.0f, .fl = 68.0f, .br = 280.0f, .bl = 280.0f, .speed = 2000, .delay_ms = 313},
            {.fr = 90.0f, .fl = 90.0f, .br = 180.0f, .bl = 180.0f, .speed = 3300, .delay_ms = 150},
            {.fr = 90.0f, .fl = 90.0f, .br = 270.0f, .bl = 270.0f, .speed = 2000, .delay_ms = 300},
        };
        
        const int kf_count = sizeof(keyframes) / sizeof(keyframes[0]);

        for (int loop = 0; loop < loops; loop++) {
            for (int i = 0; i < kf_count; i++) {
                const Keyframe &kf = keyframes[i];
                ServoMoveAll(kf.fr, kf.fl, kf.br, kf.bl, kf.speed);
                vTaskDelay(pdMS_TO_TICKS(kf.delay_ms));
            }
        }
        
        MoveReset();
        SetConversationActive(false);
    }

    void BackFlip() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform BackFlip - dog is already flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> BACK FLIP <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void BackFlipReverse() {
        if (!is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform BackFlipReverse - dog is not flipped. Use BackFlip first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> BACK FLIP REVERSE <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void TurnLeftFast() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform TurnLeftFast - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> TURN LEFT (FAST) <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void TurnRightFast() {
        if (is_flipped_) {
            ESP_LOGW(TAG, "Cannot perform TurnRightFast - dog is flipped. Use BackFlipReverse first.");
            return;
        }
        
        ESP_LOGI(TAG, "Dog: >>> TURN RIGHT (FAST) <<<");
        SetConversationActive(true);
        PrepareForMovement();
        
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
        SetConversationActive(false);
    }

    void SitAndStand(uint32_t sit_time_ms) {
        ESP_LOGI(TAG, "Dog: >>> SIT & STAND <<< (sit_time_ms=%u)", (unsigned int)sit_time_ms);
        SetConversationActive(true);
        PrepareForMovement();

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
        SetConversationActive(false);
    }

    bool IsInitialized() const { return initialized_; }
    bool IsFlipped() const { return is_flipped_; }
};

// ═══════════════════════════════════════════════════════
// AUDIO CODEC
// ═══════════════════════════════════════════════════════
class HeySantaCodec : public SantaAudioCodec {
public:
    HeySantaCodec(i2c_master_bus_handle_t i2c_bus, int input_sample_rate, int output_sample_rate,
                  gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din, uint8_t es7210_addr, bool input_reference)
        : SantaAudioCodec(i2c_bus, input_sample_rate, output_sample_rate,
                         mclk, bclk, ws, dout, din, es7210_addr, input_reference) {}

    virtual void EnableOutput(bool enable) override {
        SantaAudioCodec::EnableOutput(enable);
        SetConversationActive(enable);
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
    PushReactionController push_reaction_;
    TaskHandle_t imu_task_handle_ = nullptr;
    bool imu_task_running_ = false;
    uint32_t last_imu_time_ = 0;

    static void ImuTaskWrapper(void* param) {
        static_cast<HeySantaBoard*>(param)->ImuTask();
    }
    void ImuTask() {
        ESP_LOGI(TAG, "IMU monitoring task started");
        ESP_LOGI(TAG, "  Push reaction: %s (delta=%.1f m/s², min=%.1f m/s², cooldown=%dms)",
                 PUSH_REACTION_ENABLED_DEFAULT ? "ENABLED" : "DISABLED",
                 REACTION_DELTA_THRESHOLD, REACTION_MIN_ACCEL, REACTION_COOLDOWN_MS);
        ESP_LOGI(TAG, "  Gyro balance: %s (Mahony filter + anti-jitter)",
                 GYRO_BALANCE_ENABLED_DEFAULT ? "ENABLED" : "DISABLED");
        ESP_LOGI(TAG, "  Movement transition: pre=%dms, post=%dms, end=%dms",
                 PRE_MOVEMENT_SETTLE_MS, POST_MOVEMENT_SETTLE_MS, MOVEMENT_END_WAIT_MS);
        
        const TickType_t interval = pdMS_TO_TICKS(IMU_UPDATE_INTERVAL_MS);
        ImuData imu_data;
        bool was_paused = false;
        
        while (imu_task_running_) {
            if (imu_controller_.Read(&imu_data)) {
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                float dt = (now - last_imu_time_) / 1000.0f;
                if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;
                last_imu_time_ = now;
                
                bool is_paused = IsConversationActive() || servo_controller_.IsWalkInProgress();
                
                if (was_paused && !is_paused) {
                    ESP_LOGI(TAG, "IMU resuming after movement - syncing balance to stance");
                    gyro_balance_.SyncToStance();
                    push_reaction_.Reset();
                }
                was_paused = is_paused;
                
                if (is_paused) {
                    gyro_balance_.Process(imu_data, dt);
                    vTaskDelay(interval);
                    continue;
                }
                
                if (gyro_balance_.ProcessToggle(imu_data.gyro_x)) {
                    bool new_state = !gyro_balance_.IsEnabled();
                    gyro_balance_.Enable(new_state);
                    if (!new_state) {
                        servo_controller_.GotoStanceSmooth();
                    }
                }
                
                if (push_reaction_.IsEnabled() && !gyro_balance_.IsEnabled()) {
                    PushReactionController::PushDirection push = push_reaction_.ProcessAccel(imu_data.accel_x);
                    
                    if (push != PushReactionController::PUSH_NONE) {
                        push_reaction_.UpdateReactionTime();
                        
                        ESP_LOGI(TAG, "🚶 Triggering walk forward (%d cycles) from push",
                                 REACTION_WALK_CYCLES);
                        servo_controller_.WalkForward(REACTION_WALK_CYCLES);
                        
                        vTaskDelay(interval);
                        continue;
                    }
                }
                
                auto result = gyro_balance_.Process(imu_data, dt);
                
                if (result.should_update) {
                    servo_controller_.ApplyBalance(result.front_offset, result.back_offset, result.speed);
                }
            }
            vTaskDelay(interval);
        }
        
        ESP_LOGI(TAG, "IMU monitoring task stopped");
        vTaskDelete(NULL);
    }
    void PrintMemoryStats() {
        ESP_LOGI(TAG, "═══ MEMORY REPORT ═════════════════════════════");
        ESP_LOGI(TAG, "  Internal free:     %.1f KB",
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f);
        ESP_LOGI(TAG, "  Internal min ever: %.1f KB  <-- watch this",
            heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024.0f);
        ESP_LOGI(TAG, "  PSRAM free:        %.1f KB",
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024.0f);
        ESP_LOGI(TAG, "  DMA free:          %.1f KB",
            heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024.0f);
        ESP_LOGI(TAG, "═══════════════════════════════════════════════");
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

        mcp_server.AddTool("dog.pounce_back", "Make the robot dog pounce back", PropertyList({Property("loops", kPropertyTypeInteger, 1, 1, 100)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog pounce back command received");
                int loops = properties["loops"].value<int>();
                servo_controller_.PounceBack(loops);
                return "Dog pounced back";
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

        // ═══════════════════════════════════════════════════════
        // GYRO BALANCE TOOLS
        // ═══════════════════════════════════════════════════════
        mcp_server.AddTool("dog.balance_enable", "Enable gyro balance mode using Mahony filter (disables push reaction)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Balance enable command received");
                gyro_balance_.Enable(true);
                return "Gyro balance mode enabled (Mahony filter + anti-jitter, push reaction disabled while active)";
            });

        mcp_server.AddTool("dog.balance_disable", "Disable gyro balance mode", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Balance disable command received");
                gyro_balance_.Enable(false);
                servo_controller_.GotoStanceSmooth();
                return "Gyro balance mode disabled";
            });

        mcp_server.AddTool("dog.balance_status", "Get current gyro balance mode status with pitch/roll angles", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                bool balance_enabled = gyro_balance_.IsEnabled();
                bool push_enabled = push_reaction_.IsEnabled();
                bool imu_ok = imu_controller_.IsInitialized();
                float pitch = gyro_balance_.GetPitch();
                float roll = gyro_balance_.GetRoll();
                char status[256];
                snprintf(status, sizeof(status), 
                         "Balance: %s, Push: %s, IMU: %s (0x%02X), Pitch: %.1f°, Roll: %.1f°", 
                         balance_enabled ? "ON" : "OFF",
                         push_enabled ? "ON" : "OFF",
                         imu_ok ? "OK" : "N/A",
                         imu_controller_.GetAddress(),
                         pitch, roll);
                return std::string(status);
            });

        // ═══════════════════════════════════════════════════════
        // PUSH REACTION TOOLS
        // ═══════════════════════════════════════════════════════
        mcp_server.AddTool("dog.push_reaction_enable", "Enable automatic walk-on-push reaction via IMU", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Push reaction enable command received");
                push_reaction_.Enable(true);
                return "Push reaction enabled - dog will walk when pushed (disabled while balance mode is active)";
            });

        mcp_server.AddTool("dog.push_reaction_disable", "Disable automatic walk-on-push reaction via IMU", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Push reaction disable command received");
                push_reaction_.Enable(false);
                return "Push reaction disabled";
            });

        mcp_server.AddTool("dog.push_reaction_status", "Get current push reaction status", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = push_reaction_.IsEnabled();
                bool balance_active = gyro_balance_.IsEnabled();
                bool cooldown_ok = push_reaction_.IsCooldownExpired();
                char status[256];
                snprintf(status, sizeof(status), 
                         "Push reaction: %s, Active: %s, Ready: %s, Threshold: %.1f m/s², Cooldown: %dms",
                         enabled ? "ENABLED" : "DISABLED",
                         (enabled && !balance_active) ? "YES" : "NO (balance mode active)",
                         cooldown_ok ? "YES" : "COOLING DOWN",
                         REACTION_DELTA_THRESHOLD,
                         REACTION_COOLDOWN_MS);
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
        vTaskDelay(pdMS_TO_TICKS(200));
        
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 10 * 1000 * 1000;
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
        vTaskDelay(pdMS_TO_TICKS(150)); 
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        vTaskDelay(pdMS_TO_TICKS(50)); 
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
        
        ESP_LOGI(TAG, "✓ Display initialized");
        display_ = new anim::EmojiWidget(panel, panel_io);
    }

    void InitializeCamera() {
        gpio_set_direction(GPIO_NUM_42, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_42, 0);
        vTaskDelay(pdMS_TO_TICKS(150));

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
        config.fb_count = 3;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;

        camera_ = new Esp32Camera(config);
    }
    void TakeTestPhoto() {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            ESP_LOGI(TAG, "📸 Test photo: %d bytes", (int)fb->len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, fb->buf, 256, ESP_LOG_INFO); // First 256 bytes
            esp_camera_fb_return(fb);
        }
    }

public:
    HeySantaBoard() : boot_button_(BOOT_BUTTON_GPIO), wake_button_(WAKE_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7735Display();
        InitializeButtons();
        InitializeCamera();
        
        vTaskDelay(pdMS_TO_TICKS(500));

        // Camera test
        camera_fb_t* test_fb = esp_camera_fb_get();
        if (test_fb) {
            uint8_t* pixels = test_fb->buf;
            size_t center = (test_fb->height / 2) * test_fb->width * 2;
            ESP_LOGI(TAG, "✓ Camera frame: %zu bytes (%dx%d)",
                    test_fb->len, test_fb->width, test_fb->height);
            ESP_LOGI(TAG, "  Center pixels (raw RGB565): %02X %02X %02X %02X %02X %02X",
                    pixels[center],   pixels[center+1],
                    pixels[center+2], pixels[center+3],
                    pixels[center+4], pixels[center+5]);
            ESP_LOGI(TAG, "  All zeros = camera broken, varied values = camera working");
            esp_camera_fb_return(test_fb);
        } else {
            ESP_LOGE(TAG, "✗ Camera frame grab FAILED - check power/wiring");
        }
        TakeTestPhoto();
        
        servo_controller_.Initialize();
        vTaskDelay(pdMS_TO_TICKS(500));
        
        gyro_balance_.Initialize();
        
        if (imu_controller_.Initialize(i2c_bus_)) {
            imu_task_running_ = true;
            last_imu_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
            xTaskCreate(ImuTaskWrapper, "imu_task", 4096, this, 5, &imu_task_handle_);
            ESP_LOGI(TAG, "✓ IMU task started");
        } else {
            ESP_LOGW(TAG, "✗ IMU initialization failed - balance and push reaction disabled");
        }
        
        InitializeTools();
        // At the end of HeySantaBoard constructor, after InitializeTools()
        // StartCameraHttpServer();
        GetBacklight()->RestoreBrightness();
        
        ESP_LOGI(TAG, "═══════════════════════════════════════════");
        ESP_LOGI(TAG, "HeySanta board initialized!");
        ESP_LOGI(TAG, "  Gyro balance:   %s (Mahony filter + anti-jitter)", GYRO_BALANCE_ENABLED_DEFAULT ? "ENABLED" : "DISABLED");
        ESP_LOGI(TAG, "  Push reaction:  %s", PUSH_REACTION_ENABLED_DEFAULT ? "ENABLED" : "DISABLED");
        ESP_LOGI(TAG, "  Toggle gesture: Rotate X-axis back/forth to toggle balance");
        ESP_LOGI(TAG, "  Movement transition: pre=%dms, post=%dms, end=%dms",
                 PRE_MOVEMENT_SETTLE_MS, POST_MOVEMENT_SETTLE_MS, MOVEMENT_END_WAIT_MS);
        ESP_LOGI(TAG, "  Use MCP tools to enable/disable features");
        ESP_LOGI(TAG, "═══════════════════════════════════════════");
        PrintMemoryStats();  // <-- ADD THIS
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
    // ADD THIS:
    virtual void StartCameraHttpServer() override {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port        = 80;
        config.lru_purge_enable   = true;
        config.max_open_sockets   = 5;       // bumped: snapshot + stream + spare
        config.stack_size         = 8192;    // stream loop needs more stack
        config.recv_wait_timeout  = 10;      // seconds
        config.send_wait_timeout  = 10;

        httpd_handle_t server = nullptr;
        if (httpd_start(&server, &config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start HTTP server");
            return;
        }

        // -- snapshot --
        httpd_uri_t snapshot_uri = {
            .uri      = "/snapshot",
            .method   = HTTP_GET,
            .handler  = camera_snapshot_handler,
            .user_ctx = nullptr,
        };
        httpd_register_uri_handler(server, &snapshot_uri);

        // -- stream --
        httpd_uri_t stream_uri = {
            .uri      = "/stream",
            .method   = HTTP_GET,
            .handler  = camera_stream_handler,
            .user_ctx = nullptr,
        };
        httpd_register_uri_handler(server, &stream_uri);

        std::string ip = WifiStation::GetInstance().GetIpAddress();
        ESP_LOGI(TAG, "📷 Snapshot : http://%s/snapshot", ip.c_str());
        ESP_LOGI(TAG, "📹 Stream   : http://%s/stream",   ip.c_str());
    }
    
};

DECLARE_BOARD(HeySantaBoard);