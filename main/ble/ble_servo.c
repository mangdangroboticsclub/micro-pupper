/**
 * @file ble_servo.c
 * @brief Minimal BLE Servo Controller Implementation
 * 
 * Uses NimBLE for Web Bluetooth compatible servo control.
 */

#include "ble_servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG = "BLE_SERVO";

#if CONFIG_BT_NIMBLE_ENABLED

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// UUIDs for Web Bluetooth
// Service: 0d9be2a0-4757-43d9-83df-704ae274b8df
// Char:    8116d8c0-d45d-4fdf-998e-33ab8c471d59
#define SERVICE_UUID_128 \
    0xdf, 0xb8, 0x74, 0xe2, 0x4a, 0x70, 0xdf, 0x83, \
    0xd9, 0x43, 0x57, 0x47, 0xa0, 0xe2, 0x9b, 0x0d

#define CHAR_UUID_128 \
    0x59, 0x1d, 0x47, 0x8c, 0xab, 0x33, 0x8e, 0x99, \
    0xdf, 0x4f, 0x5d, 0xd4, 0xc0, 0xd8, 0x16, 0x81

static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(SERVICE_UUID_128);
static const ble_uuid128_t chr_uuid = BLE_UUID128_INIT(CHAR_UUID_128);

// State
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_handle;
static bool s_connected = false;

// Chunked message buffer
#define CHUNK_BUFFER_SIZE 2048
static char s_chunk_buffer[CHUNK_BUFFER_SIZE];
static size_t s_chunk_len = 0;
static uint8_t s_chunk_expected = 0;
static uint8_t s_chunk_received = 0;

// Callbacks
static ble_servo_move_cb_t s_move_cb = NULL;
static ble_servo_leg_move_cb_t s_leg_move_cb = NULL;
static ble_servo_stance_cb_t s_stance_cb = NULL;
static ble_servo_connect_cb_t s_connect_cb = NULL;

// Forward declarations
static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void on_sync(void);
static void on_reset(int reason);
static void host_task(void *param);
static void start_advertising(void);
static void process_command(const char* cmd, size_t len);

// ═══════════════════════════════════════════════════════
// CHUNKED MESSAGE HANDLING
// ═══════════════════════════════════════════════════════

/**
 * Reset chunk buffer state
 */
static void chunk_reset(void) {
    s_chunk_len = 0;
    s_chunk_expected = 0;
    s_chunk_received = 0;
    s_chunk_buffer[0] = '\0';
}

/**
 * Handle incoming data - either chunked or regular
 * Chunk format: {"k":<num>,"t":<total>,"d":"<data>"}
 * When all chunks received, concatenates and processes as single command
 */
static void handle_incoming_data(const char* data, size_t len) {
    // Try to parse as chunk header
    cJSON* json = cJSON_ParseWithLength(data, len);
    if (!json) {
        ESP_LOGW(TAG, "Invalid JSON");
        return;
    }
    
    cJSON* k = cJSON_GetObjectItem(json, "k");  // chunk number (1-based)
    cJSON* t = cJSON_GetObjectItem(json, "t");  // total chunks
    cJSON* d = cJSON_GetObjectItem(json, "d");  // data payload
    
    // Check if this is a chunked message
    if (k && t && d && cJSON_IsNumber(k) && cJSON_IsNumber(t) && cJSON_IsString(d)) {
        uint8_t chunk_num = (uint8_t)k->valueint;
        uint8_t total = (uint8_t)t->valueint;
        const char* payload = d->valuestring;
        size_t payload_len = strlen(payload);
        
        ESP_LOGI(TAG, "Chunk %d/%d (%zu bytes)", chunk_num, total, payload_len);
        
        // First chunk - reset buffer
        if (chunk_num == 1) {
            chunk_reset();
            s_chunk_expected = total;
        }
        
        // Validate sequence
        if (chunk_num != s_chunk_received + 1 || total != s_chunk_expected) {
            ESP_LOGW(TAG, "Chunk sequence error, resetting");
            chunk_reset();
            ble_servo_send_response("{\"err\":\"chunk_seq\"}");
            cJSON_Delete(json);
            return;
        }
        
        // Append to buffer if there's room
        if (s_chunk_len + payload_len < CHUNK_BUFFER_SIZE - 1) {
            memcpy(s_chunk_buffer + s_chunk_len, payload, payload_len);
            s_chunk_len += payload_len;
            s_chunk_buffer[s_chunk_len] = '\0';
            s_chunk_received = chunk_num;
            
            // Send ack for this chunk
            char ack[32];
            snprintf(ack, sizeof(ack), "{\"ack\":%d}", chunk_num);
            ble_servo_send_response(ack);
            
            // All chunks received - process complete message
            if (s_chunk_received == s_chunk_expected) {
                ESP_LOGI(TAG, "All chunks received, total %zu bytes", s_chunk_len);
                process_command(s_chunk_buffer, s_chunk_len);
                chunk_reset();
            }
        } else {
            ESP_LOGE(TAG, "Chunk buffer overflow");
            chunk_reset();
            ble_servo_send_response("{\"err\":\"overflow\"}");
        }
        
        cJSON_Delete(json);
        return;
    }
    
    // Not a chunk - process as regular command
    cJSON_Delete(json);
    process_command(data, len);
}

// ═══════════════════════════════════════════════════════
// COMMAND PROCESSING
// ═══════════════════════════════════════════════════════

static void process_move_array(cJSON* arr) {
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) < 5) return;
    
    float fr = (float)cJSON_GetArrayItem(arr, 0)->valuedouble;
    float fl = (float)cJSON_GetArrayItem(arr, 1)->valuedouble;
    float br = (float)cJSON_GetArrayItem(arr, 2)->valuedouble;
    float bl = (float)cJSON_GetArrayItem(arr, 3)->valuedouble;
    uint16_t speed = (uint16_t)cJSON_GetArrayItem(arr, 4)->valueint;
    uint16_t delay_ms = 0;
    if (cJSON_GetArraySize(arr) > 5) {
        delay_ms = (uint16_t)cJSON_GetArrayItem(arr, 5)->valueint;
    }
    
    ESP_LOGI(TAG, "Move: FR=%.0f FL=%.0f BR=%.0f BL=%.0f spd=%u dly=%u",
             fr, fl, br, bl, speed, delay_ms);
    
    if (s_move_cb) {
        s_move_cb(fr, fl, br, bl, speed, delay_ms);
    }
    
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief Parse a leg array [angle, speed, delay] into ble_leg_move_t
 */
static bool parse_leg_params(cJSON* arr, ble_leg_move_t* out) {
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) < 3) return false;
    out->angle = (float)cJSON_GetArrayItem(arr, 0)->valuedouble;
    out->speed = (uint16_t)cJSON_GetArrayItem(arr, 1)->valueint;
    out->delay_ms = (uint16_t)cJSON_GetArrayItem(arr, 2)->valueint;
    return true;
}

/**
 * @brief Process per-leg move array: [[fr_angle,fr_spd,fr_dly], [fl...], [br...], [bl...]]
 */
static void process_leg_move_array(cJSON* arr) {
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) != 4) {
        ESP_LOGW(TAG, "Per-leg move requires 4 leg arrays");
        return;
    }
    
    ble_leg_move_t fr, fl, br, bl;
    if (!parse_leg_params(cJSON_GetArrayItem(arr, 0), &fr) ||
        !parse_leg_params(cJSON_GetArrayItem(arr, 1), &fl) ||
        !parse_leg_params(cJSON_GetArrayItem(arr, 2), &br) ||
        !parse_leg_params(cJSON_GetArrayItem(arr, 3), &bl)) {
        ESP_LOGW(TAG, "Invalid leg params format");
        return;
    }
    
    ESP_LOGI(TAG, "Leg move: FR=%.0f/%u/%u FL=%.0f/%u/%u BR=%.0f/%u/%u BL=%.0f/%u/%u",
             fr.angle, fr.speed, fr.delay_ms,
             fl.angle, fl.speed, fl.delay_ms,
             br.angle, br.speed, br.delay_ms,
             bl.angle, bl.speed, bl.delay_ms);
    
    if (s_leg_move_cb) {
        s_leg_move_cb(fr, fl, br, bl);
    }
}

static void process_command(const char* cmd, size_t len) {
    ESP_LOGI(TAG, "Cmd: %.*s", (int)len, cmd);
    
    cJSON* json = cJSON_ParseWithLength(cmd, len);
    if (!json) {
        ESP_LOGW(TAG, "Invalid JSON");
        return;
    }
    
    // Single move: {"s":[fr,fl,br,bl,speed,delay]}
    cJSON* s = cJSON_GetObjectItem(json, "s");
    if (s && cJSON_IsArray(s)) {
        process_move_array(s);
        cJSON_Delete(json);
        return;
    }
    
    // Sequence: {"m":[[fr,fl,br,bl,speed,delay], ...]}
    cJSON* m = cJSON_GetObjectItem(json, "m");
    if (m && cJSON_IsArray(m)) {
        int count = cJSON_GetArraySize(m);
        ESP_LOGI(TAG, "Sequence: %d moves", count);
        for (int i = 0; i < count; i++) {
            cJSON* move = cJSON_GetArrayItem(m, i);
            if (cJSON_IsArray(move)) {
                process_move_array(move);
            }
        }
        ble_servo_send_response("{\"ok\":1}");
        cJSON_Delete(json);
        return;
    }
    
    // Per-leg single move: {"l":[[fr,spd,dly],[fl,spd,dly],[br,spd,dly],[bl,spd,dly]]}
    cJSON* l = cJSON_GetObjectItem(json, "l");
    if (l && cJSON_IsArray(l)) {
        process_leg_move_array(l);
        cJSON_Delete(json);
        return;
    }
    
    // Per-leg sequence: {"L":[[[fr],[fl],[br],[bl]], ...]}
    cJSON* L = cJSON_GetObjectItem(json, "L");
    if (L && cJSON_IsArray(L)) {
        int count = cJSON_GetArraySize(L);
        ESP_LOGI(TAG, "Per-leg sequence: %d moves", count);
        for (int i = 0; i < count; i++) {
            cJSON* legMove = cJSON_GetArrayItem(L, i);
            if (cJSON_IsArray(legMove)) {
                process_leg_move_array(legMove);
            }
        }
        ble_servo_send_response("{\"ok\":1}");
        cJSON_Delete(json);
        return;
    }
    
    // Offset gait: {"o":{"d":[fl,br,fr,bl],"s":speed,"k":[[fr,fl,br,bl],...]}}
    // d = start delays for each leg (in order FL, BR, FR, BL for diagonal gait)
    // s = servo speed
    // k = keyframes array with angles for all 4 legs
    //
    // Diagonal gait pairing (matching KTurtle):
    //   Pair 1: FL + BR (front-left and back-right)
    //   Pair 2: FR + BL (front-right and back-left)
    // The offset determines when Pair 2 starts after Pair 1
    cJSON* o = cJSON_GetObjectItem(json, "o");
    if (o && cJSON_IsObject(o)) {
        cJSON* delays = cJSON_GetObjectItem(o, "d");
        cJSON* speed_val = cJSON_GetObjectItem(o, "s");
        cJSON* keyframes = cJSON_GetObjectItem(o, "k");
        
        if (delays && cJSON_IsArray(delays) && cJSON_GetArraySize(delays) == 4 &&
            speed_val && cJSON_IsNumber(speed_val) &&
            keyframes && cJSON_IsArray(keyframes) && cJSON_GetArraySize(keyframes) > 0) {
            
            // Parse offsets: [FL offset, BR offset, FR offset, BL offset]
            // These determine when each leg starts its keyframe sequence
            int fl_delay = cJSON_GetArrayItem(delays, 0)->valueint;
            int br_delay = cJSON_GetArrayItem(delays, 1)->valueint;
            int fr_delay = cJSON_GetArrayItem(delays, 2)->valueint;
            int bl_delay = cJSON_GetArrayItem(delays, 3)->valueint;
            uint16_t speed = (uint16_t)speed_val->valueint;
            int kf_count = cJSON_GetArraySize(keyframes);
            
            ESP_LOGI(TAG, "Offset gait: FL=%d BR=%d FR=%d BL=%d spd=%u kfs=%d",
                     fl_delay, br_delay, fr_delay, bl_delay, speed, kf_count);
            
            // Get step duration from first keyframe (index 4 if present)
            cJSON* first_kf = cJSON_GetArrayItem(keyframes, 0);
            int step_duration = 100; // default 100ms per keyframe
            if (cJSON_GetArraySize(first_kf) > 4) {
                step_duration = cJSON_GetArrayItem(first_kf, 4)->valueint;
            }
            
            // Calculate total animation length
            int max_offset = fl_delay;
            if (br_delay > max_offset) max_offset = br_delay;
            if (fr_delay > max_offset) max_offset = fr_delay;
            if (bl_delay > max_offset) max_offset = bl_delay;
            int total_duration = max_offset + (kf_count * step_duration);
            
            ESP_LOGI(TAG, "Total gait duration: %d ms, step: %d ms", total_duration, step_duration);
            
            // Track current step index for each leg (-1 = not started yet)
            int fl_step = -1, br_step = -1, fr_step = -1, bl_step = -1;
            // Track elapsed time for each leg (negative = waiting to start)
            int fl_elapsed = -fl_delay;
            int br_elapsed = -br_delay;
            int fr_elapsed = -fr_delay;
            int bl_elapsed = -bl_delay;
            // Track time within current step for each leg
            int fl_step_time = 0, br_step_time = 0, fr_step_time = 0, bl_step_time = 0;
            
            int64_t start_time = esp_timer_get_time();
            int last_tick = 0;
            
            // Run the gait animation
            while (1) {
                int64_t now = esp_timer_get_time();
                int current_tick = (now - start_time) / 1000;  // Convert to ms
                int delta = current_tick - last_tick;
                
                if (delta < 10) {  // Update every 10ms minimum
                    vTaskDelay(pdMS_TO_TICKS(5));
                    continue;
                }
                
                last_tick = current_tick;
                
                // Update elapsed time for each leg
                fl_elapsed += delta;
                br_elapsed += delta;
                fr_elapsed += delta;
                bl_elapsed += delta;
                fl_step_time += delta;
                br_step_time += delta;
                fr_step_time += delta;
                bl_step_time += delta;
                
                // FL leg update
                if (fl_elapsed >= 0 && fl_step < kf_count) {
                    // Check if we need to advance to next step (or start first step)
                    int kf_duration = step_duration;
                    if (fl_step >= 0) {
                        cJSON* kf = cJSON_GetArrayItem(keyframes, fl_step);
                        if (cJSON_GetArraySize(kf) > 4) {
                            kf_duration = cJSON_GetArrayItem(kf, 4)->valueint;
                        }
                    }
                    
                    if (fl_step < 0 || fl_step_time >= kf_duration) {
                        fl_step++;
                        fl_step_time = 0;
                        
                        if (fl_step < kf_count) {
                            cJSON* kf = cJSON_GetArrayItem(keyframes, fl_step);
                            float angle = (float)cJSON_GetArrayItem(kf, 1)->valuedouble;
                            ESP_LOGI(TAG, "FL -> step %d, angle %.0f", fl_step, angle);
                            if (s_move_cb) s_move_cb(-1, angle, -1, -1, speed, 0);
                        }
                    }
                }
                
                // BR leg update
                if (br_elapsed >= 0 && br_step < kf_count) {
                    int kf_duration = step_duration;
                    if (br_step >= 0) {
                        cJSON* kf = cJSON_GetArrayItem(keyframes, br_step);
                        if (cJSON_GetArraySize(kf) > 4) {
                            kf_duration = cJSON_GetArrayItem(kf, 4)->valueint;
                        }
                    }
                    
                    if (br_step < 0 || br_step_time >= kf_duration) {
                        br_step++;
                        br_step_time = 0;
                        
                        if (br_step < kf_count) {
                            cJSON* kf = cJSON_GetArrayItem(keyframes, br_step);
                            float angle = (float)cJSON_GetArrayItem(kf, 2)->valuedouble;
                            ESP_LOGI(TAG, "BR -> step %d, angle %.0f", br_step, angle);
                            if (s_move_cb) s_move_cb(-1, -1, angle, -1, speed, 0);
                        }
                    }
                }
                
                // FR leg update
                if (fr_elapsed >= 0 && fr_step < kf_count) {
                    int kf_duration = step_duration;
                    if (fr_step >= 0) {
                        cJSON* kf = cJSON_GetArrayItem(keyframes, fr_step);
                        if (cJSON_GetArraySize(kf) > 4) {
                            kf_duration = cJSON_GetArrayItem(kf, 4)->valueint;
                        }
                    }
                    
                    if (fr_step < 0 || fr_step_time >= kf_duration) {
                        fr_step++;
                        fr_step_time = 0;
                        
                        if (fr_step < kf_count) {
                            cJSON* kf = cJSON_GetArrayItem(keyframes, fr_step);
                            float angle = (float)cJSON_GetArrayItem(kf, 0)->valuedouble;
                            ESP_LOGI(TAG, "FR -> step %d, angle %.0f", fr_step, angle);
                            if (s_move_cb) s_move_cb(angle, -1, -1, -1, speed, 0);
                        }
                    }
                }
                
                // BL leg update
                if (bl_elapsed >= 0 && bl_step < kf_count) {
                    int kf_duration = step_duration;
                    if (bl_step >= 0) {
                        cJSON* kf = cJSON_GetArrayItem(keyframes, bl_step);
                        if (cJSON_GetArraySize(kf) > 4) {
                            kf_duration = cJSON_GetArrayItem(kf, 4)->valueint;
                        }
                    }
                    
                    if (bl_step < 0 || bl_step_time >= kf_duration) {
                        bl_step++;
                        bl_step_time = 0;
                        
                        if (bl_step < kf_count) {
                            cJSON* kf = cJSON_GetArrayItem(keyframes, bl_step);
                            float angle = (float)cJSON_GetArrayItem(kf, 3)->valuedouble;
                            ESP_LOGI(TAG, "BL -> step %d, angle %.0f", bl_step, angle);
                            if (s_move_cb) s_move_cb(-1, -1, -1, angle, speed, 0);
                        }
                    }
                }
                
                // Check if all legs finished
                if (fl_step >= kf_count && br_step >= kf_count && 
                    fr_step >= kf_count && bl_step >= kf_count) {
                    ESP_LOGI(TAG, "Offset gait complete");
                    break;
                }
                
                // Safety timeout (30 seconds max)
                if (current_tick > 30000) {
                    ESP_LOGW(TAG, "Offset gait timeout");
                    break;
                }
                
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            ble_servo_send_response("{\"ok\":1}");
        } else {
            ESP_LOGW(TAG, "Invalid offset gait format");
            ble_servo_send_response("{\"err\":\"offset_fmt\"}");
        }
        cJSON_Delete(json);
        return;
    }
    
    // Ping: {"p":1}
    cJSON* p = cJSON_GetObjectItem(json, "p");
    if (p) {
        ble_servo_send_response("{\"p\":1}");
        cJSON_Delete(json);
        return;
    }
    
    // Stance: {"r":1}
    cJSON* r = cJSON_GetObjectItem(json, "r");
    if (r) {
        ESP_LOGI(TAG, "Return to stance");
        if (s_stance_cb) {
            s_stance_cb();
        }
        ble_servo_send_response("{\"ok\":1}");
        cJSON_Delete(json);
        return;
    }
    
    ESP_LOGW(TAG, "Unknown command");
    cJSON_Delete(json);
}

// ═══════════════════════════════════════════════════════
// GATT CALLBACKS
// ═══════════════════════════════════════════════════════

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (ctxt->om && ctxt->om->om_len > 0) {
            char* buf = malloc(ctxt->om->om_len + 1);
            if (buf) {
                ble_hs_mbuf_to_flat(ctxt->om, buf, ctxt->om->om_len, NULL);
                buf[ctxt->om->om_len] = '\0';
                handle_incoming_data(buf, ctxt->om->om_len);
                free(buf);
            }
        }
    }
    return 0;
}

// GATT service definition
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &chr_uuid.u,
            .access_cb = chr_access_cb,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &s_chr_handle,
        }, { 0 } }
    },
    { 0 },
};

// ═══════════════════════════════════════════════════════
// GAP / ADVERTISING
// ═══════════════════════════════════════════════════════

static void start_advertising(void) {
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = 0;
    fields.name = (uint8_t*)ble_svc_gap_device_name();
    fields.name_len = strlen((char*)fields.name);
    fields.name_is_complete = 1;
    
    ble_gap_adv_set_fields(&fields);
    
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_cb, NULL);
    
    ESP_LOGI(TAG, "Advertising as '%s'", ble_svc_gap_device_name());
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            ESP_LOGI(TAG, "Connected");
            if (s_connect_cb) s_connect_cb(true);
        } else {
            start_advertising();
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_connected = false;
        ESP_LOGI(TAG, "Disconnected");
        if (s_connect_cb) s_connect_cb(false);
        start_advertising();
        break;
        
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    }
    return 0;
}

static void on_sync(void) {
    ble_hs_util_ensure_addr(0);
    
    uint8_t addr[6];
    ble_hs_id_copy_addr(BLE_OWN_ADDR_PUBLIC, addr, NULL);
    ESP_LOGI(TAG, "BLE Addr: %02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    
    start_advertising();
}

static void on_reset(int reason) {
    ESP_LOGE(TAG, "BLE reset: %d", reason);
}

static void host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool ble_servo_init(ble_servo_move_cb_t move_cb,
                    ble_servo_leg_move_cb_t leg_move_cb,
                    ble_servo_stance_cb_t stance_cb,
                    ble_servo_connect_cb_t connect_cb) {
    ESP_LOGI(TAG, "Initializing BLE servo controller");
    
    s_move_cb = move_cb;
    s_leg_move_cb = leg_move_cb;
    s_stance_cb = stance_cb;
    s_connect_cb = connect_cb;
    
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return false;
    }
    
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    
    ble_svc_gap_device_name_set(BLE_SERVO_DEVICE_NAME);
    
    nimble_port_freertos_init(host_task);
    
    ESP_LOGI(TAG, "BLE ready - device: %s", BLE_SERVO_DEVICE_NAME);
    return true;
}

bool ble_servo_is_connected(void) {
    return s_connected;
}

bool ble_servo_send_response(const char* msg) {
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) return false;
    
    int rc = ble_gatts_notify_custom(s_conn_handle, s_chr_handle, om);
    return rc == 0;
}

bool ble_servo_send_state(float fr, float fl, float br, float bl) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"pos\":[%.0f,%.0f,%.0f,%.0f]}", fr, fl, br, bl);
    return ble_servo_send_response(buf);
}

#else // BLE disabled

bool ble_servo_init(ble_servo_move_cb_t move_cb,
                    ble_servo_leg_move_cb_t leg_move_cb,
                    ble_servo_stance_cb_t stance_cb,
                    ble_servo_connect_cb_t connect_cb) {
    ESP_LOGW(TAG, "BLE disabled in config");
    return false;
}

bool ble_servo_is_connected(void) { return false; }
bool ble_servo_send_response(const char* msg) { return false; }
bool ble_servo_send_state(float fr, float fl, float br, float bl) { return false; }

#endif
