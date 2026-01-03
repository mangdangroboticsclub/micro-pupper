#include "ble_protocol.h"
#include "esp_log.h"
#include <algorithm>
#include <cJSON.h>

static const char* TAG = "BleProtocol";

#if CONFIG_BT_NIMBLE_ENABLED
#include "nvs_flash.h"

// NimBLE includes (following ESP-IDF example structure)
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include <string>
#include <atomic>
#include <map>
#include <vector>
#include <memory>

// Santa-Bot custom UUIDs (128-bit for uniqueness and proper identification)
static const ble_uuid128_t santa_bot_service_uuid = BLE_UUID128_INIT(SANTA_BOT_SERVICE_UUID_128);
static const ble_uuid128_t santa_bot_characteristic_uuid = BLE_UUID128_INIT(SANTA_BOT_CHARACTERISTIC_UUID_128);

// Global state
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t chr_val_handle;
static std::atomic<bool> is_connected{false};
static BleProtocol::CommandCallback command_callback;
static BleProtocol::ConnectionStateCallback connection_callback;

// BLE Protocol instance pointer for callbacks
static BleProtocol* g_ble_protocol_instance = nullptr;

// Chunk reassembly data structures
struct ChunkData {
    std::vector<std::string> chunks;
    uint32_t total_chunks;
    uint32_t received_chunks;
    uint32_t message_id;
    
    ChunkData(uint32_t total, uint32_t id) : total_chunks(total), received_chunks(0), message_id(id) {
        chunks.resize(total);
    }
};

static std::map<uint32_t, ChunkData> chunk_storage;

// Helper function to process incoming commands and handle chunking
static void process_incoming_command(const std::string& command_str) {
    ESP_LOGI(TAG, "Processing incoming command: %s", command_str.c_str());
    
    // Try to parse as JSON first
    cJSON* json = cJSON_Parse(command_str.c_str());
    if (!json) {
        ESP_LOGW(TAG, "Failed to parse command as JSON, treating as plain text");
        if (command_callback) {
            command_callback(command_str);
        }
        // Also trigger the Protocol's JSON callback if available through a proper method
        if (g_ble_protocol_instance) {
            g_ble_protocol_instance->ProcessTextCommand(command_str);
        }
        return;
    }
    
    // Check if this is a chunked message
    cJSON* chunk_obj = cJSON_GetObjectItem(json, "chunk");
    if (chunk_obj) {
        ESP_LOGI(TAG, "Detected chunked command");
        
        // Extract chunk metadata
        cJSON* id_obj = cJSON_GetObjectItem(chunk_obj, "id");
        cJSON* index_obj = cJSON_GetObjectItem(chunk_obj, "index");
        cJSON* total_obj = cJSON_GetObjectItem(chunk_obj, "total");
        cJSON* data_obj = cJSON_GetObjectItem(chunk_obj, "data");
        
        if (!id_obj || !index_obj || !total_obj || !data_obj ||
            !cJSON_IsNumber(id_obj) || !cJSON_IsNumber(index_obj) || 
            !cJSON_IsNumber(total_obj) || !cJSON_IsString(data_obj)) {
            ESP_LOGE(TAG, "Invalid chunk format");
            cJSON_Delete(json);
            return;
        }
        
        uint32_t message_id = (uint32_t)id_obj->valueint;
        uint32_t chunk_index = (uint32_t)index_obj->valueint;
        uint32_t total_chunks = (uint32_t)total_obj->valueint;
        std::string chunk_data = data_obj->valuestring;
        
        ESP_LOGI(TAG, "Received chunk %lu/%lu for message %lu (data length: %u)", 
                 (unsigned long)(chunk_index + 1), (unsigned long)total_chunks, 
                 (unsigned long)message_id, (unsigned int)chunk_data.length());
        
        // Initialize storage for this message if needed
        auto it = chunk_storage.find(message_id);
        if (it == chunk_storage.end()) {
            auto result = chunk_storage.emplace(message_id, ChunkData(total_chunks, message_id));
            it = result.first;
            ESP_LOGI(TAG, "Initialized storage for message %lu expecting %lu chunks", 
                     (unsigned long)message_id, (unsigned long)total_chunks);
        }
        
        ChunkData& message_data = it->second;
        
        // Validate chunk index
        if (chunk_index >= total_chunks) {
            ESP_LOGE(TAG, "Invalid chunk index %lu (max: %lu)", 
                     (unsigned long)chunk_index, (unsigned long)(total_chunks - 1));
            cJSON_Delete(json);
            return;
        }
        
        // Store this chunk (avoid duplicates)
        if (message_data.chunks[chunk_index].empty()) {
            message_data.chunks[chunk_index] = chunk_data;
            message_data.received_chunks++;
            ESP_LOGI(TAG, "Stored chunk %lu/%lu (%lu/%lu received)", 
                     (unsigned long)(chunk_index + 1), (unsigned long)total_chunks,
                     (unsigned long)message_data.received_chunks, (unsigned long)total_chunks);
        } else {
            ESP_LOGW(TAG, "Duplicate chunk %lu/%lu ignored", 
                     (unsigned long)(chunk_index + 1), (unsigned long)total_chunks);
        }
        
        // Check if we have all chunks
        if (message_data.received_chunks == message_data.total_chunks) {
            // Reassemble the complete message
            std::string complete_message;
            for (const auto& chunk : message_data.chunks) {
                complete_message += chunk;
            }
            
            ESP_LOGI(TAG, "Message %lu reassembled: %lu chunks -> %u bytes", 
                     (unsigned long)message_id, (unsigned long)total_chunks, 
                     (unsigned int)complete_message.length());
            
            // Clean up chunk storage
            chunk_storage.erase(message_id);
            
            // Process the complete message recursively (should not be chunked)
            process_incoming_command(complete_message);
        } else {
            ESP_LOGI(TAG, "Waiting for %lu more chunks...", 
                     (unsigned long)(message_data.total_chunks - message_data.received_chunks));
        }
        
        cJSON_Delete(json);
        return;
    }
    
    // Not a chunked message, process normally
    if (command_callback) {
        command_callback(command_str);
    }
    
    // Also trigger the Protocol's JSON callback if available through a proper method
    if (g_ble_protocol_instance) {
        g_ble_protocol_instance->ProcessJsonCommand(json);
    }
    
    cJSON_Delete(json);
}

// Forward declarations
static int gatt_svr_chr_access_haven(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
static int gatt_svr_init(void);
static void bleprph_on_reset(int reason);
static void bleprph_on_sync(void);
static void bleprph_advertise(void);
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static void bleprph_host_task(void *param);

// GATT service definition
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service: Santa-Bot Robot Control Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &santa_bot_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /*** Characteristic: Robot Command & Response */
            .uuid = &santa_bot_characteristic_uuid.u,
            .access_cb = gatt_svr_chr_access_haven,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &chr_val_handle,
        }, {
            0, /* No more characteristics in this service */
        } }
    },
    {
        0, /* No more services */
    },
};

// GATT characteristic access callback
static int gatt_svr_chr_access_haven(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    ESP_LOGI(TAG, "GATT access: conn_handle=%d attr_handle=%d op=%d", 
             conn_handle, attr_handle, ctxt->op);
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "GATT read request");
        return 0;
        
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "GATT write request, len=%d", ctxt->om->om_len);
        
        if (ctxt->om->om_len > 0) {
            // Extract data from mbuf
            uint16_t len = ctxt->om->om_len;
            char* data = (char*)malloc(len + 1);
            if (data) {
                ble_hs_mbuf_to_flat(ctxt->om, data, len, NULL);
                data[len] = '\0';
                std::string command(data);
                free(data);
                
                ESP_LOGI(TAG, "Received command: %s", command.c_str());
                
                // Process command with chunk reassembly support
                process_incoming_command(command);
            }
        }
        return 0;
        
    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// GATT server initialization
static int gatt_svr_init(void) {
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "registering characteristic %s with def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGI(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

// Advertising
static void bleprph_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof fields);

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = 0; // Use default power level
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "Started advertising as '%s'", ble_svc_gap_device_name());
}

// GAP event handler
static int bleprph_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            conn_handle = event->connect.conn_handle;
            is_connected.store(true);
            
            if (connection_callback) {
                connection_callback(true);
            }
        }
        
        if (event->connect.status != 0) {
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_connected.store(false);
        
        if (connection_callback) {
            connection_callback(false);
        }
        
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d", event->adv_complete.reason);
        bleprph_advertise();
        return 0;

    default:
        return 0;
    }
}

// NimBLE callbacks
static void bleprph_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void bleprph_on_sync(void) {
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    uint8_t own_addr_type;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);
    
    bleprph_advertise();
}

static void bleprph_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// BleProtocol class implementation
BleProtocol::BleProtocol() : impl_(nullptr) {
    g_ble_protocol_instance = this;
}

BleProtocol::~BleProtocol() {
    Stop();
    g_ble_protocol_instance = nullptr;
}

bool BleProtocol::Start() {
    ESP_LOGI(TAG, "Starting BLE protocol with NimBLE stack");
    
    int rc;
    esp_err_t ret;

    // Initialize NimBLE controller and host
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d", ret);
        return false;
    }

    // Initialize the NimBLE host configuration (following ESP-IDF example pattern)
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Initialize GATT server
    rc = gatt_svr_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to init GATT server %d", rc);
        return false;
    }

    // Set the device name from configuration
#ifdef CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME
    const char* device_name = CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME;
#else
    const char* device_name = "Santa-Bot"; // Fallback name
#endif
    rc = ble_svc_gap_device_name_set(device_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name to '%s' %d", device_name, rc);
        return false;
    }
    ESP_LOGI(TAG, "Device name set to: %s", device_name);

    // Initialize store configuration
    // ble_store_config_init();  // This function may not be available in this ESP-IDF version

    // Start the NimBLE host task
    nimble_port_freertos_init(bleprph_host_task);

    ESP_LOGI(TAG, "BLE protocol started successfully");
    return true;
}

void BleProtocol::Stop() {
    ESP_LOGI(TAG, "Stopping BLE protocol");
    // Note: Proper shutdown would require nimble_port_stop(), but for simplicity
    // we'll let the task continue running. Full shutdown can be added if needed.
}

bool BleProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    // BLE is not suitable for high-bandwidth audio streaming
    // This implementation is a placeholder - in practice, you might want to
    // compress audio data heavily or use a different approach
    ESP_LOGW(TAG, "Audio streaming over BLE not implemented (bandwidth limitations)");
    return false;
}

bool BleProtocol::OpenAudioChannel() {
    ESP_LOGI(TAG, "Opening BLE audio channel (simulated)");
    audio_channel_opened_.store(true);
    
    if (on_audio_channel_opened_) {
        on_audio_channel_opened_();
    }
    return true;
}

void BleProtocol::CloseAudioChannel() {
    ESP_LOGI(TAG, "Closing BLE audio channel");
    audio_channel_opened_.store(false);
    
    if (on_audio_channel_closed_) {
        on_audio_channel_closed_();
    }
}

bool BleProtocol::IsAudioChannelOpened() const {
    return audio_channel_opened_.load();
}

bool BleProtocol::IsConnected() const {
    return is_connected.load();
}

void BleProtocol::OnCommand(CommandCallback callback) {
    command_callback_ = callback;
    command_callback = callback;
}

void BleProtocol::OnConnectionState(ConnectionStateCallback callback) {
    connection_callback_ = callback;
    connection_callback = callback;
}

bool BleProtocol::SendText(const std::string& text) {
    return SendResponse(text);
}

bool BleProtocol::HandleInternalCommand(const std::string& command) {
    ESP_LOGI(TAG, "Handling internal command via BLE thread: %s", command.c_str());
    
    // Execute the command directly on the BLE thread using the existing callback mechanism
    if (command_callback_) {
        command_callback_(command);
        return true;
    } else {
        ESP_LOGW(TAG, "No command callback registered for internal command");
        return false;
    }
}

void BleProtocol::ProcessJsonCommand(const cJSON* json) {
    if (on_incoming_json_) {
        on_incoming_json_(json);
    }
}

void BleProtocol::ProcessTextCommand(const std::string& text) {
    // Create a simple JSON object for non-JSON commands
    cJSON* wrapper = cJSON_CreateObject();
    cJSON_AddStringToObject(wrapper, "text", text.c_str());
    cJSON_AddStringToObject(wrapper, "type", "text_command");
    
    if (on_incoming_json_) {
        on_incoming_json_(wrapper);
    }
    
    cJSON_Delete(wrapper);
}

const char* BleProtocol::GetDeviceName() {
#ifdef CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME
    return CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME;
#else
    return "Mangdang-Bot"; // Fallback name
#endif
}

bool BleProtocol::SendResponse(const std::string& response) {
    if (!is_connected.load() || conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "Cannot send response: not connected");
        return false;
    }

    // Log the original response (safely truncated for display)
    ESP_LOGI(TAG, "Sending response (%u bytes)", (unsigned int)response.length());
    if (response.length() > 0) {
        size_t preview_len = std::min(response.length(), (size_t)200);
        std::string preview = response.substr(0, preview_len);
        ESP_LOGI(TAG, "Response preview: %s%s", preview.c_str(), response.length() > 200 ? "..." : "");
    }
    
    // If response is small enough, send as single notification
    if (response.length() <= MAX_CHUNK_SIZE) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(response.c_str(), response.length());
        if (om == NULL) {
            ESP_LOGE(TAG, "Failed to allocate mbuf for response");
            return false;
        }

        int rc = ble_gatts_notify_custom(conn_handle, chr_val_handle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to send notification; rc=%d", rc);
            return false;
        }
        
        ESP_LOGI(TAG, "Single response sent successfully");
        return true;
    } else {
        // Send as chunked response
        ESP_LOGI(TAG, "Response too large (%u bytes), sending in chunks", (unsigned int)response.length());
        return SendChunkedResponse(response);
    }
}

bool BleProtocol::SendChunkedResponse(const std::string& response) {
    ESP_LOGI(TAG, "Sending chunked response (%u bytes total)", (unsigned int)response.length());
    
    // Get the current BLE MTU for this connection
    uint16_t mtu = ble_att_mtu(conn_handle);
    if (mtu == 0) mtu = 23; // Default minimum ATT MTU
    
    // Account for ATT header (3 bytes) and some safety margin
    size_t effective_mtu = mtu - 10; 
    ESP_LOGI(TAG, "BLE MTU: %u, effective payload limit: %u", mtu, (unsigned int)effective_mtu);
    
    // Generate a unique message ID for this chunked message
    static uint32_t message_id = 1;
    uint32_t current_msg_id = message_id++;
    
    size_t total_chunks = (response.length() + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    size_t chunk_index = 0;
    
    for (size_t offset = 0; offset < response.length(); offset += MAX_CHUNK_SIZE) {
        size_t chunk_size = std::min(MAX_CHUNK_SIZE, response.length() - offset);
        std::string chunk_data = response.substr(offset, chunk_size);
        
        // Create chunk header with metadata
        char chunk_header[100];
        snprintf(chunk_header, sizeof(chunk_header), 
                "{\"chunk\":{\"id\":%lu,\"index\":%u,\"total\":%u,\"data\":\"", 
                (unsigned long)current_msg_id, (unsigned int)chunk_index, (unsigned int)total_chunks);
        
        // Escape the chunk data for JSON
        std::string escaped_data;
        escaped_data.reserve(chunk_data.length() * 2); // Reserve space for escaping
        for (char c : chunk_data) {
            switch (c) {
                case '"':  escaped_data += "\\\""; break;
                case '\\': escaped_data += "\\\\"; break;
                case '\b': escaped_data += "\\b"; break;
                case '\f': escaped_data += "\\f"; break;
                case '\n': escaped_data += "\\n"; break;
                case '\r': escaped_data += "\\r"; break;
                case '\t': escaped_data += "\\t"; break;
                default:
                    if ((unsigned char)c < 0x20) {
                        // Escape other control characters as unicode
                        char buf[7];
                        snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
                        escaped_data += buf;
                    } else {
                        escaped_data += c;
                    }
                    break;
            }
        }
        
        std::string chunk_message = std::string(chunk_header) + escaped_data + "\"}}";
        
        // Check if chunk message exceeds effective MTU
        if (chunk_message.length() > effective_mtu) {
            ESP_LOGW(TAG, "Chunk message (%u bytes) exceeds effective MTU (%u bytes), adjusting", 
                     (unsigned int)chunk_message.length(), (unsigned int)effective_mtu);
            
            // Recalculate with smaller chunk to fit in MTU
            size_t header_overhead = strlen(chunk_header) + 3; // +3 for "}}
            size_t max_escaped_size = effective_mtu - header_overhead;
            
            // Estimate raw data size (accounting for potential escaping)
            size_t safe_raw_size = max_escaped_size / 2; // Conservative estimate for escaping
            if (safe_raw_size < chunk_data.length()) {
                chunk_data = response.substr(offset, safe_raw_size);
                
                // Re-escape the adjusted data
                escaped_data.clear();
                escaped_data.reserve(chunk_data.length() * 2);
                for (char c : chunk_data) {
                    switch (c) {
                        case '"':  escaped_data += "\\\""; break;
                        case '\\': escaped_data += "\\\\"; break;
                        case '\b': escaped_data += "\\b"; break;
                        case '\f': escaped_data += "\\f"; break;
                        case '\n': escaped_data += "\\n"; break;
                        case '\r': escaped_data += "\\r"; break;
                        case '\t': escaped_data += "\\t"; break;
                        default:
                            if ((unsigned char)c < 0x20) {
                                // Escape other control characters as unicode
                                char buf[7];
                                snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
                                escaped_data += buf;
                            } else {
                                escaped_data += c;
                            }
                            break;
                    }
                }
                chunk_message = std::string(chunk_header) + escaped_data + "\"}}";
            }
        }
        
        ESP_LOGI(TAG, "Sending chunk %u/%u (%u bytes)", 
                 (unsigned int)(chunk_index + 1), (unsigned int)total_chunks, (unsigned int)chunk_message.length());
        
        struct os_mbuf *om = ble_hs_mbuf_from_flat(chunk_message.c_str(), chunk_message.length());
        if (om == NULL) {
            ESP_LOGE(TAG, "Failed to allocate mbuf for chunk %u", (unsigned int)chunk_index);
            return false;
        }

        int rc = ble_gatts_notify_custom(conn_handle, chr_val_handle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to send chunk %u; rc=%d", (unsigned int)chunk_index, rc);
            return false;
        }
        
        chunk_index++;
        
        // Small delay between chunks to prevent overwhelming the BLE stack
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "All %u chunks sent successfully", (unsigned int)total_chunks);
    return true;
}

#else

// BLE disabled - provide stub implementation
// TAG already defined at top of file

BleProtocol::BleProtocol() : impl_(nullptr) {
}

BleProtocol::~BleProtocol() {
}

bool BleProtocol::Start() {
    ESP_LOGI(TAG, "BLE protocol disabled (CONFIG_BT_NIMBLE_ENABLED=n)");
    return true;
}

void BleProtocol::Stop() {
    ESP_LOGI(TAG, "BLE protocol stop (disabled)");
}

bool BleProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    ESP_LOGI(TAG, "BLE send audio (disabled)");
    return true;
}

bool BleProtocol::OpenAudioChannel() {
    ESP_LOGI(TAG, "BLE open audio channel (disabled)");
    audio_channel_opened_.store(true);
    if (on_audio_channel_opened_) {
        on_audio_channel_opened_();
    }
    return true;
}

void BleProtocol::CloseAudioChannel() {
    ESP_LOGI(TAG, "BLE close audio channel (disabled)");
    audio_channel_opened_.store(false);
    if (on_audio_channel_closed_) {
        on_audio_channel_closed_();
    }
}

bool BleProtocol::IsAudioChannelOpened() const {
    return audio_channel_opened_.load();
}

bool BleProtocol::IsConnected() const {
    return false;
}

void BleProtocol::OnCommand(CommandCallback callback) {
    ESP_LOGI(TAG, "BLE command callback set (disabled)");
    command_callback_ = callback;
}

void BleProtocol::OnConnectionState(ConnectionStateCallback callback) {
    ESP_LOGI(TAG, "BLE connection state callback set (disabled)");
    connection_callback_ = callback;
}

bool BleProtocol::SendResponse(const std::string& response) {
    ESP_LOGI(TAG, "BLE send response (disabled): %s", response.c_str());
    return true;
}

bool BleProtocol::SendText(const std::string& text) {
    ESP_LOGI(TAG, "BLE send text (disabled): %s", text.c_str());
    return true;
}

bool BleProtocol::HandleInternalCommand(const std::string& command) {
    ESP_LOGI(TAG, "BLE handle internal command (disabled): %s", command.c_str());
    return true;
}

void BleProtocol::ProcessJsonCommand(const cJSON* json) {
    ESP_LOGI(TAG, "BLE process JSON command (disabled)");
}

void BleProtocol::ProcessTextCommand(const std::string& text) {
    ESP_LOGI(TAG, "BLE process text command (disabled): %s", text.c_str());
}

bool BleProtocol::SendChunkedResponse(const std::string& response) {
    ESP_LOGI(TAG, "BLE send chunked response (disabled): %u bytes", (unsigned int)response.length());
    return true;
}

const char* BleProtocol::GetDeviceName() {
    return "Santa-Bot (BLE Disabled)";
}

#endif
