#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "assets/lang_config.h"
#include "mcp_server.h"
#if CONFIG_BT_NIMBLE_ENABLED
#include "protocols/ble_protocol.h"
#endif

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#define TAG "Application"

static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "audio_testing",
    "fatal_error",
    "invalid_state"
};

Application::Application() {
    event_group_ = xEventGroupCreate();
    web_control_panel_active_ = false; // Initialize the web control panel flag
    ble_connected_ = false; // Initialize BLE connection state

#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            Application* app = (Application*)arg;
            app->OnClockTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
#if CONFIG_BT_NIMBLE_ENABLED
    if (ble_protocol_) {
        ble_protocol_->Stop();
        ble_protocol_.reset();
    }
#endif
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    vEventGroupDelete(event_group_);
}

void Application::CheckNewVersion(Ota& ota) {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10; // 初始重试延迟为10秒

    auto& board = Board::GetInstance();
    while (true) {
        SetDeviceState(kDeviceStateActivating);
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        if (!ota.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char buffer[128];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay, ota.GetCheckVersionUrl().c_str());
            Alert(Lang::Strings::ERROR, buffer, "sad", Lang::Sounds::P3_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (device_state_ == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2; // 每次重试后延迟时间翻倍
            continue;
        }
        retry_count = 0;
        retry_delay = 10; // 重置重试延迟时间

        if (ota.HasNewVersion()) {
            Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "happy", Lang::Sounds::P3_UPGRADE);

            vTaskDelay(pdMS_TO_TICKS(3000));

            SetDeviceState(kDeviceStateUpgrading);
            
            display->SetIcon(FONT_AWESOME_DOWNLOAD);
            std::string message = std::string(Lang::Strings::NEW_VERSION) + ota.GetFirmwareVersion();
            display->SetChatMessage("system", message.c_str());

            board.SetPowerSaveMode(false);
            audio_service_.Stop();
            vTaskDelay(pdMS_TO_TICKS(1000));

            bool upgrade_success = ota.StartUpgrade([display](int progress, size_t speed) {
                std::thread([display, progress, speed]() {
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                    display->SetChatMessage("system", buffer);
                }).detach();
            });

            if (!upgrade_success) {
                // Upgrade failed, restart audio service and continue running
                ESP_LOGE(TAG, "Firmware upgrade failed, restarting audio service and continuing operation...");
                audio_service_.Start(); // Restart audio service
                board.SetPowerSaveMode(true); // Restore power save mode
                Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "sad", Lang::Sounds::P3_EXCLAMATION);
                vTaskDelay(pdMS_TO_TICKS(3000));
                // Continue to normal operation (don't break, just fall through)
            } else {
                // Upgrade success, reboot immediately
                ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
                display->SetChatMessage("system", "Upgrade successful, rebooting...");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Brief pause to show message
                Reboot();
                return; // This line will never be reached after reboot
            }
        }

        // No new version, mark the current version as valid
        ota.MarkCurrentVersionValid();
        if (!ota.HasActivationCode() && !ota.HasActivationChallenge()) {
            xEventGroupSetBits(event_group_, MAIN_EVENT_CHECK_NEW_VERSION_DONE);
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota.HasActivationCode()) {
            ShowActivationCode(ota.GetActivationCode(), ota.GetActivationMessage());
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota.Activate();
            if (err == ESP_OK) {
                xEventGroupSetBits(event_group_, MAIN_EVENT_CHECK_NEW_VERSION_DONE);
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (device_state_ == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::ShowActivationCode(const std::string& code, const std::string& message) {
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', Lang::Sounds::P3_0},
        digit_sound{'1', Lang::Sounds::P3_1}, 
        digit_sound{'2', Lang::Sounds::P3_2},
        digit_sound{'3', Lang::Sounds::P3_3},
        digit_sound{'4', Lang::Sounds::P3_4},
        digit_sound{'5', Lang::Sounds::P3_5},
        digit_sound{'6', Lang::Sounds::P3_6},
        digit_sound{'7', Lang::Sounds::P3_7},
        digit_sound{'8', Lang::Sounds::P3_8},
        digit_sound{'9', Lang::Sounds::P3_9}
    }};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "happy", Lang::Sounds::P3_ACTIVATION);

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            audio_service_.PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion, const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert %s: %s [%s]", status, message, emotion);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (device_state_ == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::ToggleChatState() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (device_state_ == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }
    
    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(kListeningModeManualStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
            SetListeningMode(kListeningModeManualStop);
        });
    }
}

void Application::StopListening() {
    if (device_state_ == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    const std::array<int, 3> valid_states = {
        kDeviceStateListening,
        kDeviceStateSpeaking,
        kDeviceStateIdle,
    };
    // If not valid, do nothing
    if (std::find(valid_states.begin(), valid_states.end(), device_state_) == valid_states.end()) {
        return;
    }

    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

void Application::Start() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio service */
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    audio_service_.Start();

    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    audio_service_.SetCallbacks(callbacks);

    /* Start the clock timer to update the status bar */
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    /* Wait for the network to be re`ady */
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);

    // Check for new firmware version or get the MQTT broker address
    Ota ota;
    CheckNewVersion(ota);

    // Initialize the protocol
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    // Add MCP common tools before initializing the protocol
    McpServer::GetInstance().AddCommonTools();

    if (ota.HasMqttConfig()) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota.HasWebsocketConfig()) {
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    protocol_->OnNetworkError([this](const std::string& message) {
        last_error_message_ = message;
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });
    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        // Auto-promote to speaking if audio arrives unexpectedly
        if (device_state_ != kDeviceStateSpeaking && !web_control_panel_active_) {
            ESP_LOGW(TAG, "Incoming audio while state=%s; auto-promoting to speaking",
                     STATE_STRINGS[device_state_]);
            SetDeviceState(kDeviceStateSpeaking);
        }
        if (device_state_ == kDeviceStateSpeaking || web_control_panel_active_) {
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
        } else {
            ESP_LOGW(TAG, "Dropping audio packet (state=%s)", STATE_STRINGS[device_state_]);
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");

#ifdef CONFIG_BOARD_TYPE_HEYSANTA
            // Declare static variables outside of the if blocks so they're shared
            static std::chrono::steady_clock::time_point tts_start_time;
            static std::chrono::steady_clock::time_point last_bell_time;
            static std::chrono::steady_clock::time_point last_mcp_time;
            static const int BELL_COOLDOWN_MS = 10000; // 8 second cooldown between bells
            static const int MCP_SUPPRESS_MS = 3000;  // Suppress bell for 3 seconds after MCP activity
#endif

            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    
                    // If web control panel is active, force speaking state regardless of current state
                    if (web_control_panel_active_) {
                        SetDeviceState(kDeviceStateSpeaking);
                    } else if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });

#ifdef CONFIG_BOARD_TYPE_HEYSANTA
                // Always play bell and trigger shake (same behavior for web panel and normal conversation)
                
                auto now = std::chrono::steady_clock::now();
                auto time_since_last_bell = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_bell_time).count();
                auto time_since_mcp = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_mcp_time).count();
                
                // Record the start time of TTS
                tts_start_time = now;
                
                // Don't play bell if:
                // 1. Not enough time since last bell (cooldown)
                // 2. Recent MCP activity (likely MCP response)
                bool should_play_bell = (time_since_last_bell > BELL_COOLDOWN_MS) && 
                                    (time_since_mcp > MCP_SUPPRESS_MS || last_mcp_time.time_since_epoch().count() == 0);
                
                if (should_play_bell) {
                    ESP_LOGI(TAG, "TTS start - playing bell (bell cooldown: %d ms, MCP time: %d ms) [Web panel: %s]", 
                            (int)time_since_last_bell, (int)time_since_mcp, web_control_panel_active_ ? "active" : "inactive");
                    
                    Schedule([this]() {
                        ESP_LOGI(TAG, "Playing bell sound - device state: %s", STATE_STRINGS[device_state_]);
                        ESP_LOGI(TAG, "Playing P3_TAHU for HEYSANTA");
                        // audio_service_.PlaySound(Lang::Sounds::P3_TAHU);
                        ESP_LOGI(TAG, "P3_TAHU queued for HEYSANTA");
                    });
                    
                    last_bell_time = now;
                } else {
                    ESP_LOGI(TAG, "TTS start - skipping bell (bell cooldown: %d ms, MCP suppress: %d ms) [Web panel: %s]", 
                            (int)time_since_last_bell, (int)time_since_mcp, web_control_panel_active_ ? "active" : "inactive");
                }
#endif
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (device_state_ == kDeviceStateSpeaking) {

                        vTaskDelay(pdMS_TO_TICKS(500));
                        // Different behavior for web panel vs normal conversation
                        if (web_control_panel_active_) {
                            // Web panel: go to listening mode instead of idle
                            ESP_LOGI(TAG, "Web panel active - going to listening state after TTS");
                            SetDeviceState(kDeviceStateListening);
                        } else {
                            // Normal conversation: follow standard mode logic
                            if (listening_mode_ == kListeningModeManualStop) {
                                SetDeviceState(kDeviceStateIdle);
                            } else {
                                SetDeviceState(kDeviceStateListening);
                            }
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            // Update last STT time for timeout tracking
            last_stt_time_ = std::chrono::steady_clock::now();
            
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
#ifdef CONFIG_BOARD_TYPE_HEYSANTA
            // Update MCP activity timestamp whenever we receive MCP messages
            static std::chrono::steady_clock::time_point last_mcp_time;
            last_mcp_time = std::chrono::steady_clock::now();
            ESP_LOGI(TAG, "MCP activity detected, will suppress bells for next 3 seconds");
#endif
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() {
                        Reboot();
                    });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring, Lang::Sounds::P3_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            ESP_LOGI(TAG, "Received custom message: %s", cJSON_PrintUnformatted(root));
            if (cJSON_IsObject(payload)) {
                Schedule([this, display, payload_str = std::string(cJSON_PrintUnformatted(payload))]() {
                    display->SetChatMessage("system", payload_str.c_str());
                });
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });
    bool protocol_started = protocol_->Start();

    // Initialize BLE protocol for MCP communication
#if CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGI(TAG, "Initializing BLE protocol for MCP communication");
    ble_protocol_ = std::make_unique<BleProtocol>();
    
    ble_protocol_->OnCommand([this](const std::string& command) {
        ESP_LOGI(TAG, "BLE command received: %s", command.c_str());
        
        // Parse JSON command
        cJSON* json = cJSON_Parse(command.c_str());
        if (json) {
            cJSON* type = cJSON_GetObjectItem(json, "type");
            if (type && cJSON_IsString(type)) {
                if (strcmp(type->valuestring, "mcp") == 0) {
                    cJSON* payload = cJSON_GetObjectItem(json, "payload");
                    if (payload) {
                        // Convert payload to string to pass safely to scheduled function
                        char* payload_str = cJSON_PrintUnformatted(payload);
                        if (payload_str) {
                            std::string payload_string(payload_str);
                            cJSON_free(payload_str);
                            
                            // Process MCP command and send response back via BLE
                            Schedule([this, payload_string]() {
                                // Parse the payload string back to JSON
                                cJSON* payload_json = cJSON_Parse(payload_string.c_str());
                                if (payload_json) {
                                    ProcessBleMcpCommand(payload_json);
                                    cJSON_Delete(payload_json);
                                } else {
                                    ESP_LOGE(TAG, "Failed to parse BLE MCP payload: %s", payload_string.c_str());
                                }
                            });
                        }
                    }
                } else if (strcmp(type->valuestring, "text") == 0) {
                    // Look for text directly in the command (not in payload)
                    cJSON* text_obj = cJSON_GetObjectItem(json, "text");
                    if (text_obj && cJSON_IsString(text_obj)) {
                        std::string text_content(text_obj->valuestring);
                        ESP_LOGI(TAG, "BLE text command received: %s", text_content.c_str());
                        
                        // Process text-to-speech via BLE
                        Schedule([this, text_content]() {
                            ProcessBleTextCommand(text_content);
                        });
                    } else {
                        ESP_LOGW(TAG, "BLE text command missing text field");
                    }
                } else {
                    ESP_LOGW(TAG, "BLE: Unknown command type: %s, supported types: mcp, text", type->valuestring);
                }
            } else {
                ESP_LOGW(TAG, "BLE: Missing or invalid type field in command: %s", command.c_str());
            }
            cJSON_Delete(json);
        } else {
            ESP_LOGW(TAG, "BLE: Invalid JSON command: %s", command.c_str());
        }
    });
    
    // MODIFIED: Track BLE connection state for STT timeout
    ble_protocol_->OnConnectionState([this](bool connected) {
        ESP_LOGI(TAG, "BLE connection state changed: %s", connected ? "connected" : "disconnected");
        SetBleConnectionState(connected);
    });
    
    if (!ble_protocol_->Start()) {
        ESP_LOGE(TAG, "Failed to start BLE protocol");
        ble_protocol_.reset();
    } else {
        ESP_LOGI(TAG, "BLE protocol started successfully for MCP communication");
    }
#endif

    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota.HasServerTime();
    if (protocol_started) {
        std::string message = std::string(Lang::Strings::VERSION) + ota.GetCurrentVersion();
        display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
        // Play the success sound to indicate the device is ready
#ifdef CONFIG_BOARD_TYPE_HEYSANTA
        audio_service_.PlaySound(Lang::Sounds::P3_BALLS);
#else
        audio_service_.PlaySound(Lang::Sounds::P3_SUCCESS);
#endif
    }

    // Print heap stats
    SystemInfo::PrintHeapStats();
}

void Application::OnClockTimer() {
    clock_ticks_++;

    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar();

    // Check memory health first
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    
    // If memory is critically low, force cleanup
    if (free_heap < 50000) { // 50KB threshold
        ESP_LOGW(TAG, "Critical memory low: %d bytes, forcing cleanup", (int)free_heap);
        
        // Force close audio channel to free memory
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            ESP_LOGI(TAG, "Low memory: Force closing audio channel");
            protocol_->CloseAudioChannel();
        }
        
        // Force idle state
        SetDeviceState(kDeviceStateIdle);
        
        // Skip timeout logic when memory is low
        return;
    }

    // Check for STT timeout only if memory is healthy
    if (stt_timeout_enabled_ && !ble_connected_ &&
        (device_state_ == kDeviceStateListening || device_state_ == kDeviceStateIdle)) {
        
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_stt = std::chrono::duration_cast<std::chrono::seconds>(now - last_stt_time_).count();
        
        if (time_since_last_stt > STT_TIMEOUT_SECONDS && last_stt_time_.time_since_epoch().count() > 0) {
            ESP_LOGI(TAG, "STT timeout detected (%d seconds) with BLE disconnected", (int)time_since_last_stt);
            ESP_LOGI(TAG, "Current device state: %d, free memory: %d bytes", (int)device_state_, (int)free_heap);
            
            // Don't play sound if memory is low
            if (free_heap > 100000) {
                PlaySound(Lang::Sounds::P3_DEACTIVATE);
            } else {
                ESP_LOGW(TAG, "Skipping deactivate sound - low memory");
            }
            
            Schedule([this]() {
                if (protocol_ && protocol_->IsAudioChannelOpened()) {
                    ESP_LOGI(TAG, "STT timeout: Force closing audio channel");
                    protocol_->CloseAudioChannel();
                } else {
                    ESP_LOGI(TAG, "STT timeout: Channel already closed, forcing idle state");
                    SetDeviceState(kDeviceStateIdle);
                }
            });
            
            last_stt_time_ = std::chrono::steady_clock::time_point{};
        }
    }

    // Print memory stats every 10 seconds with more detail
    if (clock_ticks_ % 10 == 0) {
        ESP_LOGI(TAG, "Memory: free=%d, min_free=%d, device_state=%d", 
                 (int)free_heap, (int)min_free_heap, (int)device_state_);
        SystemInfo::PrintHeapStats();
    }
}

// Add a async task to MainLoop
void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
}

// The Main Event Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainEventLoop() {
    // Raise the priority of the main event loop to avoid being interrupted by background tasks (which has priority 2)
    vTaskPrioritySet(NULL, 3);

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, MAIN_EVENT_SCHEDULE |
            MAIN_EVENT_SEND_AUDIO |
            MAIN_EVENT_WAKE_WORD_DETECTED |
            MAIN_EVENT_VAD_CHANGE |
            MAIN_EVENT_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & MAIN_EVENT_ERROR) {
            SetDeviceState(kDeviceStateIdle);
            Alert(Lang::Strings::ERROR, last_error_message_.c_str(), "sad", Lang::Sounds::P3_EXCLAMATION);
        }

        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (!protocol_->SendAudio(std::move(packet))) {
                    break;
                }
            }
        }

        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            OnWakeWordDetected();
        }

        if (bits & MAIN_EVENT_VAD_CHANGE) {
            if (device_state_ == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

void Application::OnWakeWordDetected() {
    if (!protocol_) {
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        audio_service_.EncodeWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                audio_service_.EnableWakeWordDetection(true);
                return;
            }
        }

        auto wake_word = audio_service_.GetLastWakeWord();
        ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_USE_AFE_WAKE_WORD || CONFIG_USE_CUSTOM_WAKE_WORD
        // Encode and send the wake word data to the server
        while (auto packet = audio_service_.PopWakeWordPacket()) {
            protocol_->SendAudio(std::move(packet));
        }
        // Set the chat state to wake word detected
        protocol_->SendWakeWordDetected(wake_word);
        SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
#else
        SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        // Play the pop up sound to indicate the wake word is detected
        audio_service_.PlaySound(Lang::Sounds::P3_POPUP);
#endif
    } else if (device_state_ == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonWakeWordDetected);
    } else if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    
    clock_ticks_ = 0;
    auto previous_state = device_state_;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);

    // Send the state change event
    DeviceStateEventManager::GetInstance().PostStateChangeEvent(previous_state, state);

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(true);
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // Initialize STT timeout tracking when entering listening mode 
            // (only for normal mode when BLE is disconnected)
            if (!web_control_panel_active_ && !ble_connected_) {
                last_stt_time_ = std::chrono::steady_clock::now();
                ESP_LOGI(TAG, "Started STT timeout tracking for normal mode (BLE disconnected)");
            } else if (ble_connected_) {
                ESP_LOGI(TAG, "STT timeout disabled - BLE is connected");
            } else if (web_control_panel_active_) {
                ESP_LOGI(TAG, "STT timeout disabled - Web control panel is active");
            }

            // Make sure the audio processor is running
            if (!audio_service_.IsAudioProcessorRunning()) {
                // Send the start listening command
                protocol_->SendStartListening(listening_mode_);
                audio_service_.EnableVoiceProcessing(true);
                audio_service_.EnableWakeWordDetection(false);
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // Only AFE wake word can be detected in speaking mode
#if CONFIG_USE_AFE_WAKE_WORD
                audio_service_.EnableWakeWordDetection(true);
#else
                audio_service_.EnableWakeWordDetection(false);
#endif
            }
            audio_service_.ResetDecoder();
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
        Schedule([this, wake_word]() {
            if (protocol_) {
                protocol_->SendWakeWordDetected(wake_word); 
            }
        }); 
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {   
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (device_state_ != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

#if CONFIG_BT_NIMBLE_ENABLED
void Application::ProcessBleMcpCommand(const cJSON* payload) {
    ESP_LOGI(TAG, "Processing BLE MCP command");
    
    // Create a complete JSON-RPC message by adding the required fields
    cJSON* complete_message = cJSON_CreateObject();
    cJSON_AddStringToObject(complete_message, "jsonrpc", "2.0");
    
    // Copy method, id, and params from payload
    cJSON* method_obj = cJSON_GetObjectItem(payload, "method");
    cJSON* id_obj = cJSON_GetObjectItem(payload, "id");
    cJSON* params_obj = cJSON_GetObjectItem(payload, "params");
    
    if (method_obj) {
        cJSON_AddItemToObject(complete_message, "method", cJSON_Duplicate(method_obj, 1));
    }
    if (id_obj) {
        cJSON_AddItemToObject(complete_message, "id", cJSON_Duplicate(id_obj, 1));
    } else {
        cJSON_AddNumberToObject(complete_message, "id", 0); // Default ID if not provided
    }
    if (params_obj) {
        cJSON_AddItemToObject(complete_message, "params", cJSON_Duplicate(params_obj, 1));
    }
    
    // Mark BLE MCP as active so responses are routed back to BLE
    ble_mcp_active_ = true;
    
    // Let the MCP server handle the command - this will call SendMcpMessage with the response
    McpServer::GetInstance().ParseMessage(complete_message);
    
    // Clean up
    cJSON_Delete(complete_message);
    
    // Reset BLE MCP state after a short delay to allow response to be sent
    Schedule([this]() {
        ble_mcp_active_ = false;
    });
}

void Application::SendBleMcpResponse(const std::string& response) {
    if (ble_protocol_ && ble_protocol_->IsConnected()) {
        // Wrap the JSON-RPC response in the format expected by the web app
        cJSON* wrapper = cJSON_CreateObject();
        cJSON_AddStringToObject(wrapper, "type", "mcp_response");
        
        // Parse the response and add it as the payload
        cJSON* response_json = cJSON_Parse(response.c_str());
        if (response_json) {
            cJSON_AddItemToObject(wrapper, "payload", response_json);
        } else {
            // If parsing fails, add as string
            cJSON_AddStringToObject(wrapper, "payload", response.c_str());
        }
        
        char* wrapped_response = cJSON_PrintUnformatted(wrapper);
        if (wrapped_response) {
            ble_protocol_->SendResponse(std::string(wrapped_response));
            cJSON_free(wrapped_response);
        }
        cJSON_Delete(wrapper);
    }
}

void Application::ProcessBleTextCommand(const std::string& text) {
    ESP_LOGI(TAG, "🔵 === BLE ProcessBleTextCommand() CALLED ===");
    ESP_LOGI(TAG, "🔵 Input text: '%s' (length: %d)", text.c_str(), (int)text.length());

    ESP_LOGI(TAG, "DISABLED BLE TTS PROCESSING");
    SendBleTextResponse("error", "BLE TTS processing is disabled");
    return;
    
    if (text.empty()) {
        ESP_LOGW(TAG, "🔵 Text is empty, not processing");
        // Send empty response back to BLE client
        SendBleTextResponse("error", "Text is empty");
        return;
    }
    
    // URL decode the text (same as SpeakText function)
    std::string decoded_text;
    for (size_t i = 0; i < text.length(); i++) {
        if (text[i] == '+') {
            decoded_text += ' ';
        } else if (text[i] == '%' && i + 2 < text.length()) {
            // Convert hex to char
            std::string hex = text.substr(i + 1, 2);
            char decoded_char = (char)strtol(hex.c_str(), NULL, 16);
            decoded_text += decoded_char;
            i += 2; // Skip the two hex digits
        } else {
            decoded_text += text[i];
        }
    }
    
    ESP_LOGI(TAG, "🔵 Decoded text: '%s'", decoded_text.c_str());
    
    // Mark web control panel as active for TTS processing (like SpeakText does)
    // SetWebControlPanelActive(true);
    
    // Prepare the device for TTS playback (same as SpeakText function)
    Schedule([this]() {
        ESP_LOGI(TAG, "🔵 Preparing device for TTS playback via BLE");
        
        // Make sure audio channel is open
        if (!protocol_->IsAudioChannelOpened()) {
            ESP_LOGI(TAG, "🔵 Opening audio channel for BLE TTS");
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                ESP_LOGE(TAG, "🔵 Failed to open audio channel for BLE TTS");
                return;
            }
        }
        
        // Force speaking state
        ESP_LOGI(TAG, "🔵 Setting device to speaking state for BLE TTS");
        SetDeviceState(kDeviceStateSpeaking);
        
        ESP_LOGI(TAG, "🔵 Audio decoder reset for BLE TTS");
    });
    
    // Create MCP message for text-to-speech (same as SpeakText function)
    uint32_t message_id = esp_random() % 10000;
    ESP_LOGI(TAG, "🔵 Generated message ID for BLE TTS: %lu", (unsigned long)message_id);
    
    std::string mcp_message = "{\"jsonrpc\":\"2.0\",\"id\":" + 
                             std::to_string(message_id) + 
                             ",\"method\":\"tts/speak\",\"params\":{\"text\":\"" + 
                             decoded_text + "\",\"voice\":\"santa\"}}";
    
    ESP_LOGI(TAG, "🔵 Created MCP message for BLE TTS: %s", mcp_message.c_str());
    
    // Check if protocol is available
    if (!protocol_) {
        ESP_LOGE(TAG, "🔵 ERROR: protocol_ is NULL! Cannot send BLE TTS message");
        SendBleTextResponse("error", "Protocol not available");
        ble_mcp_active_ = false;
        return;
    }
    
    ESP_LOGI(TAG, "🔵 Protocol available, sending BLE TTS MCP message...");
    
    // Send the MCP message (this will trigger TTS and audio playback)
    protocol_->SendMcpMessage(mcp_message);
    
    // Send success response back to BLE client
    SendBleTextResponse("success", "Text-to-speech initiated");
    
    ESP_LOGI(TAG, "🔵 === BLE TTS MESSAGE SENT TO SERVER ===");
    ESP_LOGI(TAG, "🔵 ProcessBleTextCommand() completed successfully");
}

void Application::SendBleTextResponse(const std::string& status, const std::string& message) {
    if (ble_protocol_ && ble_protocol_->IsConnected()) {
        // Create response in expected format
        cJSON* wrapper = cJSON_CreateObject();
        cJSON_AddStringToObject(wrapper, "type", "text_response");
        
        cJSON* payload = cJSON_CreateObject();
        cJSON_AddStringToObject(payload, "status", status.c_str());
        cJSON_AddStringToObject(payload, "message", message.c_str());
        cJSON_AddItemToObject(wrapper, "payload", payload);
        
        char* response_str = cJSON_PrintUnformatted(wrapper);
        if (response_str) {
            ESP_LOGI(TAG, "🔵 Sending BLE text response: %s", response_str);
            ble_protocol_->SendResponse(std::string(response_str));
            cJSON_free(response_str);
        }
        cJSON_Delete(wrapper);
    }
}
#endif

void Application::SendMcpMessage(const std::string& payload) {
    Schedule([this, payload]() {
        // If BLE MCP is active, send response via BLE instead of main protocol
#if CONFIG_BT_NIMBLE_ENABLED
        if (ble_mcp_active_) {
            SendBleMcpResponse(payload);
        } else {
#endif
            // Send to main protocol (MQTT/WebSocket)
            if (protocol_) {
                protocol_->SendMcpMessage(payload);
            }
#if CONFIG_BT_NIMBLE_ENABLED
        }
#endif
    });
}

void Application::SendSystemCommand(const std::string& command) {
    Schedule([this, command]() {
        if (protocol_) {
            // Send system command wrapped in MCP format
            std::string mcp_payload = "{\"type\":\"system\",\"command\":\"" + command + "\"}";
            ESP_LOGI(TAG, "Sending system command via MCP: %s", mcp_payload.c_str());
            protocol_->SendMcpMessage(mcp_payload);
        } else {
            ESP_LOGW(TAG, "Protocol not available to send system command: %s", command.c_str());
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
        case kAecOff:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
            break;
        case kAecOnServerSide:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        case kAecOnDeviceSide:
            audio_service_.EnableDeviceAec(true);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::PlaySound(const std::string_view& sound) {
    audio_service_.PlaySound(sound);
}

// Add new method for BLE connection state tracking:
void Application::SetBleConnectionState(bool connected) {
    bool was_connected = ble_connected_;
    ble_connected_ = connected;
    
    ESP_LOGI(TAG, "BLE connection state: %s -> %s", 
             was_connected ? "connected" : "disconnected",
             connected ? "connected" : "disconnected");
    
    if (connected && !was_connected) {
        // BLE just connected - disable STT timeout
        ESP_LOGI(TAG, "BLE connected - STT timeout disabled");
        last_stt_time_ = std::chrono::steady_clock::time_point{}; // Reset timeout tracking
    } else if (!connected && was_connected) {
        // BLE just disconnected - enable STT timeout if in appropriate state
        if (!web_control_panel_active_ && 
            (device_state_ == kDeviceStateListening || device_state_ == kDeviceStateIdle)) {
            ESP_LOGI(TAG, "BLE disconnected - STT timeout enabled");
            last_stt_time_ = std::chrono::steady_clock::now(); // Start timeout tracking
        }
    }
}

// Add these methods for web control panel support
void Application::SetWebControlPanelActive(bool active) {
    // Force web control panel to always be false
    web_control_panel_active_ = false;
    ESP_LOGI(TAG, "Web control panel forced to inactive (always false)");
    
    // Don't execute any of the original logic since we're forcing it false
    return;
}

bool Application::IsWebControlPanelActive() const {
    return web_control_panel_active_;
}