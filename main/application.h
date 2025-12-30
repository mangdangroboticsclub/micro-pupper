#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <deque>
#include <vector>
#include <memory>
#include <chrono>  // ADD THIS LINE for std::chrono

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "device_state_event.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "protocols/ble_protocol.h"
#endif

#define MAIN_EVENT_SCHEDULE (1 << 0)
#define MAIN_EVENT_SEND_AUDIO (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED (1 << 2)
#define MAIN_EVENT_VAD_CHANGE (1 << 3)
#define MAIN_EVENT_ERROR (1 << 4)
#define MAIN_EVENT_CHECK_NEW_VERSION_DONE (1 << 5)

enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void Start();
    void MainEventLoop();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return audio_service_.IsVoiceDetected(); }
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void SpeakText(const std::string& text);
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    void SetWebControlPanelActive(bool active);
    bool IsWebControlPanelActive() const;
    void PlaySound(const std::string_view& sound);
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SendSystemCommand(const std::string& command);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    AudioService& GetAudioService() { return audio_service_; }
    
    // ADD THESE TWO METHODS for BLE connection state tracking
    void SetBleConnectionState(bool connected);
    bool IsBleConnected() const { return ble_connected_; }

private:
    Application();
    ~Application();

    std::chrono::steady_clock::time_point last_stt_time_;
    bool stt_timeout_enabled_ = true;
    static const int STT_TIMEOUT_SECONDS = 95; // 1 minute timeout for normal mode
    std::mutex mutex_;
    std::deque<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    volatile DeviceState device_state_ = kDeviceStateUnknown;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;
    std::string last_error_message_;
    AudioService audio_service_;
#if CONFIG_BT_NIMBLE_ENABLED
    std::unique_ptr<BleProtocol> ble_protocol_;
#endif

    bool web_control_panel_active_ = false;
    bool ble_mcp_active_ = false;
    bool ble_connected_ = false;  // ADD THIS LINE for BLE connection state
    bool has_server_time_ = false;
    bool aborted_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;

    void OnWakeWordDetected();
    void CheckNewVersion(Ota& ota);
    void ShowActivationCode(const std::string& code, const std::string& message);
    void OnClockTimer();
    void SetListeningMode(ListeningMode mode);
    void ProcessBleMcpCommand(const cJSON* payload);
    void ProcessBlePluginCommand(const cJSON* payload);
    void ProcessBleTextCommand(const std::string& text);
    void SendBleMcpResponse(const std::string& response);
    void SendBleTextResponse(const std::string& status, const std::string& message);
};

#endif // _APPLICATION_H_