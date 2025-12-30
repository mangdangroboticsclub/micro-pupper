#ifndef _BLE_PROTOCOL_H
#define _BLE_PROTOCOL_H

#include "protocol.h"
#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include "esp_err.h"

// Santa-Bot BLE Protocol UUIDs
// Service UUID: 0d9be2a0-4757-43d9-83df-704ae274b8df
// Characteristic UUID: 8116d8c0-d45d-4fdf-998e-33ab8c471d59
#define SANTA_BOT_SERVICE_UUID_128 \
    0xdf, 0xb8, 0x74, 0xe2, 0x4a, 0x70, 0xdf, 0x83, \
    0xd9, 0x43, 0x57, 0x47, 0xa0, 0xe2, 0x9b, 0x0d

#define SANTA_BOT_CHARACTERISTIC_UUID_128 \
    0x59, 0x1d, 0x47, 0x8c, 0xab, 0x33, 0x8e, 0x99, \
    0xdf, 0x4f, 0x5d, 0xd4, 0xc0, 0xd8, 0x16, 0x81

class BleProtocol : public Protocol {
public:
    using CommandCallback = std::function<void(const std::string&)>;
    using ConnectionStateCallback = std::function<void(bool)>;

    BleProtocol();
    ~BleProtocol();

    // Protocol interface implementation
    bool Start() override;
    void Stop();
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel() override;
    bool IsAudioChannelOpened() const override;

    // BLE specific methods
    bool IsConnected() const;
    
    // Register BLE specific callbacks (in addition to Protocol callbacks)
    void OnCommand(CommandCallback callback);
    void OnConnectionState(ConnectionStateCallback callback);

    // Send response back to browser (supports chunking for large payloads)
    bool SendResponse(const std::string& response);
    
    // Handle internal command (for MCP tool execution via BLE thread)
    bool HandleInternalCommand(const std::string& command);
    
    // Get the device name from configuration
    static const char* GetDeviceName();
    
    // Helper methods for processing commands (called from static callbacks)
    void ProcessJsonCommand(const cJSON* json);
    void ProcessTextCommand(const std::string& text);

protected:
    bool SendText(const std::string& text) override;

private:
    void* impl_;
    std::atomic<bool> audio_channel_opened_{false};
    CommandCallback command_callback_;
    ConnectionStateCallback connection_callback_;
    
    // Send large response in chunks
    bool SendChunkedResponse(const std::string& response);
    
    // Constants for chunking
    static const size_t MAX_CHUNK_SIZE = 120; // Conservative size accounting for JSON wrapper + BLE MTU limits
};

#endif // _BLE_PROTOCOL_H