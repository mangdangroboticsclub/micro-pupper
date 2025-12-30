# STS3032 Servo Driver - Refactored

A clean, modular driver for STS3032 serial bus servos for ESP32-based robotics.

## 🎯 Project Structure

```
TurtleRefactor/
├── main/
│   ├── main.c                    # Main application with example modes
│   ├── sts3032_protocol.h/c      # Low-level protocol implementation
│   ├── sts3032_servo.h/c         # High-level servo control API
│   ├── sts3032_config.h/c        # Configuration wizards and utilities
│   └── CMakeLists.txt            # Build configuration
├── CMakeLists.txt                # Project configuration
└── sdkconfig                     # ESP-IDF SDK configuration
```

## 📦 Module Overview

### 1. Protocol Layer (`sts3032_protocol.h/c`)
Low-level communication with STS3032 servos.

**Key Functions:**
- `sts_protocol_init()` - Initialize UART and GPIO
- `sts_send_packet()` - Send command packet
- `sts_read_response()` - Read servo response
- `sts_write_register()` - Write to servo register
- `sts_read_register()` - Read from servo register

### 2. Servo Control Layer (`sts3032_servo.h/c`)
High-level servo control API.

**Key Functions:**
- `sts_servo_ping()` - Check if servo is online
- `sts_servo_scan_bus()` - Scan for all servos
- `sts_servo_enable_torque()` - Enable/disable torque
- `sts_servo_set_angle()` - Move to angle (0-360°)
- `sts_servo_get_angle()` - Read current angle
- `sts_servo_change_id()` - Change servo ID

**Speed Presets:**
```c
SPEED_VERY_SLOW = 50
SPEED_SLOW      = 150
SPEED_MEDIUM    = 500
SPEED_FAST      = 1500
SPEED_VERY_FAST = 3000
SPEED_MAX       = 4095
```

### 3. Configuration Layer (`sts3032_config.h/c`)
Setup wizards and utilities.

**Key Functions:**
- `sts_config_wizard()` - Interactive ID configuration
- `sts_config_auto_detect()` - Auto-detect servos on bus

## 🚀 Quick Start

### Hardware Setup
Connect your STS3032 servos:
- **TX**: GPIO 10
- **RX**: GPIO 11  
- **TXEN**: GPIO 3
- **Baud Rate**: 1000000

### Usage Example

```c
#include "sts3032_protocol.h"
#include "sts3032_servo.h"

void app_main(void) {
    // 1. Initialize protocol
    sts_protocol_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = GPIO_NUM_10,
        .rx_pin = GPIO_NUM_11,
        .txen_pin = GPIO_NUM_3,
        .baud_rate = 1000000,
    };
    sts_protocol_init(&config);
    
    // 2. Enable servo
    sts_servo_enable_torque(1, true);
    
    // 3. Move servo
    sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);
    
    // 4. Read position
    float angle;
    if (sts_servo_get_angle(1, &angle)) {
        printf("Current angle: %.1f°\n", angle);
    }
}
```

## 🛠️ Configuration Modes

The refactored `main.c` includes three ready-to-use modes:

### Mode 1: ID Configuration
Use this when setting up new servos or changing IDs.

```c
mode_id_configuration();
```

**Steps:**
1. Disconnect all servos except ONE
2. Power on ESP32
3. Servo will be assigned the specified ID
4. Repeat for each servo with a different ID

### Mode 2: Testing Mode
Test single servo movements.

```c
mode_testing();
```

### Mode 3: Multi-Servo Demo
Control multiple servos simultaneously.

```c
mode_multi_servo_demo();
```

## 🔧 Building

```bash
cd TurtleRefactor
idf.py build
idf.py flash
idf.py monitor
```

## 📊 API Reference

### Position Control

| Function | Description |
|----------|-------------|
| `sts_servo_set_angle(id, angle, speed)` | Move to angle (0-360°) |
| `sts_servo_set_position(id, pos, speed)` | Move to raw position (0-4095) |
| `sts_servo_get_angle(id, *angle)` | Read current angle |
| `sts_servo_get_position(id, *pos)` | Read current position |

### Servo Management

| Function | Description |
|----------|-------------|
| `sts_servo_ping(id)` | Check if servo is online |
| `sts_servo_scan_bus(start, end)` | Scan ID range for servos |
| `sts_servo_enable_torque(id, enable)` | Enable/disable torque |

### ID Configuration

| Function | Description |
|----------|-------------|
| `sts_servo_change_id(old_id, new_id)` | Change servo ID (⚠️ EEPROM write) |
| `sts_servo_read_id(id, *current_id)` | Read servo ID |
| `sts_servo_broadcast_reset_id()` | Reset all servos to ID 1 |

## 🏗️ Integrating into Your Robot

### Step 1: Copy Driver Files
Copy these files to your robot project:
```
sts3032_protocol.h
sts3032_protocol.c
sts3032_servo.h
sts3032_servo.c
```

### Step 2: Add to CMakeLists.txt
```cmake
idf_component_register(
    SRCS 
        "your_main.c"
        "sts3032_protocol.c"
        "sts3032_servo.c"
    INCLUDE_DIRS "."
)
```

### Step 3: Use in Your Code
```c
#include "sts3032_protocol.h"
#include "sts3032_servo.h"

// Initialize once at startup
sts_protocol_config_t config = { /* ... */ };
sts_protocol_init(&config);

// Control servos anywhere in your code
sts_servo_set_angle(1, 90.0, SPEED_FAST);
```

## 🎓 Comparison: Before vs After

### Before (Monolithic)
- ❌ 400+ lines in single file
- ❌ Protocol mixed with application logic
- ❌ Hard to reuse in other projects
- ❌ Difficult to test individual components

### After (Modular)
- ✅ Clean separation of concerns
- ✅ Reusable protocol layer
- ✅ Easy-to-use high-level API
- ✅ Example modes for different use cases
- ✅ Well-documented and maintainable

## 📝 Notes

### EEPROM Operations
Functions that modify servo ID write to EEPROM:
- `sts_servo_change_id()`
- `sts_servo_broadcast_reset_id()`

**⚠️ Important:** Only one servo should be on the bus when changing IDs!

### Multi-Servo Setup
1. Start with all servos at factory default (ID 1)
2. Connect servos one at a time
3. Assign unique IDs: 1, 2, 3, etc.
4. Once configured, connect all servos together

## 🤝 Contributing

This driver was refactored from a monolithic codebase to support the MicroPupper robot project. Improvements welcome!

## 📄 License

Check the main project repository for license information.
