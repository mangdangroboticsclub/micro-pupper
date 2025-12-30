# STS3032 Driver Architecture

## 📐 Layer Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                     │
│                       (main.c)                          │
│  - Mode selection (ID config, testing, multi-servo)    │
│  - User application logic                              │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              CONFIGURATION UTILITIES                     │
│              (sts3032_config.h/c)                       │
│  - ID configuration wizard                             │
│  - Auto-detection utilities                            │
│  - Setup helpers                                       │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                SERVO CONTROL API                        │
│               (sts3032_servo.h/c)                       │
│  - High-level position control                         │
│  - Angle/position conversion                           │
│  - Torque management                                   │
│  - Bus scanning                                        │
│  - ID management                                       │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                 PROTOCOL LAYER                          │
│              (sts3032_protocol.h/c)                     │
│  - Packet construction/parsing                         │
│  - Checksum calculation                                │
│  - Register read/write                                 │
│  - UART communication                                  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                 HARDWARE LAYER                          │
│              (ESP32 + STS3032 Servos)                   │
│  - UART (TX/RX/TXEN)                                   │
│  - RS485 Half-Duplex                                   │
│  - Servo Bus                                           │
└─────────────────────────────────────────────────────────┘
```

## 🔄 Data Flow

### Writing to Servo (e.g., Set Angle)

```
main.c
  │
  │ sts_servo_set_angle(id=1, angle=90, speed=500)
  ▼
sts3032_servo.c
  │ • Convert angle to position (90° → 2047)
  │ • Package position + speed data
  │ • Call sts_write_register()
  ▼
sts3032_protocol.c
  │ • Construct STS packet
  │ • Calculate checksum
  │ • Enable TX (TXEN pin HIGH)
  │ • Send via UART
  │ • Disable TX (TXEN pin LOW)
  ▼
UART Hardware
  │
  ▼
Servo Motor
```

### Reading from Servo (e.g., Get Angle)

```
main.c
  │
  │ sts_servo_get_angle(id=1, &angle)
  ▼
sts3032_servo.c
  │ • Call sts_read_register()
  │ • Convert position to angle
  ▼
sts3032_protocol.c
  │ • Send READ command packet
  │ • Wait for response
  │ • Validate checksum
  │ • Extract data
  ▼
UART Hardware
  │
  ▼
Return angle value to main.c
```

## 🎯 Module Responsibilities

### Protocol Layer
**Purpose:** Handle low-level communication  
**Knows about:** UART, packets, checksums, registers  
**Doesn't know about:** Angles, servo concepts, application logic

```c
// Protocol layer only deals with raw bytes
sts_write_register(id, STS_GOAL_POSITION_L, data, 6);
sts_read_register(id, STS_PRESENT_POSITION_L, 2, buffer);
```

### Servo Control Layer
**Purpose:** Provide servo-specific abstractions  
**Knows about:** Angles, positions, speeds, servo behavior  
**Doesn't know about:** UART details, packet structure

```c
// Servo layer provides meaningful abstractions
sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);
sts_servo_enable_torque(1, true);
```

### Configuration Layer
**Purpose:** Setup and utilities  
**Knows about:** ID assignment workflows, detection  
**Doesn't know about:** Protocol details

```c
// Config layer provides guided workflows
sts_config_wizard(2);  // Assign ID 2 to connected servo
```

### Application Layer
**Purpose:** User application logic  
**Knows about:** Application requirements, control flow  
**Doesn't know about:** Low-level protocol or hardware

```c
// Application just calls high-level APIs
mode_multi_servo_demo();
```

## 📦 Dependency Graph

```
main.c
  ├── sts3032_config.c
  │     └── sts3032_servo.c
  │           └── sts3032_protocol.c
  │
  └── sts3032_servo.c
        └── sts3032_protocol.c
```

**Key principle:** Higher layers depend on lower layers, never the reverse.

## 🔌 Hardware Abstraction

The driver abstracts hardware details through configuration:

```c
sts_protocol_config_t config = {
    .uart_num = UART_NUM_1,      // Which UART peripheral
    .tx_pin = GPIO_NUM_10,       // TX pin number
    .rx_pin = GPIO_NUM_11,       // RX pin number
    .txen_pin = GPIO_NUM_3,      // Transmit enable (RS485)
    .baud_rate = 1000000,        // Communication speed
};
```

This makes it easy to:
- Change pins without modifying driver code
- Use different UART peripherals
- Port to different ESP32 variants
- Support different hardware configurations

## 🎨 Design Patterns Used

### 1. Layered Architecture
Each layer has a specific responsibility and only interacts with adjacent layers.

### 2. Hardware Abstraction
Protocol layer hides UART/GPIO details behind a clean interface.

### 3. API Facade
High-level functions hide complex multi-step operations:
```c
// One function call...
sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);

// ...handles multiple steps:
// 1. Convert angle to position
// 2. Package with speed parameters
// 3. Write to multiple registers
// 4. Handle RS485 direction control
```

### 4. Namespace Prefixing
Function names indicate their module:
- `sts_protocol_*` - Protocol layer
- `sts_servo_*` - Servo control layer
- `sts_config_*` - Configuration utilities

## 🚀 Extending the Driver

### Adding a New Servo Function

**Step 1:** Add to servo layer (`sts3032_servo.h`):
```c
bool sts_servo_get_temperature(uint8_t id, uint8_t *temp);
```

**Step 2:** Implement using protocol layer (`sts3032_servo.c`):
```c
bool sts_servo_get_temperature(uint8_t id, uint8_t *temp) {
    uint8_t data;
    if (sts_read_register(id, STS_PRESENT_TEMPERATURE, 1, &data)) {
        if (temp) *temp = data;
        return true;
    }
    return false;
}
```

**Step 3:** Use in application:
```c
uint8_t temp;
if (sts_servo_get_temperature(1, &temp)) {
    printf("Servo temperature: %d°C\n", temp);
}
```

### Adding Support for a New Servo Model

Create a new servo layer while reusing the protocol layer:

```
sts3032_protocol.c  ← Shared
     ├── sts3032_servo.c     (for STS3032)
     └── scs0009_servo.c     (for SCS0009 - different servo)
```

## 📊 Function Call Hierarchy

```
Application Functions:
  ├── sts_config_wizard()
  │     ├── sts_servo_ping()
  │     │     ├── sts_send_packet()
  │     │     └── sts_read_response()
  │     └── sts_servo_change_id()
  │           ├── sts_write_register()
  │           │     └── sts_send_packet()
  │           └── sts_servo_ping()
  │
  ├── sts_servo_set_angle()
  │     ├── sts_angle_to_position()
  │     └── sts_write_register()
  │           └── sts_send_packet()
  │
  └── sts_servo_get_angle()
        ├── sts_servo_get_position()
        │     └── sts_read_register()
        │           ├── sts_send_packet()
        │           └── sts_read_response()
        └── sts_position_to_angle()
```

## 🎓 Benefits Summary

1. **Separation of Concerns**: Each module has one job
2. **Reusability**: Protocol layer works with any STS servo
3. **Testability**: Can test each layer independently
4. **Maintainability**: Easy to find and fix bugs
5. **Extensibility**: Easy to add new features
6. **Portability**: Easy to port to different hardware
7. **Readability**: Clear structure and naming

## 🔍 File Size Comparison

| Module | Lines | Purpose |
|--------|-------|---------|
| **Old (Monolithic)** | **~400** | **Everything** |
| sts3032_protocol.c | ~150 | Protocol implementation |
| sts3032_servo.c | ~180 | Servo control API |
| sts3032_config.c | ~90 | Configuration utilities |
| main.c | ~120 | Clean application examples |
| **Total** | **~540** | **Same functionality, better organized** |

The refactored version is slightly more code due to headers and documentation, but each file is focused and maintainable.
