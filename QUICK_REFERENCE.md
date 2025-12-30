# STS3032 Driver - Quick Reference Card

## 🚀 Initialization

```c
#include "sts3032_driver.h"

sts_protocol_config_t config = {
    .uart_num = UART_NUM_1,
    .tx_pin = GPIO_NUM_10,
    .rx_pin = GPIO_NUM_11,
    .txen_pin = GPIO_NUM_3,
    .baud_rate = 1000000,
};
sts_protocol_init(&config);
```

## 🎯 Common Operations

### Enable Servo
```c
sts_servo_enable_torque(1, true);   // Turn on
sts_servo_enable_torque(1, false);  // Turn off
```

### Move to Position
```c
// By angle (0-360°)
sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);

// By raw position (0-4095)
sts_servo_set_position(1, 2047, SPEED_MEDIUM);
```

### Read Position
```c
// Get angle
float angle;
if (sts_servo_get_angle(1, &angle)) {
    printf("Angle: %.1f°\n", angle);
}

// Get raw position
uint16_t pos;
if (sts_servo_get_position(1, &pos)) {
    printf("Position: %d\n", pos);
}
```

### Check Servo Status
```c
// Ping single servo
if (sts_servo_ping(1)) {
    printf("Servo 1 is online\n");
}

// Scan bus
int count = sts_servo_scan_bus(1, 10);
printf("Found %d servos\n", count);
```

## ⚙️ Configuration

### Change Servo ID
```c
// ⚠️ Only ONE servo on bus!
sts_servo_change_id(1, 2);  // Change from 1 to 2
```

### Configuration Wizard
```c
sts_config_wizard(2);  // Assign ID 2 to connected servo
```

### Auto-Detect Servos
```c
sts_config_auto_detect();  // Scan and report
```

## 📊 Speed Presets

```c
SPEED_VERY_SLOW = 50
SPEED_SLOW      = 150
SPEED_MEDIUM    = 500      // ← Recommended default
SPEED_FAST      = 1500
SPEED_VERY_FAST = 3000
SPEED_MAX       = 4095

// Usage
sts_servo_set_angle(1, 90.0, SPEED_FAST);
```

## 🔄 Conversion Functions

```c
// Angle to position
uint16_t pos = sts_angle_to_position(90.0);  // 90° → 2047

// Position to angle
float angle = sts_position_to_angle(2047);   // 2047 → 90.0°
```

## 🎯 Multi-Servo Control

```c
uint8_t servos[] = {1, 2, 3};

// Enable all
for (int i = 0; i < 3; i++) {
    sts_servo_enable_torque(servos[i], true);
}

// Move all to different positions
sts_servo_set_angle(1, 45.0, SPEED_MEDIUM);
sts_servo_set_angle(2, 90.0, SPEED_MEDIUM);
sts_servo_set_angle(3, 135.0, SPEED_MEDIUM);

// Read all positions
for (int i = 0; i < 3; i++) {
    float angle;
    if (sts_servo_get_angle(servos[i], &angle)) {
        printf("Servo %d: %.1f°\n", servos[i], angle);
    }
}
```

## 🔌 Hardware Connections

```
ESP32          STS3032 Servo
─────          ────────────
GPIO 10  ────→ TX
GPIO 11  ←──── RX
GPIO 3   ────→ TXEN
GND      ────→ GND
5V       ────→ VCC (or external power)
```

**⚠️ Power Note:** Servos draw significant current. Use external power supply for multiple servos or high loads.

## 🐛 Debugging

### Check Communication
```c
// Basic ping test
if (!sts_servo_ping(1)) {
    ESP_LOGE(TAG, "Servo not responding!");
    // Check: wiring, power, baud rate, servo ID
}
```

### Scan for Lost Servos
```c
// Scan wider range
sts_servo_scan_bus(1, 253);  // Check all possible IDs
```

### Reset Lost ID
```c
// ⚠️ EMERGENCY: Reset ALL servos to ID 1
// Connect only ONE servo before using!
sts_servo_broadcast_reset_id();
```

## 📋 Error Handling

```c
// Always check return values
if (!sts_servo_get_angle(1, &angle)) {
    ESP_LOGE(TAG, "Failed to read angle");
    // Possible causes:
    // - Servo offline
    // - Bad communication
    // - Wrong ID
}
```

## 🎨 Example Patterns

### Smooth Movement Loop
```c
while (1) {
    sts_servo_set_angle(1, 45.0, SPEED_SLOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    sts_servo_set_angle(1, 135.0, SPEED_SLOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
}
```

### Position Feedback Control
```c
float target = 90.0;
float current;

while (1) {
    if (sts_servo_get_angle(1, &current)) {
        float error = target - current;
        
        if (fabs(error) > 1.0) {
            printf("Moving... error: %.1f°\n", error);
            sts_servo_set_angle(1, target, SPEED_MEDIUM);
        } else {
            printf("Target reached!\n");
            break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

### Timed Sequence
```c
typedef struct {
    uint8_t id;
    float angle;
    uint16_t speed;
    uint32_t delay_ms;
} servo_command_t;

servo_command_t sequence[] = {
    {1, 90.0, SPEED_MEDIUM, 1000},
    {2, 45.0, SPEED_FAST, 500},
    {3, 135.0, SPEED_MEDIUM, 1000},
    {1, 45.0, SPEED_SLOW, 2000},
};

for (int i = 0; i < 4; i++) {
    sts_servo_set_angle(
        sequence[i].id,
        sequence[i].angle,
        sequence[i].speed
    );
    vTaskDelay(pdMS_TO_TICKS(sequence[i].delay_ms));
}
```

## 🔐 Protocol Constants

```c
// Commands
STS_PING            0x01
STS_READ            0x02
STS_WRITE           0x03

// Special IDs
STS_BROADCAST_ID    0xFE  // Affects ALL servos

// Key Registers
STS_ID              0x05  // Servo ID (EEPROM)
STS_TORQUE_ENABLE   0x28  // Torque on/off
STS_GOAL_POSITION_L 0x2A  // Target position
STS_GOAL_SPEED_L    0x2E  // Movement speed
STS_PRESENT_POSITION_L 0x38  // Current position
```

## 💡 Pro Tips

1. **Always enable torque** before moving servos
2. **Wait for movement** with `vTaskDelay()` between commands
3. **Use SPEED_MEDIUM** as default - balance of speed and smoothness
4. **Read positions** to verify movement completion
5. **One servo at a time** when changing IDs
6. **External power supply** for multiple servos
7. **Check return values** to catch communication errors

## 📚 More Information

- [README.md](README.md) - Complete documentation
- [ARCHITECTURE.md](ARCHITECTURE.md) - System design
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - Upgrading from old code

## ⚡ One-Liners

```c
// Quick initialization (copy-paste ready)
sts_protocol_config_t cfg = {UART_NUM_1, GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_3, 1000000};
sts_protocol_init(&cfg);

// Quick servo test
sts_servo_enable_torque(1, true); sts_servo_set_angle(1, 90, SPEED_MEDIUM);

// Quick scan
sts_servo_scan_bus(1, 10);

// Quick ID change (one servo only!)
sts_config_wizard(2);
```

---

**Need help?** Check the full documentation in [README.md](README.md)
