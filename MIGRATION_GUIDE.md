# Migration Guide: Turtle → TurtleRefactor

This guide helps you transition from the monolithic Turtle code to the refactored modular driver.

## 📋 What Changed?

### File Structure

**Before (Turtle):**
```
Turtle/
└── main/
    └── hello_world_main.c    # Everything in one file (400+ lines)
```

**After (TurtleRefactor):**
```
TurtleRefactor/
└── main/
    ├── main.c                 # Clean application code
    ├── sts3032_protocol.h/c   # Protocol layer
    ├── sts3032_servo.h/c      # Servo control API
    ├── sts3032_config.h/c     # Configuration utilities
    └── sts3032_driver.h       # Single-include convenience header
```

## 🔄 Function Name Changes

All functions now have consistent prefixes based on their layer:

| Old Function | New Function | Module |
|-------------|-------------|---------|
| `angle_to_position()` | `sts_angle_to_position()` | servo |
| `position_to_angle()` | `sts_position_to_angle()` | servo |
| `sts_checksum()` | `sts_checksum()` | protocol |
| `sts_send_packet()` | `sts_send_packet()` | protocol |
| `sts_read_response()` | `sts_read_response()` | protocol |
| `sts_ping()` | `sts_servo_ping()` | servo |
| `sts_scan_bus()` | `sts_servo_scan_bus()` | servo |
| `sts_enable_torque()` | `sts_servo_enable_torque()` | servo |
| `sts_set_angle()` | `sts_servo_set_angle()` | servo |
| `sts_get_position()` | `sts_servo_get_position()` | servo |
| `sts_get_angle()` | `sts_servo_get_angle()` | servo |
| `sts_change_id()` | `sts_servo_change_id()` | servo |
| `sts_read_id()` | `sts_servo_read_id()` | servo |
| `sts_broadcast_reset_to_id_1()` | `sts_servo_broadcast_reset_id()` | servo |
| `id_configuration_wizard()` | `sts_config_wizard()` | config |

## 🚀 Migration Steps

### Step 1: Initialize Protocol Layer

**Old way (implicit):**
```c
void app_main(void) {
    // GPIO and UART setup was inline in app_main
    gpio_config_t io_conf = { /* ... */ };
    gpio_config(&io_conf);
    uart_config_t uart_config = { /* ... */ };
    uart_driver_install(/* ... */);
    // etc.
}
```

**New way (explicit):**
```c
#include "sts3032_driver.h"

void app_main(void) {
    // Clean initialization
    sts_protocol_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = GPIO_NUM_10,
        .rx_pin = GPIO_NUM_11,
        .txen_pin = GPIO_NUM_3,
        .baud_rate = 1000000,
    };
    
    sts_protocol_init(&config);
}
```

### Step 2: Update Function Calls

**Old way:**
```c
sts_enable_torque(1, true);
sts_set_angle(1, 90, SPEED_MEDIUM);

float angle;
sts_get_angle(1, &angle);
```

**New way:**
```c
sts_servo_enable_torque(1, true);
sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);

float angle;
sts_servo_get_angle(1, &angle);
```

### Step 3: Update Headers

**Old way:**
```c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
// All implementation in same file
```

**New way:**
```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sts3032_driver.h"  // Single include for entire driver
```

## 📝 Example Migration

### Before: Turtle/main/hello_world_main.c

```c
void app_main(void)
{
    // Hardware setup (40+ lines)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SERVO_TXEN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        // ...
    };
    gpio_config(&io_conf);
    
    uart_config_t uart_config = {
        .baud_rate = SERVO_BAUD_RATE,
        // ...
    };
    uart_driver_install(SERVO_UART_NUM, 2048, 2048, 0, NULL, 0);
    uart_param_config(SERVO_UART_NUM, &uart_config);
    uart_set_pin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Application code
    sts_scan_bus(1, 10);
    sts_enable_torque(1, true);
    sts_set_angle(1, 90, SPEED_MEDIUM);
}
```

### After: TurtleRefactor/main/main.c

```c
#include "sts3032_driver.h"

void app_main(void)
{
    // Clean initialization (5 lines!)
    sts_protocol_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = GPIO_NUM_10,
        .rx_pin = GPIO_NUM_11,
        .txen_pin = GPIO_NUM_3,
        .baud_rate = 1000000,
    };
    sts_protocol_init(&config);
    
    // Application code (identical intent, new names)
    sts_servo_scan_bus(1, 10);
    sts_servo_enable_torque(1, true);
    sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);
}
```

## 🎯 Quick Reference: Find & Replace

Use these patterns to quickly update your code:

```bash
# Function names
sts_ping           → sts_servo_ping
sts_scan_bus       → sts_servo_scan_bus
sts_enable_torque  → sts_servo_enable_torque
sts_set_angle      → sts_servo_set_angle
sts_get_angle      → sts_servo_get_angle
sts_get_position   → sts_servo_get_position
sts_change_id      → sts_servo_change_id
sts_read_id        → sts_servo_read_id

# Conversion functions
angle_to_position  → sts_angle_to_position
position_to_angle  → sts_position_to_angle

# Configuration
id_configuration_wizard()       → sts_config_wizard()
sts_broadcast_reset_to_id_1()  → sts_servo_broadcast_reset_id()
```

## ✅ Benefits of Refactored Code

1. **Modularity**: Each layer has a single responsibility
2. **Reusability**: Protocol and servo layers can be used in other projects
3. **Maintainability**: Easier to find and fix bugs
4. **Testability**: Can test each module independently
5. **Readability**: Clear separation between hardware, protocol, and application
6. **Scalability**: Easy to add new features to specific layers

## 🔧 Building the Refactored Version

```bash
cd TurtleRefactor
idf.py build
idf.py flash monitor
```

## 📦 Using in Other Projects

To use this driver in your robot project:

1. **Copy driver files:**
   ```bash
   cp TurtleRefactor/main/sts3032_*.{h,c} your_project/components/sts3032/
   ```

2. **Update your CMakeLists.txt:**
   ```cmake
   idf_component_register(
       SRCS "sts3032_protocol.c" "sts3032_servo.c"
       INCLUDE_DIRS "."
   )
   ```

3. **Include and use:**
   ```c
   #include "sts3032_driver.h"
   ```

## 🆘 Troubleshooting

### Issue: Build errors about missing functions

**Solution:** Make sure you're including `sts3032_driver.h` or the specific module headers you need.

### Issue: UART not working

**Solution:** Verify you called `sts_protocol_init()` before any servo operations.

### Issue: Function names not found

**Solution:** Update function calls to use new naming convention (see Quick Reference above).

## 💡 Tips

1. Start with the example modes in `main.c` to understand the new API
2. Use `sts3032_driver.h` for simple projects (single include)
3. For advanced use, include only the modules you need
4. The protocol layer is portable - you can use it with other servo types that use similar protocols

## 📚 Additional Resources

- [README.md](README.md) - Full API documentation
- [sts3032_servo.h](main/sts3032_servo.h) - Servo control API reference
- [sts3032_protocol.h](main/sts3032_protocol.h) - Protocol layer reference
- [main.c](main/main.c) - Example applications demonstrating all modes
