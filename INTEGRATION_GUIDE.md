# STS3032 Servo Driver - Project Structure

This document explains how the project is organized to facilitate easy integration into your main application.

## 📁 Directory Structure

```
main/
├── main.c                    # ✨ Basic example (start here)
├── CMakeLists.txt           # Build configuration
│
├── sts3032_protocol.h/c     # 🔧 Driver: UART communication layer
├── sts3032_servo.h/c        # 🔧 Driver: Servo control API
├── sts3032_config.h/c       # 🔧 Driver: Configuration utilities
│
└── examples/                # 📚 Optional advanced examples
    ├── README.md
    ├── example_modes.h
    └── example_modes.c
```

## 🎯 What Goes Where

### Driver Core (Required for your project)
These files contain the reusable driver code:
- `sts3032_protocol.h/c` - Low-level UART communication
- `sts3032_servo.h/c` - High-level servo control API  
- `sts3032_config.h/c` - Servo configuration utilities

**→ Copy these to your main project's components directory**

### Example Code (Reference only)
These files show you HOW to use the driver:
- `main.c` - Minimal working example (basic initialization and control)
- `examples/` - Additional examples (multi-servo, configuration, etc.)

**→ Use these as templates for your application code**

## 🚀 Quick Start

### For Testing This Project:
1. Review `main.c` to see the basic usage
2. Flash and run: The example will move servo ID 1
3. If you need to configure servo IDs, see `examples/README.md`

### For Integration into Your Project:

**Step 1:** Copy the driver files
```bash
# Copy to your project (adjust paths as needed)
cp sts3032_protocol.* your_project/components/sts3032_driver/
cp sts3032_servo.* your_project/components/sts3032_driver/
cp sts3032_config.* your_project/components/sts3032_driver/
```

**Step 2:** Use main.c as a starting template
```c
// In your application code:
#include "sts3032_protocol.h"
#include "sts3032_servo.h"

void app_main(void) {
    // 1. Initialize (adjust pins for your hardware)
    sts_protocol_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = GPIO_NUM_10,
        .rx_pin = GPIO_NUM_11,
        .txen_pin = GPIO_NUM_3,
        .baud_rate = 1000000,
    };
    sts_protocol_init(&config);
    
    // 2. Control servos
    sts_servo_ping(1);
    sts_servo_enable_torque(1, true);
    sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);
}
```

**Step 3:** Reference examples as needed
- Configuration: See `examples/example_modes.c` → `example_servo_configuration()`
- Multi-servo: See `examples/example_modes.c` → `example_multi_servo()`

## 📝 Key Files Explained

### main.c
**Purpose:** Clean, minimal example showing basic driver usage  
**Audience:** Developers integrating the driver  
**Contents:**
- Hardware pin configuration
- Driver initialization
- Basic servo control (ping, enable, move)
- Minimal dependencies

**This is your starting point!**

### examples/example_modes.c
**Purpose:** Advanced usage patterns and demos  
**Audience:** Developers learning advanced features  
**Contents:**
- Servo ID configuration wizard
- Multi-servo coordination
- Movement patterns
- Best practices

**Reference these when you need specific features.**

## 🔄 Migration Path

When moving from this example project to your main application:

1. ✅ **Keep:** Driver files (`sts3032_*.h/c`)
2. 📝 **Reference:** Example files (`main.c`, `examples/`)
3. ⚙️ **Adapt:** Hardware configuration (pins, UART)
4. 🏗️ **Build:** Your application using the driver API

## 📚 Documentation

- **API Reference:** See `QUICK_REFERENCE.md`
- **Architecture:** See `ARCHITECTURE.md`
- **Examples Guide:** See `examples/README.md`
- **Project Overview:** See `PROJECT_SUMMARY.md`

## 💡 Design Philosophy

**Separation of Concerns:**
- **Driver code** = Reusable, hardware-agnostic, well-tested
- **Example code** = Demonstrative, educational, easily adaptable

This structure makes it easy to:
- ✨ Copy driver code to your project
- 📖 Learn from clear examples
- 🔧 Customize for your hardware
- 🚀 Get started quickly

## 🤔 FAQ

**Q: Which files do I need in my main project?**  
A: Only the driver files: `sts3032_protocol.*`, `sts3032_servo.*`, `sts3032_config.*`

**Q: What about main.c?**  
A: It's an example. Use it as a template for your own application code.

**Q: Do I need the examples directory?**  
A: No, but it's useful as a reference for advanced features.

**Q: How do I change the pins?**  
A: Update the `#define` values in your application (see main.c lines 26-30)

**Q: Can I use this with multiple ESP32 projects?**  
A: Yes! Copy the driver files to a shared component directory, then include them in multiple projects.
