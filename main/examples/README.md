# STS3032 Servo Driver - Examples

This directory contains optional example code demonstrating various features and use cases of the STS3032 servo driver.

## 📁 Structure

```
examples/
├── README.md              # This file
├── example_modes.h        # Example function declarations
└── example_modes.c        # Example implementations
```

## 🚀 Basic Usage

The simplest way to use the driver is shown in `main.c`. For more advanced examples:

1. **Include the header** in your main.c:
   ```c
   #include "examples/example_modes.h"
   ```

2. **Enable the examples** in CMakeLists.txt:
   Uncomment this line:
   ```cmake
   # "examples/example_modes.c"
   ```

3. **Call the example** you want from app_main():
   ```c
   example_servo_configuration();  // or
   example_single_servo();         // or
   example_multi_servo();
   ```

## 📚 Available Examples

### 1. Servo Configuration (`example_servo_configuration`)
**When to use:** Setting up new servos or changing servo IDs

**Features:**
- Wizard mode for assigning specific IDs
- Auto-detection of all servos on the bus
- Manual ID changes

**Typical use case:** Initial setup of new servos

---

### 2. Single Servo Test (`example_single_servo`)
**When to use:** Testing basic servo functionality

**Features:**
- Scanning for servos
- Enabling torque
- Simple movement patterns
- Reading current position

**Typical use case:** Verifying servo connections and basic operation

---

### 3. Multi-Servo Control (`example_multi_servo`)
**When to use:** Controlling multiple servos simultaneously

**Features:**
- Managing multiple servo IDs
- Coordinated movement patterns
- Sequential positioning
- Wave motion effects

**Typical use case:** Robot projects with multiple servos (legs, arms, etc.)

## 🔧 Integration into Your Project

When moving the driver to your main project:

### What to Copy (Required):
```
sts3032_protocol.h/c  - Low-level UART communication
sts3032_servo.h/c     - Servo control API
sts3032_config.h/c    - Configuration utilities
```

### What to Reference (Optional):
```
examples/             - Example implementations
main.c                - Basic usage example
```

### Integration Steps:

1. **Copy driver files** to your project's components directory
2. **Use main.c as a template** for your application
3. **Reference examples/** when you need specific features
4. **Adapt hardware configuration** (pins, UART, etc.) to your hardware

## 💡 Tips

- Start with the basic example in `main.c`
- Use `example_servo_configuration()` first to set up servo IDs
- The examples are designed to be self-contained and educational
- You can copy and modify examples for your specific needs

## 📖 See Also

- `../main.c` - Minimal working example
- `../QUICK_REFERENCE.md` - API quick reference
- `../PROJECT_SUMMARY.md` - Project overview
