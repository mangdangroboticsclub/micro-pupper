# STS3032 Component

This component provides a protocol layer and high-level API to control STS3032 serial-bus servos.

Usage
-----

- Add the `components/sts3032/` folder to your project root (already in this repo). ESP-IDF auto-discovers components placed under `components/`.
- In your application source (for example `main/app_main.c`) include the public header:

```c
#include "sts3032_driver.h"   // or include specific headers: sts3032_servo.h
```

Initialization
--------------

Provide a `sts_protocol_config_t` with the UART and GPIO pins you want the component to use, then call `sts_protocol_init()` before using servo functions. Example:

```c
sts_protocol_config_t cfg = {
    .uart_num = UART_NUM_1,
    .tx_pin = GPIO_NUM_10,
    .rx_pin = GPIO_NUM_11,
    .txen_pin = GPIO_NUM_3,   // RS-485 / TX enable pin (set to -1 if unused)
    .baud_rate = 1000000,
};

ESP_ERROR_CHECK(sts_protocol_init(&cfg));

// now you can use high-level APIs
sts_servo_enable_torque(1, true);
sts_servo_set_angle(1, 90.0, SPEED_MEDIUM);
```

Notes and recommendations
-------------------------
- This component exposes two parts:
  - `include/` — public headers (`sts3032_protocol.h`, `sts3032_servo.h`, `sts3032_driver.h`)
  - `src/` — implementation files (`sts3032_protocol.c`, `sts3032_servo.c`)
- GPIO pins and UART selection are provided at runtime through `sts_protocol_config_t`. This is flexible and preferred for test/dev.
- EEPROM writes (for example `sts_servo_change_id`) modify servo non-volatile memory. Use caution and connect only a single servo when changing IDs.
- If you want build-time defaults or options (e.g., default ID, default UART), consider adding a `Kconfig` later and referencing `CONFIG_` macros.
- Runtime-permanent settings (per-servo calibration, IDs) are better stored in NVS or a config file rather than hardcoding.

Building
--------

From the project root run:

```sh
idf.py build
```

If `main/` includes `#include "sts3032_driver.h"` or `#include "sts3032_servo.h"`, the component will be built automatically.

Troubleshooting
---------------
- Missing includes: ensure your `main` files include the header via the plain filename (e.g., `#include "sts3032_servo.h"`) — don't use relative paths.
- If the UART driver or GPIO APIs are unresolved, add `REQUIRES driver` to the component `CMakeLists.txt` (already added in this repo).
- To verify the component compiled, inspect build output or `build/compile_commands.json`.

Where to look in this repo
-------------------------
- Component path: [components/sts3032/README.md](components/sts3032/README.md)
- Public headers: [components/sts3032/include/sts3032_servo.h](components/sts3032/include/sts3032_servo.h)
- Sources: [components/sts3032/src/sts3032_protocol.c](components/sts3032/src/sts3032_protocol.c) and [components/sts3032/src/sts3032_servo.c](components/sts3032/src/sts3032_servo.c)
