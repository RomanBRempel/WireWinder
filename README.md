# WireWinder ESP32 Firmware

An automated control system designed for wire winding. It manages spool rotation, traverse positioning, and winding speed to ensure uniform layering and consistent coil geometry.

## Hardware Configuration

### GPIO Pin Mapping
- **GPIO 4**: Limit switch input (connected to 3.3V with a 2.3k Ohm pull-down resistor)
- **GPIO 16**: PWM input for external control (1000-2000 μs pulse width)
- **GPIO 18**: Servo output for the winding mechanism
- **GPIO 19**: Servo output for spool fixation

### Components
- ESP32 development board
- 2x Servo motors (winding mechanism and spool fixation)
- Limit switch with pull-down resistor
- External PWM controller (optional)

## Features

1. **Limit Switch Detection**: Monitors GPIO 4 for limit switch activation to prevent overwinding
2. **PWM Speed Control**: Reads PWM signal on GPIO 16 to dynamically adjust winding speed
3. **Winding Servo Control**: Controls the winding mechanism servo on GPIO 18 based on PWM input
4. **Spool Fixation**: Automatically locks/unlocks the spool using servo on GPIO 19
5. **Safety Logic**: Stops winding and engages fixation when limit switch is triggered

## Building and Uploading

### Prerequisites
- [PlatformIO](https://platformio.org/) installed (either CLI or IDE extension)
- ESP32 drivers installed for your operating system

### Build Instructions

Using PlatformIO CLI:
```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

Using PlatformIO IDE (VS Code):
1. Open the project folder in VS Code
2. Click the PlatformIO icon in the sidebar
3. Click "Build" to compile
4. Click "Upload" to flash the firmware
5. Click "Monitor" to view serial output

## Operation

### Normal Operation
- The winding servo responds to the PWM input signal (1000-2000 μs)
- PWM pulse width is mapped to servo angle (0-180 degrees)
- Spool fixation is released during normal winding

### Limit Switch Activation
- When the limit switch is pressed (GPIO 4 goes HIGH):
  - Winding servo moves to neutral position (90 degrees)
  - Spool fixation engages (180 degrees) to lock the spool
  - System waits for limit switch to be released

### PWM Input Mapping
- 1000 μs pulse → 0° servo angle (minimum speed)
- 1500 μs pulse → 90° servo angle (medium speed)
- 2000 μs pulse → 180° servo angle (maximum speed)

## Serial Monitor Output

The firmware outputs diagnostic information at 115200 baud:
- Initialization status for all components
- PWM pulse width and calculated winding speed
- Limit switch state changes
- Spool fixation engage/release events

## Safety Features

- Automatic stop on limit switch activation
- Constrained PWM input range to prevent invalid servo commands
- Spool fixation to prevent unwinding when limit is reached

## License

Open source - modify and use as needed for your wire winding applications.
