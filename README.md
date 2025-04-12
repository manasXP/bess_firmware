# Battery Energy Storage System (BESS) Firmware Design

## Overview
This repository contains firmware for a 100kW/200kWh Battery Energy Storage System (BESS) based on LFP battery modules. The system uses 48V/16kWh modular battery packs, an ESP32-P4 microcontroller, and FreeRTOS as the real-time operating system.

## Core Features

1. **Battery Management**
   - Cell voltage monitoring
   - Temperature monitoring
   - State of Charge (SoC) calculation
   - State of Health (SoH) estimation
   - Cell balancing control

2. **Power Management**
   - Charge/discharge control
   - Power conversion monitoring
   - Grid synchronization
   - Load management

3. **Safety Systems**
   - Over/under voltage protection
   - Over/under temperature protection
   - Over-current protection
   - Isolation monitoring
   - Smoke/fire detection
   - Emergency shutdown sequence

4. **Communication Interfaces**
   - Modbus RTU/TCP support
   - CANbus interface for battery module communication
   - WiFi/Ethernet for remote monitoring
   - MQTT client for AWS IoT integration

5. **System Monitoring & Control**
   - Real-time performance metrics
   - Efficiency calculations
   - Thermal management
   - Automated charge/discharge scheduling

6. **Data Logging**
   - Console output
   - SD card storage
   - AWS CloudWatch integration
   - Event logging with severity levels

7. **Firmware Management**
   - OTA updates
   - Configuration management
   - Bootloader with recovery mode

8. **User Interface**
   - Local display support
   - Web interface for configuration
   - API for mobile app integration

## File Structure

```
/bess_firmware/
├── CMakeLists.txt                  # Main build system file
├── sdkconfig                       # ESP-IDF configuration
├── main/                           # Main application code
│   ├── CMakeLists.txt              # Build system for main component
│   ├── main.c                      # Entry point
│   ├── bess_config.h               # System configuration
│   └── bess_types.h                # Common type definitions
├── components/                     # Modular components
│   ├── battery_management/         # Battery monitoring and control
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── battery_manager.h
│   │   │   ├── soc_calculator.h
│   │   │   ├── cell_balancer.h
│   │   │   └── thermal_monitor.h
│   │   └── src/
│   │       ├── battery_manager.c
│   │       ├── soc_calculator.c
│   │       ├── cell_balancer.c
│   │       └── thermal_monitor.c
│   ├── power_management/           # Power flow control
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── power_manager.h
│   │   │   ├── converter_interface.h
│   │   │   └── load_controller.h
│   │   └── src/
│   │       ├── power_manager.c
│   │       ├── converter_interface.c
│   │       └── load_controller.c
│   ├── safety_systems/             # Safety monitoring and protection
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── safety_manager.h
│   │   │   ├── fault_detector.h
│   │   │   └── emergency_handler.h
│   │   └── src/
│   │       ├── safety_manager.c
│   │       ├── fault_detector.c
│   │       └── emergency_handler.c
│   ├── communication/              # Communication interfaces
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── comm_manager.h
│   │   │   ├── modbus_interface.h
│   │   │   ├── canbus_interface.h
│   │   │   └── mqtt_client.h
│   │   └── src/
│   │       ├── comm_manager.c
│   │       ├── modbus_interface.c
│   │       ├── canbus_interface.c
│   │       └── mqtt_client.c
│   ├── system_control/             # System monitoring and control
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── system_manager.h
│   │   │   ├── scheduler.h
│   │   │   └── metrics_analyzer.h
│   │   └── src/
│   │       ├── system_manager.c
│   │       ├── scheduler.c
│   │       └── metrics_analyzer.c
│   ├── data_logging/               # Logging functionality
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── logger.h
│   │   │   ├── console_logger.h
│   │   │   ├── sd_logger.h
│   │   │   └── cloudwatch_logger.h
│   │   └── src/
│   │       ├── logger.c
│   │       ├── console_logger.c
│   │       ├── sd_logger.c
│   │       └── cloudwatch_logger.c
│   ├── firmware_management/        # OTA and configuration
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── ota_manager.h
│   │   │   └── config_manager.h
│   │   └── src/
│   │       ├── ota_manager.c
│   │       └── config_manager.c
│   └── user_interface/             # UI components
│       ├── CMakeLists.txt
│       ├── include/
│       │   ├── ui_manager.h
│       │   ├── web_server.h
│       │   └── display_controller.h
│       └── src/
│           ├── ui_manager.c
│           ├── web_server.c
│           └── display_controller.c
├── drivers/                        # Hardware-specific drivers
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── hw_interface.h
│   │   ├── gpio_controller.h
│   │   └── sensor_interface.h
│   └── src/
│       ├── hw_interface.c
│       ├── gpio_controller.c
│       └── sensor_interface.c
├── middleware/                     # FreeRTOS tasks and utils
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── task_manager.h
│   │   ├── queue_manager.h
│   │   └── semaphore_manager.h
│   └── src/
│       ├── task_manager.c
│       ├── queue_manager.c
│       └── semaphore_manager.c
├── utils/                          # Common utilities
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── common_utils.h
│   │   ├── math_utils.h
│   │   └── time_utils.h
│   └── src/
│       ├── common_utils.c
│       ├── math_utils.c
│       └── time_utils.c
└── tests/                          # Unit tests
    ├── CMakeLists.txt
    ├── test_battery_management.c
    ├── test_power_management.c
    ├── test_safety_systems.c
    └── test_communication.c
```

## Key Design Considerations

1. **Modularity**: The firmware is organized into separate components that can be developed and tested independently.

2. **FreeRTOS Integration**: The system will use multiple tasks for different functions with appropriate priorities:
   - Safety monitoring (highest priority)
   - Battery management 
   - Power control
   - Communication
   - Logging (lower priority)

3. **Redundancy**: Critical safety systems have multiple layers of protection.

4. **Scalability**: The system is designed to handle multiple 48V/16kWh battery modules (13 modules to reach 200kWh capacity).

5. **Fault Tolerance**: The system can operate in degraded mode if non-critical components fail.

## Technical Specifications
- MCU: ESP32-P4
- RTOS: FreeRTOS
- Programming Languages: C/C++
- Communication Protocols: Modbus, CANbus
- Logging: Console, SD Card, AWS CloudWatch
- Battery Type: LFP (Lithium Iron Phosphate)
- System Capacity: 200kWh
- System Power: 100kW
- Battery Module Configuration: 48V, 16kWh modules
