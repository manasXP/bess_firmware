# BESS Controller Firmware

## Overview

This is the main controller firmware for a Battery Energy Storage System (BESS) with 100KW power capacity and 200KWH energy storage. The system utilizes LFP (Lithium Iron Phosphate) battery modular packs, each with 48V and 16KWH capacity, managed by a comprehensive Battery Management System (BMS).

### Hardware Specifications
- **Microcontroller**: ESP32-P4
- **Operating System**: FreeRTOS
- **Battery Type**: LFP (Lithium Iron Phosphate)
- **System Capacity**: 200KWH total (modular design)
- **Module Configuration**: 48V, 16KWH modules
- **Power Rating**: 100KW

## System Architecture

The firmware is designed with a modular architecture that separates concerns across multiple components:

### Core Components

1. **System Manager**
   - Central coordinator for all subsystems
   - Manages operation modes and system state
   - Handles event processing and alarm management
   - Provides diagnostic capabilities and system monitoring
   - Controls startup and shutdown sequences

2. **Battery Management System (BMS)**
   - **Battery Manager**: Coordinates all battery functions
   - **SoC Calculator**: Estimates State of Charge using multiple algorithms
   - **Cell Balancer**: Manages cell balancing for uniform voltage levels
   - **Thermal Monitor**: Monitors temperatures and controls cooling systems

3. **Communication Interfaces**
   - **Modbus Interface**: Industry-standard protocol for SCADA/PLC integration
   - **CANBus Interface**: Real-time communication with power converters and other components
   - **AWS Interface**: Cloud connectivity for remote monitoring and management

4. **Logging System**
   - Multi-destination logging (console, SD card, AWS CloudWatch)
   - Configurable log levels and rotation
   - Timestamped event recording

## Operation Modes

The BESS controller supports the following operating modes:

1. **Standby Mode**: System initialized but not actively charging or discharging
2. **Charging Mode**: System is accepting energy from the grid or renewable sources
3. **Discharging Mode**: System is providing energy to the connected load
4. **Maintenance Mode**: Special mode for service operations like firmware updates
5. **Calibration Mode**: For calibrating sensors and SoC algorithms
6. **Fault Mode**: Triggered when alarms are active, limited operation
7. **Emergency Shutdown**: Critical fault condition, immediate safe shutdown

## Safety Features

The firmware implements multiple layers of safety protection:

### Active Monitoring
- Real-time monitoring of cell voltages, currents, and temperatures
- Early detection of thermal anomalies and potential runaway conditions
- Continuous imbalance detection across cells and modules

### Protection Systems
- **Voltage Protection**: Guards against over/under voltage conditions
- **Current Protection**: Prevents excessive charge/discharge currents
- **Thermal Protection**: Multi-level thermal management system
- **Emergency Shutdown**: Rapid response to critical conditions

### Fault Handling
- Comprehensive alarm system with multiple severity levels
- Automatic transition to safe states during fault conditions
- Event logging and notification for post-incident analysis

## Communication Interfaces

### Modbus Interface
- Industry-standard Modbus RTU protocol
- Configurable parameters (slave address, baud rate, parity)
- Register mapping for all critical system parameters
- Support for multiple Modbus function codes

### CANBus Interface
- High-speed, real-time communication
- Configurable bit rate and node ID
- Standard and extended frame support
- Error detection and recovery mechanisms

### AWS IoT Integration
- Secure MQTT communication with AWS IoT Core
- Device shadow for state synchronization
- Over-the-air command processing
- CloudWatch integration for remote logging

## Logging System

The firmware implements a versatile logging system supporting multiple destinations:

### Logging Destinations
1. **Console**: Standard output for debugging and development
2. **SD Card**: Persistent storage with automatic daily log rotation
3. **AWS CloudWatch**: Remote cloud-based logging for monitoring

### Log Levels
- **ERROR**: Critical issues requiring immediate attention
- **WARNING**: Potential issues that may require intervention
- **INFO**: Normal operational information
- **DEBUG**: Detailed information for troubleshooting
- **VERBOSE**: Extremely detailed diagnostic information

## Firmware Organization

The firmware is organized into the following main files:

1. **system_manager.c/h**: Core system coordination
2. **battery_manager.c/h**: Main BMS implementation and API
3. **soc_calculator.c/h**: SoC calculation algorithms
4. **cell_balancer.c/h**: Cell balancing functionality
5. **thermal_monitor.c/h**: Temperature monitoring and cooling control
6. **modbus_interface.c/h**: Modbus communication implementation
7. **canbus_interface.c/h**: CANBus communication implementation
8. **aws_interface.c/h**: AWS IoT connectivity

## API Reference

### System Manager API

#### Initialization and Control
- `system_manager_init()`: Initialize the system manager
- `system_manager_start()`: Start system operations
- `system_manager_stop()`: Stop system operations
- `system_manager_set_mode()`: Set system operating mode
- `system_manager_emergency_shutdown()`: Trigger emergency shutdown

#### Status and Information
- `system_manager_get_info()`: Get system information
- `system_manager_get_comm_stats()`: Get communication statistics
- `system_manager_get_active_alarms()`: Get bitmap of active alarms
- `system_manager_run_diagnostics()`: Run system diagnostics

#### Configuration
- `system_manager_update_config()`: Update system configuration
- `system_manager_save_config()`: Save configuration to NVS
- `system_manager_load_config()`: Load configuration from NVS
- `system_manager_configure_logging()`: Configure logging parameters

#### Events and Alarms
- `system_manager_register_event_callback()`: Register for event notifications
- `system_manager_unregister_event_callback()`: Unregister from event notifications
- `system_manager_trigger_alarm()`: Trigger a system alarm
- `system_manager_clear_alarm()`: Clear an active alarm

#### Logging
- `system_manager_log()`: Log a message to configured destinations
- `system_manager_sync_time()`: Synchronize system time with NTP

### BMS API

#### Battery Manager
- `battery_manager_init()`: Initialize the battery manager
- `battery_manager_start()`: Start BMS operations
- `battery_manager_get_module_data()`: Get detailed data for all modules
- `battery_manager_get_system_status()`: Get aggregated battery system status
- `battery_manager_get_soc()`: Get current State of Charge
- `battery_manager_get_soh()`: Get current State of Health

#### Cell Balancing
- `cell_balancer_init()`: Initialize the cell balancer
- `cell_balancer_update_cell_data()`: Update cell voltage data
- `cell_balancer_get_status()`: Get balancing status
- `cell_balancer_start_balancing()`: Start cell balancing process
- `cell_balancer_stop_balancing()`: Stop cell balancing process

#### Thermal Management
- `thermal_monitor_init()`: Initialize thermal monitoring
- `thermal_monitor_set_thresholds()`: Configure temperature thresholds
- `thermal_monitor_get_temperatures()`: Get current temperature data
- `thermal_monitor_check_runaway()`: Check for thermal runaway conditions
- `thermal_monitor_activate_cooling()`: Manually activate cooling system

## Build and Deploy

### Prerequisites
- ESP-IDF v5.0 or newer
- CMake 3.16 or newer
- Python 3.7 or newer (for ESP-IDF tools)

### Build Instructions
```bash
# Configure project
idf.py menuconfig

# Build the project
idf.py build

# Flash to the device
idf.py -p [PORT] flash

# Monitor serial output
idf.py -p [PORT] monitor
```

### Configuration Options
The system provides several configuration options accessible through the ESP-IDF menuconfig system:

- **WiFi Configuration**: SSID and password for cloud connectivity
- **AWS Configuration**: Endpoint URL and credentials
- **BMS Parameters**: Module count, voltage limits, temperature thresholds
- **Communication Settings**: Modbus and CANBus parameters

## Event System

The firmware implements a flexible event system that allows components to subscribe to various system events:

### Event Types
- `BESS_EVENT_ERROR`: Error conditions requiring attention
- `BESS_EVENT_WARNING`: Warning conditions that may need attention
- `BESS_EVENT_STATE_CHANGE`: System state or mode changes
- `BESS_EVENT_MODULE_STATUS`: Changes in module status
- `BESS_EVENT_SOC_THRESHOLD`: State of Charge threshold crossings
- `BESS_EVENT_TEMPERATURE_THRESHOLD`: Temperature threshold crossings

### Alarm Types
- `BESS_ALARM_OVERCURRENT`: Excessive current detected
- `BESS_ALARM_OVERVOLTAGE`: Cell or module voltage too high
- `BESS_ALARM_UNDERVOLTAGE`: Cell or module voltage too low
- `BESS_ALARM_OVER_TEMPERATURE`: Temperature exceeds safe limits
- `BESS_ALARM_UNDER_TEMPERATURE`: Temperature below safe limits
- `BESS_ALARM_THERMAL_RUNAWAY`: Dangerous thermal condition detected
- `BESS_ALARM_SOC_LOW`: State of Charge critically low
- `BESS_ALARM_COMMUNICATION_ERROR`: Persistent communication failures
- `BESS_ALARM_CLOUD_CONNECTION_ERROR`: Failure to connect to cloud services
- `BESS_ALARM_EMERGENCY_STOP_ACTIVATED`: Emergency stop triggered

## Reliability Features

The firmware includes several features to ensure reliable operation:

### Watchdog Timers
- Task watchdog for detecting task lockups
- System watchdog for overall firmware health monitoring

### Error Recovery
- Automatic reconnection for communication interfaces
- Graceful degradation during partial failures
- Automatic transition back to normal operation when faults clear

### Data Persistence
- Configuration storage in non-volatile memory (NVS)
- Backup configuration on SD card
- Comprehensive logging for post-incident analysis

## Future Enhancements

Planned enhancements for future firmware versions:

1. **Machine Learning**: ML-based SoC/SoH prediction models
2. **Adaptive Balancing**: Implement adaptive balancing strategies based on cell characteristics
3. **Aging Models**: Incorporate battery aging models for better SoH estimation
4. **Advanced Diagnostics**: More sophisticated diagnostic capabilities
5. **Power Prediction**: Add power availability prediction based on historical usage patterns
6. **Enhanced Security**: Implement additional security features for cloud connectivity
7. **Grid Integration**: Enhanced features for grid services and demand response
