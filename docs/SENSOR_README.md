# Sensor Interface for BESS 100KW/200KWH System

## Overview

The Sensor Interface is a critical component of the Battery Energy Storage System (BESS) firmware that provides a standardized API for interacting with various sensor types across the system. It abstracts hardware-specific details and provides a unified approach to sensor data acquisition, configuration, and event handling.

This component is essential for monitoring the health and operational status of the LFP battery modules (48V, 16KWH), supporting critical functions such as voltage, current, and temperature monitoring, as well as environmental and safety sensors.

## Architecture

The Sensor Interface consists of two main files:

1. **sensor_interface.h**: Header file that defines the API and data structures
2. **sensor_interface.c**: Implementation file that provides the core functionality

### Key Components

The implementation includes several core components:

- **Sensor Management**: Create, configure, and delete sensor instances
- **Data Acquisition**: Read individual sensors or perform batch operations
- **Event System**: Threshold detection and callback notifications
- **Configuration Management**: Adjust thresholds, sampling rates, and other parameters
- **Task Management**: Background tasks for autonomous sensor sampling
- **Error Handling**: Comprehensive error detection and reporting

### Supported Sensor Types

The interface supports multiple sensor types critical for BESS operation:

- **Voltage Sensors**: Monitor cell and module voltages
- **Current Sensors**: Track charge/discharge current
- **Temperature Sensors**: Monitor cell, module, and system temperatures
- **Environmental Sensors**: Humidity, pressure
- **Safety Sensors**: Smoke detection, door switches
- **Flow Sensors**: For liquid cooling systems

### Communication Protocols

Multiple communication protocols are supported for flexibility:

- **I2C**: Common digital interface for many sensors
- **SPI**: High-speed interface for specialized sensors
- **OneWire**: Interface for Dallas/Maxim temperature sensors
- **Analog**: Direct ADC measurements
- **Digital**: Binary inputs
- **Modbus**: Industrial standard communication
- **CANBus**: Automotive and industrial networking

## Implementation Details

### Thread Safety

All functions in the sensor interface are thread-safe, utilizing FreeRTOS primitives:

- **Mutexes**: Protect shared data structures during concurrent access
- **Event Queues**: Handle asynchronous event notifications
- **Tasks**: Perform background operations without blocking

### Memory Management

The implementation is optimized for the ESP32-P4 with limited resources:

- **Static Allocation**: Pre-allocated sensor slots to prevent fragmentation
- **Configurable Limits**: Adjustable maximum number of sensors
- **Efficient Data Structures**: Compact representations of sensor configuration and data

### Task Structure

The implementation uses a task-based architecture for asynchronous operation:

1. **Event Task**: Processes and dispatches all sensor events
2. **Update Tasks**: Per-sensor tasks that periodically sample data
3. **Main Thread**: API calls from application code

## Usage Examples

### Initializing the Sensor Interface

```c
// Initialize the sensor interface
esp_err_t result = sensor_interface_init();
if (result != ESP_OK) {
    // Handle initialization error
}
```

### Creating a Temperature Sensor

```c
// Configure temperature sensor
sensor_config_t config = {
    .type = SENSOR_TYPE_TEMPERATURE,
    .protocol = SENSOR_PROTOCOL_ONEWIRE,
    .resolution = SENSOR_RESOLUTION_HIGH,
    .update_freq = SENSOR_UPDATE_MEDIUM,
    .config.onewire = {
        .pin = 5,
        .rom_code = {0x28, 0xFF, 0x64, 0x02, 0xEC, 0x12, 0x34, 0x56}
    },
    .min_value = -20.0f,
    .max_value = 100.0f,
    .sampling_interval_ms = 1000,
    .warning_threshold = 45.0f,
    .critical_threshold = 55.0f,
    .emergency_threshold = 65.0f,
    .name = "Module1_Temp1",
    .location = "Top Center",
    .module_id = 1,
    .enabled = 1
};

// Create the sensor
sensor_handle_t temp_sensor;
esp_err_t result = sensor_create(&config, &temp_sensor);
if (result != ESP_OK) {
    // Handle sensor creation error
}

// Start the sensor
sensor_start(temp_sensor);
```

### Reading a Sensor

```c
// Read temperature value
sensor_value_t value;
esp_err_t result = sensor_read(temp_sensor, &value);
if (result == ESP_OK && value.valid) {
    float temperature = value.value.float_value;
    printf("Temperature: %.2f°C at %u ms\n", 
            temperature, value.timestamp_ms);
}
```

### Finding Sensors by Type or Module

```c
// Find all temperature sensors
sensor_handle_t temp_sensors[10];
size_t count = 0;
sensor_find_by_type(SENSOR_TYPE_TEMPERATURE, temp_sensors, 10, &count);
printf("Found %zu temperature sensors\n", count);

// Find all sensors for module 3
sensor_handle_t module_sensors[20];
sensor_find_by_module(3, module_sensors, 20, &count);
printf("Found %zu sensors for module 3\n", count);
```

### Registering for Events

```c
// Callback function
void temp_alarm_callback(sensor_event_t *event) {
    printf("Temperature alarm: %.2f°C\n", event->value.value.float_value);
    
    // Take appropriate action based on event type
    switch (event->type) {
        case SENSOR_EVENT_THRESHOLD_WARNING:
            // Handle warning
            break;
        case SENSOR_EVENT_THRESHOLD_CRITICAL:
            // Handle critical condition
            break;
        case SENSOR_EVENT_THRESHOLD_EMERGENCY:
            // Handle emergency
            break;
    }
}

// Register for critical threshold events
sensor_register_event_callback(temp_sensor, 
                             SENSOR_EVENT_THRESHOLD_CRITICAL,
                             temp_alarm_callback, 
                             NULL);
```

### Calibrating a Sensor

```c
// Calibrate a voltage sensor with a known reference value
esp_err_t result = sensor_calibrate(voltage_sensor, 3.3f);
if (result != ESP_OK) {
    // Handle calibration error
}
```

## Integration with BMS Components

The Sensor Interface integrates with other BESS components:

1. **Battery Manager**: Uses voltage and current sensors to track module status
2. **Thermal Monitor**: Relies on temperature sensors for thermal management
3. **SoC Calculator**: Requires voltage/current data for SoC estimation
4. **Cell Balancer**: Uses cell voltage readings to determine balancing needs

## Error Handling

All functions return `esp_err_t` status codes:

- **ESP_OK**: Operation completed successfully
- **ESP_ERR_INVALID_ARG**: Invalid function arguments
- **ESP_ERR_NO_MEM**: Memory allocation failure
- **ESP_ERR_TIMEOUT**: Operation timed out
- **ESP_ERR_INVALID_STATE**: Component in invalid state
- **ESP_ERR_NOT_FOUND**: Requested item not found
- **ESP_ERR_NOT_SUPPORTED**: Operation not supported

Detailed error information is available through:

```c
uint32_t error_code;
char error_message[64];
sensor_get_last_error(sensor, &error_code, error_message, sizeof(error_message));
```

## Future Enhancements

Potential improvements for future versions:

1. **Dynamic Sensor Discovery**: Auto-detect sensors on supported buses
2. **Advanced Filtering**: Implement Kalman filters for noise reduction
3. **Sensor Fusion**: Combine readings from multiple sensors for improved accuracy
4. **Power Management**: Dynamic adjustment of sampling rates based on system state
5. **Remote Sensors**: Support for network-connected sensors
6. **Data Logging**: Built-in historical data storage
7. **Self-Calibration**: Automated calibration routines

## Implementation File Contents

### sensor_interface.h

This header file defines the API including:

- Sensor types, protocols, and configuration structures
- Value representation and event types
- Core API functions for sensor management
- Error handling, calibration, and diagnostic functions

### sensor_interface.c

The implementation file provides:

- Task and memory management
- Protocol-specific implementations
- Event handling and callback management
- Thread-safe data access
- Threshold monitoring
- Simulation logic for testing without hardware
