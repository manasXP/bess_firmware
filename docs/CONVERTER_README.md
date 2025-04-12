# Converter Interface System

## Overview

The Converter Interface System is a critical component of the 100KW/200KWH Battery Energy Storage System (BESS) firmware. It provides a comprehensive abstraction layer for controlling the power converters, including DC-DC converters and DC-AC inverters (bidirectional converters). This module ensures safe and efficient power conversion operations while offering a unified API for other system components.

## Architecture

The Converter Interface consists of the following main components:

### Core Components

1. **Converter Manager**
   - Central component that coordinates all converter operations
   - Handles converter registration and configuration
   - Implements thread-safe access to converter data
   - Provides unified API for different converter types

2. **Converter Monitor**
   - Background task that monitors converter status
   - Detects fault conditions and anomalies
   - Updates operational parameters
   - Manages thermal monitoring and protection

3. **Event System**
   - Provides callback mechanism for converter events
   - Enables asynchronous notification of state changes
   - Supports multiple listeners for each event type
   - Facilitates coordination with other system components

4. **Grid Interface**
   - Manages grid connection and disconnection
   - Controls grid-forming and grid-following modes
   - Handles grid synchronization
   - Implements grid-code compliance features

## Key Features

- **Multi-Converter Support**: Manages multiple converters of different types simultaneously
- **Comprehensive Control**: Provides full control over voltage, current, power, and frequency
- **Thread Safety**: All operations are protected with FreeRTOS synchronization primitives
- **Fault Management**: Detects, reports, and manages fault conditions
- **Operational Modes**: Supports multiple operational modes for different converter types
- **Dynamic Reconfiguration**: Allows runtime parameter adjustments
- **Thermal Protection**: Monitors and manages converter temperatures
- **Self-Test Capability**: Built-in self-testing functionality
- **Firmware Updates**: Support for converter firmware updates
- **Status Monitoring**: Comprehensive status reporting

## Implementation Details

### Task Structure

The implementation uses FreeRTOS tasks:

1. **Monitoring Task**: 
   - Runs at 500ms intervals
   - Updates converter status data
   - Checks for fault conditions
   - Manages protective actions

### Synchronization Mechanisms

The system employs multiple synchronization mechanisms:

- **Mutex (`s_converter_mutex`)**: Protects shared converter data during concurrent access
- **Event Group (`s_converter_events`)**: Signals converter events and coordinates operations
- **Callback System**: Enables asynchronous event notifications

### Converter Types and Modes

The system supports three types of converters:
- **DC-DC**: For DC voltage conversion
- **DC-AC**: For inverter/rectifier operations
- **Bidirectional**: Supporting both DC-DC and DC-AC functions

Each converter can operate in various modes:
- **Off**: Converter is completely disabled
- **Standby**: Powered but not actively converting
- **Buck**: DC-DC step-down operation
- **Boost**: DC-DC step-up operation
- **Inverter**: DC to AC conversion (discharging)
- **Rectifier**: AC to DC conversion (charging)
- **Grid-Forming**: Creating a grid reference
- **Grid-Following**: Following an existing grid reference
- **MPPT**: Maximum Power Point Tracking (for PV integration)

### Safety Features

The implementation includes multiple safety features:

- **Limit Enforcement**: Validates all parameters against safe operating limits
- **Fault Detection**: Identifies abnormal conditions and issues appropriate warnings/errors
- **Thermal Protection**: Monitors temperature and implements protective actions
- **Emergency Shutdown**: Provides immediate shutdown capability for critical conditions
- **Parameter Validation**: Ensures all API calls use valid parameters

## API Interface

The Converter Interface exposes a comprehensive API:

### Initialization and Configuration

- `converter_interface_init()`: Initialize the converter interface
- `converter_register()`: Register a new converter
- `converter_configure_dcdc()`: Configure a DC-DC converter
- `converter_configure_inverter()`: Configure an inverter

### Control Functions

- `converter_set_mode()`: Set converter operation mode
- `converter_start()`: Start a converter
- `converter_stop()`: Stop a converter
- `converter_set_voltage()`: Set output voltage
- `converter_set_current_limit()`: Set current limit
- `converter_set_power()`: Set power setpoint
- `converter_set_frequency()`: Set AC output frequency
- `converter_set_power_factor()`: Set power factor
- `converter_set_reactive_power()`: Set reactive power

### Grid Management

- `converter_connect_grid()`: Connect inverter to grid
- `converter_disconnect_grid()`: Disconnect from grid

### Status and Monitoring

- `converter_get_mode()`: Get current operation mode
- `converter_get_status()`: Get detailed status information
- `converter_get_efficiency()`: Get current efficiency
- `converter_get_firmware_version()`: Get firmware version

### Fault Management

- `converter_reset_fault()`: Reset a fault condition
- `converter_self_test()`: Perform self-test

### Event Management

- `converter_register_callback()`: Register for event notifications
- `converter_unregister_callback()`: Unregister from notifications

## Event Types

The system generates various events:

- `CONVERTER_EVENT_STARTUP`: Converter started up
- `CONVERTER_EVENT_SHUTDOWN`: Converter shut down
- `CONVERTER_EVENT_MODE_CHANGE`: Operation mode changed
- `CONVERTER_EVENT_FAULT`: Fault occurred
- `CONVERTER_EVENT_WARNING`: Warning condition
- `CONVERTER_EVENT_GRID_CONNECT`: Connected to grid
- `CONVERTER_EVENT_GRID_DISCONNECT`: Disconnected from grid

## Configuration Parameters

DC-DC converter configuration includes:
- Input and output voltage ranges
- Current and power limits
- Soft start parameters

Inverter configuration includes:
- DC voltage range
- AC voltage and frequency settings
- Grid-tie and island mode settings
- Power factor and reactive power parameters

## Integration with BESS System

The Converter Interface integrates with other BESS components:

- **Battery Management System**: Receives power limits based on battery state
- **System Controller**: Receives operational commands and mode settings
- **Grid Interface**: Coordinates grid connection and power flow
- **Safety Monitoring**: Reports status and receives emergency commands

## Usage Examples

### Initializing the Converter Interface

```c
// Initialize converter interface with capacity for 3 converters
esp_err_t result = converter_interface_init(3);
if (result != ESP_OK) {
    // Handle initialization error
}
```

### Registering a Converter

```c
// Register a DC-DC converter
uint8_t dcdc_id;
esp_err_t result = converter_register(CONVERTER_TYPE_DC_DC, &dcdc_id);
if (result == ESP_OK) {
    printf("Registered DC-DC converter with ID: %d\n", dcdc_id);
}
```

### Configuring a DC-DC Converter

```c
// Configure the DC-DC converter
dcdc_converter_config_t config = {
    .input_voltage_min = 40.0f,
    .input_voltage_max = 60.0f,
    .output_voltage_min = 350.0f,
    .output_voltage_max = 450.0f,
    .current_limit = 100.0f,
    .power_limit = 20.0f,
    .soft_start = true,
    .soft_start_ms = 500
};

esp_err_t result = converter_configure_dcdc(dcdc_id, &config);
```

### Setting Converter Operation

```c
// Start converter in boost mode
converter_set_mode(dcdc_id, CONVERTER_MODE_BOOST);
converter_set_voltage(dcdc_id, 400.0f);
converter_start(dcdc_id);
```

### Registering for Events

```c
// Callback function for converter events
void converter_event_handler(uint8_t converter_id,
                           converter_event_t event,
                           void *event_data,
                           void *user_data) {
    switch (event) {
        case CONVERTER_EVENT_FAULT:
            printf("Converter %d fault: %u\n", converter_id, (uint32_t)event_data);
            break;
        // Handle other events...
    }
}

// Register for all events from a specific converter
converter_register_callback(dcdc_id, 0xFF, converter_event_handler, NULL);
```

## Technical Specifications

- **Power Range**: Supports converters up to 100kW
- **Voltage Range**: DC: 0-1000V, AC: 0-480V
- **Frequency Range**: 45-65Hz
- **Control Update Rate**: 500ms typical
- **Maximum Converters**: 8 converters simultaneously
- **Event Callbacks**: Up to 10 callbacks per converter
- **Thread Safety**: All functions are thread-safe

## Safety and Error Handling

The Converter Interface implements robust error handling:

- All public functions return ESP error codes
- Parameter validation prevents unsafe operations
- Mutex timeouts prevent deadlocks
- Fault reporting allows system-level response
- Self-diagnostic capabilities identify issues

## Resource Requirements

- **Memory**: Approximately 2KB per converter
- **Task Stack**: 4KB for monitoring task
- **CPU**: Low to moderate usage, primarily during status updates
- **Task Priority**: Mid-level priority (5) for monitoring task
