# GPIO Controller Module for BESS

## Overview

The GPIO Controller module provides a comprehensive interface for managing all GPIO pins and related functionalities in the Battery Energy Storage System (BESS). It is designed for the ESP32-P4 MCU platform using FreeRTOS, focusing on safety, reliability, and ease of use.

This module handles the following GPIO-related functions:
- Safety circuits and relay control
- Status indicator LEDs
- Cooling fan control (PWM)
- Communication interface control (MODBUS, CAN)
- Sensor and alarm inputs/outputs
- SD card detection

## Key Features

### Safety-First Design
- **Precharge Protection**: Enforces proper precharge sequence before main relay activation
- **Emergency Stop Handling**: All relay operations check E-STOP status
- **Fail-Safe Logic**: Proper relay disabling sequence during shutdown
- **Timing Controls**: Enforced minimum precharge time before main relay activation

### Thread Safety and Resource Management
- **Mutex Protection**: All operations are protected with mutexes for thread safety in FreeRTOS
- **ISR Handling**: Proper interrupt service routine management for critical alerts
- **Safe Initialization/Deinitialization**: Proper resource allocation and cleanup

### Smart Peripheral Management
- **PWM Fan Control**: Implementation using LEDC for precise cooling control
- **Communication Controls**: Dedicated functions for MODBUS and CANbus transceiver control
- **Status Indicators**: Comprehensive LED control for system status visualization

### Flexible Configuration
- **Configurable Pins**: Ability to reconfigure pin assignments and behavior
- **Interrupt Management**: Registration and control of interrupt handlers
- **Logic Inversion**: Support for active-high and active-low signals with consistent API

## Usage Examples

### Initialization

Initialize the GPIO controller at system startup:

```c
// Initialize the GPIO controller with default settings
esp_err_t result = gpio_controller_init();
if (result != ESP_OK) {
    // Handle initialization error
    printf("GPIO controller initialization failed: %d\n", result);
}
```

### Relay Control

Control the main power and precharge relays with proper sequencing:

```c
// Start precharge process
esp_err_t result = gpio_controller_set_precharge_relay(true);
if (result != ESP_OK) {
    // Handle precharge error
}

// Wait for precharge (system should monitor capacitor voltage)
vTaskDelay(pdMS_TO_TICKS(2000));

// Enable main relay after precharge
result = gpio_controller_set_main_relay(true);
if (result != ESP_OK) {
    // Handle main relay error
}
```

### LED Status Control

Control the status indicator LEDs:

```c
// Set system status LEDs
// fault, running, charging, discharging
gpio_controller_set_status_leds(false, true, true, false);
```

### Fan Control

Adjust cooling fan speed:

```c
// Set fan to 75% duty cycle
gpio_controller_set_fan_duty(75);
```

### Emergency Stop Monitoring

Check emergency stop status:

```c
bool estop_active = false;
esp_err_t result = gpio_controller_is_estop_active(&estop_active);
if (result == ESP_OK && estop_active) {
    printf("Emergency stop is active!\n");
    // Implement safety shutdown procedure
}
```

### Communication Interface Control

Control the MODBUS and CAN interfaces:

```c
// Set MODBUS to transmit mode
gpio_controller_set_modbus_mode(true);

// Send data...

// Return MODBUS to receive mode
gpio_controller_set_modbus_mode(false);

// Put CAN bus in standby mode when not in use to save power
gpio_controller_set_canbus_standby(true);
```

### Interrupt Handling

Register interrupt handlers for critical events:

```c
// Callback function for emergency stop
void estop_handler(gpio_bess_pin_t pin, void* user_data) {
    printf("Emergency stop triggered!\n");
    // Implement safety shutdown procedure
}

// Register the callback for emergency stop
gpio_controller_register_interrupt(GPIO_BESS_ESTOP_INPUT, 
                                 GPIO_INTR_NEGEDGE, 
                                 estop_handler, 
                                 NULL);
```

## Pin Configuration

The module provides default pin mappings but also allows custom configuration:

```c
// Custom configuration for a pin
gpio_bess_config_t custom_config = {
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = true,
    .pull_down_en = false,
    .invert_logic = true,
    .interrupt_type = GPIO_INTR_ANYEDGE
};

// Apply custom configuration
gpio_controller_configure_pin(GPIO_BESS_AUX_IO_1, &custom_config);
```

## Safety Considerations

The GPIO controller implements several safety mechanisms:

1. **Relay Sequence**: The main relay cannot be activated until precharge is complete
2. **E-Stop Monitoring**: All critical operations check emergency stop status
3. **Fault Detection**: Monitoring of BMS alerts and inverter faults
4. **Safe Shutdown**: Proper sequence for deactivating relays during shutdown

## Error Handling

All functions return ESP-IDF error codes:

- `ESP_OK`: Operation successful
- `ESP_ERR_INVALID_ARG`: Invalid parameter
- `ESP_ERR_INVALID_STATE`: Component in wrong state
- `ESP_FAIL`: Generic failure

Example:

```c
esp_err_t result = gpio_controller_set_main_relay(true);
if (result != ESP_OK) {
    switch (result) {
        case ESP_ERR_INVALID_STATE:
            printf("Cannot enable main relay: System not initialized\n");
            break;
        case ESP_FAIL:
            printf("Cannot enable main relay: Safety check failed\n");
            break;
        default:
            printf("Cannot enable main relay: Error %d\n", result);
            break;
    }
}
```

## Integration with Other Subsystems

The GPIO controller is designed to integrate with other BESS components:

- **Battery Management System**: For BMS alert monitoring
- **Inverter Control**: For fault detection and grid presence
- **Logging System**: For recording critical events
- **Communication Modules**: For controlling MODBUS and CAN interfaces

## Files

- `gpio_controller.h`: Interface declaration
- `gpio_controller.c`: Implementation of all functionality
