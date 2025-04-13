# Emergency Handler System

## Overview

The Emergency Handler System is a critical safety component of the 100KW/200KWH Battery Energy Storage System (BESS) firmware. It provides comprehensive emergency detection, response, and recovery functionality to protect the system against electrical, thermal, and operational hazards.

Built for the ESP32-P4 MCU running FreeRTOS, this system integrates closely with the Battery Management System (BMS) and other BESS components to provide a multi-layered safety approach for the LFP battery modules (48V, 16KWH).

## Key Features

### Comprehensive Emergency Detection

The Emergency Handler can detect and respond to a wide range of emergency conditions:

- **Electrical Emergencies**: Over-voltage, under-voltage, over-current, short circuit
- **Thermal Emergencies**: Over-temperature, thermal runaway
- **Battery Emergencies**: Cell imbalance, isolation faults
- **System Emergencies**: Communication failures, BMS failures, watchdog timeouts
- **External Emergencies**: Grid faults, manual emergency triggers

### Multi-level Response System

Different emergency types trigger appropriate responses based on severity:

- **Information**: Logging and monitoring only
- **Low Severity**: Alerts and minor interventions
- **Medium Severity**: Operational limitations and active interventions
- **High Severity**: Immediate protective actions
- **Critical Severity**: Complete system shutdown and lockout

### Configurable Actions

The system provides multiple response options that can be mapped to different emergency types:

- **Logging**: Event recording to various destinations
- **Alerting**: User notifications and external system alerts
- **Power Limiting**: Reducing system power capabilities
- **Operational Control**: Stopping charging or discharging
- **Disconnection**: Removing system from grid or load
- **Thermal Management**: Activating cooling systems
- **System Protection**: Contactor control and shutdown procedures

### Automatic Recovery

For non-critical emergencies, the system attempts to recover automatically:

- **Cooldown Period**: Configurable delay between recovery attempts
- **Limited Attempts**: Prevents infinite recovery loops
- **Condition Verification**: Checks if emergency conditions have cleared
- **State Tracking**: Manages transition between emergency and normal states

### Watchdog Monitoring

Built-in watchdog ensures system responsiveness:

- **Timeout Detection**: Identifies system unresponsiveness
- **Automatic Response**: Triggers appropriate actions when timeout occurs
- **Recovery Integration**: Includes recovery procedures for watchdog events

### Event Callback System

External components can register for emergency notifications:

- **Selective Registration**: Components can select which events to monitor
- **Context Data**: Callbacks receive detailed event information
- **Multiple Listeners**: Support for multiple registered callbacks

### Flexible Logging

Comprehensive event logging to multiple destinations:

- **Console Output**: Real-time monitoring via serial console
- **SD Card Storage**: Persistent local storage for post-incident analysis
- **AWS CloudWatch**: Remote monitoring and alerting capabilities

### Self-Testing Capability

Built-in self-test functionality verifies emergency handling:

- **Basic Testing**: Verifies internal functionality without disruption
- **Intermediate Testing**: Tests emergency triggering and notifications
- **Advanced Testing**: Optional testing of physical response mechanisms

## Integration with BESS

The Emergency Handler is designed to work seamlessly with other BESS components:

- **Battery Manager**: Processes battery events and conditions
- **Thermal Monitor**: Responds to thermal zones and temperature alerts
- **Communications**: Handles communication failures from Modbus and CANBus
- **Main Controller**: Provides system-wide emergency coordination

## System Architecture

The Emergency Handler uses a task-based architecture with FreeRTOS:

- **Handler Task**: Continuously monitors for emergency conditions
- **Event Group**: Coordinates emergency signaling between components
- **Mutexes**: Ensure thread safety for shared data structures
- **Callback Registry**: Manages notification subscriptions

## API Reference

### Initialization and Control

```c
// Initialize with custom configuration
emergency_handler_config_t config = {
    .watchdog_timeout_ms = 5000,         // 5 seconds timeout
    .auto_recover_enabled = true,        // Enable automatic recovery
    .recovery_cooldown_ms = 60000,       // 1 minute between attempts
    .max_recovery_attempts = 3,          // Maximum 3 recovery attempts
    .log_to_console = true,              // Enable console logging
    .log_to_sd = true,                   // Enable SD card logging
    .log_to_cloud = false                // Disable cloud logging initially
};

// Initialize and start
esp_err_t res = emergency_handler_init(&config);
if (res == ESP_OK) {
    emergency_handler_start();
}
```

### Emergency Triggering

```c
// Trigger an emergency for a specific module
emergency_handler_trigger(
    EMERGENCY_OVER_TEMP,        // Emergency type
    SEVERITY_HIGH,              // Severity level
    3,                          // Module ID (3)
    85.5f,                      // Measured temperature (85.5°C)
    80.0f,                      // Threshold temperature (80.0°C)
    "Critical temperature in module 3" // Description
);

// Manual emergency trigger
emergency_handler_manual_trigger("Emergency stop button pressed");
```

### Event Handling

```c
// Callback function
void emergency_event_handler(emergency_event_t event, void *user_data) {
    printf("Emergency: %s (Severity: %s) in module %d\n",
           emergency_handler_type_to_string(event.type),
           emergency_handler_severity_to_string(event.severity),
           event.module_id);
    
    // Take appropriate actions based on event
}

// Register callback for all emergency types
emergency_handler_register_callback(emergency_event_handler, NULL, 0);

// Register callback for specific emergency types
emergency_handler_register_callback(
    thermal_emergency_handler, 
    NULL, 
    EMERGENCY_OVER_TEMP | EMERGENCY_THERMAL_RUNAWAY
);
```

### Recovery and Reset

```c
// Check if any emergency is active
bool is_active;
emergency_handler_is_active(&is_active);

// Get current emergency state
emergency_state_t state;
emergency_handler_get_state(&state);

// Clear specific emergency
emergency_handler_clear(EMERGENCY_OVER_VOLTAGE, false);

// Clear all emergencies (force)
emergency_handler_clear(EMERGENCY_NONE, true);

// Reset emergency handler
emergency_handler_reset(false); // Soft reset
```

### Watchdog Management

```c
// Feed the watchdog periodically from main system task
void system_task(void *pvParameters) {
    while (1) {
        // System operations here
        
        // Feed watchdog
        emergency_handler_feed_watchdog();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Configuration

```c
// Configure specific emergency response
emergency_handler_set_response(
    EMERGENCY_OVER_CURRENT,
    RESPONSE_ALERT | RESPONSE_LIMIT_POWER | RESPONSE_STOP_CHARGE
);

// Configure logging destinations
emergency_handler_configure_logging(true, true, true); // Console, SD, Cloud
```

### Diagnostics

```c
// Run self-test
emergency_handler_self_test(1, NULL); // Level 1 test, no results storage

// Get event statistics
uint32_t event_count;
emergency_handler_get_event_count(&event_count);

// Get last emergency event
emergency_event_t last_event;
emergency_handler_get_last_event(&last_event);
```

## Integration Examples

### Integration with Battery Manager

```c
// In Battery Manager event handler
void battery_event_handler(uint32_t event_type, uint8_t module_id, void *data) {
    // Process battery event internally
    
    // Forward to emergency handler if appropriate
    if (is_emergency_condition(event_type)) {
        emergency_handler_process_battery_event(event_type, module_id, data);
    }
}
```

### Integration with Thermal Monitor

```c
// In Thermal Monitor zone transition handler
void thermal_zone_transition(uint8_t zone_id, float temperature, float threshold) {
    // Handle thermal zone transition internally
    
    // Forward to emergency handler if in critical or emergency zones
    if (zone_id >= THERMAL_ZONE_CRITICAL) {
        emergency_handler_process_thermal_event(zone_id, temperature, threshold);
    }
}
```

### Integration with Communication Systems

```c
// In Modbus/CANBus error handler
void comms_error_handler(uint8_t comms_type, uint16_t device_id, int32_t error_code) {
    // Handle communication error internally
    
    // Forward critical errors to emergency handler
    if (error_code < CRITICAL_ERROR_THRESHOLD) {
        emergency_handler_process_comms_event(comms_type, device_id, error_code);
    }
}
```

## Safety Considerations

The Emergency Handler implements several critical safety features:

1. **Fail-Safe Design**: Conservative defaults for all safety parameters
2. **Multiple Detection Layers**: Redundant monitoring for critical conditions
3. **Priority-Based Response**: Most serious threats trigger strongest responses
4. **Manual Override**: Support for manual emergency triggers
5. **Recovery Limitations**: Prevents endless recovery cycles
6. **Watchdog Protection**: Guards against system freezes and crashes
7. **Persistent Logging**: Maintains history for post-incident analysis

## Implementation Notes

- The Emergency Handler uses a high-priority FreeRTOS task to ensure reliable operation
- All functions are protected with mutexes for thread safety
- Response actions are configurable through the API
- Integration with other components uses dedicated hook functions
- The system supports both automatic and manual recovery methods
- Logging supports console output, SD card storage, and AWS CloudWatch integration
- Self-test functionality helps verify system operation

## Future Enhancements

Potential improvements for future versions:

1. **Machine Learning Integration**: Predictive detection of emerging emergencies
2. **Remote Management**: Remote monitoring and control capabilities
3. **Enhanced Diagnostics**: More detailed self-testing and diagnostics
4. **Pattern Recognition**: Identification of recurring issues
5. **Adaptive Responses**: Self-tuning response mechanisms based on effectiveness
6. **Multi-Core Support**: Optimized operation on both cores of the ESP32-P4
