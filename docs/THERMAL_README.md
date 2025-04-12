# Thermal Monitoring System

## Overview

The Thermal Monitoring System is a critical component of the Battery Management System (BMS) for the 100KW/200KWH BESS. It provides comprehensive thermal management capabilities for the LFP battery modules, ensuring safe operating temperatures and protecting against thermal runaway events.

## Core Functionality

### Temperature Monitoring

- Continuously tracks temperature readings from multiple sensors across all battery modules
- Calculates key metrics including maximum, minimum, and average temperatures
- Determines the hottest and coolest modules for targeted management
- Supports up to 8 temperature sensors per module for detailed thermal mapping

### Thermal Zone Classification

The system classifies the thermal state into five distinct zones:

1. **Normal**: Standard operating temperatures, no action required
2. **Elevated**: Temperatures above normal but within safe operating range, monitoring recommended
3. **Warning**: Elevated temperatures requiring attention and potential action
4. **Critical**: High temperatures requiring immediate cooling intervention
5. **Emergency**: Extreme temperatures requiring emergency shutdown procedures

All zone thresholds are configurable to adapt to specific deployment environments and battery characteristics.

### Cooling System Control

- Provides automatic control of cooling systems with power levels from 0-100%
- Supports multiple cooling methods:
  - Passive cooling (natural convection)
  - Forced air cooling (fans)
  - Liquid cooling systems
  - Hybrid approaches combining multiple methods
- Adjusts cooling power proportionally to thermal zones:
  - Elevated: 25% cooling power
  - Warning: 50% cooling power
  - Critical: 75% cooling power
  - Emergency: 100% cooling power

### Thermal Runaway Detection

- Analyzes temperature rise rates to detect potential thermal runaway conditions
- Maintains historical temperature data for trend analysis
- Implements configurable thresholds for runaway detection
- Provides immediate alerts when thermal runaway risk is detected

### Historical Data Management

- Maintains a circular buffer of temperature history for each module
- Supports retrieval of historical data for analysis and diagnostics
- Enables calculation of temperature change rates over time
- Assists in predictive maintenance and thermal pattern recognition

## Advanced Features

### Manual and Automatic Control

- Supports both automatic cooling based on thermal zones and manual control
- Allows setting specific cooling power levels manually
- Provides configurable duration for manual control operations
- Gracefully transitions between manual and automatic modes

### Event Callback System

- Allows other system components to register for notifications when thermal zone transitions occur
- Supports multiple callbacks per thermal zone for distributed response
- Provides context data to callbacks for informed decision-making
- Ensures proper synchronization to prevent race conditions

### Ambient Temperature Tracking

- Maintains reference to ambient temperature for relative temperature analysis
- Supports manual setting of ambient temperature from external sensors
- Provides context for interpreting internal temperature readings
- Assists in determining appropriate cooling responses

### Thread-Safe Design

- All functions are protected with mutexes for safe operation in a multitasking environment
- Implements careful lock management to prevent deadlocks
- Provides timeout mechanisms to prevent indefinite blocking
- Ensures consistent data access across multiple tasks

## Implementation Details

- The component uses a dedicated FreeRTOS task for continuous monitoring and cooling control
- Temperature data is protected by a mutex to ensure thread safety
- An event group is used for signaling system states like cooling activation and emergency conditions
- The internal monitoring task runs at 500ms intervals for timely response to thermal events
- Cooling control decisions are made at 1-second intervals to prevent excessive cycling

## Integration with BMS

This Thermal Monitoring System is designed to work seamlessly with other BMS components:

- Provides thermal status information to the Battery Manager
- Coordinates with Cell Balancer to pause balancing during high temperature conditions
- Feeds temperature data to SoC Calculator for temperature-compensated SoC calculations
- Triggers system-wide alerts and shutdown procedures during thermal emergencies

## Usage Examples

### Initialization

```c
// Initialize with forced air cooling
esp_err_t result = thermal_monitor_init(COOLING_METHOD_FORCED_AIR);
if (result == ESP_OK) {
    // Start the thermal monitoring system
    thermal_monitor_start();
}
```

### Setting Custom Thresholds

```c
// Set custom thermal zone thresholds
thermal_monitor_set_thresholds(
    30.0f,  // Elevated threshold (°C)
    40.0f,  // Warning threshold (°C)
    50.0f,  // Critical threshold (°C)
    60.0f   // Emergency threshold (°C)
);
```

### Manually Controlling Cooling

```c
// Activate cooling at 75% power for 10 minutes
thermal_monitor_activate_cooling(75, 600);

// Later, return to automatic control
thermal_monitor_deactivate_cooling();
```

### Registering for Thermal Events

```c
// Callback function for thermal events
void thermal_event_handler(bess_event_t event, 
                          uint32_t data, 
                          void *user_data) {
    thermal_zone_t zone = (thermal_zone_t)data;
    printf("Entered thermal zone: %d\n", zone);
    
    if (zone >= THERMAL_ZONE_CRITICAL) {
        // Take immediate action for critical conditions
    }
}

// Register for critical thermal zone events
thermal_monitor_register_callback(
    THERMAL_ZONE_CRITICAL,
    thermal_event_handler,
    NULL
);
```

### Checking Thermal Runaway

```c
// Periodically check for thermal runaway conditions
bool runaway_detected;
uint8_t affected_module;

esp_err_t result = thermal_monitor_check_runaway(&runaway_detected, &affected_module);
if (result == ESP_OK && runaway_detected) {
    printf("WARNING: Thermal runaway detected in module %d\n", affected_module);
    // Implement emergency procedures
}
```

## Security and Safety Considerations

The Thermal Monitoring System implements several safety features:

- Fail-safe operation ensuring cooling activation during communication failures
- Redundant temperature monitoring with cross-validation
- Emergency shutdown triggers for extreme temperature conditions
- Graceful degradation during partial sensor failures
- Comprehensive logging of thermal events for post-incident analysis
