# Safety Management System

## Overview

The Safety Management System is a critical component of the Battery Energy Storage System (BESS) firmware, responsible for monitoring and protecting the 100KW/200KWH BESS with LFP battery modular packs (48V, 16KWH). It provides comprehensive safety oversight by integrating with all BMS subsystems to ensure safe operation under all conditions.

## Core Features

- **Multi-level Safety Protection**: Implements multiple layers of protection against voltage, current, and temperature hazards
- **Real-time Monitoring**: Continuously monitors critical parameters across all battery modules
- **Fault Management**: Detects, records, and responds to fault conditions with appropriate safety actions
- **Emergency Response**: Implements emergency shutdown procedures for critical conditions
- **Event Notification**: Provides callback system for safety events to coordinate system-wide responses
- **Communication Integration**: Supports Modbus and CANBus reporting of safety status and events
- **Comprehensive Logging**: Logs safety events to console, SD Card, and AWS Cloudwatch
- **FreeRTOS Integration**: Uses FreeRTOS tasks and synchronization primitives for reliable real-time operation

## Architecture

The Safety Management System consists of the following main components:

### Core Components

1. **Safety Monitor**
   - Central monitoring task that continuously checks all safety parameters
   - Runs periodic checks for voltage, current, temperature, and SoC
   - Coordinates safety responses across the system
   - Maintains system safety status

2. **Fault Manager**
   - Records and tracks fault conditions
   - Manages fault history and current active faults
   - Provides fault information to other system components
   - Implements fault clearing and reset mechanisms

3. **Safety Action System**
   - Determines appropriate actions based on event type and severity
   - Executes safety actions through coordination with other components
   - Implements progressive response levels from logging to emergency shutdown
   - Provides configurable action thresholds and delays

4. **Thermal Runaway Detection**
   - Specialized algorithms to detect potential thermal runaway conditions
   - Monitors temperature rate of change across battery modules
   - Triggers emergency responses for critical thermal conditions
   - Maintains temperature history for trend analysis

## Implementation Details

### Key Files

1. **safety_manager.h**: Interface definition for the Safety Management System
2. **safety_manager.c**: Implementation of the Safety Management System

### Data Structures

1. **Safety Status Enum**: Represents overall system safety status levels
   ```c
   typedef enum {
       SAFETY_STATUS_OK = 0,           // All systems operating normally
       SAFETY_STATUS_WARNING,          // Warning conditions detected
       SAFETY_STATUS_ALARM,            // Alarm conditions requiring attention
       SAFETY_STATUS_CRITICAL,         // Critical conditions requiring immediate action
       SAFETY_STATUS_EMERGENCY,        // Emergency conditions requiring shutdown
       SAFETY_STATUS_FAULT,            // System fault detected
       SAFETY_STATUS_MAINTENANCE,      // System in maintenance mode
       SAFETY_STATUS_UNKNOWN           // System status unknown or initializing
   } safety_status_t;
   ```

2. **Safety Event Types**: Defines all possible safety events that can be detected
   ```c
   typedef enum {
       SAFETY_EVENT_VOLTAGE_HIGH,          // Module voltage exceeds high threshold
       SAFETY_EVENT_VOLTAGE_LOW,           // Module voltage below low threshold
       SAFETY_EVENT_CELL_VOLTAGE_HIGH,     // Cell voltage exceeds high threshold
       SAFETY_EVENT_CELL_VOLTAGE_LOW,      // Cell voltage below low threshold
       SAFETY_EVENT_CURRENT_HIGH,          // Current exceeds high threshold
       SAFETY_EVENT_CURRENT_REVERSE,       // Reverse current detected
       SAFETY_EVENT_TEMPERATURE_HIGH,      // Temperature exceeds high threshold
       SAFETY_EVENT_TEMPERATURE_LOW,       // Temperature below low threshold
       SAFETY_EVENT_THERMAL_RUNAWAY,       // Thermal runaway condition detected
       // ... additional events
   } safety_event_t;
   ```

3. **Safety Action Types**: Defines possible actions in response to safety events
   ```c
   typedef enum {
       SAFETY_ACTION_NONE = 0,             // No action required
       SAFETY_ACTION_LOG_ONLY,             // Log the event only
       SAFETY_ACTION_ALERT_OPERATOR,       // Alert the operator
       SAFETY_ACTION_LIMIT_CURRENT,        // Limit system current
       SAFETY_ACTION_INCREASE_COOLING,     // Increase cooling system power
       SAFETY_ACTION_PAUSE_CHARGING,       // Pause charging operations
       SAFETY_ACTION_PAUSE_DISCHARGING,    // Pause discharging operations
       // ... additional actions
   } safety_action_t;
   ```

4. **Configuration Structure**: Defines all configurable parameters for the safety system
   ```c
   typedef struct {
       /* System Configuration */
       uint8_t number_of_modules;          // Number of battery modules
       uint8_t cells_per_module;           // Number of cells per module
       
       /* Voltage Limits */
       float module_voltage_high;          // High voltage threshold for modules (V)
       float module_voltage_low;           // Low voltage threshold for modules (V)
       // ... additional parameters
   } safety_config_t;
   ```

5. **Fault Structure**: Records detailed information about detected faults
   ```c
   typedef struct {
       safety_event_t event;               // Type of event that caused the fault
       uint8_t module_id;                  // Module ID where fault occurred
       uint32_t timestamp;                 // Timestamp when fault was detected
       float measured_value;               // Measured value that triggered the fault
       float threshold_value;              // Threshold value that was exceeded
       // ... additional fields
   } safety_fault_t;
   ```

### Task Structure

The Safety Management System uses a dedicated FreeRTOS task for continuous monitoring:

1. **Monitor Task**: High-priority task that continuously checks all safety parameters
   - Runs at configurable intervals (typically 500ms)
   - Performs voltage, current, temperature, and SoC checks
   - Executes safety actions in response to detected events
   - Updates system status and reports through communication channels

### Synchronization Mechanisms

To ensure thread-safe operation in a multi-tasking environment, the system uses:

1. **Data Mutex (`s_data_mutex`)**: Protects shared data structures during concurrent access
2. **Event Group (`s_event_group`)**: Coordinates task operations and signals system events
3. **Event Callbacks**: Allows external components to register for event notifications

### Safety Features

The system implements several safety features:

1. **Limit Checking**: Monitors parameters against configurable safe operating limits
2. **Fault Detection**: Identifies abnormal conditions and issues appropriate warnings/errors
3. **Emergency Response**: Implements appropriate responses to critical conditions
4. **Thermal Runaway Detection**: Specialized algorithms to detect potential thermal runaway
5. **Redundant Monitoring**: Multiple checks and cross-validation for critical parameters

## API Reference

### Initialization and Control

- `safety_manager_init()`: Initialize the safety management system
- `safety_manager_start()`: Start the safety monitoring tasks
- `safety_manager_stop()`: Stop the safety monitoring tasks

### Data Update Functions

- `safety_manager_update_voltage_data()`: Update voltage data for a specific module
- `safety_manager_update_current_data()`: Update current data for a specific module
- `safety_manager_update_temperature_data()`: Update temperature data for a specific module
- `safety_manager_update_soc_data()`: Update State of Charge data for a specific module

### Event Management

- `safety_manager_register_callback()`: Register for safety event notifications
- `safety_manager_unregister_callback()`: Unregister from safety event notifications
- `safety_manager_trigger_event()`: Manually trigger a safety event for testing

### Status and Fault Management

- `safety_manager_get_status()`: Get current system safety status
- `safety_manager_get_faults()`: Get list of current faults
- `safety_manager_clear_fault()`: Clear a specific fault
- `safety_manager_clear_all_faults()`: Clear all faults in the system
- `safety_manager_self_diagnostic()`: Perform safety system self-diagnostic

### System Control Functions

- `safety_manager_set_log_destination()`: Configure log destinations
- `safety_manager_configure_contactor_control()`: Set automatic/manual contactor control
- `safety_manager_emergency_shutdown()`: Force emergency shutdown
- `safety_manager_update_threshold()`: Dynamically update safety thresholds

### Permission Checking

- `safety_manager_is_charging_allowed()`: Check if charging is permitted
- `safety_manager_is_discharging_allowed()`: Check if discharging is permitted
- `safety_manager_is_in_failsafe()`: Check if system is in failsafe mode

## Usage Examples

### Initialization

```c
// Create default configuration
safety_config_t config = {
    .number_of_modules = 12,            // 12 battery modules
    .cells_per_module = 16,             // 16 cells per module (48V)
    
    .module_voltage_high = 54.0f,       // Maximum module voltage (V)
    .module_voltage_low = 42.0f,        // Minimum module voltage (V)
    .cell_voltage_high = 3.65f,         // Maximum cell voltage (V)
    .cell_voltage_low = 2.5f,           // Minimum cell voltage (V)
    
    .current_high_charge = 200.0f,      // Maximum charging current (A)
    .current_high_discharge = 250.0f,   // Maximum discharging current (A)
    
    .temperature_high = 45.0f,          // High temperature threshold (°C)
    .temperature_critical = 60.0f,      // Critical temperature threshold (°C)
    .temperature_low = 0.0f,            // Low temperature threshold (°C)
    .temp_rate_runaway = 2.0f,          // Thermal runaway indicator (°C/s)
    
    .soc_high = 95.0f,                  // High SoC threshold (%)
    .soc_low = 5.0f,                    // Low SoC threshold (%)
    
    .monitoring_interval_ms = 500,      // Safety check interval (ms)
    .log_destination = SAFETY_LOG_ALL,  // Log to all destinations
    
    .enable_modbus_reporting = true,    // Enable Modbus reporting
    .enable_canbus_reporting = true,    // Enable CANBus reporting
    
    .max_fault_count = 32               // Maximum number of faults to track
};

// Initialize the safety system
esp_err_t result = safety_manager_init(&config);
if (result == ESP_OK) {
    // Start the safety monitoring system
    safety_manager_start();
}
```

### Updating Safety Data

```c
// Update voltage data for a module
float cell_voltages[16] = {3.3f, 3.31f, 3.29f, /* ... */};
safety_manager_update_voltage_data(module_id, 52.8f, cell_voltages, 16);

// Update current data
safety_manager_update_current_data(module_id, 125.5f);  // 125.5A discharge

// Update temperature data
float temperatures[8] = {32.1f, 33.2f, 31.9f, /* ... */};
safety_manager_update_temperature_data(module_id, temperatures, 8);

// Update SoC data
safety_manager_update_soc_data(module_id, 78.5f);  // 78.5% SoC
```

### Registering for Event Callbacks

```c
// Callback function for safety events
void safety_event_handler(safety_event_t event, 
                         uint8_t module_id, 
                         float measured_value, 
                         float threshold_value, 
                         void *user_data) {
    printf("Safety event detected: Event=%d, Module=%d, Value=%.2f, Threshold=%.2f\n",
           event, module_id, measured_value, threshold_value);
    
    // Take appropriate action based on event type
    switch (event) {
        case SAFETY_EVENT_TEMPERATURE_HIGH:
            // Notify thermal management system
            break;
        case SAFETY_EVENT_CELL_VOLTAGE_LOW:
            // Notify power management system
            break;
        // Handle other events...
    }
}

// Register for all safety events
safety_manager_register_callback(SAFETY_EVENT_COUNT, safety_event_handler, NULL);

// Or register for specific events
safety_manager_register_callback(SAFETY_EVENT_THERMAL_RUNAWAY, safety_event_handler, NULL);
```

### Checking System Status

```c
// Check if charging is allowed
bool charging_allowed;
safety_event_t limiting_event;
safety_manager_is_charging_allowed(&charging_allowed, &limiting_event);

if (!charging_allowed) {
    printf("Charging not allowed due to event: %d\n", limiting_event);
}

// Check if system is in failsafe mode
bool in_failsafe;
safety_event_t critical_event;
safety_manager_is_in_failsafe(&in_failsafe, &critical_event);

if (in_failsafe) {
    printf("System in failsafe mode due to event: %d\n", critical_event);
}
```

### Fault Management

```c
// Get current faults
safety_fault_t faults[32];
uint8_t fault_count = 0;
safety_manager_get_faults(faults, 32, &fault_count);

printf("Found %d active faults\n", fault_count);
for (uint8_t i = 0; i < fault_count; i++) {
    printf("Fault %d: Event=%d, Module=%d, Value=%.2f, Active=%d\n",
           i, faults[i].event, faults[i].module_id, 
           faults[i].measured_value, faults[i].is_active);
}

// Clear a specific fault
safety_manager_clear_fault(SAFETY_EVENT_VOLTAGE_HIGH, module_id);

// Clear all faults
safety_manager_clear_all_faults();
```

## Integration with Other BMS Components

The Safety Management System integrates with other BMS components:

1. **Battery Manager**
   - Provides safety status information
   - Controls charge/discharge permissions
   - Coordinates system-wide safety responses

2. **Cell Balancer**
   - Pauses balancing during safety events
   - Monitors for balancing faults
   - Coordinates with thermal protection

3. **SoC Calculator**
   - Monitors SoC for safety thresholds
   - Provides data for safety decisions
   - Uses safety limits to adjust calculations

4. **Thermal Monitor**
   - Coordinates thermal protection responses
   - Provides data for thermal runaway detection
   - Implements multi-level thermal safety

## Logging System

The Safety Management System supports logging to multiple destinations:

1. **Console Logging**: Standard ESP-IDF logging to console
2. **SD Card Logging**: Persistent logging to SD card for post-incident analysis
3. **AWS Cloudwatch**: Remote logging to AWS Cloudwatch for monitoring and analytics

## Communication Protocols

Safety information is communicated through multiple channels:

1. **Modbus**: Industry standard for SCADA and control systems
2. **CANBus**: Real-time communication with other system components

## Performance Considerations

1. **Memory Usage**: Approximately 4KB for module data, plus fault history
2. **CPU Utilization**: Less than 5% on ESP32-P4 at 500ms monitoring interval
3. **Response Time**: Typical response to critical events under 100ms
4. **Task Priority**: High priority (configMAX_PRIORITIES - 2) to ensure timely response

## Future Enhancements

Potential improvements for future firmware versions:

1. **Machine Learning Integration**: Predictive safety algorithms using ML models
2. **Advanced Diagnostics**: More sophisticated diagnostic capabilities
3. **Remote Management**: Enhanced remote monitoring and control capabilities
4. **Adaptive Thresholds**: Self-adjusting thresholds based on operating conditions
5. **Detailed Analytics**: Comprehensive safety performance analytics
