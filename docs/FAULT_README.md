# Fault Detection System

## Overview

The Fault Detection System is a critical component of the Battery Energy Storage System (BESS) firmware for the 100KW/200KWH installation with LFP battery modules (48V, 16KWH). It provides comprehensive monitoring, detection, and response capabilities for a wide range of fault conditions to ensure safe and reliable operation of the entire system.

## Architecture

The fault detection system consists of the following main components:

### Core Components

1. **Fault Detector**
   - Central component that coordinates all fault detection functions
   - Monitors system parameters against configurable thresholds
   - Maintains fault status and history
   - Implements fault response actions
   - Notifies registered components about fault events

2. **Fault Thresholds**
   - Defines conditions that trigger fault detection
   - Configurable thresholds and hysteresis values
   - Classification by fault type and severity
   - Response action mapping

3. **Fault Event System**
   - Fault event queue for asynchronous processing
   - Callback registration mechanism
   - Event notification to registered components

4. **Fault Logger**
   - Multi-destination logging (console, SD card, AWS CloudWatch)
   - Severity-based log filtering
   - Timestamp and context information

## Key Features

- **Real-time Fault Monitoring**: Continuously monitors system parameters and detects fault conditions
- **Configurable Thresholds**: Adjustable thresholds and hysteresis values for all fault types
- **Severity Classification**: Ranks faults by severity (Info, Warning, Error, Critical, Emergency)
- **Automatic Response**: Configurable actions triggered by fault conditions
- **Fault Latching**: Keeps track of faults until explicitly cleared or auto-reset condition
- **Fault Persistence**: Configurable fault persistence to prevent false alarms
- **Consecutive Reading Requirements**: Configurable number of consecutive readings to trigger a fault
- **Self-Testing**: Built-in self-test capability to verify fault detection functionality
- **Event Notification**: Callback system allowing other components to be notified of fault events
- **Cloud Reporting**: Optional reporting of faults to AWS CloudWatch

## Fault Types

The system detects and responds to the following categories of faults:

### Voltage-related Faults
- Cell undervoltage/overvoltage
- Module undervoltage/overvoltage
- System undervoltage/overvoltage
- Voltage imbalance

### Current-related Faults
- Charge overcurrent
- Discharge overcurrent
- Short circuit
- Current imbalance

### Temperature-related Faults
- Cell undertemperature/overtemperature
- Module undertemperature/overtemperature
- Thermal runaway
- Temperature sensor failure

### System-level Faults
- Insulation failure
- Contactor failure
- Precharge failure
- BMS communication error
- External communication error
- Watchdog timeout
- Memory corruption

### Connectivity Faults
- CAN bus error
- Modbus error

### Sensor Faults
- Voltage sensor failure
- Current sensor failure

### Physical Faults
- Enclosure breach
- Water leak
- Smoke detection

## Default Fault Thresholds

The fault detector is configured with the following default thresholds:

| Fault Type | Threshold | Hysteresis | Severity | Default Action |
|------------|-----------|------------|----------|----------------|
| Cell Undervoltage | 2.5V | 0.1V | Error | Limit Discharge |
| Cell Overvoltage | 3.65V | 0.1V | Error | Limit Charge |
| Module Undervoltage | 40V | 2V | Error | Limit Discharge |
| Module Overvoltage | 58V | 2V | Error | Limit Charge |
| System Undervoltage | 480V | 20V | Critical | Open Contactors |
| System Overvoltage | 580V | 20V | Critical | Open Contactors |
| Charge Overcurrent | 120A | 10A | Error | Limit Charge |
| Discharge Overcurrent | 120A | 10A | Error | Limit Discharge |
| Short Circuit | 300A | 50A | Emergency | Shutdown |
| Cell Undertemperature | -10°C | 2°C | Warning | Limit Charge |
| Cell Overtemperature | 55°C | 5°C | Critical | Limit Discharge |
| Thermal Runaway | Detection | N/A | Emergency | Shutdown |
| Smoke Detected | Detection | N/A | Emergency | Shutdown |
| Water Leak | Detection | N/A | Critical | Shutdown |

## Response Actions

The system can take the following actions in response to faults:

- **None**: No action taken
- **Log Only**: Record the fault but take no action
- **Alert**: Generate alert but continue operation
- **Limit Charge**: Restrict charging capability
- **Limit Discharge**: Restrict discharging capability
- **Pause Operation**: Temporarily pause system operation
- **Open Contactors**: Isolate battery by opening contactors
- **Shutdown**: Complete system shutdown

## Implementation Details

### Task Structure

The fault detector implementation uses a dedicated FreeRTOS task:

- **Fault Detector Task**: Runs at a configurable interval (default 100ms)
  - Processes fault events from the queue
  - Monitors for auto-reset conditions
  - Performs periodic self-tests
  - Updates fault status

### Data Synchronization

Multiple synchronization mechanisms are used:

- **Mutex**: Protects shared data structures during concurrent access
- **Event Group**: Coordinates task operations
- **Queue**: Handles fault events asynchronously

### Safety Features

The fault detector implements several safety features:

- **Watchdog**: Ensures the fault detector task is running
- **Self-Test**: Verifies fault detection functionality
- **Persistence**: Requires fault condition to persist for a configurable period
- **Consecutive Readings**: Requires multiple consecutive readings above threshold

## API Reference

### Initialization and Control

- `fault_detector_init()`: Initialize the fault detector
- `fault_detector_start()`: Start the fault detection task
- `fault_detector_stop()`: Stop the fault detection task
- `fault_detector_reset_all()`: Reset all active and latched faults
- `fault_detector_reset_fault()`: Reset a specific fault

### Configuration

- `fault_detector_set_threshold()`: Set threshold values for a fault
- `fault_detector_set_response()`: Configure response action for a fault
- `fault_detector_enable_fault()`: Enable or disable a specific fault type
- `fault_detector_load_config()`: Load configuration from file
- `fault_detector_save_config()`: Save configuration to file

### Status and Monitoring

- `fault_detector_get_status()`: Get current fault status
- `fault_detector_is_fault_active()`: Check if a specific fault is active
- `fault_detector_is_severity_active()`: Check if any fault of a specific severity is active
- `fault_detector_get_description()`: Get text description for a fault type

### Event Management

- `fault_detector_report_fault()`: Manually report a fault
- `fault_detector_register_callback()`: Register for fault event notifications
- `fault_detector_unregister_callback()`: Unregister from notifications

### Testing

- `fault_detector_self_test()`: Run self-test on the fault detection system

## Integration Guide

### Initialization

Initialize the fault detector at system startup:

```c
// Create default configuration
fault_detector_config_t config = FAULT_DETECTOR_DEFAULT_CONFIG();

// Optional: Customize configuration
config.monitoring_interval = 200;    // 200ms monitoring interval
config.consecutive_readings = 2;     // 2 consecutive readings to trigger
config.auto_reset = true;            // Auto-reset when condition clears

// Initialize the fault detector
esp_err_t result = fault_detector_init(&config);
if (result != ESP_OK) {
    // Handle initialization error
}

// Start the fault detector
result = fault_detector_start();
if (result != ESP_OK) {
    // Handle start error
}
```

### Registering for Fault Events

Register callbacks to be notified of fault events:

```c
// Callback function
void fault_callback(fault_event_t *event, void *user_data) {
    printf("Fault detected: %s, Severity: %d\n", 
           fault_detector_get_description(event->type),
           event->severity);
    
    // Handle the fault
    switch (event->type) {
        case FAULT_CELL_UNDERVOLTAGE:
            // Handle cell undervoltage
            break;
        
        case FAULT_THERMAL_RUNAWAY:
            // Handle thermal runaway
            break;
            
        // Handle other faults...
    }
}

// Register for all faults (mask=0)
fault_detector_register_callback(fault_callback, 0, NULL);

// Or register for specific faults
uint32_t mask = FAULT_CELL_UNDERVOLTAGE | FAULT_CELL_OVERVOLTAGE;
fault_detector_register_callback(fault_callback, mask, NULL);
```

### Reporting Faults

Manually report faults from other components:

```c
// Create fault event
fault_event_t event = {
    .type = FAULT_INSULATION_FAILURE,
    .severity = FAULT_SEVERITY_CRITICAL,
    .action = FAULT_ACTION_OPEN_CONTACTORS,
    .timestamp = 0,  // 0 = use current time
    .module_id = 2,  // Module 2
    .cell_id = 0xFF, // N/A
    .measured_value = 0.05f,  // 0.05 MOhm
    .threshold_value = 0.5f,  // 0.5 MOhm threshold
    .description = "Insulation resistance below threshold"
};

// Report the fault
fault_detector_report_fault(&event);
```

### Checking Fault Status

Check fault status for system decisions:

```c
// Check if any critical or emergency fault is active
bool critical_fault_active;
fault_detector_is_severity_active(FAULT_SEVERITY_CRITICAL, &critical_fault_active);

if (critical_fault_active) {
    // Take appropriate action
}

// Check if a specific fault is active
bool overtemp_active;
fault_detector_is_fault_active(FAULT_CELL_OVERTEMP, &overtemp_active);

if (overtemp_active) {
    // Handle overtemperature condition
}
```

## Best Practices

1. **Fault Thresholds**: Set appropriate thresholds based on battery specifications and system requirements.

2. **Response Actions**: Configure response actions according to fault severity and system impact.

3. **Consecutive Readings**: Use consecutive readings to prevent transient conditions from triggering faults.

4. **Persistence Time**: Set reasonable persistence times to avoid false alarms.

5. **Hysteresis**: Use appropriate hysteresis values to prevent fault oscillation.

6. **Testing**: Regularly test the fault detection system to ensure proper operation.

7. **Callbacks**: Keep fault callbacks simple and fast to avoid blocking the fault detector task.

8. **Logging**: Configure appropriate log levels and destinations based on deployment environment.

## Future Enhancements

1. **Machine Learning**: Implement predictive fault detection using ML algorithms.

2. **Remote Configuration**: Enable remote configuration of fault thresholds and responses.

3. **Fault Correlation**: Implement correlation analysis to identify related fault patterns.

4. **Expanded Diagnostics**: Add more detailed diagnostic information for fault conditions.

5. **Historical Analysis**: Implement trend analysis for fault frequency and patterns.

6. **Adaptive Thresholds**: Dynamically adjust thresholds based on operating conditions.

7. **Fault Recovery Procedures**: Add automated recovery procedures for specific fault types.
