# Load Controller Module

## Overview

The Load Controller is a crucial component of the Battery Energy Storage System (BESS) Power Management Module. It manages power flow between the battery system, grid connection, and connected loads for a 100KW/200KWH BESS utilizing LFP battery modules (48V, 16KWH each).

The Load Controller ensures efficient and safe operation by handling power setpoints, operational mode transitions, and coordinating with other system components such as the Battery Management System (BMS).

## Key Features

- **Multiple Operation Modes**: Supports various operational modes including standby, charge, discharge, island mode, and emergency conditions
- **Power Flow Management**: Controls bidirectional power flow between battery, grid, and loads
- **Smooth Power Ramping**: Implements controlled power transitions with configurable ramp rates
- **Grid Connection Control**: Manages grid connection/disconnection with safety verifications
- **Island Mode Operation**: Supports off-grid functionality with seamless transitions
- **Safety Monitoring**: Continuous monitoring of operating parameters with protective measures
- **Event Notification System**: Comprehensive callback mechanism for system events
- **Diagnostics**: Self-diagnostic capabilities for maintenance and troubleshooting

## Architecture

The Load Controller is built with a modular architecture consisting of:

### Core Components

1. **Command Processor**
   - Handles operational commands (mode changes, power setpoints, etc.)
   - Validates command parameters
   - Ensures safe transitions between operational states

2. **Power Controller**
   - Manages actual power flow through the hardware
   - Implements power ramping for smooth transitions
   - Monitors actual vs. requested power levels

3. **Grid Interface**
   - Controls grid connection circuitry
   - Manages synchronization requirements
   - Ensures safe grid connection/disconnection

4. **Safety Monitor**
   - Continuously checks safety conditions
   - Triggers emergency responses when needed
   - Coordinates with the BMS for battery-related limits

### Operational Modes

The Load Controller supports the following operational modes:

| Mode | Description | Power Flow |
|------|-------------|------------|
| Standby | System in standby, no active power flow | None |
| Discharge | System supplying power to grid/loads | Battery → Grid/Loads |
| Charge | System consuming power from grid | Grid → Battery |
| Idle | Connected but not actively charging/discharging | None |
| Island | Off-grid operation supplying local loads | Battery → Loads |
| Emergency | Emergency mode triggered by critical conditions | None |
| Maintenance | Special mode for servicing | None |
| Test | Mode for system validation | Varies |

## Implementation Details

### Task Structure

The Load Controller utilizes a dedicated FreeRTOS task:

```c
static void load_controller_task(void *arg);
```

This task handles:
- Command processing from queue
- Power ramping updates
- Status monitoring
- Safety condition verification
- Event notification

### Synchronization Mechanisms

The module employs several synchronization primitives:

- **Mutex (`s_controller.mutex`)**: Protects shared state during concurrent access
- **Event Group (`s_controller.event_group`)**: Signals task operations and system events
- **Command Queue (`s_controller.cmd_queue`)**: Handles incoming commands asynchronously

### Event System

The Load Controller implements a comprehensive event notification system:

- Events include mode changes, grid connections/disconnections, power setpoint changes, etc.
- Multiple callbacks can be registered per event type
- Thread-safe callback invocation with user context data

## Integration with BESS

### Interaction with Battery Management System

The Load Controller works closely with the BMS:
- Obtains battery State of Charge (SoC) to determine charging/discharging allowance
- Respects battery safety limits and conditions
- Coordinates power flow based on battery health and status

### Hardware Interface

The module controls physical hardware through abstracted interfaces:
- Power converter control
- Grid connection contactors
- Power measurement systems
- Protection circuits

## API Reference

### Initialization and Control

```c
esp_err_t load_controller_init(const load_controller_config_t *config);
esp_err_t load_controller_deinit(void);
esp_err_t load_controller_start(void);
esp_err_t load_controller_stop(void);
```

### Mode and Power Control

```c
esp_err_t load_controller_set_mode(load_controller_mode_t mode);
esp_err_t load_controller_get_mode(load_controller_mode_t *mode);
esp_err_t load_controller_set_power(float power_kw);
esp_err_t load_controller_get_status(load_controller_status_t *status);
```

### Grid and Converter Control

```c
esp_err_t load_controller_connect_grid(void);
esp_err_t load_controller_disconnect_grid(void);
esp_err_t load_controller_enter_island_mode(void);
esp_err_t load_controller_exit_island_mode(void);
esp_err_t load_controller_enable_converter(void);
esp_err_t load_controller_disable_converter(void);
```

### Safety and Diagnostics

```c
esp_err_t load_controller_emergency_shutdown(uint32_t reason);
esp_err_t load_controller_is_charging_allowed(bool *allowed);
esp_err_t load_controller_is_discharging_allowed(bool *allowed);
esp_err_t load_controller_set_power_limits(float max_charge_kw, float max_discharge_kw);
esp_err_t load_controller_run_diagnostics(void *results);
```

### Event Management

```c
esp_err_t load_controller_register_event_callback(
    load_controller_event_t event,
    load_controller_event_callback_t callback,
    void *user_data
);

esp_err_t load_controller_unregister_event_callback(
    load_controller_event_t event,
    load_controller_event_callback_t callback
);
```

## Configuration Parameters

The following parameters can be configured through the `load_controller_config_t` structure:

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `max_charge_power_kw` | Maximum charging power | 100.0 kW |
| `max_discharge_power_kw` | Maximum discharging power | 100.0 kW |
| `min_power_setpoint_kw` | Minimum power setpoint | 0.5 kW |
| `ramp_rate_kw_per_sec` | Power ramp rate | 10.0 kW/s |
| `power_update_interval_ms` | Power update interval | 100 ms |
| `soc_reserve_percent` | SoC reserve level | 10% |
| `soc_max_percent` | Maximum SoC level | 95% |
| `island_mode_enabled` | Island mode capability | true/false |
| `grid_forming_enabled` | Grid-forming capability | true/false |
| `load_limiting_enabled` | Load limiting capability | true/false |

## Usage Examples

### System Initialization

```c
// Create default configuration
load_controller_config_t config = {
    .max_charge_power_kw = 100.0f,
    .max_discharge_power_kw = 100.0f,
    .min_power_setpoint_kw = 0.5f,
    .ramp_rate_kw_per_sec = 10.0f,
    .power_update_interval_ms = 100,
    .soc_reserve_percent = 10,
    .soc_max_percent = 95,
    .island_mode_enabled = true,
    .grid_forming_enabled = true,
    .load_limiting_enabled = true
};

// Initialize the load controller
esp_err_t result = load_controller_init(&config);
if (result != ESP_OK) {
    // Handle initialization error
}

// Start operation
result = load_controller_start();
if (result != ESP_OK) {
    // Handle start error
}
```

### Power Control

```c
// Set power setpoint (positive for discharge, negative for charge)
esp_err_t result = load_controller_set_power(50.0f);  // 50 kW discharge
if (result == ESP_OK) {
    printf("Setting power to 50 kW discharge\n");
}

// Set to charging mode with 30 kW
result = load_controller_set_mode(LOAD_MODE_CHARGE);
if (result == ESP_OK) {
    result = load_controller_set_power(-30.0f);  // 30 kW charge
}
```

### Grid Management

```c
// Disconnect from grid and enter island mode
esp_err_t result = load_controller_enter_island_mode();
if (result == ESP_OK) {
    printf("Entered island mode\n");
}

// Later, reconnect to grid
result = load_controller_exit_island_mode();
if (result == ESP_OK) {
    printf("Exited island mode, reconnected to grid\n");
}
```

### Event Handling

```c
// Callback function for mode change events
void mode_change_handler(load_controller_event_t event, 
                        uint32_t data, 
                        void *user_data) {
    printf("Mode changed to: %d\n", (load_controller_mode_t)data);
    
    // Handle specific mode transitions
    switch ((load_controller_mode_t)data) {
        case LOAD_MODE_DISCHARGE:
            printf("System now discharging\n");
            break;
            
        case LOAD_MODE_EMERGENCY:
            printf("Emergency mode activated!\n");
            // Trigger system-wide emergency response
            break;
            
        // Handle other modes
    }
}

// Register for mode change events
result = load_controller_register_event_callback(
    LOAD_EVENT_MODE_CHANGE,
    mode_change_handler,
    NULL
);
```

### Safety Checks

```c
// Check if charging is allowed
bool charging_allowed = false;
esp_err_t result = load_controller_is_charging_allowed(&charging_allowed);
if (result == ESP_OK && charging_allowed) {
    printf("Charging is currently allowed\n");
} else {
    printf("Charging is not allowed at this time\n");
}
```

## Safety Features

The Load Controller implements several critical safety features:

1. **Controlled Power Ramping**: Prevents sudden power changes that could damage components
2. **Parameter Validation**: All setpoints and commands are validated against safe limits
3. **SoC Limit Enforcement**: Prevents over-discharge or over-charge of the battery
4. **Emergency Shutdown**: Fast response to critical conditions
5. **Continuous Monitoring**: Regular checks of all operational parameters
6. **Coordinated Protection**: Works with BMS and other subsystems for comprehensive safety

## Error Handling

The module implements robust error handling:

- All API functions return ESP-IDF standard error codes
- Critical errors trigger appropriate safety responses
- Comprehensive logging of error conditions
- Error count tracking for diagnostics

## Future Enhancements

Potential improvements for future firmware versions:

1. **Predictive Control**: Anticipating grid and load conditions for optimized operation
2. **Advanced Island Mode**: Enhanced capabilities for prolonged off-grid operation
3. **Grid Services**: Support for frequency regulation, voltage support, and other grid services
4. **Power Quality Improvements**: Active harmonic filtering and power factor correction
5. **Machine Learning Integration**: Adaptive control based on usage patterns
6. **Enhanced Diagnostics**: More comprehensive self-testing and predictive maintenance

## Dependencies

The Load Controller module depends on:

- ESP-IDF Framework
- FreeRTOS
- Battery Management System (BMS) module
- Hardware abstraction layer for power electronics

## Technical Notes

1. **Thread Safety**: All public API functions are thread-safe
2. **Task Priorities**: The load controller task runs at priority 5
3. **Memory Footprint**: Approximately 4KB of RAM for internal state
4. **Real-time Constraints**: Power updates occur at configurable intervals (typically 100ms)
5. **Fault Tolerance**: Designed to fail safe in all error conditions
