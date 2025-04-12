# Power Management System

## Overview

The Power Management System is a critical component of the 100KW/200KWH Battery Energy Storage System (BESS) firmware. It's responsible for managing power flow between the battery, grid, and load, enabling efficient energy transfer while maintaining system safety and stability.

This component provides comprehensive power management capabilities for the LFP battery modules (48V, 16KWH), supporting controlled charging and discharging operations, grid interaction, and energy scheduling.

## Core Features

### Operation Modes

The system supports multiple operational modes:

1. **Standby Mode**: System is powered but no active power flow is occurring
2. **Charge Mode**: Battery is actively charging from grid or other source
3. **Discharge Mode**: Battery is actively discharging to grid or local load
4. **Idle Mode**: System is ready but not actively charging or discharging
5. **Emergency Mode**: System has detected a critical condition and halted operations
6. **Maintenance Mode**: Special mode for servicing and maintenance operations

### Power Flow Management

- **Bidirectional Control**: Manages power flow in both directions (charge/discharge)
- **Ramping Control**: Smooth power transitions to prevent grid instability
- **Power Limiting**: Dynamic power limits based on system state, battery SoC, and configuration
- **Flow Direction**: Tracks and manages multiple power flow paths:
  - Grid to Battery (charging)
  - Battery to Grid (discharging)
  - Grid to Load (pass-through)
  - Battery to Load (local power)
  - Mixed modes

### Grid Interaction

The system supports multiple grid connection types:

- **Isolated**: No grid connection (island mode)
- **Backup**: Grid backup only
- **Interactive**: Full bidirectional grid interaction
- **Export**: Export to grid only

### Charging Management

- **Multiple Charge Methods**: Supports CC-CV, pulsed, multi-stage, and trickle charging
- **Parameterized Control**: Configurable current, voltage, and power limits
- **Target-Based Operation**: Charging to specific SoC targets
- **Timeout Protection**: Automatic termination of extended charging sessions

### Discharging Management

- **Configurable Limits**: Current, power, and minimum voltage thresholds
- **SoC Protection**: Prevents discharge below minimum safe SoC
- **Constant Power**: Maintains stable power output during discharge
- **Duration Control**: Timed discharge operations with automatic termination

### Scheduling System

- **Time-Based Scheduling**: Set operations to occur at specific times
- **Day-of-Week Support**: Configure schedules for specific days or all days
- **Multiple Schedule Entries**: Up to 16 independent schedule entries
- **Flexible Parameters**: Each schedule can specify mode, power level, and duration
- **Activation Management**: Automatic activation and deactivation of scheduled operations

### Safety Features

- **Emergency Shutdown**: Immediate termination of all power operations
- **Limit Enforcement**: Strict enforcement of system power limits
- **Ramping Protection**: Controlled power changes to prevent system stress
- **State Validation**: Protection against invalid state transitions
- **Grid Connection Safety**: Grid-aware operations with connection type validation

### Event System

- **Comprehensive Events**: Mode changes, flow direction changes, schedule activations, etc.
- **Callback Registration**: Multiple components can register for event notifications
- **User Data Support**: Callbacks receive context data for informed decision-making
- **Selective Registration**: Components can listen for specific events or all events

## Implementation Details

### Architecture

The Power Management System is implemented with a modular architecture:

- **Core Manager**: Coordinates all power operations and state transitions
- **Monitor Task**: Continuously monitors system state and performs scheduled operations
- **Event System**: Distributes notifications of significant system events
- **Scheduler**: Manages timed operations based on configured schedules

### Technical Features

- **Thread Safety**: All functions are protected with mutexes for concurrent access
- **Power Ramping**: Controlled power transitions with configurable ramp rates
- **Event Groups**: FreeRTOS event groups for efficient state signaling
- **Error Handling**: Comprehensive validation and error reporting
- **Logging**: Detailed logging via ESP-IDF's logging system

## API Interface

The Power Manager exposes a comprehensive API for other system components:

### Initialization and Control

```c
// Initialize with configuration
power_manager_config_t config = {
    .grid_connection = GRID_CONNECTION_INTERACTIVE,
    .module_count = 4,
    .system_voltage_nominal = 48.0f * 4,  // 4 modules in series
    .max_charge_power = 100.0f,
    .max_discharge_power = 100.0f,
    .max_grid_export_power = 100.0f,
    .min_soc = 20.0f,
    .max_soc = 90.0f,
    .allow_grid_charging = true,
    .allow_grid_export = true,
    .inverter_efficiency = 95.0f,
    .auto_restart = true
};

esp_err_t result = power_manager_init(&config);

// Start/stop operations
power_manager_start();
power_manager_stop();
```

### Mode and Parameter Configuration

```c
// Set operation mode
power_manager_set_mode(POWER_MODE_CHARGE);

// Configure charging parameters
charge_params_t charge_params = {
    .charge_current_max = 80.0f,
    .charge_voltage_max = 54.0f * 4,  // 4 modules
    .charge_power_max = 80.0f,
    .method = CHARGE_METHOD_CC_CV,
    .enable_balancing = true,
    .timeout_minutes = 120,
    .target_soc = 90.0f
};
power_manager_set_charge_params(&charge_params);

// Configure discharging parameters
discharge_params_t discharge_params = {
    .discharge_current_max = 90.0f,
    .discharge_power_max = 90.0f,
    .min_voltage = 44.0f * 4,  // 4 modules
    .min_soc = 20.0f,
    .timeout_minutes = 120
};
power_manager_set_discharge_params(&discharge_params);
```

### Operations

```c
// Start charging to 80% SoC
power_manager_start_charging(80.0f);

// Start discharging at 50kW for 60 minutes
power_manager_start_discharging(50.0f, 60);

// Stop current operation
power_manager_stop_operation();

// Set constant power (-20kW for charge, 30kW for discharge)
power_manager_set_constant_power(-20.0f, 0);  // Charge at 20kW indefinitely
power_manager_set_constant_power(30.0f, 120);  // Discharge at 30kW for 2 hours
```

### Scheduling

```c
// Add a schedule entry (charge every day at 1:00 AM for 4 hours)
power_schedule_entry_t schedule = {
    .enabled = true,
    .day_of_week = 7,  // All days
    .hour = 1,
    .minute = 0,
    .mode = POWER_MODE_CHARGE,
    .power_setpoint = -40.0f,  // Charge at 40kW
    .duration_minutes = 240,
    .repeat = true
};

uint8_t entry_id;
power_manager_add_schedule(&schedule, &entry_id);

// Enable/disable scheduling
power_manager_enable_scheduling(true);
```

### Event Registration

```c
// Define callback function
void power_event_handler(power_event_t event, void *event_data, void *user_data) {
    switch (event) {
        case POWER_EVENT_MODE_CHANGE:
            printf("Mode changed to %d\n", (int)(intptr_t)event_data);
            break;
        case POWER_EVENT_CHARGE_COMPLETE:
            printf("Charging completed\n");
            break;
        // Handle other events...
    }
}

// Register for specific event
power_manager_register_callback(POWER_EVENT_CHARGE_COMPLETE, 
                               power_event_handler, 
                               NULL);

// Register for all events
power_manager_register_callback(-1, power_event_handler, NULL);
```

### Emergency Handling

```c
// Trigger emergency shutdown
power_manager_emergency_shutdown();

// Reset after emergency condition is resolved
power_manager_reset_emergency();

// Check emergency state
bool is_emergency;
power_manager_is_emergency(&is_emergency);
```

## Integration with BESS System

The Power Management System interacts with other major BESS components:

- **Battery Manager**: Receives battery status and limits for power operations
- **Cell Balancer**: Coordinates with charging to enable cell balancing
- **Thermal Monitor**: Adjusts power levels based on thermal conditions
- **SoC Calculator**: Uses SoC information to enforce limits and optimize operations
- **Communication Interfaces**: Provides status information and receives commands via Modbus and CANBus

## Future Enhancements

Potential improvements for future firmware versions:

1. **Predictive Power Management**: Anticipating load and generation patterns
2. **Advanced Grid Services**: Frequency response, voltage support, and other ancillary services
3. **Machine Learning Integration**: Optimizing power flows based on usage patterns
4. **Multi-Source Management**: Coordinating with solar, wind, or other generation sources
5. **Virtual Power Plant**: Enabling participation in aggregated energy services
6. **Dynamic Tariff Support**: Optimizing operations based on time-of-use pricing
7. **Weather-Aware Scheduling**: Integrating forecast data for optimal operation
