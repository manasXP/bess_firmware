# Battery Management System (BMS)

## Overview

The Battery Management System (BMS) is a critical component of the BESS firmware responsible for monitoring and controlling the battery modules. It ensures safe and efficient operation of the LFP battery packs, including monitoring voltages, currents, temperatures, calculating state of charge, controlling cell balancing, and implementing protection features.

## Architecture

The BMS consists of the following main components:

### Core Components

1. **Battery Manager**
   - Central component that coordinates all BMS functions
   - Handles module data collection and aggregation
   - Implements safety checks and limit monitoring
   - Provides external API for other system components

2. **SoC Calculator**
   - Estimates State of Charge using multiple algorithms
   - Supports Coulomb counting, OCV, and hybrid methods
   - Provides temperature compensation
   - Calibration functionality for accurate SoC tracking

3. **Cell Balancer**
   - Manages cell balancing to ensure uniform voltage levels
   - Supports passive and active balancing methods
   - Schedules balancing operations
   - Tracks balancing status and effectiveness

4. **Thermal Monitor**
   - Monitors cell and module temperatures
   - Detects thermal anomalies and runaway conditions
   - Controls cooling systems
   - Implements multi-level thermal protection

## Key Features

- **Real-time Battery Monitoring**: Continuously tracks voltage, current, and temperature across all modules and cells
- **State Estimation**: Calculates State of Charge (SoC) and State of Health (SoH) using advanced algorithms
- **Cell Balancing**: Ensures uniform voltage levels across all cells to maximize capacity and lifespan
- **Thermal Management**: Monitors temperatures and controls cooling systems to maintain optimal operating conditions
- **Safety Protection**: Implements multiple layers of protection against over/under voltage, over/under temperature, and over-current
- **Fault Detection**: Advanced diagnostics to detect and respond to abnormal conditions
- **Event Notification**: Provides callbacks for system events to allow coordinated responses
- **Configurable Parameters**: Adjustable thresholds and operational parameters

## Implementation Details

### Task Structure

The BMS implementation uses multiple FreeRTOS tasks:

1. **Monitoring Task**: High-priority task that continuously monitors battery parameters
   - Runs at `BESS_BMS_TASK_INTERVAL_MS` (typically 500ms)
   - Updates all module data
   - Performs safety checks
   - Updates system status

2. **Balancing Task**: Manages cell balancing operations
   - Runs at a lower frequency than monitoring (typically 2.5s)
   - Determines when balancing is needed
   - Controls the balancing process
   - Updates balancing status

3. **Diagnostics Task**: Performs periodic diagnostic checks
   - Runs at a much lower frequency (typically 5 minutes)
   - Analyzes system trends
   - Performs consistency checks
   - Identifies potential issues before they become critical

### Data Synchronization

The BMS uses multiple synchronization mechanisms:

- **Mutex (`s_data_mutex`)**: Protects shared data structures during concurrent access
- **Event Group (`s_bms_event_group`)**: Coordinates task operations and signals system events
- **Event Callbacks**: Allows external components to register for event notifications

### Safety Features

The BMS implements several safety features:

- **Limit Checking**: Monitors parameters against safe operating limits
- **Fault Detection**: Identifies abnormal conditions and issues appropriate warnings/errors
- **Emergency Response**: Implements appropriate responses to critical conditions
- **Thermal Runaway Detection**: Specialized algorithms to detect potential thermal runaway conditions
- **Redundant Monitoring**: Multiple sensors and cross-checks for critical parameters

## API Interface

The Battery Manager exposes a comprehensive API for other system components:

### Information Retrieval

- `battery_manager_get_module_data()`: Get detailed data for all modules
- `battery_manager_get_system_status()`: Get aggregated battery system status
- `battery_manager_get_soc()`: Get current State of Charge
- `battery_manager_get_soh()`: Get current State of Health

### Control Functions

- `battery_manager_start_balancing()`: Start cell balancing process
- `battery_manager_stop_balancing()`: Stop cell balancing process
- `battery_manager_set_charging_params()`: Configure charging parameters
- `battery_manager_set_discharging_params()`: Configure discharging parameters
- `battery_manager_standby()`: Put BMS in standby mode
- `battery_manager_resume()`: Resume normal operation from standby
- `battery_manager_calibrate()`: Calibrate battery sensors

### Status Checking

- `battery_manager_is_charging_allowed()`: Check if charging is permitted
- `battery_manager_is_discharging_allowed()`: Check if discharging is permitted
- `battery_manager_run_diagnostics()`: Run comprehensive diagnostics

### Event Management

- `battery_manager_register_event_callback()`: Register for event notifications
- `battery_manager_unregister_event_callback()`: Unregister from event notifications

## Event Types

The BMS generates various events that other components can subscribe to:

- `BESS_EVENT_ERROR`: Error conditions that require attention
- `BESS_EVENT_WARNING`: Warning conditions that don't require immediate action
- `BESS_EVENT_STATE_CHANGE`: Changes in system operational state
- `BESS_EVENT_MODULE_STATUS`: Changes in module status
- `BESS_EVENT_SOC_THRESHOLD`: SoC threshold crossings
- `BESS_EVENT_TEMPERATURE_THRESHOLD`: Temperature threshold crossings

## Configuration Parameters

Key BMS parameters are defined in `bess_config.h`:

- **Module Specifications**: Voltage, capacity, cell count
- **Safety Limits**: Temperature, voltage, and current limits
- **BMS Parameters**: Balancing thresholds, SoC limits
- **Task Configuration**: Priority, stack size, core assignment

## Code Structure

The BMS implementation consists of the following files:

1. **battery_manager.h/c**: Main BMS implementation and API
2. **soc_calculator.h/c**: SoC calculation algorithms
3. **cell_balancer.h/c**: Cell balancing functionality
4. **thermal_monitor.h/c**: Temperature monitoring and management

## Future Enhancements

- **Machine Learning**: Add ML-based SoC/SoH prediction models
- **Adaptive Balancing**: Implement adaptive balancing strategies based on cell characteristics
- **Aging Models**: Incorporate battery aging models for better SoH estimation
- **Advanced Diagnostics**: Add more sophisticated diagnostic capabilities
- **Power Prediction**: Add power availability prediction based on historical usage patterns
