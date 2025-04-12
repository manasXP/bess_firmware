# Cell Balancer Module

## Overview

The Cell Balancer is a critical component of the Battery Management System (BMS) for the 100KW/200KWH BESS. It ensures uniform voltage levels across all cells within the LFP battery modules (48V, 16KWH), which is essential for maximizing capacity utilization, extending battery lifespan, and preventing premature cell deterioration.

## Key Features

### Multiple Balancing Modes

The Cell Balancer supports three distinct balancing strategies:

1. **Passive Balancing**
   - Uses resistive elements to discharge higher-voltage cells
   - Energy-efficient for small imbalances
   - Suitable for maintenance balancing
   - Relatively slow but low-cost implementation

2. **Active Balancing**
   - Transfers energy from higher-voltage to lower-voltage cells
   - Higher efficiency with minimal energy loss
   - Faster balancing performance
   - Ideal for correcting significant imbalances

3. **Hybrid Approach**
   - Intelligently combines passive and active methods
   - Uses active balancing for significant imbalances
   - Switches to passive for fine-tuning and maintenance
   - Optimizes for both efficiency and performance

### Configurable Parameters

The Cell Balancer offers extensive configurability to adapt to different operating conditions:

- **Voltage Threshold**: Minimum voltage difference to trigger balancing (configurable in mV)
- **Balancing Current**: Control of maximum current for balancing operations (mA)
- **Minimum Cell Voltage**: Safety threshold to prevent over-discharge during balancing
- **Maximum Balance Time**: Time limit for continuous balancing to prevent overheating
- **Rest Periods**: Configurable cool-down periods between balancing cycles
- **Cells Per Module**: Adaptable to different module configurations

### Safety Features

Built-in safeguards ensure reliable operation under all conditions:

- **Emergency Stop**: Immediate cessation of all balancing activity upon thermal or other safety triggers
- **Timeout Protection**: Automatic termination of excessively long balancing sessions
- **Cell Voltage Monitoring**: Prevents balancing below safe voltage thresholds
- **Thermal Integration**: Works with thermal monitoring to prevent overheating
- **Event Notifications**: Real-time alerts for critical conditions

### Status Monitoring

Comprehensive visibility into balancing status:

- **Cell Voltage Tracking**: Continuous monitoring of individual cell voltages
- **Imbalance Detection**: Automatic identification of voltage disparities
- **Balancing Progress Estimation**: Real-time feedback on balancing completion percentage
- **Cell Identification**: Tracking of highest and lowest voltage cells
- **Activity Mapping**: Bitmap representation of cells currently undergoing balancing

### Event System

Flexible notification system enables integration with other BMS components:

- **Balancing Start/Stop Events**: Notifications when balancing operations begin or end
- **Completion Events**: Alerts when balancing successfully equalizes cell voltages
- **Emergency Notifications**: Immediate signaling of safety-related interruptions
- **State Change Alerts**: Notifications for paused/resumed operations
- **Timeout Events**: Alerts when balancing exceeds maximum duration

## Integration Guide

### Initialization

Initialize the cell balancer as part of the BMS startup sequence:

```c
// Create default configuration
cell_balancer_config_t config = {
    .voltage_threshold_mv = 50,    // 50mV threshold
    .balance_current_ma = 100,     // 100mA balance current
    .min_cell_voltage_mv = 2800,   // 2.8V minimum cell voltage
    .max_balance_time_ms = 3600000, // 1 hour maximum
    .rest_time_ms = 300000,        // 5 minutes rest time
    .cells_per_module = 16,        // 16 cells per 48V module
    .mode = 1                      // Passive balancing mode
};

// Initialize with 12 modules
esp_err_t result = cell_balancer_init(&config, 12);
if (result != ESP_OK) {
    // Handle initialization error
}
```

### Regular Operation

During normal system operation, continuously update cell data:

```c
// In the BMS monitoring task
void bms_monitor_task(void *arg) {
    while (1) {
        // For each module
        for (uint8_t module = 0; module < module_count; module++) {
            // Get cell data from monitoring hardware
            bess_cell_data_t cell_data[16];
            uint8_t cell_count = get_cell_data(module, cell_data);
            
            // Update cell balancer with latest readings
            cell_balancer_update_cell_data(module, cell_data, cell_count);
        }
        
        // Task delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### Event Handling

Register for balancing events to coordinate with other BMS functions:

```c
// Event callback function
void balancing_event_handler(cell_balancer_event_t event, 
                           uint8_t module_id, 
                           void *arg) {
    switch (event) {
        case CELL_BALANCER_EVENT_STARTED:
            ESP_LOGI(TAG, "Balancing started for module %d", module_id);
            // Notify other BMS components
            break;
            
        case CELL_BALANCER_EVENT_COMPLETED:
            ESP_LOGI(TAG, "Balancing completed for module %d", module_id);
            // Update system status
            break;
            
        case CELL_BALANCER_EVENT_EMERGENCY_STOP:
            ESP_LOGW(TAG, "Emergency stop for module %d", module_id);
            // Trigger system alarm
            break;
            
        // Handle other events...
    }
}

// Register the callback
cell_balancer_register_callback(balancing_event_handler, NULL);
```

### Status Monitoring

Retrieve detailed balancing status for system monitoring:

```c
// Get balancing status for a module
cell_balancer_status_t status;
esp_err_t result = cell_balancer_get_status(module_id, &status);

if (result == ESP_OK) {
    // Use status information
    printf("Module %d: %.1f%% balanced, highest cell: %.3fV, lowest cell: %.3fV\n",
           module_id, status.balance_progress,
           status.highest_cell_voltage, status.lowest_cell_voltage);
}
```

### Safety Integration

Integrate with thermal monitoring for safety:

```c
// In thermal monitoring alert handler
void thermal_alert_handler(uint8_t module_id, float temperature) {
    if (temperature > TEMPERATURE_WARNING_THRESHOLD) {
        // Pause balancing during high temperature
        cell_balancer_pause();
    }
    
    if (temperature > TEMPERATURE_CRITICAL_THRESHOLD) {
        // Emergency stop for severe overheating
        cell_balancer_set_emergency_stop(true);
    }
}
```

## Technical Details

### Resource Requirements

- **Memory**: Approximately 4 bytes per cell for balancing status
- **CPU**: Minimal processing requirements for passive mode; moderate for active/hybrid
- **Task Priority**: Typically lower priority than voltage/current monitoring

### Thread Safety

All functions are protected with semaphores for thread-safe operation in the multitasking FreeRTOS environment.

### Error Handling

Comprehensive validation includes:
- Parameter boundary checking
- State validation
- Mutex timeout handling
- Module ID validation

### Performance Considerations

- **Passive Balancing**: Generates heat, use with adequate cooling
- **Active Balancing**: Higher efficiency but requires more complex hardware
- **Balancing Current**: Higher currents balance faster but generate more heat
- **Balancing Time**: Typical full balancing may take 1-8 hours depending on imbalance severity

## Future Enhancements

Potential improvements for future firmware versions:

1. **Adaptive Balancing Algorithms**: Self-tuning parameters based on cell characteristics
2. **Predictive Balancing**: Anticipating imbalance trends before they become significant
3. **Statistical Tracking**: Long-term analysis of cell imbalance patterns
4. **Temperature-Aware Balancing**: Adjusting balancing current based on thermal conditions
5. **Machine Learning Integration**: Optimizing balancing strategies based on usage patterns
6. **Advanced Cell Modeling**: Incorporating electrochemical models for more precise balancing

## Integration with BESS System

The Cell Balancer is designed to operate seamlessly with other BMS components, particularly:

- **SoC Calculator**: Provides more accurate SoC estimates with balanced cells
- **Thermal Monitor**: Coordinates operations based on temperature constraints
- **Battery Manager**: Receives commands and provides status information
- **Protection Systems**: Interacts with safety mechanisms for fault handling
