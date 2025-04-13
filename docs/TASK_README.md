# Task Manager for BESS Firmware

## Overview

The Task Manager is a middleware component for the Battery Energy Storage System (BESS) firmware that provides comprehensive task management capabilities for the ESP32-P4 platform running FreeRTOS. It enables efficient coordination of multiple tasks responsible for battery monitoring, cell balancing, thermal management, communication, and other critical functions of the 100KW/200KWH BESS system.

## Key Features

- **Task Creation and Management**: Create, delete, suspend, and resume tasks with configurable priorities, stack sizes, and core affinity
- **Task Scheduling**: Support for periodic, event-driven, and one-shot task execution patterns
- **Synchronization Primitives**: Comprehensive management of mutexes, semaphores, event groups, and queues
- **Performance Monitoring**: Track execution time, CPU usage, and stack usage for all tasks
- **Safety and Reliability**: Watchdog functionality, stack overflow detection, and automatic task restart
- **Multi-core Support**: Task allocation across both cores of the ESP32-P4
- **Diagnostic Tools**: Status reporting and comprehensive system statistics

## Implementation Details

### Task Types

The Task Manager supports various task types tailored for BESS operations:

```c
typedef enum {
    TASK_TYPE_BATTERY_MONITOR = 0,  // Battery monitoring task
    TASK_TYPE_CELL_BALANCER,        // Cell balancing task
    TASK_TYPE_THERMAL_MONITOR,      // Thermal monitoring task
    TASK_TYPE_SOC_CALCULATOR,       // State of Charge calculation task
    TASK_TYPE_MODBUS_SERVER,        // Modbus communication server task
    TASK_TYPE_CANBUS_INTERFACE,     // CAN Bus interface task
    TASK_TYPE_SYSTEM_PROTECTION,    // System protection and safety task
    TASK_TYPE_DATA_LOGGING,         // Data logging task
    TASK_TYPE_CLOUD_SYNC,           // AWS CloudWatch synchronization task
    TASK_TYPE_USER_INTERFACE,       // User interface task
    TASK_TYPE_DIAGNOSTICS,          // Diagnostics and maintenance task
    TASK_TYPE_CUSTOM,               // Custom user-defined task
    TASK_TYPE_MAX                   // Maximum number of task types
} task_type_t;
```

### Priority Levels

Tasks are assigned priorities according to their criticality:

```c
typedef enum {
    TASK_PRIORITY_CRITICAL = configMAX_PRIORITIES - 1,  // Critical safety tasks
    TASK_PRIORITY_HIGH = configMAX_PRIORITIES - 2,      // High priority tasks (e.g. battery monitoring)
    TASK_PRIORITY_MEDIUM_HIGH = configMAX_PRIORITIES - 3, // Medium-high priority tasks
    TASK_PRIORITY_MEDIUM = configMAX_PRIORITIES - 4,    // Medium priority tasks
    TASK_PRIORITY_LOW = configMAX_PRIORITIES - 5,       // Low priority tasks
    TASK_PRIORITY_LOWEST = configMAX_PRIORITIES - 6     // Lowest priority tasks
} task_priority_t;
```

### System Events

The Task Manager uses event flags for inter-task communication:

```c
typedef enum {
    TASK_EVENT_BATT_DATA_READY = (1 << 0),      // Battery data is ready
    TASK_EVENT_THERMAL_ALERT = (1 << 1),        // Thermal alert triggered
    TASK_EVENT_COMM_REQUEST = (1 << 2),         // Communication request received
    TASK_EVENT_PROTECTION_TRIGGER = (1 << 3),   // Protection system triggered
    TASK_EVENT_LOG_READY = (1 << 4),            // Log data ready to be written
    TASK_EVENT_BALANCE_COMPLETE = (1 << 5),     // Cell balancing completed
    TASK_EVENT_SOC_UPDATED = (1 << 6),          // State of Charge updated
    TASK_EVENT_SYSTEM_ERROR = (1 << 7),         // System error occurred
    TASK_EVENT_USER_DEFINED_1 = (1 << 8),       // User defined event 1
    TASK_EVENT_USER_DEFINED_2 = (1 << 9),       // User defined event 2
    TASK_EVENT_ALL = 0x3FF                      // All events mask
} task_event_t;
```

## Usage Examples

### Initialization

```c
// Create default configuration
task_manager_config_t config = TASK_MANAGER_DEFAULT_CONFIG();

// Customize configuration if needed
config.max_tasks = 30;
config.enable_watchdog = true;
config.watchdog_timeout_ms = 5000;

// Initialize the task manager
esp_err_t result = task_manager_init(&config);
if (result != ESP_OK) {
    // Handle initialization error
    ESP_LOGE(TAG, "Failed to initialize task manager: %s", esp_err_to_name(result));
    return result;
}
```

### Creating a Battery Monitoring Task

```c
// Create a periodic battery monitoring task
task_handle_t *batt_monitor_task;
result = task_manager_create_periodic_task(
    "batt_monitor",                   // Task name
    battery_monitor_task_function,    // Task function
    NULL,                             // Task argument
    4096,                             // Stack size (bytes)
    TASK_PRIORITY_HIGH,               // High priority
    TASK_TYPE_BATTERY_MONITOR,        // Task type
    500,                              // 500ms period
    0,                                // Run on core 0
    &batt_monitor_task                // Task handle output
);

if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create battery monitor task: %s", esp_err_to_name(result));
    return result;
}
```

### Creating Synchronization Objects

```c
// Create an event group for battery events
task_sync_t *battery_events;
result = task_manager_create_sync(
    "battery_events",         // Sync object name
    TASK_SYNC_EVENT_GROUP,    // Type: event group
    &battery_events           // Sync handle output
);

// Create a mutex for data protection
task_sync_t *data_mutex;
result = task_manager_create_sync(
    "data_mutex",             // Sync object name
    TASK_SYNC_MUTEX,          // Type: mutex
    &data_mutex               // Sync handle output
);
```

### Using Event Groups for Task Communication

```c
// In the battery monitoring task:
void battery_monitor_task_function(void *arg) {
    while (1) {
        // Monitor battery parameters
        // ...
        
        // Signal that battery data is ready
        task_manager_set_event(battery_events, TASK_EVENT_BATT_DATA_READY, NULL);
        
        // Wait for next period (handled by task_manager for periodic tasks)
    }
}

// In the SOC calculator task:
void soc_calculator_task_function(void *arg) {
    while (1) {
        // Wait for battery data to be ready
        EventBits_t bits;
        esp_err_t result = task_manager_wait_event(
            battery_events,               // Event group
            TASK_EVENT_BATT_DATA_READY,   // Event to wait for
            true,                         // Clear on exit
            false,                        // Wait for any bit
            5000,                         // 5 second timeout
            &bits                         // Received bits
        );
        
        if (result == ESP_OK) {
            // Process battery data and calculate SOC
            // ...
            
            // Signal SOC update complete
            task_manager_set_event(battery_events, TASK_EVENT_SOC_UPDATED, NULL);
        }
    }
}
```

### Using Mutex for Thread Safety

```c
void update_battery_data(battery_data_t *new_data) {
    // Take mutex with 100ms timeout
    if (task_manager_take_sync(data_mutex, 100) == ESP_OK) {
        // Update shared data structure
        memcpy(&g_battery_data, new_data, sizeof(battery_data_t));
        
        // Release mutex
        task_manager_give_sync(data_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire data mutex for update");
    }
}
```

### Getting Task Statistics

```c
void print_system_stats(void) {
    // Get task manager statistics
    task_stats_t stats;
    esp_err_t result = task_manager_get_stats(&stats);
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Task Statistics:");
        ESP_LOGI(TAG, "  Total tasks: %lu", (unsigned long)stats.total_tasks);
        ESP_LOGI(TAG, "  Running tasks: %lu", (unsigned long)stats.running_tasks);
        ESP_LOGI(TAG, "  Free heap: %lu bytes", (unsigned long)stats.free_heap);
        ESP_LOGI(TAG, "  Task crashes: %lu", (unsigned long)stats.task_crashes);
    }
    
    // Dump detailed task info to logs
    task_manager_dump_info(true);
}
```

## Integration with BESS Components

The Task Manager integrates with other BESS components as follows:

1. **Battery Manager**: Uses high-priority tasks to monitor battery parameters at 500ms intervals
2. **Cell Balancer**: Uses medium-priority tasks with longer periods (typically 2.5s)
3. **Thermal Monitor**: Uses medium-high priority tasks for temperature monitoring and control
4. **Communication Interfaces**: Uses medium-priority tasks for Modbus and CANBus communication
5. **System Protection**: Uses critical priority tasks for immediate response to safety issues
6. **Data Logging**: Uses low-priority tasks for logging to console, SD card, and AWS CloudWatch

## Task Priorities and Scheduling

For optimal system performance, follow these guidelines:

1. **Critical Tasks**: System protection and emergency response (TASK_PRIORITY_CRITICAL)
2. **High Priority Tasks**: Battery monitoring and parameter checks (TASK_PRIORITY_HIGH)
3. **Medium-High Tasks**: Thermal monitoring and management (TASK_PRIORITY_MEDIUM_HIGH)
4. **Medium Tasks**: Communication protocols and normal operations (TASK_PRIORITY_MEDIUM)
5. **Low Priority Tasks**: Data logging, statistics, and non-critical operations (TASK_PRIORITY_LOW)
6. **Lowest Priority Tasks**: Diagnostics and maintenance functions (TASK_PRIORITY_LOWEST)

## Memory Considerations

- Typical task stack sizes: 2048-4096 bytes (adjust based on function complexity)
- Monitor stack high-water marks to optimize memory usage
- The Task Manager itself requires approximately 1KB plus memory for task and sync object arrays

## Error Handling and Recovery

The Task Manager implements several safety features:

1. **Watchdog Monitoring**: Detects task hangs and crashes
2. **Automatic Task Restart**: Recovers from task crashes when auto_restart is enabled
3. **Stack Overflow Detection**: Warns when a task is approaching stack capacity
4. **Error Logging**: Comprehensive error reporting and logging

## Future Enhancements

Planned enhancements for future versions:

1. **Task Profiling**: Advanced performance monitoring and bottleneck detection
2. **Dynamic Task Priorities**: Runtime adjustment of task priorities based on system state
3. **Power Management Integration**: Coordination with ESP32-P4 power management features
4. **Enhanced Diagnostics**: More detailed task performance metrics and analysis tools
5. **Task Dependencies**: Declarative specification of task dependencies and startup sequence
