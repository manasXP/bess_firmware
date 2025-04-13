# Task Scheduler Module

## Overview

The Task Scheduler is a critical component of the 100KW/200KWH BESS (Battery Energy Storage System) firmware that manages concurrent tasks on the ESP32-P4 platform. It provides a high-level abstraction over FreeRTOS task management, enabling efficient coordination of system activities while ensuring proper resource allocation.

This module is designed to simplify task lifecycle management, standardize priority handling, facilitate core assignment, and monitor system resource utilization across the entire BESS firmware.

## Architecture

The Task Scheduler consists of the following main components:

### Core Components

1. **Task Registry**
   - Maintains a registry of all created tasks
   - Handles task lookup and validation
   - Provides centralized management of task resources
   - Supports up to 32 concurrent tasks

2. **Task Lifecycle Manager**
   - Creates, starts, suspends, resumes, and deletes tasks
   - Manages task states and transitions
   - Handles proper resource cleanup
   - Provides status information for tasks

3. **CPU Utilization Monitor**
   - Tracks CPU usage across cores
   - Calculates percentage utilization
   - Identifies potential system bottlenecks
   - Supports system-wide performance monitoring

4. **Priority Manager**
   - Defines standard priority levels for different task types
   - Provides runtime priority adjustment
   - Ensures consistent priority assignment
   - Prevents priority inversion issues

## Key Features

- **FreeRTOS Integration**: Seamless integration with underlying FreeRTOS task management
- **Multiple Core Support**: Configurable task assignment to specific cores on the ESP32-P4
- **Standardized Priority Levels**: Pre-defined priority levels for different task categories
- **Thread-Safe APIs**: All operations protected by mutex for concurrent access
- **Dynamic Task Management**: Runtime creation, suspension, and deletion of tasks
- **Resource Monitoring**: Real-time tracking of CPU utilization
- **Error Handling**: Comprehensive error reporting and validation
- **Configurable Parameters**: Adjustable stack sizes and other task parameters

## Implementation Details

### Task Structure

Each task in the scheduler is represented by the `scheduler_task_t` structure containing:

- FreeRTOS task handle
- Task name
- Running status flag
- Stack size
- Priority level
- Core assignment

### Priority Levels

The scheduler defines four standard priority levels:

1. **Low (5)**: Background or non-critical tasks
2. **Medium (10)**: Standard operational tasks
3. **High (15)**: Time-sensitive tasks
4. **Critical (20)**: Safety-critical operations

### Core Assignment

Tasks can be assigned to:

- **Core 0**: First core in the ESP32-P4
- **Core 1**: Second core in the ESP32-P4
- **Any Core**: Let FreeRTOS decide which core to use

### Thread Safety

All scheduler operations are protected by a mutex to ensure thread-safe operation in the multi-tasking environment. The mutex implementation uses a timeout to prevent indefinite blocking.

### CPU Utilization Monitoring

The scheduler includes a dedicated task that monitors CPU utilization by:

1. Tracking the idle task runtime on each core
2. Calculating the percentage of non-idle time
3. Updating utilization metrics at regular intervals (1 second)
4. Providing this data to other system components

## API Interface

The Scheduler exposes a comprehensive API for other system components:

### Initialization

- `scheduler_init()`: Initialize the scheduler module
- `scheduler_deinit()`: Clean up resources and shut down

### Task Management

- `scheduler_create_task()`: Create a new task
- `scheduler_start_task()`: Start a previously created task
- `scheduler_suspend_task()`: Temporarily pause a task
- `scheduler_resume_task()`: Resume a suspended task
- `scheduler_delete_task()`: Remove a task and free resources

### Status and Information

- `scheduler_is_task_running()`: Check if a task is currently running
- `scheduler_get_task_count()`: Get the number of tasks
- `scheduler_get_tasks()`: Get information about all tasks
- `scheduler_find_task_by_name()`: Look up a task by name
- `scheduler_get_cpu_utilization()`: Get CPU usage percentages

### Control Functions

- `scheduler_set_task_priority()`: Change a task's priority
- `scheduler_yield()`: Yield execution to other tasks
- `scheduler_delay_ms()`: Delay a task for specified milliseconds

## Usage Examples

### Initializing the Scheduler

```c
// Initialize the scheduler at system startup
esp_err_t result = scheduler_init();
if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize scheduler: %d", result);
    // Handle error
}
```

### Creating and Starting Tasks

```c
// Define a task function
void battery_monitor_task(void *arg) {
    while (1) {
        // Perform battery monitoring operations
        
        // Yield time to other tasks
        scheduler_delay_ms(500);
    }
}

// Create the task
scheduler_task_t *batt_monitor;
esp_err_t result = scheduler_create_task(
    battery_monitor_task,          // Task function
    "batt_monitor",                // Task name
    4096,                          // Stack size (bytes)
    NULL,                          // No task arguments
    SCHEDULER_PRIORITY_HIGH,       // High priority task
    SCHEDULER_CORE_1,              // Assign to core 1
    &batt_monitor                  // Task handle
);

if (result == ESP_OK) {
    // Start the task
    scheduler_start_task(batt_monitor);
}
```

### Managing Task Priority

```c
// Temporarily increase task priority during critical operations
scheduler_set_task_priority(batt_monitor, SCHEDULER_PRIORITY_CRITICAL);

// Perform critical operations

// Return to normal priority
scheduler_set_task_priority(batt_monitor, SCHEDULER_PRIORITY_HIGH);
```

### Monitoring CPU Usage

```c
// Monitor CPU usage for performance optimization
float cpu_usage[2];
esp_err_t result = scheduler_get_cpu_utilization(cpu_usage, 2);

if (result == ESP_OK) {
    ESP_LOGI(TAG, "CPU Usage - Core 0: %.1f%%, Core 1: %.1f%%", 
             cpu_usage[0], cpu_usage[1]);
    
    // Check for potential overload
    if (cpu_usage[0] > 90.0f || cpu_usage[1] > 90.0f) {
        ESP_LOGW(TAG, "High CPU usage detected!");
        // Take remedial action
    }
}
```

### Finding Tasks by Name

```c
// Find a task by name for dynamic management
scheduler_task_t *cell_balancer;
if (scheduler_find_task_by_name("cell_balancer", &cell_balancer) == ESP_OK) {
    // Temporarily suspend cell balancing during a high-current operation
    scheduler_suspend_task(cell_balancer);
    
    // Perform high-current operation
    
    // Resume cell balancing
    scheduler_resume_task(cell_balancer);
}
```

## Integration with BMS Components

The Task Scheduler is designed to work with all other BMS components:

- **Battery Manager**: Manages high-priority monitoring tasks
- **SoC Calculator**: Runs as a medium-priority periodic task
- **Cell Balancer**: Operates as a background task with adjustable priority
- **Thermal Monitor**: Runs as a high-priority safety-critical task
- **Communication Interfaces**: Manages Modbus and CANBus task scheduling

## Performance Considerations

1. **Stack Size**: Appropriate sizing based on task complexity and use of recursion
2. **Priority Assignment**: Proper balancing to prevent starvation
3. **Core Assignment**: Distribute compute-intensive tasks across cores
4. **Mutex Timeout**: Set appropriate timeouts to prevent deadlocks
5. **CPU Monitoring**: Use data to optimize task distribution

## Security and Safety

The scheduler implements several important safety features:

- **Task Validation**: All operations validate task handles
- **Resource Limits**: Maximum task count to prevent resource exhaustion
- **Graceful Failures**: Comprehensive error handling
- **Memory Management**: Proper allocation and cleanup of resources
- **Timeout Mechanisms**: Prevent indefinite blocking situations

## Future Enhancements

Potential improvements for future firmware versions:

1. **Task Statistics**: More detailed monitoring of individual task resource usage
2. **Dynamic Stack Sizing**: Runtime adjustment of stack allocation
3. **Task Groups**: Logical grouping of related tasks
4. **Power Management**: Integration with ESP32 power management features
5. **Watchdog Integration**: Automatic watchdog feeding for long-running tasks
6. **Adaptive Scheduling**: Dynamic task priority adjustment based on system load
