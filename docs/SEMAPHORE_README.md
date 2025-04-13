# Semaphore Manager

## Overview

The Semaphore Manager is a core middleware component of the 100KW/200KWH BESS firmware that provides centralized semaphore management across all system components. It enables thread-safe operation of the Battery Management System (BMS) components, communication interfaces, and logging subsystems in the FreeRTOS environment.

This component is designed to support the specific requirements of the LFP battery modular packs (48V, 16KWH) while ensuring proper synchronization between concurrent tasks running on the ESP32-P4 MCU.

## Key Features

- **Centralized Resource Management**: Tracks and manages up to 32 semaphores in a single registry
- **Multiple Semaphore Types**: Supports mutex, binary, counting, and recursive mutex types
- **Named Semaphores**: Each semaphore can be identified by a unique name for easy reference
- **Safety Features**: Comprehensive error handling, timeouts, and deadlock prevention
- **Diagnostic Capabilities**: Detailed statistics and status tracking for each semaphore
- **Thread Safety**: All operations are protected against concurrent access
- **System Integration**: Pre-configured semaphores for BMS components and subsystems

## Architecture

The Semaphore Manager acts as a middleware layer between FreeRTOS and the application code:

```
┌──────────────────────────────────────────────────────────────┐
│                      Application Layer                        │
│   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐        │
│   │Battery      │   │Cell         │   │Thermal      │        │
│   │Manager      │   │Balancer     │   │Monitor      │        │
│   └─────────────┘   └─────────────┘   └─────────────┘        │
├──────────────────────────────────────────────────────────────┤
│                     Middleware Layer                          │
│   ┌─────────────────────────────────────────────────┐        │
│   │              Semaphore Manager                   │        │
│   └─────────────────────────────────────────────────┘        │
├──────────────────────────────────────────────────────────────┤
│                        RTOS Layer                             │
│   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐        │
│   │FreeRTOS     │   │FreeRTOS     │   │FreeRTOS     │        │
│   │Semaphores   │   │Tasks        │   │Queues       │        │
│   └─────────────┘   └─────────────┘   └─────────────┘        │
└──────────────────────────────────────────────────────────────┘
```

## Implementation Details

### Data Structures

The Semaphore Manager maintains an internal registry of semaphores with detailed metadata:

```c
typedef struct {
    bool in_use;                     // Whether this slot is in use
    char name[32];                   // Name of the semaphore
    semaphore_type_t type;           // Type of semaphore
    SemaphoreHandle_t handle;        // FreeRTOS semaphore handle
    uint32_t initial_count;          // Initial count for counting semaphores
    uint32_t max_count;              // Maximum count for counting semaphores
    uint32_t access_count;           // Number of times semaphore has been accessed
    uint32_t timeout_count;          // Number of timeouts that occurred
    TickType_t last_taken_tick;      // When the semaphore was last taken
    bool is_taken;                   // Whether the semaphore is currently taken
    uint32_t waiting_tasks;          // Estimated number of waiting tasks
} semaphore_data_t;
```

### Thread Safety

The Semaphore Manager itself is protected by a mutex to ensure thread-safe operation when multiple tasks interact with the manager simultaneously.

### Error Handling

All functions include comprehensive error checking with detailed error codes:

```c
typedef enum {
    SEMAPHORE_OK = 0,                // Operation successful
    SEMAPHORE_ERR_INVALID_HANDLE,    // Invalid semaphore handle
    SEMAPHORE_ERR_OUT_OF_RESOURCES,  // No more semaphores available
    SEMAPHORE_ERR_ALREADY_EXISTS,    // Semaphore with this name already exists
    SEMAPHORE_ERR_NOT_FOUND,         // Semaphore not found by name
    SEMAPHORE_ERR_TIMEOUT,           // Timed out waiting for semaphore
    SEMAPHORE_ERR_INTERNAL           // Internal error
} semaphore_err_t;
```

### Performance Considerations

- **Memory Usage**: Approximately 64 bytes per semaphore entry
- **Time Complexity**: O(1) for most operations, O(n) for name lookups
- **Resource Limits**: Maximum of 32 semaphores (configurable)
- **Timeout Protection**: All operations have configurable timeouts

## Usage Guide

### Initialization

Initialize the Semaphore Manager at system startup:

```c
#include "semaphore_manager.h"

void app_main(void) {
    // Initialize the semaphore manager
    esp_err_t result = semaphore_manager_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize semaphore manager: %d", result);
        return;
    }
    
    // Create default system semaphores
    semaphore_create_system_defaults();
    
    // Continue with system initialization
    // ...
}
```

### Creating Semaphores

Create semaphores as needed for your components:

```c
// Create a mutex semaphore
semaphore_handle_t mutex_handle;
esp_err_t result = semaphore_create("my_component_mutex", 
                                    SEMAPHORE_TYPE_MUTEX,
                                    0, 1, &mutex_handle);
if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create mutex: %s", semaphore_err_to_str(result));
    // Handle error
}
```

### Using Semaphores

Use the provided macros for critical sections:

```c
void some_function(void) {
    semaphore_handle_t mutex_handle;
    
    // Find semaphore by name
    esp_err_t result = semaphore_find_by_name("my_component_mutex", &mutex_handle);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to find mutex: %s", semaphore_err_to_str(result));
        return;
    }
    
    // Use critical section macro
    SEMAPHORE_CRITICAL_SECTION_START(mutex_handle, 1000, error_exit) {
        // Protected code here
        // ...
    } SEMAPHORE_CRITICAL_SECTION_END(mutex_handle, error_exit);
    
    return;
    
error_exit:
    ESP_LOGE(TAG, "Semaphore operation failed");
}
```

Alternatively, use the direct take/give API:

```c
esp_err_t result = semaphore_take(mutex_handle, 1000);
if (result == ESP_OK) {
    // Protected code here
    // ...
    
    // Release semaphore
    semaphore_give(mutex_handle);
} else {
    ESP_LOGE(TAG, "Failed to take semaphore: %s", semaphore_err_to_str(result));
}
```

### Monitoring Semaphore Status

Monitor semaphores for debugging:

```c
// Dump status of all semaphores
semaphore_dump_status(ESP_LOG_INFO);

// Get information about a specific semaphore
semaphore_info_t info;
if (semaphore_get_info(mutex_handle, &info) == ESP_OK) {
    ESP_LOGI(TAG, "Semaphore '%s': taken=%d, access_count=%u, timeouts=%u",
             info.name, info.is_taken, info.access_count, info.timeout_count);
}
```

## Default System Semaphores

The Semaphore Manager creates the following default semaphores for BMS components:

| Name | Type | Purpose |
|------|------|---------|
| battery_manager_mutex | MUTEX | Protects Battery Manager data structures |
| soc_calculator_mutex | MUTEX | Protects SoC Calculator data |
| cell_balancer_mutex | MUTEX | Protects Cell Balancer operations |
| thermal_monitor_mutex | MUTEX | Protects Thermal Monitor data |
| communication_mutex | MUTEX | Protects communication interfaces |
| log_mutex | MUTEX | Protects logging operations |
| sync_event | BINARY | General synchronization events |
| resource_count | COUNTING | Resource pool management |

## Error Handling

All functions return ESP error codes or Semaphore Manager-specific error codes:

```c
esp_err_t result = semaphore_take(handle, timeout_ms);
if (result == ESP_OK) {
    // Operation succeeded
} else if (result == SEMAPHORE_ERR_TIMEOUT) {
    // Handle timeout
    ESP_LOGW(TAG, "Timed out waiting for semaphore");
} else {
    // Handle other errors
    ESP_LOGE(TAG, "Semaphore error: %s", semaphore_err_to_str(result));
}
```

## Debugging Tips

1. **Deadlock Detection**: Use `semaphore_dump_status()` to identify deadlocks
2. **Timeout Analysis**: Check `timeout_count` to identify contentious semaphores
3. **Resource Usage**: Monitor `in_use` count to ensure resources aren't being leaked
4. **Wait Time**: Calculate wait time using `last_taken_tick` to identify performance issues

## Integration with BESS Components

### Battery Manager

```c
// In battery_manager.c initialization
semaphore_handle_t bm_mutex;
esp_err_t result = semaphore_find_by_name("battery_manager_mutex", &bm_mutex);
if (result != ESP_OK) {
    // Create the mutex if not found
    result = semaphore_create("battery_manager_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &bm_mutex);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create battery manager mutex");
        return result;
    }
}
```

### SoC Calculator

```c
// In soc_calculator.c operations
semaphore_handle_t soc_mutex;
if (semaphore_find_by_name("soc_calculator_mutex", &soc_mutex) == ESP_OK) {
    SEMAPHORE_CRITICAL_SECTION_START(soc_mutex, 1000, error) {
        // Update SoC calculations
        // ...
    } SEMAPHORE_CRITICAL_SECTION_END(soc_mutex, error);
}
```

### Cell Balancer

```c
// In cell_balancer.c
esp_err_t cell_balancer_update_cell_data(uint8_t module_id, const bess_cell_data_t *cell_data, uint8_t count) {
    semaphore_handle_t balancer_mutex;
    esp_err_t result = semaphore_find_by_name("cell_balancer_mutex", &balancer_mutex);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Cell balancer mutex not found");
        return result;
    }
    
    result = semaphore_take(balancer_mutex, 500);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to take cell balancer mutex: %s", semaphore_err_to_str(result));
        return result;
    }
    
    // Update cell data
    // ...
    
    semaphore_give(balancer_mutex);
    return ESP_OK;
}
```

## Security Considerations

1. **Timeout Protection**: All semaphore operations use timeouts to prevent indefinite blocking
2. **Resource Limits**: The maximum number of semaphores is capped to prevent resource exhaustion
3. **Reset Capability**: Semaphores can be reset to recover from error conditions
4. **Diagnostic Tools**: Comprehensive status reporting helps identify issues

## Future Enhancements

1. **Deadlock Detection**: Automatic detection of potential deadlock conditions
2. **Priority Inheritance**: Support for priority inheritance to prevent priority inversion
3. **Performance Metrics**: Detailed timing statistics for semaphore wait times
4. **Extended Diagnostics**: Call stack tracking for semaphore operations
5. **Watchdog Integration**: Coordination with system watchdog for hung task detection
