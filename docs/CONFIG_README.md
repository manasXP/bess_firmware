# Configuration Manager Module

## Overview

The Configuration Manager is a critical component of the 100KW/200KWH BESS firmware responsible for handling all system configuration parameters. It provides a unified interface for storing, retrieving, and managing configuration across the Battery Energy Storage System.

This module enables persistent storage of configuration data in either ESP32's NVS (Non-Volatile Storage) or SD card, with support for default fallback values. It implements a comprehensive parameter management system with type checking, validation, and change notifications.

## Core Features

- **Multiple Storage Options**: Store configuration in NVS, SD card, or use default values
- **Thread-Safe Design**: Safe concurrent access through FreeRTOS mutex protection
- **Notification System**: Component-specific change callbacks for coordinated responses
- **JSON Import/Export**: Serialization for easy configuration backup and transfer
- **Parameter Path Access**: Access specific values using dot notation (e.g., "battery.module_capacity")
- **Configuration Validation**: Verify configuration parameters against valid ranges
- **Type-Safe Operations**: Proper handling of different data types (int, float, bool, etc.)

## Configuration Parameters

The module manages configuration for all subsystems including:

- **Battery Parameters**: Module count, voltage, capacity, cell configuration
- **Protection Thresholds**: Voltage, temperature, current, and SoC limits
- **Communication Settings**: Modbus and CANBus configuration
- **Logging Configuration**: Console, SD card, and AWS CloudWatch logging options
- **SoC Calculation**: Algorithm selection and parameters
- **Cell Balancing**: Mode, thresholds, and timing settings
- **Thermal Management**: Cooling method and temperature thresholds
- **Task Settings**: Stack sizes, priorities, and execution intervals

## Usage Examples

### Initialization

```c
// Initialize with default values
esp_err_t ret = config_manager_init(CONFIG_STORAGE_DEFAULT, NULL);
if (ret != ESP_OK) {
    // Handle initialization error
}

// Or load from NVS
ret = config_manager_init(CONFIG_STORAGE_NVS, NULL);

// Or load from SD card
ret = config_manager_init(CONFIG_STORAGE_SD_CARD, "/sdcard/bess_config.json");
```

### Accessing Configuration

```c
// Get the entire configuration
bess_config_t config;
esp_err_t ret = config_manager_get_config(&config);
if (ret == ESP_OK) {
    // Use the configuration
    printf("Number of modules: %d\n", config.battery.num_modules);
    printf("System capacity: %.1f kWh\n", config.battery.system_capacity);
}
```

### Modifying Parameters

```c
// Modify a single parameter (with persistence)
esp_err_t ret = config_manager_set_param_string("battery.module_capacity", "16.5", true);
if (ret != ESP_OK) {
    // Handle error
}

// Or update the full configuration
bess_config_t config;
config_manager_get_config(&config);
config.battery.max_power_kw = 110.0f;
config.protection.temperature.max = 60.0f;
config_manager_set_config(&config, true);
```

### Registering for Change Notifications

```c
// Define a callback function
void battery_config_changed(const char *component_name, void *user_data) {
    printf("Battery configuration changed: %s\n", component_name);
    // Update module behavior based on the new configuration
}

// Register the callback
config_manager_register_callback("battery", battery_config_changed, NULL);
```

### JSON Import/Export

```c
// Export configuration to JSON
char json_buffer[4096];
esp_err_t ret = config_manager_export_json(json_buffer, sizeof(json_buffer));
if (ret == ESP_OK) {
    // Use the JSON string (e.g., save to file or send over network)
}

// Import configuration from JSON
const char *json_str = "{\"battery\":{\"num_modules\":12}}";
ret = config_manager_import_json(json_str, true);
```

### Saving and Loading Configuration

```c
// Save current configuration to NVS
esp_err_t ret = config_manager_save(CONFIG_STORAGE_NVS, NULL);

// Save to SD card
ret = config_manager_save(CONFIG_STORAGE_SD_CARD, "/sdcard/bess_config.json");

// Load from storage
ret = config_manager_load(CONFIG_STORAGE_SD_CARD, "/sdcard/bess_config.json");
```

### Resetting to Defaults

```c
// Reset to defaults without persisting
esp_err_t ret = config_manager_reset_to_defaults(false);

// Reset and save to storage
ret = config_manager_reset_to_defaults(true);
```

## Implementation Details

### Files

1. **config_manager.h**: Header file with API definitions and data structures
2. **config_manager.c**: Implementation of the configuration manager

### Data Structures

The module defines a comprehensive configuration structure hierarchy:

```c
typedef struct {
    char device_id[32];               // Unique device identifier
    char system_name[64];             // System name/description
    uint8_t firmware_version[3];      // Firmware version [major, minor, patch]
    bess_operation_mode_t mode;       // System operation mode
    
    comm_config_t comm;               // Communication configuration
    logging_config_t logging;         // Logging configuration
    battery_config_t battery;         // Battery system configuration
    protection_config_t protection;   // Protection thresholds
    soc_config_t soc;                 // SoC calculation configuration
    balancing_config_t balancing;     // Cell balancing configuration
    thermal_config_t thermal;         // Thermal management configuration
    task_config_t tasks;              // FreeRTOS task configuration
} bess_config_t;
```

### Thread Safety

All functions are protected with mutexes for thread-safe operation:

```c
/* Take mutex */
if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take configuration mutex");
    return ESP_ERR_TIMEOUT;
}

/* Configuration operations */
// ...

xSemaphoreGive(s_config_mutex);
```

### Logging

The module uses ESP-IDF's logging system for comprehensive status and error reporting:

```c
ESP_LOGI(TAG, "Configuration saved successfully to NVS");
ESP_LOGE(TAG, "Failed to parse JSON: %s", error_ptr);
ESP_LOGW(TAG, "No configuration found in NVS");
```

## Dependencies

- ESP-IDF framework
- FreeRTOS
- NVS Flash component
- cJSON for JSON parsing and serialization

## Future Enhancements

- Web interface for remote configuration
- Version control for configuration changes
- Configuration schema validation
- Encrypted configuration storage
- Configuration presets for different operating modes
