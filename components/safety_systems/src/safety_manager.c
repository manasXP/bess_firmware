/**
 * @file safety_manager.c
 * @brief Implementation of Safety Management System for 100KW/200KWH BESS with LFP Battery Modules
 * 
 * This file implements the safety management system defined in safety_manager.h. It provides
 * real-time monitoring and protection for the Battery Energy Storage System.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "safety_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// External component headers (these would be included in a real implementation)
#include "bess_config.h"           // System-wide configuration
#include "logger.h"                // Logging system
#include "modbus_interface.h"      // Modbus communication
#include "canbus_interface.h"      // CANBus communication
#include "cloudwatch_logger.h"     // AWS Cloudwatch logging

// Define tag for ESP logging
static const char *TAG = "SAFETY_MANAGER";

// Maximum number of callback functions per event
#define MAX_CALLBACKS_PER_EVENT 5

// Event group bits for safety task coordination
#define SAFETY_BIT_INIT_COMPLETE   (1 << 0)
#define SAFETY_BIT_TASK_RUNNING    (1 << 1)
#define SAFETY_BIT_EMERGENCY       (1 << 2)
#define SAFETY_BIT_FAULT_DETECTED  (1 << 3)
#define SAFETY_BIT_SELF_TEST       (1 << 4)

// Define internal structure for temperature history (for thermal runaway detection)
typedef struct {
    float temperatures[8];         // Up to 8 temperature sensors per module
    uint8_t sensor_count;          // Actual number of sensors for this module
    uint32_t timestamp;            // Timestamp of last update
} temp_history_entry_t;

// Define internal structure for module data tracking
typedef struct {
    // Voltage data
    float module_voltage;
    float *cell_voltages;
    uint8_t cell_count;
    uint32_t voltage_timestamp;
    
    // Current data
    float current;
    uint32_t current_timestamp;
    
    // Temperature data
    float *temperatures;
    uint8_t sensor_count;
    uint32_t temp_timestamp;
    
    // Temperature history for thermal runaway detection
    temp_history_entry_t temp_history[5];  // Circular buffer of last 5 readings
    uint8_t temp_history_index;            // Current index in circular buffer
    
    // State of Charge data
    float soc;
    uint32_t soc_timestamp;
    
    // Fault tracking
    uint32_t fault_bits;           // Bitfield of active faults for this module
} module_data_t;

// Define callback registration structure
typedef struct {
    safety_event_callback_t callback;
    void *user_data;
} callback_entry_t;

// Static variables for internal state
static bool s_initialized = false;
static safety_config_t s_config;
static module_data_t *s_modules = NULL;
static TaskHandle_t s_monitor_task_handle = NULL;
static SemaphoreHandle_t s_data_mutex = NULL;
static EventGroupHandle_t s_event_group = NULL;
static safety_status_t s_system_status = SAFETY_STATUS_UNKNOWN;
static safety_fault_t *s_fault_list = NULL;
static uint8_t s_fault_count = 0;
static callback_entry_t s_callbacks[SAFETY_EVENT_COUNT][MAX_CALLBACKS_PER_EVENT];
static bool s_auto_contactor_control = true;
static uint32_t s_last_fault_report_time = 0;
static bool s_emergency_shutdown_active = false;

// Forward declarations for internal functions
static void safety_monitor_task(void *pvParameters);
static esp_err_t check_module_voltage(uint8_t module_id);
static esp_err_t check_cell_voltages(uint8_t module_id);
static esp_err_t check_module_current(uint8_t module_id);
static esp_err_t check_module_temperature(uint8_t module_id);
static esp_err_t check_thermal_runaway(uint8_t module_id);
static esp_err_t check_module_soc(uint8_t module_id);
static esp_err_t update_system_status(void);
static void process_safety_event(safety_event_t event, uint8_t module_id, float measured_value, float threshold_value);
static void execute_safety_action(safety_event_t event, uint8_t module_id, safety_action_t action);
static esp_err_t record_fault(safety_event_t event, uint8_t module_id, float measured_value, float threshold_value, safety_action_t action);
static void report_faults_to_communication_channels(void);
static safety_action_t determine_action_for_event(safety_event_t event);
static void log_safety_event(safety_event_t event, uint8_t module_id, float measured_value, float threshold_value, safety_action_t action);
static bool is_fault_active(safety_event_t event, uint8_t module_id);
static float get_threshold_for_event(safety_event_t event);
static float calculate_temperature_rate_of_change(const module_data_t *module);
static void trigger_emergency_shutdown(safety_event_t event, uint8_t module_id, const char *reason);

/**
 * @brief Initialize the safety management system
 */
esp_err_t safety_manager_init(const safety_config_t *config) {
    // Check if already initialized
    if (s_initialized) {
        ESP_LOGW(TAG, "Safety manager already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate configuration
    if (config == NULL) {
        ESP_LOGE(TAG, "NULL configuration provided");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->number_of_modules == 0 || config->number_of_modules > 32) {
        ESP_LOGE(TAG, "Invalid number of modules: %d", config->number_of_modules);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create mutex for data protection
    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Create event group for task coordination
    s_event_group = xEventGroupCreate();
    if (s_event_group == NULL) {
        vSemaphoreDelete(s_data_mutex);
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    memcpy(&s_config, config, sizeof(safety_config_t));
    
    // Allocate module data
    s_modules = calloc(config->number_of_modules, sizeof(module_data_t));
    if (s_modules == NULL) {
        vSemaphoreDelete(s_data_mutex);
        vEventGroupDelete(s_event_group);
        ESP_LOGE(TAG, "Failed to allocate module data");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize module data arrays
    for (uint8_t i = 0; i < config->number_of_modules; i++) {
        // Allocate cell voltage array
        s_modules[i].cell_voltages = calloc(config->cells_per_module, sizeof(float));
        if (s_modules[i].cell_voltages == NULL) {
            ESP_LOGE(TAG, "Failed to allocate cell voltage array for module %d", i);
            goto cleanup;
        }
        
        // Allocate temperature sensor array (assuming max 8 sensors per module)
        s_modules[i].temperatures = calloc(8, sizeof(float));
        if (s_modules[i].temperatures == NULL) {
            ESP_LOGE(TAG, "Failed to allocate temperature array for module %d", i);
            goto cleanup;
        }
        
        // Initialize other module data
        s_modules[i].cell_count = 0;
        s_modules[i].sensor_count = 0;
        s_modules[i].module_voltage = 0.0f;
        s_modules[i].current = 0.0f;
        s_modules[i].soc = 50.0f;  // Default to 50% until real data is available
        s_modules[i].fault_bits = 0;
        s_modules[i].temp_history_index = 0;
    }
    
    // Allocate fault list
    s_fault_list = calloc(config->max_fault_count, sizeof(safety_fault_t));
    if (s_fault_list == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fault list");
        goto cleanup;
    }
    
    // Initialize callbacks
    memset(s_callbacks, 0, sizeof(s_callbacks));
    
    // Initialize system status
    s_system_status = SAFETY_STATUS_UNKNOWN;
    s_fault_count = 0;
    s_auto_contactor_control = true;
    s_emergency_shutdown_active = false;
    
    // Initialize communication interfaces if enabled
    if (config->enable_modbus_reporting) {
        if (modbus_interface_init() != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize Modbus interface");
            // Continue anyway, we'll disable it later
        }
    }
    
    if (config->enable_canbus_reporting) {
        if (canbus_interface_init() != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize CANBus interface");
            // Continue anyway, we'll disable it later
        }
    }
    
    // Mark as initialized
    s_initialized = true;
    
    ESP_LOGI(TAG, "Safety manager initialized with %d modules", config->number_of_modules);
    return ESP_OK;
    
cleanup:
    // Clean up allocated memory if initialization failed
    if (s_modules != NULL) {
        for (uint8_t i = 0; i < config->number_of_modules; i++) {
            if (s_modules[i].cell_voltages != NULL) {
                free(s_modules[i].cell_voltages);
            }
            if (s_modules[i].temperatures != NULL) {
                free(s_modules[i].temperatures);
            }
        }
        free(s_modules);
        s_modules = NULL;
    }
    
    if (s_data_mutex != NULL) {
        vSemaphoreDelete(s_data_mutex);
        s_data_mutex = NULL;
    }
    
    if (s_event_group != NULL) {
        vEventGroupDelete(s_event_group);
        s_event_group = NULL;
    }
    
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Start the safety management system monitoring tasks
 */
esp_err_t safety_manager_start(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Safety manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if task is already running
    if (s_monitor_task_handle != NULL) {
        ESP_LOGW(TAG, "Safety monitor task already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create monitoring task
    BaseType_t task_created = xTaskCreatePinnedToCore(
        safety_monitor_task,            // Task function
        "safety_monitor",               // Name
        4096,                           // Stack size (bytes)
        NULL,                           // Parameters
        configMAX_PRIORITIES - 2,       // Priority (high, but not highest)
        &s_monitor_task_handle,         // Task handle
        0                               // Core (0 for ESP32)
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create safety monitor task");
        s_monitor_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Wait for task to complete initialization
    EventBits_t bits = xEventGroupWaitBits(
        s_event_group,
        SAFETY_BIT_INIT_COMPLETE,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(5000)
    );
    
    if ((bits & SAFETY_BIT_INIT_COMPLETE) == 0) {
        ESP_LOGE(TAG, "Safety monitor task initialization timeout");
        vTaskDelete(s_monitor_task_handle);
        s_monitor_task_handle = NULL;
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Safety manager started");
    return ESP_OK;
}

/**
 * @brief Stop the safety management system monitoring tasks
 */
esp_err_t safety_manager_stop(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Safety manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_monitor_task_handle == NULL) {
        ESP_LOGW(TAG, "Safety monitor task not running");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Signal task to stop
    xEventGroupClearBits(s_event_group, SAFETY_BIT_TASK_RUNNING);
    
    // Wait for task to end
    EventBits_t bits = xEventGroupWaitBits(
        s_event_group,
        SAFETY_BIT_TASK_RUNNING,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(5000)
    );
    
    // If task is still running, delete it forcefully
    if ((bits & SAFETY_BIT_TASK_RUNNING) != 0) {
        ESP_LOGW(TAG, "Safety monitor task did not stop gracefully, deleting");
        vTaskDelete(s_monitor_task_handle);
    }
    
    s_monitor_task_handle = NULL;
    ESP_LOGI(TAG, "Safety manager stopped");
    return ESP_OK;
}

/**
 * @brief Update voltage data for a specific module
 */
esp_err_t safety_manager_update_voltage_data(uint8_t module_id, 
                                           float module_voltage,
                                           const float *cell_voltages,
                                           uint8_t cell_count) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (module_id >= s_config.number_of_modules || cell_voltages == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (cell_count > s_config.cells_per_module) {
        cell_count = s_config.cells_per_module;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for voltage update");
        return ESP_ERR_TIMEOUT;
    }
    
    // Update module data
    s_modules[module_id].module_voltage = module_voltage;
    memcpy(s_modules[module_id].cell_voltages, cell_voltages, cell_count * sizeof(float));
    s_modules[module_id].cell_count = cell_count;
    s_modules[module_id].voltage_timestamp = esp_timer_get_time() / 1000;  // Convert to ms
    
    xSemaphoreGive(s_data_mutex);
    
    // Run immediate voltage checks outside the mutex
    check_module_voltage(module_id);
    check_cell_voltages(module_id);
    
    return ESP_OK;
}

/**
 * @brief Update current data for a specific module
 */
esp_err_t safety_manager_update_current_data(uint8_t module_id, float current) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for current update");
        return ESP_ERR_TIMEOUT;
    }
    
    // Update module data
    s_modules[module_id].current = current;
    s_modules[module_id].current_timestamp = esp_timer_get_time() / 1000;  // Convert to ms
    
    xSemaphoreGive(s_data_mutex);
    
    // Run immediate current check outside the mutex
    check_module_current(module_id);
    
    return ESP_OK;
}

/**
 * @brief Update temperature data for a specific module
 */
esp_err_t safety_manager_update_temperature_data(uint8_t module_id,
                                               const float *temperatures,
                                               uint8_t sensor_count) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (module_id >= s_config.number_of_modules || temperatures == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (sensor_count > 8) {  // Maximum 8 sensors per module
        sensor_count = 8;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for temperature update");
        return ESP_ERR_TIMEOUT;
    }
    
    // Update module data
    memcpy(s_modules[module_id].temperatures, temperatures, sensor_count * sizeof(float));
    s_modules[module_id].sensor_count = sensor_count;
    uint32_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    s_modules[module_id].temp_timestamp = current_time;
    
    // Update temperature history for thermal runaway detection
    uint8_t idx = s_modules[module_id].temp_history_index;
    s_modules[module_id].temp_history_index = (idx + 1) % 5;  // Circular buffer of 5 entries
    
    memcpy(s_modules[module_id].temp_history[idx].temperatures, 
           temperatures, 
           sensor_count * sizeof(float));
    s_modules[module_id].temp_history[idx].sensor_count = sensor_count;
    s_modules[module_id].temp_history[idx].timestamp = current_time;
    
    xSemaphoreGive(s_data_mutex);
    
    // Run immediate temperature checks outside the mutex
    check_module_temperature(module_id);
    check_thermal_runaway(module_id);
    
    return ESP_OK;
}

/**
 * @brief Update State of Charge data for a specific module
 */
esp_err_t safety_manager_update_soc_data(uint8_t module_id, float soc) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Ensure SoC is within valid range
    if (soc < 0.0f) {
        soc = 0.0f;
    } else if (soc > 100.0f) {
        soc = 100.0f;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for SoC update");
        return ESP_ERR_TIMEOUT;
    }
    
    // Update module data
    s_modules[module_id].soc = soc;
    s_modules[module_id].soc_timestamp = esp_timer_get_time() / 1000;  // Convert to ms
    
    xSemaphoreGive(s_data_mutex);
    
    // Run immediate SoC check outside the mutex
    check_module_soc(module_id);
    
    return ESP_OK;
}

/**
 * @brief Register a callback function for safety events
 */
esp_err_t safety_manager_register_callback(safety_event_t event,
                                         safety_event_callback_t callback,
                                         void *user_data) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For SAFETY_EVENT_COUNT, register for all events
    if (event == SAFETY_EVENT_COUNT) {
        esp_err_t ret = ESP_OK;
        for (int i = 0; i < SAFETY_EVENT_COUNT; i++) {
            esp_err_t result = safety_manager_register_callback((safety_event_t)i, callback, user_data);
            if (result != ESP_OK) {
                ret = result;
            }
        }
        return ret;
    }
    
    if (event >= SAFETY_EVENT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for callback registration");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find an empty slot in the callback array
    for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
        if (s_callbacks[event][i].callback == NULL) {
            s_callbacks[event][i].callback = callback;
            s_callbacks[event][i].user_data = user_data;
            xSemaphoreGive(s_data_mutex);
            return ESP_OK;
        }
    }
    
    xSemaphoreGive(s_data_mutex);
    ESP_LOGW(TAG, "No free callback slots for event %d", event);
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Unregister a callback function for safety events
 */
esp_err_t safety_manager_unregister_callback(safety_event_t event,
                                           safety_event_callback_t callback) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For SAFETY_EVENT_COUNT, unregister from all events
    if (event == SAFETY_EVENT_COUNT) {
        esp_err_t ret = ESP_OK;
        for (int i = 0; i < SAFETY_EVENT_COUNT; i++) {
            esp_err_t result = safety_manager_unregister_callback((safety_event_t)i, callback);
            if (result != ESP_OK) {
                ret = result;
            }
        }
        return ret;
    }
    
    if (event >= SAFETY_EVENT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for callback unregistration");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find the callback in the array
    bool found = false;
    for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
        if (s_callbacks[event][i].callback == callback) {
            s_callbacks[event][i].callback = NULL;
            s_callbacks[event][i].user_data = NULL;
            found = true;
            break;
        }
    }
    
    xSemaphoreGive(s_data_mutex);
    
    if (!found) {
        ESP_LOGW(TAG, "Callback not found for event %d", event);
        return ESP_ERR_NOT_FOUND;
    }
    
    return ESP_OK;
}

/**
 * @brief Manual trigger of a safety event for testing or external conditions
 */
esp_err_t safety_manager_trigger_event(safety_event_t event,
                                     uint8_t module_id,
                                     float measured_value) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (event >= SAFETY_EVENT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (module_id != 0xFF && module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get threshold for the event
    float threshold = get_threshold_for_event(event);
    
    // Process the event
    process_safety_event(event, module_id, measured_value, threshold);
    
    return ESP_OK;
}

/**
 * @brief Get current system safety status
 */
esp_err_t safety_manager_get_status(safety_status_t *status) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *status = s_system_status;
    return ESP_OK;
}

/**
 * @brief Get current fault list
 */
esp_err_t safety_manager_get_faults(safety_fault_t *faults,
                                  uint8_t max_faults,
                                  uint8_t *fault_count) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (faults == NULL || fault_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for fault retrieval");
        return ESP_ERR_TIMEOUT;
    }
    
    // Copy fault list
    uint8_t copy_count = (max_faults < s_fault_count) ? max_faults : s_fault_count;
    memcpy(faults, s_fault_list, copy_count * sizeof(safety_fault_t));
    *fault_count = copy_count;
    
    xSemaphoreGive(s_data_mutex);
    return ESP_OK;
}

/**
 * @brief Clear a specific fault
 */
esp_err_t safety_manager_clear_fault(safety_event_t event, uint8_t module_id) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (event >= SAFETY_EVENT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (module_id != 0xFF && module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for fault clearing");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find and clear the fault in the fault list
    bool found = false;
    for (uint8_t i = 0; i < s_fault_count; i++) {
        if (s_fault_list[i].event == event && 
            (s_fault_list[i].module_id == module_id || module_id == 0xFF)) {
            
            // If active fault, mark as inactive but keep in list
            if (s_fault_list[i].is_active) {
                s_fault_list[i].is_active = false;
                
                // Clear the fault bit in the module
                if (s_fault_list[i].module_id != 0xFF) {
                    s_modules[s_fault_list[i].module_id].fault_bits &= ~(1 << event);
                }
                
                found = true;
            } 
            // If inactive and module_id is exact match, remove from list
            else if (s_fault_list[i].module_id == module_id) {
                // Move the last fault to this position and decrement count
                if (i < s_fault_count - 1) {
                    s_fault_list[i] = s_fault_list[s_fault_count - 1];
                }
                s_fault_count--;
                i--; // Check this index again as it now has a new fault
                found = true;
            }
        }
    }
    
    // Update system status after clearing faults
    if (found) {
        update_system_status();
    }
    
    xSemaphoreGive(s_data_mutex);
    
    return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * @brief Clear all faults in the system
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t safety_manager_clear_all_faults(void) {
    // Check if safety manager is initialized
    if (s_safety_manager == NULL || !s_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Acquire the safety manager mutex to prevent concurrent access
    if (xSemaphoreTake(s_safety_mutex, pdMS_TO_TICKS(SAFETY_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for clearing all faults");
        return ESP_ERR_TIMEOUT;
    }
    
    // Log the action
    ESP_LOGI(TAG, "Clearing all system faults");
    
    // Record this action in all configured log destinations
    safety_log_event(SAFETY_LOG_INFO, "User cleared all system faults", 
                     s_safety_config.log_destination);
    
    // Clear all faults in the fault registry
    for (int i = 0; i < s_safety_config.max_fault_count; i++) {
        if (s_fault_registry[i].is_active) {
            // Log each cleared fault for record-keeping
            safety_log_event(SAFETY_LOG_INFO, 
                             "Cleared fault: %s on module %d", 
                             s_safety_config.log_destination,
                             safety_event_to_string(s_fault_registry[i].event),
                             s_fault_registry[i].module_id);
            
            // Mark the fault as inactive
            s_fault_registry[i].is_active = false;
            
            // Call any registered callbacks for the fault clearing
            safety_notify_event_callbacks(s_fault_registry[i].event, 
                                         s_fault_registry[i].module_id,
                                         0.0f,  // Measured value no longer relevant
                                         0.0f); // Threshold value no longer relevant
        }
    }
    
    // Reset fault counters
    s_active_fault_count = 0;
    s_total_fault_count = 0;
    
    // Reset the safety status if appropriate
    // Note: Only reset to OK if there are no remaining hardware conditions
    // that would immediately trigger new faults
    if (!safety_check_for_fault_conditions()) {
        s_safety_status = SAFETY_STATUS_OK;
        
        // Signal any tasks waiting on fault clearing
        xEventGroupSetBits(s_safety_event_group, SAFETY_EVENT_ALL_FAULTS_CLEARED);
    }
    
    // Release the mutex
    xSemaphoreGive(s_safety_mutex);
    
    // If Modbus or CANBus reporting is enabled, update the remote systems
    if (s_safety_config.enable_modbus_reporting) {
        modbus_report_safety_status(s_safety_status);
    }
    
    if (s_safety_config.enable_canbus_reporting) {
        canbus_report_safety_status(s_safety_status);
    }
    
    // If we're connected to AWS, update the cloud status
    if (s_safety_config.log_destination & SAFETY_LOG_CLOUDWATCH) {
        safety_update_cloud_status();
    }
    
    return ESP_OK;
}

/**
 * @brief Check cell voltages against limits
 */
static esp_err_t check_cell_voltages(uint8_t module_id) {
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;

/**
 * @brief Trigger emergency shutdown procedure
 */
static void trigger_emergency_shutdown(safety_event_t event, uint8_t module_id, const char *reason) {
    if (s_emergency_shutdown_active) {
        // Already in emergency shutdown
        return;
    }
    
    ESP_LOGE(TAG, "EMERGENCY SHUTDOWN: %s", reason);
    
    // Set emergency shutdown flag
    s_emergency_shutdown_active = true;
    
    // Set emergency bit in event group
    xEventGroupSetBits(s_event_group, SAFETY_BIT_EMERGENCY);
    
    // Log to all destinations
    char message[256];
    snprintf(message, sizeof(message), "EMERGENCY SHUTDOWN: Event=%d, Module=%d, Reason=%s",
             event, module_id, reason);
    
    // Console logging via ESP_LOG
    ESP_LOGE(TAG, "%s", message);
    
    // SD Card logging
    if (s_config.log_destination & SAFETY_LOG_SD_CARD) {
        logger_log_message(message, LOGGER_LEVEL_CRITICAL);
    }
    
    // AWS Cloudwatch logging
    if (s_config.log_destination & SAFETY_LOG_CLOUDWATCH) {
        cloudwatch_log_message("BESS_SAFETY", message, CLOUDWATCH_LEVEL_CRITICAL);
    }
    
    // Execute emergency procedures
    
    // 1. Open all contactors immediately
    // (In a real implementation, this would call the contactor control module)
    
    // 2. Alert all communication channels
    if (s_config.enable_modbus_reporting) {
        modbus_interface_emergency_shutdown(event, module_id, reason);
    }
    
    if (s_config.enable_canbus_reporting) {
        canbus_interface_emergency_shutdown(event, module_id, reason);
    }
    
    // 3. Update system status
    s_system_status = SAFETY_STATUS_EMERGENCY;
    
    // 4. Record the emergency event as a fault if not already recorded
    safety_fault_t emergency_fault = {
        .event = event,
        .module_id = module_id,
        .timestamp = esp_timer_get_time() / 1000,
        .measured_value = 0.0f,  // No specific value for emergency
        .threshold_value = 0.0f,
        .duration_ms = 0,
        .action_taken = SAFETY_ACTION_EMERGENCY_SHUTDOWN,
        .requires_reset = true,
        .is_active = true
    };
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check if we already have this fault recorded
        bool found = false;
        for (uint8_t i = 0; i < s_fault_count; i++) {
            if (s_fault_list[i].event == event && s_fault_list[i].module_id == module_id) {
                // Update existing fault
                s_fault_list[i].action_taken = SAFETY_ACTION_EMERGENCY_SHUTDOWN;
                s_fault_list[i].requires_reset = true;
                s_fault_list[i].is_active = true;
                found = true;
                break;
            }
        }
        
        // Add new fault if not found and we have space
        if (!found && s_fault_count < s_config.max_fault_count) {
            s_fault_list[s_fault_count] = emergency_fault;
            s_fault_count++;
        }
        
        xSemaphoreGive(s_data_mutex);
    }
}
    
    float *cell_voltages = NULL;
    uint8_t cell_count = 0;
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for cell voltage check");
        return ESP_ERR_TIMEOUT;
    }
    
    // Get cell voltages
    cell_voltages = s_modules[module_id].cell_voltages;
    cell_count = s_modules[module_id].cell_count;
    
    // Ensure valid data before checking
    if (cell_voltages == NULL || cell_count == 0) {
        xSemaphoreGive(s_data_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Make local copies to avoid holding mutex during event processing
    float cell_voltages_copy[s_config.cells_per_module];
    memcpy(cell_voltages_copy, cell_voltages, cell_count * sizeof(float));
    
    xSemaphoreGive(s_data_mutex);
    
    // Check each cell against limits
    for (uint8_t i = 0; i < cell_count; i++) {
        // Check high cell voltage limit
        if (cell_voltages_copy[i] > s_config.cell_voltage_high) {
            process_safety_event(SAFETY_EVENT_CELL_VOLTAGE_HIGH, module_id, 
                               cell_voltages_copy[i], s_config.cell_voltage_high);
        }
        
        // Check low cell voltage limit
        if (cell_voltages_copy[i] < s_config.cell_voltage_low) {
            process_safety_event(SAFETY_EVENT_CELL_VOLTAGE_LOW, module_id, 
                               cell_voltages_copy[i], s_config.cell_voltage_low);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Check module current against limits
 */
static esp_err_t check_module_current(uint8_t module_id) {
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float current = 0.0f;
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for current check");
        return ESP_ERR_TIMEOUT;
    }
    
    // Get current current value (positive for discharge, negative for charge)
    current = s_modules[module_id].current;
    
    xSemaphoreGive(s_data_mutex);
    
    // Check discharge current limit
    if (current > s_config.current_high_discharge) {
        process_safety_event(SAFETY_EVENT_CURRENT_HIGH, module_id, 
                           current, s_config.current_high_discharge);
    }
    
    // Check charge current limit (negative values)
    if (current < -s_config.current_high_charge) {
        process_safety_event(SAFETY_EVENT_CURRENT_HIGH, module_id, 
                           current, -s_config.current_high_charge);
    }
    
    // Check for reverse current (small positive when supposed to be charging, 
    // or small negative when supposed to be discharging)
    // This would be dependent on system state, simplified here
    
    return ESP_OK;
}

/**
 * @brief Check module temperatures against limits
 */
static esp_err_t check_module_temperature(uint8_t module_id) {
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float *temperatures = NULL;
    uint8_t sensor_count = 0;
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for temperature check");
        return ESP_ERR_TIMEOUT;
    }
    
    // Get temperature readings
    temperatures = s_modules[module_id].temperatures;
    sensor_count = s_modules[module_id].sensor_count;
    
    // Ensure valid data before checking
    if (temperatures == NULL || sensor_count == 0) {
        xSemaphoreGive(s_data_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Make local copies to avoid holding mutex during event processing
    float temperatures_copy[8];  // Maximum 8 sensors per module
    memcpy(temperatures_copy, temperatures, sensor_count * sizeof(float));
    
    xSemaphoreGive(s_data_mutex);
    
    // Check each sensor against limits
    for (uint8_t i = 0; i < sensor_count; i++) {
        // Check high temperature limit
        if (temperatures_copy[i] > s_config.temperature_high) {
            process_safety_event(SAFETY_EVENT_TEMPERATURE_HIGH, module_id, 
                               temperatures_copy[i], s_config.temperature_high);
        }
        
        // Check critical temperature limit
        if (temperatures_copy[i] > s_config.temperature_critical) {
            // Trigger emergency shutdown for critical temperature
            char reason[64];
            snprintf(reason, sizeof(reason), "Critical temperature: %.1f°C", temperatures_copy[i]);
            trigger_emergency_shutdown(SAFETY_EVENT_TEMPERATURE_HIGH, module_id, reason);
        }
        
        // Check low temperature limit
        if (temperatures_copy[i] < s_config.temperature_low) {
            process_safety_event(SAFETY_EVENT_TEMPERATURE_LOW, module_id, 
                               temperatures_copy[i], s_config.temperature_low);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Check for thermal runaway conditions
 */
static esp_err_t check_thermal_runaway(uint8_t module_id) {
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for thermal runaway check");
        return ESP_ERR_TIMEOUT;
    }
    
    // Calculate temperature rate of change
    float temp_rate = calculate_temperature_rate_of_change(&s_modules[module_id]);
    
    xSemaphoreGive(s_data_mutex);
    
    // Check against thermal runaway threshold
    if (temp_rate > s_config.temp_rate_runaway) {
        // This is a critical condition
        char reason[64];
        snprintf(reason, sizeof(reason), "Thermal runaway detected: %.2f°C/s", temp_rate);
        process_safety_event(SAFETY_EVENT_THERMAL_RUNAWAY, module_id, temp_rate, s_config.temp_rate_runaway);
        
        // If rate is significantly above threshold, trigger emergency shutdown
        if (temp_rate > s_config.temp_rate_runaway * 1.5) {
            trigger_emergency_shutdown(SAFETY_EVENT_THERMAL_RUNAWAY, module_id, reason);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Calculate temperature rate of change from history
 */
static float calculate_temperature_rate_of_change(const module_data_t *module) {
    if (module == NULL || module->sensor_count == 0) {
        return 0.0f;
    }
    
    // Find the oldest and newest valid temperature readings
    uint32_t newest_time = 0;
    uint32_t oldest_time = UINT32_MAX;
    float newest_avg_temp = 0.0f;
    float oldest_avg_temp = 0.0f;
    bool found_newest = false;
    bool found_oldest = false;
    
    // First pass: find the newest and oldest timestamps
    for (int i = 0; i < 5; i++) {  // 5 entries in history
        if (module->temp_history[i].timestamp > 0 && 
            module->temp_history[i].sensor_count > 0) {
            
            if (module->temp_history[i].timestamp > newest_time) {
                newest_time = module->temp_history[i].timestamp;
                found_newest = true;
            }
            
            if (module->temp_history[i].timestamp < oldest_time) {
                oldest_time = module->temp_history[i].timestamp;
                found_oldest = true;
            }
        }
    }
    
    // Return 0 if we don't have enough history
    if (!found_newest || !found_oldest || newest_time == oldest_time) {
        return 0.0f;
    }
    
    // Second pass: calculate average temperatures for newest and oldest readings
    for (int i = 0; i < 5; i++) {
        if (module->temp_history[i].timestamp == newest_time) {
            // Calculate average of all sensors for newest reading
            float sum = 0.0f;
            for (int j = 0; j < module->temp_history[i].sensor_count; j++) {
                sum += module->temp_history[i].temperatures[j];
            }
            newest_avg_temp = sum / module->temp_history[i].sensor_count;
        }
        
        if (module->temp_history[i].timestamp == oldest_time) {
            // Calculate average of all sensors for oldest reading
            float sum = 0.0f;
            for (int j = 0; j < module->temp_history[i].sensor_count; j++) {
                sum += module->temp_history[i].temperatures[j];
            }
            oldest_avg_temp = sum / module->temp_history[i].sensor_count;
        }
    }
    
    // Calculate rate of change in °C/s
    uint32_t time_diff_ms = newest_time - oldest_time;
    if (time_diff_ms == 0) {
        return 0.0f;  // Avoid division by zero
    }
    
    float temp_diff = newest_avg_temp - oldest_avg_temp;
    float rate = (temp_diff * 1000.0f) / time_diff_ms;  // Convert to °C per second
    
    return rate;
}

/**
 * @brief Check module State of Charge against limits
 */
static esp_err_t check_module_soc(uint8_t module_id) {
    if (module_id >= s_config.number_of_modules) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float soc = 0.0f;
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for SoC check");
        return ESP_ERR_TIMEOUT;
    }
    
    // Get current SoC value
    soc = s_modules[module_id].soc;
    
    xSemaphoreGive(s_data_mutex);
    
    // Check high SoC limit
    if (soc > s_config.soc_high) {
        process_safety_event(SAFETY_EVENT_SOC_HIGH, module_id, soc, s_config.soc_high);
    }
    
    // Check low SoC limit
    if (soc < s_config.soc_low) {
        process_safety_event(SAFETY_EVENT_SOC_LOW, module_id, soc, s_config.soc_low);
    }
    
    return ESP_OK;
}

/**
 * @brief Update system status based on active faults
 */
static esp_err_t update_system_status(void) {
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for status update");
        return ESP_ERR_TIMEOUT;
    }
    
    // Default to OK if no faults
    safety_status_t new_status = SAFETY_STATUS_OK;
    
    // Check for emergency shutdown
    if (s_emergency_shutdown_active) {
        new_status = SAFETY_STATUS_EMERGENCY;
    } else {
        // Otherwise determine status based on active faults
        bool has_emergency = false;
        bool has_critical = false;
        bool has_alarm = false;
        bool has_warning = false;
        
        for (uint8_t i = 0; i < s_fault_count; i++) {
            if (s_fault_list[i].is_active) {
                switch (s_fault_list[i].action_taken) {
                    case SAFETY_ACTION_EMERGENCY_SHUTDOWN:
                        has_emergency = true;
                        break;
                    case SAFETY_ACTION_OPEN_CONTACTORS:
                        has_critical = true;
                        break;
                    case SAFETY_ACTION_PAUSE_CHARGING:
                    case SAFETY_ACTION_PAUSE_DISCHARGING:
                    case SAFETY_ACTION_LIMIT_CURRENT:
                        has_alarm = true;
                        break;
                    case SAFETY_ACTION_ALERT_OPERATOR:
                    case SAFETY_ACTION_LOG_ONLY:
                        has_warning = true;
                        break;
                    default:
                        break;
                }
            }
        }
        
        // Set status based on highest severity fault
        if (has_emergency) {
            new_status = SAFETY_STATUS_EMERGENCY;
        } else if (has_critical) {
            new_status = SAFETY_STATUS_CRITICAL;
        } else if (has_alarm) {
            new_status = SAFETY_STATUS_ALARM;
        } else if (has_warning) {
            new_status = SAFETY_STATUS_WARNING;
        }
    }
    
    // Update status if changed
    if (new_status != s_system_status) {
        ESP_LOGI(TAG, "System status changed: %d -> %d", s_system_status, new_status);
        s_system_status = new_status;
        
        // Set event group bit if fault detected
        if (new_status != SAFETY_STATUS_OK) {
            xEventGroupSetBits(s_event_group, SAFETY_BIT_FAULT_DETECTED);
        } else {
            xEventGroupClearBits(s_event_group, SAFETY_BIT_FAULT_DETECTED);
        }
    }
    
    xSemaphoreGive(s_data_mutex);
    return ESP_OK;
}

/**
 * @brief Process a safety event
 */
static void process_safety_event(safety_event_t event, uint8_t module_id, float measured_value, float threshold_value) {
    // Skip if event is already active for this module
    if (is_fault_active(event, module_id)) {
        return;
    }
    
    // Determine action to take for this event
    safety_action_t action = determine_action_for_event(event);
    
    // Record the fault
    record_fault(event, module_id, measured_value, threshold_value, action);
    
    // Execute the safety action
    execute_safety_action(event, module_id, action);
    
    // Log the event
    log_safety_event(event, module_id, measured_value, threshold_value, action);
    
    // Update system status
    update_system_status();
    
    // Notify callbacks
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
            if (s_callbacks[event][i].callback != NULL) {
                // Release mutex before calling callbacks to prevent deadlocks
                xSemaphoreGive(s_data_mutex);
                s_callbacks[event][i].callback(event, module_id, measured_value, threshold_value, s_callbacks[event][i].user_data);
                if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to retake mutex after callback");
                    break;
                }
            }
        }
        xSemaphoreGive(s_data_mutex);
    }
}

/**
 * @brief Execute a safety action in response to an event
 */
static void execute_safety_action(safety_event_t event, uint8_t module_id, safety_action_t action) {
    ESP_LOGI(TAG, "Executing safety action %d for event %d on module %d", action, event, module_id);
    
    switch (action) {
        case SAFETY_ACTION_NONE:
        case SAFETY_ACTION_LOG_ONLY:
            // No action required
            break;
            
        case SAFETY_ACTION_ALERT_OPERATOR:
            // Send alert to operator via communication channels
            if (s_config.enable_modbus_reporting) {
                modbus_interface_alert(event, module_id);
            }
            if (s_config.enable_canbus_reporting) {
                canbus_interface_alert(event, module_id);
            }
            break;
            
        case SAFETY_ACTION_LIMIT_CURRENT:
            // Signal to power management to limit current
            // This would be implemented by coordinating with the power control module
            break;
            
        case SAFETY_ACTION_INCREASE_COOLING:
            // Request increased cooling from thermal management
            // This would be implemented by coordinating with the thermal module
            break;
            
        case SAFETY_ACTION_PAUSE_CHARGING:
            // Signal to power management to stop charging
            // This would be implemented by coordinating with the power control module
            break;
            
        case SAFETY_ACTION_PAUSE_DISCHARGING:
            // Signal to power management to stop discharging
            // This would be implemented by coordinating with the power control module
            break;
            
        case SAFETY_ACTION_PAUSE_BALANCING:
            // Signal to cell balancer to pause balancing
            // This would be implemented by coordinating with the cell balancer module
            break;
            
        case SAFETY_ACTION_OPEN_CONTACTORS:
            // Open contactors if auto contactor control is enabled
            if (s_auto_contactor_control) {
                // This would be implemented by coordinating with the contactor control module
                ESP_LOGW(TAG, "Opening contactors due to event %d on module %d", event, module_id);
            } else {
                ESP_LOGW(TAG, "Manual contactor control is enabled, cannot auto-open contactors");
            }
            break;
            
        case SAFETY_ACTION_EMERGENCY_SHUTDOWN:
            // Call emergency shutdown function
            {
                char reason[64];
                snprintf(reason, sizeof(reason), "Event %d on module %d triggered emergency shutdown", event, module_id);
                trigger_emergency_shutdown(event, module_id, reason);
            }
            break;
            
        case SAFETY_ACTION_REQUIRE_RESET:
            // This action requires operator interaction to reset
            // Just mark the fault as requiring reset
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown safety action: %d", action);
            break;
    }
}

/**
 * @brief Record a fault in the fault list
 */
static esp_err_t record_fault(safety_event_t event, uint8_t module_id, float measured_value, float threshold_value, safety_action_t action) {
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for fault recording");
        return ESP_ERR_TIMEOUT;
    }
    
    // Set fault bit in module data
    if (module_id < s_config.number_of_modules) {
        s_modules[module_id].fault_bits |= (1 << event);
    }
    
    // Check if we already have this fault recorded
    for (uint8_t i = 0; i < s_fault_count; i++) {
        if (s_fault_list[i].event == event && s_fault_list[i].module_id == module_id) {
            // Update existing fault
            s_fault_list[i].measured_value = measured_value;
            s_fault_list[i].timestamp = esp_timer_get_time() / 1000;  // Convert to ms
            s_fault_list[i].is_active = true;
            s_fault_list[i].duration_ms = 0;  // Reset duration for newly active fault
            
            xSemaphoreGive(s_data_mutex);
            return ESP_OK;
        }
    }
    
    // Add new fault if we have space
    if (s_fault_count < s_config.max_fault_count) {
        s_fault_list[s_fault_count].event = event;
        s_fault_list[s_fault_count].module_id = module_id;
        s_fault_list[s_fault_count].timestamp = esp_timer_get_time() / 1000;  // Convert to ms
        s_fault_list[s_fault_count].measured_value = measured_value;
        s_fault_list[s_fault_count].threshold_value = threshold_value;
        s_fault_list[s_fault_count].duration_ms = 0;
        s_fault_list[s_fault_count].action_taken = action;
        s_fault_list[s_fault_count].requires_reset = (action == SAFETY_ACTION_REQUIRE_RESET);
        s_fault_list[s_fault_count].is_active = true;
        
        s_fault_count++;
    } else {
        ESP_LOGW(TAG, "Fault list full, cannot record new fault");
        xSemaphoreGive(s_data_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    xSemaphoreGive(s_data_mutex);
    return ESP_OK;
}

/**
 * @brief Report faults to external communication channels
 */
static void report_faults_to_communication_channels(void) {
    if (!s_config.enable_modbus_reporting && !s_config.enable_canbus_reporting) {
        return;  // Nothing to do if reporting is disabled
    }
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for fault reporting");
        return;
    }
    
    // Prepare fault data for reporting
    uint8_t active_fault_count = 0;
    for (uint8_t i = 0; i < s_fault_count; i++) {
        if (s_fault_list[i].is_active) {
            active_fault_count++;
            
            // Report to enabled channels
            if (s_config.enable_modbus_reporting) {
                modbus_interface_report_fault(&s_fault_list[i]);
            }
            
            if (s_config.enable_canbus_reporting) {
                canbus_interface_report_fault(&s_fault_list[i]);
            }
        }
    }
    
    // Also report system status
    if (s_config.enable_modbus_reporting) {
        modbus_interface_report_status(s_system_status, active_fault_count);
    }
    
    if (s_config.enable_canbus_reporting) {
        canbus_interface_report_status(s_system_status, active_fault_count);
    }
    
    xSemaphoreGive(s_data_mutex);
}

/**
 * @brief Determine the appropriate action for a safety event
 */
static safety_action_t determine_action_for_event(safety_event_t event) {
    // Define action mapping based on event type
    switch (event) {
        case SAFETY_EVENT_VOLTAGE_HIGH:
            return SAFETY_ACTION_PAUSE_CHARGING;
            
        case SAFETY_EVENT_VOLTAGE_LOW:
            return SAFETY_ACTION_PAUSE_DISCHARGING;
            
        case SAFETY_EVENT_CELL_VOLTAGE_HIGH:
            return SAFETY_ACTION_PAUSE_CHARGING;
            
        case SAFETY_EVENT_CELL_VOLTAGE_LOW:
            return SAFETY_ACTION_PAUSE_DISCHARGING;
            
        case SAFETY_EVENT_CURRENT_HIGH:
            return SAFETY_ACTION_LIMIT_CURRENT;
            
        case SAFETY_EVENT_CURRENT_REVERSE:
            return SAFETY_ACTION_OPEN_CONTACTORS;
            
        case SAFETY_EVENT_TEMPERATURE_HIGH:
            return SAFETY_ACTION_INCREASE_COOLING;
            
        case SAFETY_EVENT_TEMPERATURE_LOW:
            return SAFETY_ACTION_ALERT_OPERATOR;
            
        case SAFETY_EVENT_THERMAL_RUNAWAY:
            return SAFETY_ACTION_EMERGENCY_SHUTDOWN;
            
        case SAFETY_EVENT_SOC_HIGH:
            return SAFETY_ACTION_PAUSE_CHARGING;
            
        case SAFETY_EVENT_SOC_LOW:
            return SAFETY_ACTION_PAUSE_DISCHARGING;
            
        case SAFETY_EVENT_ISOLATION_FAULT:
            return SAFETY_ACTION_OPEN_CONTACTORS;
            
        case SAFETY_EVENT_COMMUNICATION_ERROR:
            return SAFETY_ACTION_ALERT_OPERATOR;
            
        case SAFETY_EVENT_CONTACTOR_FAULT:
            return SAFETY_ACTION_EMERGENCY_SHUTDOWN;
            
        case SAFETY_EVENT_BALANCING_FAULT:
            return SAFETY_ACTION_PAUSE_BALANCING;
            
        case SAFETY_EVENT_WATCHDOG_TIMEOUT:
            return SAFETY_ACTION_EMERGENCY_SHUTDOWN;
            
        case SAFETY_EVENT_SENSOR_FAULT:
            return SAFETY_ACTION_ALERT_OPERATOR;
            
        case SAFETY_EVENT_COOLING_FAULT:
            return SAFETY_ACTION_ALERT_OPERATOR;
            
        case SAFETY_EVENT_SYSTEM_RESET:
            return SAFETY_ACTION_LOG_ONLY;
            
        case SAFETY_EVENT_CONFIG_ERROR:
            return SAFETY_ACTION_ALERT_OPERATOR;
            
        case SAFETY_EVENT_EXTERNAL_COMMAND:
            return SAFETY_ACTION_LOG_ONLY;
            
        default:
            return SAFETY_ACTION_ALERT_OPERATOR;
    }
}

/**
 * @brief Log a safety event to configured destinations
 */
static void log_safety_event(safety_event_t event, uint8_t module_id, float measured_value, float threshold_value, safety_action_t action) {
    char message[256];
    
    // Format message with event details
    snprintf(message, sizeof(message), 
             "SAFETY EVENT: Type=%d, Module=%d, Value=%.3f, Threshold=%.3f, Action=%d",
             event, module_id, measured_value, threshold_value, action);
    
    // Log to ESP log system
    ESP_LOGW(TAG, "%s", message);
    
    // Log to configured destinations
    if (s_config.log_destination & SAFETY_LOG_CONSOLE) {
        // Already logged to console via ESP_LOG above
    }
    
    if (s_config.log_destination & SAFETY_LOG_SD_CARD) {
        // Log to SD card using external logging system
        logger_log_message(message, LOGGER_LEVEL_WARNING);
    }
    
    if (s_config.log_destination & SAFETY_LOG_CLOUDWATCH) {
        // Log to AWS Cloudwatch
        cloudwatch_log_message("BESS_SAFETY", message, CLOUDWATCH_LEVEL_WARNING);
    }
}

/**
 * @brief Check if a fault is already active
 */
static bool is_fault_active(safety_event_t event, uint8_t module_id) {
    bool active = false;
    
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for fault check");
        return false;
    }
    
    // Check module fault bits
    if (module_id < s_config.number_of_modules) {
        active = (s_modules[module_id].fault_bits & (1 << event)) != 0;
    } else {
        // For system-wide events (module_id = 0xFF), check fault list
        for (uint8_t i = 0; i < s_fault_count; i++) {
            if (s_fault_list[i].event == event && 
                s_fault_list[i].module_id == module_id && 
                s_fault_list[i].is_active) {
                active = true;
                break;
            }
        }
    }
    
    xSemaphoreGive(s_data_mutex);
    return active;
}

/**
 * @brief Get threshold value for a safety event
 */
static float get_threshold_for_event(safety_event_t event) {
    switch (event) {
        case SAFETY_EVENT_VOLTAGE_HIGH:
            return s_config.module_voltage_high;
        case SAFETY_EVENT_VOLTAGE_LOW:
            return s_config.module_voltage_low;
        case SAFETY_EVENT_CELL_VOLTAGE_HIGH:
            return s_config.cell_voltage_high;
        case SAFETY_EVENT_CELL_VOLTAGE_LOW:
            return s_config.cell_voltage_low;
        case SAFETY_EVENT_CURRENT_HIGH:
            return s_config.current_high_discharge;
        case SAFETY_EVENT_TEMPERATURE_HIGH:
            return s_config.temperature_high;
        case SAFETY_EVENT_TEMPERATURE_LOW:
            return s_config.temperature_low;
        case SAFETY_EVENT_THERMAL_RUNAWAY:
            return s_config.temp_rate_runaway;
        case SAFETY_EVENT_SOC_HIGH:
            return s_config.soc_high;
        case SAFETY_EVENT_SOC_LOW:
            return s_config.soc_low;
        default:
            return 0.0f;
    }
}