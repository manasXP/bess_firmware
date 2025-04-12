/**
 * @file thermal_monitor.c
 * @brief Implementation of thermal monitoring system for BESS
 * 
 * This file implements thermal monitoring functionality for the Battery Energy Storage System (BESS),
 * including temperature measurement, thermal zone determination, runaway detection, and cooling control.
 */

 #include "thermal_monitor.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include <string.h>
 #include <math.h>
 
 #define TAG "THERMAL"
 
 // Constants
 #define MAX_MODULES                  16      // Maximum number of battery modules
 #define MAX_SENSORS_PER_MODULE       8       // Maximum number of temperature sensors per module
 #define HISTORY_BUFFER_SIZE          60      // Store last 60 temperature readings (circular buffer)
 #define TEMP_MONITOR_TASK_STACK_SIZE 4096    // Stack size for thermal monitoring task
 #define TEMP_MONITOR_TASK_PRIORITY   5       // Priority for thermal monitoring task
 #define COOLING_CONTROL_INTERVAL_MS  1000    // Cooling control interval in milliseconds
 #define THERMAL_RUNAWAY_THRESHOLD    5.0f    // Temperature rise rate for potential thermal runaway (°C/min)
 #define MAX_CALLBACKS_PER_ZONE       5       // Maximum number of callbacks per thermal zone
 
 // Default temperature thresholds
 #define DEFAULT_ELEVATED_TEMP        35.0f   // Celsius
 #define DEFAULT_WARNING_TEMP         45.0f   // Celsius
 #define DEFAULT_CRITICAL_TEMP        55.0f   // Celsius
 #define DEFAULT_EMERGENCY_TEMP       65.0f   // Celsius
 
 // Event group bits
 #define COOLING_ACTIVE_BIT          (1 << 0)
 #define COOLING_MANUAL_BIT          (1 << 1)
 #define EMERGENCY_SHUTDOWN_BIT      (1 << 2)
 #define MONITOR_RUNNING_BIT         (1 << 3)
 
 // Temperature history entry
 typedef struct {
     uint32_t timestamp;          // Timestamp in milliseconds
     float temperature;           // Temperature in Celsius
 } temp_history_entry_t;
 
 // Module temperature data
 typedef struct {
     float current_temps[MAX_SENSORS_PER_MODULE];    // Current temperature readings
     uint8_t sensor_count;                           // Number of active sensors
     float max_temp;                                 // Maximum temperature in this module
     float min_temp;                                 // Minimum temperature in this module
     float avg_temp;                                 // Average temperature in this module
     temp_history_entry_t history[HISTORY_BUFFER_SIZE]; // Temperature history (circular buffer)
     uint16_t history_index;                         // Current index in history buffer
     uint16_t history_count;                         // Number of entries in history buffer
     float temp_rise_rate;                           // Temperature rise rate (°C/minute)
     bool runaway_risk;                              // Thermal runaway risk detected
 } module_temp_data_t;
 
 // Callback registration
 typedef struct {
     bess_event_callback_t callback;
     void *user_data;
 } thermal_callback_t;
 
 // Global state
 typedef struct {
     module_temp_data_t modules[MAX_MODULES];        // Temperature data for each module
     uint8_t module_count;                           // Number of modules in the system
     float thresholds[4];                            // Temperature thresholds for each zone
     thermal_zone_t current_zone;                    // Current thermal zone
     uint32_t zone_entry_time;                       // Time when current zone was entered
     cooling_method_t cooling_method;                // Active cooling method
     uint8_t cooling_power;                          // Current cooling power (0-100%)
     uint32_t cooling_end_time;                      // Time when manual cooling should end (0 for auto)
     float ambient_temp;                             // Ambient temperature (°C)
     
     thermal_callback_t zone_callbacks[5][MAX_CALLBACKS_PER_ZONE]; // Callbacks for zone transitions
     uint8_t callback_count[5];                      // Number of registered callbacks per zone
     
     TaskHandle_t monitor_task;                      // Task handle for monitoring task
     SemaphoreHandle_t data_mutex;                   // Mutex for thread-safe data access
     EventGroupHandle_t event_group;                 // Event flags
     
     bool is_initialized;                            // Initialization flag
 } thermal_monitor_state_t;
 
 // Static (global) state
 static thermal_monitor_state_t s_state = {
     .module_count = 0,
     .thresholds = {
         DEFAULT_ELEVATED_TEMP,
         DEFAULT_WARNING_TEMP,
         DEFAULT_CRITICAL_TEMP,
         DEFAULT_EMERGENCY_TEMP
     },
     .current_zone = THERMAL_ZONE_NORMAL,
     .zone_entry_time = 0,
     .cooling_method = COOLING_METHOD_FORCED_AIR,
     .cooling_power = 0,
     .cooling_end_time = 0,
     .ambient_temp = 25.0f,
     .is_initialized = false
 };
 
 // Forward declarations of internal functions
 static void thermal_monitor_task(void *arg);
 static void update_thermal_zone(void);
 static void control_cooling_system(void);
 static void calculate_temp_rise_rate(uint8_t module_id);
 static thermal_zone_t determine_thermal_zone(float temperature);
 static float calculate_average_temperature(const float *temps, uint8_t count);
 static void notify_zone_callbacks(thermal_zone_t new_zone);
 static void add_to_history(uint8_t module_id, float temperature);
 static esp_err_t check_module_id(uint8_t module_id);
 
 /**
  * @brief Initialize the thermal monitoring system
  * 
  * @param cooling_method Cooling method to use
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_init(cooling_method_t cooling_method) {
     // Check if already initialized
     if (s_state.is_initialized) {
         ESP_LOGW(TAG, "Thermal monitor already initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     // Initialize state
     memset(&s_state.modules, 0, sizeof(s_state.modules));
     memset(&s_state.zone_callbacks, 0, sizeof(s_state.zone_callbacks));
     memset(&s_state.callback_count, 0, sizeof(s_state.callback_count));
     
     s_state.module_count = MAX_MODULES; // Will be adjusted as modules report in
     s_state.cooling_method = cooling_method;
     s_state.current_zone = THERMAL_ZONE_NORMAL;
     s_state.zone_entry_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000; // Current time in seconds
     
     // Set default minimum and maximum temperatures for each module
     for (int i = 0; i < MAX_MODULES; i++) {
         for (int j = 0; j < MAX_SENSORS_PER_MODULE; j++) {
             s_state.modules[i].current_temps[j] = 25.0f; // Default ambient temperature
         }
         s_state.modules[i].max_temp = 25.0f;
         s_state.modules[i].min_temp = 25.0f;
         s_state.modules[i].avg_temp = 25.0f;
         s_state.modules[i].sensor_count = 0;
         s_state.modules[i].history_index = 0;
         s_state.modules[i].history_count = 0;
         s_state.modules[i].temp_rise_rate = 0.0f;
         s_state.modules[i].runaway_risk = false;
     }
     
     // Create mutex for thread-safe access to temperature data
     s_state.data_mutex = xSemaphoreCreateMutex();
     if (s_state.data_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create data mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Create event group
     s_state.event_group = xEventGroupCreate();
     if (s_state.event_group == NULL) {
         vSemaphoreDelete(s_state.data_mutex);
         ESP_LOGE(TAG, "Failed to create event group");
         return ESP_ERR_NO_MEM;
     }
     
     s_state.is_initialized = true;
     ESP_LOGI(TAG, "Thermal monitor initialized with %s cooling", 
              cooling_method == COOLING_METHOD_PASSIVE ? "passive" :
              cooling_method == COOLING_METHOD_FORCED_AIR ? "forced air" :
              cooling_method == COOLING_METHOD_LIQUID ? "liquid" : "hybrid");
     
     return ESP_OK;
 }
 
 /**
  * @brief Start the thermal monitoring system
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_start(void) {
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Thermal monitor not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if already running
     EventBits_t bits = xEventGroupGetBits(s_state.event_group);
     if (bits & MONITOR_RUNNING_BIT) {
         ESP_LOGW(TAG, "Thermal monitor already running");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create the monitoring task
     BaseType_t result = xTaskCreate(thermal_monitor_task,
                                    "thermal_monitor",
                                    TEMP_MONITOR_TASK_STACK_SIZE,
                                    NULL,
                                    TEMP_MONITOR_TASK_PRIORITY,
                                    &s_state.monitor_task);
     
     if (result != pdPASS) {
         ESP_LOGE(TAG, "Failed to create thermal monitor task");
         return ESP_ERR_NO_MEM;
     }
     
     // Set the running bit
     xEventGroupSetBits(s_state.event_group, MONITOR_RUNNING_BIT);
     
     ESP_LOGI(TAG, "Thermal monitor started");
     return ESP_OK;
 }
 
 /**
  * @brief Update temperature data for a module
  * 
  * @param module_id ID of the module
  * @param temperatures Array of temperature sensor readings (°C)
  * @param sensor_count Number of temperature sensors
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_update_module(uint8_t module_id,
                                       const float *temperatures,
                                       uint8_t sensor_count) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     esp_err_t err = check_module_id(module_id);
     if (err != ESP_OK) {
         return err;
     }
     
     if (temperatures == NULL || sensor_count == 0 || 
         sensor_count > MAX_SENSORS_PER_MODULE) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Determine number of samples to return
     size_t available_samples = s_state.modules[module_id].history_count;
     size_t samples_to_return = (available_samples < buffer_size) ? 
                               available_samples : buffer_size;
     
     // Copy history data to buffer, starting from the oldest entry
     if (available_samples > 0) {
         // Find the index of the oldest entry in the circular buffer
         uint16_t oldest_idx;
         if (available_samples == HISTORY_BUFFER_SIZE) {
             // Buffer is full, oldest is right after the current index
             oldest_idx = (s_state.modules[module_id].history_index + 1) % HISTORY_BUFFER_SIZE;
         } else {
             // Buffer is not full, oldest is at index 0
             oldest_idx = 0;
         }
         
         // Copy data in chronological order
         for (size_t i = 0; i < samples_to_return; i++) {
             uint16_t source_idx = (oldest_idx + i) % HISTORY_BUFFER_SIZE;
             history_buffer[i] = s_state.modules[module_id].history[source_idx].temperature;
         }
     }
     
     *samples_returned = samples_to_return;
     
     xSemaphoreGive(s_state.data_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set the ambient temperature
  * 
  * @param ambient_temp Ambient temperature in °C
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_set_ambient_temp(float ambient_temp) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Basic validation
     if (ambient_temp < -50.0f || ambient_temp > 70.0f) {
         ESP_LOGW(TAG, "Ambient temperature out of reasonable range: %.1f°C", ambient_temp);
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     s_state.ambient_temp = ambient_temp;
     
     xSemaphoreGive(s_state.data_mutex);
     
     ESP_LOGI(TAG, "Ambient temperature set to %.1f°C", ambient_temp);
     return ESP_OK;
 }
 
 /**
  * @brief Get the ambient temperature
  * 
  * @param[out] ambient_temp Pointer to store ambient temperature
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_ambient_temp(float *ambient_temp) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (ambient_temp == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     *ambient_temp = s_state.ambient_temp;
     
     xSemaphoreGive(s_state.data_mutex);
     return ESP_OK;
 }
 
 /*
  * Internal helper functions
  */
 
 /**
  * Thermal monitoring task
  */
 static void thermal_monitor_task(void *arg) {
     ESP_LOGI(TAG, "Thermal monitoring task started");
     
     const TickType_t control_interval = pdMS_TO_TICKS(COOLING_CONTROL_INTERVAL_MS);
     TickType_t last_control_time = xTaskGetTickCount();
     
     while (1) {
         // Control cooling system at regular intervals
         if ((xTaskGetTickCount() - last_control_time) >= control_interval) {
             control_cooling_system();
             last_control_time = xTaskGetTickCount();
         }
         
         // Task delay
         vTaskDelay(pdMS_TO_TICKS(500)); // 500ms task cycle
     }
 }
 
 /**
  * Update the current thermal zone based on temperature readings
  */
 static void update_thermal_zone(void) {
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex in update_thermal_zone");
         return;
     }
     
     // Find the highest temperature across all modules
     float max_temperature = -100.0f;
     for (int i = 0; i < s_state.module_count; i++) {
         if (s_state.modules[i].sensor_count > 0 && 
             s_state.modules[i].max_temp > max_temperature) {
             max_temperature = s_state.modules[i].max_temp;
         }
     }
     
     // Determine the appropriate thermal zone
     thermal_zone_t new_zone = determine_thermal_zone(max_temperature);
     
     // If zone changed, update and notify
     if (new_zone != s_state.current_zone) {
         ESP_LOGI(TAG, "Thermal zone changed: %d -> %d (%.1f°C)", 
                  s_state.current_zone, new_zone, max_temperature);
         
         // Update zone and entry time
         thermal_zone_t old_zone = s_state.current_zone;
         s_state.current_zone = new_zone;
         s_state.zone_entry_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000; // Current time in seconds
         
         // Special handling for emergency zone
         if (new_zone == THERMAL_ZONE_EMERGENCY) {
             ESP_LOGW(TAG, "EMERGENCY thermal zone entered! Immediate action required.");
             xEventGroupSetBits(s_state.event_group, EMERGENCY_SHUTDOWN_BIT);
         }
         
         // Release mutex before callbacks to prevent deadlock
         xSemaphoreGive(s_state.data_mutex);
         
         // Notify callbacks about zone change
         notify_zone_callbacks(new_zone);
         
         return; // Mutex already given
     }
     
     xSemaphoreGive(s_state.data_mutex);
 }
 
 /**
  * Control the cooling system based on thermal zone and settings
  */
 static void control_cooling_system(void) {
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex in control_cooling_system");
         return;
     }
     
     // Get status bits
     EventBits_t bits = xEventGroupGetBits(s_state.event_group);
     bool manual_control = (bits & COOLING_MANUAL_BIT) != 0;
     bool cooling_active = (bits & COOLING_ACTIVE_BIT) != 0;
     
     // Check if manual control has timed out
     if (manual_control && s_state.cooling_end_time > 0) {
         uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000; // Current time in seconds
         
         if (current_time >= s_state.cooling_end_time) {
             // Manual cooling timed out, switch to automatic
             manual_control = false;
             xEventGroupClearBits(s_state.event_group, COOLING_MANUAL_BIT);
             ESP_LOGI(TAG, "Manual cooling duration expired, switching to automatic");
         }
     }
     
     // If under manual control, use the manually set power level
     if (manual_control) {
         // Just maintain the manually set cooling power
         if (!cooling_active && s_state.cooling_power > 0) {
             // Activate cooling
             xEventGroupSetBits(s_state.event_group, COOLING_ACTIVE_BIT);
             ESP_LOGI(TAG, "Cooling activated (manual): %d%%", s_state.cooling_power);
         } else if (cooling_active && s_state.cooling_power == 0) {
             // Deactivate cooling
             xEventGroupClearBits(s_state.event_group, COOLING_ACTIVE_BIT);
             ESP_LOGI(TAG, "Cooling deactivated (manual)");
         }
     } else {
         // Automatic control based on thermal zone
         uint8_t target_power = 0;
         bool should_activate = false;
         
         switch (s_state.current_zone) {
             case THERMAL_ZONE_NORMAL:
                 target_power = 0; // No cooling needed
                 should_activate = false;
                 break;
                 
             case THERMAL_ZONE_ELEVATED:
                 if (s_state.cooling_method != COOLING_METHOD_PASSIVE) {
                     target_power = 25; // Low power cooling
                     should_activate = true;
                 }
                 break;
                 
             case THERMAL_ZONE_WARNING:
                 if (s_state.cooling_method != COOLING_METHOD_PASSIVE) {
                     target_power = 50; // Medium power cooling
                     should_activate = true;
                 }
                 break;
                 
             case THERMAL_ZONE_CRITICAL:
                 target_power = 75; // High power cooling
                 should_activate = true;
                 break;
                 
             case THERMAL_ZONE_EMERGENCY:
                 target_power = 100; // Maximum cooling
                 should_activate = true;
                 break;
         }
         
         // Update cooling power and state
         if (should_activate != cooling_active) {
             if (should_activate) {
                 xEventGroupSetBits(s_state.event_group, COOLING_ACTIVE_BIT);
                 ESP_LOGI(TAG, "Cooling activated (auto): %d%%", target_power);
             } else {
                 xEventGroupClearBits(s_state.event_group, COOLING_ACTIVE_BIT);
                 ESP_LOGI(TAG, "Cooling deactivated (auto)");
             }
         }
         
         // Update power level if changed
         if (s_state.cooling_power != target_power) {
             s_state.cooling_power = target_power;
             ESP_LOGI(TAG, "Cooling power adjusted to %d%%", target_power);
         }
     }
     
     xSemaphoreGive(s_state.data_mutex);
 }
 
 /**
  * Calculate temperature rise rate for a module
  */
 static void calculate_temp_rise_rate(uint8_t module_id) {
     // Need at least 2 history entries to calculate rate
     if (s_state.modules[module_id].history_count < 2) {
         s_state.modules[module_id].temp_rise_rate = 0.0f;
         return;
     }
     
     // Find the oldest and newest entries
     uint16_t newest_idx = (s_state.modules[module_id].history_index == 0) ? 
                           (HISTORY_BUFFER_SIZE - 1) : 
                           (s_state.modules[module_id].history_index - 1);
     
     uint16_t oldest_idx;
     if (s_state.modules[module_id].history_count == HISTORY_BUFFER_SIZE) {
         // Buffer is full, oldest is at current index
         oldest_idx = s_state.modules[module_id].history_index;
     } else {
         // Buffer is not full, oldest is at index 0
         oldest_idx = 0;
     }
     
     // Get temperatures and timestamps
     float oldest_temp = s_state.modules[module_id].history[oldest_idx].temperature;
     float newest_temp = s_state.modules[module_id].history[newest_idx].temperature;
     uint32_t oldest_time = s_state.modules[module_id].history[oldest_idx].timestamp;
     uint32_t newest_time = s_state.modules[module_id].history[newest_idx].timestamp;
     
     // Calculate time difference in minutes
     float time_diff_min = (float)(newest_time - oldest_time) / (60.0f * 1000.0f);
     
     // Avoid division by zero
     if (time_diff_min < 0.01f) {
         s_state.modules[module_id].temp_rise_rate = 0.0f;
         return;
     }
     
     // Calculate rate of temperature change in °C/min
     float temp_diff = newest_temp - oldest_temp;
     float rate = temp_diff / time_diff_min;
     
     // Only consider positive rates for runaway detection
     s_state.modules[module_id].temp_rise_rate = (rate > 0) ? rate : 0.0f;
 }
 
 /**
  * Determine the thermal zone based on temperature
  */
 static thermal_zone_t determine_thermal_zone(float temperature) {
     if (temperature >= s_state.thresholds[3]) {
         return THERMAL_ZONE_EMERGENCY;
     } else if (temperature >= s_state.thresholds[2]) {
         return THERMAL_ZONE_CRITICAL;
     } else if (temperature >= s_state.thresholds[1]) {
         return THERMAL_ZONE_WARNING;
     } else if (temperature >= s_state.thresholds[0]) {
         return THERMAL_ZONE_ELEVATED;
     } else {
         return THERMAL_ZONE_NORMAL;
     }
 }
 
 /**
  * Calculate average temperature from multiple sensors
  */
 static float calculate_average_temperature(const float *temps, uint8_t count) {
     if (count == 0) {
         return 0.0f;
     }
     
     float sum = 0.0f;
     for (int i = 0; i < count; i++) {
         sum += temps[i];
     }
     
     return sum / count;
 }
 
 /**
  * Notify registered callbacks for a thermal zone
  */
 static void notify_zone_callbacks(thermal_zone_t new_zone) {
     // Make a local copy of callbacks to avoid holding the mutex during callbacks
     thermal_callback_t callbacks[MAX_CALLBACKS_PER_ZONE];
     uint8_t callback_count = 0;
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex in notify_zone_callbacks");
         return;
     }
     
     // Copy callbacks
     callback_count = s_state.callback_count[new_zone];
     memcpy(callbacks, s_state.zone_callbacks[new_zone], 
            callback_count * sizeof(thermal_callback_t));
     
     xSemaphoreGive(s_state.data_mutex);
     
     // Invoke callbacks
     for (int i = 0; i < callback_count; i++) {
         if (callbacks[i].callback != NULL) {
             callbacks[i].callback(BESS_EVENT_TEMPERATURE_THRESHOLD, 
                                 new_zone, 
                                 callbacks[i].user_data);
         }
     }
 }
 
 /**
  * Add a temperature reading to the history buffer
  */
 static void add_to_history(uint8_t module_id, float temperature) {
     // Add to circular buffer
     uint16_t idx = s_state.modules[module_id].history_index;
     
     // Update history entry
     s_state.modules[module_id].history[idx].temperature = temperature;
     s_state.modules[module_id].history[idx].timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
     
     // Increment index and wrap around
     s_state.modules[module_id].history_index = (idx + 1) % HISTORY_BUFFER_SIZE;
     
     // Update count if not already full
     if (s_state.modules[module_id].history_count < HISTORY_BUFFER_SIZE) {
         s_state.modules[module_id].history_count++;
     }
 }
 
 /**
  * Check if module ID is valid
  */
 static esp_err_t check_module_id(uint8_t module_id) {
     if (module_id >= MAX_MODULES) {
         ESP_LOGE(TAG, "Invalid module ID: %d", module_id);
         return ESP_ERR_INVALID_ARG;
     }
     return ESP_OK;
 }
 
 /**
  * @brief Get the current thermal status
  * 
  * @param[out] status Pointer to store the thermal status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_status(thermal_status_t *status) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Find highest and lowest temperatures across all modules
     float highest_temp = -100.0f;
     float lowest_temp = 1000.0f;
     float sum_temp = 0.0f;
     uint8_t hottest_module = 0;
     uint8_t coolest_module = 0;
     uint16_t total_sensors = 0;
     
     for (int i = 0; i < s_state.module_count; i++) {
         if (s_state.modules[i].sensor_count > 0) {
             if (s_state.modules[i].max_temp > highest_temp) {
                 highest_temp = s_state.modules[i].max_temp;
                 hottest_module = i;
             }
             
             if (s_state.modules[i].min_temp < lowest_temp) {
                 lowest_temp = s_state.modules[i].min_temp;
                 coolest_module = i;
             }
             
             sum_temp += s_state.modules[i].avg_temp * s_state.modules[i].sensor_count;
             total_sensors += s_state.modules[i].sensor_count;
         }
     }
     
     // Calculate average temperature
     float avg_temp = (total_sensors > 0) ? (sum_temp / total_sensors) : s_state.ambient_temp;
     
     // Get current zone time
     uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000; // Current time in seconds
     uint32_t time_in_zone = current_time - s_state.zone_entry_time;
     
     // Find maximum temperature rate of rise across all modules
     float max_temp_rate = 0.0f;
     for (int i = 0; i < s_state.module_count; i++) {
         if (s_state.modules[i].temp_rise_rate > max_temp_rate) {
             max_temp_rate = s_state.modules[i].temp_rise_rate;
         }
     }
     
     // Get cooling status
     EventBits_t bits = xEventGroupGetBits(s_state.event_group);
     bool cooling_active = (bits & COOLING_ACTIVE_BIT) != 0;
     
     // Populate status struct
     status->highest_temperature = highest_temp;
     status->lowest_temperature = lowest_temp;
     status->average_temperature = avg_temp;
     status->temperature_rate = max_temp_rate;
     status->current_zone = s_state.current_zone;
     status->hottest_module = hottest_module;
     status->coolest_module = coolest_module;
     status->cooling_active = cooling_active;
     status->cooling_power = s_state.cooling_power;
     status->time_in_current_zone = time_in_zone;
     
     xSemaphoreGive(s_state.data_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set temperature thresholds for different thermal zones
  * 
  * @param elevated_temp Threshold for elevated zone (°C)
  * @param warning_temp Threshold for warning zone (°C)
  * @param critical_temp Threshold for critical zone (°C)
  * @param emergency_temp Threshold for emergency zone (°C)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_set_thresholds(float elevated_temp,
                                        float warning_temp,
                                        float critical_temp,
                                        float emergency_temp) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate threshold ordering
     if (elevated_temp >= warning_temp || 
         warning_temp >= critical_temp || 
         critical_temp >= emergency_temp) {
         ESP_LOGE(TAG, "Invalid threshold ordering");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update thresholds
     s_state.thresholds[0] = elevated_temp;
     s_state.thresholds[1] = warning_temp;
     s_state.thresholds[2] = critical_temp;
     s_state.thresholds[3] = emergency_temp;
     
     ESP_LOGI(TAG, "Thermal thresholds updated: %.1f/%.1f/%.1f/%.1f°C",
              elevated_temp, warning_temp, critical_temp, emergency_temp);
     
     xSemaphoreGive(s_state.data_mutex);
     
     // Re-evaluate current zone with new thresholds
     update_thermal_zone();
     
     return ESP_OK;
 }
 
 /**
  * @brief Get current temperature thresholds
  * 
  * @param[out] elevated_temp Pointer to store elevated threshold
  * @param[out] warning_temp Pointer to store warning threshold
  * @param[out] critical_temp Pointer to store critical threshold
  * @param[out] emergency_temp Pointer to store emergency threshold
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_thresholds(float *elevated_temp,
                                        float *warning_temp,
                                        float *critical_temp,
                                        float *emergency_temp) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (elevated_temp == NULL || warning_temp == NULL || 
         critical_temp == NULL || emergency_temp == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy thresholds
     *elevated_temp = s_state.thresholds[0];
     *warning_temp = s_state.thresholds[1];
     *critical_temp = s_state.thresholds[2];
     *emergency_temp = s_state.thresholds[3];
     
     xSemaphoreGive(s_state.data_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Manually activate the cooling system
  * 
  * @param power_level Cooling power level (0-100%)
  * @param duration_seconds Duration to activate cooling (0 for indefinite)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_activate_cooling(uint8_t power_level, uint32_t duration_seconds) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (power_level > 100) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Set cooling parameters
     s_state.cooling_power = power_level;
     
     // Calculate end time if duration specified
     uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000; // Current time in seconds
     s_state.cooling_end_time = (duration_seconds > 0) ? 
                                (current_time + duration_seconds) : 0;
     
     xSemaphoreGive(s_state.data_mutex);
     
     // Set manual cooling bit and active bit
     xEventGroupSetBits(s_state.event_group, COOLING_MANUAL_BIT | COOLING_ACTIVE_BIT);
     
     ESP_LOGI(TAG, "Manual cooling activated: %d%% power, %s", 
              power_level, 
              duration_seconds > 0 ? 
              "specified duration" : 
              "unlimited duration");
     
     return ESP_OK;
 }
 
 /**
  * @brief Deactivate the cooling system
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_deactivate_cooling(void) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Clear manual cooling parameters
     s_state.cooling_end_time = 0;
     
     xSemaphoreGive(s_state.data_mutex);
     
     // Clear manual cooling bit
     xEventGroupClearBits(s_state.event_group, COOLING_MANUAL_BIT);
     
     // The COOLING_ACTIVE_BIT will be updated by the monitor task based on temperature
     
     ESP_LOGI(TAG, "Manual cooling deactivated");
     
     return ESP_OK;
 }
 
 /**
  * @brief Check for thermal runaway condition
  * 
  * @param[out] runaway_detected Pointer to store result (true if runaway detected)
  * @param[out] affected_module Pointer to store ID of affected module (if any)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_check_runaway(bool *runaway_detected, uint8_t *affected_module) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (runaway_detected == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check for runaway conditions in any module
     bool runaway = false;
     uint8_t module = 0;
     
     for (int i = 0; i < s_state.module_count; i++) {
         if (s_state.modules[i].runaway_risk) {
             runaway = true;
             module = i;
             break;
         }
     }
     
     *runaway_detected = runaway;
     if (affected_module != NULL && runaway) {
         *affected_module = module;
     }
     
     xSemaphoreGive(s_state.data_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for thermal events
  * 
  * @param zone Thermal zone to trigger the callback
  * @param callback Function to call when thermal zone is entered
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_register_callback(thermal_zone_t zone,
                                           bess_event_callback_t callback,
                                           void *user_data) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (zone > THERMAL_ZONE_EMERGENCY || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if we have room for another callback
     if (s_state.callback_count[zone] >= MAX_CALLBACKS_PER_ZONE) {
         xSemaphoreGive(s_state.data_mutex);
         ESP_LOGE(TAG, "Maximum callbacks reached for zone %d", zone);
         return ESP_ERR_NO_MEM;
     }
     
     // Add callback to the list
     uint8_t idx = s_state.callback_count[zone];
     s_state.zone_callbacks[zone][idx].callback = callback;
     s_state.zone_callbacks[zone][idx].user_data = user_data;
     s_state.callback_count[zone]++;
     
     xSemaphoreGive(s_state.data_mutex);
     
     ESP_LOGI(TAG, "Registered callback for thermal zone %d", zone);
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a previously registered callback
  * 
  * @param zone Thermal zone the callback was registered for
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_unregister_callback(thermal_zone_t zone,
                                             bess_event_callback_t callback) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (zone > THERMAL_ZONE_EMERGENCY || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     bool found = false;
     
     // Find and remove the callback
     for (int i = 0; i < s_state.callback_count[zone]; i++) {
         if (s_state.zone_callbacks[zone][i].callback == callback) {
             // Found the callback, remove it by shifting the rest down
             found = true;
             
             // Shift remaining callbacks
             for (int j = i; j < s_state.callback_count[zone] - 1; j++) {
                 s_state.zone_callbacks[zone][j] = s_state.zone_callbacks[zone][j + 1];
             }
             
             s_state.callback_count[zone]--;
             break;
         }
     }
     
     xSemaphoreGive(s_state.data_mutex);
     
     if (!found) {
         ESP_LOGW(TAG, "Callback not found for zone %d", zone);
         return ESP_ERR_NOT_FOUND;
     }
     
     ESP_LOGI(TAG, "Unregistered callback for thermal zone %d", zone);
     return ESP_OK;
 }
 
 /**
  * @brief Get temperature history for a module
  * 
  * @param module_id ID of the module
  * @param[out] history_buffer Buffer to store temperature history
  * @param buffer_size Size of the history buffer
  * @param[out] samples_returned Pointer to store number of samples returned
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_history(uint8_t module_id,
                                     float *history_buffer,
                                     size_t buffer_size,
                                     size_t *samples_returned) {
     if (!s_state.is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     esp_err_t err = check_module_id(module_id);
     if (err != ESP_OK) {
         return err;
     }
     
     if (history_buffer == NULL || buffer_size == 0 || samples_returned == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
     }

    // Copy temperature data
    float max_temp = -100.0f;  // Initialize to very low value
    float min_temp = 1000.0f;  // Initialize to very high value
    float sum_temp = 0.0f;
    
    for (int i = 0; i < sensor_count; i++) {
        // Validate temperature reading (basic sanity check)
        if (temperatures[i] < -50.0f || temperatures[i] > 150.0f) {
            ESP_LOGW(TAG, "Invalid temperature reading: %.1f°C from sensor %d on module %d", 
                     temperatures[i], i, module_id);
            continue;
        }
        
        s_state.modules[module_id].current_temps[i] = temperatures[i];
        
        if (temperatures[i] > max_temp) {
            max_temp = temperatures[i];
        }
        
        if (temperatures[i] < min_temp) {
            min_temp = temperatures[i];
        }
        
        sum_temp += temperatures[i];
    }
    
    // Update module data
    s_state.modules[module_id].sensor_count = sensor_count;
    s_state.modules[module_id].max_temp = max_temp;
    s_state.modules[module_id].min_temp = min_temp;
    s_state.modules[module_id].avg_temp = sum_temp / sensor_count;
    
    // Add to history for trend analysis
    add_to_history(module_id, s_state.modules[module_id].avg_temp);
    
    // Calculate temperature rise rate
    calculate_temp_rise_rate(module_id);
    
    // Check for thermal runaway condition
    if (s_state.modules[module_id].temp_rise_rate > THERMAL_RUNAWAY_THRESHOLD) {
        ESP_LOGW(TAG, "Potential thermal runaway detected in module %d: %.1f°C/min rise rate", 
                 module_id, s_state.modules[module_id].temp_rise_rate);
        s_state.modules[module_id].runaway_risk = true;
    } else {
        s_state.modules[module_id].runaway_risk = false;
    }
    
    xSemaphoreGive(s_state.data_mutex);
    
    // Update thermal zone (outside of mutex to avoid deadlocks)
    update_thermal_zone();
    
    return ESP_OK;
}
