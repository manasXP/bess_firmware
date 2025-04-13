/**
 * @file sensor_interface.c
 * @brief Implementation of the sensor interface for BESS 100KW/200KWH system
 * 
 * This file implements the standardized interface defined in sensor_interface.h
 * for interacting with various sensors in the BESS system.
 * 
 * @note Designed for ESP32-P4 with FreeRTOS
 * 
 * @copyright Copyright (c) 2025
 */

 #include "sensor_interface.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 #include "esp_timer.h"
 #include <string.h>
 #include <stdlib.h>
 
 #define TAG "SENSOR_IF"
 #define MAX_SENSORS 64
 #define MAX_CALLBACKS_PER_EVENT 8
 #define SENSOR_TASK_STACK_SIZE 4096
 #define SENSOR_TASK_PRIORITY 10
 #define EVENT_QUEUE_SIZE 32
 
 /**
  * @brief Structure for callback registration
  */
 typedef struct {
     sensor_event_cb_t callback;
     void *user_data;
     bool used;
 } sensor_callback_t;
 
 /**
  * @brief Structure for sensor internal data
  */
 typedef struct {
     sensor_config_t config;                                        /**< Sensor configuration */
     sensor_value_t last_value;                                     /**< Last sensor reading */
     uint32_t last_error_code;                                      /**< Last error code */
     char last_error_msg[64];                                       /**< Last error message */
     sensor_callback_t callbacks[SENSOR_EVENT_MAX][MAX_CALLBACKS_PER_EVENT]; /**< Event callbacks */
     TaskHandle_t task_handle;                                      /**< Task handle for periodic updates */
     SemaphoreHandle_t mutex;                                       /**< Mutex for thread safety */
     bool is_running;                                               /**< Flag indicating if sensor is active */
     bool is_connected;                                             /**< Flag indicating if sensor is connected */
     uint32_t last_read_time;                                       /**< Timestamp of last reading */
     bool is_valid;                                                 /**< Flag indicating if sensor instance is valid */
 } sensor_data_t;
 
 // Static variables
 static sensor_data_t s_sensors[MAX_SENSORS];
 static SemaphoreHandle_t s_sensor_mutex = NULL;
 static QueueHandle_t s_event_queue = NULL;
 static TaskHandle_t s_event_task_handle = NULL;
 static bool s_initialized = false;
 
 // Forward declarations for internal functions
 static void sensor_event_task(void *arg);
 static void sensor_update_task(void *arg);
 static void sensor_process_event(sensor_event_t *event);
 static esp_err_t sensor_read_internal(sensor_handle_t handle, sensor_value_t *value);
 static esp_err_t sensor_check_thresholds(sensor_handle_t handle, const sensor_value_t *value);
 static void sensor_invoke_callbacks(sensor_handle_t handle, sensor_event_type_t event_type, const sensor_value_t *value);
 static esp_err_t sensor_validate_handle(sensor_handle_t handle);
 
 // Utility function to convert index to handle and vice versa
 static inline sensor_handle_t index_to_handle(int index) {
     return (sensor_handle_t)(intptr_t)(index + 1); // 1-based to avoid NULL handle
 }
 
 static inline int handle_to_index(sensor_handle_t handle) {
     return ((int)(intptr_t)handle) - 1; // Convert back to 0-based index
 }
 
 // String representation tables
 static const char *s_sensor_type_str[] = {
     [SENSOR_TYPE_VOLTAGE] = "Voltage",
     [SENSOR_TYPE_CURRENT] = "Current",
     [SENSOR_TYPE_TEMPERATURE] = "Temperature",
     [SENSOR_TYPE_HUMIDITY] = "Humidity",
     [SENSOR_TYPE_PRESSURE] = "Pressure",
     [SENSOR_TYPE_FLOW] = "Flow",
     [SENSOR_TYPE_DOOR_SWITCH] = "Door Switch",
     [SENSOR_TYPE_SMOKE] = "Smoke",
     [SENSOR_TYPE_CUSTOM] = "Custom"
 };
 
 static const char *s_sensor_protocol_str[] = {
     [SENSOR_PROTOCOL_I2C] = "I2C",
     [SENSOR_PROTOCOL_SPI] = "SPI",
     [SENSOR_PROTOCOL_ONEWIRE] = "OneWire",
     [SENSOR_PROTOCOL_ANALOG] = "Analog",
     [SENSOR_PROTOCOL_DIGITAL] = "Digital",
     [SENSOR_PROTOCOL_MODBUS] = "Modbus",
     [SENSOR_PROTOCOL_CANBUS] = "CANBus",
     [SENSOR_PROTOCOL_CUSTOM] = "Custom"
 };
 
 /**
  * @brief Initialize the sensor subsystem
  */
 esp_err_t sensor_interface_init(void) {
     ESP_LOGI(TAG, "Initializing sensor interface");
     
     if (s_initialized) {
         ESP_LOGW(TAG, "Sensor interface already initialized");
         return ESP_OK;
 }
 
 /**
  * @brief Get a string representation of a sensor type
  */
 const char* sensor_type_to_str(sensor_type_t type) {
     if (type >= SENSOR_TYPE_MAX) {
         return "Unknown";
     }
     return s_sensor_type_str[type];
 }
 
 /**
  * @brief Get a string representation of a sensor protocol
  */
 const char* sensor_protocol_to_str(sensor_protocol_t protocol) {
     if (protocol >= SENSOR_PROTOCOL_MAX) {
         return "Unknown";
     }
     return s_sensor_protocol_str[protocol];
 }
 
 /**
  * @brief Find sensors by type
  */
 esp_err_t sensor_find_by_type(sensor_type_t type, 
                              sensor_handle_t *handles, 
                              size_t max_count, 
                              size_t *count) {
     ESP_LOGD(TAG, "Finding sensors of type: %d", type);
     
     if (handles == NULL || count == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (type >= SENSOR_TYPE_MAX) {
         ESP_LOGE(TAG, "Invalid sensor type");
         return ESP_ERR_INVALID_ARG;
     }
     
     *count = 0;
     
     if (xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Search for sensors of the specified type
     for (int i = 0; i < MAX_SENSORS && *count < max_count; i++) {
         if (s_sensors[i].is_valid && s_sensors[i].config.type == type) {
             handles[*count] = index_to_handle(i);
             (*count)++;
         }
     }
     
     xSemaphoreGive(s_sensor_mutex);
     
     ESP_LOGI(TAG, "Found %zu sensors of type %s", *count, sensor_type_to_str(type));
     
     return ESP_OK;
 }
 
 /**
  * @brief Find sensors by module ID
  */
 esp_err_t sensor_find_by_module(uint8_t module_id, 
                                sensor_handle_t *handles, 
                                size_t max_count, 
                                size_t *count) {
     ESP_LOGD(TAG, "Finding sensors for module: %d", module_id);
     
     if (handles == NULL || count == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_ERR_INVALID_ARG;
     }
     
     *count = 0;
     
     if (xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Search for sensors associated with the specified module
     for (int i = 0; i < MAX_SENSORS && *count < max_count; i++) {
         if (s_sensors[i].is_valid && s_sensors[i].config.module_id == module_id) {
             handles[*count] = index_to_handle(i);
             (*count)++;
         }
     }
     
     xSemaphoreGive(s_sensor_mutex);
     
     ESP_LOGI(TAG, "Found %zu sensors for module %d", *count, module_id);
     
     return ESP_OK;
 }
 
 /**
  * @brief Set update interval for a sensor
  */
 esp_err_t sensor_set_update_interval(sensor_handle_t handle, uint32_t interval_ms) {
     ESP_LOGD(TAG, "Setting update interval for sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     bool was_running = sensor->is_running;
     
     // Stop sensor if running
     if (was_running) {
         xSemaphoreGive(sensor->mutex);
         sensor_stop(handle);
         if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
             ESP_LOGE(TAG, "Failed to take sensor mutex");
             return ESP_ERR_TIMEOUT;
         }
     }
     
     // Update interval
     sensor->config.sampling_interval_ms = interval_ms;
     
     // Restart if it was running
     if (was_running) {
         xSemaphoreGive(sensor->mutex);
         sensor_start(handle);
     } else {
         xSemaphoreGive(sensor->mutex);
     }
     
     ESP_LOGI(TAG, "Update interval for sensor '%s' set to %u ms", 
              sensor->config.name, interval_ms);
     
     return ESP_OK;
 }
 
 /**
  * @brief Get the last error for a sensor
  */
 esp_err_t sensor_get_last_error(sensor_handle_t handle, 
                                uint32_t *error_code,
                                char *error_msg,
                                size_t error_msg_size) {
     ESP_LOGD(TAG, "Getting last error for sensor: %p", handle);
     
     if (error_code == NULL) {
         ESP_LOGE(TAG, "error_code pointer is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy error information
     *error_code = sensor->last_error_code;
     
     if (error_msg != NULL && error_msg_size > 0) {
         strncpy(error_msg, sensor->last_error_msg, error_msg_size);
         error_msg[error_msg_size - 1] = '\0';  // Ensure null termination
     }
     
     xSemaphoreGive(sensor->mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Check if a sensor is connected and responding
  */
 esp_err_t sensor_is_connected(sensor_handle_t handle, bool *is_connected) {
     ESP_LOGD(TAG, "Checking if sensor is connected: %p", handle);
     
     if (is_connected == NULL) {
         ESP_LOGE(TAG, "is_connected pointer is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Perform a basic check by attempting to read the sensor
     // For now, just pretend it's connected, as actual hardware access would be in driver-specific code
     *is_connected = true;
     sensor->is_connected = true;
     
     // In a real implementation, we would try to communicate with the sensor
     // based on its protocol and set is_connected flag accordingly
     
     xSemaphoreGive(sensor->mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Calibrate a sensor
  */
 esp_err_t sensor_calibrate(sensor_handle_t handle, float reference_value) {
     ESP_LOGD(TAG, "Calibrating sensor: %p with reference value %.2f", handle, reference_value);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Read current value
     sensor_value_t current_value;
     err = sensor_read_internal(handle, &current_value);
     
     if (err != ESP_OK) {
         xSemaphoreGive(sensor->mutex);
         ESP_LOGE(TAG, "Failed to read sensor for calibration");
         return err;
     }
     
     // Calculate calibration factors (in a real implementation, this would be sensor-specific)
     // For analog sensors, we might adjust the conversion factor and offset
     if (sensor->config.protocol == SENSOR_PROTOCOL_ANALOG) {
         float current_reading = current_value.value.float_value;
         if (current_reading != 0) {
             float adjustment = reference_value / current_reading;
             sensor->config.config.analog.conversion_factor *= adjustment;
             
             ESP_LOGI(TAG, "Analog sensor calibrated: adjustment factor %.4f applied", adjustment);
         } else {
             xSemaphoreGive(sensor->mutex);
             ESP_LOGE(TAG, "Cannot calibrate with zero reading");
             return ESP_ERR_INVALID_STATE;
         }
     } else {
         // For other sensor types, calibration would be protocol/hardware specific
         ESP_LOGW(TAG, "Calibration not implemented for this sensor protocol");
     }
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' calibrated with reference value %.2f", 
              sensor->config.name, reference_value);
     
     return ESP_OK;
 }
 
 /**
  * @brief Reset a sensor to default settings
  */
 esp_err_t sensor_reset(sensor_handle_t handle) {
     ESP_LOGD(TAG, "Resetting sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     bool was_running = sensor->is_running;
     
     // Stop sensor if running
     if (was_running) {
         xSemaphoreGive(sensor->mutex);
         sensor_stop(handle);
         if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
             ESP_LOGE(TAG, "Failed to take sensor mutex");
             return ESP_ERR_TIMEOUT;
         }
     }
     
     // Reset protocol-specific configuration (example for analog sensors)
     if (sensor->config.protocol == SENSOR_PROTOCOL_ANALOG) {
         sensor->config.config.analog.conversion_factor = 1.0f;
         sensor->config.config.analog.offset = 0.0f;
     }
     
     // Reset general configuration
     sensor->config.warning_threshold = 0.0f;
     sensor->config.critical_threshold = 0.0f;
     sensor->config.emergency_threshold = 0.0f;
     
     // Reset error status
     sensor->last_error_code = 0;
     strcpy(sensor->last_error_msg, "No error");
     
     // Reset last value
     sensor->last_value.valid = 0;
     
     // Restart if it was running
     if (was_running) {
         xSemaphoreGive(sensor->mutex);
         sensor_start(handle);
     } else {
         xSemaphoreGive(sensor->mutex);
     }
     
     ESP_LOGI(TAG, "Sensor '%s' reset to default settings", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Apply a custom command to a sensor
  */
 esp_err_t sensor_custom_command(sensor_handle_t handle,
                                uint32_t command,
                                const void *in_data,
                                size_t in_size,
                                void *out_data,
                                size_t *out_size) {
     ESP_LOGD(TAG, "Custom command %u for sensor: %p", command, handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // In a real implementation, this would dispatch to protocol-specific handlers
     ESP_LOGW(TAG, "Custom command not implemented for this sensor");
     
     xSemaphoreGive(sensor->mutex);
     
     return ESP_ERR_NOT_SUPPORTED;
 }
 
 /* Internal helper functions */
 
 /**
  * @brief Event processing task
  * 
  * This task processes sensor events from the event queue
  */
 static void sensor_event_task(void *arg) {
     ESP_LOGI(TAG, "Event task started");
     
     sensor_event_t event;
     
     while (1) {
         if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) == pdTRUE) {
             sensor_process_event(&event);
         }
     }
 }
 
 /**
  * @brief Sensor update task
  * 
  * This task periodically reads a sensor based on its sampling interval
  */
 static void sensor_update_task(void *arg) {
     sensor_handle_t handle = (sensor_handle_t)arg;
     int index = handle_to_index(handle);
     
     ESP_LOGI(TAG, "Update task started for sensor '%s'", s_sensors[index].config.name);
     
     while (1) {
         // Check if task should exit
         if (xSemaphoreTake(s_sensors[index].mutex, 0) == pdTRUE) {
             if (!s_sensors[index].is_running || s_sensors[index].task_handle == NULL) {
                 xSemaphoreGive(s_sensors[index].mutex);
                 break;  // Exit task
             }
             
             uint32_t interval = s_sensors[index].config.sampling_interval_ms;
             xSemaphoreGive(s_sensors[index].mutex);
             
             // Read sensor
             sensor_value_t value;
             esp_err_t err = sensor_read(handle, &value);
             
             if (err != ESP_OK) {
                 ESP_LOGW(TAG, "Failed to read sensor '%s': %s", 
                          s_sensors[index].config.name, esp_err_to_name(err));
             }
             
             // Wait for next sampling interval
             vTaskDelay(pdMS_TO_TICKS(interval));
         } else {
             // If we can't get the mutex, wait and try again
             vTaskDelay(pdMS_TO_TICKS(10));
         }
     }
     
     ESP_LOGI(TAG, "Update task stopped for sensor '%s'", s_sensors[index].config.name);
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Process a sensor event
  */
 static void sensor_process_event(sensor_event_t *event) {
     if (event == NULL) {
         return;
     }
     
     sensor_handle_t handle = event->sensor;
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         ESP_LOGW(TAG, "Invalid handle in event");
         return;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     ESP_LOGD(TAG, "Processing event type %d for sensor '%s'", 
              event->type, sensor->config.name);
     
     // Invoke callbacks for this event type
     sensor_invoke_callbacks(handle, event->type, &event->value);
 }
 
 /**
  * @brief Read sensor internal implementation
  */
 static esp_err_t sensor_read_internal(sensor_handle_t handle, sensor_value_t *value) {
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if sensor is enabled
     if (!sensor->config.enabled) {
         xSemaphoreGive(sensor->mutex);
         ESP_LOGW(TAG, "Sensor '%s' is disabled", sensor->config.name);
         return ESP_ERR_INVALID_STATE;
     }
     
     // In a real implementation, this would use protocol-specific code to read the sensor
     // For now, simulate reading with random values for demonstration
     uint32_t now = esp_timer_get_time() / 1000;  // Convert us to ms
     
     // Initialize value
     memset(value, 0, sizeof(sensor_value_t));
     value->timestamp_ms = now;
     value->valid = 1;
     value->accuracy = 90;  // 90% accuracy
     
     switch (sensor->config.type) {
         case SENSOR_TYPE_VOLTAGE:
             // Simulate a voltage reading (e.g., 3.0V to 4.2V for a Li-ion cell)
             value->value.float_value = 3.0f + ((float)rand() / RAND_MAX) * 1.2f;
             break;
             
         case SENSOR_TYPE_CURRENT:
             // Simulate a current reading (-10A to +10A)
             value->value.float_value = ((float)rand() / RAND_MAX) * 20.0f - 10.0f;
             break;
             
         case SENSOR_TYPE_TEMPERATURE:
             // Simulate a temperature reading (20°C to 40°C)
             value->value.float_value = 20.0f + ((float)rand() / RAND_MAX) * 20.0f;
             break;
             
         case SENSOR_TYPE_HUMIDITY:
             // Simulate a humidity reading (30% to 70%)
             value->value.float_value = 30.0f + ((float)rand() / RAND_MAX) * 40.0f;
             break;
             
         case SENSOR_TYPE_DOOR_SWITCH:
             // Simulate a door switch (boolean)
             value->value.bool_value = (rand() % 10) > 8;  // 20% chance of being open
             break;
             
         default:
             // Generic simulation for other sensor types
             value->value.float_value = ((float)rand() / RAND_MAX) * 100.0f;
             break;
     }
     
     // Update last value
     memcpy(&sensor->last_value, value, sizeof(sensor_value_t));
     sensor->last_read_time = now;
     
     // Generate DATA_READY event
     sensor_event_t event = {
         .type = SENSOR_EVENT_DATA_READY,
         .sensor = handle,
         .value = *value,
         .user_data = NULL
     };
     
     if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to queue DATA_READY event");
     }
     
     xSemaphoreGive(sensor->mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Check thresholds and generate events if needed
  */
 static esp_err_t sensor_check_thresholds(sensor_handle_t handle, const sensor_value_t *value) {
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Only check thresholds for valid float values
     if (!value->valid || sensor->config.type == SENSOR_TYPE_DOOR_SWITCH) {
         xSemaphoreGive(sensor->mutex);
         return ESP_OK;
     }
     
     float val = value->value.float_value;
     sensor_event_type_t event_type = SENSOR_EVENT_MAX;  // No event by default
     
     // Check emergency threshold first (highest priority)
     if (sensor->config.emergency_threshold > 0 && val >= sensor->config.emergency_threshold) {
         event_type = SENSOR_EVENT_THRESHOLD_EMERGENCY;
         ESP_LOGW(TAG, "EMERGENCY threshold exceeded for sensor '%s': %.2f >= %.2f", 
                  sensor->config.name, val, sensor->config.emergency_threshold);
     }
     // Then check critical threshold
     else if (sensor->config.critical_threshold > 0 && val >= sensor->config.critical_threshold) {
         event_type = SENSOR_EVENT_THRESHOLD_CRITICAL;
         ESP_LOGW(TAG, "CRITICAL threshold exceeded for sensor '%s': %.2f >= %.2f", 
                  sensor->config.name, val, sensor->config.critical_threshold);
     }
     // Finally check warning threshold
     else if (sensor->config.warning_threshold > 0 && val >= sensor->config.warning_threshold) {
         event_type = SENSOR_EVENT_THRESHOLD_WARNING;
         ESP_LOGW(TAG, "WARNING threshold exceeded for sensor '%s': %.2f >= %.2f", 
                  sensor->config.name, val, sensor->config.warning_threshold);
     }
     
     xSemaphoreGive(sensor->mutex);
     
     // Generate threshold event if needed
     if (event_type != SENSOR_EVENT_MAX) {
         sensor_event_t event = {
             .type = event_type,
             .sensor = handle,
             .value = *value,
             .user_data = NULL
         };
         
         if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
             ESP_LOGW(TAG, "Failed to queue threshold event");
         }
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Invoke callbacks for an event
  */
 static void sensor_invoke_callbacks(sensor_handle_t handle, sensor_event_type_t event_type, const sensor_value_t *value) {
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return;
     }
     
     // Create a local copy of callbacks to invoke after releasing the mutex
     sensor_callback_t callbacks[MAX_CALLBACKS_PER_EVENT];
     int callback_count = 0;
     
     for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
         if (sensor->callbacks[event_type][i].used) {
             callbacks[callback_count] = sensor->callbacks[event_type][i];
             callback_count++;
         }
     }
     
     xSemaphoreGive(sensor->mutex);
     
     // Invoke callbacks
     sensor_event_t event = {
         .type = event_type,
         .sensor = handle,
         .value = *value,
         .user_data = NULL
     };
     
     for (int i = 0; i < callback_count; i++) {
         event.user_data = callbacks[i].user_data;
         callbacks[i].callback(&event);
     }
 }
 
 /**
  * @brief Validate sensor handle
  */
 static esp_err_t sensor_validate_handle(sensor_handle_t handle) {
     if (handle == NULL) {
         ESP_LOGE(TAG, "Handle is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     int index = handle_to_index(handle);
     
     if (index < 0 || index >= MAX_SENSORS) {
         ESP_LOGE(TAG, "Handle out of range");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_sensors[index].is_valid) {
         ESP_LOGE(TAG, "Invalid sensor handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }
     }
     
     // Initialize sensor data array
     memset(s_sensors, 0, sizeof(s_sensors));
     
     // Create global mutex
     s_sensor_mutex = xSemaphoreCreateMutex();
     if (s_sensor_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create sensor mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Create event queue
     s_event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(sensor_event_t));
     if (s_event_queue == NULL) {
         vSemaphoreDelete(s_sensor_mutex);
         s_sensor_mutex = NULL;
         ESP_LOGE(TAG, "Failed to create event queue");
         return ESP_ERR_NO_MEM;
     }
     
     // Create event processing task
     BaseType_t task_created = xTaskCreate(
         sensor_event_task,
         "sensor_evt",
         SENSOR_TASK_STACK_SIZE,
         NULL,
         SENSOR_TASK_PRIORITY,
         &s_event_task_handle
     );
     
     if (task_created != pdPASS) {
         vQueueDelete(s_event_queue);
         s_event_queue = NULL;
         vSemaphoreDelete(s_sensor_mutex);
         s_sensor_mutex = NULL;
         ESP_LOGE(TAG, "Failed to create event task");
         return ESP_ERR_NO_MEM;
     }
     
     s_initialized = true;
     ESP_LOGI(TAG, "Sensor interface initialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Deinitialize the sensor subsystem
  */
 esp_err_t sensor_interface_deinit(void) {
     ESP_LOGI(TAG, "Deinitializing sensor interface");
     
     if (!s_initialized) {
         ESP_LOGW(TAG, "Sensor interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Stop and delete all sensors
     for (int i = 0; i < MAX_SENSORS; i++) {
         if (s_sensors[i].is_valid) {
             sensor_handle_t handle = index_to_handle(i);
             sensor_stop(handle);
             sensor_delete(handle);
         }
     }
     
     // Delete event task
     if (s_event_task_handle != NULL) {
         vTaskDelete(s_event_task_handle);
         s_event_task_handle = NULL;
     }
     
     // Delete event queue
     if (s_event_queue != NULL) {
         vQueueDelete(s_event_queue);
         s_event_queue = NULL;
     }
     
     // Delete global mutex
     if (s_sensor_mutex != NULL) {
         vSemaphoreDelete(s_sensor_mutex);
         s_sensor_mutex = NULL;
     }
     
     s_initialized = false;
     ESP_LOGI(TAG, "Sensor interface deinitialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Create a new sensor instance
  */
 esp_err_t sensor_create(const sensor_config_t *config, sensor_handle_t *handle) {
     ESP_LOGD(TAG, "Creating sensor: %s", config->name);
     
     if (!s_initialized) {
         ESP_LOGE(TAG, "Sensor interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (config == NULL || handle == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Validate configuration
     if (config->type >= SENSOR_TYPE_MAX || config->protocol >= SENSOR_PROTOCOL_MAX) {
         ESP_LOGE(TAG, "Invalid sensor type or protocol");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Find an empty slot
     int empty_slot = -1;
     
     if (xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     for (int i = 0; i < MAX_SENSORS; i++) {
         if (!s_sensors[i].is_valid) {
             empty_slot = i;
             break;
         }
     }
     
     if (empty_slot == -1) {
         xSemaphoreGive(s_sensor_mutex);
         ESP_LOGE(TAG, "No free sensor slots available");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize sensor data
     sensor_data_t *sensor = &s_sensors[empty_slot];
     memset(sensor, 0, sizeof(sensor_data_t));
     
     // Copy configuration
     memcpy(&sensor->config, config, sizeof(sensor_config_t));
     
     // Create mutex for this sensor
     sensor->mutex = xSemaphoreCreateMutex();
     if (sensor->mutex == NULL) {
         xSemaphoreGive(s_sensor_mutex);
         ESP_LOGE(TAG, "Failed to create sensor mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize other fields
     sensor->is_valid = true;
     sensor->is_running = false;
     sensor->is_connected = false;
     sensor->last_read_time = 0;
     sensor->last_error_code = 0;
     strcpy(sensor->last_error_msg, "No error");
     
     // Initialize last value
     sensor->last_value.value.float_value = 0.0f;
     sensor->last_value.timestamp_ms = 0;
     sensor->last_value.valid = 0;
     sensor->last_value.accuracy = 0;
     
     // Return handle
     *handle = index_to_handle(empty_slot);
     
     xSemaphoreGive(s_sensor_mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' created successfully (Handle: %p)", 
              config->name, *handle);
     
     return ESP_OK;
 }
 
 /**
  * @brief Delete a sensor instance
  */
 esp_err_t sensor_delete(sensor_handle_t handle) {
     ESP_LOGD(TAG, "Deleting sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Stop the sensor if running
     if (sensor->is_running) {
         xSemaphoreGive(s_sensor_mutex);
         sensor_stop(handle);
         if (xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) != pdTRUE) {
             ESP_LOGE(TAG, "Failed to take sensor mutex");
             return ESP_ERR_TIMEOUT;
         }
     }
     
     // Clean up resources
     if (sensor->mutex != NULL) {
         vSemaphoreDelete(sensor->mutex);
         sensor->mutex = NULL;
     }
     
     // Mark slot as available
     sensor->is_valid = false;
     
     xSemaphoreGive(s_sensor_mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' deleted successfully", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Start a sensor
  */
 esp_err_t sensor_start(sensor_handle_t handle) {
     ESP_LOGD(TAG, "Starting sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     if (sensor->is_running) {
         xSemaphoreGive(sensor->mutex);
         ESP_LOGW(TAG, "Sensor already running");
         return ESP_OK;
     }
     
     // Create update task if sampling interval is specified
     if (sensor->config.sampling_interval_ms > 0) {
         char task_name[16];
         snprintf(task_name, sizeof(task_name), "sens_%d", index);
         
         BaseType_t task_created = xTaskCreate(
             sensor_update_task,
             task_name,
             SENSOR_TASK_STACK_SIZE,
             handle,  // Pass handle as parameter
             SENSOR_TASK_PRIORITY - 1,  // Lower priority than event task
             &sensor->task_handle
         );
         
         if (task_created != pdPASS) {
             sensor->task_handle = NULL;
             xSemaphoreGive(sensor->mutex);
             ESP_LOGE(TAG, "Failed to create sensor update task");
             return ESP_ERR_NO_MEM;
         }
     }
     
     sensor->is_running = true;
     sensor->last_read_time = 0;
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' started successfully", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Stop a sensor
  */
 esp_err_t sensor_stop(sensor_handle_t handle) {
     ESP_LOGD(TAG, "Stopping sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     if (!sensor->is_running) {
         xSemaphoreGive(sensor->mutex);
         ESP_LOGW(TAG, "Sensor already stopped");
         return ESP_OK;
     }
     
     // Stop update task if exists
     if (sensor->task_handle != NULL) {
         TaskHandle_t task_to_delete = sensor->task_handle;
         sensor->task_handle = NULL;
         
         // We need to give the mutex before deleting the task
         // to avoid a potential deadlock if the task is holding it
         xSemaphoreGive(sensor->mutex);
         
         vTaskDelete(task_to_delete);
         
         if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
             ESP_LOGE(TAG, "Failed to take sensor mutex");
             return ESP_ERR_TIMEOUT;
         }
     }
     
     sensor->is_running = false;
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' stopped successfully", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Read a sensor value
  */
 esp_err_t sensor_read(sensor_handle_t handle, sensor_value_t *value) {
     ESP_LOGD(TAG, "Reading sensor: %p", handle);
     
     if (value == NULL) {
         ESP_LOGE(TAG, "Value pointer is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     // Call internal read function
     err = sensor_read_internal(handle, value);
     if (err != ESP_OK) {
         return err;
     }
     
     // Check thresholds and generate events if needed
     return sensor_check_thresholds(handle, value);
 }
 
 /**
  * @brief Read multiple sensors in batch
  */
 esp_err_t sensor_read_batch(const sensor_handle_t *handles, sensor_value_t *values, size_t count) {
     ESP_LOGD(TAG, "Reading %zu sensors in batch", count);
     
     if (handles == NULL || values == NULL || count == 0) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t result = ESP_OK;
     
     // Read each sensor and track overall result
     for (size_t i = 0; i < count; i++) {
         esp_err_t err = sensor_read(handles[i], &values[i]);
         if (err != ESP_OK) {
             result = err;
             ESP_LOGW(TAG, "Failed to read sensor %zu: %s", i, esp_err_to_name(err));
             // Continue with other sensors even if one fails
         }
     }
     
     return result;
 }
 
 /**
  * @brief Register a callback for sensor events
  */
 esp_err_t sensor_register_event_callback(sensor_handle_t handle, 
                                         sensor_event_type_t event_type,
                                         sensor_event_cb_t callback,
                                         void *user_data) {
     ESP_LOGD(TAG, "Registering callback for sensor: %p, event: %d", handle, event_type);
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Callback is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (event_type >= SENSOR_EVENT_MAX) {
         ESP_LOGE(TAG, "Invalid event type");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Find an empty callback slot
     int slot = -1;
     for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
         if (!sensor->callbacks[event_type][i].used) {
             slot = i;
             break;
         }
     }
     
     if (slot == -1) {
         xSemaphoreGive(sensor->mutex);
         ESP_LOGE(TAG, "Maximum callbacks reached for this event type");
         return ESP_ERR_NO_MEM;
     }
     
     // Register callback
     sensor->callbacks[event_type][slot].callback = callback;
     sensor->callbacks[event_type][slot].user_data = user_data;
     sensor->callbacks[event_type][slot].used = true;
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Callback registered for sensor '%s', event type %d", 
              sensor->config.name, event_type);
     
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a callback for sensor events
  */
 esp_err_t sensor_unregister_event_callback(sensor_handle_t handle,
                                           sensor_event_type_t event_type,
                                           sensor_event_cb_t callback) {
     ESP_LOGD(TAG, "Unregistering callback for sensor: %p, event: %d", handle, event_type);
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Callback is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (event_type >= SENSOR_EVENT_MAX) {
         ESP_LOGE(TAG, "Invalid event type");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Find the callback
     bool found = false;
     for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
         if (sensor->callbacks[event_type][i].used && 
             sensor->callbacks[event_type][i].callback == callback) {
             sensor->callbacks[event_type][i].used = false;
             found = true;
             break;
         }
     }
     
     xSemaphoreGive(sensor->mutex);
     
     if (!found) {
         ESP_LOGW(TAG, "Callback not found");
         return ESP_ERR_NOT_FOUND;
     }
     
     ESP_LOGI(TAG, "Callback unregistered for sensor '%s', event type %d", 
              sensor->config.name, event_type);
     
     return ESP_OK;
 }
 
 /**
  * @brief Set sensor thresholds
  */
 esp_err_t sensor_set_thresholds(sensor_handle_t handle,
                                float warning_threshold,
                                float critical_threshold,
                                float emergency_threshold) {
     ESP_LOGD(TAG, "Setting thresholds for sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update thresholds
     sensor->config.warning_threshold = warning_threshold;
     sensor->config.critical_threshold = critical_threshold;
     sensor->config.emergency_threshold = emergency_threshold;
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Thresholds updated for sensor '%s': W=%.2f, C=%.2f, E=%.2f", 
              sensor->config.name, warning_threshold, critical_threshold, emergency_threshold);
     
     return ESP_OK;
 }
 
 /**
  * @brief Get sensor configuration
  */
 esp_err_t sensor_get_config(sensor_handle_t handle, sensor_config_t *config) {
     ESP_LOGD(TAG, "Getting configuration for sensor: %p", handle);
     
     if (config == NULL) {
         ESP_LOGE(TAG, "Config pointer is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy configuration
     memcpy(config, &sensor->config, sizeof(sensor_config_t));
     
     xSemaphoreGive(sensor->mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Update sensor configuration
  */
 esp_err_t sensor_update_config(sensor_handle_t handle, const sensor_config_t *config) {
     ESP_LOGD(TAG, "Updating configuration for sensor: %p", handle);
     
     if (config == NULL) {
         ESP_LOGE(TAG, "Config pointer is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     bool was_running = sensor->is_running;
     
     // Stop sensor if running
     if (was_running) {
         xSemaphoreGive(sensor->mutex);
         sensor_stop(handle);
         if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
             ESP_LOGE(TAG, "Failed to take sensor mutex");
             return ESP_ERR_TIMEOUT;
         }
     }
     
     // Update configuration
     memcpy(&sensor->config, config, sizeof(sensor_config_t));
     
     // Restart if it was running
     if (was_running) {
         xSemaphoreGive(sensor->mutex);
         sensor_start(handle);
     } else {
         xSemaphoreGive(sensor->mutex);
     }
     
     ESP_LOGI(TAG, "Configuration updated for sensor '%s'", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Enable a sensor
  */
 esp_err_t sensor_enable(sensor_handle_t handle) {
     ESP_LOGD(TAG, "Enabling sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     sensor->config.enabled = 1;
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' enabled", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Disable a sensor
  */
 esp_err_t sensor_disable(sensor_handle_t handle) {
     ESP_LOGD(TAG, "Disabling sensor: %p", handle);
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     sensor->config.enabled = 0;
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Sensor '%s' disabled", sensor->config.name);
     
     return ESP_OK;
 }
 
 /**
  * @brief Run diagnostics on a sensor
  */
 esp_err_t sensor_run_diagnostics(sensor_handle_t handle, 
                                 bool *diagnostics_passed, 
                                 uint32_t *error_code) {
     ESP_LOGD(TAG, "Running diagnostics for sensor: %p", handle);
     
     if (diagnostics_passed == NULL) {
         ESP_LOGE(TAG, "diagnostics_passed pointer is NULL");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t err = sensor_validate_handle(handle);
     if (err != ESP_OK) {
         return err;
     }
     
     int index = handle_to_index(handle);
     sensor_data_t *sensor = &s_sensors[index];
     
     if (xSemaphoreTake(sensor->mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take sensor mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Perform simple connectivity check as a basic diagnostic
     bool connected = false;
     err = sensor_is_connected(handle, &connected);
     
     if (err != ESP_OK) {
         xSemaphoreGive(sensor->mutex);
         return err;
     }
     
     *diagnostics_passed = connected;
     
     if (error_code != NULL) {
         *error_code = connected ? 0 : sensor->last_error_code;
     }
     
     xSemaphoreGive(sensor->mutex);
     
     ESP_LOGI(TAG, "Diagnostics for sensor '%s': %s", 
              sensor->config.name, connected ? "PASSED" : "FAILED");
     
     return ESP_OK;
}