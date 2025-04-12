/**
 * @file converter_interface.c
 * @brief Implementation of interface for power converters in BESS
 * 
 * Implements the abstraction layer for controlling DC-DC and DC-AC converters
 * in the 100KW/200KWH Battery Energy Storage System.
 */

 #include "converter_interface.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include <string.h>
 
 #define TAG "CONVERTER"
 #define MAX_CONVERTERS 8
 #define MAX_CALLBACKS_PER_CONVERTER 10
 
 // Converter event bit definitions for the event group
 #define CONVERTER_EVENT_BIT_STARTUP        (1 << 0)
 #define CONVERTER_EVENT_BIT_SHUTDOWN       (1 << 1)
 #define CONVERTER_EVENT_BIT_MODE_CHANGE    (1 << 2)
 #define CONVERTER_EVENT_BIT_FAULT          (1 << 3)
 #define CONVERTER_EVENT_BIT_WARNING        (1 << 4)
 #define CONVERTER_EVENT_BIT_GRID_CONNECT   (1 << 5)
 #define CONVERTER_EVENT_BIT_GRID_DISCONNECT (1 << 6)
 
 /**
  * @brief Callback registration structure
  */
 typedef struct {
     bool in_use;
     uint8_t event_type;
     converter_event_callback_t callback_func;
     void *user_data;
 }
  converter_callback_t;
 
 /**
  * @brief Converter instance structure
  */
 typedef struct {
     bool initialized;
     converter_type_t type;
     converter_mode_t mode;
     union {
         dcdc_converter_config_t dcdc_config;
         inverter_config_t inverter_config;
     };
     converter_status_t status;
     converter_callback_t callbacks[MAX_CALLBACKS_PER_CONVERTER];
 } converter_instance_t;
 
 // Static variables
 static converter_instance_t s_converters[MAX_CONVERTERS];
 static uint8_t s_converter_count = 0;
 static SemaphoreHandle_t s_converter_mutex = NULL;
 static EventGroupHandle_t s_converter_events = NULL;
 static TaskHandle_t s_converter_task_handle = NULL;
 
 // Forward declarations for internal functions
 static void converter_task(void *pvParameters);
 static void notify_converter_event(uint8_t converter_id, converter_event_t event, void *event_data);
 static esp_err_t validate_converter_id(uint8_t converter_id);
 
 // Implementation of public functions
 
 esp_err_t converter_interface_init(uint8_t converter_count) {
     if (converter_count > MAX_CONVERTERS) {
         ESP_LOGE(TAG, "Too many converters requested: %d (max: %d)", 
                  converter_count, MAX_CONVERTERS);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for protecting converter data
     s_converter_mutex = xSemaphoreCreateMutex();
     if (s_converter_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create converter mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Create event group for converter events
     s_converter_events = xEventGroupCreate();
     if (s_converter_events == NULL) {
         vSemaphoreDelete(s_converter_mutex);
         ESP_LOGE(TAG, "Failed to create converter event group");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize converter instances
     memset(s_converters, 0, sizeof(s_converters));
     s_converter_count = converter_count;
     
     // Create monitoring task
     BaseType_t task_created = xTaskCreatePinnedToCore(
         converter_task,
         "converter_task",
         4096,
         NULL,
         5,
         &s_converter_task_handle,
         1  // Run on core 1
     );
     
     if (task_created != pdPASS) {
         vEventGroupDelete(s_converter_events);
         vSemaphoreDelete(s_converter_mutex);
         ESP_LOGE(TAG, "Failed to create converter task");
         return ESP_ERR_NO_MEM;
     }
     
     ESP_LOGI(TAG, "Converter interface initialized with capacity for %d converters", 
              converter_count);
     return ESP_OK;
 }
 
 esp_err_t converter_register(converter_type_t converter_type, uint8_t *converter_id) {
     if (converter_id == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in register");
         return ESP_ERR_TIMEOUT;
     }
     
     // Find an unused converter slot
     esp_err_t result = ESP_ERR_NO_MEM;
     for (uint8_t i = 0; i < s_converter_count; i++) {
         if (!s_converters[i].initialized) {
             s_converters[i].initialized = true;
             s_converters[i].type = converter_type;
             s_converters[i].mode = CONVERTER_MODE_OFF;
             
             // Initialize status
             memset(&s_converters[i].status, 0, sizeof(converter_status_t));
             s_converters[i].status.mode = CONVERTER_MODE_OFF;
             
             // Clear callbacks
             memset(s_converters[i].callbacks, 0, 
                    sizeof(converter_callback_t) * MAX_CALLBACKS_PER_CONVERTER);
             
             *converter_id = i;
             result = ESP_OK;
             ESP_LOGI(TAG, "Registered converter ID %d of type %d", i, converter_type);
             break;
         }
     }
     
     xSemaphoreGive(s_converter_mutex);
     return result;
 }
 
 esp_err_t converter_configure_dcdc(uint8_t converter_id, const dcdc_converter_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in configure_dcdc");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check converter type
     if (s_converters[converter_id].type != CONVERTER_TYPE_DC_DC &&
         s_converters[converter_id].type != CONVERTER_TYPE_BIDIRECTIONAL) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Converter %d is not a DC-DC type", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set configuration
     memcpy(&s_converters[converter_id].dcdc_config, config, 
            sizeof(dcdc_converter_config_t));
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Configured DC-DC converter %d", converter_id);
     return ESP_OK;
 }
 
 esp_err_t converter_configure_inverter(uint8_t converter_id, const inverter_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in configure_inverter");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check converter type
     if (s_converters[converter_id].type != CONVERTER_TYPE_DC_AC &&
         s_converters[converter_id].type != CONVERTER_TYPE_BIDIRECTIONAL) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Converter %d is not a DC-AC type", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set configuration
     memcpy(&s_converters[converter_id].inverter_config, config, 
            sizeof(inverter_config_t));
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Configured DC-AC inverter %d", converter_id);
     return ESP_OK;
 }
 
 esp_err_t converter_set_mode(uint8_t converter_id, converter_mode_t mode) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_mode");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if mode is valid for this converter type
     bool is_valid_mode = false;
     switch (s_converters[converter_id].type) {
         case CONVERTER_TYPE_DC_DC:
             is_valid_mode = (mode == CONVERTER_MODE_OFF || 
                             mode == CONVERTER_MODE_STANDBY ||
                             mode == CONVERTER_MODE_BUCK ||
                             mode == CONVERTER_MODE_BOOST);
             break;
             
         case CONVERTER_TYPE_DC_AC:
             is_valid_mode = (mode == CONVERTER_MODE_OFF || 
                             mode == CONVERTER_MODE_STANDBY ||
                             mode == CONVERTER_MODE_INVERTER ||
                             mode == CONVERTER_MODE_GRID_FORMING ||
                             mode == CONVERTER_MODE_GRID_FOLLOWING);
             break;
             
         case CONVERTER_TYPE_BIDIRECTIONAL:
             // All modes are potentially valid for bidirectional converters
             is_valid_mode = true;
             break;
     }
     
     if (!is_valid_mode) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Invalid mode %d for converter %d of type %d", 
                  mode, converter_id, s_converters[converter_id].type);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Set the new mode
     converter_mode_t old_mode = s_converters[converter_id].mode;
     s_converters[converter_id].mode = mode;
     s_converters[converter_id].status.mode = mode;
     
     // Set active flag based on mode
     s_converters[converter_id].status.active = (mode != CONVERTER_MODE_OFF && 
                                                mode != CONVERTER_MODE_STANDBY);
     
     xSemaphoreGive(s_converter_mutex);
     
     // Notify mode change event
     if (old_mode != mode) {
         uint32_t mode_data = mode;
         notify_converter_event(converter_id, CONVERTER_EVENT_MODE_CHANGE, (void*)mode_data);
         ESP_LOGI(TAG, "Converter %d mode changed from %d to %d", 
                  converter_id, old_mode, mode);
     }
     
     return ESP_OK;
 }
 
 esp_err_t converter_get_mode(uint8_t converter_id, converter_mode_t *mode) {
     if (mode == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in get_mode");
         return ESP_ERR_TIMEOUT;
     }
     
     *mode = s_converters[converter_id].mode;
     
     xSemaphoreGive(s_converter_mutex);
     return ESP_OK;
 }
 
 esp_err_t converter_start(uint8_t converter_id) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in start");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if already active
     if (s_converters[converter_id].status.active) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGW(TAG, "Converter %d already active", converter_id);
         return ESP_OK;
     }
     
     // Activate the converter based on its current mode
     if (s_converters[converter_id].mode == CONVERTER_MODE_OFF) {
         // If off, change to standby mode first
         s_converters[converter_id].mode = CONVERTER_MODE_STANDBY;
         s_converters[converter_id].status.mode = CONVERTER_MODE_STANDBY;
     }
     
     s_converters[converter_id].status.active = true;
     
     xSemaphoreGive(s_converter_mutex);
     
     // Notify startup event
     notify_converter_event(converter_id, CONVERTER_EVENT_STARTUP, NULL);
     ESP_LOGI(TAG, "Converter %d started", converter_id);
     
     return ESP_OK;
 }
 
 esp_err_t converter_stop(uint8_t converter_id) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in stop");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if already inactive
     if (!s_converters[converter_id].status.active) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGW(TAG, "Converter %d already inactive", converter_id);
         return ESP_OK;
     }
     
     // Deactivate the converter
     s_converters[converter_id].status.active = false;
     s_converters[converter_id].mode = CONVERTER_MODE_STANDBY;
     s_converters[converter_id].status.mode = CONVERTER_MODE_STANDBY;
     
     xSemaphoreGive(s_converter_mutex);
     
     // Notify shutdown event
     notify_converter_event(converter_id, CONVERTER_EVENT_SHUTDOWN, NULL);
     ESP_LOGI(TAG, "Converter %d stopped", converter_id);
     
     return ESP_OK;
 }
 
 esp_err_t converter_set_voltage(uint8_t converter_id, float voltage_v) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_voltage");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check converter type and mode
     bool is_valid = false;
     float min_voltage = 0.0f;
     float max_voltage = 0.0f;
     
     if (s_converters[converter_id].type == CONVERTER_TYPE_DC_DC ||
         s_converters[converter_id].type == CONVERTER_TYPE_BIDIRECTIONAL) {
         min_voltage = s_converters[converter_id].dcdc_config.output_voltage_min;
         max_voltage = s_converters[converter_id].dcdc_config.output_voltage_max;
         is_valid = true;
     }
     else if (s_converters[converter_id].type == CONVERTER_TYPE_DC_AC) {
         // For inverters, we're setting AC output voltage
         min_voltage = s_converters[converter_id].inverter_config.ac_voltage_nominal * 0.8f; // 80% of nominal
         max_voltage = s_converters[converter_id].inverter_config.ac_voltage_nominal * 1.2f; // 120% of nominal
         is_valid = true;
     }
     
     if (!is_valid) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Invalid converter type for setting voltage");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate voltage range
     if (voltage_v < min_voltage || voltage_v > max_voltage) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Voltage %.2fV outside valid range %.2f-%.2fV", 
                  voltage_v, min_voltage, max_voltage);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Set output voltage (hardware-specific implementation would go here)
     // For now, just update status
     s_converters[converter_id].status.output_voltage = voltage_v;
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Set converter %d output voltage to %.2fV", converter_id, voltage_v);
     
     return ESP_OK;
 }
 
 esp_err_t converter_set_current_limit(uint8_t converter_id, float current_a) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_current_limit");
         return ESP_ERR_TIMEOUT;
     }
     
     // Validate current limit (simple validation, would be more specific in real implementation)
     if (current_a <= 0.0f) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Current limit must be positive");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Apply the current limit (hardware-specific implementation would go here)
     
     // Update configuration based on converter type
     if (s_converters[converter_id].type == CONVERTER_TYPE_DC_DC ||
         s_converters[converter_id].type == CONVERTER_TYPE_BIDIRECTIONAL) {
         s_converters[converter_id].dcdc_config.current_limit = current_a;
     }
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Set converter %d current limit to %.2fA", converter_id, current_a);
     
     return ESP_OK;
 }
 
 esp_err_t converter_set_power(uint8_t converter_id, float power_kw) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_power");
         return ESP_ERR_TIMEOUT;
     }
     
     // Get power limit based on converter type
     float power_limit = 0.0f;
     if (s_converters[converter_id].type == CONVERTER_TYPE_DC_DC ||
         s_converters[converter_id].type == CONVERTER_TYPE_BIDIRECTIONAL) {
         power_limit = s_converters[converter_id].dcdc_config.power_limit;
     }
     else if (s_converters[converter_id].type == CONVERTER_TYPE_DC_AC) {
         power_limit = s_converters[converter_id].inverter_config.power_limit;
     }
     
     // Validate power setpoint
     if (fabsf(power_kw) > power_limit) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Power setpoint %.2fkW exceeds limit %.2fkW", 
                  fabsf(power_kw), power_limit);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Set power setpoint (hardware-specific implementation would go here)
     
     // Update status
     if (power_kw >= 0) {
         // Discharge mode (output power)
         s_converters[converter_id].status.output_power = power_kw;
         
         // Estimate input power based on efficiency (e.g., 95%)
         s_converters[converter_id].status.input_power = power_kw / 0.95f;
     } else {
         // Charge mode (input power)
         s_converters[converter_id].status.input_power = -power_kw;
         
         // Estimate output power based on efficiency (e.g., 95%)
         s_converters[converter_id].status.output_power = -power_kw * 0.95f;
     }
     
     // Update converter mode based on power direction if bidirectional
     if (s_converters[converter_id].type == CONVERTER_TYPE_BIDIRECTIONAL) {
         if (power_kw > 0) {
             // Discharge mode
             if (s_converters[converter_id].mode != CONVERTER_MODE_BOOST &&
                 s_converters[converter_id].mode != CONVERTER_MODE_INVERTER) {
                 // Set appropriate mode based on previous configuration
                 converter_mode_t new_mode = (s_converters[converter_id].mode == CONVERTER_MODE_DC_AC) ? 
                                             CONVERTER_MODE_INVERTER : CONVERTER_MODE_BOOST;
                 s_converters[converter_id].mode = new_mode;
                 s_converters[converter_id].status.mode = new_mode;
             }
         } else if (power_kw < 0) {
             // Charge mode
             if (s_converters[converter_id].mode != CONVERTER_MODE_BUCK &&
                 s_converters[converter_id].mode != CONVERTER_MODE_RECTIFIER) {
                 // Set appropriate mode based on previous configuration
                 converter_mode_t new_mode = (s_converters[converter_id].mode == CONVERTER_MODE_DC_AC) ? 
                                             CONVERTER_MODE_RECTIFIER : CONVERTER_MODE_BUCK;
                 s_converters[converter_id].mode = new_mode;
                 s_converters[converter_id].status.mode = new_mode;
             }
         }
     }
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Set converter %d power to %.2fkW", converter_id, power_kw);
     
     return ESP_OK;
 }
 
 esp_err_t converter_get_status(uint8_t converter_id, converter_status_t *status) {
     if (status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in get_status");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy status information
     memcpy(status, &s_converters[converter_id].status, sizeof(converter_status_t));
     
     xSemaphoreGive(s_converter_mutex);
     return ESP_OK;
 }
 
 esp_err_t converter_set_frequency(uint8_t converter_id, float frequency_hz) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_frequency");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if this is an inverter
     if (s_converters[converter_id].type != CONVERTER_TYPE_DC_AC &&
         s_converters[converter_id].type != CONVERTER_TYPE_BIDIRECTIONAL) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Converter %d is not an inverter", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate frequency range (typical grid frequency ranges)
     float nominal_frequency = s_converters[converter_id].inverter_config.ac_frequency_nominal;
     float min_frequency = nominal_frequency * 0.95f; // 95% of nominal
     float max_frequency = nominal_frequency * 1.05f; // 105% of nominal
     
     if (frequency_hz < min_frequency || frequency_hz > max_frequency) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Frequency %.2fHz outside valid range %.2f-%.2fHz", 
                  frequency_hz, min_frequency, max_frequency);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Set frequency (hardware-specific implementation would go here)
     
     // Update status
     s_converters[converter_id].status.grid_frequency = frequency_hz;
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Set converter %d frequency to %.2fHz", converter_id, frequency_hz);
     
     return ESP_OK;
 }
 
 esp_err_t converter_set_power_factor(uint8_t converter_id, float power_factor) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_power_factor");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if this is an inverter
     if (s_converters[converter_id].type != CONVERTER_TYPE_DC_AC &&
         s_converters[converter_id].type != CONVERTER_TYPE_BIDIRECTIONAL) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Converter %d is not an inverter", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate power factor range (0.0 to 1.0)
     if (power_factor < 0.0f || power_factor > 1.0f) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Power factor %.2f outside valid range 0.0-1.0", power_factor);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Set power factor (hardware-specific implementation would go here)
     
     // Update configuration
     s_converters[converter_id].inverter_config.power_factor = power_factor;
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Set converter %d power factor to %.2f", converter_id, power_factor);
     
     return ESP_OK;
 }
 
 esp_err_t converter_set_reactive_power(uint8_t converter_id, float reactive_power_kvar) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in set_reactive_power");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if this is an inverter
     if (s_converters[converter_id].type != CONVERTER_TYPE_DC_AC &&
         s_converters[converter_id].type != CONVERTER_TYPE_BIDIRECTIONAL) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Converter %d is not an inverter", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if reactive power control is enabled
     if (!s_converters[converter_id].inverter_config.reactive_power_enabled) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Reactive power control not enabled for converter %d", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set reactive power (hardware-specific implementation would go here)
     
     // Update configuration
     s_converters[converter_id].inverter_config.reactive_power_setpoint = reactive_power_kvar;
     
     xSemaphoreGive(s_converter_mutex);
     ESP_LOGI(TAG, "Set converter %d reactive power to %.2fkVAR", 
              converter_id, reactive_power_kvar);
     
     return ESP_OK;
 }
 
 esp_err_t converter_connect_grid(uint8_t converter_id) {
     esp_err_t result = validate_converter_id(converter_id);
     if (result != ESP_OK) {
         return result;
     }
     
     if (xSemaphoreTake(s_converter_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take converter mutex in connect_grid");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if this is an inverter
     if (s_converters[converter_id].type != CONVERTER_TYPE_DC_AC &&
         s_converters[converter_id].type != CONVERTER_TYPE_BIDIRECTIONAL) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Converter %d is not an inverter", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if grid-tie operation is enabled
     if (!s_converters[converter_id].inverter_config.grid_tie_enabled) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGE(TAG, "Grid-tie operation not enabled for converter %d", converter_id);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if already connected
     if (s_converters[converter_id].status.grid_connected) {
         xSemaphoreGive(s_converter_mutex);
         ESP_LOGW(TAG, "Converter %d already connected to grid", converter_id);
         return ESP_OK;
     }
     
     // Connect to grid (hardware-specific implementation would go here)
     
     // Update status
     s_converters[converter_id].status.grid_connected = true;
     
     // Set appropriate mode for grid connection
     s_converters[converter_id].mode = CONVERTER_MODE_GRID_FOLLOWING;
     s_converters[converter_id].status.mode = CONVERTER_MODE_GRID_FOLLOWING;
     
     xSemaphoreGive(s_converter_mutex);
     
     // Notify grid connect event
     notify_converter_event(converter_id, CONVERTER_EVENT_GRID_CONNECT, NULL);
     ESP_LOGI(TAG, "Converter %d connected to grid", converter_id);
     
     return ESP_OK;
}