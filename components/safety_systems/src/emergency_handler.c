/**
 * @file emergency_handler.c
 * @brief Implementation of the emergency handling subsystem for BESS Safety Systems
 * 
 * This file implements the emergency detection, response, and recovery functionality
 * for the 100KW/200KWH Battery Energy Storage System (BESS) using 48V, 16KWH LFP modules.
 *
 * @copyright Copyright (c) 2025
 */

 #include "emergency_handler.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include <string.h>
 #include <stdarg.h>
 
 // Logging includes
 #include "log_manager.h"  // Custom logging component that supports console, SD card, and AWS CloudWatch
 
 // Define tag for ESP logging
 static const char *TAG = "EMERGENCY";
 
 // Maximum number of registered callbacks
 #define MAX_CALLBACKS 10
 
 // Event group bits for emergency handler
 #define EG_EMERGENCY_ACTIVE      (1 << 0)
 #define EG_WATCHDOG_ACTIVE       (1 << 1)
 #define EG_MANUAL_TRIGGER        (1 << 2)
 #define EG_SELF_TEST_RUNNING     (1 << 3)
 #define EG_RECOVERY_ATTEMPT      (1 << 4)
 #define EG_SHUTDOWN_REQUESTED    (1 << 5)
 #define EG_CONTACTOR_OPEN        (1 << 6)
 
 // Emergency event history size
 #define EVENT_HISTORY_SIZE 20
 
 // Structure to hold callback information
 typedef struct {
     emergency_callback_t cb_func;    /*!< Callback function */
     void *user_data;                 /*!< User data to pass to callback */
     uint32_t mask;                   /*!< Event type mask for this callback */
     bool active;                     /*!< Whether this callback slot is active */
 } callback_info_t;
 
 // Main emergency handler control structure
 typedef struct {
     emergency_handler_config_t config;           /*!< Configuration settings */
     TaskHandle_t handler_task;                   /*!< Handle for emergency handler task */
     uint32_t watchdog_last_feed;                 /*!< Last time the watchdog was fed */
     bool initialized;                            /*!< Initialization state */
     bool running;                                /*!< Task running state */
     emergency_state_t state;                     /*!< Current emergency state */
     emergency_type_t active_emergencies;         /*!< Bitmask of currently active emergencies */
     emergency_response_t response_map[16];       /*!< Mapping of emergency types to responses */
     uint8_t recovery_attempts;                   /*!< Current recovery attempt count */
     uint32_t last_recovery_time;                 /*!< Timestamp of last recovery attempt */
     EventGroupHandle_t event_group;              /*!< FreeRTOS event group for signaling */
     SemaphoreHandle_t ctrl_mutex;                /*!< Mutex for protecting control structure */
     callback_info_t callbacks[MAX_CALLBACKS];    /*!< Array of registered callbacks */
     emergency_event_t event_history[EVENT_HISTORY_SIZE]; /*!< Circular buffer of recent events */
     uint8_t history_index;                       /*!< Current index in circular buffer */
     uint32_t event_count;                        /*!< Total count of emergency events */
     SemaphoreHandle_t log_mutex;                 /*!< Mutex for protecting logging operations */
 } emergency_handler_ctrl_t;
 
 // Static instance of the control structure
 static emergency_handler_ctrl_t s_ctrl;
 
 // Forward declarations for internal functions
 static void emergency_handler_task(void *pvParameters);
 static esp_err_t execute_emergency_response(emergency_event_t *event);
 static void notify_callbacks(emergency_event_t *event);
 static esp_err_t log_emergency_event(emergency_event_t *event);
 static esp_err_t add_to_event_history(emergency_event_t *event);
 static esp_err_t check_watchdog(void);
 static esp_err_t attempt_recovery(void);
 static bool is_emergency_condition_cleared(emergency_type_t type);
 static void execute_specific_response(emergency_response_t response, emergency_event_t *event);
 
 // Default configuration
 static const emergency_handler_config_t default_config = {
     .watchdog_timeout_ms = 5000,         // 5 seconds watchdog timeout
     .auto_recover_enabled = true,        // Auto-recovery enabled
     .recovery_cooldown_ms = 60000,       // 1 minute cooldown
     .max_recovery_attempts = 3,          // 3 maximum recovery attempts
     .default_resp = RESPONSE_ALERT | RESPONSE_LIMIT_POWER, // Default response
     .task_stack_size = 4096,             // 4K stack
     .task_priority = 10,                 // High priority (tskIDLE_PRIORITY + 10)
     .log_to_console = true,              // Log to console by default
     .log_to_sd = true,                   // Log to SD by default
     .log_to_cloud = false                // Don't log to cloud by default
 };
 
 // String mappings for enum values
 static const char* emergency_type_strings[] = {
     "None",
     "Overvoltage",
     "Undervoltage",
     "Overcurrent",
     "Short Circuit",
     "Overtemperature",
     "Thermal Runaway",
     "Cell Imbalance",
     "Isolation Fault",
     "Communication Failure",
     "BMS Failure",
     "Grid Fault",
     "Manual Trigger",
     "Watchdog Timeout",
     "",  // 14 (unused)
     "Unknown"
 };
 
 static const char* emergency_severity_strings[] = {
     "Info",
     "Low",
     "Medium",
     "High",
     "Critical"
 };
 
 static const char* emergency_state_strings[] = {
     "Normal",
     "Active",
     "Recovering",
     "Lockout"
 };
 
 /**
  * @brief Initialize the emergency handler with the specified configuration
  *
  * @param config Pointer to configuration structure, or NULL for defaults
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_init(const emergency_handler_config_t *config) {
     if (s_ctrl.initialized) {
         ESP_LOGW(TAG, "Emergency handler already initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Clear the entire control structure
     memset(&s_ctrl, 0, sizeof(emergency_handler_ctrl_t));
     
     // Use provided config or default
     if (config) {
         memcpy(&s_ctrl.config, config, sizeof(emergency_handler_config_t));
     } else {
         memcpy(&s_ctrl.config, &default_config, sizeof(emergency_handler_config_t));
     }
     
     // Create mutex and event group
     s_ctrl.ctrl_mutex = xSemaphoreCreateMutex();
     if (!s_ctrl.ctrl_mutex) {
         ESP_LOGE(TAG, "Failed to create control mutex");
         return ESP_ERR_NO_MEM;
     }
     
     s_ctrl.log_mutex = xSemaphoreCreateMutex();
     if (!s_ctrl.log_mutex) {
         vSemaphoreDelete(s_ctrl.ctrl_mutex);
         ESP_LOGE(TAG, "Failed to create log mutex");
         return ESP_ERR_NO_MEM;
     }
     
     s_ctrl.event_group = xEventGroupCreate();
     if (!s_ctrl.event_group) {
         vSemaphoreDelete(s_ctrl.ctrl_mutex);
         vSemaphoreDelete(s_ctrl.log_mutex);
         ESP_LOGE(TAG, "Failed to create event group");
         return ESP_ERR_NO_MEM;
     }
     
     // Setup default response map
     for (int i = 0; i < 16; i++) {
         s_ctrl.response_map[i] = s_ctrl.config.default_resp;
     }
     
     // Set specific responses for certain emergency types
     s_ctrl.response_map[EMERGENCY_OVER_VOLTAGE] = RESPONSE_ALERT | 
                                                   RESPONSE_STOP_CHARGE |
                                                   RESPONSE_DISCONNECT;
     
     s_ctrl.response_map[EMERGENCY_UNDER_VOLTAGE] = RESPONSE_ALERT | 
                                                    RESPONSE_STOP_DISCHARGE |
                                                    RESPONSE_DISCONNECT;
     
     s_ctrl.response_map[EMERGENCY_OVER_CURRENT] = RESPONSE_ALERT | 
                                                  RESPONSE_LIMIT_POWER |
                                                  RESPONSE_DISCONNECT;
     
     s_ctrl.response_map[EMERGENCY_SHORT_CIRCUIT] = RESPONSE_ALERT | 
                                                   RESPONSE_DISCONNECT |
                                                   RESPONSE_CONTACTOR_OPEN |
                                                   RESPONSE_SHUTDOWN;
     
     s_ctrl.response_map[EMERGENCY_OVER_TEMP] = RESPONSE_ALERT | 
                                               RESPONSE_COOL_SYSTEM |
                                               RESPONSE_LIMIT_POWER;
     
     s_ctrl.response_map[EMERGENCY_THERMAL_RUNAWAY] = RESPONSE_ALERT | 
                                                     RESPONSE_COOL_SYSTEM |
                                                     RESPONSE_DISCONNECT |
                                                     RESPONSE_CONTACTOR_OPEN |
                                                     RESPONSE_SHUTDOWN;
     
     // Initialize state values
     s_ctrl.state = EMERGENCY_STATE_NORMAL;
     s_ctrl.active_emergencies = EMERGENCY_NONE;
     s_ctrl.watchdog_last_feed = xTaskGetTickCount() * portTICK_PERIOD_MS;
     s_ctrl.recovery_attempts = 0;
     s_ctrl.initialized = true;
     
     // Configure logging
     log_manager_configure(s_ctrl.config.log_to_console, 
                          s_ctrl.config.log_to_sd, 
                          s_ctrl.config.log_to_cloud);
     
     ESP_LOGI(TAG, "Emergency handler initialized");
     return ESP_OK;
 }
 
 /**
  * @brief Start the emergency handler task
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_start(void) {
     if (!s_ctrl.initialized) {
         ESP_LOGE(TAG, "Emergency handler not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (s_ctrl.running) {
         ESP_LOGW(TAG, "Emergency handler already running");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create the emergency handler task
     BaseType_t res = xTaskCreate(
         emergency_handler_task,
         "emergency_handler",
         s_ctrl.config.task_stack_size,
         NULL,
         s_ctrl.config.task_priority,
         &s_ctrl.handler_task
     );
     
     if (res != pdPASS) {
         ESP_LOGE(TAG, "Failed to create emergency handler task");
         return ESP_ERR_NO_MEM;
     }
     
     s_ctrl.running = true;
     ESP_LOGI(TAG, "Emergency handler started");
     return ESP_OK;
 }
 
 /**
  * @brief Main emergency handler task function
  */
 static void emergency_handler_task(void *pvParameters) {
     TickType_t last_wake_time = xTaskGetTickCount();
     const TickType_t check_interval = pdMS_TO_TICKS(250); // 4 Hz check rate
     
     ESP_LOGI(TAG, "Emergency handler task started");
     
     // Initial watchdog feed
     s_ctrl.watchdog_last_feed = xTaskGetTickCount() * portTICK_PERIOD_MS;
     
     while (1) {
         // Check watchdog status
         check_watchdog();
         
         // Get current system state
         emergency_state_t current_state = EMERGENCY_STATE_NORMAL;
         
         if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             current_state = s_ctrl.state;
             xSemaphoreGive(s_ctrl.ctrl_mutex);
         }
         
         // Handle different states
         switch (current_state) {
             case EMERGENCY_STATE_NORMAL:
                 // In normal state, just monitor for emergencies
                 break;
                 
             case EMERGENCY_STATE_ACTIVE:
                 // Active emergency state, check if we should try recovery
                 if (s_ctrl.config.auto_recover_enabled) {
                     uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                     if ((now - s_ctrl.last_recovery_time) > s_ctrl.config.recovery_cooldown_ms &&
                         s_ctrl.recovery_attempts < s_ctrl.config.max_recovery_attempts) {
                         attempt_recovery();
                     }
                 }
                 break;
                 
             case EMERGENCY_STATE_RECOVERING:
                 // Check if recovery completed successfully
                 if (is_emergency_condition_cleared(s_ctrl.active_emergencies)) {
                     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                         s_ctrl.state = EMERGENCY_STATE_NORMAL;
                         s_ctrl.active_emergencies = EMERGENCY_NONE;
                         xSemaphoreGive(s_ctrl.ctrl_mutex);
                         
                         ESP_LOGI(TAG, "Recovery successful, returning to normal operation");
                         
                         // Log recovery event
                         emergency_event_t recovery_event = {
                             .type = EMERGENCY_NONE,
                             .severity = SEVERITY_INFO,
                             .module_id = 0xFF,
                             .measured_value = 0,
                             .threshold_value = 0,
                             .timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS
                         };
                         snprintf(recovery_event.description, sizeof(recovery_event.description),
                                 "System recovered after %d attempts", s_ctrl.recovery_attempts);
                         
                         log_emergency_event(&recovery_event);
                     }
                 }
                 break;
                 
             case EMERGENCY_STATE_LOCKOUT:
                 // In lockout state, only manual intervention can help
                 break;
                 
             default:
                 ESP_LOGW(TAG, "Unknown emergency state: %d", current_state);
                 break;
         }
         
         // Check event group for manual triggers or other signals
         EventBits_t bits = xEventGroupGetBits(s_ctrl.event_group);
         
         if (bits & EG_MANUAL_TRIGGER) {
             ESP_LOGI(TAG, "Manual emergency trigger detected");
             xEventGroupClearBits(s_ctrl.event_group, EG_MANUAL_TRIGGER);
             
             // Manual trigger already handled by the trigger function
         }
         
         if (bits & EG_SHUTDOWN_REQUESTED) {
             ESP_LOGI(TAG, "Emergency shutdown in progress");
             // Additional shutdown code could go here
         }
         
         // Run at our specified interval
         vTaskDelayUntil(&last_wake_time, check_interval);
     }
 }
 
 /**
  * @brief Register a callback function for emergency notifications
  */
 esp_err_t emergency_handler_register_callback(emergency_callback_t callback, 
                                             void *user_data,
                                             uint32_t mask) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (!callback) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t ret = ESP_ERR_NO_MEM;
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Find an empty slot
         for (int i = 0; i < MAX_CALLBACKS; i++) {
             if (!s_ctrl.callbacks[i].active) {
                 s_ctrl.callbacks[i].cb_func = callback;
                 s_ctrl.callbacks[i].user_data = user_data;
                 s_ctrl.callbacks[i].mask = mask;
                 s_ctrl.callbacks[i].active = true;
                 ret = ESP_OK;
                 break;
             }
         }
         xSemaphoreGive(s_ctrl.ctrl_mutex);
     } else {
         ret = ESP_ERR_TIMEOUT;
     }
     
     return ret;
 }
 
 /**
  * @brief Unregister a previously registered callback function
  */
 esp_err_t emergency_handler_unregister_callback(emergency_callback_t callback) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (!callback) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t ret = ESP_ERR_NOT_FOUND;
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Find the callback
         for (int i = 0; i < MAX_CALLBACKS; i++) {
             if (s_ctrl.callbacks[i].active && s_ctrl.callbacks[i].cb_func == callback) {
                 s_ctrl.callbacks[i].active = false;
                 ret = ESP_OK;
                 break;
             }
         }
         xSemaphoreGive(s_ctrl.ctrl_mutex);
     } else {
         ret = ESP_ERR_TIMEOUT;
     }
     
     return ret;
 }
 
 /**
  * @brief Trigger an emergency response
  */
 esp_err_t emergency_handler_trigger(emergency_type_t type,
                                   emergency_severity_t severity,
                                   uint8_t module_id,
                                   float measured_value,
                                   float threshold_value,
                                   const char *description) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (type == EMERGENCY_NONE) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create an emergency event
     emergency_event_t event = {
         .type = type,
         .severity = severity,
         .module_id = module_id,
         .measured_value = measured_value,
         .threshold_value = threshold_value,
         .timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS
     };
     
     // Copy description if provided
     if (description) {
         strncpy(event.description, description, sizeof(event.description) - 1);
         event.description[sizeof(event.description) - 1] = '\0';
     } else {
         snprintf(event.description, sizeof(event.description), 
                 "%s emergency detected", emergency_handler_type_to_string(type));
     }
     
     // Update control data
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Add this to active emergencies
         s_ctrl.active_emergencies |= type;
         
         // Update state based on severity
         if (severity >= SEVERITY_HIGH) {
             s_ctrl.state = EMERGENCY_STATE_ACTIVE;
             xEventGroupSetBits(s_ctrl.event_group, EG_EMERGENCY_ACTIVE);
         }
         
         // Handle critical severity
         if (severity == SEVERITY_CRITICAL) {
             // Critical severity forces lockout state
             s_ctrl.state = EMERGENCY_STATE_LOCKOUT;
         }
         
         xSemaphoreGive(s_ctrl.ctrl_mutex);
     }
     
     // Log the event
     log_emergency_event(&event);
     
     // Add to history
     add_to_event_history(&event);
     
     // Execute the response
     execute_emergency_response(&event);
     
     // Notify callbacks
     notify_callbacks(&event);
     
     return ESP_OK;
 }
 
 /**
  * @brief Manual emergency trigger
  */
 esp_err_t emergency_handler_manual_trigger(const char *description) {
     xEventGroupSetBits(s_ctrl.event_group, EG_MANUAL_TRIGGER);
     
     return emergency_handler_trigger(
         EMERGENCY_MANUAL_TRIGGER,
         SEVERITY_HIGH,
         0xFF,  // System-wide
         0.0f,  // No measured value
         0.0f,  // No threshold
         description ? description : "Manual emergency trigger"
     );
 }
 
 /**
  * @brief Check if any emergency is currently active
  */
 esp_err_t emergency_handler_is_active(bool *active) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (!active) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         *active = (s_ctrl.active_emergencies != EMERGENCY_NONE);
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Get the current emergency state
  */
 esp_err_t emergency_handler_get_state(emergency_state_t *state) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (!state) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         *state = s_ctrl.state;
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Get details of the most recent emergency event
  */
 esp_err_t emergency_handler_get_last_event(emergency_event_t *event) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (!event) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Get previous index in circular buffer
         uint8_t prev_idx = (s_ctrl.history_index == 0) ? 
                           (EVENT_HISTORY_SIZE - 1) : 
                           (s_ctrl.history_index - 1);
         
         // Copy the last event
         memcpy(event, &s_ctrl.event_history[prev_idx], sizeof(emergency_event_t));
         
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Attempt to clear an active emergency
  */
 esp_err_t emergency_handler_clear(emergency_type_t type, bool force) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     esp_err_t ret = ESP_OK;
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Check if in lockout state
         if (s_ctrl.state == EMERGENCY_STATE_LOCKOUT && !force) {
             ESP_LOGW(TAG, "System in lockout state, cannot clear emergency without force");
             ret = ESP_ERR_INVALID_STATE;
         } else {
             if (type == EMERGENCY_NONE) {
                 // Clear all emergencies if forced or conditions are actually cleared
                 if (force || is_emergency_condition_cleared(s_ctrl.active_emergencies)) {
                     s_ctrl.active_emergencies = EMERGENCY_NONE;
                     s_ctrl.state = EMERGENCY_STATE_NORMAL;
                     s_ctrl.recovery_attempts = 0;
                     xEventGroupClearBits(s_ctrl.event_group, EG_EMERGENCY_ACTIVE);
                     ESP_LOGI(TAG, "All emergencies cleared");
                 } else {
                     ESP_LOGW(TAG, "Cannot clear emergencies, conditions still present");
                     ret = ESP_ERR_INVALID_STATE;
                 }
             } else {
                 // Clear specific emergency
                 if (force || is_emergency_condition_cleared(type)) {
                     s_ctrl.active_emergencies &= ~type;
                     
                     // If no more active emergencies, return to normal state
                     if (s_ctrl.active_emergencies == EMERGENCY_NONE) {
                         s_ctrl.state = EMERGENCY_STATE_NORMAL;
                         s_ctrl.recovery_attempts = 0;
                         xEventGroupClearBits(s_ctrl.event_group, EG_EMERGENCY_ACTIVE);
                     }
                     
                     ESP_LOGI(TAG, "Emergency %s cleared", emergency_handler_type_to_string(type));
                 } else {
                     ESP_LOGW(TAG, "Cannot clear emergency %s, condition still present", 
                             emergency_handler_type_to_string(type));
                     ret = ESP_ERR_INVALID_STATE;
                 }
             }
         }
         
         xSemaphoreGive(s_ctrl.ctrl_mutex);
     } else {
         ret = ESP_ERR_TIMEOUT;
     }
     
     return ret;
 }
 
 /**
  * @brief Set the response action for a specific emergency type
  */
 esp_err_t emergency_handler_set_response(emergency_type_t type, emergency_response_t response) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (type == EMERGENCY_NONE || type >= EMERGENCY_UNKNOWN) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Find the bit position of the emergency type
         int bit_pos = 0;
         emergency_type_t temp = type;
         while (temp > 1) {
             temp >>= 1;
             bit_pos++;
         }
         
         if (bit_pos < 16) {
             s_ctrl.response_map[bit_pos] = response;
             ESP_LOGI(TAG, "Set response for %s emergency to 0x%04X", 
                     emergency_handler_type_to_string(type), response);
         }
         
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Reset the emergency handler
  */
 esp_err_t emergency_handler_reset(bool hard_reset) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         // Reset state
         s_ctrl.state = EMERGENCY_STATE_NORMAL;
         s_ctrl.active_emergencies = EMERGENCY_NONE;
         s_ctrl.recovery_attempts = 0;
         s_ctrl.last_recovery_time = 0;
         
         // Clear event groups
         xEventGroupClearBits(s_ctrl.event_group, 
                            EG_EMERGENCY_ACTIVE | 
                            EG_WATCHDOG_ACTIVE | 
                            EG_MANUAL_TRIGGER | 
                            EG_RECOVERY_ATTEMPT |
                            EG_SHUTDOWN_REQUESTED |
                            EG_CONTACTOR_OPEN);
         
         if (hard_reset) {
             // Reset history and counters
             memset(s_ctrl.event_history, 0, sizeof(s_ctrl.event_history));
             s_ctrl.history_index = 0;
             s_ctrl.event_count = 0;
             
             // Reset response map to defaults
             for (int i = 0; i < 16; i++) {
                 s_ctrl.response_map[i] = s_ctrl.config.default_resp;
             }
             
             // Reset specific responses
             s_ctrl.response_map[EMERGENCY_OVER_VOLTAGE] = RESPONSE_ALERT | 
                                                           RESPONSE_STOP_CHARGE |
                                                           RESPONSE_DISCONNECT;
             
             s_ctrl.response_map[EMERGENCY_UNDER_VOLTAGE] = RESPONSE_ALERT | 
                                                            RESPONSE_STOP_DISCHARGE |
                                                            RESPONSE_DISCONNECT;
             
             s_ctrl.response_map[EMERGENCY_OVER_CURRENT] = RESPONSE_ALERT | 
                                                          RESPONSE_LIMIT_POWER |
                                                          RESPONSE_DISCONNECT;
             
             s_ctrl.response_map[EMERGENCY_SHORT_CIRCUIT] = RESPONSE_ALERT | 
                                                           RESPONSE_DISCONNECT |
                                                           RESPONSE_CONTACTOR_OPEN |
                                                           RESPONSE_SHUTDOWN;
             
             s_ctrl.response_map[EMERGENCY_OVER_TEMP] = RESPONSE_ALERT | 
                                                       RESPONSE_COOL_SYSTEM |
                                                       RESPONSE_LIMIT_POWER;
             
             s_ctrl.response_map[EMERGENCY_THERMAL_RUNAWAY] = RESPONSE_ALERT | 
                                                             RESPONSE_COOL_SYSTEM |
                                                             RESPONSE_DISCONNECT |
                                                             RESPONSE_CONTACTOR_OPEN |
                                                             RESPONSE_SHUTDOWN;
         }
         
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         
         ESP_LOGI(TAG, "Emergency handler reset (%s)", hard_reset ? "hard" : "soft");
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Feed the emergency watchdog timer
  */
 esp_err_t emergency_handler_feed_watchdog(void) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         s_ctrl.watchdog_last_feed = xTaskGetTickCount() * portTICK_PERIOD_MS;
         xEventGroupClearBits(s_ctrl.event_group, EG_WATCHDOG_ACTIVE);
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Configure logging destinations
  */
 esp_err_t emergency_handler_configure_logging(bool console, bool sd_card, bool cloud) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         s_ctrl.config.log_to_console = console;
         s_ctrl.config.log_to_sd = sd_card;
         s_ctrl.config.log_to_cloud = cloud;
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         
         // Update log manager configuration
         log_manager_configure(console, sd_card, cloud);
         
         ESP_LOGI(TAG, "Logging configuration updated: console=%d, sd_card=%d, cloud=%d",
                 console, sd_card, cloud);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Get the number of emergency events that have occurred
  */
 esp_err_t emergency_handler_get_event_count(uint32_t *count) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (!count) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         *count = s_ctrl.event_count;
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Run a comprehensive self-test of the emergency systems
  */
 esp_err_t emergency_handler_self_test(uint8_t test_level, void *results) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xEventGroupGetBits(s_ctrl.event_group) & EG_SELF_TEST_RUNNING) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set self-test running bit
     xEventGroupSetBits(s_ctrl.event_group, EG_SELF_TEST_RUNNING);
     
     ESP_LOGI(TAG, "Starting emergency system self-test (level %d)", test_level);
     
     bool test_passed = true;
     char test_message[64] = "Self-test completed successfully";
     
     // Level 0: Basic checks of the emergency handler itself
     if (test_level >= 0) {
         // Check mutex
         if (xSemaphoreTake(s_ctrl.ctrl_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
             test_passed = false;
             strcpy(test_message, "Control mutex test failed");
             goto end_test;
         }
         xSemaphoreGive(s_ctrl.ctrl_mutex);
         
         // Check log mutex
         if (xSemaphoreTake(s_ctrl.log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
             test_passed = false;
             strcpy(test_message, "Log mutex test failed");
             goto end_test;
         }
         xSemaphoreGive(s_ctrl.log_mutex);
         
         // Check event group functionality
         xEventGroupClearBits(s_ctrl.event_group, EG_SHUTDOWN_REQUESTED);
         xEventGroupSetBits(s_ctrl.event_group, EG_SHUTDOWN_REQUESTED);
         if ((xEventGroupGetBits(s_ctrl.event_group) & EG_SHUTDOWN_REQUESTED) == 0) {
             test_passed = false;
             strcpy(test_message, "Event group test failed");
             goto end_test;
         }
         xEventGroupClearBits(s_ctrl.event_group, EG_SHUTDOWN_REQUESTED);
     }
     
     // Level 1: Test emergency triggering and callbacks
     if (test_level >= 1) {
         // Define a test callback to verify callback system
         bool callback_triggered = false;
         
         void test_callback(emergency_event_t event, void *user_data) {
             bool *triggered = (bool *)user_data;
             *triggered = true;
         }
         
         // Register test callback
         esp_err_t err = emergency_handler_register_callback(test_callback, &callback_triggered, 0);
         if (err != ESP_OK) {
             test_passed = false;
             strcpy(test_message, "Callback registration failed");
             goto end_test;
         }
         
         // Trigger a test emergency
         err = emergency_handler_trigger(
             EMERGENCY_OVER_VOLTAGE,  // Use a real type but with informational severity
             SEVERITY_INFO,           // Use lowest severity
             0xFF,                    // System-wide
             0.0f,                    // No values
             0.0f,
             "Self-test emergency trigger"
         );
         
         if (err != ESP_OK) {
             test_passed = false;
             strcpy(test_message, "Emergency trigger test failed");
             goto end_test;
         }
         
         // Allow time for callback to trigger
         vTaskDelay(pdMS_TO_TICKS(10));
         
         // Check if callback was triggered
         if (!callback_triggered) {
             test_passed = false;
             strcpy(test_message, "Callback trigger test failed");
             goto end_test;
         }
         
         // Unregister test callback
         emergency_handler_unregister_callback(test_callback);
     }
     
     // Level 2: Test actual response mechanisms (more extensive)
     if (test_level >= 2) {
         // This would test actual system responses, but is implementation-specific
         // and potentially disruptive to normal operation
         // For safety reasons, this is usually not run in production environments
         // 
         // For the implementation, we're simply logging this level but not actually
         // testing the physical responses
         ESP_LOGW(TAG, "Level 2 self-test would test physical responses (skipped for safety)");
     }
     
 end_test:
     // Clear self-test running bit
     xEventGroupClearBits(s_ctrl.event_group, EG_SELF_TEST_RUNNING);
     
     // Log test result
     if (test_passed) {
         ESP_LOGI(TAG, "Self-test passed: %s", test_message);
     } else {
         ESP_LOGE(TAG, "Self-test failed: %s", test_message);
     }
     
     // If results pointer was provided, fill it
     if (results) {
         // Cast to proper structure when defined
         // For now, we can't do anything with this
     }
     
     return test_passed ? ESP_OK : ESP_FAIL;
 }
 
 /**
  * @brief Get string representation of emergency type
  */
 const char* emergency_handler_type_to_string(emergency_type_t type) {
     // Find the highest bit set in the emergency type
     for (int i = 15; i >= 0; i--) {
         if (type & (1 << i)) {
             return emergency_type_strings[i];
         }
     }
     
     return emergency_type_strings[0]; // "None"
 }
 
 /**
  * @brief Get string representation of emergency severity
  */
 const char* emergency_handler_severity_to_string(emergency_severity_t severity) {
     if (severity <= SEVERITY_CRITICAL) {
         return emergency_severity_strings[severity];
     }
     return "Unknown";
 }
 
 /**
  * @brief Integration hook for Battery Manager module
  */
 esp_err_t emergency_handler_process_battery_event(uint32_t event_type, 
                                                  uint8_t module_id, 
                                                  void *data) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Different event types from battery manager would trigger different responses
     // This is a simplified example implementation
     
     emergency_type_t emergency_type = EMERGENCY_NONE;
     emergency_severity_t severity = SEVERITY_INFO;
     float measured_value = 0.0f;
     float threshold_value = 0.0f;
     char description[64] = {0};
     
     // Process based on event type
     // Note: Actual event types would come from battery manager definitions
     switch (event_type) {
         case 0x01: // Example: Overvoltage event
             emergency_type = EMERGENCY_OVER_VOLTAGE;
             severity = SEVERITY_HIGH;
             
             // If data contains voltage information
             if (data) {
                 float *voltage_data = (float *)data;
                 measured_value = voltage_data[0];   // Measured voltage
                 threshold_value = voltage_data[1];  // Threshold voltage
             }
             
             snprintf(description, sizeof(description), 
                     "Module %d: Battery over-voltage detected", module_id);
             break;
             
         case 0x02: // Example: Undervoltage event
             emergency_type = EMERGENCY_UNDER_VOLTAGE;
             severity = SEVERITY_HIGH;
             
             // If data contains voltage information
             if (data) {
                 float *voltage_data = (float *)data;
                 measured_value = voltage_data[0];   // Measured voltage
                 threshold_value = voltage_data[1];  // Threshold voltage
             }
             
             snprintf(description, sizeof(description), 
                     "Module %d: Battery under-voltage detected", module_id);
             break;
             
         case 0x03: // Example: Overcurrent event
             emergency_type = EMERGENCY_OVER_CURRENT;
             severity = SEVERITY_HIGH;
             
             // If data contains current information
             if (data) {
                 float *current_data = (float *)data;
                 measured_value = current_data[0];   // Measured current
                 threshold_value = current_data[1];  // Threshold current
             }
             
             snprintf(description, sizeof(description), 
                     "Module %d: Battery over-current detected", module_id);
             break;
             
         case 0x04: // Example: Cell imbalance event
             emergency_type = EMERGENCY_CELL_IMBALANCE;
             severity = SEVERITY_MEDIUM;
             
             // If data contains cell voltage delta information
             if (data) {
                 float *imbalance_data = (float *)data;
                 measured_value = imbalance_data[0];   // Measured delta
                 threshold_value = imbalance_data[1];  // Threshold delta
             }
             
             snprintf(description, sizeof(description), 
                     "Module %d: Dangerous cell imbalance detected", module_id);
             break;
             
         // Additional event types would be handled here
             
         default:
             // Unrecognized event type
             ESP_LOGW(TAG, "Unrecognized battery event type: 0x%04X", event_type);
             return ESP_ERR_INVALID_ARG;
     }
     
     // If we identified an emergency, trigger it
     if (emergency_type != EMERGENCY_NONE) {
         return emergency_handler_trigger(
             emergency_type,
             severity,
             module_id,
             measured_value,
             threshold_value,
             description
         );
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Integration hook for Thermal Monitor module
  */
 esp_err_t emergency_handler_process_thermal_event(uint8_t zone_id,
                                                 float temperature,
                                                 float threshold) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Map thermal zones to emergency types and severity
     emergency_type_t emergency_type = EMERGENCY_NONE;
     emergency_severity_t severity = SEVERITY_INFO;
     char description[64] = {0};
     
     // Note: Zone IDs would be defined in the thermal monitor component
     switch (zone_id) {
         case 3: // Example: Warning zone (elevated temperature)
             // This might not trigger an emergency, just a warning
             return ESP_OK;
             
         case 4: // Example: Critical zone
             emergency_type = EMERGENCY_OVER_TEMP;
             severity = SEVERITY_HIGH;
             snprintf(description, sizeof(description), 
                     "Critical temperature detected: %.1f°C (threshold: %.1f°C)",
                     temperature, threshold);
             break;
             
         case 5: // Example: Emergency zone
             emergency_type = EMERGENCY_OVER_TEMP;
             severity = SEVERITY_CRITICAL;
             snprintf(description, sizeof(description), 
                     "Emergency temperature detected: %.1f°C (threshold: %.1f°C)",
                     temperature, threshold);
             break;
             
         case 6: // Example: Thermal runaway detection
             emergency_type = EMERGENCY_THERMAL_RUNAWAY;
             severity = SEVERITY_CRITICAL;
             snprintf(description, sizeof(description), 
                     "Thermal runaway detected: %.1f°C/min rise rate",
                     temperature); // In this case, temperature is the rise rate
             break;
             
         default:
             // Unrecognized zone or non-emergency zone
             return ESP_OK;
     }
     
     // If we identified an emergency, trigger it
     if (emergency_type != EMERGENCY_NONE) {
         return emergency_handler_trigger(
             emergency_type,
             severity,
             0xFF, // System-wide for thermal events
             temperature,
             threshold,
             description
         );
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Integration hook for network/communications events
  */
 esp_err_t emergency_handler_process_comms_event(uint8_t comms_type,
                                               uint16_t device_id,
                                               int32_t error_code) {
     if (!s_ctrl.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Only severe communication failures should trigger emergencies
     // Most communication errors are handled at the protocol level
     
     // Critical error codes that should trigger emergency
     const int32_t CRITICAL_ERROR_THRESHOLD = -1000;
     
     if (error_code < CRITICAL_ERROR_THRESHOLD) {
         char description[64];
         snprintf(description, sizeof(description), 
                 "Critical communication failure: Type %d, Device 0x%04X, Error %d",
                 comms_type, device_id, error_code);
         
         return emergency_handler_trigger(
             EMERGENCY_COMMUNICATION,
             SEVERITY_HIGH, // Communication failures are typically high severity
             0xFF, // System-wide
             (float)error_code,
             (float)CRITICAL_ERROR_THRESHOLD,
             description
         );
     }
     
     return ESP_OK;
 }