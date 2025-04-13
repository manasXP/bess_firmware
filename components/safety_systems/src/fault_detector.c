/**
 * @file fault_detector.c
 * @brief Implementation of fault detection system for BESS 100KW/200KWH energy storage system
 * @version 1.0
 * @date 2025-04-13
 */

 #include "fault_detector.h"
 #include <string.h>
 #include <stdio.h>
 #include "esp_log.h"
 #include "esp_system.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "esp_timer.h"
 #include "esp_heap_caps.h"
 #include "esp_task_wdt.h"
 #include "esp_sleep.h"
 #include "esp_timer.h"
 #include "driver/gpio.h"
 #include "sdkconfig.h"
 
 /* Include logging module for various outputs */
 #include "logging.h" // Custom logging module supporting console, SD, and CloudWatch
 
 /* Internal definitions */
 #define FAULT_DETECTOR_TAG                  "FAULT_DETECTOR"
 #define FAULT_DETECTOR_TASK_STACK_SIZE      4096
 #define FAULT_DETECTOR_TASK_PRIORITY        10
 #define FAULT_DETECTOR_WATCHDOG_TIMEOUT     5000    // 5 seconds
 #define FAULT_DETECTOR_MAX_CALLBACKS        10
 #define FAULT_DETECTOR_MAX_THRESHOLDS       32
 #define FAULT_DETECTOR_QUEUE_SIZE           20
 #define FAULT_DETECTOR_EVENT_BIT_STARTED    (1 << 0)
 #define FAULT_DETECTOR_EVENT_BIT_STOPPED    (1 << 1)
 #define FAULT_DETECTOR_EVENT_BIT_FAULT      (1 << 2)
 #define FAULT_DETECTOR_EVENT_BIT_RESET      (1 << 3)
 
 /* Internal types */
 typedef struct {
     fault_type_t fault_type;
     float threshold;
     float hysteresis;
     bool enabled;
     fault_severity_t severity;
     fault_action_t action;
     uint32_t last_logged;  // Timestamp of last log for this fault
     uint8_t consecutive_count; // Count of consecutive readings above threshold
 } fault_threshold_t;
 
 typedef struct {
     fault_callback_t callback;
     uint32_t fault_mask;
     void *user_data;
     bool active;
 } fault_callback_entry_t;
 
 /* Static variables */
 static fault_detector_config_t s_config;
 static fault_status_t s_status;
 static fault_threshold_t s_thresholds[FAULT_DETECTOR_MAX_THRESHOLDS];
 static fault_callback_entry_t s_callbacks[FAULT_DETECTOR_MAX_CALLBACKS];
 static TaskHandle_t s_fault_task_handle = NULL;
 static SemaphoreHandle_t s_fault_mutex = NULL;
 static QueueHandle_t s_fault_queue = NULL;
 static EventGroupHandle_t s_fault_event_group = NULL;
 static bool s_initialized = false;
 static bool s_running = false;
 static uint32_t s_uptime_seconds = 0;
 
 /* Forward declarations for internal functions */
 static void fault_detector_task(void *pvParameters);
 static esp_err_t execute_fault_action(fault_action_t action, const fault_event_t *event);
 static void notify_fault_callbacks(const fault_event_t *event);
 static bool check_threshold_exceeded(fault_type_t fault_type, float value);
 static const char* get_fault_type_string(fault_type_t type);
 static const char* get_severity_string(fault_severity_t severity);
 static const char* get_action_string(fault_action_t action);
 static void fault_detector_timer_callback(void* arg);
 static esp_err_t fault_detector_perform_self_test(void);
 static esp_err_t fault_detector_init_default_thresholds(void);
 static esp_err_t process_fault_event(fault_event_t *event);
 static void uptime_counter_timer_callback(void* arg);
 
 /* Timer handle for uptime counter */
 static esp_timer_handle_t s_uptime_timer = NULL;
 
 /**
  * @brief Initialize the fault detector
  */
 esp_err_t fault_detector_init(const fault_detector_config_t *config) {
     if (s_initialized) {
         LOG_WARN(FAULT_DETECTOR_TAG, "Fault detector already initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (config == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL configuration provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     /* Create synchronization primitives */
     s_fault_mutex = xSemaphoreCreateMutex();
     if (s_fault_mutex == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to create fault mutex");
         return ESP_ERR_NO_MEM;
     }
 
     s_fault_queue = xQueueCreate(FAULT_DETECTOR_QUEUE_SIZE, sizeof(fault_event_t));
     if (s_fault_queue == NULL) {
         vSemaphoreDelete(s_fault_mutex);
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to create fault queue");
         return ESP_ERR_NO_MEM;
     }
 
     s_fault_event_group = xEventGroupCreate();
     if (s_fault_event_group == NULL) {
         vQueueDelete(s_fault_queue);
         vSemaphoreDelete(s_fault_mutex);
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to create fault event group");
         return ESP_ERR_NO_MEM;
     }
 
     /* Copy configuration */
     memcpy(&s_config, config, sizeof(fault_detector_config_t));
 
     /* Initialize status */
     memset(&s_status, 0, sizeof(fault_status_t));
     s_status.last_fault_type = FAULT_NONE;
 
     /* Initialize callback array */
     memset(s_callbacks, 0, sizeof(s_callbacks));
 
     /* Initialize thresholds with default values */
     memset(s_thresholds, 0, sizeof(s_thresholds));
     fault_detector_init_default_thresholds();
 
     /* Set up uptime counter timer */
     const esp_timer_create_args_t uptime_timer_args = {
         .callback = &uptime_counter_timer_callback,
         .name = "uptime_counter"
     };
     
     esp_err_t ret = esp_timer_create(&uptime_timer_args, &s_uptime_timer);
     if (ret != ESP_OK) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to create uptime timer: %s", esp_err_to_name(ret));
         vEventGroupDelete(s_fault_event_group);
         vQueueDelete(s_fault_queue);
         vSemaphoreDelete(s_fault_mutex);
         return ret;
     }
     
     // Start the timer with a period of 1 second
     ret = esp_timer_start_periodic(s_uptime_timer, 1000000); // 1 second in microseconds
     if (ret != ESP_OK) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to start uptime timer: %s", esp_err_to_name(ret));
         esp_timer_delete(s_uptime_timer);
         vEventGroupDelete(s_fault_event_group);
         vQueueDelete(s_fault_queue);
         vSemaphoreDelete(s_fault_mutex);
         return ret;
     }
 
     s_initialized = true;
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector initialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Perform self-test on the fault detection system
  */
 static esp_err_t fault_detector_perform_self_test(void) {
     LOG_INFO(FAULT_DETECTOR_TAG, "Running fault detector self-test");
     
     /* Check memory allocation */
     if (s_fault_mutex == NULL || s_fault_queue == NULL || s_fault_event_group == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Memory allocation check failed");
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Verify mutex functionality */
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Mutex functionality check failed");
         return ESP_ERR_TIMEOUT;
     }
     xSemaphoreGive(s_fault_mutex);
     
     /* Test queue functionality */
     fault_event_t test_event = {
         .type = FAULT_NONE,
         .severity = FAULT_SEVERITY_INFO,
         .action = FAULT_ACTION_NONE,
         .timestamp = s_uptime_seconds,
         .module_id = 0xFF,
         .cell_id = 0xFF,
         .measured_value = 0,
         .threshold_value = 0,
         .description = "Self-test event"
     };
     
     if (xQueueSend(s_fault_queue, &test_event, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Queue functionality check failed");
         return ESP_ERR_TIMEOUT;
     }
     
     if (xQueueReceive(s_fault_queue, &test_event, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Queue receive check failed");
         return ESP_ERR_TIMEOUT;
     }
     
     /* Test event group functionality */
     xEventGroupClearBits(s_fault_event_group, 0xFF);
     xEventGroupSetBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_FAULT);
     EventBits_t bits = xEventGroupGetBits(s_fault_event_group);
     if ((bits & FAULT_DETECTOR_EVENT_BIT_FAULT) == 0) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Event group functionality check failed");
         return ESP_ERR_INVALID_STATE;
     }
     xEventGroupClearBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_FAULT);
     
     /* Verify threshold configuration */
     bool found_enabled_threshold = false;
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].enabled) {
             found_enabled_threshold = true;
             break;
         }
     }
     
     if (!found_enabled_threshold) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "No enabled thresholds found");
         return ESP_ERR_INVALID_STATE;
     }
     
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector self-test passed");
     return ESP_OK;
 }
 
 /**
  * @brief Fault detector timer callback
  * This is used for periodic testing and monitoring
  */
 static void fault_detector_timer_callback(void* arg) {
     /* Check if task is still running */
     if (s_fault_task_handle != NULL && !s_running) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector task not running but handle exists");
         
         /* Report fault */
         fault_event_t watchdog_event = {
             .type = FAULT_WATCHDOG_TIMEOUT,
             .severity = FAULT_SEVERITY_ERROR,
             .action = FAULT_ACTION_ALERT,
             .timestamp = s_uptime_seconds,
             .module_id = 0xFF,
             .cell_id = 0xFF,
             .measured_value = 0,
             .threshold_value = 0,
             .description = "Fault detector task watchdog timeout"
         };
         
         if (xQueueSend(s_fault_queue, &watchdog_event, 0) != pdTRUE) {
             LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to send watchdog fault event to queue");
         }
     }
 }
 }
 
 /**
  * @brief Start the fault detection task
  */
 esp_err_t fault_detector_start(void) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_running) {
         LOG_WARN(FAULT_DETECTOR_TAG, "Fault detector already running");
         return ESP_OK;
     }
 
     /* Clear fault event flags */
     xEventGroupClearBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_STOPPED);
     
     /* Create and start fault detector task */
     BaseType_t ret = xTaskCreatePinnedToCore(
         fault_detector_task,
         "fault_detector",
         FAULT_DETECTOR_TASK_STACK_SIZE,
         NULL,
         FAULT_DETECTOR_TASK_PRIORITY,
         &s_fault_task_handle,
         1  // Run on core 1
     );
 
     if (ret != pdPASS) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to create fault detector task");
         return ESP_ERR_NO_MEM;
     }
 
     /* Wait for task to start */
     EventBits_t bits = xEventGroupWaitBits(
         s_fault_event_group,
         FAULT_DETECTOR_EVENT_BIT_STARTED,
         pdTRUE,       // Clear on exit
         pdFALSE,      // Don't wait for all bits
         pdMS_TO_TICKS(1000)
     );
 
     if ((bits & FAULT_DETECTOR_EVENT_BIT_STARTED) == 0) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Timeout waiting for fault detector task to start");
         vTaskDelete(s_fault_task_handle);
         s_fault_task_handle = NULL;
         return ESP_ERR_TIMEOUT;
     }
 
     s_running = true;
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector started successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Stop the fault detection task
  */
 esp_err_t fault_detector_stop(void) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!s_running) {
         LOG_WARN(FAULT_DETECTOR_TAG, "Fault detector already stopped");
         return ESP_OK;
     }
 
     /* Clear fault event flags */
     xEventGroupClearBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_STARTED);
     
     /* Signal task to stop */
     xEventGroupSetBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_STOPPED);
     
     /* Wait for task to stop */
     if (s_fault_task_handle != NULL) {
         EventBits_t bits = xEventGroupWaitBits(
             s_fault_event_group,
             FAULT_DETECTOR_EVENT_BIT_STOPPED,
             pdTRUE,       // Clear on exit
             pdFALSE,      // Don't wait for all bits
             pdMS_TO_TICKS(1000)
         );
 
         if ((bits & FAULT_DETECTOR_EVENT_BIT_STOPPED) == 0) {
             LOG_ERROR(FAULT_DETECTOR_TAG, "Timeout waiting for fault detector task to stop");
             vTaskDelete(s_fault_task_handle);
         }
         
         s_fault_task_handle = NULL;
     }
 
     s_running = false;
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector stopped successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Reset (clear) all active and latched faults
  */
 esp_err_t fault_detector_reset_all(void) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     s_status.active_faults = 0;
     s_status.latched_faults = 0;
     
     /* Reset consecutive counts for all thresholds */
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         s_thresholds[i].consecutive_count = 0;
     }
 
     xSemaphoreGive(s_fault_mutex);
 
     /* Signal reset event */
     xEventGroupSetBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_RESET);
 
     LOG_INFO(FAULT_DETECTOR_TAG, "All faults reset successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Reset (clear) a specific fault
  */
 esp_err_t fault_detector_reset_fault(fault_type_t fault_type) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (fault_type == FAULT_NONE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid fault type");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Clear active and latched bits for this fault */
     s_status.active_faults &= ~fault_type;
     s_status.latched_faults &= ~fault_type;
     
     /* Reset consecutive count for this fault */
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].fault_type == fault_type) {
             s_thresholds[i].consecutive_count = 0;
             break;
         }
     }
 
     xSemaphoreGive(s_fault_mutex);
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault %s reset successfully", get_fault_type_string(fault_type));
     return ESP_OK;
 }
 
 /**
  * @brief Manually report a fault
  */
 esp_err_t fault_detector_report_fault(const fault_event_t *event) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (event == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL event provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (event->type == FAULT_NONE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid fault type");
         return ESP_ERR_INVALID_ARG;
     }
 
     /* Create a copy of the event with current timestamp if not provided */
     fault_event_t local_event = *event;
     if (local_event.timestamp == 0) {
         local_event.timestamp = s_uptime_seconds;
     }
 
     /* Send event to queue for processing */
     if (xQueueSend(s_fault_queue, &local_event, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to send fault event to queue");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Signal fault event */
     xEventGroupSetBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_FAULT);
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault %s reported successfully", get_fault_type_string(event->type));
     return ESP_OK;
 }
 
 /**
  * @brief Set fault threshold values
  */
 esp_err_t fault_detector_set_threshold(fault_type_t fault_type, float threshold, float hysteresis) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (fault_type == FAULT_NONE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid fault type");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (hysteresis >= threshold) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Hysteresis must be less than threshold");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Find existing threshold or empty slot */
     int index = -1;
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].fault_type == fault_type) {
             index = i;
             break;
         } else if (s_thresholds[i].fault_type == FAULT_NONE && index == -1) {
             index = i;  // Remember first empty slot
         }
     }
 
     if (index == -1) {
         xSemaphoreGive(s_fault_mutex);
         LOG_ERROR(FAULT_DETECTOR_TAG, "No space for new fault threshold");
         return ESP_ERR_NO_MEM;
     }
 
     /* Update threshold */
     s_thresholds[index].fault_type = fault_type;
     s_thresholds[index].threshold = threshold;
     s_thresholds[index].hysteresis = hysteresis;
     if (!s_thresholds[index].enabled) {
         s_thresholds[index].enabled = true;
     }
 
     xSemaphoreGive(s_fault_mutex);
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Threshold for %s set to %.2f (hysteresis %.2f)", 
              get_fault_type_string(fault_type), threshold, hysteresis);
     return ESP_OK;
 }
 
 /**
  * @brief Configure fault response action
  */
 esp_err_t fault_detector_set_response(fault_type_t fault_type, fault_severity_t severity, fault_action_t action) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (fault_type == FAULT_NONE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid fault type");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (severity < FAULT_SEVERITY_INFO || severity > FAULT_SEVERITY_EMERGENCY) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid severity level");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (action < FAULT_ACTION_NONE || action > FAULT_ACTION_SHUTDOWN) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid action");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Find existing threshold or empty slot */
     int index = -1;
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].fault_type == fault_type) {
             index = i;
             break;
         } else if (s_thresholds[i].fault_type == FAULT_NONE && index == -1) {
             index = i;  // Remember first empty slot
         }
     }
 
     if (index == -1) {
         xSemaphoreGive(s_fault_mutex);
         LOG_ERROR(FAULT_DETECTOR_TAG, "No space for new fault response");
         return ESP_ERR_NO_MEM;
     }
 
     /* Update response */
     s_thresholds[index].fault_type = fault_type;
     s_thresholds[index].severity = severity;
     s_thresholds[index].action = action;
     if (!s_thresholds[index].enabled) {
         s_thresholds[index].enabled = true;
     }
 
     xSemaphoreGive(s_fault_mutex);
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Response for %s set to severity %s, action %s", 
              get_fault_type_string(fault_type), 
              get_severity_string(severity),
              get_action_string(action));
     return ESP_OK;
 }
 
 /**
  * @brief Get current fault status information
  */
 esp_err_t fault_detector_get_status(fault_status_t *status) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (status == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL status pointer provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Copy status */
     memcpy(status, &s_status, sizeof(fault_status_t));
 
     xSemaphoreGive(s_fault_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Check if a specific fault is active
  */
 esp_err_t fault_detector_is_fault_active(fault_type_t fault_type, bool *active) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (active == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL active pointer provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (fault_type == FAULT_NONE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid fault type");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Check if fault is active */
     *active = (s_status.active_faults & fault_type) != 0;
 
     xSemaphoreGive(s_fault_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Check if any fault of a specific severity level or higher is active
  */
 esp_err_t fault_detector_is_severity_active(fault_severity_t min_severity, bool *active) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (active == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL active pointer provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (min_severity < FAULT_SEVERITY_INFO || min_severity > FAULT_SEVERITY_EMERGENCY) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid severity level");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Check for active faults at or above the specified severity */
     *active = false;
     if (s_status.active_faults != 0) {
         for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
             if (s_thresholds[i].enabled && 
                 (s_status.active_faults & s_thresholds[i].fault_type) && 
                 s_thresholds[i].severity >= min_severity) {
                 *active = true;
                 break;
             }
         }
     }
 
     xSemaphoreGive(s_fault_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Register callback for fault events
  */
 esp_err_t fault_detector_register_callback(fault_callback_t callback, uint32_t fault_mask, void *user_data) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (callback == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL callback provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Find empty slot */
     int index = -1;
     for (int i = 0; i < FAULT_DETECTOR_MAX_CALLBACKS; i++) {
         if (!s_callbacks[i].active) {
             index = i;
             break;
         }
     }
 
     if (index == -1) {
         xSemaphoreGive(s_fault_mutex);
         LOG_ERROR(FAULT_DETECTOR_TAG, "No space for new callback");
         return ESP_ERR_NO_MEM;
     }
 
     /* Register callback */
     s_callbacks[index].callback = callback;
     s_callbacks[index].fault_mask = fault_mask;
     s_callbacks[index].user_data = user_data;
     s_callbacks[index].active = true;
 
     xSemaphoreGive(s_fault_mutex);
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Callback registered successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a previously registered callback
  */
 esp_err_t fault_detector_unregister_callback(fault_callback_t callback) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (callback == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL callback provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     /* Find and remove callback */
     bool found = false;
     for (int i = 0; i < FAULT_DETECTOR_MAX_CALLBACKS; i++) {
         if (s_callbacks[i].active && s_callbacks[i].callback == callback) {
             s_callbacks[i].active = false;
             found = true;
             break;
         }
     }
 
     xSemaphoreGive(s_fault_mutex);
 
     if (!found) {
         LOG_WARN(FAULT_DETECTOR_TAG, "Callback not found for unregistration");
         return ESP_ERR_NOT_FOUND;
     }
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Callback unregistered successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Get a text description for a fault type
  */
 const char* fault_detector_get_description(fault_type_t fault_type) {
     return get_fault_type_string(fault_type);
 }
 
 /**
  * @brief Run self-test on the fault detection system
  */
 esp_err_t fault_detector_self_test(void) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Running fault detector self-test...");
     
     esp_err_t result = fault_detector_perform_self_test();
     
     if (result == ESP_OK) {
         LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector self-test passed");
     } else {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector self-test failed: %s", esp_err_to_name(result));
     }
     
     return result;
 }
 
 /**
  * @brief Configure fault thresholds from a configuration file
  */
 esp_err_t fault_detector_load_config(const char *config_file) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (config_file == NULL) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "NULL config file path provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     // This function would implement reading fault thresholds from a configuration file
     // For now, we'll provide a stub implementation that returns an error
     LOG_ERROR(FAULT_DETECTOR_TAG, "Loading configuration from file not implemented yet");
     return ESP_ERR_NOT_SUPPORTED;
 }
 
 /* Internal function implementations */
 
 /**
  * @brief Fault detector task implementation
  */
 static void fault_detector_task(void *pvParameters) {
     /* Register with watchdog if enabled */
     if (s_config.enable_watchdog) {
         esp_task_wdt_add(NULL);
     }
 
     /* Signal that task has started */
     xEventGroupSetBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_STARTED);
     
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector task started");
 
     fault_event_t event;
     TickType_t last_reset_time = xTaskGetTickCount();
     bool self_test_passed = false;
     
     /* Run initial self-test */
     if (fault_detector_perform_self_test() == ESP_OK) {
         self_test_passed = true;
         LOG_INFO(FAULT_DETECTOR_TAG, "Initial self-test passed");
     } else {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Initial self-test failed");
         
         /* Report self-test failure fault */
         fault_event_t self_test_event = {
             .type = FAULT_MEMORY_CORRUPTION,
             .severity = FAULT_SEVERITY_ERROR,
             .action = FAULT_ACTION_ALERT,
             .timestamp = s_uptime_seconds,
             .module_id = 0xFF,
             .cell_id = 0xFF,
             .measured_value = 0,
             .threshold_value = 0,
             .description = "Fault detector self-test failed"
         };
         
         process_fault_event(&self_test_event);
     }
 
     /* Main task loop */
     while (1) {
         /* Check for stop request */
         EventBits_t bits = xEventGroupGetBits(s_fault_event_group);
         if (bits & FAULT_DETECTOR_EVENT_BIT_STOPPED) {
             break;
         }
         
         /* Reset watchdog if enabled */
         if (s_config.enable_watchdog) {
             esp_task_wdt_reset();
         }
         
         /* Process fault events from queue */
         while (xQueueReceive(s_fault_queue, &event, 0) == pdTRUE) {
             process_fault_event(&event);
         }
         
         /* Check if enough time has passed for auto-reset */
         if (s_config.auto_reset && 
             (xTaskGetTickCount() - last_reset_time) >= pdMS_TO_TICKS(s_config.fault_persistence * 1000)) {
             
             /* Auto-reset faults that are no longer active */
             if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                 uint32_t faults_to_clear = 0;
                 
                 for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
                     if (s_thresholds[i].enabled && 
                         (s_status.active_faults & s_thresholds[i].fault_type) &&
                         s_thresholds[i].consecutive_count == 0) {
                         
                         /* This fault is no longer active, clear it */
                         faults_to_clear |= s_thresholds[i].fault_type;
                     }
                 }
                 
                 if (faults_to_clear != 0) {
                     s_status.active_faults &= ~faults_to_clear;
                     LOG_INFO(FAULT_DETECTOR_TAG, "Auto-reset cleared faults: 0x%08X", faults_to_clear);
                 }
                 
                 xSemaphoreGive(s_fault_mutex);
             }
             
             last_reset_time = xTaskGetTickCount();
         }
         
         /* Delay for monitoring interval */
         vTaskDelay(pdMS_TO_TICKS(s_config.monitoring_interval));
     }
     
     /* Unregister from watchdog if enabled */
     if (s_config.enable_watchdog) {
         esp_task_wdt_delete(NULL);
     }
     
     /* Signal that task has stopped */
     xEventGroupSetBits(s_fault_event_group, FAULT_DETECTOR_EVENT_BIT_STOPPED);
     
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault detector task stopped");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Process a fault event
  */
 static esp_err_t process_fault_event(fault_event_t *event) {
     if (event == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     /* Find the threshold configuration for this fault type */
     fault_threshold_t *threshold = NULL;
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].fault_type == event->type) {
             threshold = &s_thresholds[i];
             break;
         }
     }
     
     /* If threshold not found or disabled, use event defaults */
     fault_severity_t severity = event->severity;
     fault_action_t action = event->action;
     
     if (threshold != NULL && threshold->enabled) {
         severity = threshold->severity;
         action = threshold->action;
     }
     
     /* Update status */
     s_status.active_faults |= event->type;
     s_status.latched_faults |= event->type;
     s_status.fault_counts[__builtin_ctz(event->type)]++;
     s_status.last_fault_time = event->timestamp;
     s_status.last_fault_type = event->type;
     s_status.total_faults++;
     
     /* Check logging interval */
     bool should_log = true;
     if (threshold != NULL) {
         if ((event->timestamp - threshold->last_logged) < s_config.log_interval) {
             should_log = false;
         } else {
             threshold->last_logged = event->timestamp;
         }
     }
     
     xSemaphoreGive(s_fault_mutex);
     
     /* Execute the appropriate action */
     execute_fault_action(action, event);
     
     /* Log the fault if needed */
     if (should_log) {
         char log_message[256];
         snprintf(log_message, sizeof(log_message),
                 "FAULT: %s, Severity: %s, Action: %s, Module: %d, Cell: %d, Value: %.2f, Threshold: %.2f, %s",
                 get_fault_type_string(event->type),
                 get_severity_string(severity),
                 get_action_string(action),
                 event->module_id != 0xFF ? event->module_id : -1,
                 event->cell_id != 0xFF ? event->cell_id : -1,
                 event->measured_value,
                 event->threshold_value,
                 event->description);
         
         switch (severity) {
             case FAULT_SEVERITY_INFO:
                 LOG_INFO(FAULT_DETECTOR_TAG, "%s", log_message);
                 break;
             case FAULT_SEVERITY_WARNING:
                 LOG_WARN(FAULT_DETECTOR_TAG, "%s", log_message);
                 break;
             case FAULT_SEVERITY_ERROR:
                 LOG_ERROR(FAULT_DETECTOR_TAG, "%s", log_message);
                 break;
             case FAULT_SEVERITY_CRITICAL:
             case FAULT_SEVERITY_EMERGENCY:
                 LOG_CRITICAL(FAULT_DETECTOR_TAG, "%s", log_message);
                 break;
             default:
                 LOG_ERROR(FAULT_DETECTOR_TAG, "%s", log_message);
                 break;
         }
         
         /* If cloud reporting is enabled, log to CloudWatch */
         if (s_config.cloud_reporting) {
             // This would call the CloudWatch logging function from the logging module
             // LOG_CLOUD(severity, FAULT_DETECTOR_TAG, log_message);
         }
     }
     
     /* Notify registered callbacks */
     notify_fault_callbacks(event);
     
     return ESP_OK;
 }
 
 /**
  * @brief Execute the appropriate action for a fault
  */
 static esp_err_t execute_fault_action(fault_action_t action, const fault_event_t *event) {
     switch (action) {
         case FAULT_ACTION_NONE:
         case FAULT_ACTION_LOG_ONLY:
             /* No action needed beyond logging */
             break;
             
         case FAULT_ACTION_ALERT:
             /* Generate alert but continue operation */
             // This would trigger an alert through the system's alert mechanism
             // alert_system_trigger(event->type, event->severity);
             break;
             
         case FAULT_ACTION_LIMIT_CHARGE:
             /* Limit charging capability */
             // This would call into the battery manager to limit charging
             // battery_manager_limit_charging(true, event->module_id);
             break;
             
         case FAULT_ACTION_LIMIT_DISCHARGE:
             /* Limit discharging capability */
             // This would call into the battery manager to limit discharging
             // battery_manager_limit_discharging(true, event->module_id);
             break;
             
         case FAULT_ACTION_PAUSE_OPERATION:
             /* Temporarily pause system operation */
             // This would call into the main controller to pause operations
             // system_controller_pause_operation(event->type);
             break;
             
         case FAULT_ACTION_OPEN_CONTACTORS:
             /* Open contactors to isolate battery */
             // This would call into the contactor controller to open contactors
             // contactor_controller_open_all();
             break;
             
         case FAULT_ACTION_SHUTDOWN:
             /* Complete system shutdown */
             // This would call into the system controller to initiate shutdown
             // system_controller_emergency_shutdown(event->type);
             break;
             
         default:
             LOG_ERROR(FAULT_DETECTOR_TAG, "Unknown fault action: %d", action);
             return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Notify all registered callbacks about a fault event
  */
 static void notify_fault_callbacks(const fault_event_t *event) {
     if (event == NULL) {
         return;
     }
     
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex for callbacks");
         return;
     }
     
     /* Notify all registered callbacks */
     for (int i = 0; i < FAULT_DETECTOR_MAX_CALLBACKS; i++) {
         if (s_callbacks[i].active && 
             (s_callbacks[i].fault_mask == 0 || (s_callbacks[i].fault_mask & event->type))) {
             
             /* Release mutex during callback to prevent deadlocks */
             xSemaphoreGive(s_fault_mutex);
             
             /* Call the callback */
             s_callbacks[i].callback((fault_event_t *)event, s_callbacks[i].user_data);
             
             /* Re-acquire mutex */
             if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
                 LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to re-take fault mutex after callback");
                 return;
             }
         }
     }
     
     xSemaphoreGive(s_fault_mutex);
 }
 
 /**
  * @brief Check if value exceeds the threshold for a fault type
  */
 static bool check_threshold_exceeded(fault_type_t fault_type, float value) {
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex for threshold check");
         return false;
     }
     
     bool exceeded = false;
     
     /* Find the threshold for this fault type */
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].fault_type == fault_type && s_thresholds[i].enabled) {
             /* Check if value exceeds threshold */
             exceeded = (value >= s_thresholds[i].threshold);
             
             /* Update consecutive count */
             if (exceeded) {
                 s_thresholds[i].consecutive_count++;
                 
                 /* Check if consecutive count meets requirement */
                 exceeded = (s_thresholds[i].consecutive_count >= s_config.consecutive_readings);
             } else {
                 s_thresholds[i].consecutive_count = 0;
             }
             
             break;
         }
     }
     
     xSemaphoreGive(s_fault_mutex);
     return exceeded;
 }
 
 /**
  * @brief Get string representation of fault type
  */
 static const char* get_fault_type_string(fault_type_t type) {
     switch (type) {
         case FAULT_NONE:                     return "No Fault";
         case FAULT_CELL_UNDERVOLTAGE:        return "Cell Undervoltage";
         case FAULT_CELL_OVERVOLTAGE:         return "Cell Overvoltage";
         case FAULT_MODULE_UNDERVOLTAGE:      return "Module Undervoltage";
         case FAULT_MODULE_OVERVOLTAGE:       return "Module Overvoltage";
         case FAULT_SYSTEM_UNDERVOLTAGE:      return "System Undervoltage";
         case FAULT_SYSTEM_OVERVOLTAGE:       return "System Overvoltage";
         case FAULT_VOLTAGE_IMBALANCE:        return "Voltage Imbalance";
         case FAULT_OVERCURRENT_CHARGE:       return "Charge Overcurrent";
         case FAULT_OVERCURRENT_DISCHARGE:    return "Discharge Overcurrent";
         case FAULT_SHORTCIRCUIT:             return "Short Circuit";
         case FAULT_CURRENT_IMBALANCE:        return "Current Imbalance";
         case FAULT_CELL_UNDERTEMP:           return "Cell Undertemperature";
         case FAULT_CELL_OVERTEMP:            return "Cell Overtemperature";
         case FAULT_MODULE_UNDERTEMP:         return "Module Undertemperature";
         case FAULT_MODULE_OVERTEMP:          return "Module Overtemperature";
         case FAULT_THERMAL_RUNAWAY:          return "Thermal Runaway";
         case FAULT_TEMP_SENSOR_FAILURE:      return "Temperature Sensor Failure";
         case FAULT_INSULATION_FAILURE:       return "Insulation Failure";
         case FAULT_CONTACTOR_FAILURE:        return "Contactor Failure";
         case FAULT_PRECHARGE_FAILURE:        return "Precharge Failure";
         case FAULT_BMS_COMMUNICATION:        return "BMS Communication Error";
         case FAULT_EXTERNAL_COMMUNICATION:   return "External Communication Error";
         case FAULT_WATCHDOG_TIMEOUT:         return "Watchdog Timeout";
         case FAULT_MEMORY_CORRUPTION:        return "Memory Corruption";
         case FAULT_CAN_BUS_ERROR:            return "CAN Bus Error";
         case FAULT_MODBUS_ERROR:             return "Modbus Error";
         case FAULT_VOLTAGE_SENSOR:           return "Voltage Sensor Failure";
         case FAULT_CURRENT_SENSOR:           return "Current Sensor Failure";
         case FAULT_ENCLOSURE_BREACH:         return "Enclosure Breach";
         case FAULT_WATER_LEAK:               return "Water Leak";
         case FAULT_SMOKE_DETECTED:           return "Smoke Detected";
         case FAULT_SYSTEM_CRITICAL:          return "System Critical Fault";
         default:                             return "Unknown Fault";
     }
 }
 
 /**
  * @brief Get string representation of fault severity
  */
 static const char* get_severity_string(fault_severity_t severity) {
     switch (severity) {
         case FAULT_SEVERITY_INFO:        return "Info";
         case FAULT_SEVERITY_WARNING:     return "Warning";
         case FAULT_SEVERITY_ERROR:       return "Error";
         case FAULT_SEVERITY_CRITICAL:    return "Critical";
         case FAULT_SEVERITY_EMERGENCY:   return "Emergency";
         default:                         return "Unknown";
     }
 }
 
 /**
  * @brief Get string representation of fault action
  */
 static const char* get_action_string(fault_action_t action) {
     switch (action) {
         case FAULT_ACTION_NONE:              return "None";
         case FAULT_ACTION_LOG_ONLY:          return "Log Only";
         case FAULT_ACTION_ALERT:             return "Alert";
         case FAULT_ACTION_LIMIT_CHARGE:      return "Limit Charging";
         case FAULT_ACTION_LIMIT_DISCHARGE:   return "Limit Discharging";
         case FAULT_ACTION_PAUSE_OPERATION:   return "Pause Operation";
         case FAULT_ACTION_OPEN_CONTACTORS:   return "Open Contactors";
         case FAULT_ACTION_SHUTDOWN:          return "Shutdown";
         default:                             return "Unknown";
     }
 }
 
 /**
  * @brief Callback for the uptime timer
  */
 static void uptime_counter_timer_callback(void* arg) {
     s_uptime_seconds++;
 }
 
 /**
  * @brief Initialize default fault thresholds
  */
 static esp_err_t fault_detector_init_default_thresholds(void) {
     /* Cell voltage thresholds */
     s_thresholds[0].fault_type = FAULT_CELL_UNDERVOLTAGE;
     s_thresholds[0].threshold = 2.5f;  // 2.5V
     s_thresholds[0].hysteresis = 0.1f; // 0.1V
     s_thresholds[0].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[0].action = FAULT_ACTION_LIMIT_DISCHARGE;
     s_thresholds[0].enabled = true;
     
     s_thresholds[1].fault_type = FAULT_CELL_OVERVOLTAGE;
     s_thresholds[1].threshold = 3.65f; // 3.65V
     s_thresholds[1].hysteresis = 0.1f; // 0.1V
     s_thresholds[1].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[1].action = FAULT_ACTION_LIMIT_CHARGE;
     s_thresholds[1].enabled = true;
     
     /* Module voltage thresholds */
     s_thresholds[2].fault_type = FAULT_MODULE_UNDERVOLTAGE;
     s_thresholds[2].threshold = 40.0f; // 40V for 48V module
     s_thresholds[2].hysteresis = 2.0f; // 2V
     s_thresholds[2].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[2].action = FAULT_ACTION_LIMIT_DISCHARGE;
     s_thresholds[2].enabled = true;
     
     s_thresholds[3].fault_type = FAULT_MODULE_OVERVOLTAGE;
     s_thresholds[3].threshold = 58.0f; // 58V for 48V module
     s_thresholds[3].hysteresis = 2.0f; // 2V
     s_thresholds[3].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[3].action = FAULT_ACTION_LIMIT_CHARGE;
     s_thresholds[3].enabled = true;
     
     /* System voltage thresholds */
     s_thresholds[4].fault_type = FAULT_SYSTEM_UNDERVOLTAGE;
     s_thresholds[4].threshold = 480.0f; // 480V for 10 modules
     s_thresholds[4].hysteresis = 20.0f; // 20V
     s_thresholds[4].severity = FAULT_SEVERITY_CRITICAL;
     s_thresholds[4].action = FAULT_ACTION_OPEN_CONTACTORS;
     s_thresholds[4].enabled = true;
     
     s_thresholds[5].fault_type = FAULT_SYSTEM_OVERVOLTAGE;
     s_thresholds[5].threshold = 580.0f; // 580V for 10 modules
     s_thresholds[5].hysteresis = 20.0f; // 20V
     s_thresholds[5].severity = FAULT_SEVERITY_CRITICAL;
     s_thresholds[5].action = FAULT_ACTION_OPEN_CONTACTORS;
     s_thresholds[5].enabled = true;
     
     /* Current thresholds */
     s_thresholds[6].fault_type = FAULT_OVERCURRENT_CHARGE;
     s_thresholds[6].threshold = 120.0f; // 120A (slightly over 100kW at 500V)
     s_thresholds[6].hysteresis = 10.0f; // 10A
     s_thresholds[6].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[6].action = FAULT_ACTION_LIMIT_CHARGE;
     s_thresholds[6].enabled = true;
     
     s_thresholds[7].fault_type = FAULT_OVERCURRENT_DISCHARGE;
     s_thresholds[7].threshold = 120.0f; // 120A
     s_thresholds[7].hysteresis = 10.0f; // 10A
     s_thresholds[7].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[7].action = FAULT_ACTION_LIMIT_DISCHARGE;
     s_thresholds[7].enabled = true;
     
     s_thresholds[8].fault_type = FAULT_SHORTCIRCUIT;
     s_thresholds[8].threshold = 300.0f; // 300A
     s_thresholds[8].hysteresis = 50.0f; // 50A
     s_thresholds[8].severity = FAULT_SEVERITY_EMERGENCY;
     s_thresholds[8].action = FAULT_ACTION_SHUTDOWN;
     s_thresholds[8].enabled = true;
     
     /* Temperature thresholds */
     s_thresholds[9].fault_type = FAULT_CELL_UNDERTEMP;
     s_thresholds[9].threshold = -10.0f; // -10°C
     s_thresholds[9].hysteresis = 2.0f;  // 2°C
     s_thresholds[9].severity = FAULT_SEVERITY_WARNING;
     s_thresholds[9].action = FAULT_ACTION_LIMIT_CHARGE;
     s_thresholds[9].enabled = true;
     
     s_thresholds[10].fault_type = FAULT_CELL_OVERTEMP;
     s_thresholds[10].threshold = 55.0f; // 55°C
     s_thresholds[10].hysteresis = 5.0f; // 5°C
     s_thresholds[10].severity = FAULT_SEVERITY_CRITICAL;
     s_thresholds[10].action = FAULT_ACTION_LIMIT_DISCHARGE;
     s_thresholds[10].enabled = true;
     
     /* Communication thresholds */
     s_thresholds[11].fault_type = FAULT_BMS_COMMUNICATION;
     s_thresholds[11].threshold = 3.0f;  // 3 communication errors
     s_thresholds[11].hysteresis = 1.0f; // 1 error
     s_thresholds[11].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[11].action = FAULT_ACTION_PAUSE_OPERATION;
     s_thresholds[11].enabled = true;
     
     s_thresholds[12].fault_type = FAULT_EXTERNAL_COMMUNICATION;
     s_thresholds[12].threshold = 5.0f;  // 5 communication errors
     s_thresholds[12].hysteresis = 1.0f; // 1 error
     s_thresholds[12].severity = FAULT_SEVERITY_WARNING;
     s_thresholds[12].action = FAULT_ACTION_ALERT;
     s_thresholds[12].enabled = true;
     
     /* Thermal runaway detection */
     s_thresholds[13].fault_type = FAULT_THERMAL_RUNAWAY;
     s_thresholds[13].threshold = 1.0f;  // Detection flag
     s_thresholds[13].hysteresis = 0.5f; // Hysteresis
     s_thresholds[13].severity = FAULT_SEVERITY_EMERGENCY;
     s_thresholds[13].action = FAULT_ACTION_SHUTDOWN;
     s_thresholds[13].enabled = true;
     
     /* Sensor failures */
     s_thresholds[14].fault_type = FAULT_TEMP_SENSOR_FAILURE;
     s_thresholds[14].threshold = 1.0f;  // Detection flag
     s_thresholds[14].hysteresis = 0.5f; // Hysteresis
     s_thresholds[14].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[14].action = FAULT_ACTION_ALERT;
     s_thresholds[14].enabled = true;
     
     s_thresholds[15].fault_type = FAULT_VOLTAGE_SENSOR;
     s_thresholds[15].threshold = 1.0f;  // Detection flag
     s_thresholds[15].hysteresis = 0.5f; // Hysteresis
     s_thresholds[15].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[15].action = FAULT_ACTION_ALERT;
     s_thresholds[15].enabled = true;
     
     s_thresholds[16].fault_type = FAULT_CURRENT_SENSOR;
     s_thresholds[16].threshold = 1.0f;  // Detection flag
     s_thresholds[16].hysteresis = 0.5f; // Hysteresis
     s_thresholds[16].severity = FAULT_SEVERITY_ERROR;
     s_thresholds[16].action = FAULT_ACTION_ALERT;
     s_thresholds[16].enabled = true;
     
     /* Physical faults */
     s_thresholds[17].fault_type = FAULT_SMOKE_DETECTED;
     s_thresholds[17].threshold = 1.0f;  // Detection flag
     s_thresholds[17].hysteresis = 0.5f; // Hysteresis
     s_thresholds[17].severity = FAULT_SEVERITY_EMERGENCY;
     s_thresholds[17].action = FAULT_ACTION_SHUTDOWN;
     s_thresholds[17].enabled = true;
     
     s_thresholds[18].fault_type = FAULT_WATER_LEAK;
     s_thresholds[18].threshold = 1.0f;  // Detection flag
     s_thresholds[18].hysteresis = 0.5f; // Hysteresis
     s_thresholds[18].severity = FAULT_SEVERITY_CRITICAL;
     s_thresholds[18].action = FAULT_ACTION_SHUTDOWN;
     s_thresholds[18].enabled = true;
     
     s_thresholds[19].fault_type = FAULT_ENCLOSURE_BREACH;
     s_thresholds[19].threshold = 1.0f;  // Detection flag
     s_thresholds[19].hysteresis = 0.5f; // Hysteresis
     s_thresholds[19].severity = FAULT_SEVERITY_WARNING;
     s_thresholds[19].action = FAULT_ACTION_ALERT;
     s_thresholds[19].enabled = true;
     
     /* System critical fault */
     s_thresholds[20].fault_type = FAULT_SYSTEM_CRITICAL;
     s_thresholds[20].threshold = 1.0f;  // Detection flag
     s_thresholds[20].hysteresis = 0.5f; // Hysteresis
     s_thresholds[20].severity = FAULT_SEVERITY_EMERGENCY;
     s_thresholds[20].action = FAULT_ACTION_SHUTDOWN;
     s_thresholds[20].enabled = true;
     
     return ESP_OK;
 esp_err_t fault_detector_enable_fault(fault_type_t fault_type, bool enable) {
     if (!s_initialized) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Fault detector not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (fault_type == FAULT_NONE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Invalid fault type");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         LOG_ERROR(FAULT_DETECTOR_TAG, "Failed to take fault mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     bool found = false;
     for (int i = 0; i < FAULT_DETECTOR_MAX_THRESHOLDS; i++) {
         if (s_thresholds[i].fault_type == fault_type) {
             s_thresholds[i].enabled = enable;
             found = true;
             break;
         }
     }
 
     xSemaphoreGive(s_fault_mutex);
 
     if (!found) {
         LOG_WARN(FAULT_DETECTOR_TAG, "Fault type not found");
         return ESP_ERR_NOT_FOUND;
     }
 
     LOG_INFO(FAULT_DETECTOR_TAG, "Fault %s %s", 
              get_fault_type_string(fault_type), 
              enable ? "enabled" : "disabled");
     return ESP_OK;
 }
 }
 
/**
 * @brief Save current fault thresholds to a configuration file
 * 
 * @param config_file Path to the configuration file
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t fault_detector_save_config(const char *config_file) {
    if (config_file == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Open the file for writing
    FILE *file = fopen(config_file, "w");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open config file for writing: %s", config_file);
        return ESP_ERR_INVALID_STATE;
    }

    // Acquire mutex to prevent configuration changes during save
    if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for saving configuration");
        fclose(file);
        return ESP_ERR_TIMEOUT;
    }

    // Write header
    fprintf(file, "# BESS 100KW/200KWH Fault Detector Configuration\n");
    fprintf(file, "# Auto-generated on: %s\n\n", __DATE__);

    // Write general settings
    fprintf(file, "[General]\n");
    fprintf(file, "auto_reset=%d\n", s_config.auto_reset);
    fprintf(file, "monitoring_interval=%d\n", s_config.monitoring_interval);
    fprintf(file, "consecutive_readings=%d\n", s_config.consecutive_readings);
    fprintf(file, "fault_persistence=%d\n", s_config.fault_persistence);
    fprintf(file, "log_interval=%d\n", s_config.log_interval);
    fprintf(file, "cloud_reporting=%d\n", s_config.cloud_reporting);
    fprintf(file, "enable_watchdog=%d\n\n", s_config.enable_watchdog);

    // Write fault thresholds section
    fprintf(file, "[Thresholds]\n");
    
    // Iterate through all known fault types to save their thresholds
    for (uint32_t i = 0; i < FAULT_TYPE_COUNT; i++) {
        fault_type_t fault = (1UL << i);
        
        // Skip if this bit position doesn't correspond to a defined fault
        if (fault_detector_get_description(fault) == NULL) {
            continue;
        }
        
        // Get threshold data for this fault
        fault_threshold_t *threshold = &s_thresholds[i];
        
        // Only write configured thresholds (where valid flag is set)
        if (threshold->valid) {
            fprintf(file, "fault_%08x_threshold=%.4f\n", fault, threshold->threshold);
            fprintf(file, "fault_%08x_hysteresis=%.4f\n", fault, threshold->hysteresis);
            fprintf(file, "fault_%08x_enabled=%d\n", fault, threshold->enabled);
        }
    }
    
    fprintf(file, "\n[Responses]\n");
    
    // Iterate through all fault types to save their response actions
    for (uint32_t i = 0; i < FAULT_TYPE_COUNT; i++) {
        fault_type_t fault = (1UL << i);
        
        // Skip if this bit position doesn't correspond to a defined fault
        if (fault_detector_get_description(fault) == NULL) {
            continue;
        }
        
        // Get response data for this fault
        fault_response_t *response = &s_responses[i];
        
        // Only write configured responses (where valid flag is set)
        if (response->valid) {
            fprintf(file, "fault_%08x_severity=%d\n", fault, response->severity);
            fprintf(file, "fault_%08x_action=%d\n", fault, response->action);
        }
    }

    // Release mutex
    xSemaphoreGive(s_fault_mutex);
    
    // Close the file
    if (fclose(file) != 0) {
        ESP_LOGE(TAG, "Error closing config file: %s", config_file);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Successfully saved fault detector configuration to %s", config_file);
    return ESP_OK;
}

/**
 * @brief Enable or disable specific fault detection
 * 
 * @param fault_type The fault type to enable/disable
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t fault_detector_enable_fault(fault_type_t fault_type, bool enable) {
    // Check for valid fault type
    if (fault_type == FAULT_NONE || (fault_type & (fault_type - 1)) != 0) {
        // Invalid fault_type: either FAULT_NONE or multiple bits set
        ESP_LOGE(TAG, "Invalid fault type specified: 0x%08x", fault_type);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find the fault index by determining the bit position
    uint8_t fault_index = 0;
    uint32_t temp = fault_type;
    
    while (temp > 1) {
        temp >>= 1;
        fault_index++;
    }
    
    // Check if the fault index is valid
    if (fault_index >= FAULT_TYPE_COUNT) {
        ESP_LOGE(TAG, "Fault index exceeds maximum supported faults: %d", fault_index);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Acquire mutex to ensure thread-safety
    if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for enabling/disabling fault");
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if the threshold is configured for this fault
    if (!s_thresholds[fault_index].valid) {
        ESP_LOGW(TAG, "No threshold configured for fault type: 0x%08x", fault_type);
        xSemaphoreGive(s_fault_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Update the enabled status
    bool previous_state = s_thresholds[fault_index].enabled;
    s_thresholds[fault_index].enabled = enable;
    
    // Log the change
    ESP_LOGI(TAG, "Fault 0x%08x (%s) %s", 
             fault_type, 
             fault_detector_get_description(fault_type),
             enable ? "enabled" : "disabled");
    
    // If disabling a fault, clear any active instances of this fault
    if (!enable && (s_active_faults & fault_type)) {
        s_active_faults &= ~fault_type;
        ESP_LOGI(TAG, "Cleared active fault 0x%08x after disabling", fault_type);
        
        // If we have callbacks registered, notify them of the change
        if (s_callback_count > 0) {
            fault_event_t event = {
                .type = fault_type,
                .severity = FAULT_SEVERITY_INFO,
                .action = FAULT_ACTION_NONE,
                .timestamp = esp_timer_get_time() / 1000000, // Convert to seconds
                .module_id = 0xFF, // System level
                .cell_id = 0xFF,   // Not applicable
                .measured_value = 0.0f,
                .threshold_value = 0.0f,
                .description = "Fault cleared due to detection disable"
            };
            
            // Notify callbacks
            notify_fault_callbacks(&event);
        }
    }
    
    // Release mutex
    xSemaphoreGive(s_fault_mutex);
    
    // If the state actually changed, save the configuration to persistent storage
    if (previous_state != enable && s_config.auto_save_config) {
        ESP_LOGI(TAG, "Auto-saving configuration changes");
        fault_detector_save_config(s_config_file_path);
    }
    
    return ESP_OK;
}