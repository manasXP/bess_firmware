/**
 * @file common_utils.c
 * @brief Implementation of common utilities for the BESS firmware
 * 
 * This file implements the common utilities, functions, and helpers
 * defined in common_utils.h for the BESS (Battery Energy Storage System) firmware.
 * 
 * @copyright Copyright (c) 2025
 */

 #include <stdlib.h>
 #include <stdarg.h>
 #include <stdio.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "esp_log.h"
 #include "esp_system.h"
 #include "common_utils.h"
 
 // Tag for ESP log messages
 static const char *TAG = "BESS_COMMON";
 
 // Current log level
 static bess_log_level_t s_log_level = BESS_LOG_INFO;
 
 // Current log destinations
 static uint8_t s_log_destinations = BESS_LOG_DEST_CONSOLE;
 
 // Mutex for log operations
 static SemaphoreHandle_t s_log_mutex = NULL;
 
 // Structure for protected memory
 typedef struct {
     void *ptr;
     SemaphoreHandle_t mutex;
 } protected_memory_t;
 
 // Structure for event callback
 typedef struct event_callback_node {
     bess_event_callback_t callback;
     void *user_data;
     struct event_callback_node *next;
 } event_callback_node_t;
 
 // Linked list of event callbacks for each event type
 static event_callback_node_t *s_event_callbacks[BESS_EVENT_MAX] = {NULL};
 
 // Mutex for event callbacks
 static SemaphoreHandle_t s_event_mutex = NULL;
 
 /**
  * @brief Initialize the logging system
  */
 esp_err_t bess_log_init(bess_log_level_t level, uint8_t destinations) {
     // Create mutex if not already created
     if (s_log_mutex == NULL) {
         s_log_mutex = xSemaphoreCreateMutex();
         if (s_log_mutex == NULL) {
             ESP_LOGE(TAG, "Failed to create log mutex");
             return ESP_FAIL;
         }
     }
 
     // Set log level and destinations
     s_log_level = level;
     s_log_destinations = destinations;
 
     ESP_LOGI(TAG, "Log system initialized, level: %d, destinations: 0x%02x", level, destinations);
     return ESP_OK;
 }
 
 /**
  * @brief Set the log level
  */
 void bess_log_set_level(bess_log_level_t level) {
     if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == pdTRUE) {
         s_log_level = level;
         xSemaphoreGive(s_log_mutex);
     }
 }
 
 /**
  * @brief Set the log destinations
  */
 void bess_log_set_destinations(uint8_t destinations) {
     if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == pdTRUE) {
         s_log_destinations = destinations;
         xSemaphoreGive(s_log_mutex);
     }
 }
 
 /**
  * @brief Map BESS log level to ESP log level
  */
 static esp_log_level_t map_log_level(bess_log_level_t level) {
     switch (level) {
         case BESS_LOG_NONE:    return ESP_LOG_NONE;
         case BESS_LOG_ERROR:   return ESP_LOG_ERROR;
         case BESS_LOG_WARN:    return ESP_LOG_WARN;
         case BESS_LOG_INFO:    return ESP_LOG_INFO;
         case BESS_LOG_DEBUG:   return ESP_LOG_DEBUG;
         case BESS_LOG_VERBOSE: return ESP_LOG_VERBOSE;
         default:               return ESP_LOG_INFO;
     }
 }
 
 /**
  * @brief Log to SD card
  */
 static void log_to_sd_card(bess_log_level_t level, const char *tag, const char *msg) {
     // TODO: Implement SD card logging
     // This would typically involve:
     // 1. Opening a log file on the SD card (creating if it doesn't exist)
     // 2. Writing a formatted log entry with timestamp
     // 3. Closing the file or flushing the buffer
     
     // For now, just report that SD card logging is not implemented
     ESP_LOGD(TAG, "SD card logging not implemented");
 }
 
 /**
  * @brief Log to AWS CloudWatch
  */
 static void log_to_cloudwatch(bess_log_level_t level, const char *tag, const char *msg) {
     // TODO: Implement AWS CloudWatch logging
     // This would typically involve:
     // 1. Formatting the log message according to CloudWatch requirements
     // 2. Using AWS IoT or another AWS SDK to send the log
     // 3. Handling any connectivity issues
     
     // For now, just report that CloudWatch logging is not implemented
     ESP_LOGD(TAG, "CloudWatch logging not implemented");
 }
 
 /**
  * @brief Log a message to all enabled destinations
  */
 void bess_log(bess_log_level_t level, const char *tag, const char *format, ...) {
     // Check if we should log at this level
     if (level > s_log_level) {
         return;
     }
 
     // Take the log mutex
     if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         // If we can't get the mutex, still try to log to console as a fallback
         ESP_LOGE(TAG, "Failed to take log mutex");
         return;
     }
 
     // Format the message
     va_list args;
     va_start(args, format);
     char msg[512]; // Fixed buffer size, can be adjusted as needed
     vsnprintf(msg, sizeof(msg), format, args);
     va_end(args);
 
     // Log to console if enabled
     if (s_log_destinations & BESS_LOG_DEST_CONSOLE) {
         esp_log_level_t esp_level = map_log_level(level);
         esp_log_write(esp_level, tag, "%s", msg);
     }
 
     // Log to SD card if enabled
     if (s_log_destinations & BESS_LOG_DEST_SDCARD) {
         log_to_sd_card(level, tag, msg);
     }
 
     // Log to CloudWatch if enabled
     if (s_log_destinations & BESS_LOG_DEST_CLOUD) {
         log_to_cloudwatch(level, tag, msg);
     }
 
     // Release the mutex
     xSemaphoreGive(s_log_mutex);
 }
 
 /**
  * @brief Initialize the event system
  */
 static esp_err_t init_event_system(void) {
     if (s_event_mutex == NULL) {
         s_event_mutex = xSemaphoreCreateMutex();
         if (s_event_mutex == NULL) {
             ESP_LOGE(TAG, "Failed to create event mutex");
             return ESP_FAIL;
         }
     }
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for a specific event
  */
 esp_err_t bess_register_event_callback(bess_event_t event, bess_event_callback_t callback, void *user_data) {
     // Check parameters
     if (event >= BESS_EVENT_MAX || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Initialize event system if needed
     esp_err_t ret = init_event_system();
     if (ret != ESP_OK) {
         return ret;
     }
 
     // Create new callback node
     event_callback_node_t *new_node = (event_callback_node_t *)malloc(sizeof(event_callback_node_t));
     if (new_node == NULL) {
         ESP_LOGE(TAG, "Failed to allocate memory for event callback");
         return ESP_ERR_NO_MEM;
     }
 
     new_node->callback = callback;
     new_node->user_data = user_data;
     new_node->next = NULL;
 
     // Take the mutex
     if (xSemaphoreTake(s_event_mutex, portMAX_DELAY) != pdTRUE) {
         free(new_node);
         return ESP_FAIL;
     }
 
     // Add to the linked list
     if (s_event_callbacks[event] == NULL) {
         s_event_callbacks[event] = new_node;
     } else {
         // Add to the end of the list
         event_callback_node_t *node = s_event_callbacks[event];
         while (node->next != NULL) {
             node = node->next;
         }
         node->next = new_node;
     }
 
     // Release the mutex
     xSemaphoreGive(s_event_mutex);
 
     ESP_LOGI(TAG, "Registered event callback for event %d", event);
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a callback for a specific event
  */
 esp_err_t bess_unregister_event_callback(bess_event_t event, bess_event_callback_t callback) {
     // Check parameters
     if (event >= BESS_EVENT_MAX || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Check if event system is initialized
     if (s_event_mutex == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Take the mutex
     if (xSemaphoreTake(s_event_mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_FAIL;
     }
 
     // Find and remove the callback
     event_callback_node_t *prev = NULL;
     event_callback_node_t *node = s_event_callbacks[event];
     bool found = false;
 
     while (node != NULL) {
         if (node->callback == callback) {
             // Remove this node
             if (prev == NULL) {
                 // First node
                 s_event_callbacks[event] = node->next;
             } else {
                 prev->next = node->next;
             }
             free(node);
             found = true;
             break;
         }
         prev = node;
         node = node->next;
     }
 
     // Release the mutex
     xSemaphoreGive(s_event_mutex);
 
     if (!found) {
         ESP_LOGW(TAG, "Event callback not found for event %d", event);
         return ESP_ERR_NOT_FOUND;
     }
 
     ESP_LOGI(TAG, "Unregistered event callback for event %d", event);
     return ESP_OK;
 }
 
 /**
  * @brief Trigger an event
  * 
  * This function is internal but provided here to help understand the event system.
  * In a real implementation, this would be exposed through an appropriate interface.
  */
 esp_err_t bess_trigger_event(bess_event_t event, uint32_t data) {
     // Check parameters
     if (event >= BESS_EVENT_MAX) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Check if event system is initialized
     if (s_event_mutex == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Take the mutex
     if (xSemaphoreTake(s_event_mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_FAIL;
     }
 
     // Call all callbacks
     event_callback_node_t *node = s_event_callbacks[event];
     while (node != NULL) {
         // Store next pointer in case the callback unregisters itself
         event_callback_node_t *next = node->next;
         
         // Release the mutex during the callback to avoid deadlocks
         xSemaphoreGive(s_event_mutex);
         
         // Call the callback
         node->callback(event, data, node->user_data);
         
         // Take the mutex again
         if (xSemaphoreTake(s_event_mutex, portMAX_DELAY) != pdTRUE) {
             return ESP_FAIL;
         }
         
         node = next;
     }
 
     // Release the mutex
     xSemaphoreGive(s_event_mutex);
 
     ESP_LOGD(TAG, "Triggered event %d with data %lu", event, (unsigned long)data);
     return ESP_OK;
 }
 
 /**
  * @brief Create a mutex-protected memory block
  */
 void* bess_create_protected_memory(size_t size) {
     if (size == 0) {
         ESP_LOGE(TAG, "Invalid memory size");
         return NULL;
     }
 
     // Allocate the protected memory structure
     protected_memory_t *protected = (protected_memory_t *)malloc(sizeof(protected_memory_t));
     if (protected == NULL) {
         ESP_LOGE(TAG, "Failed to allocate protected memory structure");
         return NULL;
     }
 
     // Allocate the memory block
     protected->ptr = malloc(size);
     if (protected->ptr == NULL) {
         ESP_LOGE(TAG, "Failed to allocate memory block of size %zu", size);
         free(protected);
         return NULL;
     }
 
     // Create the mutex
     protected->mutex = xSemaphoreCreateMutex();
     if (protected->mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex for protected memory");
         free(protected->ptr);
         free(protected);
         return NULL;
     }
 
     // Clear the memory
     memset(protected->ptr, 0, size);
 
     ESP_LOGD(TAG, "Created protected memory block of size %zu at %p", size, protected);
     return (void *)protected;
 }
 
 /**
  * @brief Free a protected memory block
  */
 void bess_free_protected_memory(void *ptr) {
     if (ptr == NULL) {
         ESP_LOGW(TAG, "Attempt to free NULL protected memory");
         return;
     }
 
     protected_memory_t *protected = (protected_memory_t *)ptr;
 
     // Delete the mutex
     if (protected->mutex != NULL) {
         vSemaphoreDelete(protected->mutex);
     }
 
     // Free the memory block
     if (protected->ptr != NULL) {
         free(protected->ptr);
     }
 
     // Free the protected memory structure
     free(protected);
 
     ESP_LOGD(TAG, "Freed protected memory at %p", ptr);
 }
 
 /**
  * @brief Get the firmware version string
  */
 const char* bess_get_firmware_version(void) {
     static char version[16];
     snprintf(version, sizeof(version), "%d.%d.%d", 
              BESS_FIRMWARE_VERSION_MAJOR, 
              BESS_FIRMWARE_VERSION_MINOR, 
              BESS_FIRMWARE_VERSION_PATCH);
     return version;
 }
 
 /**
  * @brief Convert a BESS error code to a string
  */
 const char* bess_error_to_string(bess_error_t err) {
     switch (err) {
         case BESS_OK:                       return "No error";
         case BESS_ERROR_INVALID_PARAMETER:  return "Invalid parameter";
         case BESS_ERROR_TIMEOUT:            return "Timeout";
         case BESS_ERROR_COMMUNICATION:      return "Communication error";
         case BESS_ERROR_RESOURCE_ALLOCATION: return "Resource allocation failed";
         case BESS_ERROR_NOT_INITIALIZED:    return "Not initialized";
         case BESS_ERROR_ALREADY_INITIALIZED: return "Already initialized";
         case BESS_ERROR_MODULE_OFFLINE:     return "Module offline";
         case BESS_ERROR_SAFETY_VIOLATION:   return "Safety violation";
         case BESS_ERROR_SYSTEM_FAULT:       return "System fault";
         case BESS_ERROR_FIRMWARE_MISMATCH:  return "Firmware mismatch";
         case BESS_ERROR_UNKNOWN:            return "Unknown error";
         default:                            return "Undefined error";
     }
 }
 
 /**
  * @brief Convert a BESS state to a string
  */
 const char* bess_state_to_string(bess_state_t state) {
     switch (state) {
         case BESS_STATE_INIT:       return "Initializing";
         case BESS_STATE_STANDBY:    return "Standby";
         case BESS_STATE_CHARGING:   return "Charging";
         case BESS_STATE_DISCHARGING: return "Discharging";
         case BESS_STATE_ERROR:      return "Error";
         case BESS_STATE_MAINTENANCE: return "Maintenance";
         case BESS_STATE_SHUTDOWN:   return "Shutdown";
         default:                    return "Unknown state";
     }
 }
 
 /**
  * @brief Safe version of strncpy that ensures null-termination
  */
 char* bess_strncpy(char *dest, const char *src, size_t size) {
     if (dest == NULL || src == NULL || size == 0) {
         return dest;
     }
 
     // Copy the string
     strncpy(dest, src, size - 1);
     
     // Ensure null-termination
     dest[size - 1] = '\0';
     
     return dest;
 }
 
 /**
  * @brief Create a string copy on the heap
  */
 char* bess_strdup(const char *str) {
     if (str == NULL) {
         return NULL;
     }
 
     // Allocate memory for the string
     size_t len = strlen(str) + 1;
     char *result = (char *)malloc(len);
     
     if (result == NULL) {
         ESP_LOGE(TAG, "Failed to allocate memory for string duplication");
         return NULL;
     }
 
     // Copy the string
     memcpy(result, str, len);
     
     return result;
 }
 
 /**
  * @brief Take the mutex of a protected memory block
  * 
  * This function is internal but provided here to help understand the protected memory system.
  * In a real implementation, this would be part of a more comprehensive interface.
  */
 esp_err_t bess_protected_memory_take(void *ptr, TickType_t timeout) {
     if (ptr == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     protected_memory_t *protected = (protected_memory_t *)ptr;
     
     if (protected->mutex == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (xSemaphoreTake(protected->mutex, timeout) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
 
     return ESP_OK;
 }
 
 /**
  * @brief Give the mutex of a protected memory block
  * 
  * This function is internal but provided here to help understand the protected memory system.
  * In a real implementation, this would be part of a more comprehensive interface.
  */
 esp_err_t bess_protected_memory_give(void *ptr) {
     if (ptr == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     protected_memory_t *protected = (protected_memory_t *)ptr;
     
     if (protected->mutex == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (xSemaphoreGive(protected->mutex) != pdTRUE) {
         return ESP_FAIL;
     }
 
     return ESP_OK;
 }
 
 /**
  * @brief Get a pointer to the data in a protected memory block
  * 
  * This function is internal but provided here to help understand the protected memory system.
  * In a real implementation, this would be part of a more comprehensive interface.
  */
 void* bess_protected_memory_get_ptr(void *ptr) {
     if (ptr == NULL) {
         return NULL;
     }
 
     protected_memory_t *protected = (protected_memory_t *)ptr;
     return protected->ptr;
 }