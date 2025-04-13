/**
 * @file queue_manager.c
 * @brief Implementation of queue management system for BESS middleware
 *
 * This file implements the centralized queue management system for the 
 * 100KW/200KWH BESS controller firmware, providing message passing between
 * different subsystems using FreeRTOS queue primitives with support for 
 * priority messaging, timeout handling, and queue monitoring.
 *
 * @author JouleWorX Firmware Team
 * @date April 2025
 */

 #include "queue_manager.h"
 #include "esp_log.h"
 #include "freertos/semphr.h"
 #include "esp_timer.h"
 #include <string.h>
 
 #define TAG "QUEUE_MGR"
 
 /**
  * @brief Internal queue manager state structure
  */
 typedef struct {
     queue_handle_t queues[QUEUE_MANAGER_MAX_QUEUES];  /**< Array of managed queues */
     uint32_t queue_count;                             /**< Number of active queues */
     uint32_t default_timeout_ms;                      /**< Default timeout for operations */
     SemaphoreHandle_t mutex;                          /**< Mutex for thread safety */
     bool initialized;                                 /**< Initialization status */
     bool suspended;                                   /**< Whether operations are suspended */
     uint32_t next_message_id;                         /**< For generating unique message IDs */
     
     /**
      * @brief Callback registration structure
      */
     struct {
         queue_event_callback_t callback;              /**< Callback function */
         void* context;                                /**< User context for callback */
         bool in_use;                                  /**< Whether this slot is in use */
     } callbacks[QUEUE_MANAGER_MAX_QUEUES];
 } queue_manager_state_t;
 
 /* Global state for the queue manager */
 static queue_manager_state_t s_state = {
     .queue_count = 0,
     .default_timeout_ms = QUEUE_MANAGER_DEFAULT_TIMEOUT_MS,
     .mutex = NULL,
     .initialized = false,
     .suspended = false,
     .next_message_id = 1
 };
 
 /* Forward declarations for internal functions */
 static void queue_manager_update_stats(queue_handle_t* handle, bool is_send, bool is_successful);
 static esp_err_t queue_manager_notify_callbacks(queue_type_t type, void* data);
 static bool queue_manager_take_mutex(uint32_t timeout_ms);
 static void queue_manager_give_mutex(void);
 
 esp_err_t queue_manager_init(void) {
     if (s_state.initialized) {
         ESP_LOGW(TAG, "Queue manager already initialized");
         return ESP_OK;
     }
     
     // Create mutex for thread safety
     s_state.mutex = xSemaphoreCreateMutex();
     if (s_state.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_FAIL;
     }
     
     // Initialize queue array
     memset(s_state.queues, 0, sizeof(s_state.queues));
     s_state.queue_count = 0;
     
     // Initialize callback registry
     memset(s_state.callbacks, 0, sizeof(s_state.callbacks));
     
     // Set initialization flag
     s_state.initialized = true;
     s_state.next_message_id = 1;
     
     ESP_LOGI(TAG, "Queue manager initialized");
     return ESP_OK;
 }
 
 esp_err_t queue_manager_deinit(void) {
     if (!s_state.initialized) {
         ESP_LOGW(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in deinit");
         return ESP_FAIL;
     }
     
     // Delete all queues
     for (uint32_t i = 0; i < s_state.queue_count; i++) {
         if (s_state.queues[i].is_active && s_state.queues[i].handle != NULL) {
             vQueueDelete(s_state.queues[i].handle);
             s_state.queues[i].handle = NULL;
             s_state.queues[i].is_active = false;
         }
     }
     
     // Reset state
     s_state.queue_count = 0;
     s_state.initialized = false;
     s_state.suspended = false;
     
     // Release and delete mutex
     queue_manager_give_mutex();
     vSemaphoreDelete(s_state.mutex);
     s_state.mutex = NULL;
     
     ESP_LOGI(TAG, "Queue manager deinitialized");
     return ESP_OK;
 }
 
 esp_err_t queue_manager_create_queue(const char* name, 
                                     queue_type_t type,
                                     uint32_t queue_size, 
                                     uint32_t item_size,
                                     queue_handle_t** handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (name == NULL || handle == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (strlen(name) >= QUEUE_MANAGER_NAME_MAX_LEN) {
         ESP_LOGE(TAG, "Queue name too long");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in create_queue");
         return ESP_FAIL;
     }
     
     // Check if we've reached the maximum number of queues
     if (s_state.queue_count >= QUEUE_MANAGER_MAX_QUEUES) {
         ESP_LOGE(TAG, "Maximum number of queues reached");
         queue_manager_give_mutex();
         return ESP_FAIL;
     }
     
     // Check if queue with this name already exists
     for (uint32_t i = 0; i < s_state.queue_count; i++) {
         if (s_state.queues[i].is_active && 
             strcmp(s_state.queues[i].name, name) == 0) {
             ESP_LOGE(TAG, "Queue with name %s already exists", name);
             queue_manager_give_mutex();
             return ESP_FAIL;
         }
     }
     
     // Find an available slot
     int slot = -1;
     for (uint32_t i = 0; i < QUEUE_MANAGER_MAX_QUEUES; i++) {
         if (!s_state.queues[i].is_active) {
             slot = i;
             break;
         }
     }
     
     if (slot == -1) {
         ESP_LOGE(TAG, "No available queue slots");
         queue_manager_give_mutex();
         return ESP_FAIL;
     }
     
     // Create FreeRTOS queue
     QueueHandle_t queue_handle = xQueueCreate(queue_size, item_size);
     if (queue_handle == NULL) {
         ESP_LOGE(TAG, "Failed to create FreeRTOS queue %s", name);
         queue_manager_give_mutex();
         return ESP_FAIL;
     }
     
     // Initialize the queue handle structure
     queue_handle_t* queue = &s_state.queues[slot];
     strncpy(queue->name, name, QUEUE_MANAGER_NAME_MAX_LEN - 1);
     queue->name[QUEUE_MANAGER_NAME_MAX_LEN - 1] = '\0';
     queue->handle = queue_handle;
     queue->type = type;
     queue->size = queue_size;
     queue->item_size = item_size;
     queue->is_active = true;
     
     // Initialize statistics
     memset(&queue->stats, 0, sizeof(queue_stats_t));
     
     // Update queue count if we're using a new slot
     if (slot == s_state.queue_count) {
         s_state.queue_count++;
     }
     
     *handle = queue;
     
     ESP_LOGI(TAG, "Created queue '%s' (type: %d, size: %u, item size: %u bytes)",
              name, type, queue_size, item_size);
     
     queue_manager_give_mutex();
     return ESP_OK;
 }
 
 esp_err_t queue_manager_delete_queue(queue_handle_t* handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active) {
         ESP_LOGE(TAG, "Invalid queue handle");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in delete_queue");
         return ESP_FAIL;
     }
     
     // Verify this is one of our managed queues
     bool found = false;
     for (uint32_t i = 0; i < s_state.queue_count; i++) {
         if (&s_state.queues[i] == handle) {
             found = true;
             break;
         }
     }
     
     if (!found) {
         ESP_LOGE(TAG, "Queue handle not found in managed queues");
         queue_manager_give_mutex();
         return ESP_FAIL;
     }
     
     // Delete the FreeRTOS queue
     if (handle->handle != NULL) {
         vQueueDelete(handle->handle);
         handle->handle = NULL;
     }
     
     ESP_LOGI(TAG, "Deleted queue '%s'", handle->name);
     
     // Mark as inactive
     handle->is_active = false;
     
     queue_manager_give_mutex();
     return ESP_OK;
 }
 
 esp_err_t queue_manager_send(queue_handle_t* handle, 
                              const void* data,
                              uint32_t timeout_ms) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || data == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (s_state.suspended) {
         ESP_LOGW(TAG, "Queue operations suspended");
         return ESP_FAIL;
     }
     
     // Default timeout if 0 is specified
     if (timeout_ms == 0) {
         timeout_ms = s_state.default_timeout_ms;
     }
     
     // Send message to queue
     BaseType_t result = xQueueSend(handle->handle, data, pdMS_TO_TICKS(timeout_ms));
     
     // Update statistics (no mutex needed for atomic updates)
     queue_manager_update_stats(handle, true, result == pdTRUE);
     
     if (result != pdTRUE) {
         ESP_LOGW(TAG, "Failed to send to queue '%s' (timeout: %u ms)", 
                  handle->name, timeout_ms);
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_send_with_priority(queue_handle_t* handle, 
                                           queue_message_t* message,
                                           uint32_t timeout_ms) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || message == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (s_state.suspended) {
         ESP_LOGW(TAG, "Queue operations suspended");
         return ESP_FAIL;
     }
     
     // Default timeout if 0 is specified
     if (timeout_ms == 0) {
         timeout_ms = s_state.default_timeout_ms;
     }
     
     // Set timestamp and message ID
     message->timestamp = esp_timer_get_time();
     
     // Generate unique message ID (thread-safe)
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex for message ID");
         return ESP_FAIL;
     }
     message->message_id = s_state.next_message_id++;
     queue_manager_give_mutex();
     
     BaseType_t result;
     
     // Send based on priority
     if (message->priority >= QUEUE_PRIORITY_HIGH) {
         result = xQueueSendToFront(handle->handle, message, pdMS_TO_TICKS(timeout_ms));
     } else {
         result = xQueueSend(handle->handle, message, pdMS_TO_TICKS(timeout_ms));
     }
     
     // Update statistics
     queue_manager_update_stats(handle, true, result == pdTRUE);
     
     // Notify callbacks about the message
     if (result == pdTRUE) {
         queue_manager_notify_callbacks(handle->type, message->data);
     }
     
     if (result != pdTRUE) {
         ESP_LOGW(TAG, "Failed to send priority message to queue '%s' (timeout: %u ms)", 
                  handle->name, timeout_ms);
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_receive(queue_handle_t* handle, 
                                void* data,
                                uint32_t timeout_ms) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || data == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (s_state.suspended) {
         ESP_LOGW(TAG, "Queue operations suspended");
         return ESP_FAIL;
     }
     
     // Receive message from queue
     BaseType_t result = xQueueReceive(handle->handle, data, pdMS_TO_TICKS(timeout_ms));
     
     // Update statistics
     queue_manager_update_stats(handle, false, result == pdTRUE);
     
     if (result != pdTRUE) {
         if (timeout_ms > 0) {
             ESP_LOGV(TAG, "Timeout receiving from queue '%s' (timeout: %u ms)", 
                      handle->name, timeout_ms);
         }
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_receive_with_metadata(queue_handle_t* handle, 
                                              queue_message_t* message,
                                              uint32_t timeout_ms) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || message == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (s_state.suspended) {
         ESP_LOGW(TAG, "Queue operations suspended");
         return ESP_FAIL;
     }
     
     // Receive message from queue
     BaseType_t result = xQueueReceive(handle->handle, message, pdMS_TO_TICKS(timeout_ms));
     
     // Update statistics
     queue_manager_update_stats(handle, false, result == pdTRUE);
     
     if (result != pdTRUE) {
         if (timeout_ms > 0) {
             ESP_LOGV(TAG, "Timeout receiving message with metadata from queue '%s' (timeout: %u ms)", 
                      handle->name, timeout_ms);
         }
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_flush(queue_handle_t* handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active) {
         ESP_LOGE(TAG, "Invalid queue handle");
         return ESP_FAIL;
     }
     
     // Flush the queue
     xQueueReset(handle->handle);
     
     ESP_LOGI(TAG, "Flushed queue '%s'", handle->name);
     return ESP_OK;
 }
 
 esp_err_t queue_manager_is_empty(queue_handle_t* handle, bool* is_empty) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || is_empty == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     UBaseType_t count = uxQueueMessagesWaiting(handle->handle);
     *is_empty = (count == 0);
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_is_full(queue_handle_t* handle, bool* is_full) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || is_full == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     UBaseType_t spaces = uxQueueSpacesAvailable(handle->handle);
     *is_full = (spaces == 0);
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_get_waiting_count(queue_handle_t* handle, 
                                          uint32_t* messages_waiting) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || messages_waiting == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     *messages_waiting = (uint32_t)uxQueueMessagesWaiting(handle->handle);
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_get_stats(queue_handle_t* handle, queue_stats_t* stats) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active || stats == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in get_stats");
         return ESP_FAIL;
     }
     
     // Copy statistics
     memcpy(stats, &handle->stats, sizeof(queue_stats_t));
     
     // Update current usage
     stats->current_usage = (uint32_t)uxQueueMessagesWaiting(handle->handle);
     
     queue_manager_give_mutex();
     return ESP_OK;
 }
 
 esp_err_t queue_manager_reset_stats(queue_handle_t* handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL || !handle->is_active) {
         ESP_LOGE(TAG, "Invalid queue handle");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in reset_stats");
         return ESP_FAIL;
     }
     
     // Reset statistics
     memset(&handle->stats, 0, sizeof(queue_stats_t));
     
     // Set current usage
     handle->stats.current_usage = (uint32_t)uxQueueMessagesWaiting(handle->handle);
     
     queue_manager_give_mutex();
     
     ESP_LOGI(TAG, "Reset statistics for queue '%s'", handle->name);
     return ESP_OK;
 }
 
 esp_err_t queue_manager_get_queue_by_name(const char* name, queue_handle_t** handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (name == NULL || handle == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in get_queue_by_name");
         return ESP_FAIL;
     }
     
     // Search for queue with matching name
     for (uint32_t i = 0; i < s_state.queue_count; i++) {
         if (s_state.queues[i].is_active && 
             strcmp(s_state.queues[i].name, name) == 0) {
             *handle = &s_state.queues[i];
             queue_manager_give_mutex();
             return ESP_OK;
         }
     }
     
     // Not found
     queue_manager_give_mutex();
     ESP_LOGW(TAG, "Queue with name '%s' not found", name);
     return ESP_FAIL;
 }
 
 esp_err_t queue_manager_get_queue_by_type(queue_type_t type, queue_handle_t** handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (handle == NULL) {
         ESP_LOGE(TAG, "Invalid handle parameter");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in get_queue_by_type");
         return ESP_FAIL;
     }
     
     // Search for first queue with matching type
     for (uint32_t i = 0; i < s_state.queue_count; i++) {
         if (s_state.queues[i].is_active && s_state.queues[i].type == type) {
             *handle = &s_state.queues[i];
             queue_manager_give_mutex();
             return ESP_OK;
         }
     }
     
     // Not found
     queue_manager_give_mutex();
     ESP_LOGW(TAG, "Queue with type %d not found", type);
     return ESP_FAIL;
 }
 
 esp_err_t queue_manager_register_callback(queue_type_t type, 
                                          queue_event_callback_t callback,
                                          void* context) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback parameter");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in register_callback");
         return ESP_FAIL;
     }
     
     // Find an available callback slot
     int slot = -1;
     for (uint32_t i = 0; i < QUEUE_MANAGER_MAX_QUEUES; i++) {
         if (!s_state.callbacks[i].in_use) {
             slot = i;
             break;
         }
     }
     
     if (slot == -1) {
         ESP_LOGE(TAG, "No available callback slots");
         queue_manager_give_mutex();
         return ESP_FAIL;
     }
     
     // Register the callback
     s_state.callbacks[slot].callback = callback;
     s_state.callbacks[slot].context = context;
     s_state.callbacks[slot].in_use = true;
     
     queue_manager_give_mutex();
     
     ESP_LOGI(TAG, "Registered callback for queue type %d", type);
     return ESP_OK;
 }
 
 esp_err_t queue_manager_unregister_callback(queue_type_t type, 
                                            queue_event_callback_t callback) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback parameter");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in unregister_callback");
         return ESP_FAIL;
     }
     
     // Find the callback in the registry
     bool found = false;
     for (uint32_t i = 0; i < QUEUE_MANAGER_MAX_QUEUES; i++) {
         if (s_state.callbacks[i].in_use && 
             s_state.callbacks[i].callback == callback) {
             // Unregister the callback
             s_state.callbacks[i].in_use = false;
             found = true;
             break;
         }
     }
     
     queue_manager_give_mutex();
     
     if (!found) {
         ESP_LOGW(TAG, "Callback not found in registry");
         return ESP_FAIL;
     }
     
     ESP_LOGI(TAG, "Unregistered callback for queue type %d", type);
     return ESP_OK;
 }
 
 esp_err_t queue_manager_create_message(void* data,
                                       size_t data_size,
                                       queue_priority_t priority,
                                       uint32_t source_id,
                                       uint32_t target_id,
                                       queue_message_t* message) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (data == NULL || message == NULL) {
         ESP_LOGE(TAG, "Invalid parameters");
         return ESP_FAIL;
     }
     
     // Initialize message fields
     message->data = data;
     message->data_size = data_size;
     message->priority = priority;
     message->timestamp = esp_timer_get_time();
     message->source_id = source_id;
     message->target_id = target_id;
     
     // Generate unique message ID (thread-safe)
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex for message ID");
         return ESP_FAIL;
     }
     message->message_id = s_state.next_message_id++;
     queue_manager_give_mutex();
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_set_default_timeout(uint32_t timeout_ms) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     s_state.default_timeout_ms = timeout_ms;
     
     ESP_LOGI(TAG, "Set default timeout to %u ms", timeout_ms);
     return ESP_OK;
 }
 
 esp_err_t queue_manager_get_default_timeout(uint32_t* timeout_ms) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (timeout_ms == NULL) {
         ESP_LOGE(TAG, "Invalid timeout_ms parameter");
         return ESP_FAIL;
     }
     
     *timeout_ms = s_state.default_timeout_ms;
     
     return ESP_OK;
 }
 
 esp_err_t queue_manager_get_queue_count(uint32_t* count) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (count == NULL) {
         ESP_LOGE(TAG, "Invalid count parameter");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in get_queue_count");
         return ESP_FAIL;
     }
     
     // Count active queues
     uint32_t active_count = 0;
     for (uint32_t i = 0; i < s_state.queue_count; i++) {
         if (s_state.queues[i].is_active) {
             active_count++;
         }
     }
     
     *count = active_count;
     
     queue_manager_give_mutex();
     return ESP_OK;
 }
 
 esp_err_t queue_manager_suspend(void) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in suspend");
         return ESP_FAIL;
     }
     
     s_state.suspended = true;
     
     queue_manager_give_mutex();
     
     ESP_LOGI(TAG, "Queue operations suspended");
     return ESP_OK;
 }
 
 esp_err_t queue_manager_resume(void) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in resume");
         return ESP_FAIL;
     }
     
     s_state.suspended = false;
     
     queue_manager_give_mutex();
     
     ESP_LOGI(TAG, "Queue operations resumed");
     return ESP_OK;
 }
 
 esp_err_t queue_manager_print_stats(queue_handle_t* handle) {
     if (!s_state.initialized) {
         ESP_LOGE(TAG, "Queue manager not initialized");
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in print_stats");
         return ESP_FAIL;
     }
     
     if (handle == NULL) {
         // Print stats for all queues
         ESP_LOGI(TAG, "Queue Statistics Summary:");
         ESP_LOGI(TAG, "------------------------------------------------------------------------------");
         ESP_LOGI(TAG, "%-20s | %-6s | %-6s | %-6s | %-6s | %-6s", 
                  "Queue Name", "Sent", "Rcvd", "Drop", "HWM", "Usage");
         ESP_LOGI(TAG, "------------------------------------------------------------------------------");
         
         for (uint32_t i = 0; i < s_state.queue_count; i++) {
             if (s_state.queues[i].is_active) {
                 // Update current usage
                 s_state.queues[i].stats.current_usage = 
                     (uint32_t)uxQueueMessagesWaiting(s_state.queues[i].handle);
                 
                 ESP_LOGI(TAG, "%-20s | %-6u | %-6u | %-6u | %-6u | %-6u", 
                          s_state.queues[i].name,
                          s_state.queues[i].stats.messages_sent,
                          s_state.queues[i].stats.messages_received,
                          s_state.queues[i].stats.messages_dropped,
                          s_state.queues[i].stats.high_watermark,
                          s_state.queues[i].stats.current_usage);
             }
         }
         ESP_LOGI(TAG, "------------------------------------------------------------------------------");
     } else {
         // Print detailed stats for a specific queue
         if (!handle->is_active) {
             ESP_LOGE(TAG, "Invalid queue handle");
             queue_manager_give_mutex();
             return ESP_FAIL;
         }
         
         // Update current usage
         handle->stats.current_usage = (uint32_t)uxQueueMessagesWaiting(handle->handle);
         
         ESP_LOGI(TAG, "Statistics for queue '%s':", handle->name);
         ESP_LOGI(TAG, "  Type:              %d", handle->type);
         ESP_LOGI(TAG, "  Size:              %u items", handle->size);
         ESP_LOGI(TAG, "  Item size:         %u bytes", handle->item_size);
         ESP_LOGI(TAG, "  Messages sent:     %u", handle->stats.messages_sent);
         ESP_LOGI(TAG, "  Messages received: %u", handle->stats.messages_received);
         ESP_LOGI(TAG, "  Messages dropped:  %u", handle->stats.messages_dropped);
         ESP_LOGI(TAG, "  High watermark:    %u items", handle->stats.high_watermark);
         ESP_LOGI(TAG, "  Current usage:     %u items", handle->stats.current_usage);
         ESP_LOGI(TAG, "  Timeouts:          %u", handle->stats.timeouts);
     }
     
     queue_manager_give_mutex();
     return ESP_OK;
 }
 
 /**
  * @brief Helper function to update queue statistics
  * 
  * @param handle Queue handle
  * @param is_send Whether this is a send (true) or receive (false) operation
  * @param is_successful Whether the operation was successful
  */
 static void queue_manager_update_stats(queue_handle_t* handle, bool is_send, bool is_successful) {
     if (handle == NULL || !handle->is_active) {
         return;
     }
     
     // We don't need a mutex here as these are simple atomic increments
     if (is_send) {
         if (is_successful) {
             handle->stats.messages_sent++;
         } else {
             handle->stats.messages_dropped++;
         }
     } else {
         if (is_successful) {
             handle->stats.messages_received++;
         } else {
             handle->stats.timeouts++;
         }
     }
     
     // Update current usage and high watermark
     UBaseType_t current = uxQueueMessagesWaiting(handle->handle);
     handle->stats.current_usage = current;
     
     if (current > handle->stats.high_watermark) {
         handle->stats.high_watermark = current;
     }
 }
 
 /**
  * @brief Notify registered callbacks for a queue event
  * 
  * @param type Queue type
  * @param data Event data
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 static esp_err_t queue_manager_notify_callbacks(queue_type_t type, void* data) {
     if (!s_state.initialized) {
         return ESP_FAIL;
     }
     
     if (!queue_manager_take_mutex(portMAX_DELAY)) {
         ESP_LOGE(TAG, "Failed to acquire mutex in notify_callbacks");
         return ESP_FAIL;
     }
     
     // Invoke all registered callbacks for this queue type
     for (uint32_t i = 0; i < QUEUE_MANAGER_MAX_QUEUES; i++) {
         if (s_state.callbacks[i].in_use) {
             queue_manager_give_mutex();
             
             // Call the callback outside of mutex lock to prevent deadlocks
             s_state.callbacks[i].callback(type, data, s_state.callbacks[i].context);
             
             if (!queue_manager_take_mutex(portMAX_DELAY)) {
                 ESP_LOGE(TAG, "Failed to reacquire mutex in notify_callbacks");
                 return ESP_FAIL;
             }
         }
     }
     
     queue_manager_give_mutex();
     return ESP_OK;
 }
 
 /**
  * @brief Take the queue manager mutex with timeout
  * 
  * @param timeout_ms Timeout in milliseconds
  * @return true if mutex acquired, false if timeout
  */
 static bool queue_manager_take_mutex(uint32_t timeout_ms) {
     if (s_state.mutex == NULL) {
         return false;
     }
     
     return (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
 }
 
 /**
  * @brief Give (release) the queue manager mutex
  */
 static void queue_manager_give_mutex(void) {
     if (s_state.mutex != NULL) {
         xSemaphoreGive(s_state.mutex);
     }
 }