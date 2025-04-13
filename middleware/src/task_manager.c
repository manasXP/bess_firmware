/**
 * @file task_manager.c
 * @brief Implementation of Task Manager for BESS 100KW/200KWH FreeRTOS middleware
 *
 * This module provides task management functionality for the Battery Energy Storage System (BESS).
 * It coordinates the creation, scheduling, monitoring, and synchronization of various system tasks.
 *
 * @copyright Copyright (c) 2025
 */

 #include "task_manager.h"
 #include <string.h>
 #include "esp_system.h"
 #include "esp_timer.h"
 #include "esp_heap_caps.h"
 
 /** @defgroup TASK_MANAGER_PRIVATE Private
  * @{
  */
 
 // Constants
 #define TASK_MANAGER_TAG              "TASK_MGR"
 #define TASK_MANAGER_MAX_SYNC_OBJECTS 20
 #define TASK_MANAGER_STATS_TASK_NAME  "task_stats"
 #define TASK_MANAGER_STATS_STACK_SIZE 4096
 #define TASK_MANAGER_WATCHDOG_CHECK   1000  // Check watchdog every 1000ms
 
 // Task Manager state structure
 typedef struct {
     bool initialized;                         // Initialization flag
     task_manager_config_t config;             // Configuration
     task_handle_t **tasks;                    // Array of task handles
     uint32_t task_count;                      // Current number of tasks
     uint32_t max_tasks;                       // Maximum number of tasks
     task_sync_t **sync_objects;               // Array of synchronization objects
     uint32_t sync_count;                      // Current number of sync objects
     uint32_t max_sync_objects;                // Maximum number of sync objects
     TaskHandle_t stats_task_handle;           // Statistics task handle
     task_stats_t stats;                       // Task statistics
     SemaphoreHandle_t manager_mutex;          // Mutex for thread safety
     uint64_t last_stats_update;               // Last statistics update time
     uint64_t last_watchdog_check;             // Last watchdog check time
 } task_manager_state_t;
 
 // Static instance of task manager state
 static task_manager_state_t s_task_manager = {
     .initialized = false,
     .task_count = 0,
     .sync_count = 0,
     .stats_task_handle = NULL,
     .manager_mutex = NULL,
     .last_stats_update = 0,
     .last_watchdog_check = 0
 };
 
 // Forward declarations of private functions
 static void task_manager_stats_task(void *arg);
 static esp_err_t task_manager_update_stats(void);
 static esp_err_t task_manager_check_watchdogs(void);
 static void task_wrapper_function(void *arg);
 static esp_err_t task_manager_add_task(task_handle_t *task);
 static esp_err_t task_manager_remove_task(task_handle_t *task);
 static esp_err_t task_manager_update_task_status(task_handle_t *task);
 static esp_err_t task_manager_restart_task_if_needed(task_handle_t *task);
 
 /**
  * @brief Locks the task manager mutex
  * 
  * @param timeout_ms Timeout in milliseconds
  * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
  */
 static esp_err_t task_manager_lock(uint32_t timeout_ms) {
     if (s_task_manager.manager_mutex == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     TickType_t ticks = (timeout_ms == portMAX_DELAY) ? 
                         portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
     
     if (xSemaphoreTake(s_task_manager.manager_mutex, ticks) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Unlocks the task manager mutex
  * 
  * @return esp_err_t ESP_OK on success
  */
 static esp_err_t task_manager_unlock(void) {
     if (s_task_manager.manager_mutex == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     xSemaphoreGive(s_task_manager.manager_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Find task by name
  * 
  * @param name Task name
  * @return task_handle_t* Pointer to task handle, or NULL if not found
  */
 static task_handle_t* task_manager_find_task_by_name(const char *name) {
     if (name == NULL || s_task_manager.tasks == NULL) {
         return NULL;
     }
     
     for (uint32_t i = 0; i < s_task_manager.task_count; i++) {
         if (s_task_manager.tasks[i] != NULL && 
             strcmp(s_task_manager.tasks[i]->name, name) == 0) {
             return s_task_manager.tasks[i];
         }
     }
     
     return NULL;
 }
 
 /**
  * @brief Find synchronization object by name
  * 
  * @param name Synchronization object name
  * @return task_sync_t* Pointer to synchronization object, or NULL if not found
  */
 static task_sync_t* task_manager_find_sync_by_name(const char *name) {
     if (name == NULL || s_task_manager.sync_objects == NULL) {
         return NULL;
     }
     
     for (uint32_t i = 0; i < s_task_manager.sync_count; i++) {
         if (s_task_manager.sync_objects[i] != NULL && 
             strcmp(s_task_manager.sync_objects[i]->name, name) == 0) {
             return s_task_manager.sync_objects[i];
         }
     }
     
     return NULL;
 }
 
 /**
  * @brief Task wrapper function that monitors execution time and catches crashes
  * 
  * @param arg Task handle pointer
  */
 static void task_wrapper_function(void *arg) {
     task_handle_t *task = (task_handle_t *)arg;
     
     if (task == NULL || task->handle == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task in wrapper function");
         vTaskDelete(NULL);
         return;
     }
     
     // Task function pointer and argument
     task_function_t task_func = (task_function_t)task->user_data;
     void *task_arg = NULL;
     
     ESP_LOGI(TASK_MANAGER_TAG, "Starting task %s (type %d, priority %d)",
              task->name, task->type, task->priority);
     
     // Main task loop for periodic and event-driven tasks
     TickType_t last_wake_time = xTaskGetTickCount();
     task->last_wake_time = last_wake_time;
     
     while (1) {
         uint64_t start_time = esp_timer_get_time();
         
         task->status = TASK_STATUS_RUNNING;
         task->run_count++;
         
         // Call the actual task function
         if (task_func != NULL) {
             task_func(task_arg);
         }
         
         // Update execution time statistics
         uint64_t end_time = esp_timer_get_time();
         uint32_t execution_time = (uint32_t)(end_time - start_time);
         task->execution_time = execution_time;
         
         // Update average execution time
         if (task->avg_execution_time == 0) {
             task->avg_execution_time = execution_time;
         } else {
             task->avg_execution_time = (task->avg_execution_time * 7 + execution_time) / 8;
         }
         
         // Update maximum execution time
         if (execution_time > task->max_execution_time) {
             task->max_execution_time = execution_time;
         }
         
         // Update task status
         task->status = TASK_STATUS_BLOCKED;
         
         // For one-shot tasks, exit after one execution
         if (task->schedule == TASK_SCHEDULE_ONE_SHOT) {
             task->status = TASK_STATUS_DELETED;
             
             // Remove from task manager (must be careful with mutex here)
             task_manager_lock(portMAX_DELAY);
             task_manager_remove_task(task);
             task_manager_unlock();
             
             vTaskDelete(NULL);
             return;
         }
         
         // For periodic tasks, delay until next period
         if (task->schedule == TASK_SCHEDULE_PERIODIC && task->period_ticks > 0) {
             vTaskDelayUntil(&last_wake_time, task->period_ticks);
             task->last_wake_time = last_wake_time;
         }
     }
 }
 
 /**
  * @brief Task manager statistics task
  * 
  * @param arg Task argument (unused)
  */
 static void task_manager_stats_task(void *arg) {
     TickType_t last_wake_time = xTaskGetTickCount();
     uint32_t stats_period_ticks = pdMS_TO_TICKS(s_task_manager.config.stats_update_period_ms);
     uint32_t watchdog_period_ticks = pdMS_TO_TICKS(TASK_MANAGER_WATCHDOG_CHECK);
     
     while (1) {
         // Update statistics
         task_manager_update_stats();
         
         // Check watchdogs if enabled
         if (s_task_manager.config.enable_watchdog) {
             task_manager_check_watchdogs();
         }
         
         // Delay until next stats update
         vTaskDelayUntil(&last_wake_time, stats_period_ticks);
     }
 }
 
 /**
  * @brief Update task manager statistics
  * 
  * @return esp_err_t ESP_OK on success
  */
 static esp_err_t task_manager_update_stats(void) {
     esp_err_t result = ESP_OK;
     
     // Lock task manager
     result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Reset counters
     s_task_manager.stats.running_tasks = 0;
     s_task_manager.stats.blocked_tasks = 0;
     s_task_manager.stats.suspended_tasks = 0;
     
     // Update task statistics
     for (uint32_t i = 0; i < s_task_manager.task_count; i++) {
         task_handle_t *task = s_task_manager.tasks[i];
         
         if (task != NULL && task->handle != NULL) {
             // Update task status
             task_manager_update_task_status(task);
             
             // Update counters
             switch (task->status) {
                 case TASK_STATUS_RUNNING:
                     s_task_manager.stats.running_tasks++;
                     break;
                 case TASK_STATUS_BLOCKED:
                     s_task_manager.stats.blocked_tasks++;
                     break;
                 case TASK_STATUS_SUSPENDED:
                     s_task_manager.stats.suspended_tasks++;
                     break;
                 default:
                     break;
             }
             
             // Update stack high water mark
             task->high_water_mark = uxTaskGetStackHighWaterMark(task->handle);
             
             // Check for stack overflow if enabled
             if (s_task_manager.config.enable_stack_overflow_check && 
                 task->high_water_mark < 64) {  // Less than 64 bytes remaining
                 ESP_LOGW(TASK_MANAGER_TAG, "Task %s is close to stack overflow! Only %lu bytes remaining",
                         task->name, (unsigned long)task->high_water_mark);
             }
         }
     }
     
     // Update system statistics
     s_task_manager.stats.total_tasks = s_task_manager.task_count;
     s_task_manager.stats.free_heap = esp_get_free_heap_size();
     s_task_manager.stats.min_free_heap = esp_get_minimum_free_heap_size();
     
     // Unlock task manager
     task_manager_unlock();
     
     // Update timestamp
     s_task_manager.last_stats_update = esp_timer_get_time();
     
     return ESP_OK;
 }
 
 /**
  * @brief Check task watchdogs
  * 
  * @return esp_err_t ESP_OK on success
  */
 static esp_err_t task_manager_check_watchdogs(void) {
     esp_err_t result = ESP_OK;
     
     // Lock task manager
     result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Check all tasks
     for (uint32_t i = 0; i < s_task_manager.task_count; i++) {
         task_handle_t *task = s_task_manager.tasks[i];
         
         if (task != NULL && task->handle != NULL) {
             // Try to restart crashed tasks
             task_manager_restart_task_if_needed(task);
         }
     }
     
     // Unlock task manager
     task_manager_unlock();
     
     // Update timestamp
     s_task_manager.last_watchdog_check = esp_timer_get_time();
     
     return ESP_OK;
 }
 
 /**
  * @brief Add a task to the task manager
  * 
  * @param task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 static esp_err_t task_manager_add_task(task_handle_t *task) {
     if (task == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (s_task_manager.task_count >= s_task_manager.max_tasks) {
         ESP_LOGE(TASK_MANAGER_TAG, "Cannot add task: maximum number of tasks reached");
         return ESP_ERR_NO_MEM;
     }
     
     s_task_manager.tasks[s_task_manager.task_count++] = task;
     
     return ESP_OK;
 }
 
 /**
  * @brief Remove a task from the task manager
  * 
  * @param task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 static esp_err_t task_manager_remove_task(task_handle_t *task) {
     if (task == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Find the task in the array
     int32_t index = -1;
     for (uint32_t i = 0; i < s_task_manager.task_count; i++) {
         if (s_task_manager.tasks[i] == task) {
             index = i;
             break;
         }
     }
     
     if (index < 0) {
         return ESP_ERR_NOT_FOUND;
     }
     
     // Remove the task by shifting remaining tasks
     for (uint32_t i = index; i < s_task_manager.task_count - 1; i++) {
         s_task_manager.tasks[i] = s_task_manager.tasks[i + 1];
     }
     
     s_task_manager.task_count--;
     
     return ESP_OK;
 }
 
 /**
  * @brief Update task status based on FreeRTOS state
  * 
  * @param task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 static esp_err_t task_manager_update_task_status(task_handle_t *task) {
     if (task == NULL || task->handle == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     eTaskState state = eTaskGetState(task->handle);
     
     switch (state) {
         case eRunning:
             task->status = TASK_STATUS_RUNNING;
             break;
         case eReady:
             // Task is ready to run but not currently running
             break;
         case eBlocked:
             task->status = TASK_STATUS_BLOCKED;
             break;
         case eSuspended:
             task->status = TASK_STATUS_SUSPENDED;
             break;
         case eDeleted:
             task->status = TASK_STATUS_DELETED;
             break;
         default:
             break;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Restart task if it has crashed and auto-restart is enabled
  * 
  * @param task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 static esp_err_t task_manager_restart_task_if_needed(task_handle_t *task) {
     if (task == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if task has crashed (handle is NULL but status is not DELETED)
     if (task->handle == NULL && task->status != TASK_STATUS_DELETED) {
         if (task->auto_restart) {
             ESP_LOGW(TASK_MANAGER_TAG, "Task %s appears to have crashed. Attempting restart...", task->name);
             
             // Create a new task
             BaseType_t result = xTaskCreatePinnedToCore(
                 task_wrapper_function,
                 task->name,
                 task->stack_size,
                 task,
                 task->priority,
                 &task->handle,
                 task->core_id
             );
             
             if (result != pdPASS) {
                 ESP_LOGE(TASK_MANAGER_TAG, "Failed to restart task %s", task->name);
                 return ESP_FAIL;
             }
             
             s_task_manager.stats.task_crashes++;
             ESP_LOGI(TASK_MANAGER_TAG, "Task %s was successfully restarted", task->name);
             
             return ESP_OK;
         } else {
             ESP_LOGW(TASK_MANAGER_TAG, "Task %s appears to have crashed but auto-restart is disabled", task->name);
             task->status = TASK_STATUS_ERROR;
             return ESP_FAIL;
         }
     }
     
     return ESP_OK;
 }
 
 /** @} */
 
 /** @defgroup TASK_MANAGER_PUBLIC Public
  * @{
  */
 
 esp_err_t task_manager_init(const task_manager_config_t *config) {
     // Check if already initialized
     if (s_task_manager.initialized) {
         ESP_LOGW(TASK_MANAGER_TAG, "Task manager already initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (config == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid config");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Initialize state
     memset(&s_task_manager, 0, sizeof(task_manager_state_t));
     
     // Copy configuration
     memcpy(&s_task_manager.config, config, sizeof(task_manager_config_t));
     
     // Allocate task array
     s_task_manager.max_tasks = config->max_tasks;
     s_task_manager.tasks = (task_handle_t **)heap_caps_calloc(
         config->max_tasks, 
         sizeof(task_handle_t *), 
         MALLOC_CAP_DEFAULT
     );
     
     if (s_task_manager.tasks == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to allocate task array");
         return ESP_ERR_NO_MEM;
     }
     
     // Allocate synchronization object array
     s_task_manager.max_sync_objects = TASK_MANAGER_MAX_SYNC_OBJECTS;
     s_task_manager.sync_objects = (task_sync_t **)heap_caps_calloc(
         s_task_manager.max_sync_objects, 
         sizeof(task_sync_t *), 
         MALLOC_CAP_DEFAULT
     );
     
     if (s_task_manager.sync_objects == NULL) {
         heap_caps_free(s_task_manager.tasks);
         s_task_manager.tasks = NULL;
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to allocate sync objects array");
         return ESP_ERR_NO_MEM;
     }
     
     // Create manager mutex
     s_task_manager.manager_mutex = xSemaphoreCreateMutex();
     if (s_task_manager.manager_mutex == NULL) {
         heap_caps_free(s_task_manager.tasks);
         heap_caps_free(s_task_manager.sync_objects);
         s_task_manager.tasks = NULL;
         s_task_manager.sync_objects = NULL;
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to create manager mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize statistics
     memset(&s_task_manager.stats, 0, sizeof(task_stats_t));
     
     // Create statistics task if runtime stats are enabled
     if (config->enable_runtime_stats) {
         BaseType_t result = xTaskCreatePinnedToCore(
             task_manager_stats_task,
             TASK_MANAGER_STATS_TASK_NAME,
             TASK_MANAGER_STATS_STACK_SIZE,
             NULL,
             TASK_PRIORITY_LOW,
             &s_task_manager.stats_task_handle,
             tskNO_AFFINITY
         );
         
         if (result != pdPASS) {
             vSemaphoreDelete(s_task_manager.manager_mutex);
             heap_caps_free(s_task_manager.tasks);
             heap_caps_free(s_task_manager.sync_objects);
             s_task_manager.tasks = NULL;
             s_task_manager.sync_objects = NULL;
             s_task_manager.manager_mutex = NULL;
             ESP_LOGE(TASK_MANAGER_TAG, "Failed to create statistics task");
             return ESP_FAIL;
         }
     }
     
     // Set initialization flag
     s_task_manager.initialized = true;
     
     ESP_LOGI(TASK_MANAGER_TAG, "Task manager initialized successfully");
     return ESP_OK;
 }
 
 esp_err_t task_manager_deinit(void) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGW(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Delete all tasks
     for (uint32_t i = 0; i < s_task_manager.task_count; i++) {
         task_handle_t *task = s_task_manager.tasks[i];
         
         if (task != NULL) {
             if (task->handle != NULL) {
                 vTaskDelete(task->handle);
                 task->handle = NULL;
             }
             
             heap_caps_free(task);
             s_task_manager.tasks[i] = NULL;
         }
     }
     
     // Delete all synchronization objects
     for (uint32_t i = 0; i < s_task_manager.sync_count; i++) {
         task_sync_t *sync = s_task_manager.sync_objects[i];
         
         if (sync != NULL) {
             switch (sync->method) {
                 case TASK_SYNC_MUTEX:
                     if (sync->handle.mutex != NULL) {
                         vSemaphoreDelete(sync->handle.mutex);
                     }
                     break;
                 case TASK_SYNC_SEMAPHORE:
                     if (sync->handle.semaphore != NULL) {
                         vSemaphoreDelete(sync->handle.semaphore);
                     }
                     break;
                 case TASK_SYNC_EVENT_GROUP:
                     if (sync->handle.event_group != NULL) {
                         vEventGroupDelete(sync->handle.event_group);
                     }
                     break;
                 case TASK_SYNC_QUEUE:
                     if (sync->handle.queue != NULL) {
                         vQueueDelete(sync->handle.queue);
                     }
                     break;
                 default:
                     break;
             }
             
             heap_caps_free(sync);
             s_task_manager.sync_objects[i] = NULL;
         }
     }
     
     // Delete statistics task
     if (s_task_manager.stats_task_handle != NULL) {
         vTaskDelete(s_task_manager.stats_task_handle);
         s_task_manager.stats_task_handle = NULL;
     }
     
     // Free arrays
     heap_caps_free(s_task_manager.tasks);
     heap_caps_free(s_task_manager.sync_objects);
     s_task_manager.tasks = NULL;
     s_task_manager.sync_objects = NULL;
     
     // Unlock and delete mutex
     task_manager_unlock();
     vSemaphoreDelete(s_task_manager.manager_mutex);
     s_task_manager.manager_mutex = NULL;
     
     // Reset state
     s_task_manager.initialized = false;
     s_task_manager.task_count = 0;
     s_task_manager.sync_count = 0;
     
     ESP_LOGI(TASK_MANAGER_TAG, "Task manager deinitialized successfully");
     return ESP_OK;
 }
 
 esp_err_t task_manager_create_task(const char *name, 
                                   task_function_t func, 
                                   void *arg, 
                                   uint32_t stack_size, 
                                   task_priority_t priority,
                                   task_type_t type,
                                   UBaseType_t core_id,
                                   task_handle_t **task_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (name == NULL || func == NULL || stack_size == 0) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task creation parameters");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Check if task with same name already exists
     if (task_manager_find_task_by_name(name) != NULL) {
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Task with name %s already exists", name);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Allocate task handle
     task_handle_t *task = (task_handle_t *)heap_caps_calloc(1, sizeof(task_handle_t), MALLOC_CAP_DEFAULT);
     if (task == NULL) {
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to allocate task handle");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize task handle
     strlcpy(task->name, name, configMAX_TASK_NAME_LEN);
     task->type = type;
     task->priority = priority;
     task->status = TASK_STATUS_IDLE;
     task->schedule = TASK_SCHEDULE_EVENT_DRIVEN;
     task->stack_size = stack_size;
     task->run_count = 0;
     task->execution_time = 0;
     task->avg_execution_time = 0;
     task->max_execution_time = 0;
     task->last_wake_time = 0;
     task->period_ticks = 0;
     task->core_id = core_id;
     task->cpu_usage = 0;
     task->high_water_mark = 0;
     task->user_data = func;  // Store function pointer in user_data temporarily
     task->auto_restart = true;  // Enable auto-restart by default
     
     // Create the FreeRTOS task
     BaseType_t xResult = xTaskCreatePinnedToCore(
         task_wrapper_function,
         name,
         stack_size,
         task,
         priority,
         &task->handle,
         core_id
     );
     
     if (xResult != pdPASS) {
         heap_caps_free(task);
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to create task %s", name);
         return ESP_FAIL;
     }
     
     // Add task to manager
     result = task_manager_add_task(task);
     if (result != ESP_OK) {
         vTaskDelete(task->handle);
         heap_caps_free(task);
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to add task to manager");
         return result;
     }
     
     // Return task handle if requested
     if (task_out != NULL) {
         *task_out = task;
     }
     
     // Unlock task manager
     task_manager_unlock();
     
     ESP_LOGI(TASK_MANAGER_TAG, "Created task %s (type %d, priority %d, core %lu)",
              name, type, priority, (unsigned long)core_id);
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_create_periodic_task(const char *name,
                                            task_function_t func,
                                            void *arg,
                                            uint32_t stack_size,
                                            task_priority_t priority,
                                            task_type_t type,
                                            uint32_t period_ms,
                                            UBaseType_t core_id,
                                            task_handle_t **task_out) {
     // Create normal task first
     esp_err_t result = task_manager_create_task(
         name, func, arg, stack_size, priority, type, core_id, task_out
     );
     
     if (result != ESP_OK) {
         return result;
     }
     
     // Update task schedule and period
     if (task_out != NULL && *task_out != NULL) {
         task_handle_t *task = *task_out;
         
         // Lock task manager
         result = task_manager_lock(portMAX_DELAY);
         if (result != ESP_OK) {
             return result;
         }
         
         // Set as periodic task
         task->schedule = TASK_SCHEDULE_PERIODIC;
         task->period_ticks = pdMS_TO_TICKS(period_ms);
         
         // Unlock task manager
         task_manager_unlock();
         
         ESP_LOGI(TASK_MANAGER_TAG, "Set task %s as periodic with period %lu ms",
                  name, (unsigned long)period_ms);
     }
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_delete_task(task_handle_t *task) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Delete FreeRTOS task if it exists
     if (task->handle != NULL) {
         vTaskDelete(task->handle);
         task->handle = NULL;
     }
     
     // Remove task from manager
     result = task_manager_remove_task(task);
     if (result != ESP_OK) {
         task_manager_unlock();
         ESP_LOGW(TASK_MANAGER_TAG, "Failed to remove task from manager: %s", esp_err_to_name(result));
         return result;
     }
     
     // Copy task name for logging
     char task_name[configMAX_TASK_NAME_LEN];
     strlcpy(task_name, task->name, configMAX_TASK_NAME_LEN);
     
     // Free task handle
     heap_caps_free(task);
     
     // Unlock task manager
     task_manager_unlock();
     
     ESP_LOGI(TASK_MANAGER_TAG, "Deleted task %s", task_name);
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_suspend_task(task_handle_t *task) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL || task->handle == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Suspend FreeRTOS task
     vTaskSuspend(task->handle);
     
     // Update task status
     task->status = TASK_STATUS_SUSPENDED;
     
     // Unlock task manager
     task_manager_unlock();
     
     ESP_LOGI(TASK_MANAGER_TAG, "Suspended task %s", task->name);
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_resume_task(task_handle_t *task) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL || task->handle == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Resume FreeRTOS task
     vTaskResume(task->handle);
     
     // Unlock task manager
     task_manager_unlock();
     
     ESP_LOGI(TASK_MANAGER_TAG, "Resumed task %s", task->name);
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_get_task_by_name(const char *name, task_handle_t **task_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (name == NULL || task_out == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Find task by name
     task_handle_t *task = task_manager_find_task_by_name(name);
     
     // Unlock task manager
     task_manager_unlock();
     
     if (task == NULL) {
         return ESP_ERR_NOT_FOUND;
     }
     
     *task_out = task;
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_get_tasks_by_type(task_type_t type, task_handle_t **tasks_out, uint32_t *count) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (tasks_out == NULL || count == NULL || *count == 0) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Find tasks by type
     uint32_t found_count = 0;
     for (uint32_t i = 0; i < s_task_manager.task_count && found_count < *count; i++) {
         if (s_task_manager.tasks[i] != NULL && s_task_manager.tasks[i]->type == type) {
             tasks_out[found_count++] = s_task_manager.tasks[i];
         }
     }
     
     // Update count
     *count = found_count;
     
     // Unlock task manager
     task_manager_unlock();
     
     return (found_count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
 }
 
 esp_err_t task_manager_create_sync(const char *name, task_sync_method_t method, task_sync_t **sync_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (name == NULL || sync_out == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Check if sync object with same name already exists
     if (task_manager_find_sync_by_name(name) != NULL) {
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object with name %s already exists", name);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if maximum number of sync objects reached
     if (s_task_manager.sync_count >= s_task_manager.max_sync_objects) {
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Maximum number of sync objects reached");
         return ESP_ERR_NO_MEM;
     }
     
     // Allocate sync object
     task_sync_t *sync = (task_sync_t *)heap_caps_calloc(1, sizeof(task_sync_t), MALLOC_CAP_DEFAULT);
     if (sync == NULL) {
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Failed to allocate sync object");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize sync object
     strlcpy(sync->name, name, configMAX_TASK_NAME_LEN);
     sync->method = method;
     
     // Create FreeRTOS synchronization object
     switch (method) {
         case TASK_SYNC_MUTEX:
             sync->handle.mutex = xSemaphoreCreateMutex();
             if (sync->handle.mutex == NULL) {
                 heap_caps_free(sync);
                 task_manager_unlock();
                 ESP_LOGE(TASK_MANAGER_TAG, "Failed to create mutex");
                 return ESP_ERR_NO_MEM;
             }
             break;
             
         case TASK_SYNC_SEMAPHORE:
             sync->handle.semaphore = xSemaphoreCreateBinary();
             if (sync->handle.semaphore == NULL) {
                 heap_caps_free(sync);
                 task_manager_unlock();
                 ESP_LOGE(TASK_MANAGER_TAG, "Failed to create semaphore");
                 return ESP_ERR_NO_MEM;
             }
             break;
             
         case TASK_SYNC_EVENT_GROUP:
             sync->handle.event_group = xEventGroupCreate();
             if (sync->handle.event_group == NULL) {
                 heap_caps_free(sync);
                 task_manager_unlock();
                 ESP_LOGE(TASK_MANAGER_TAG, "Failed to create event group");
                 return ESP_ERR_NO_MEM;
             }
             break;
             
         case TASK_SYNC_QUEUE:
             // Default queue length of 10 items of 4 bytes each
             // Can be resized later if needed
             sync->handle.queue = xQueueCreate(10, 4);
             if (sync->handle.queue == NULL) {
                 heap_caps_free(sync);
                 task_manager_unlock();
                 ESP_LOGE(TASK_MANAGER_TAG, "Failed to create queue");
                 return ESP_ERR_NO_MEM;
             }
             break;
             
         default:
             heap_caps_free(sync);
             task_manager_unlock();
             ESP_LOGE(TASK_MANAGER_TAG, "Invalid synchronization method");
             return ESP_ERR_INVALID_ARG;
     }
     
     // Add sync object to manager
     s_task_manager.sync_objects[s_task_manager.sync_count++] = sync;
     
     // Set output
     *sync_out = sync;
     
     // Unlock task manager
     task_manager_unlock();
     
     ESP_LOGI(TASK_MANAGER_TAG, "Created sync object %s (method %d)", name, method);
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_delete_sync(task_sync_t *sync) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid sync object");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Find the sync object in the array
     int32_t index = -1;
     for (uint32_t i = 0; i < s_task_manager.sync_count; i++) {
         if (s_task_manager.sync_objects[i] == sync) {
             index = i;
             break;
         }
     }
     
     if (index < 0) {
         task_manager_unlock();
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object not found");
         return ESP_ERR_NOT_FOUND;
     }
     
     // Delete FreeRTOS synchronization object
     switch (sync->method) {
         case TASK_SYNC_MUTEX:
             if (sync->handle.mutex != NULL) {
                 vSemaphoreDelete(sync->handle.mutex);
             }
             break;
             
         case TASK_SYNC_SEMAPHORE:
             if (sync->handle.semaphore != NULL) {
                 vSemaphoreDelete(sync->handle.semaphore);
             }
             break;
             
         case TASK_SYNC_EVENT_GROUP:
             if (sync->handle.event_group != NULL) {
                 vEventGroupDelete(sync->handle.event_group);
             }
             break;
             
         case TASK_SYNC_QUEUE:
             if (sync->handle.queue != NULL) {
                 vQueueDelete(sync->handle.queue);
             }
             break;
             
         default:
             break;
     }
     
     // Copy sync name for logging
     char sync_name[configMAX_TASK_NAME_LEN];
     strlcpy(sync_name, sync->name, configMAX_TASK_NAME_LEN);
     
     // Free sync object
     heap_caps_free(sync);
     
     // Remove from sync objects array by shifting remaining objects
     for (uint32_t i = index; i < s_task_manager.sync_count - 1; i++) {
         s_task_manager.sync_objects[i] = s_task_manager.sync_objects[i + 1];
     }
     
     s_task_manager.sync_count--;
     
     // Unlock task manager
     task_manager_unlock();
     
     ESP_LOGI(TASK_MANAGER_TAG, "Deleted sync object %s", sync_name);
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_take_sync(task_sync_t *sync, uint32_t timeout) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid sync object");
         return ESP_ERR_INVALID_ARG;
     }
     
     TickType_t ticks = (timeout == portMAX_DELAY) ? 
                         portMAX_DELAY : pdMS_TO_TICKS(timeout);
     
     BaseType_t result = pdFALSE;
     
     // Take synchronization object
     switch (sync->method) {
         case TASK_SYNC_MUTEX:
             if (sync->handle.mutex != NULL) {
                 result = xSemaphoreTake(sync->handle.mutex, ticks);
             }
             break;
             
         case TASK_SYNC_SEMAPHORE:
             if (sync->handle.semaphore != NULL) {
                 result = xSemaphoreTake(sync->handle.semaphore, ticks);
             }
             break;
             
         default:
             ESP_LOGE(TASK_MANAGER_TAG, "Cannot take sync object of type %d", sync->method);
             return ESP_ERR_INVALID_ARG;
     }
     
     return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
 }
 
 esp_err_t task_manager_give_sync(task_sync_t *sync) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid sync object");
         return ESP_ERR_INVALID_ARG;
     }
     
     BaseType_t result = pdFALSE;
     
     // Give synchronization object
     switch (sync->method) {
         case TASK_SYNC_MUTEX:
             if (sync->handle.mutex != NULL) {
                 result = xSemaphoreGive(sync->handle.mutex);
             }
             break;
             
         case TASK_SYNC_SEMAPHORE:
             if (sync->handle.semaphore != NULL) {
                 result = xSemaphoreGive(sync->handle.semaphore);
             }
             break;
             
         default:
             ESP_LOGE(TASK_MANAGER_TAG, "Cannot give sync object of type %d", sync->method);
             return ESP_ERR_INVALID_ARG;
     }
     
     return (result == pdTRUE) ? ESP_OK : ESP_FAIL;
 }
 
 esp_err_t task_manager_wait_event(task_sync_t *sync, 
                                  EventBits_t bits_to_wait,
                                  bool clear_on_exit,
                                  bool wait_for_all,
                                  uint32_t timeout,
                                  EventBits_t *bits_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid sync object");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check sync type
     if (sync->method != TASK_SYNC_EVENT_GROUP) {
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object is not an event group");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (sync->handle.event_group == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Event group handle is NULL");
         return ESP_ERR_INVALID_STATE;
     }
     
     TickType_t ticks = (timeout == portMAX_DELAY) ? 
                         portMAX_DELAY : pdMS_TO_TICKS(timeout);
     
     // Wait for events
     EventBits_t bits = xEventGroupWaitBits(
         sync->handle.event_group,
         bits_to_wait,
         clear_on_exit ? pdTRUE : pdFALSE,
         wait_for_all ? pdTRUE : pdFALSE,
         ticks
     );
     
     // Check if any of the requested bits were set
     bool bits_set = wait_for_all ? 
                     ((bits & bits_to_wait) == bits_to_wait) : 
                     ((bits & bits_to_wait) != 0);
     
     // Store bits if requested
     if (bits_out != NULL) {
         *bits_out = bits;
     }
     
     return bits_set ? ESP_OK : ESP_ERR_TIMEOUT;
 }
 
 esp_err_t task_manager_set_event(task_sync_t *sync, EventBits_t bits_to_set, EventBits_t *bits_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid sync object");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check sync type
     if (sync->method != TASK_SYNC_EVENT_GROUP) {
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object is not an event group");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (sync->handle.event_group == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Event group handle is NULL");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set events
     EventBits_t bits = xEventGroupSetBits(sync->handle.event_group, bits_to_set);
     
     // Store bits if requested
     if (bits_out != NULL) {
         *bits_out = bits;
     }
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_clear_event(task_sync_t *sync, EventBits_t bits_to_clear, EventBits_t *bits_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid sync object");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check sync type
     if (sync->method != TASK_SYNC_EVENT_GROUP) {
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object is not an event group");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (sync->handle.event_group == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Event group handle is NULL");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Clear events
     EventBits_t bits = xEventGroupClearBits(sync->handle.event_group, bits_to_clear);
     
     // Store bits if requested
     if (bits_out != NULL) {
         *bits_out = bits;
     }
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_send_to_queue(task_sync_t *sync, const void *data, uint32_t timeout) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL || data == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check sync type
     if (sync->method != TASK_SYNC_QUEUE) {
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object is not a queue");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (sync->handle.queue == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Queue handle is NULL");
         return ESP_ERR_INVALID_STATE;
     }
     
     TickType_t ticks = (timeout == portMAX_DELAY) ? 
                         portMAX_DELAY : pdMS_TO_TICKS(timeout);
     
     // Send to queue
     BaseType_t result = xQueueSend(sync->handle.queue, data, ticks);
     
     return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
 }
 
 esp_err_t task_manager_receive_from_queue(task_sync_t *sync, void *data, uint32_t timeout) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (sync == NULL || data == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check sync type
     if (sync->method != TASK_SYNC_QUEUE) {
         ESP_LOGE(TASK_MANAGER_TAG, "Sync object is not a queue");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (sync->handle.queue == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Queue handle is NULL");
         return ESP_ERR_INVALID_STATE;
     }
     
     TickType_t ticks = (timeout == portMAX_DELAY) ? 
                         portMAX_DELAY : pdMS_TO_TICKS(timeout);
     
     // Receive from queue
     BaseType_t result = xQueueReceive(sync->handle.queue, data, ticks);
     
     return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
 }
 
 esp_err_t task_manager_get_stats(task_stats_t *stats) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (stats == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Copy statistics
     memcpy(stats, &s_task_manager.stats, sizeof(task_stats_t));
     
     // Unlock task manager
     task_manager_unlock();
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_get_task_info(task_handle_t *task, task_handle_t *info) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL || info == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Update task status
     task_manager_update_task_status(task);
     
     // Update stack high water mark
     if (task->handle != NULL) {
         task->high_water_mark = uxTaskGetStackHighWaterMark(task->handle);
     }
     
     // Copy task info
     memcpy(info, task, sizeof(task_handle_t));
     
     // Unlock task manager
     task_manager_unlock();
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_set_task_user_data(task_handle_t *task, void *user_data) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Set user data
     task->user_data = user_data;
     
     // Unlock task manager
     task_manager_unlock();
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_get_task_user_data(task_handle_t *task, void **user_data_out) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL || user_data_out == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Get user data
     *user_data_out = task->user_data;
     
     // Unlock task manager
     task_manager_unlock();
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_reset_watchdog(task_handle_t *task) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if watchdog is enabled
     if (!s_task_manager.config.enable_watchdog) {
         return ESP_OK;  // Nothing to do
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Update task last wake time (used for watchdog)
     task->last_wake_time = xTaskGetTickCount();
     
     // Unlock task manager
     task_manager_unlock();
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_dump_info(bool include_tasks) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Lock task manager
     esp_err_t result = task_manager_lock(portMAX_DELAY);
     if (result != ESP_OK) {
         return result;
     }
     
     // Print general statistics
     ESP_LOGI(TASK_MANAGER_TAG, "===== Task Manager Status =====");
     ESP_LOGI(TASK_MANAGER_TAG, "Total tasks: %lu", (unsigned long)s_task_manager.stats.total_tasks);
     ESP_LOGI(TASK_MANAGER_TAG, "Running tasks: %lu", (unsigned long)s_task_manager.stats.running_tasks);
     ESP_LOGI(TASK_MANAGER_TAG, "Blocked tasks: %lu", (unsigned long)s_task_manager.stats.blocked_tasks);
     ESP_LOGI(TASK_MANAGER_TAG, "Suspended tasks: %lu", (unsigned long)s_task_manager.stats.suspended_tasks);
     ESP_LOGI(TASK_MANAGER_TAG, "Free heap: %lu bytes", (unsigned long)s_task_manager.stats.free_heap);
     ESP_LOGI(TASK_MANAGER_TAG, "Minimum free heap: %lu bytes", (unsigned long)s_task_manager.stats.min_free_heap);
     ESP_LOGI(TASK_MANAGER_TAG, "Task crashes: %lu", (unsigned long)s_task_manager.stats.task_crashes);
     ESP_LOGI(TASK_MANAGER_TAG, "Watchdog resets: %lu", (unsigned long)s_task_manager.stats.watchdog_resets);
     
     // Print task details if requested
     if (include_tasks) {
         ESP_LOGI(TASK_MANAGER_TAG, "===== Task Details =====");
         
         for (uint32_t i = 0; i < s_task_manager.task_count; i++) {
             task_handle_t *task = s_task_manager.tasks[i];
             
             if (task != NULL) {
                 // Update task status
                 task_manager_update_task_status(task);
                 
                 // Update stack high water mark
                 if (task->handle != NULL) {
                     task->high_water_mark = uxTaskGetStackHighWaterMark(task->handle);
                 }
                 
                 const char *status_str = "Unknown";
                 switch (task->status) {
                     case TASK_STATUS_IDLE:     status_str = "Idle"; break;
                     case TASK_STATUS_RUNNING:  status_str = "Running"; break;
                     case TASK_STATUS_SUSPENDED:status_str = "Suspended"; break;
                     case TASK_STATUS_BLOCKED:  status_str = "Blocked"; break;
                     case TASK_STATUS_DELETED:  status_str = "Deleted"; break;
                     case TASK_STATUS_ERROR:    status_str = "Error"; break;
                     default: break;
                 }
                 
                 ESP_LOGI(TASK_MANAGER_TAG, "Task %s (type %d, priority %d):", 
                          task->name, task->type, task->priority);
                 ESP_LOGI(TASK_MANAGER_TAG, "  Status: %s", status_str);
                 ESP_LOGI(TASK_MANAGER_TAG, "  Stack: %lu/%lu bytes (high water: %lu bytes)",
                          (unsigned long)(task->stack_size - task->high_water_mark),
                          (unsigned long)task->stack_size,
                          (unsigned long)task->high_water_mark);
                 ESP_LOGI(TASK_MANAGER_TAG, "  Run count: %lu", (unsigned long)task->run_count);
                 ESP_LOGI(TASK_MANAGER_TAG, "  Execution time: %lu us (avg: %lu us, max: %lu us)",
                          (unsigned long)task->execution_time,
                          (unsigned long)task->avg_execution_time,
                          (unsigned long)task->max_execution_time);
                 
                 if (task->schedule == TASK_SCHEDULE_PERIODIC) {
                     ESP_LOGI(TASK_MANAGER_TAG, "  Period: %lu ms",
                              (unsigned long)(task->period_ticks * portTICK_PERIOD_MS));
                 }
             }
         }
     }
     
     // Print synchronization object details
     ESP_LOGI(TASK_MANAGER_TAG, "===== Sync Objects =====");
     ESP_LOGI(TASK_MANAGER_TAG, "Total sync objects: %lu", (unsigned long)s_task_manager.sync_count);
     
     for (uint32_t i = 0; i < s_task_manager.sync_count; i++) {
         task_sync_t *sync = s_task_manager.sync_objects[i];
         
         if (sync != NULL) {
             const char *method_str = "Unknown";
             switch (sync->method) {
                 case TASK_SYNC_MUTEX:      method_str = "Mutex"; break;
                 case TASK_SYNC_SEMAPHORE:  method_str = "Semaphore"; break;
                 case TASK_SYNC_EVENT_GROUP:method_str = "Event Group"; break;
                 case TASK_SYNC_QUEUE:      method_str = "Queue"; break;
                 default: break;
             }
             
             ESP_LOGI(TASK_MANAGER_TAG, "  %s (%s)", sync->name, method_str);
         }
     }
     
     // Unlock task manager
     task_manager_unlock();
     
     return ESP_OK;
 }
 
 esp_err_t task_manager_notify_task(task_handle_t *task, uint32_t value, eNotifyAction action) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check arguments
     if (task == NULL || task->handle == NULL) {
         ESP_LOGE(TASK_MANAGER_TAG, "Invalid task handle");
         return ESP_ERR_INVALID_ARG;
     }
     
     // Send notification
     BaseType_t result = xTaskNotify(task->handle, value, action);
     
     return (result == pdPASS) ? ESP_OK : ESP_FAIL;
 }
 
 esp_err_t task_manager_wait_notification(uint32_t timeout, uint32_t *notification_value, bool clear_on_exit) {
     // Check if initialized
     if (!s_task_manager.initialized) {
         ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     TickType_t ticks = (timeout == portMAX_DELAY) ? 
                         portMAX_DELAY : pdMS_TO_TICKS(timeout);
     
     // Wait for notification
     BaseType_t result = xTaskNotifyWait(
         0,                        // Don't clear any bits on entry
         clear_on_exit ? 0xFFFFFFFF : 0, // Clear all bits on exit if requested
         notification_value,       // Store notification value if pointer provided
         ticks                     // Timeout
     );
     
     return (result == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
 }
 
esp_err_t task_manager_get_notification_value(task_handle_t *task, uint32_t *notification_value) {
    // Check if initialized
    if (!s_task_manager.initialized) {
        ESP_LOGE(TASK_MANAGER_TAG, "Task manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check arguments
    if (task == NULL || notification_value == NULL) {
        ESP_LOGE(TASK_MANAGER_TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get notification value
    BaseType_t result = xTaskNotifyStateClear(task->handle);
    
    return (result == pdTRUE) ? ESP_OK : ESP_FAIL;
}
