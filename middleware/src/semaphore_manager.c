/**
 * @file semaphore_manager.c
 * @brief Implementation of centralized semaphore management for BESS firmware
 * 
 * This module provides a centralized system for creating, managing, and using
 * semaphores across the BESS firmware for the 100KW/200KWH system with LFP
 * battery modules (48V, 16KWH). It ensures proper synchronization between
 * multiple FreeRTOS tasks.
 * 
 * @copyright Copyright (c) 2025
 */

 #include "semaphore_manager.h"
 #include <string.h>
 #include "esp_system.h"
 #include "freertos/task.h"
 
 /** @brief Tag for ESP logging */
 static const char *TAG = SEMAPHORE_TAG;
 
 /**
  * @brief Structure to store information about a managed semaphore
  */
 typedef struct {
     bool in_use;                     /**< Whether this slot is in use */
     char name[32];                   /**< Name of the semaphore */
     semaphore_type_t type;           /**< Type of semaphore */
     SemaphoreHandle_t handle;        /**< FreeRTOS semaphore handle */
     uint32_t initial_count;          /**< Initial count for counting semaphores */
     uint32_t max_count;              /**< Maximum count for counting semaphores */
     uint32_t access_count;           /**< Number of times semaphore has been accessed */
     uint32_t timeout_count;          /**< Number of timeouts that occurred */
     TickType_t last_taken_tick;      /**< When the semaphore was last taken */
     bool is_taken;                   /**< Whether the semaphore is currently taken */
     uint32_t waiting_tasks;          /**< Estimated number of waiting tasks */
 } semaphore_data_t;
 
 /** @brief Array of managed semaphores */
 static semaphore_data_t s_semaphores[SEMAPHORE_MAX_COUNT];
 
 /** @brief Mutex to protect access to the semaphore manager itself */
 static SemaphoreHandle_t s_manager_mutex = NULL;
 
 /** @brief Whether the semaphore manager has been initialized */
 static bool s_initialized = false;
 
 /**
  * @brief Check if a semaphore handle is valid
  * 
  * @param handle The semaphore handle to check
  * @return true if valid, false otherwise
  */
 static bool is_valid_handle(semaphore_handle_t handle) {
     return (handle >= 0 && handle < SEMAPHORE_MAX_COUNT && s_semaphores[handle].in_use);
 }
 
 /**
  * @brief Find a free slot in the semaphore array
  * 
  * @param index Pointer to store the found index
  * @return ESP_OK if a free slot was found, otherwise an error code
  */
 static esp_err_t find_free_slot(semaphore_handle_t *index) {
     if (index == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     for (int i = 0; i < SEMAPHORE_MAX_COUNT; i++) {
         if (!s_semaphores[i].in_use) {
             *index = i;
             return ESP_OK;
         }
     }
 
     return SEMAPHORE_ERR_OUT_OF_RESOURCES;
 }
 
 /**
  * @brief Find a semaphore by name
  * 
  * @param name The name to search for
  * @param index Pointer to store the found index
  * @return ESP_OK if found, SEMAPHORE_ERR_NOT_FOUND if not found
  */
 static esp_err_t find_by_name(const char *name, semaphore_handle_t *index) {
     if (name == NULL || index == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     for (int i = 0; i < SEMAPHORE_MAX_COUNT; i++) {
         if (s_semaphores[i].in_use && strcmp(s_semaphores[i].name, name) == 0) {
             *index = i;
             return ESP_OK;
         }
     }
 
     return SEMAPHORE_ERR_NOT_FOUND;
 }
 
 esp_err_t semaphore_manager_init(void) {
     if (s_initialized) {
         ESP_LOGW(TAG, "Semaphore manager already initialized");
         return ESP_OK;
     }
 
     // Clear the semaphore array
     memset(s_semaphores, 0, sizeof(s_semaphores));
 
     // Create the manager mutex
     s_manager_mutex = xSemaphoreCreateMutex();
     if (s_manager_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create manager mutex");
         return ESP_FAIL;
     }
 
     s_initialized = true;
     ESP_LOGI(TAG, "Semaphore manager initialized");
     return ESP_OK;
 }
 
 esp_err_t semaphore_manager_deinit(void) {
     if (!s_initialized) {
         ESP_LOGW(TAG, "Semaphore manager not initialized");
         return ESP_OK;
     }
 
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex for deinit");
         return ESP_FAIL;
     }
 
     // Delete all managed semaphores
     for (int i = 0; i < SEMAPHORE_MAX_COUNT; i++) {
         if (s_semaphores[i].in_use && s_semaphores[i].handle != NULL) {
             vSemaphoreDelete(s_semaphores[i].handle);
             s_semaphores[i].in_use = false;
             s_semaphores[i].handle = NULL;
         }
     }
 
     // Release and delete the manager mutex
     xSemaphoreGive(s_manager_mutex);
     vSemaphoreDelete(s_manager_mutex);
     s_manager_mutex = NULL;
 
     s_initialized = false;
     ESP_LOGI(TAG, "Semaphore manager deinitialized");
     return ESP_OK;
 }
 
 esp_err_t semaphore_create(const char *name, 
                           semaphore_type_t type,
                           uint32_t initial_count,
                           uint32_t max_count,
                           semaphore_handle_t *handle) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (name == NULL || handle == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Check if name is too long
     if (strlen(name) >= sizeof(s_semaphores[0].name)) {
         ESP_LOGE(TAG, "Semaphore name too long: %s", name);
         return ESP_ERR_INVALID_ARG;
     }
 
     // Take the manager mutex
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex");
         return SEMAPHORE_ERR_TIMEOUT;
     }
 
     // Check if semaphore with this name already exists
     semaphore_handle_t existing_idx;
     if (find_by_name(name, &existing_idx) == ESP_OK) {
         xSemaphoreGive(s_manager_mutex);
         ESP_LOGE(TAG, "Semaphore with name '%s' already exists", name);
         return SEMAPHORE_ERR_ALREADY_EXISTS;
     }
 
     // Find a free slot
     semaphore_handle_t idx;
     esp_err_t err = find_free_slot(&idx);
     if (err != ESP_OK) {
         xSemaphoreGive(s_manager_mutex);
         ESP_LOGE(TAG, "No free semaphore slots available");
         return err;
     }
 
     // Create the semaphore
     SemaphoreHandle_t sem_handle;
     switch (type) {
         case SEMAPHORE_TYPE_MUTEX:
             sem_handle = xSemaphoreCreateMutex();
             break;
 
         case SEMAPHORE_TYPE_RECURSIVE_MUTEX:
             sem_handle = xSemaphoreCreateRecursiveMutex();
             break;
 
         case SEMAPHORE_TYPE_BINARY:
             sem_handle = xSemaphoreCreateBinary();
             // xSemaphoreCreateBinary creates in "taken" state, so give it
             if (sem_handle) {
                 xSemaphoreGive(sem_handle);
             }
             break;
 
         case SEMAPHORE_TYPE_COUNTING:
             sem_handle = xSemaphoreCreateCounting(max_count, initial_count);
             break;
 
         default:
             xSemaphoreGive(s_manager_mutex);
             ESP_LOGE(TAG, "Invalid semaphore type: %d", type);
             return ESP_ERR_INVALID_ARG;
     }
 
     if (sem_handle == NULL) {
         xSemaphoreGive(s_manager_mutex);
         ESP_LOGE(TAG, "Failed to create semaphore of type %d", type);
         return ESP_FAIL;
     }
 
     // Initialize the semaphore data
     strncpy(s_semaphores[idx].name, name, sizeof(s_semaphores[idx].name) - 1);
     s_semaphores[idx].name[sizeof(s_semaphores[idx].name) - 1] = '\0';
     s_semaphores[idx].type = type;
     s_semaphores[idx].handle = sem_handle;
     s_semaphores[idx].initial_count = initial_count;
     s_semaphores[idx].max_count = max_count;
     s_semaphores[idx].access_count = 0;
     s_semaphores[idx].timeout_count = 0;
     s_semaphores[idx].last_taken_tick = 0;
     s_semaphores[idx].is_taken = false;
     s_semaphores[idx].waiting_tasks = 0;
     s_semaphores[idx].in_use = true;
 
     *handle = idx;
     xSemaphoreGive(s_manager_mutex);
     
     ESP_LOGI(TAG, "Created semaphore '%s' (type: %s, handle: %d)", 
               name, semaphore_type_to_str(type), idx);
     return ESP_OK;
 }
 
 esp_err_t semaphore_delete(semaphore_handle_t handle) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!is_valid_handle(handle)) {
         ESP_LOGE(TAG, "Invalid semaphore handle: %d", handle);
         return SEMAPHORE_ERR_INVALID_HANDLE;
     }
 
     // Take the manager mutex
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex");
         return SEMAPHORE_ERR_TIMEOUT;
     }
 
     // Delete the semaphore
     if (s_semaphores[handle].handle != NULL) {
         vSemaphoreDelete(s_semaphores[handle].handle);
         s_semaphores[handle].handle = NULL;
     }
 
     ESP_LOGI(TAG, "Deleted semaphore '%s' (handle: %d)", 
               s_semaphores[handle].name, handle);
 
     // Mark the slot as free
     s_semaphores[handle].in_use = false;
     memset(s_semaphores[handle].name, 0, sizeof(s_semaphores[handle].name));
 
     xSemaphoreGive(s_manager_mutex);
     return ESP_OK;
 }
 
 esp_err_t semaphore_take(semaphore_handle_t handle, uint32_t timeout_ms) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!is_valid_handle(handle)) {
         ESP_LOGE(TAG, "Invalid semaphore handle: %d", handle);
         return SEMAPHORE_ERR_INVALID_HANDLE;
     }
 
     // Convert timeout from ms to ticks
     TickType_t timeout_ticks = (timeout_ms == UINT32_MAX) ? 
                                 portMAX_DELAY : (timeout_ms / portTICK_PERIOD_MS);
 
     // Take the semaphore based on its type
     bool success = false;
     if (s_semaphores[handle].type == SEMAPHORE_TYPE_RECURSIVE_MUTEX) {
         success = (xSemaphoreTakeRecursive(s_semaphores[handle].handle, timeout_ticks) == pdTRUE);
     } else {
         success = (xSemaphoreTake(s_semaphores[handle].handle, timeout_ticks) == pdTRUE);
     }
 
     if (success) {
         // Update semaphore statistics
         if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) == pdTRUE) {
             s_semaphores[handle].access_count++;
             s_semaphores[handle].last_taken_tick = xTaskGetTickCount();
             s_semaphores[handle].is_taken = true;
             xSemaphoreGive(s_manager_mutex);
         }
         return ESP_OK;
     } else {
         // Update timeout statistics
         if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) == pdTRUE) {
             s_semaphores[handle].timeout_count++;
             s_semaphores[handle].waiting_tasks++;
             xSemaphoreGive(s_manager_mutex);
         }
         
         ESP_LOGW(TAG, "Timeout waiting for semaphore '%s' (handle: %d)", 
                  s_semaphores[handle].name, handle);
         return SEMAPHORE_ERR_TIMEOUT;
     }
 }
 
 esp_err_t semaphore_give(semaphore_handle_t handle) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!is_valid_handle(handle)) {
         ESP_LOGE(TAG, "Invalid semaphore handle: %d", handle);
         return SEMAPHORE_ERR_INVALID_HANDLE;
     }
 
     // Give the semaphore based on its type
     bool success = false;
     if (s_semaphores[handle].type == SEMAPHORE_TYPE_RECURSIVE_MUTEX) {
         success = (xSemaphoreGiveRecursive(s_semaphores[handle].handle) == pdTRUE);
     } else {
         success = (xSemaphoreGive(s_semaphores[handle].handle) == pdTRUE);
     }
 
     if (success) {
         // Update semaphore statistics
         if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) == pdTRUE) {
             s_semaphores[handle].is_taken = false;
             // Decrement waiting tasks if any (may not be accurate but a best guess)
             if (s_semaphores[handle].waiting_tasks > 0) {
                 s_semaphores[handle].waiting_tasks--;
             }
             xSemaphoreGive(s_manager_mutex);
         }
         return ESP_OK;
     } else {
         ESP_LOGE(TAG, "Failed to give semaphore '%s' (handle: %d)", 
                  s_semaphores[handle].name, handle);
         return SEMAPHORE_ERR_INTERNAL;
     }
 }
 
 esp_err_t semaphore_find_by_name(const char *name, semaphore_handle_t *handle) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (name == NULL || handle == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Take the manager mutex
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex");
         return SEMAPHORE_ERR_TIMEOUT;
     }
 
     esp_err_t result = find_by_name(name, handle);
     xSemaphoreGive(s_manager_mutex);
 
     return result;
 }
 
 esp_err_t semaphore_get_info(semaphore_handle_t handle, semaphore_info_t *info) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!is_valid_handle(handle) || info == NULL) {
         return handle < 0 ? SEMAPHORE_ERR_INVALID_HANDLE : ESP_ERR_INVALID_ARG;
     }
 
     // Take the manager mutex
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex");
         return SEMAPHORE_ERR_TIMEOUT;
     }
 
     strncpy(info->name, s_semaphores[handle].name, sizeof(info->name));
     info->type = s_semaphores[handle].type;
     info->max_count = s_semaphores[handle].max_count;
     info->is_taken = s_semaphores[handle].is_taken;
     info->last_taken_tick = s_semaphores[handle].last_taken_tick;
     info->access_count = s_semaphores[handle].access_count;
     info->timeout_count = s_semaphores[handle].timeout_count;
     info->waiting_tasks = s_semaphores[handle].waiting_tasks;
 
     // For counting semaphores, get the current count
     if (info->type == SEMAPHORE_TYPE_COUNTING) {
         // We can only estimate the count - this is not atomic
         UBaseType_t count = uxSemaphoreGetCount(s_semaphores[handle].handle);
         info->current_count = count;
     } else if (info->type == SEMAPHORE_TYPE_BINARY) {
         // For binary semaphores, 1 = available, 0 = taken
         UBaseType_t count = uxSemaphoreGetCount(s_semaphores[handle].handle);
         info->current_count = count;
     } else {
         // For mutexes, we can't reliably get the count
         info->current_count = s_semaphores[handle].is_taken ? 0 : 1;
     }
 
     xSemaphoreGive(s_manager_mutex);
     return ESP_OK;
 }
 
 esp_err_t semaphore_reset(semaphore_handle_t handle) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!is_valid_handle(handle)) {
         ESP_LOGE(TAG, "Invalid semaphore handle: %d", handle);
         return SEMAPHORE_ERR_INVALID_HANDLE;
     }
 
     // Take the manager mutex
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex");
         return SEMAPHORE_ERR_TIMEOUT;
     }
 
     ESP_LOGI(TAG, "Resetting semaphore '%s' (handle: %d)", 
               s_semaphores[handle].name, handle);
 
     // Delete the old semaphore
     if (s_semaphores[handle].handle != NULL) {
         vSemaphoreDelete(s_semaphores[handle].handle);
     }
 
     // Create a new semaphore of the same type
     SemaphoreHandle_t sem_handle;
     switch (s_semaphores[handle].type) {
         case SEMAPHORE_TYPE_MUTEX:
             sem_handle = xSemaphoreCreateMutex();
             break;
 
         case SEMAPHORE_TYPE_RECURSIVE_MUTEX:
             sem_handle = xSemaphoreCreateRecursiveMutex();
             break;
 
         case SEMAPHORE_TYPE_BINARY:
             sem_handle = xSemaphoreCreateBinary();
             // xSemaphoreCreateBinary creates in "taken" state, so give it
             if (sem_handle) {
                 xSemaphoreGive(sem_handle);
             }
             break;
 
         case SEMAPHORE_TYPE_COUNTING:
             sem_handle = xSemaphoreCreateCounting(
                 s_semaphores[handle].max_count,
                 s_semaphores[handle].initial_count
             );
             break;
 
         default:
             xSemaphoreGive(s_manager_mutex);
             return ESP_ERR_INVALID_ARG;
     }
 
     if (sem_handle == NULL) {
         xSemaphoreGive(s_manager_mutex);
         ESP_LOGE(TAG, "Failed to recreate semaphore");
         return ESP_FAIL;
     }
 
     // Update the semaphore data
     s_semaphores[handle].handle = sem_handle;
     s_semaphores[handle].is_taken = false;
     s_semaphores[handle].waiting_tasks = 0;
     s_semaphores[handle].last_taken_tick = 0;
     
     // Keep access and timeout counts for debugging purposes
 
     xSemaphoreGive(s_manager_mutex);
     return ESP_OK;
 }
 
 esp_err_t semaphore_is_available(semaphore_handle_t handle, bool *is_available) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (!is_valid_handle(handle) || is_available == NULL) {
         return handle < 0 ? SEMAPHORE_ERR_INVALID_HANDLE : ESP_ERR_INVALID_ARG;
     }
 
     semaphore_info_t info;
     esp_err_t result = semaphore_get_info(handle, &info);
     if (result != ESP_OK) {
         return result;
     }
 
     // Determine availability based on semaphore type
     switch (info.type) {
         case SEMAPHORE_TYPE_MUTEX:
         case SEMAPHORE_TYPE_RECURSIVE_MUTEX:
         case SEMAPHORE_TYPE_BINARY:
             *is_available = !info.is_taken;
             break;
 
         case SEMAPHORE_TYPE_COUNTING:
             *is_available = (info.current_count > 0);
             break;
 
         default:
             return ESP_ERR_INVALID_ARG;
     }
 
     return ESP_OK;
 }
 
 esp_err_t semaphore_dump_status(esp_log_level_t log_level) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     // Take the manager mutex
     if (xSemaphoreTake(s_manager_mutex, SEMAPHORE_DEFAULT_WAIT_TICKS) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take manager mutex");
         return SEMAPHORE_ERR_TIMEOUT;
     }
 
     ESP_LOG_LEVEL(log_level, TAG, "======= Semaphore Manager Status =======");
     ESP_LOG_LEVEL(log_level, TAG, "Total semaphores: %d", SEMAPHORE_MAX_COUNT);
 
     int in_use_count = 0;
     for (int i = 0; i < SEMAPHORE_MAX_COUNT; i++) {
         if (s_semaphores[i].in_use) {
             in_use_count++;
             const char *state;
             if (s_semaphores[i].type == SEMAPHORE_TYPE_COUNTING) {
                 UBaseType_t count = uxSemaphoreGetCount(s_semaphores[i].handle);
                 ESP_LOG_LEVEL(log_level, TAG, "[%d] %s (%s): Count=%u/%u, Access=%u, Timeouts=%u, Waiting=%u",
                     i, s_semaphores[i].name, semaphore_type_to_str(s_semaphores[i].type),
                     count, s_semaphores[i].max_count,
                     s_semaphores[i].access_count, s_semaphores[i].timeout_count,
                     s_semaphores[i].waiting_tasks);
             } else {
                 state = s_semaphores[i].is_taken ? "Taken" : "Available";
                 ESP_LOG_LEVEL(log_level, TAG, "[%d] %s (%s): %s, Access=%u, Timeouts=%u, Waiting=%u",
                     i, s_semaphores[i].name, semaphore_type_to_str(s_semaphores[i].type),
                     state, s_semaphores[i].access_count, s_semaphores[i].timeout_count,
                     s_semaphores[i].waiting_tasks);
             }
         }
     }
 
     ESP_LOG_LEVEL(log_level, TAG, "Semaphores in use: %d/%d", in_use_count, SEMAPHORE_MAX_COUNT);
     ESP_LOG_LEVEL(log_level, TAG, "=========================================");
 
     xSemaphoreGive(s_manager_mutex);
     return ESP_OK;
 }
 
 esp_err_t semaphore_create_system_defaults(void) {
     if (!s_initialized) {
         ESP_LOGE(TAG, "Semaphore manager not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     ESP_LOGI(TAG, "Creating default system semaphores");
 
     // Create default BMS semaphores
     semaphore_handle_t handle;
     esp_err_t err;
 
     // Battery Manager mutex
     err = semaphore_create("battery_manager_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create battery_manager_mutex: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // SoC Calculator mutex
     err = semaphore_create("soc_calculator_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create soc_calculator_mutex: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // Cell Balancer mutex
     err = semaphore_create("cell_balancer_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create cell_balancer_mutex: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // Thermal Monitor mutex
     err = semaphore_create("thermal_monitor_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create thermal_monitor_mutex: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // Communication mutex
     err = semaphore_create("communication_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create communication_mutex: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // Log mutex
     err = semaphore_create("log_mutex", SEMAPHORE_TYPE_MUTEX, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create log_mutex: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // Create a binary semaphore for synchronization events
     err = semaphore_create("sync_event", SEMAPHORE_TYPE_BINARY, 0, 1, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create sync_event: %s", semaphore_err_to_str(err));
         return err;
     }
 
     // Create a counting semaphore for resource management
     err = semaphore_create("resource_count", SEMAPHORE_TYPE_COUNTING, 4, 4, &handle);
     if (err != ESP_OK && err != SEMAPHORE_ERR_ALREADY_EXISTS) {
         ESP_LOGE(TAG, "Failed to create resource_count: %s", semaphore_err_to_str(err));
         return err;
     }
 
     ESP_LOGI(TAG, "Default system semaphores created successfully");
     return ESP_OK;
 }
 
 const char *semaphore_err_to_str(semaphore_err_t err) {
     switch (err) {
         case SEMAPHORE_OK:
             return "SEMAPHORE_OK";
         case SEMAPHORE_ERR_INVALID_HANDLE:
             return "SEMAPHORE_ERR_INVALID_HANDLE";
         case SEMAPHORE_ERR_OUT_OF_RESOURCES:
             return "SEMAPHORE_ERR_OUT_OF_RESOURCES";
         case SEMAPHORE_ERR_ALREADY_EXISTS:
             return "SEMAPHORE_ERR_ALREADY_EXISTS";
         case SEMAPHORE_ERR_NOT_FOUND:
             return "SEMAPHORE_ERR_NOT_FOUND";
         case SEMAPHORE_ERR_TIMEOUT:
             return "SEMAPHORE_ERR_TIMEOUT";
         case SEMAPHORE_ERR_INTERNAL:
             return "SEMAPHORE_ERR_INTERNAL";
         default:
             return "UNKNOWN_SEMAPHORE_ERROR";
     }
 }
 
 const char *semaphore_type_to_str(semaphore_type_t type) {
     switch (type) {
         case SEMAPHORE_TYPE_MUTEX:
             return "MUTEX";
         case SEMAPHORE_TYPE_BINARY:
             return "BINARY";
         case SEMAPHORE_TYPE_COUNTING:
             return "COUNTING";
         case SEMAPHORE_TYPE_RECURSIVE_MUTEX:
             return "RECURSIVE_MUTEX";
         default:
             return "UNKNOWN_TYPE";
     }
 }