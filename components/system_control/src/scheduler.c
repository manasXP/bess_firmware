/**
 * @file scheduler.c
 * @brief Implementation of task scheduler for BESS 100KW/200KWH main controller
 * 
 * This module handles task scheduling and management for the Battery Energy
 * Storage System (BESS) controller firmware. It abstracts FreeRTOS task creation
 * and management, providing a simplified interface for other components.
 * 
 * @copyright Copyright (c) 2025
 */

#include "scheduler.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define TAG "BESS_SCHEDULER"
#define MAX_TASKS 32  // Maximum number of tasks the scheduler can manage
#define CPU_STATS_UPDATE_INTERVAL_MS 1000  // Interval to update CPU stats

// Structure to manage all scheduler tasks
typedef struct {
    scheduler_task_t *tasks[MAX_TASKS];    // Array of task handles
    uint32_t task_count;                   // Number of registered tasks
    SemaphoreHandle_t mutex;               // Mutex for thread-safe operations
    bool initialized;                       // Initialization flag
    TaskHandle_t cpu_stats_task_handle;    // Task handle for CPU statistics
    float cpu_utilization[portNUM_PROCESSORS]; // CPU utilization percentages
    uint64_t last_idle_ticks[portNUM_PROCESSORS]; // Last recorded idle ticks
    uint64_t last_total_ticks[portNUM_PROCESSORS]; // Last recorded total ticks
} scheduler_context_t;

// Static scheduler context
static scheduler_context_t s_scheduler_ctx = {0};

// Forward declarations of internal functions
static void cpu_stats_task(void *arg);
static void get_cpu_ticks(uint64_t *idle_ticks, uint64_t *total_ticks, uint8_t core_id);
static bool is_task_valid(scheduler_task_t *task_handle);

esp_err_t scheduler_init(void) {
    ESP_LOGI(TAG, "Initializing task scheduler");
    
    // Check if already initialized
    if (s_scheduler_ctx.initialized) {
        ESP_LOGW(TAG, "Scheduler already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize context
    memset(&s_scheduler_ctx, 0, sizeof(scheduler_context_t));
    
    // Create mutex for thread safety
    s_scheduler_ctx.mutex = xSemaphoreCreateMutex();
    if (s_scheduler_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create scheduler mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize CPU statistics
    for (int i = 0; i < portNUM_PROCESSORS; i++) {
        s_scheduler_ctx.cpu_utilization[i] = 0.0f;
        s_scheduler_ctx.last_idle_ticks[i] = 0;
        s_scheduler_ctx.last_total_ticks[i] = 0;
    }
    
    // Create CPU statistics monitoring task
    BaseType_t result = xTaskCreatePinnedToCore(
        cpu_stats_task,
        "cpu_stats",
        2048,
        NULL,
        SCHEDULER_PRIORITY_LOW,
        &s_scheduler_ctx.cpu_stats_task_handle,
        SCHEDULER_CORE_0
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CPU stats task");
        vSemaphoreDelete(s_scheduler_ctx.mutex);
        return ESP_ERR_NO_MEM;
    }
    
    s_scheduler_ctx.initialized = true;
    ESP_LOGI(TAG, "Scheduler initialized successfully");
    return ESP_OK;
}

esp_err_t scheduler_create_task(
    scheduler_task_func_t task_func,
    const char *name,
    uint32_t stack_size,
    void *arg,
    scheduler_priority_t priority,
    scheduler_core_t core_id,
    scheduler_task_t **task_handle
) {
    // Validate parameters
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (task_func == NULL || name == NULL || task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (stack_size < configMINIMAL_STACK_SIZE) {
        ESP_LOGW(TAG, "Stack size too small, adjusting to minimum");
        stack_size = configMINIMAL_STACK_SIZE;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if we have room for another task
    if (s_scheduler_ctx.task_count >= MAX_TASKS) {
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Maximum task count reached");
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate task structure
    scheduler_task_t *new_task = (scheduler_task_t *)malloc(sizeof(scheduler_task_t));
    if (new_task == NULL) {
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Failed to allocate memory for task");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize task structure
    new_task->name = strdup(name);  // Create a copy of the name
    if (new_task->name == NULL) {
        free(new_task);
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Failed to allocate memory for task name");
        return ESP_ERR_NO_MEM;
    }
    
    new_task->stack_size = stack_size;
    new_task->priority = priority;
    new_task->core_id = core_id;
    new_task->is_running = false;
    new_task->handle = NULL;
    
    // Create the FreeRTOS task but don't start it yet
    BaseType_t result = xTaskCreatePinnedToCore(
        task_func,
        name,
        stack_size,
        arg,
        (UBaseType_t)priority,
        &new_task->handle,
        (int)core_id
    );
    
    if (result != pdPASS) {
        free((void*)new_task->name);
        free(new_task);
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Failed to create task: %s", name);
        return ESP_ERR_NO_MEM;
    }
    
    // Suspend the task immediately so it doesn't run until explicitly started
    vTaskSuspend(new_task->handle);
    
    // Add task to registry
    s_scheduler_ctx.tasks[s_scheduler_ctx.task_count] = new_task;
    s_scheduler_ctx.task_count++;
    
    // Return the task handle
    *task_handle = new_task;
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    ESP_LOGI(TAG, "Created task: %s (priority: %d, core: %d)", 
             name, (int)priority, (int)core_id);
    
    return ESP_OK;
}

esp_err_t scheduler_start_task(scheduler_task_t *task_handle) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid task handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if task is valid
    bool task_found = false;
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (s_scheduler_ctx.tasks[i] == task_handle) {
            task_found = true;
            break;
        }
    }
    
    if (!task_found) {
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Task not found in scheduler registry");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Resume the task if it's not already running
    if (!task_handle->is_running) {
        vTaskResume(task_handle->handle);
        task_handle->is_running = true;
        ESP_LOGI(TAG, "Started task: %s", task_handle->name);
    } else {
        ESP_LOGW(TAG, "Task already running: %s", task_handle->name);
    }
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

esp_err_t scheduler_suspend_task(scheduler_task_t *task_handle) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid task handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if task is valid
    bool task_found = false;
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (s_scheduler_ctx.tasks[i] == task_handle) {
            task_found = true;
            break;
        }
    }
    
    if (!task_found) {
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Task not found in scheduler registry");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Suspend the task if it's running
    if (task_handle->is_running) {
        vTaskSuspend(task_handle->handle);
        task_handle->is_running = false;
        ESP_LOGI(TAG, "Suspended task: %s", task_handle->name);
    } else {
        ESP_LOGW(TAG, "Task already suspended: %s", task_handle->name);
    }
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

esp_err_t scheduler_resume_task(scheduler_task_t *task_handle) {
    // This is essentially the same as start_task for our implementation
    return scheduler_start_task(task_handle);
}

esp_err_t scheduler_delete_task(scheduler_task_t *task_handle) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid task handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find task in registry
    int task_index = -1;
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (s_scheduler_ctx.tasks[i] == task_handle) {
            task_index = i;
            break;
        }
    }
    
    if (task_index == -1) {
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Task not found in scheduler registry");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Delete the task
    ESP_LOGI(TAG, "Deleting task: %s", task_handle->name);
    vTaskDelete(task_handle->handle);
    
    // Free resources
    free((void*)task_handle->name);
    free(task_handle);
    
    // Remove from registry by shifting remaining tasks
    for (uint32_t i = task_index; i < s_scheduler_ctx.task_count - 1; i++) {
        s_scheduler_ctx.tasks[i] = s_scheduler_ctx.tasks[i + 1];
    }
    s_scheduler_ctx.task_count--;
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

bool scheduler_is_task_running(scheduler_task_t *task_handle) {
    if (!s_scheduler_ctx.initialized || task_handle == NULL) {
        return false;
    }
    
    bool result = false;
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check if task is in registry and running
        for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
            if (s_scheduler_ctx.tasks[i] == task_handle) {
                result = task_handle->is_running;
                break;
            }
        }
        xSemaphoreGive(s_scheduler_ctx.mutex);
    }
    
    return result;
}

esp_err_t scheduler_get_task_count(uint32_t *count) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (count == NULL) {
        ESP_LOGE(TAG, "Invalid parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    *count = s_scheduler_ctx.task_count;
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

esp_err_t scheduler_get_tasks(scheduler_task_t *tasks, uint32_t *count) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (tasks == NULL || count == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Calculate how many tasks we can copy
    uint32_t copy_count = (*count < s_scheduler_ctx.task_count) ? 
                           *count : s_scheduler_ctx.task_count;
    
    // Copy task information
    for (uint32_t i = 0; i < copy_count; i++) {
        memcpy(&tasks[i], s_scheduler_ctx.tasks[i], sizeof(scheduler_task_t));
    }
    
    // Update count with the actual number copied
    *count = copy_count;
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

esp_err_t scheduler_find_task_by_name(const char *name, scheduler_task_t **task_handle) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (name == NULL || task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t result = ESP_ERR_NOT_FOUND;
    
    // Search for the task by name
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (strcmp(s_scheduler_ctx.tasks[i]->name, name) == 0) {
            *task_handle = s_scheduler_ctx.tasks[i];
            result = ESP_OK;
            break;
        }
    }
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    
    if (result != ESP_OK) {
        ESP_LOGD(TAG, "Task not found: %s", name);
    }
    
    return result;
}

esp_err_t scheduler_set_task_priority(scheduler_task_t *task_handle, scheduler_priority_t priority) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid task handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if task is in registry
    bool task_found = false;
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (s_scheduler_ctx.tasks[i] == task_handle) {
            task_found = true;
            break;
        }
    }
    
    if (!task_found) {
        xSemaphoreGive(s_scheduler_ctx.mutex);
        ESP_LOGE(TAG, "Task not found in scheduler registry");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Update priority
    ESP_LOGI(TAG, "Changing priority of task %s from %d to %d", 
             task_handle->name, task_handle->priority, priority);
    
    vTaskPrioritySet(task_handle->handle, (UBaseType_t)priority);
    task_handle->priority = priority;
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

esp_err_t scheduler_get_cpu_utilization(float *utilization_percent, uint8_t num_cores) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGE(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (utilization_percent == NULL) {
        ESP_LOGE(TAG, "Invalid parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (num_cores > portNUM_PROCESSORS) {
        ESP_LOGW(TAG, "Requesting more cores than available, limiting to %d", portNUM_PROCESSORS);
        num_cores = portNUM_PROCESSORS;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Copy utilization data
    for (uint8_t i = 0; i < num_cores; i++) {
        utilization_percent[i] = s_scheduler_ctx.cpu_utilization[i];
    }
    
    xSemaphoreGive(s_scheduler_ctx.mutex);
    return ESP_OK;
}

void scheduler_yield(void) {
    taskYIELD();
}

void scheduler_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

esp_err_t scheduler_deinit(void) {
    if (!s_scheduler_ctx.initialized) {
        ESP_LOGW(TAG, "Scheduler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Deinitializing scheduler");
    
    // Delete the CPU stats task first
    if (s_scheduler_ctx.cpu_stats_task_handle != NULL) {
        vTaskDelete(s_scheduler_ctx.cpu_stats_task_handle);
    }
    
    // Delete all managed tasks
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (s_scheduler_ctx.tasks[i]->handle != NULL) {
            vTaskDelete(s_scheduler_ctx.tasks[i]->handle);
        }
        free((void*)s_scheduler_ctx.tasks[i]->name);
        free(s_scheduler_ctx.tasks[i]);
    }
    
    // Release the mutex before deleting it
    xSemaphoreGive(s_scheduler_ctx.mutex);
    vSemaphoreDelete(s_scheduler_ctx.mutex);
    
    // Reset the context
    memset(&s_scheduler_ctx, 0, sizeof(scheduler_context_t));
    
    ESP_LOGI(TAG, "Scheduler deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Helper function to verify if a task handle is valid
 * 
 * @param task_handle Task handle to check
 * @return true if valid, false otherwise
 */
static bool is_task_valid(scheduler_task_t *task_handle) {
    if (!s_scheduler_ctx.initialized || task_handle == NULL) {
        return false;
    }
    
    for (uint32_t i = 0; i < s_scheduler_ctx.task_count; i++) {
        if (s_scheduler_ctx.tasks[i] == task_handle) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Task to monitor CPU utilization statistics
 * 
 * This task runs periodically and updates CPU utilization percentages
 * for all available cores.
 * 
 * @param arg Not used
 */
static void cpu_stats_task(void *arg) {
    uint64_t idle_ticks[portNUM_PROCESSORS];
    uint64_t total_ticks[portNUM_PROCESSORS];
    
    // Initialize tick counts
    for (int i = 0; i < portNUM_PROCESSORS; i++) {
        get_cpu_ticks(&s_scheduler_ctx.last_idle_ticks[i], &s_scheduler_ctx.last_total_ticks[i], i);
    }
    
    while (1) {
        // Calculate CPU utilization for each core
        for (int i = 0; i < portNUM_PROCESSORS; i++) {
            // Get current tick counts
            get_cpu_ticks(&idle_ticks[i], &total_ticks[i], i);
            
            // Calculate delta ticks
            uint64_t idle_delta = idle_ticks[i] - s_scheduler_ctx.last_idle_ticks[i];
            uint64_t total_delta = total_ticks[i] - s_scheduler_ctx.last_total_ticks[i];
            
            // Update metrics
            if (total_delta > 0) {
                // Take mutex to update the utilization value
                if (xSemaphoreTake(s_scheduler_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_scheduler_ctx.cpu_utilization[i] = 100.0f * (1.0f - ((float)idle_delta / total_delta));
                    xSemaphoreGive(s_scheduler_ctx.mutex);
                }
            }
            
            // Update last tick counts
            s_scheduler_ctx.last_idle_ticks[i] = idle_ticks[i];
            s_scheduler_ctx.last_total_ticks[i] = total_ticks[i];
        }
        
        // Log CPU utilization at debug level
        ESP_LOGD(TAG, "CPU Utilization - Core 0: %.1f%%, Core 1: %.1f%%", 
                 s_scheduler_ctx.cpu_utilization[0], s_scheduler_ctx.cpu_utilization[1]);
        
        // Sleep until next update
        vTaskDelay(pdMS_TO_TICKS(CPU_STATS_UPDATE_INTERVAL_MS));
    }
}

/**
 * @brief Get CPU idle and total ticks for a specific core
 * 
 * This is a platform-specific function that retrieves the CPU tick counts
 * used for calculating CPU utilization.
 * 
 * @param idle_ticks Pointer to store idle ticks
 * @param total_ticks Pointer to store total ticks
 * @param core_id Core to get statistics for
 */
static void get_cpu_ticks(uint64_t *idle_ticks, uint64_t *total_ticks, uint8_t core_id) {
    // This is ESP32-specific implementation using the FreeRTOS task runtime stats
    TaskStatus_t *task_status_array;
    UBaseType_t task_count, i;
    uint32_t total_runtime;
    
    // Get the number of tasks
    task_count = uxTaskGetNumberOfTasks();
    
    // Allocate array for task status
    task_status_array = (TaskStatus_t *)malloc(sizeof(TaskStatus_t) * task_count);
    if (task_status_array == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for task status");
        *idle_ticks = 0;
        *total_ticks = 1; // Avoid division by zero
        return;
    }
    
    // Fill the array with task status
    task_count = uxTaskGetSystemState(task_status_array, task_count, &total_runtime);
    
    // Calculate idle time
    uint64_t idle_time = 0;
    uint64_t total_time = total_runtime;
    
    // Find the idle task for the specified core
    char idle_task_name[16];
    snprintf(idle_task_name, sizeof(idle_task_name), "IDLE%u", core_id);
    
    for (i = 0; i < task_count; i++) {
        if (strcmp(task_status_array[i].pcTaskName, idle_task_name) == 0) {
            idle_time = task_status_array[i].ulRunTimeCounter;
            break;
        }
    }
    
    // Free the array
    free(task_status_array);
    
    // Set the output values
    *idle_ticks = idle_time;
    *total_ticks = total_time;
}