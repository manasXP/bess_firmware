/**
 * @file task_manager.h
 * @brief Task Manager for BESS 100KW/200KWH FreeRTOS middleware
 *
 * This module provides task management functionality for the Battery Energy Storage System (BESS).
 * It coordinates the creation, scheduling, monitoring, and synchronization of various system tasks.
 * The task manager ensures proper resource allocation, priority management, and real-time performance
 * for the 100KW/200KWH BESS system using LFP battery modules (48V, 16KWH).
 *
 * @copyright Copyright (c) 2025
 */

 #ifndef TASK_MANAGER_H
 #define TASK_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/event_groups.h"
 #include "freertos/semphr.h"
 #include "esp_err.h"
 #include "esp_log.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /** @defgroup TASK_MANAGER Task Manager
  * @{
  */
 
 /**
  * @brief Task status enumeration
  */
 typedef enum {
     TASK_STATUS_IDLE = 0,        /*!< Task is idle */
     TASK_STATUS_RUNNING,         /*!< Task is currently running */
     TASK_STATUS_SUSPENDED,       /*!< Task is suspended */
     TASK_STATUS_BLOCKED,         /*!< Task is blocked on a resource */
     TASK_STATUS_DELETED,         /*!< Task has been deleted */
     TASK_STATUS_ERROR            /*!< Task encountered an error */
 } task_status_t;
 
 /**
  * @brief Task priority levels
  * 
  * Defined in decreasing order of priority with consideration for FreeRTOS
  * configMAX_PRIORITIES and system stability.
  */
 typedef enum {
     TASK_PRIORITY_CRITICAL = configMAX_PRIORITIES - 1,  /*!< Critical safety tasks (e.g. protection systems) */
     TASK_PRIORITY_HIGH = configMAX_PRIORITIES - 2,      /*!< High priority tasks (e.g. battery monitoring) */
     TASK_PRIORITY_MEDIUM_HIGH = configMAX_PRIORITIES - 3, /*!< Medium-high priority tasks (e.g. thermal mgmt) */
     TASK_PRIORITY_MEDIUM = configMAX_PRIORITIES - 4,    /*!< Medium priority tasks (e.g. communication) */
     TASK_PRIORITY_LOW = configMAX_PRIORITIES - 5,       /*!< Low priority tasks (e.g. data logging) */
     TASK_PRIORITY_LOWEST = configMAX_PRIORITIES - 6     /*!< Lowest priority tasks (e.g. diagnostics) */
 } task_priority_t;
 
 /**
  * @brief Task types in the BESS system
  */
 typedef enum {
     TASK_TYPE_BATTERY_MONITOR = 0,  /*!< Battery monitoring task */
     TASK_TYPE_CELL_BALANCER,        /*!< Cell balancing task */
     TASK_TYPE_THERMAL_MONITOR,      /*!< Thermal monitoring task */
     TASK_TYPE_SOC_CALCULATOR,       /*!< State of Charge calculation task */
     TASK_TYPE_MODBUS_SERVER,        /*!< Modbus communication server task */
     TASK_TYPE_CANBUS_INTERFACE,     /*!< CAN Bus interface task */
     TASK_TYPE_SYSTEM_PROTECTION,    /*!< System protection and safety task */
     TASK_TYPE_DATA_LOGGING,         /*!< Data logging task */
     TASK_TYPE_CLOUD_SYNC,           /*!< AWS CloudWatch synchronization task */
     TASK_TYPE_USER_INTERFACE,       /*!< User interface task */
     TASK_TYPE_DIAGNOSTICS,          /*!< Diagnostics and maintenance task */
     TASK_TYPE_CUSTOM,               /*!< Custom user-defined task */
     TASK_TYPE_MAX                   /*!< Maximum number of task types */
 } task_type_t;
 
 /**
  * @brief Task scheduling policy
  */
 typedef enum {
     TASK_SCHEDULE_PERIODIC = 0,     /*!< Task runs periodically */
     TASK_SCHEDULE_EVENT_DRIVEN,     /*!< Task runs in response to events */
     TASK_SCHEDULE_ONE_SHOT          /*!< Task runs once and then deletes itself */
 } task_schedule_t;
 
 /**
  * @brief Event flags for task synchronization
  */
 typedef enum {
     TASK_EVENT_BATT_DATA_READY = (1 << 0),      /*!< Battery data is ready */
     TASK_EVENT_THERMAL_ALERT = (1 << 1),        /*!< Thermal alert triggered */
     TASK_EVENT_COMM_REQUEST = (1 << 2),         /*!< Communication request received */
     TASK_EVENT_PROTECTION_TRIGGER = (1 << 3),   /*!< Protection system triggered */
     TASK_EVENT_LOG_READY = (1 << 4),            /*!< Log data ready to be written */
     TASK_EVENT_BALANCE_COMPLETE = (1 << 5),     /*!< Cell balancing completed */
     TASK_EVENT_SOC_UPDATED = (1 << 6),          /*!< State of Charge updated */
     TASK_EVENT_SYSTEM_ERROR = (1 << 7),         /*!< System error occurred */
     TASK_EVENT_USER_DEFINED_1 = (1 << 8),       /*!< User defined event 1 */
     TASK_EVENT_USER_DEFINED_2 = (1 << 9),       /*!< User defined event 2 */
     TASK_EVENT_ALL = 0x3FF                      /*!< All events mask */
 } task_event_t;
 
 /**
  * @brief Task synchronization method
  */
 typedef enum {
     TASK_SYNC_NONE = 0,     /*!< No synchronization required */
     TASK_SYNC_MUTEX,        /*!< Mutex-based synchronization */
     TASK_SYNC_SEMAPHORE,    /*!< Semaphore-based synchronization */
     TASK_SYNC_EVENT_GROUP,  /*!< Event group based synchronization */
     TASK_SYNC_QUEUE         /*!< Queue-based synchronization */
 } task_sync_method_t;
 
 /**
  * @brief Task handle structure containing details about a managed task
  */
 typedef struct {
     TaskHandle_t handle;              /*!< FreeRTOS task handle */
     char name[configMAX_TASK_NAME_LEN]; /*!< Task name */
     task_type_t type;                 /*!< Task type */
     task_priority_t priority;         /*!< Task priority */
     task_status_t status;             /*!< Current task status */
     task_schedule_t schedule;         /*!< Task scheduling policy */
     uint32_t stack_size;              /*!< Task stack size in bytes */
     uint32_t run_count;               /*!< Number of times task has run */
     uint32_t execution_time;          /*!< Last execution time in microseconds */
     uint32_t avg_execution_time;      /*!< Average execution time in microseconds */
     uint32_t max_execution_time;      /*!< Maximum execution time in microseconds */
     TickType_t last_wake_time;        /*!< Last wake time for periodic tasks */
     TickType_t period_ticks;          /*!< Period in ticks for periodic tasks */
     UBaseType_t core_id;              /*!< CPU core ID for the task (0 or 1) */
     BaseType_t cpu_usage;             /*!< CPU usage percentage (0-100) */
     uint32_t high_water_mark;         /*!< Stack high water mark */
     void *user_data;                  /*!< User-defined data pointer */
     bool auto_restart;                /*!< Whether to auto-restart on crash */
 } task_handle_t;
 
 /**
  * @brief Task synchronization object
  */
 typedef struct {
     task_sync_method_t method;        /*!< Synchronization method */
     union {
         SemaphoreHandle_t mutex;      /*!< Mutex handle */
         SemaphoreHandle_t semaphore;  /*!< Semaphore handle */
         EventGroupHandle_t event_group; /*!< Event group handle */
         QueueHandle_t queue;          /*!< Queue handle */
     } handle;                         /*!< Synchronization object handle */
     char name[configMAX_TASK_NAME_LEN]; /*!< Synchronization object name */
 } task_sync_t;
 
 /**
  * @brief Task statistics structure
  */
 typedef struct {
     uint32_t total_tasks;             /*!< Total number of managed tasks */
     uint32_t running_tasks;           /*!< Number of currently running tasks */
     uint32_t blocked_tasks;           /*!< Number of blocked tasks */
     uint32_t suspended_tasks;         /*!< Number of suspended tasks */
     uint32_t task_switches;           /*!< Total number of task switches */
     uint32_t cpu_usage_percent;       /*!< Total CPU usage percentage */
     uint32_t free_heap;               /*!< Free heap size in bytes */
     uint32_t min_free_heap;           /*!< Minimum free heap ever in bytes */
     uint32_t task_crashes;            /*!< Number of task crashes detected */
     uint32_t watchdog_resets;         /*!< Number of watchdog timer resets */
 } task_stats_t;
 
 /**
  * @brief Function signature for task functions
  */
 typedef void (*task_function_t)(void *arg);
 
 /**
  * @brief Function signature for task error callbacks
  */
 typedef void (*task_error_callback_t)(task_handle_t *task, esp_err_t error, void *arg);
 
 /**
  * @brief Task manager configuration structure
  */
 typedef struct {
     uint32_t max_tasks;                  /*!< Maximum number of tasks to manage */
     uint32_t stats_update_period_ms;     /*!< Period for updating task statistics in ms */
     bool enable_watchdog;                /*!< Enable watchdog timer for tasks */
     uint32_t watchdog_timeout_ms;        /*!< Watchdog timeout period in ms */
     bool enable_stack_overflow_check;    /*!< Enable stack overflow checking */
     bool enable_runtime_stats;           /*!< Enable runtime statistics collection */
     task_error_callback_t error_callback; /*!< Callback for task errors */
     void *error_callback_arg;            /*!< Argument for error callback */
 } task_manager_config_t;
 
 /**
  * @brief Default task manager configuration
  */
 #define TASK_MANAGER_DEFAULT_CONFIG() { \
     .max_tasks = 20, \
     .stats_update_period_ms = 5000, \
     .enable_watchdog = true, \
     .watchdog_timeout_ms = 10000, \
     .enable_stack_overflow_check = true, \
     .enable_runtime_stats = true, \
     .error_callback = NULL, \
     .error_callback_arg = NULL \
 }
 
 /**
  * @brief Initialize the task manager
  * 
  * @param[in] config Pointer to task manager configuration
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_init(const task_manager_config_t *config);
 
 /**
  * @brief Deinitialize the task manager and clean up resources
  * 
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_deinit(void);
 
 /**
  * @brief Create a new task and add it to the task manager
  * 
  * @param[in] name Task name
  * @param[in] func Task function
  * @param[in] arg Task function argument
  * @param[in] stack_size Task stack size in bytes
  * @param[in] priority Task priority
  * @param[in] type Task type
  * @param[in] core_id Core ID for the task (0 or 1), or tskNO_AFFINITY for any core
  * @param[out] task_out Pointer to store the created task handle (can be NULL)
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_create_task(const char *name, 
                                   task_function_t func, 
                                   void *arg, 
                                   uint32_t stack_size, 
                                   task_priority_t priority,
                                   task_type_t type,
                                   UBaseType_t core_id,
                                   task_handle_t **task_out);
 
 /**
  * @brief Create a periodic task that runs at specified intervals
  * 
  * @param[in] name Task name
  * @param[in] func Task function
  * @param[in] arg Task function argument
  * @param[in] stack_size Task stack size in bytes
  * @param[in] priority Task priority
  * @param[in] type Task type
  * @param[in] period_ms Period in milliseconds
  * @param[in] core_id Core ID for the task (0 or 1), or tskNO_AFFINITY for any core
  * @param[out] task_out Pointer to store the created task handle (can be NULL)
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_create_periodic_task(const char *name,
                                            task_function_t func,
                                            void *arg,
                                            uint32_t stack_size,
                                            task_priority_t priority,
                                            task_type_t type,
                                            uint32_t period_ms,
                                            UBaseType_t core_id,
                                            task_handle_t **task_out);
 
 /**
  * @brief Delete a task
  * 
  * @param[in] task Task handle obtained from task_manager_create_task
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_delete_task(task_handle_t *task);
 
 /**
  * @brief Suspend a task
  * 
  * @param[in] task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_suspend_task(task_handle_t *task);
 
 /**
  * @brief Resume a suspended task
  * 
  * @param[in] task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_resume_task(task_handle_t *task);
 
 /**
  * @brief Get task by name
  * 
  * @param[in] name Task name
  * @param[out] task_out Pointer to store the task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_get_task_by_name(const char *name, task_handle_t **task_out);
 
 /**
  * @brief Get task by type
  * 
  * @param[in] type Task type
  * @param[out] tasks_out Array to store the task handles
  * @param[in,out] count On input, the size of the tasks_out array; on output, the number of tasks found
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_get_tasks_by_type(task_type_t type, task_handle_t **tasks_out, uint32_t *count);
 
 /**
  * @brief Create a synchronization object
  * 
  * @param[in] name Name of the synchronization object
  * @param[in] method Synchronization method
  * @param[out] sync_out Pointer to store the created synchronization object
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_create_sync(const char *name, task_sync_method_t method, task_sync_t **sync_out);
 
 /**
  * @brief Delete a synchronization object
  * 
  * @param[in] sync Synchronization object
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_delete_sync(task_sync_t *sync);
 
 /**
  * @brief Take (acquire) a synchronization object
  * 
  * @param[in] sync Synchronization object
  * @param[in] timeout Timeout in milliseconds, or portMAX_DELAY to wait indefinitely
  * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or other error code
  */
 esp_err_t task_manager_take_sync(task_sync_t *sync, uint32_t timeout);
 
 /**
  * @brief Give (release) a synchronization object
  * 
  * @param[in] sync Synchronization object
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_give_sync(task_sync_t *sync);
 
 /**
  * @brief Wait for an event or events on an event group
  * 
  * @param[in] sync Event group synchronization object
  * @param[in] bits_to_wait Bits to wait for
  * @param[in] clear_on_exit Whether to clear bits on exit
  * @param[in] wait_for_all Whether to wait for all bits or any bit
  * @param[in] timeout Timeout in milliseconds, or portMAX_DELAY to wait indefinitely
  * @param[out] bits_out Pointer to store the actual bits that were set (can be NULL)
  * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or other error code
  */
 esp_err_t task_manager_wait_event(task_sync_t *sync, 
                                  EventBits_t bits_to_wait,
                                  bool clear_on_exit,
                                  bool wait_for_all,
                                  uint32_t timeout,
                                  EventBits_t *bits_out);
 
 /**
  * @brief Set event bits on an event group
  * 
  * @param[in] sync Event group synchronization object
  * @param[in] bits_to_set Bits to set
  * @param[out] bits_out Pointer to store the previous bits (can be NULL)
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_set_event(task_sync_t *sync, EventBits_t bits_to_set, EventBits_t *bits_out);
 
 /**
  * @brief Clear event bits on an event group
  * 
  * @param[in] sync Event group synchronization object
  * @param[in] bits_to_clear Bits to clear
  * @param[out] bits_out Pointer to store the previous bits (can be NULL)
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_clear_event(task_sync_t *sync, EventBits_t bits_to_clear, EventBits_t *bits_out);
 
 /**
  * @brief Send data to a task queue
  * 
  * @param[in] sync Queue synchronization object
  * @param[in] data Pointer to the data to send
  * @param[in] timeout Timeout in milliseconds, or portMAX_DELAY to wait indefinitely
  * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or other error code
  */
 esp_err_t task_manager_send_to_queue(task_sync_t *sync, const void *data, uint32_t timeout);
 
 /**
  * @brief Receive data from a task queue
  * 
  * @param[in] sync Queue synchronization object
  * @param[out] data Pointer to store the received data
  * @param[in] timeout Timeout in milliseconds, or portMAX_DELAY to wait indefinitely
  * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or other error code
  */
 esp_err_t task_manager_receive_from_queue(task_sync_t *sync, void *data, uint32_t timeout);
 
 /**
  * @brief Get task manager statistics
  * 
  * @param[out] stats Pointer to store the statistics
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_get_stats(task_stats_t *stats);
 
 /**
  * @brief Get detailed information about a specific task
  * 
  * @param[in] task Task handle
  * @param[out] info Pointer to store the task information
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_get_task_info(task_handle_t *task, task_handle_t *info);
 
 /**
  * @brief Set user data for a task
  * 
  * @param[in] task Task handle
  * @param[in] user_data Pointer to user data
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_set_task_user_data(task_handle_t *task, void *user_data);
 
 /**
  * @brief Get user data from a task
  * 
  * @param[in] task Task handle
  * @param[out] user_data_out Pointer to store the user data pointer
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_get_task_user_data(task_handle_t *task, void **user_data_out);
 
 /**
  * @brief Reset watchdog timer for a task
  * 
  * @param[in] task Task handle
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_reset_watchdog(task_handle_t *task);
 
 /**
  * @brief Dump task manager information to the log
  * 
  * @param[in] include_tasks Whether to include detailed task information
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_dump_info(bool include_tasks);
 
 /**
  * @brief Notify a task that's waiting on a direct-to-task notification
  * 
  * @param[in] task Task handle
  * @param[in] value Notification value to send
  * @param[in] action Notification action
  * @return esp_err_t ESP_OK on success, ESP_FAIL or other error code on failure
  */
 esp_err_t task_manager_notify_task(task_handle_t *task, uint32_t value, eNotifyAction action);
 
 /**
  * @brief Wait for a notification from another task
  * 
  * @param[in] timeout Timeout in milliseconds, or portMAX_DELAY to wait indefinitely
  * @param[out] notification_value Pointer to store the notification value (can be NULL)
  * @param[in] clear_on_exit Whether to clear the notification value on exit
  * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or other error code
  */
 esp_err_t task_manager_wait_notification(uint32_t timeout, uint32_t *notification_value, bool clear_on_exit);
 
 /** @} */
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* TASK_MANAGER_H */