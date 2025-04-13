/**
 * @file semaphore_manager.h
 * @brief Centralized semaphore management for BESS firmware
 * 
 * This module provides a centralized system for creating, managing, and using
 * semaphores across the BESS firmware. It ensures proper synchronization between
 * multiple FreeRTOS tasks across battery management, communication interfaces,
 * and logging subsystems.
 * 
 * The 100KW/200KWH BESS with LFP battery modules (48V, 16KWH) requires careful
 * resource management to maintain thread safety across all subsystems while
 * preserving real-time performance characteristics.
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef SEMAPHORE_MANAGER_H
 #define SEMAPHORE_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/semphr.h"
 #include "esp_err.h"
 #include "esp_log.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /** @brief Tag for ESP logging */
 #define SEMAPHORE_TAG "SEM_MGR"
 
 /**
  * @brief Maximum number of managed semaphores
  * 
  * This value can be adjusted based on the specific needs of the BESS firmware.
  * Current allocation supports core BMS systems (Battery Manager, SoC Calculator,
  * Cell Balancer, Thermal Monitor) plus communications and middleware.
  */
 #define SEMAPHORE_MAX_COUNT 32
 
 /**
  * @brief Default timeout for semaphore operations in milliseconds
  */
 #define SEMAPHORE_DEFAULT_TIMEOUT_MS 1000
 
 /**
  * @brief Default wait time for mutex operations in ticks
  */
 #define SEMAPHORE_DEFAULT_WAIT_TICKS (SEMAPHORE_DEFAULT_TIMEOUT_MS / portTICK_PERIOD_MS)
 
 /**
  * @brief Special timeout value indicating infinite wait
  */
 #define SEMAPHORE_WAIT_FOREVER portMAX_DELAY
 
 /**
  * @brief Types of synchronization primitives managed by this module
  */
 typedef enum {
     SEMAPHORE_TYPE_MUTEX,           /**< Mutual exclusion semaphore */
     SEMAPHORE_TYPE_BINARY,          /**< Binary semaphore */
     SEMAPHORE_TYPE_COUNTING,        /**< Counting semaphore */
     SEMAPHORE_TYPE_RECURSIVE_MUTEX  /**< Recursive mutex for nested locks */
 } semaphore_type_t;
 
 /**
  * @brief Error codes specific to semaphore manager
  */
 typedef enum {
     SEMAPHORE_OK = 0,                /**< Operation successful */
     SEMAPHORE_ERR_INVALID_HANDLE,    /**< Invalid semaphore handle */
     SEMAPHORE_ERR_OUT_OF_RESOURCES,  /**< No more semaphores available */
     SEMAPHORE_ERR_ALREADY_EXISTS,    /**< Semaphore with this name already exists */
     SEMAPHORE_ERR_NOT_FOUND,         /**< Semaphore not found by name */
     SEMAPHORE_ERR_TIMEOUT,           /**< Timed out waiting for semaphore */
     SEMAPHORE_ERR_INTERNAL           /**< Internal error */
 } semaphore_err_t;
 
 /**
  * @brief Handle to a managed semaphore
  */
 typedef int32_t semaphore_handle_t;
 
 /**
  * @brief Invalid semaphore handle value
  */
 #define SEMAPHORE_INVALID_HANDLE (-1)
 
 /**
  * @brief Semaphore information structure
  */
 typedef struct {
     char name[32];                /**< Semaphore name */
     semaphore_type_t type;        /**< Semaphore type */
     uint32_t max_count;           /**< Maximum count for counting semaphores */
     uint32_t current_count;       /**< Current count (for counting semaphores) */
     uint32_t waiting_tasks;       /**< Number of tasks waiting on this semaphore */
     bool is_taken;                /**< Whether the semaphore is currently taken */
     TickType_t last_taken_tick;   /**< Tick count when semaphore was last taken */
     uint32_t access_count;        /**< Number of times semaphore has been accessed */
     uint32_t timeout_count;       /**< Number of timeout events for this semaphore */
 } semaphore_info_t;
 
 /**
  * @brief Initialize the semaphore manager
  * 
  * Must be called before any other semaphore manager functions.
  * 
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_manager_init(void);
 
 /**
  * @brief Deinitialize the semaphore manager and release all resources
  * 
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_manager_deinit(void);
 
 /**
  * @brief Create a new semaphore
  * 
  * @param name Unique name for the semaphore
  * @param type Type of semaphore to create
  * @param initial_count Initial count value (only for counting semaphores)
  * @param max_count Maximum count value (only for counting semaphores)
  * @param handle Pointer to store the created semaphore handle
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_create(const char *name, 
                           semaphore_type_t type,
                           uint32_t initial_count,
                           uint32_t max_count,
                           semaphore_handle_t *handle);
 
 /**
  * @brief Delete a previously created semaphore
  * 
  * @param handle Handle of the semaphore to delete
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_delete(semaphore_handle_t handle);
 
 /**
  * @brief Take (acquire) a semaphore with timeout
  * 
  * @param handle Handle of the semaphore to take
  * @param timeout_ms Maximum time to wait in milliseconds
  * @return ESP_OK if successful, ESP_ERR_TIMEOUT if timed out, otherwise an error code
  */
 esp_err_t semaphore_take(semaphore_handle_t handle, uint32_t timeout_ms);
 
 /**
  * @brief Give (release) a previously taken semaphore
  * 
  * @param handle Handle of the semaphore to give
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_give(semaphore_handle_t handle);
 
 /**
  * @brief Get a semaphore handle by name
  * 
  * @param name Name of the semaphore to find
  * @param handle Pointer to store the found semaphore handle
  * @return ESP_OK if successful, SEMAPHORE_ERR_NOT_FOUND if not found
  */
 esp_err_t semaphore_find_by_name(const char *name, semaphore_handle_t *handle);
 
 /**
  * @brief Get information about a semaphore
  * 
  * @param handle Handle of the semaphore to query
  * @param info Pointer to store the semaphore information
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_get_info(semaphore_handle_t handle, semaphore_info_t *info);
 
 /**
  * @brief Reset a semaphore to its initial state
  * 
  * For mutexes and binary semaphores, this ensures they are not taken.
  * For counting semaphores, this resets the count to the initial value.
  * 
  * @param handle Handle of the semaphore to reset
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_reset(semaphore_handle_t handle);
 
 /**
  * @brief Check if a semaphore is currently available
  * 
  * @param handle Handle of the semaphore to check
  * @param is_available Pointer to store the result (true if available)
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_is_available(semaphore_handle_t handle, bool *is_available);
 
 /**
  * @brief Dump status of all semaphores to the log
  * 
  * Useful for debugging deadlock situations.
  * 
  * @param log_level ESP log level to use
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_dump_status(esp_log_level_t log_level);
 
 /**
  * @brief Create standard system semaphores for BMS components
  * 
  * Creates a standard set of semaphores used by the BMS components:
  * - battery_manager_mutex
  * - soc_calculator_mutex
  * - cell_balancer_mutex
  * - thermal_monitor_mutex
  * - communication_mutex
  * - log_mutex
  * 
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t semaphore_create_system_defaults(void);
 
 /**
  * @brief Macro for easy semaphore-protected critical section
  * 
  * Usage:
  * SEMAPHORE_CRITICAL_SECTION_START(semaphore_handle, timeout_ms, err_label) {
  *     // Critical section code here
  * } SEMAPHORE_CRITICAL_SECTION_END(semaphore_handle, err_label);
  */
 #define SEMAPHORE_CRITICAL_SECTION_START(handle, timeout_ms, err_label) \
     do { \
         esp_err_t __err = semaphore_take(handle, timeout_ms); \
         if (__err != ESP_OK) { \
             ESP_LOGE(SEMAPHORE_TAG, "Failed to take semaphore: %d", __err); \
             goto err_label; \
         } \
         {
 
 #define SEMAPHORE_CRITICAL_SECTION_END(handle, err_label) \
         } \
         __err = semaphore_give(handle); \
         if (__err != ESP_OK) { \
             ESP_LOGE(SEMAPHORE_TAG, "Failed to give semaphore: %d", __err); \
             goto err_label; \
         } \
     } while (0)
 
 /**
  * @brief Convert a semaphore error code to a string
  * 
  * @param err The error code to convert
  * @return String representation of the error code
  */
 const char *semaphore_err_to_str(semaphore_err_t err);
 
 /**
  * @brief Convert a semaphore type to a string
  * 
  * @param type The semaphore type to convert
  * @return String representation of the semaphore type
  */
 const char *semaphore_type_to_str(semaphore_type_t type);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* SEMAPHORE_MANAGER_H */