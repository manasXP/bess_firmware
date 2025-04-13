/**
 * @file queue_manager.h
 * @brief Queue management system for BESS middleware
 *
 * This component provides a centralized queue management system for the 
 * 100KW/200KWH BESS controller firmware. It handles message passing between
 * different subsystems of the Battery Energy Storage System using FreeRTOS
 * queue primitives, with support for priority messaging, timeout handling,
 * and queue monitoring.
 *
 * @author BESS Firmware Team
 * @date April 2025
 */

 #ifndef QUEUE_MANAGER_H
 #define QUEUE_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/queue.h"
 #include "esp_err.h"
 
 /**
  * @brief Maximum number of queues that can be managed simultaneously
  */
 #define QUEUE_MANAGER_MAX_QUEUES 20
 
 /**
  * @brief Maximum queue name length
  */
 #define QUEUE_MANAGER_NAME_MAX_LEN 32
 
 /**
  * @brief Default timeout for queue operations in milliseconds
  */
 #define QUEUE_MANAGER_DEFAULT_TIMEOUT_MS 100
 
 /**
  * @brief Message priority levels
  */
 typedef enum {
     QUEUE_PRIORITY_LOW = 0,    /**< Low priority message */
     QUEUE_PRIORITY_NORMAL = 1, /**< Normal priority message */
     QUEUE_PRIORITY_HIGH = 2,   /**< High priority message */
     QUEUE_PRIORITY_CRITICAL = 3 /**< Critical priority message (emergency) */
 } queue_priority_t;
 
 /**
  * @brief Types of queues in the system
  */
 typedef enum {
     QUEUE_TYPE_BATTERY_DATA,    /**< Battery monitoring data */
     QUEUE_TYPE_THERMAL_DATA,    /**< Thermal monitoring data */
     QUEUE_TYPE_CONTROL_CMD,     /**< Control commands */
     QUEUE_TYPE_MODBUS_MSG,      /**< Modbus communication messages */
     QUEUE_TYPE_CANBUS_MSG,      /**< CANBus communication messages */
     QUEUE_TYPE_LOGGING,         /**< System logging messages */
     QUEUE_TYPE_ALARM,           /**< Alarm and error notifications */
     QUEUE_TYPE_CLOUD_DATA,      /**< Data to be sent to AWS CloudWatch */
     QUEUE_TYPE_USER_INTERFACE,  /**< User interface related messages */
     QUEUE_TYPE_CUSTOM           /**< Custom queue type */
 } queue_type_t;
 
 /**
  * @brief Queue statistics structure
  */
 typedef struct {
     uint32_t messages_sent;     /**< Total messages sent to this queue */
     uint32_t messages_received; /**< Total messages received from this queue */
     uint32_t messages_dropped;  /**< Messages dropped due to queue full */
     uint32_t high_watermark;    /**< Maximum queue usage observed */
     uint32_t current_usage;     /**< Current number of items in queue */
     uint32_t timeouts;          /**< Number of operation timeouts */
 } queue_stats_t;
 
 /**
  * @brief Queue handle structure
  */
 typedef struct {
     char name[QUEUE_MANAGER_NAME_MAX_LEN]; /**< Queue name */
     QueueHandle_t handle;                  /**< FreeRTOS queue handle */
     queue_type_t type;                     /**< Queue type */
     uint32_t size;                         /**< Queue size in items */
     uint32_t item_size;                    /**< Size of each queue item in bytes */
     queue_stats_t stats;                   /**< Queue statistics */
     bool is_active;                        /**< Whether queue is currently active */
 } queue_handle_t;
 
 /**
  * @brief Message envelope structure
  * 
  * This structure wraps the actual message data with metadata
  * like priority and timestamp for advanced queue processing.
  */
 typedef struct {
     void* data;                /**< Pointer to message data */
     size_t data_size;          /**< Size of message data */
     queue_priority_t priority; /**< Message priority */
     uint64_t timestamp;        /**< Message timestamp (microseconds since boot) */
     uint32_t source_id;        /**< ID of source task/component */
     uint32_t target_id;        /**< ID of target task/component (0 for broadcast) */
     uint32_t message_id;       /**< Unique message identifier */
 } queue_message_t;
 
 /**
  * @brief Queue event callback function type
  */
 typedef void (*queue_event_callback_t)(queue_type_t queue_type, void* data, void* context);
 
 /**
  * @brief Initialize the queue manager
  *
  * Must be called before any other queue manager functions.
  *
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_init(void);
 
 /**
  * @brief Deinitialize the queue manager and free all resources
  *
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_deinit(void);
 
 /**
  * @brief Create a new message queue
  *
  * @param name Queue name (for debugging and identification)
  * @param type Queue type
  * @param queue_size Number of items the queue can hold
  * @param item_size Size of each item in bytes
  * @param[out] handle Pointer to store the created queue handle
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_create_queue(const char* name, 
                                     queue_type_t type,
                                     uint32_t queue_size, 
                                     uint32_t item_size,
                                     queue_handle_t** handle);
 
 /**
  * @brief Delete an existing queue
  *
  * @param handle Queue handle to delete
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_delete_queue(queue_handle_t* handle);
 
 /**
  * @brief Send a message to a queue (non-envelope version)
  *
  * This is a simpler version for when priority and other metadata
  * are not needed.
  *
  * @param handle Queue handle
  * @param data Pointer to the data to send
  * @param timeout_ms Maximum time to wait in milliseconds (0 for no wait)
  * @return ESP_OK if successful, ESP_ERR_TIMEOUT on timeout, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_send(queue_handle_t* handle, 
                             const void* data,
                             uint32_t timeout_ms);
 
 /**
  * @brief Send a message to a queue with priority (envelope version)
  *
  * @param handle Queue handle
  * @param message Message envelope containing data and metadata
  * @param timeout_ms Maximum time to wait in milliseconds (0 for no wait)
  * @return ESP_OK if successful, ESP_ERR_TIMEOUT on timeout, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_send_with_priority(queue_handle_t* handle, 
                                           queue_message_t* message,
                                           uint32_t timeout_ms);
 
 /**
  * @brief Receive a message from a queue (non-envelope version)
  *
  * @param handle Queue handle
  * @param[out] data Pointer to store the received data
  * @param timeout_ms Maximum time to wait in milliseconds (0 for no wait, portMAX_DELAY for infinite)
  * @return ESP_OK if successful, ESP_ERR_TIMEOUT on timeout, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_receive(queue_handle_t* handle, 
                                void* data,
                                uint32_t timeout_ms);
 
 /**
  * @brief Receive a message from a queue with metadata (envelope version)
  *
  * @param handle Queue handle
  * @param[out] message Pointer to store the received message envelope
  * @param timeout_ms Maximum time to wait in milliseconds (0 for no wait, portMAX_DELAY for infinite)
  * @return ESP_OK if successful, ESP_ERR_TIMEOUT on timeout, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_receive_with_metadata(queue_handle_t* handle, 
                                              queue_message_t* message,
                                              uint32_t timeout_ms);
 
 /**
  * @brief Flush all items from a queue
  *
  * @param handle Queue handle
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_flush(queue_handle_t* handle);
 
 /**
  * @brief Check if a queue is empty
  *
  * @param handle Queue handle
  * @param[out] is_empty Pointer to store the result (true if empty)
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_is_empty(queue_handle_t* handle, bool* is_empty);
 
 /**
  * @brief Check if a queue is full
  *
  * @param handle Queue handle
  * @param[out] is_full Pointer to store the result (true if full)
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_is_full(queue_handle_t* handle, bool* is_full);
 
 /**
  * @brief Get the number of messages in a queue
  *
  * @param handle Queue handle
  * @param[out] messages_waiting Pointer to store the result
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_get_waiting_count(queue_handle_t* handle, 
                                          uint32_t* messages_waiting);
 
 /**
  * @brief Get queue statistics
  *
  * @param handle Queue handle
  * @param[out] stats Pointer to store the queue statistics
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_get_stats(queue_handle_t* handle, queue_stats_t* stats);
 
 /**
  * @brief Reset queue statistics counters
  *
  * @param handle Queue handle
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_reset_stats(queue_handle_t* handle);
 
 /**
  * @brief Get a queue handle by name
  *
  * @param name Queue name
  * @param[out] handle Pointer to store the queue handle
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_get_queue_by_name(const char* name, queue_handle_t** handle);
 
 /**
  * @brief Get a queue handle by type
  *
  * @param type Queue type
  * @param[out] handle Pointer to store the queue handle
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_get_queue_by_type(queue_type_t type, queue_handle_t** handle);
 
 /**
  * @brief Register a callback for queue events
  *
  * @param type Queue type to monitor
  * @param callback Callback function to register
  * @param context User context pointer passed to callback
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_register_callback(queue_type_t type, 
                                          queue_event_callback_t callback,
                                          void* context);
 
 /**
  * @brief Unregister a callback for queue events
  *
  * @param type Queue type
  * @param callback Callback function to unregister
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_unregister_callback(queue_type_t type, 
                                            queue_event_callback_t callback);
 
 /**
  * @brief Create a message envelope
  * 
  * Helper function to construct a message envelope with proper initialization.
  *
  * @param data Pointer to message data
  * @param data_size Size of message data in bytes
  * @param priority Message priority
  * @param source_id Source component/task ID
  * @param target_id Target component/task ID (0 for broadcast)
  * @param[out] message Pointer to store the created message envelope
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_create_message(void* data,
                                       size_t data_size,
                                       queue_priority_t priority,
                                       uint32_t source_id,
                                       uint32_t target_id,
                                       queue_message_t* message);
 
 /**
  * @brief Set the default timeout for queue operations
  *
  * @param timeout_ms Default timeout in milliseconds
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_set_default_timeout(uint32_t timeout_ms);
 
 /**
  * @brief Get the default timeout for queue operations
  *
  * @param[out] timeout_ms Pointer to store the default timeout
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_get_default_timeout(uint32_t* timeout_ms);
 
 /**
  * @brief Get the number of active queues in the system
  *
  * @param[out] count Pointer to store the number of active queues
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_get_queue_count(uint32_t* count);
 
 /**
  * @brief Suspend all queue operations
  * 
  * This function can be used during critical system events
  * to temporarily halt all queue processing.
  *
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_suspend(void);
 
 /**
  * @brief Resume all queue operations after suspension
  *
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_resume(void);
 
 /**
  * @brief Print queue statistics to the console
  *
  * Useful for debugging and monitoring system performance.
  *
  * @param handle Queue handle (NULL to print all queues)
  * @return ESP_OK if successful, ESP_FAIL otherwise
  */
 esp_err_t queue_manager_print_stats(queue_handle_t* handle);
 
 #endif /* QUEUE_MANAGER_H */