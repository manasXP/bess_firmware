/**
 * @file comm_manager.h
 * @brief Communication manager for BESS
 *
 * Provides central management of all communication interfaces
 * including Modbus, CANbus, and network communications.
 */

 #ifndef COMM_MANAGER_H
 #define COMM_MANAGER_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 
 /**
  * @brief Communication interface types
  */
 typedef enum {
     COMM_INTERFACE_MODBUS_RTU,     /**< Modbus RTU (serial) */
     COMM_INTERFACE_MODBUS_TCP,     /**< Modbus TCP */
     COMM_INTERFACE_CANBUS,         /**< CANbus */
     COMM_INTERFACE_MQTT,           /**< MQTT */
     COMM_INTERFACE_HTTP,           /**< HTTP */
     COMM_INTERFACE_BLE,            /**< Bluetooth Low Energy */
     COMM_INTERFACE_COUNT           /**< Number of communication interfaces */
 } comm_interface_type_t;
 
 /**
  * @brief Communication interface status
  */
 typedef struct {
     bool enabled;                  /**< Interface is enabled */
     bool connected;                /**< Interface is connected */
     uint32_t tx_count;             /**< Number of transmitted messages */
     uint32_t rx_count;             /**< Number of received messages */
     uint32_t error_count;          /**< Number of communication errors */
     uint32_t last_tx_time;         /**< Last transmission timestamp */
     uint32_t last_rx_time;         /**< Last reception timestamp */
     uint32_t bytes_tx;             /**< Number of bytes transmitted */
     uint32_t bytes_rx;             /**< Number of bytes received */
 } comm_interface_status_t;
 
 /**
  * @brief Initialize the communication manager
  *
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_init(void);
 
 /**
  * @brief Start the communication manager tasks
  *
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_start(void);
 
 /**
  * @brief Get status of a specific communication interface
  *
  * @param interface_type Type of interface to get status for
  * @param[out] status Pointer to store the interface status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_get_interface_status(comm_interface_type_t interface_type,
                                             comm_interface_status_t *status);
 
 /**
  * @brief Enable a specific communication interface
  *
  * @param interface_type Type of interface to enable
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_enable_interface(comm_interface_type_t interface_type);
 
 /**
  * @brief Disable a specific communication interface
  *
  * @param interface_type Type of interface to disable
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_disable_interface(comm_interface_type_t interface_type);
 
 /**
  * @brief Register a callback for communication events
  *
  * @param interface_type Type of interface to register callback for
  * @param callback Function to call when event occurs
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_register_callback(comm_interface_type_t interface_type,
                                          bess_event_callback_t callback,
                                          void *user_data);
 
 /**
  * @brief Unregister a previously registered callback
  *
  * @param interface_type Type of interface
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_unregister_callback(comm_interface_type_t interface_type,
                                           bess_event_callback_t callback);
 
 /**
  * @brief Process data from all communication interfaces
  *
  * This function is called periodically to process incoming data
  * from all enabled interfaces.
  *
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_process_data(void);
 
 /**
  * @brief Get statistics for all communication interfaces
  *
  * @param[out] stats Array to store statistics for each interface
  * @param[in,out] count On input, the size of the stats array; on output, the number of interfaces
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_get_statistics(comm_interface_status_t *stats, size_t *count);
 
 /**
  * @brief Reset statistics for a specific interface
  *
  * @param interface_type Type of interface
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_reset_statistics(comm_interface_type_t interface_type);
 
 /**
  * @brief Check if a specific interface is enabled
  *
  * @param interface_type Type of interface
  * @param[out] enabled Pointer to store the enabled status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_is_interface_enabled(comm_interface_type_t interface_type, bool *enabled);
 
 /**
  * @brief Check if a specific interface is connected
  *
  * @param interface_type Type of interface
  * @param[out] connected Pointer to store the connected status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t comm_manager_is_interface_connected(comm_interface_type_t interface_type, bool *connected);
 
 #endif /* COMM_MANAGER_H */