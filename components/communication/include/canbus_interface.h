/**
 * @file canbus_interface.h
 * @brief CANbus communication interface for BESS
 *
 * Provides functions for CANbus communication with battery modules 
 * and other components in the BESS system.
 */

 #ifndef CANBUS_INTERFACE_H
 #define CANBUS_INTERFACE_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 #include "driver/twai.h"  /* ESP-IDF CAN driver, previously called "can" now "twai" */
 
 /**
  * @brief CANbus message callback function type
  */
 typedef void (*canbus_message_callback_t)(twai_message_t *message, void *user_data);
 
 /**
  * @brief CANbus event types
  */
 typedef enum {
     CANBUS_EVENT_BUS_OFF,          /**< CAN controller is in bus-off state */
     CANBUS_EVENT_ERROR_PASSIVE,    /**< CAN controller is in error-passive state */
     CANBUS_EVENT_ERROR_WARNING,    /**< CAN controller is in error-warning state */
     CANBUS_EVENT_ERROR_ACTIVE,     /**< CAN controller is in error-active state */
     CANBUS_EVENT_BUS_RECOVERED     /**< CAN bus has recovered from bus-off state */
 } canbus_event_type_t;
 
 /**
  * @brief CANbus event callback function type
  */
 typedef void (*canbus_event_callback_t)(canbus_event_type_t event, void *user_data);
 
 /**
  * @brief CANbus module status message format
  */
 typedef struct {
     uint8_t module_id;              /**< Module ID */
     float voltage;                  /**< Module voltage in V */
     float current;                  /**< Module current in A */
     float temperature;              /**< Module temperature in °C */
     float state_of_charge;          /**< Module state of charge (0-100%) */
     uint8_t status_flags;           /**< Module status flags */
     bess_error_code_t error_code;   /**< Module error code */
 } canbus_module_status_t;
 
 /**
  * @brief Initialize the CANbus interface
  *
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_init(void);
 
 /**
  * @brief Start the CANbus interface
  *
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_start(void);
 
 /**
  * @brief Stop the CANbus interface
  *
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_stop(void);
 
 /**
  * @brief Send a CAN message
  *
  * @param message Pointer to the message to send
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_send_message(const twai_message_t *message);
 
 /**
  * @brief Register a callback for receiving CAN messages
  *
  * @param id Message ID to register for (0 for all messages)
  * @param mask ID mask for filtering (0 for exact match)
  * @param callback Function to call when message is received
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_register_message_callback(uint32_t id,
                                                     uint32_t mask,
                                                     canbus_message_callback_t callback,
                                                     void *user_data);
 
 /**
  * @brief Unregister a previously registered message callback
  *
  * @param id Message ID that was registered
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_unregister_message_callback(uint32_t id,
                                                       canbus_message_callback_t callback);
 
 /**
  * @brief Register a callback for CANbus events
  *
  * @param event_type Type of event to register for
  * @param callback Function to call when event occurs
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_register_event_callback(canbus_event_type_t event_type,
                                                   canbus_event_callback_t callback,
                                                   void *user_data);
 
 /**
  * @brief Unregister a previously registered event callback
  *
  * @param event_type Type of event
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_unregister_event_callback(canbus_event_type_t event_type,
                                                     canbus_event_callback_t callback);
 
 /**
  * @brief Get the current status of the CANbus interface
  *
  * @param[out] status Pointer to store the interface status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_get_status(twai_status_info_t *status);
 
 /**
  * @brief Set the CANbus bitrate
  *
  * @param bitrate Bitrate in bits per second
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_set_bitrate(uint32_t bitrate);
 
 /**
  * @brief Request data from a specific module
  *
  * @param module_id ID of the module
  * @param data_type Type of data to request
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_request_module_data(uint8_t module_id, uint8_t data_type);
 
 /**
  * @brief Send a command to a specific module
  *
  * @param module_id ID of the module
  * @param command Command to send
  * @param data Command data
  * @param data_len Length of command data
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_send_module_command(uint8_t module_id,
                                               uint8_t command,
                                               const uint8_t *data,
                                               size_t data_len);
 
 /**
  * @brief Parse a CANbus message containing module status
  *
  * @param message Pointer to CAN message
  * @param[out] status Pointer to store parsed module status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_parse_module_status(const twai_message_t *message,
                                               canbus_module_status_t *status);
 
 /**
  * @brief Create a CANbus message for heartbeat
  *
  * @param[out] message Pointer to store the created message
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_create_heartbeat_message(twai_message_t *message);
 
 /**
  * @brief Create a CANbus message for system status
  *
  * @param status System status to encode in the message
  * @param[out] message Pointer to store the created message
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t canbus_interface_create_system_status_message(const bess_system_status_t *status,
                                                        twai_message_t *message);
 
 #endif /* CANBUS_INTERFACE_H */