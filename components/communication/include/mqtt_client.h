/**
 * @file mqtt_client.h
 * @brief MQTT Client for the BESS Main Controller
 * 
 * This module provides MQTT client functionality for the Battery Energy Storage
 * System (BESS) main controller, enabling communication with AWS IoT Core and
 * other MQTT brokers for telemetry, control, and logging.
 * 
 * @version 1.0.0
 * @date 2025-04-12
 * @copyright Proprietary
 */

 #ifndef BESS_MQTT_CLIENT_H
 #define BESS_MQTT_CLIENT_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/event_groups.h"
 #include "esp_log.h"
 #include "mqtt_client.h"
 #include "cJSON.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /** @brief Maximum number of MQTT topics to subscribe to */
 #define BESS_MQTT_MAX_SUBSCRIPTIONS 16
 
 /** @brief Maximum MQTT message size in bytes */
 #define BESS_MQTT_MAX_MESSAGE_SIZE 4096
 
 /** @brief Maximum length of MQTT topic string */
 #define BESS_MQTT_MAX_TOPIC_LENGTH 256
 
 /** @brief MQTT QoS levels */
 typedef enum {
     BESS_MQTT_QOS_0 = 0,  /**< At most once delivery */
     BESS_MQTT_QOS_1 = 1,  /**< At least once delivery */
     BESS_MQTT_QOS_2 = 2   /**< Exactly once delivery */
 } bess_mqtt_qos_t;
 
 /** @brief MQTT connection state */
 typedef enum {
     BESS_MQTT_STATE_DISCONNECTED = 0,  /**< Not connected to broker */
     BESS_MQTT_STATE_CONNECTING,         /**< Connection in progress */
     BESS_MQTT_STATE_CONNECTED,          /**< Connected to broker */
     BESS_MQTT_STATE_DISCONNECTING,      /**< Disconnection in progress */
     BESS_MQTT_STATE_ERROR               /**< Error state */
 } bess_mqtt_state_t;
 
 /** @brief MQTT client error codes */
 typedef enum {
     BESS_MQTT_ERR_OK = 0,                  /**< No error */
     BESS_MQTT_ERR_INIT_FAILED,             /**< Initialization failed */
     BESS_MQTT_ERR_CONNECT_FAILED,          /**< Connection to broker failed */
     BESS_MQTT_ERR_SUBSCRIBE_FAILED,        /**< Subscription failed */
     BESS_MQTT_ERR_PUBLISH_FAILED,          /**< Message publishing failed */
     BESS_MQTT_ERR_DISCONNECTED,            /**< Client disconnected */
     BESS_MQTT_ERR_TIMEOUT,                 /**< Operation timed out */
     BESS_MQTT_ERR_BUFFER_OVERFLOW,         /**< Message or topic too large */
     BESS_MQTT_ERR_TOPIC_EXISTS,            /**< Subscription already exists */
     BESS_MQTT_ERR_TOPIC_NOT_FOUND,         /**< Subscription not found */
     BESS_MQTT_ERR_MAX_SUBSCRIPTIONS,       /**< Maximum subscriptions reached */
     BESS_MQTT_ERR_INVALID_PARAMETERS,      /**< Invalid function parameters */
     BESS_MQTT_ERR_AUTHENTICATION_FAILED,   /**< Authentication failed */
     BESS_MQTT_ERR_TLS_FAILED,              /**< TLS/SSL error */
     BESS_MQTT_ERR_NOT_SUPPORTED,           /**< Feature not supported */
     BESS_MQTT_ERR_UNKNOWN                  /**< Unknown error */
 } bess_mqtt_err_t;
 
 /** @brief MQTT event types reported to callbacks */
 typedef enum {
     BESS_MQTT_EVENT_CONNECTED,          /**< Connected to the broker */
     BESS_MQTT_EVENT_DISCONNECTED,       /**< Disconnected from the broker */
     BESS_MQTT_EVENT_SUBSCRIBED,         /**< Successfully subscribed to topic */
     BESS_MQTT_EVENT_UNSUBSCRIBED,       /**< Successfully unsubscribed from topic */
     BESS_MQTT_EVENT_PUBLISHED,          /**< Message published successfully */
     BESS_MQTT_EVENT_DATA,               /**< Message received on subscribed topic */
     BESS_MQTT_EVENT_ERROR,              /**< Error occurred */
     BESS_MQTT_EVENT_BEFORE_CONNECT,     /**< About to connect to broker */
 } bess_mqtt_event_t;
 
 /** @brief MQTT message received on a subscribed topic */
 typedef struct {
     char topic[BESS_MQTT_MAX_TOPIC_LENGTH];  /**< Topic on which message was received */
     uint8_t *data;                           /**< Message payload */
     size_t data_len;                         /**< Length of message payload */
     bess_mqtt_qos_t qos;                     /**< QoS level of the message */
     bool retain;                             /**< Retain flag */
 } bess_mqtt_message_t;
 
 /** @brief MQTT event data passed to callbacks */
 typedef struct {
     bess_mqtt_event_t event_id;            /**< Type of event */
     union {
         /** @brief Data for BESS_MQTT_EVENT_DATA event */
         bess_mqtt_message_t message;  
         
         /** @brief Data for BESS_MQTT_EVENT_ERROR event */
         struct {
             bess_mqtt_err_t error_code;    /**< Error code */
             char error_msg[128];           /**< Error message */
         } error;
         
         /** @brief Data for BESS_MQTT_EVENT_SUBSCRIBED event */
         struct {
             char topic[BESS_MQTT_MAX_TOPIC_LENGTH];  /**< Subscribed topic */
             bess_mqtt_qos_t qos;                     /**< Granted QoS level */
         } subscribed;
         
         /** @brief Data for BESS_MQTT_EVENT_UNSUBSCRIBED event */
         struct {
             char topic[BESS_MQTT_MAX_TOPIC_LENGTH];  /**< Unsubscribed topic */
         } unsubscribed;
         
         /** @brief Data for BESS_MQTT_EVENT_PUBLISHED event */
         struct {
             uint16_t message_id;                     /**< Message ID */
         } published;
     };
 } bess_mqtt_event_data_t;
 
 /**
  * @brief MQTT event callback function type
  * 
  * @param event_data Event data including event type and relevant information
  * @param user_context User-provided context pointer (set during registration)
  */
 typedef void (*bess_mqtt_event_callback_t)(const bess_mqtt_event_data_t *event_data, void *user_context);
 
 /** @brief MQTT client configuration */
 typedef struct {
     /** MQTT broker URL (mqtt:// or mqtts://) */
     char broker_uri[128];
     
     /** Client identifier (must be unique per broker) */
     char client_id[64];
     
     /** Username for authentication (can be NULL) */
     char *username;
     
     /** Password for authentication (can be NULL) */
     char *password;
     
     /** Path to certificate file for TLS (can be NULL) */
     char *cert_file;
     
     /** Path to key file for TLS (can be NULL) */
     char *key_file;
     
     /** Path to CA certificate for TLS (can be NULL) */
     char *ca_file;
     
     /** Keep-alive interval in seconds */
     uint16_t keepalive;
     
     /** LWT (Last Will and Testament) settings */
     struct {
         /** Enable LWT message */
         bool enabled;
         
         /** LWT topic */
         char topic[BESS_MQTT_MAX_TOPIC_LENGTH];
         
         /** LWT message */
         char message[128];
         
         /** LWT QoS level */
         bess_mqtt_qos_t qos;
         
         /** LWT retain flag */
         bool retain;
     } lwt;
     
     /** Clean session flag */
     bool clean_session;
     
     /** Connection timeout in seconds */
     uint16_t timeout;
     
     /** Reconnect attempt interval in milliseconds */
     uint32_t reconnect_interval_ms;
     
     /** Maximum number of reconnect attempts (0 = infinite) */
     uint16_t max_reconnect_attempts;
     
     /** Event callback function */
     bess_mqtt_event_callback_t event_callback;
     
     /** User context pointer passed to callback */
     void *user_context;
 } bess_mqtt_config_t;
 
 /** @brief Default MQTT client configuration */
 #define BESS_MQTT_DEFAULT_CONFIG() { \
     .broker_uri = "", \
     .client_id = "bess_main_controller", \
     .username = NULL, \
     .password = NULL, \
     .cert_file = NULL, \
     .key_file = NULL, \
     .ca_file = NULL, \
     .keepalive = 60, \
     .lwt = { \
         .enabled = true, \
         .topic = "bess/status/connection", \
         .message = "{\"status\":\"disconnected\",\"reason\":\"unexpected\"}", \
         .qos = BESS_MQTT_QOS_1, \
         .retain = true \
     }, \
     .clean_session = true, \
     .timeout = 5, \
     .reconnect_interval_ms = 5000, \
     .max_reconnect_attempts = 0, \
     .event_callback = NULL, \
     .user_context = NULL \
 }
 
 /**
  * @brief Initialize MQTT client
  * 
  * @param config MQTT client configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_init(const bess_mqtt_config_t *config);
 
 /**
  * @brief Connect to MQTT broker
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_connect(void);
 
 /**
  * @brief Disconnect from MQTT broker
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise 
  */
 esp_err_t bess_mqtt_disconnect(void);
 
 /**
  * @brief Get current MQTT connection state
  * 
  * @return bess_mqtt_state_t Current state
  */
 bess_mqtt_state_t bess_mqtt_get_state(void);
 
 /**
  * @brief Check if MQTT client is connected
  * 
  * @return true if connected, false otherwise
  */
 bool bess_mqtt_is_connected(void);
 
 /**
  * @brief Subscribe to MQTT topic
  * 
  * @param topic Topic to subscribe to
  * @param qos QoS level for the subscription
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_subscribe(const char *topic, bess_mqtt_qos_t qos);
 
 /**
  * @brief Unsubscribe from MQTT topic
  * 
  * @param topic Topic to unsubscribe from
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_unsubscribe(const char *topic);
 
 /**
  * @brief Publish message to MQTT topic
  * 
  * @param topic Topic to publish to
  * @param data Message data
  * @param len Length of message data
  * @param qos QoS level for publishing
  * @param retain Retain flag
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish(const char *topic, const void *data, size_t len, 
                             bess_mqtt_qos_t qos, bool retain);
 
 /**
  * @brief Publish string message to MQTT topic
  * 
  * @param topic Topic to publish to
  * @param str Null-terminated string message
  * @param qos QoS level for publishing
  * @param retain Retain flag
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish_string(const char *topic, const char *str, 
                                  bess_mqtt_qos_t qos, bool retain);
 
 /**
  * @brief Publish JSON message to MQTT topic
  * 
  * @param topic Topic to publish to
  * @param json_root cJSON object to publish
  * @param qos QoS level for publishing
  * @param retain Retain flag
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish_json(const char *topic, const cJSON *json_root, 
                                bess_mqtt_qos_t qos, bool retain);
 
 /**
  * @brief Set event callback function
  * 
  * @param event_callback Callback function
  * @param user_context User context pointer passed to callback
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_set_callback(bess_mqtt_event_callback_t event_callback, void *user_context);
 
 /**
  * @brief Set LWT (Last Will and Testament) message
  * 
  * @param topic LWT topic
  * @param message LWT message
  * @param qos QoS level for LWT
  * @param retain Retain flag for LWT
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_set_lwt(const char *topic, const char *message, 
                           bess_mqtt_qos_t qos, bool retain);
 
 /**
  * @brief Publish BESS telemetry data to MQTT
  * 
  * This function creates a standardized telemetry JSON message containing
  * key battery system metrics and publishes it to the telemetry topic.
  * 
  * @param system_voltage Current system voltage
  * @param system_current Current system current
  * @param system_soc System State of Charge percentage
  * @param system_soh System State of Health percentage
  * @param power_kw Current power in kilowatts (+ charging, - discharging)
  * @param max_temp Maximum temperature across all modules
  * @param min_temp Minimum temperature across all modules
  * @param avg_temp Average temperature across all modules
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish_telemetry(float system_voltage, float system_current,
                                     float system_soc, float system_soh,
                                     float power_kw, float max_temp,
                                     float min_temp, float avg_temp);
 
 /**
  * @brief Publish system status update to MQTT
  * 
  * @param status_code Status code
  * @param status_message Status message
  * @param is_error True if this is an error status
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish_status(uint32_t status_code, const char *status_message, bool is_error);
 
 /**
  * @brief Publish alarm or event to MQTT
  * 
  * @param alarm_code Alarm or event code
  * @param alarm_message Alarm or event message
  * @param severity Severity level (0-5, with 5 being most severe)
  * @param module_id Module ID (if applicable, -1 if system-wide)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish_alarm(uint32_t alarm_code, const char *alarm_message, 
                                 uint8_t severity, int16_t module_id);
 
 /**
  * @brief Handle control commands received via MQTT
  * 
  * This helper function processes standardized control commands
  * received on the control topic.
  * 
  * @param message MQTT message containing control command
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_handle_control_command(const bess_mqtt_message_t *message);
 
 /**
  * @brief Register predefined topics for BESS operation
  * 
  * Subscribes to standard topics for control, configuration, and firmware updates
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_register_standard_topics(void);
 
 /**
  * @brief Publish log message to MQTT for CloudWatch integration
  * 
  * @param log_level Log level (0-5)
  * @param component Component name
  * @param message Log message
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_publish_log(uint8_t log_level, const char *component, const char *message);
 
 /**
  * @brief Clean up MQTT client resources
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_mqtt_cleanup(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* BESS_MQTT_CLIENT_H */