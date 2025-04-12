/**
 * @file cloudwatch_logger.h
 * @brief AWS CloudWatch logging integration for the BESS 100KW/200KWH system
 * 
 * This module provides functionality to log BESS data and events to AWS CloudWatch.
 * It is part of the data_logger subsystem which also supports console and SD card logging.
 * 
 * @note This implementation is designed for the ESP32-P4 platform running FreeRTOS.
 * 
 * @copyright Copyright (c) 2025 BESS Systems Inc.
 */

 #ifndef CLOUDWATCH_LOGGER_H
 #define CLOUDWATCH_LOGGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Log level enumeration for CloudWatch logs
  * 
  * These match the ESP logging levels for consistency across platforms
  */
 typedef enum {
     CLOUDWATCH_LOG_NONE,     /*!< No logging */
     CLOUDWATCH_LOG_ERROR,    /*!< Critical errors, system stability affected */
     CLOUDWATCH_LOG_WARN,     /*!< Warning conditions, potential issues */
     CLOUDWATCH_LOG_INFO,     /*!< General operational information */
     CLOUDWATCH_LOG_DEBUG,    /*!< Detailed information, debugging purpose */
     CLOUDWATCH_LOG_VERBOSE   /*!< Extra verbose debugging information */
 } cloudwatch_log_level_t;
 
 /**
  * @brief CloudWatch log message structure
  * 
  * Represents a log message to be sent to CloudWatch
  */
 typedef struct {
     char *log_group;                 /*!< CloudWatch log group name */
     char *log_stream;                /*!< CloudWatch log stream name */
     char *message;                   /*!< Log message content */
     uint64_t timestamp;              /*!< Message timestamp (milliseconds since epoch) */
     cloudwatch_log_level_t level;    /*!< Log level */
     uint32_t sequence_token;         /*!< CloudWatch sequence token for batch operations */
 } cloudwatch_log_message_t;
 
 /**
  * @brief Configuration parameters for CloudWatch logger
  */
 typedef struct {
     const char *aws_region;          /*!< AWS region (e.g., "us-east-1") */
     const char *aws_access_key;      /*!< AWS access key ID */
     const char *aws_secret_key;      /*!< AWS secret access key */
     const char *log_group_name;      /*!< Default CloudWatch log group name */
     const char *device_id;           /*!< Device identifier used in log stream names */
     bool use_secure_connection;      /*!< Use HTTPS for CloudWatch API calls */
     cloudwatch_log_level_t min_level;/*!< Minimum log level to send to CloudWatch */
     uint16_t queue_size;             /*!< Maximum number of pending log messages */
     uint16_t batch_size;             /*!< Maximum number of logs to send in one batch */
     uint32_t upload_period_ms;       /*!< Period between upload attempts */
     bool enable_compression;         /*!< Enable gzip compression for log messages */
     uint32_t retry_interval_ms;      /*!< Retry interval on connection failure */
     uint8_t max_retries;             /*!< Maximum number of retries on failure */
     size_t max_message_size;         /*!< Maximum size of a single log message in bytes */
     bool cache_on_failure;           /*!< Whether to cache logs to SD card on connection failure */
 } cloudwatch_logger_config_t;
 
 /**
  * @brief Default configuration for CloudWatch logger
  */
 #define CLOUDWATCH_LOGGER_DEFAULT_CONFIG() { \
     .aws_region = CONFIG_BESS_AWS_REGION, \
     .aws_access_key = CONFIG_BESS_AWS_ACCESS_KEY, \
     .aws_secret_key = CONFIG_BESS_AWS_SECRET_KEY, \
     .log_group_name = CONFIG_BESS_CLOUDWATCH_LOG_GROUP, \
     .device_id = CONFIG_BESS_DEVICE_ID, \
     .use_secure_connection = true, \
     .min_level = CLOUDWATCH_LOG_INFO, \
     .queue_size = 100, \
     .batch_size = 10, \
     .upload_period_ms = 60000, \
     .enable_compression = true, \
     .retry_interval_ms = 5000, \
     .max_retries = 3, \
     .max_message_size = 256 * 1024, /* 256 KB maximum */ \
     .cache_on_failure = true \
 }
 
 /**
  * @brief Initialize the CloudWatch logger
  * 
  * @param config Configuration parameters for the logger
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_init(const cloudwatch_logger_config_t *config);
 
 /**
  * @brief Deinitialize the CloudWatch logger and free resources
  * 
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_deinit(void);
 
 /**
  * @brief Start the CloudWatch logger task
  * 
  * @param task_priority Priority for the logger task
  * @param stack_size Stack size for the logger task in bytes
  * @param core_id CPU core to run the task on, or -1 for no affinity
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_start(uint8_t task_priority, uint32_t stack_size, int core_id);
 
 /**
  * @brief Stop the CloudWatch logger task
  * 
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_stop(void);
 
 /**
  * @brief Log a message to CloudWatch
  * 
  * @param level Log level
  * @param tag Module tag
  * @param format Format string (printf style)
  * @param ... Variable arguments for format string
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_log(cloudwatch_log_level_t level, const char *tag, const char *format, ...);
 
 /**
  * @brief Directly post a formatted JSON message to CloudWatch
  * 
  * @param level Log level
  * @param tag Module tag
  * @param json_msg Pre-formatted JSON message
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_log_json(cloudwatch_log_level_t level, const char *tag, const char *json_msg);
 
 /**
  * @brief Explicitly flush pending log messages to CloudWatch
  * 
  * @param timeout_ms Maximum time to wait for flush completion
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_flush(uint32_t timeout_ms);
 
 /**
  * @brief Check if CloudWatch logger is connected to AWS
  * 
  * @return true if connected, false otherwise
  */
 bool cloudwatch_logger_is_connected(void);
 
 /**
  * @brief Set the log level filter for CloudWatch
  * 
  * Only messages at or above this level will be sent to CloudWatch
  * 
  * @param level Minimum log level to send
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_set_level(cloudwatch_log_level_t level);
 
 /**
  * @brief Get the current log level filter for CloudWatch
  * 
  * @return Current minimum log level
  */
 cloudwatch_log_level_t cloudwatch_logger_get_level(void);
 
 /**
  * @brief Create a new log stream in the configured log group
  * 
  * @param stream_name Name of the log stream to create
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_create_stream(const char *stream_name);
 
 /**
  * @brief Set a custom log group and stream for subsequent log messages
  * 
  * @param log_group CloudWatch log group name
  * @param log_stream CloudWatch log stream name
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_set_destination(const char *log_group, const char *log_stream);
 
 /**
  * @brief Reset to default log group and stream for subsequent log messages
  * 
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_reset_destination(void);
 
 /**
  * @brief BMS event callback function for logging BMS events to CloudWatch
  * 
  * Register this function with the battery_manager_register_event_callback() API
  * 
  * @param event_type BMS event type
  * @param event_data Event-specific data
  * @param user_data User data (unused)
  */
 void cloudwatch_logger_bms_event_handler(uint32_t event_type, void *event_data, void *user_data);
 
 /**
  * @brief Log battery system status to CloudWatch
  * 
  * @param module_id Module ID (0xFF for system-wide status)
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_log_battery_status(uint8_t module_id);
 
 /**
  * @brief Log thermal system status to CloudWatch
  * 
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_log_thermal_status(void);
 
 /**
  * @brief Log system metrics to CloudWatch (CPU usage, memory, etc.)
  * 
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_log_system_metrics(void);
 
 /**
  * @brief Upload cached logs from SD card to CloudWatch
  * 
  * @param max_logs Maximum number of logs to upload in this call
  * @return ESP_OK on success, or an error code on failure
  */
 esp_err_t cloudwatch_logger_upload_cached_logs(uint32_t max_logs);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* CLOUDWATCH_LOGGER_H */