/**
 * @file logger.h
 * @brief Logging subsystem for BESS 100KW/200KWH Firmware
 * 
 * This module provides a unified logging interface that supports multiple
 * output destinations including console, SD card, and AWS CloudWatch.
 * It is designed for the Battery Energy Storage System (BESS) with LFP
 * battery modules (48V, 16KWH) running on ESP32-P4 with FreeRTOS.
 * 
 * @copyright (c) 2025 BESS Firmware Team
 * @version 1.0.0
 */

 #ifndef BESS_LOGGER_H
 #define BESS_LOGGER_H
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #include <stdbool.h>
 #include <stdint.h>
 #include <stdarg.h>
 #include "esp_err.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 
 /**
  * @brief Log levels matching ESP-IDF's logging system with additional levels
  */
 typedef enum {
     BESS_LOG_NONE = 0,       /**< No logging */
     BESS_LOG_ERROR = 1,      /**< Critical errors that prevent system operation */
     BESS_LOG_WARN = 2,       /**< Warning conditions that should be addressed */
     BESS_LOG_INFO = 3,       /**< Informational messages about system operation */
     BESS_LOG_DEBUG = 4,      /**< Detailed debugging information */
     BESS_LOG_VERBOSE = 5,    /**< Verbose debugging information */
     BESS_LOG_EVENT = 6,      /**< System events for monitoring */
     BESS_LOG_AUDIT = 7,      /**< Audit events for security tracking */
     BESS_LOG_ALL = 0xFF      /**< All log levels */
 } bess_log_level_t;
 
 /**
  * @brief Log destination flags
  */
 typedef enum {
     BESS_LOG_DEST_NONE = 0x00,           /**< No logging destination */
     BESS_LOG_DEST_CONSOLE = 0x01,         /**< Console output */
     BESS_LOG_DEST_SD_CARD = 0x02,         /**< SD card storage */
     BESS_LOG_DEST_CLOUD = 0x04,           /**< AWS CloudWatch */
     BESS_LOG_DEST_ALL = 0xFF              /**< All destinations */
 } bess_log_destination_t;
 
 /**
  * @brief Module identifiers for logging subsystem
  */
 typedef enum {
     BESS_MODULE_SYSTEM = 0,               /**< System-wide messages */
     BESS_MODULE_BATTERY_MANAGER,          /**< Battery Manager module */
     BESS_MODULE_SOC_CALCULATOR,           /**< State of Charge calculator */
     BESS_MODULE_CELL_BALANCER,            /**< Cell Balancer module */
     BESS_MODULE_THERMAL_MONITOR,          /**< Thermal monitoring module */
     BESS_MODULE_MODBUS,                   /**< Modbus communication */
     BESS_MODULE_CANBUS,                   /**< CANBus communication */
     BESS_MODULE_DATA_LOGGER,              /**< Data logging subsystem */
     BESS_MODULE_CLOUD_SYNC,               /**< Cloud synchronization */
     BESS_MODULE_POWER_CONTROL,            /**< Power control module */
     BESS_MODULE_USER_INTERFACE,           /**< User interface */
     BESS_MODULE_SCHEDULER,                /**< Task scheduling */
     BESS_MODULE_PROTECTION,               /**< Protection mechanisms */
     BESS_MODULE_DIAGNOSTICS,              /**< Diagnostic subsystem */
     BESS_MODULE_MAX                      /**< Maximum module identifier */
 } bess_module_t;
 
 /**
  * @brief Configuration for the logging subsystem
  */
 typedef struct {
     bess_log_level_t console_level;       /**< Logging level for console output */
     bess_log_level_t sd_card_level;       /**< Logging level for SD card storage */
     bess_log_level_t cloud_level;         /**< Logging level for CloudWatch */
     const char *sd_card_path;             /**< Path for log files on SD card */
     const char *aws_region;               /**< AWS region for CloudWatch */
     const char *aws_log_group;            /**< AWS CloudWatch log group */
     const char *aws_log_stream;           /**< AWS CloudWatch log stream */
     uint32_t queue_size;                  /**< Size of the logging queue */
     uint32_t task_priority;               /**< Priority for the logging task */
     uint32_t task_stack_size;             /**< Stack size for the logging task */
     uint16_t max_msg_length;              /**< Maximum length of log messages */
     bool enable_timestamp;                /**< Enable timestamp in log messages */
     bool enable_module_name;              /**< Enable module name in log messages */
     bool enable_log_colors;               /**< Enable colored console logging */
     bool enable_cloud_batch;              /**< Enable batching for cloud uploads */
     uint16_t cloud_batch_size;            /**< Number of logs per cloud batch */
     uint32_t cloud_batch_timeout_ms;      /**< Timeout for cloud batching in ms */
     uint32_t sd_max_file_size;            /**< Maximum size of log files in bytes */
     uint32_t sd_max_files;                /**< Maximum number of rotated log files */
 } bess_logger_config_t;
 
 /**
  * @brief Log message structure for internal use
  */
 typedef struct {
     bess_log_level_t level;               /**< Log level */
     bess_module_t module;                 /**< Source module */
     uint32_t timestamp;                   /**< Timestamp (milliseconds since boot) */
     bess_log_destination_t destinations;  /**< Target destinations for this message */
     char *message;                        /**< Log message content */
 } bess_log_message_t;
 
 /**
  * @brief Default logging configuration
  */
 extern const bess_logger_config_t BESS_LOGGER_DEFAULT_CONFIG;
 
 /**
  * @brief Initialize the logging subsystem with specified configuration
  * 
  * @param config Pointer to logger configuration structure
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_init(const bess_logger_config_t *config);
 
 /**
  * @brief Deinitialize the logging subsystem and free resources
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_deinit(void);
 
 /**
  * @brief Set logging level for a specific destination
  * 
  * @param destination Destination to configure
  * @param level Logging level to set
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_set_level(bess_log_destination_t destination, bess_log_level_t level);
 
 /**
  * @brief Get current logging level for a specific destination
  * 
  * @param destination Destination to query
  * @param level Pointer to store the logging level
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_get_level(bess_log_destination_t destination, bess_log_level_t *level);
 
 /**
  * @brief Enable or disable a logging destination
  * 
  * @param destination Destination to configure
  * @param enable True to enable, false to disable
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_enable_destination(bess_log_destination_t destination, bool enable);
 
 /**
  * @brief Check if a specific logging destination is enabled
  * 
  * @param destination Destination to query
  * @param enabled Pointer to store the enabled state
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_is_destination_enabled(bess_log_destination_t destination, bool *enabled);
 
 /**
  * @brief Flush all pending log messages
  * 
  * @param timeout_ms Timeout in milliseconds, or portMAX_DELAY for indefinite wait
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_flush(uint32_t timeout_ms);
 
 /**
  * @brief Log a message with variable arguments
  * 
  * @param level Log level of the message
  * @param module Source module identifier
  * @param destinations Bitwise combination of destination flags
  * @param format Printf-style format string
  * @param ... Variable arguments
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_log(bess_log_level_t level, bess_module_t module, 
                          bess_log_destination_t destinations, 
                          const char *format, ...);
 
 /**
  * @brief Log a message with va_list arguments
  * 
  * @param level Log level of the message
  * @param module Source module identifier
  * @param destinations Bitwise combination of destination flags
  * @param format Printf-style format string
  * @param args Variable argument list
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_vlog(bess_log_level_t level, bess_module_t module, 
                           bess_log_destination_t destinations, 
                           const char *format, va_list args);
 
 /**
  * @brief Configure SD card logging parameters
  * 
  * @param path Path on SD card for log files
  * @param max_file_size Maximum size of each log file before rotation
  * @param max_files Maximum number of rotated log files to keep
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_configure_sd(const char *path, uint32_t max_file_size, uint32_t max_files);
 
 /**
  * @brief Configure AWS CloudWatch parameters
  * 
  * @param region AWS region name
  * @param log_group CloudWatch log group name
  * @param log_stream CloudWatch log stream name
  * @param batch_size Number of logs per upload batch
  * @param batch_timeout_ms Timeout for batching in milliseconds
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_configure_cloud(const char *region, const char *log_group, 
                                      const char *log_stream, uint16_t batch_size,
                                      uint32_t batch_timeout_ms);
 
 /**
  * @brief Get statistics for the logging subsystem
  * 
  * @param stats Pointer to statistics structure
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_get_stats(bess_logger_stats_t *stats);
 
 /* Convenience macros for commonly used log operations */
 
 /**
  * @brief Log an error message to all configured destinations
  */
 #define BESS_LOGE(module, format, ...) \
     bess_logger_log(BESS_LOG_ERROR, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log a warning message to all configured destinations
  */
 #define BESS_LOGW(module, format, ...) \
     bess_logger_log(BESS_LOG_WARN, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log an info message to all configured destinations
  */
 #define BESS_LOGI(module, format, ...) \
     bess_logger_log(BESS_LOG_INFO, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log a debug message to all configured destinations
  */
 #define BESS_LOGD(module, format, ...) \
     bess_logger_log(BESS_LOG_DEBUG, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log a verbose message to all configured destinations
  */
 #define BESS_LOGV(module, format, ...) \
     bess_logger_log(BESS_LOG_VERBOSE, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log a system event to all configured destinations
  */
 #define BESS_LOGE_EVENT(module, format, ...) \
     bess_logger_log(BESS_LOG_EVENT, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log an audit record to all configured destinations
  */
 #define BESS_LOGE_AUDIT(module, format, ...) \
     bess_logger_log(BESS_LOG_AUDIT, module, BESS_LOG_DEST_ALL, format, ##__VA_ARGS__)
 
 /**
  * @brief Log directly to console only
  */
 #define BESS_LOG_CONSOLE(level, module, format, ...) \
     bess_logger_log(level, module, BESS_LOG_DEST_CONSOLE, format, ##__VA_ARGS__)
 
 /**
  * @brief Log directly to SD card only
  */
 #define BESS_LOG_SD(level, module, format, ...) \
     bess_logger_log(level, module, BESS_LOG_DEST_SD_CARD, format, ##__VA_ARGS__)
 
 /**
  * @brief Log directly to AWS CloudWatch only
  */
 #define BESS_LOG_CLOUD(level, module, format, ...) \
     bess_logger_log(level, module, BESS_LOG_DEST_CLOUD, format, ##__VA_ARGS__)
 
 /**
  * @brief Statistics for the logging system
  */
 typedef struct {
     uint32_t messages_queued;           /**< Number of messages currently in queue */
     uint32_t messages_processed;         /**< Total number of messages processed */
     uint32_t messages_dropped;           /**< Number of messages dropped due to queue full */
     uint32_t console_messages;           /**< Number of messages sent to console */
     uint32_t sd_card_messages;           /**< Number of messages written to SD card */
     uint32_t cloud_messages;             /**< Number of messages sent to CloudWatch */
     uint32_t cloud_batches;              /**< Number of CloudWatch batch uploads */
     uint32_t cloud_failures;             /**< Number of CloudWatch upload failures */
     uint32_t sd_card_errors;             /**< Number of SD card write errors */
     uint32_t queue_high_watermark;       /**< Highest queue utilization */
 } bess_logger_stats_t;
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* BESS_LOGGER_H */