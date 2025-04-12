/**
 * @file sd_logger.h
 * @brief SD Card logging component for BESS 100KW/200KWH system
 * 
 * This module provides functions for logging data to an SD card in the 
 * Battery Energy Storage System. It's part of the data_logger subsystem
 * that also includes console and cloud logging capabilities.
 * 
 * @note This component is designed for the ESP32-P4 platform running FreeRTOS
 *
 * @copyright (c) 2025 BESS Systems
 */

 #ifndef SD_LOGGER_H
 #define SD_LOGGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include <time.h>
 #include "esp_err.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/queue.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Log levels for the SD card logger
  * 
  * These levels match the ESP-IDF log levels for consistency
  */
 typedef enum {
     SD_LOG_NONE = 0,     /**< No logging */
     SD_LOG_ERROR,        /**< Critical errors that require attention */
     SD_LOG_WARN,         /**< Warning conditions */
     SD_LOG_INFO,         /**< Informational messages */
     SD_LOG_DEBUG,        /**< Debug messages */
     SD_LOG_VERBOSE       /**< Verbose debug messages */
 } sd_log_level_t;
 
 /**
  * @brief Configuration structure for the SD card logger
  */
 typedef struct {
     const char *mount_point;       /**< SD card mount point */
     const char *base_path;         /**< Base path for log files */
     const char *filename_prefix;   /**< Prefix for log file names */
     uint32_t max_file_size_kb;     /**< Maximum size of each log file in KB */
     uint8_t max_files;             /**< Maximum number of log files to keep */
     sd_log_level_t log_level;      /**< Minimum log level to record */
     bool auto_flush;               /**< Whether to flush after each write */
     uint32_t buffer_size;          /**< Internal buffer size in bytes */
     uint32_t queue_size;           /**< Size of the message queue */
     bool include_timestamp;        /**< Whether to include timestamps */
     uint8_t max_write_retry;       /**< Maximum number of write retries */
     uint16_t task_priority;        /**< Priority of the logger task */
     uint32_t task_stack_size;      /**< Stack size for the logger task */
     bool enable_sd_checks;         /**< Enable periodic SD card health checks */
     uint32_t sd_check_interval_ms; /**< Interval for SD health checks */
 } sd_logger_config_t;
 
 /**
  * @brief Log message structure for internal use
  */
 typedef struct {
     time_t timestamp;              /**< Message timestamp */
     sd_log_level_t level;          /**< Log level of the message */
     char tag[16];                  /**< Module tag */
     char message[256];             /**< Log message content */
 } sd_log_message_t;
 
 /**
  * @brief Status information for the SD logger
  */
 typedef struct {
     bool is_initialized;           /**< Whether the logger is initialized */
     bool is_mounted;               /**< Whether the SD card is mounted */
     uint32_t bytes_written;        /**< Total bytes written in current session */
     uint32_t messages_logged;      /**< Total messages logged in current session */
     uint32_t failed_writes;        /**< Count of failed write operations */
     uint32_t queue_highwater;      /**< Highest queue usage */
     uint8_t current_file_index;    /**< Current log file index */
     time_t start_time;             /**< Time when logging started */
     uint64_t free_space_bytes;     /**< Free space remaining on SD card */
     uint8_t card_health;           /**< SD card health indicator (0-100%) */
 } sd_logger_status_t;
 
 /**
  * @brief Default configuration for the SD logger
  */
 #define SD_LOGGER_DEFAULT_CONFIG() { \
     .mount_point = "/sdcard", \
     .base_path = "/sdcard/logs", \
     .filename_prefix = "bess_log", \
     .max_file_size_kb = 1024, \
     .max_files = 10, \
     .log_level = SD_LOG_INFO, \
     .auto_flush = true, \
     .buffer_size = 4096, \
     .queue_size = 100, \
     .include_timestamp = true, \
     .max_write_retry = 3, \
     .task_priority = tskIDLE_PRIORITY + 2, \
     .task_stack_size = 4096, \
     .enable_sd_checks = true, \
     .sd_check_interval_ms = 60000 \
 }
 
 /**
  * @brief Initialize the SD card logger
  * 
  * @param config Pointer to the configuration structure
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_init(const sd_logger_config_t *config);
 
 /**
  * @brief Deinitialize the SD card logger
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_deinit(void);
 
 /**
  * @brief Write a log message to the SD card
  * 
  * @param level Log level of the message
  * @param tag Module tag (max 15 characters)
  * @param format Printf-style format string
  * @param ... Variable arguments for the format string
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_write(sd_log_level_t level, const char *tag, const char *format, ...);
 
 /**
  * @brief Write a log message with a binary blob
  * 
  * @param level Log level of the message
  * @param tag Module tag (max 15 characters)
  * @param message Text message to accompany the binary data
  * @param data Pointer to the binary data
  * @param data_len Length of the binary data in bytes
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_write_binary(sd_log_level_t level, const char *tag, 
                                 const char *message, const void *data, size_t data_len);
 
 /**
  * @brief Set the minimum log level
  * 
  * @param level Minimum log level to record
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_set_level(sd_log_level_t level);
 
 /**
  * @brief Get the current log level
  * 
  * @param level Pointer to store the current log level
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_get_level(sd_log_level_t *level);
 
 /**
  * @brief Flush all pending log data to the SD card
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_flush(void);
 
 /**
  * @brief Get the status of the SD logger
  * 
  * @param status Pointer to the status structure to fill
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_get_status(sd_logger_status_t *status);
 
 /**
  * @brief Check the health of the SD card
  * 
  * @param health_percent Pointer to store the health percentage (0-100%)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_check_health(uint8_t *health_percent);
 
 /**
  * @brief Start a new log file
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_rotate_file(void);
 
 /**
  * @brief Register a callback for SD card events
  * 
  * @param callback Function pointer to the callback
  * @param arg User-defined argument to pass to the callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 typedef void (*sd_logger_event_cb_t)(void *arg, esp_err_t event);
 esp_err_t sd_logger_register_event_handler(sd_logger_event_cb_t callback, void *arg);
 
 /**
  * @brief BMS-specific log macros for easy use
  * 
  * These macros match the ESP-IDF logging style but direct to SD card
  */
 #define SD_LOGE(tag, format, ...) sd_logger_write(SD_LOG_ERROR, tag, format, ##__VA_ARGS__)
 #define SD_LOGW(tag, format, ...) sd_logger_write(SD_LOG_WARN, tag, format, ##__VA_ARGS__)
 #define SD_LOGI(tag, format, ...) sd_logger_write(SD_LOG_INFO, tag, format, ##__VA_ARGS__)
 #define SD_LOGD(tag, format, ...) sd_logger_write(SD_LOG_DEBUG, tag, format, ##__VA_ARGS__)
 #define SD_LOGV(tag, format, ...) sd_logger_write(SD_LOG_VERBOSE, tag, format, ##__VA_ARGS__)
 
 /**
  * @brief Log battery-specific events to SD card
  * 
  * @param module_id Battery module ID
  * @param event_type Type of battery event
  * @param event_data Event-specific data
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_battery_event(uint8_t module_id, 
                                 uint32_t event_type, 
                                 const void *event_data,
                                 size_t event_data_size);
 
 /**
  * @brief Create a CSV-formatted log entry for battery telemetry
  * 
  * @param module_id Battery module ID
  * @param voltage Module voltage in millivolts
  * @param current Current in milliamps
  * @param temperature Temperature in degrees Celsius
  * @param soc State of charge percentage
  * @param soh State of health percentage
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t sd_logger_battery_telemetry(uint8_t module_id,
                                     uint32_t voltage,
                                     int32_t current,
                                     float temperature,
                                     float soc,
                                     float soh);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* SD_LOGGER_H */