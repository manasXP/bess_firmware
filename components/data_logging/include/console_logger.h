/**
 * @file console_logger.h
 * @brief Console logging component for BESS 100KW/200KWH Firmware
 * 
 * This module implements console-specific logging functionality for the
 * Battery Energy Storage System (BESS). It leverages ESP-IDF's built-in
 * logging system with additional features for BESS-specific requirements.
 * 
 * @copyright (c) 2025 BESS Firmware Team
 * @version 1.0.0
 */

 #ifndef BESS_CONSOLE_LOGGER_H
 #define BESS_CONSOLE_LOGGER_H
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #include <stdbool.h>
 #include <stdint.h>
 #include "esp_err.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/semphr.h"
 #include "logger.h"
 
 /**
  * @brief Console logger configuration structure
  */
 typedef struct {
     bool color_enabled;        /**< Enable colored log output */
     bool timestamp_enabled;    /**< Include timestamp in log messages */
     bool level_enabled;        /**< Include log level in output */
     bool module_enabled;       /**< Include module name in output */
     bool mutex_enabled;        /**< Use mutex protection for console output */
     const char *tag_format;    /**< Format string for log tags, e.g., "[%s]" */
     bool use_esp_log;          /**< Use ESP-IDF logging system for output */
     esp_log_level_t esp_level_map[BESS_LOG_ALL+1]; /**< Mapping from BESS to ESP log levels */
 } bess_console_logger_config_t;
 
 /**
  * @brief Default configuration for console logger
  */
 extern const bess_console_logger_config_t BESS_CONSOLE_LOGGER_DEFAULT_CONFIG;
 
 /**
  * @brief Initialize the console logger component
  * 
  * @param config Pointer to console logger configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_init(const bess_console_logger_config_t *config);
 
 /**
  * @brief Deinitialize the console logger component
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_deinit(void);
 
 /**
  * @brief Write a log message to the console
  * 
  * @param level Log level
  * @param module Source module
  * @param timestamp Timestamp in milliseconds
  * @param message Log message string
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_write(bess_log_level_t level, bess_module_t module, 
                                   uint32_t timestamp, const char *message);
 
 /**
  * @brief Set console log level
  * 
  * @param level New log level for console output
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_set_level(bess_log_level_t level);
 
 /**
  * @brief Get current console log level
  * 
  * @param level Pointer to store current log level
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_get_level(bess_log_level_t *level);
 
 /**
  * @brief Enable or disable console output
  * 
  * @param enable True to enable, false to disable
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_enable(bool enable);
 
 /**
  * @brief Check if console logger is enabled
  * 
  * @param enabled Pointer to store enabled state
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_is_enabled(bool *enabled);
 
 /**
  * @brief Enable or disable colored console output
  * 
  * @param enable True to enable, false to disable
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_set_color(bool enable);
 
 /**
  * @brief Get module name string from module ID
  * 
  * @param module Module identifier
  * @return const char* Module name string
  */
 const char *bess_console_logger_get_module_name(bess_module_t module);
 
 /**
  * @brief Get log level name string from level ID
  * 
  * @param level Log level identifier
  * @return const char* Log level name string
  */
 const char *bess_console_logger_get_level_name(bess_log_level_t level);
 
 /**
  * @brief Get ANSI color code for a log level
  * 
  * @param level Log level
  * @return const char* ANSI color code string
  */
 const char *bess_console_logger_get_level_color(bess_log_level_t level);
 
 /**
  * @brief Convert BESS log level to ESP-IDF log level
  * 
  * @param level BESS log level
  * @return esp_log_level_t Corresponding ESP-IDF log level
  */
 esp_log_level_t bess_console_logger_bess_to_esp_level(bess_log_level_t level);
 
 /**
  * @brief Configure ESP-IDF log level mapping
  * 
  * @param bess_level BESS log level
  * @param esp_level ESP-IDF log level
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_set_level_mapping(bess_log_level_t bess_level, 
                                               esp_log_level_t esp_level);
 
 /**
  * @brief Enable or disable timestamp in console logs
  * 
  * @param enable True to enable, false to disable
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_set_timestamp(bool enable);
 
 /**
  * @brief Enable or disable module name in console logs
  * 
  * @param enable True to enable, false to disable
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_set_module_name(bool enable);
 
 /**
  * @brief Custom log message formatting function
  * 
  * This function formats a log message with all required elements (timestamp,
  * level, module name, etc.) according to the current configuration.
  * 
  * @param buffer Output buffer
  * @param buffer_size Size of output buffer
  * @param level Log level
  * @param module Source module
  * @param timestamp Timestamp in milliseconds
  * @param message Log message
  * @return int Number of bytes written to buffer
  */
 int bess_console_logger_format_message(char *buffer, size_t buffer_size,
                                      bess_log_level_t level, bess_module_t module,
                                      uint32_t timestamp, const char *message);
 
 /**
  * @brief Register log output override function
  * 
  * This allows replacing the standard console output mechanism with a custom
  * implementation (useful for unit testing or redirecting output).
  * 
  * @param output_func Function pointer for custom output
  * @param arg User argument passed to output function
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_register_output_func(
     int (*output_func)(const char *str, void *arg), void *arg);
 
 /**
  * @brief Get current console logger statistics
  * 
  * @param stats Pointer to statistics structure
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t bess_console_logger_get_stats(bess_console_logger_stats_t *stats);
 
 /**
  * @brief Console logger statistics
  */
 typedef struct {
     uint32_t messages_written;        /**< Total messages written to console */
     uint32_t buffer_overflows;        /**< Messages truncated due to buffer overflow */
     uint32_t mutex_timeouts;          /**< Times mutex acquisition timed out */
     uint32_t esp_log_messages;        /**< Messages processed via ESP-IDF logging */
     uint32_t direct_console_messages; /**< Messages written directly to console */
 } bess_console_logger_stats_t;
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* BESS_CONSOLE_LOGGER_H */