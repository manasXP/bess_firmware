/**
 * @file console_logger.c
 * @brief Console logging implementation for BESS 100KW/200KWH firmware
 * 
 * This module provides console logging functionality for the Battery Energy 
 * Storage System, supporting various log levels and formatted output.
 * It interfaces with the ESP-IDF logging system and FreeRTOS.
 * 
 * @copyright (c) 2025 BESS Firmware Team
 * @version 1.0.0
 */

 #include <stdio.h>
 #include <string.h>
 #include <stdarg.h>
 #include <time.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "esp_log.h"
 #include "esp_system.h"
 #include "console_logger.h"
 
 /* Private variables */
 static console_logger_config_t s_config = {0};
 static SemaphoreHandle_t s_console_mutex = NULL;
 static bool s_is_initialized = false;
 static console_logger_stats_t s_stats = {0};
 static const char *LOG_LEVEL_TAGS[] = {
     "NONE", "ERROR", "WARN", "INFO", "DEBUG", "VERBOSE"
 };
 static const char *LOG_LEVEL_COLORS[] = {
     "", "\033[31m", "\033[33m", "\033[32m", "\033[36m", "\033[35m"
 };
 static const char *COLOR_RESET = "\033[0m";
 
 /* Module names for displaying in logs */
 static const char *MODULE_NAMES[] = {
     "SYSTEM", "BATTERY", "SOC", "CELL", "THERMAL", "MODBUS", 
     "CANBUS", "DATALOG", "CLOUD", "POWER", "UI", "SCHED", 
     "PROTECT", "DIAG", "UNKNOWN"
 };
 
 /* Private function prototypes */
 static void format_timestamp(char *buffer, size_t size);
 static console_log_level_t esp_level_to_console_level(esp_log_level_t level);
 static esp_log_level_t console_level_to_esp_level(console_log_level_t level);
 
 /* ESP-IDF log hook function */
 static void esp_log_hook(const char *tag, esp_log_level_t level, 
                         const char *fmt, va_list args);
 
 /**
  * @brief Initialize the console logger with specified configuration
  * 
  * @param config Pointer to logger configuration structure
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_init(const console_logger_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create mutex for thread safety
     s_console_mutex = xSemaphoreCreateMutex();
     if (s_console_mutex == NULL) {
         return ESP_ERR_NO_MEM;
     }
     
     // Copy configuration
     memcpy(&s_config, config, sizeof(console_logger_config_t));
     
     // Initialize statistics
     memset(&s_stats, 0, sizeof(console_logger_stats_t));
     s_stats.start_time = time(NULL);
     
     // Set ESP-IDF log level
     esp_log_level_set("*", console_level_to_esp_level(s_config.log_level));
     
     // Redirect ESP-IDF logs if requested
     if (s_config.redirect_esp_logs) {
         esp_log_set_vprintf((vprintf_like_t)esp_log_hook);
     }
     
     s_is_initialized = true;
     return ESP_OK;
 }
 
 /**
  * @brief Deinitialize the console logger and free resources
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_deinit(void) {
     if (!s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Reset ESP-IDF log vprintf function if we changed it
     if (s_config.redirect_esp_logs) {
         esp_log_set_vprintf(vprintf);
     }
     
     if (s_console_mutex != NULL) {
         vSemaphoreDelete(s_console_mutex);
         s_console_mutex = NULL;
     }
     
     s_is_initialized = false;
     return ESP_OK;
 }
 
 /**
  * @brief Set the console logging level
  * 
  * @param level Log level to set
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_set_level(console_log_level_t level) {
     if (!s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (level > CONSOLE_LOG_VERBOSE) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     s_config.log_level = level;
     
     // Update ESP-IDF log level if we're not redirecting
     if (!s_config.redirect_esp_logs) {
         esp_log_level_set("*", console_level_to_esp_level(level));
     }
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get the current console logging level
  * 
  * @param level Pointer to store the current log level
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_get_level(console_log_level_t *level) {
     if (!s_is_initialized || level == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     *level = s_config.log_level;
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get statistics for the console logger
  * 
  * @param stats Pointer to statistics structure to fill
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_get_stats(console_logger_stats_t *stats) {
     if (!s_is_initialized || stats == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     memcpy(stats, &s_stats, sizeof(console_logger_stats_t));
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Enable or disable colorized console output
  * 
  * @param enable True to enable colors, false to disable
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_set_color_output(bool enable) {
     if (!s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     s_config.colorize_output = enable;
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Write a log message to the console
  * 
  * @param level Log level of the message
  * @param module Source module identifier
  * @param format Printf-style format string
  * @param ... Variable arguments
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_write(console_log_level_t level, 
                               console_module_t module, 
                               const char *format, ...) {
     if (!s_is_initialized || format == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (level > s_config.log_level || level == CONSOLE_LOG_NONE) {
         return ESP_OK; // Skip logging if level is filtered out
     }
     
     if (module >= CONSOLE_MODULE_MAX) {
         module = CONSOLE_MODULE_MAX - 1; // Use "UNKNOWN" for invalid modules
     }
     
     va_list args;
     va_start(args, format);
     esp_err_t result = console_logger_vwrite(level, module, format, args);
     va_end(args);
     
     return result;
 }
 
 /**
  * @brief Write a log message to the console with va_list arguments
  * 
  * @param level Log level of the message
  * @param module Source module identifier
  * @param format Printf-style format string
  * @param args Variable argument list
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_vwrite(console_log_level_t level, 
                                console_module_t module, 
                                const char *format, 
                                va_list args) {
     if (!s_is_initialized || format == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (level > s_config.log_level || level == CONSOLE_LOG_NONE) {
         return ESP_OK; // Skip logging if level is filtered out
     }
     
     if (module >= CONSOLE_MODULE_MAX) {
         module = CONSOLE_MODULE_MAX - 1; // Use "UNKNOWN" for invalid modules
     }
     
     // Take mutex to ensure thread safety
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Format the header with timestamp and module info
     char timestamp_buf[32] = {0};
     char header[128] = {0};
     
     if (s_config.include_timestamp) {
         format_timestamp(timestamp_buf, sizeof(timestamp_buf));
     }
     
     // Build the log header
     if (s_config.include_timestamp) {
         snprintf(header, sizeof(header), "[%s]", timestamp_buf);
     }
     
     // Add the level and module
     size_t current_len = strlen(header);
     if (current_len > 0) {
         snprintf(header + current_len, sizeof(header) - current_len, " ");
         current_len++;
     }
     
     if (level < sizeof(LOG_LEVEL_TAGS) / sizeof(LOG_LEVEL_TAGS[0])) {
         snprintf(header + current_len, sizeof(header) - current_len, 
                 "[%s] [%s]: ", LOG_LEVEL_TAGS[level], MODULE_NAMES[module]);
     } else {
         snprintf(header + current_len, sizeof(header) - current_len, 
                 "[?] [%s]: ", MODULE_NAMES[module]);
     }
     
     // Apply colors if enabled
     if (s_config.colorize_output && level < sizeof(LOG_LEVEL_COLORS) / sizeof(LOG_LEVEL_COLORS[0])) {
         printf("%s%s", LOG_LEVEL_COLORS[level], header);
     } else {
         printf("%s", header);
     }
     
     // Print the actual message
     vprintf(format, args);
     
     // Reset color if needed
     if (s_config.colorize_output) {
         printf("%s", COLOR_RESET);
     }
     
     // Always add a newline if not already present
     if (format[strlen(format)-1] != '\n') {
         printf("\n");
     }
     
     // Update statistics
     s_stats.messages_logged++;
     s_stats.bytes_output += strlen(header);
     
     // Optional flush
     if (s_config.auto_flush) {
         fflush(stdout);
     }
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Create a horizontal separator line in the console
  * 
  * @param c Character to use for the separator
  * @param length Length of the separator
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_separator(char c, uint8_t length) {
     if (!s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (length == 0 || length > 120) {
         length = 80; // Default length for separator
     }
     
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     for (uint8_t i = 0; i < length; i++) {
         putchar(c);
     }
     putchar('\n');
     
     if (s_config.auto_flush) {
         fflush(stdout);
     }
     
     s_stats.messages_logged++;
     s_stats.bytes_output += length + 1;
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Format current time into a timestamp string
  * 
  * @param buffer Buffer to store the formatted timestamp
  * @param size Size of the buffer
  */
 static void format_timestamp(char *buffer, size_t size) {
     // Get current time since boot in milliseconds
     uint32_t uptime_ms = pdTICKS_TO_MS(xTaskGetTickCount());
     uint32_t hours = uptime_ms / 3600000;
     uint32_t minutes = (uptime_ms % 3600000) / 60000;
     uint32_t seconds = (uptime_ms % 60000) / 1000;
     uint32_t milliseconds = uptime_ms % 1000;
     
     snprintf(buffer, size, "%02u:%02u:%02u.%03u", hours, minutes, seconds, milliseconds);
 }
 
 /**
  * @brief Convert ESP-IDF log level to console log level
  * 
  * @param level ESP-IDF log level
  * @return console_log_level_t Corresponding console log level
  */
 static console_log_level_t esp_level_to_console_level(esp_log_level_t level) {
     switch (level) {
         case ESP_LOG_NONE:    return CONSOLE_LOG_NONE;
         case ESP_LOG_ERROR:   return CONSOLE_LOG_ERROR;
         case ESP_LOG_WARN:    return CONSOLE_LOG_WARN;
         case ESP_LOG_INFO:    return CONSOLE_LOG_INFO;
         case ESP_LOG_DEBUG:   return CONSOLE_LOG_DEBUG;
         case ESP_LOG_VERBOSE: return CONSOLE_LOG_VERBOSE;
         default:              return CONSOLE_LOG_INFO;
     }
 }
 
 /**
  * @brief Convert console log level to ESP-IDF log level
  * 
  * @param level Console log level
  * @return esp_log_level_t Corresponding ESP-IDF log level
  */
 static esp_log_level_t console_level_to_esp_level(console_log_level_t level) {
     switch (level) {
         case CONSOLE_LOG_NONE:    return ESP_LOG_NONE;
         case CONSOLE_LOG_ERROR:   return ESP_LOG_ERROR;
         case CONSOLE_LOG_WARN:    return ESP_LOG_WARN;
         case CONSOLE_LOG_INFO:    return ESP_LOG_INFO;
         case CONSOLE_LOG_DEBUG:   return ESP_LOG_DEBUG;
         case CONSOLE_LOG_VERBOSE: return ESP_LOG_VERBOSE;
         default:                  return ESP_LOG_INFO;
     }
 }
 
 /**
  * @brief Hook function for ESP-IDF logging system
  * 
  * @param tag Original ESP log tag
  * @param level ESP log level
  * @param fmt Format string
  * @param args Variable arguments
  */
 static void esp_log_hook(const char *tag, esp_log_level_t level, 
                         const char *fmt, va_list args) {
     if (tag == NULL || fmt == NULL) {
         return;
     }
     
     // Convert ESP level to console level
     console_log_level_t console_level = esp_level_to_console_level(level);
     
     // Skip if level is filtered
     if (console_level > s_config.log_level || console_level == CONSOLE_LOG_NONE) {
         return;
     }
     
     // Determine module from tag, default to SYSTEM
     console_module_t module = CONSOLE_MODULE_SYSTEM;
     
     // Map common ESP-IDF tags to our modules
     if (strstr(tag, "wifi") || strstr(tag, "net")) {
         module = CONSOLE_MODULE_SYSTEM;
     } else if (strstr(tag, "MQTT") || strstr(tag, "AWS")) {
         module = CONSOLE_MODULE_CLOUD;
     } else if (strstr(tag, "mod") || strstr(tag, "bus")) {
         module = CONSOLE_MODULE_MODBUS;
     } else if (strstr(tag, "can")) {
         module = CONSOLE_MODULE_CANBUS;
     } else if (strstr(tag, "sd") || strstr(tag, "fs")) {
         module = CONSOLE_MODULE_DATA_LOGGER;
     }
     
     // Format for our console logger
     char new_format[256];
     snprintf(new_format, sizeof(new_format), "[ESP:%s] %s", tag, fmt);
     
     // Use our logging function
     console_logger_vwrite(console_level, module, new_format, args);
 }
 
 /**
  * @brief Flush the console output buffer
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_flush(void) {
     if (!s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     fflush(stdout);
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Write system health summary to console
  * 
  * @param free_heap Free heap memory in bytes
  * @param uptime_s System uptime in seconds
  * @param temp_c CPU temperature in Celsius
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_system_health(uint32_t free_heap, 
                                       uint32_t uptime_s, 
                                       float temp_c) {
     if (!s_is_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create a health report
     char report[128];
     snprintf(report, sizeof(report), 
              "System Health: Heap: %u bytes | Uptime: %u sec | Temp: %.1f°C",
              free_heap, uptime_s, temp_c);
     
     return console_logger_write(CONSOLE_LOG_INFO, CONSOLE_MODULE_SYSTEM, "%s", report);
 }
 
 /**
  * @brief Write system startup banner to console
  * 
  * @param version Firmware version string
  * @param build_date Build date string
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t console_logger_startup_banner(const char *version, const char *build_date) {
     if (!s_is_initialized || version == NULL || build_date == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if we can take the mutex
     if (xSemaphoreTake(s_console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Apply color if enabled
     if (s_config.colorize_output) {
         printf("\033[1;36m"); // Bright cyan
     }
     
     printf("\n");
     console_logger_separator('=', 80);
     printf("  Battery Energy Storage System (BESS) - 100KW/200KWH\n");
     printf("  Firmware Version: %s\n", version);
     printf("  Build Date: %s\n", build_date);
     printf("  ESP32-P4 Platform with FreeRTOS\n");
     console_logger_separator('=', 80);
     printf("\n");
     
     // Reset color if needed
     if (s_config.colorize_output) {
         printf("%s", COLOR_RESET);
     }
     
     if (s_config.auto_flush) {
         fflush(stdout);
     }
     
     // Update statistics
     s_stats.messages_logged++;
     s_stats.bytes_output += 250; // Approximate
     
     xSemaphoreGive(s_console_mutex);
     return ESP_OK;
 }