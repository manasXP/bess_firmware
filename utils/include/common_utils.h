/**
 * @file common_utils.h
 * @brief Common utilities for the BESS firmware
 * 
 * This file contains common utilities, macros, and helper functions
 * used throughout the BESS (Battery Energy Storage System) firmware.
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef COMMON_UTILS_H
 #define COMMON_UTILS_H
 
 #include <stdio.h>
 #include <stdint.h>
 #include <stdbool.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "esp_err.h"
 #include "esp_log.h"
 #include "sdkconfig.h"
 
 /**
  * @brief BESS firmware version information
  */
 #define BESS_FIRMWARE_VERSION_MAJOR    1
 #define BESS_FIRMWARE_VERSION_MINOR    0
 #define BESS_FIRMWARE_VERSION_PATCH    0
 
 /**
  * @brief System configuration constants for a 100KW/200KWH BESS with LFP battery modules
  */
 #define BESS_SYSTEM_POWER_KW           100
 #define BESS_SYSTEM_CAPACITY_KWH       200
 #define BESS_MODULE_VOLTAGE_V          48
 #define BESS_MODULE_CAPACITY_KWH       16
 #define BESS_MAX_MODULES               16  // Maximum number of battery modules the system can handle
 
 /**
  * @brief Communication protocol constants
  */
 #define BESS_MODBUS_ENABLED            1
 #define BESS_CANBUS_ENABLED            1
 
 /**
  * @brief Log levels matching ESP-IDF's log levels
  */
 typedef enum {
     BESS_LOG_NONE = 0,
     BESS_LOG_ERROR,
     BESS_LOG_WARN,
     BESS_LOG_INFO,
     BESS_LOG_DEBUG,
     BESS_LOG_VERBOSE
 } bess_log_level_t;
 
 /**
  * @brief Log destinations
  */
 typedef enum {
     BESS_LOG_DEST_NONE   = 0x00,
     BESS_LOG_DEST_CONSOLE = 0x01,
     BESS_LOG_DEST_SDCARD  = 0x02,
     BESS_LOG_DEST_CLOUD   = 0x04,
     BESS_LOG_DEST_ALL     = 0x07
 } bess_log_destination_t;
 
 /**
  * @brief BESS system events
  */
 typedef enum {
     BESS_EVENT_NONE = 0,
     BESS_EVENT_ERROR,
     BESS_EVENT_WARNING,
     BESS_EVENT_STATE_CHANGE,
     BESS_EVENT_MODULE_STATUS,
     BESS_EVENT_SOC_THRESHOLD,
     BESS_EVENT_TEMPERATURE_THRESHOLD,
     BESS_EVENT_SYSTEM_STARTUP,
     BESS_EVENT_SYSTEM_SHUTDOWN,
     BESS_EVENT_COMMUNICATION_ERROR,
     BESS_EVENT_MAX
 } bess_event_t;
 
 /**
  * @brief BESS operating states
  */
 typedef enum {
     BESS_STATE_INIT = 0,
     BESS_STATE_STANDBY,
     BESS_STATE_CHARGING,
     BESS_STATE_DISCHARGING,
     BESS_STATE_ERROR,
     BESS_STATE_MAINTENANCE,
     BESS_STATE_SHUTDOWN
 } bess_state_t;
 
 /**
  * @brief Error codes specific to the BESS system
  */
 typedef enum {
     BESS_OK = 0,
     BESS_ERROR_INVALID_PARAMETER = 0x1000,
     BESS_ERROR_TIMEOUT,
     BESS_ERROR_COMMUNICATION,
     BESS_ERROR_RESOURCE_ALLOCATION,
     BESS_ERROR_NOT_INITIALIZED,
     BESS_ERROR_ALREADY_INITIALIZED,
     BESS_ERROR_MODULE_OFFLINE,
     BESS_ERROR_SAFETY_VIOLATION,
     BESS_ERROR_SYSTEM_FAULT,
     BESS_ERROR_FIRMWARE_MISMATCH,
     BESS_ERROR_UNKNOWN
 } bess_error_t;
 
 /**
  * @brief Event callback function type
  */
 typedef void (*bess_event_callback_t)(bess_event_t event, uint32_t data, void *user_data);
 
 /**
  * @brief Battery module data structure
  */
 typedef struct {
     uint8_t id;                 // Module ID
     float voltage;              // Module voltage (V)
     float current;              // Module current (A)
     float soc;                  // State of Charge (%)
     float soh;                  // State of Health (%)
     float temp_max;             // Maximum temperature (°C)
     float temp_min;             // Minimum temperature (°C)
     float temp_avg;             // Average temperature (°C)
     uint16_t cell_voltages[16]; // Individual cell voltages (mV)
     uint16_t status;            // Module status bitfield
     uint16_t error_flags;       // Error flags
     bool is_balancing;          // Whether cell balancing is active
     bool is_online;             // Whether the module is online
 } bess_module_data_t;
 
 /**
  * @brief System status structure
  */
 typedef struct {
     bess_state_t state;         // Current system state
     float system_voltage;       // System voltage (V)
     float system_current;       // System current (A)
     float system_power;         // System power (kW)
     float system_soc;           // System state of charge (%)
     float system_soh;           // System state of health (%)
     float max_temp;             // Maximum temperature across all modules (°C)
     uint8_t active_modules;     // Number of active modules
     uint32_t uptime;            // System uptime in seconds
     uint16_t status_flags;      // Status flags
     uint16_t error_flags;       // Error flags
     uint32_t total_energy_in;   // Total energy charged (kWh)
     uint32_t total_energy_out;  // Total energy discharged (kWh)
 } bess_system_status_t;
 
 /**
  * @brief Initialize the logging system
  * 
  * @param level The maximum log level to display
  * @param destinations Bitwise combination of log destinations
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t bess_log_init(bess_log_level_t level, uint8_t destinations);
 
 /**
  * @brief Set the log level
  * 
  * @param level The new log level
  */
 void bess_log_set_level(bess_log_level_t level);
 
 /**
  * @brief Set the log destinations
  * 
  * @param destinations Bitwise combination of log destinations
  */
 void bess_log_set_destinations(uint8_t destinations);
 
 /**
  * @brief Log a message to all enabled destinations
  * 
  * @param level Log level of the message
  * @param tag Module tag
  * @param format Format string
  * @param ... Variable arguments
  */
 void bess_log(bess_log_level_t level, const char *tag, const char *format, ...);
 
 /**
  * @brief Register a callback for a specific event
  * 
  * @param event The event to register for
  * @param callback The callback function
  * @param user_data User data to pass to the callback
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t bess_register_event_callback(bess_event_t event, bess_event_callback_t callback, void *user_data);
 
 /**
  * @brief Unregister a callback for a specific event
  * 
  * @param event The event to unregister from
  * @param callback The callback function to unregister
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t bess_unregister_event_callback(bess_event_t event, bess_event_callback_t callback);
 
 /**
  * @brief Create a mutex-protected memory block
  * 
  * @param size Size of the memory block in bytes
  * @return void* Pointer to the allocated memory, or NULL on failure
  */
 void* bess_create_protected_memory(size_t size);
 
 /**
  * @brief Free a protected memory block
  * 
  * @param ptr Pointer to the memory block
  */
 void bess_free_protected_memory(void *ptr);
 
 /**
  * @brief Get the firmware version string
  * 
  * @return const char* Version string
  */
 const char* bess_get_firmware_version(void);
 
 /**
  * @brief Convert a BESS error code to a string
  * 
  * @param err The error code
  * @return const char* Error string
  */
 const char* bess_error_to_string(bess_error_t err);
 
 /**
  * @brief Convert a BESS state to a string
  * 
  * @param state The state
  * @return const char* State string
  */
 const char* bess_state_to_string(bess_state_t state);
 
 /**
  * @brief Macro for logging errors
  */
 #define BESS_LOGE(tag, format, ...) bess_log(BESS_LOG_ERROR, tag, format, ##__VA_ARGS__)
 
 /**
  * @brief Macro for logging warnings
  */
 #define BESS_LOGW(tag, format, ...) bess_log(BESS_LOG_WARN, tag, format, ##__VA_ARGS__)
 
 /**
  * @brief Macro for logging information
  */
 #define BESS_LOGI(tag, format, ...) bess_log(BESS_LOG_INFO, tag, format, ##__VA_ARGS__)
 
 /**
  * @brief Macro for logging debug messages
  */
 #define BESS_LOGD(tag, format, ...) bess_log(BESS_LOG_DEBUG, tag, format, ##__VA_ARGS__)
 
 /**
  * @brief Macro for logging verbose messages
  */
 #define BESS_LOGV(tag, format, ...) bess_log(BESS_LOG_VERBOSE, tag, format, ##__VA_ARGS__)
 
 /**
  * @brief Check if an expression is true, otherwise return an error
  */
 #define BESS_CHECK(expr, err_code) do { \
     if (!(expr)) { \
         return err_code; \
     } \
 } while (0)
 
 /**
  * @brief Check if an expression is true, otherwise goto a label
  */
 #define BESS_CHECK_GOTO(expr, err_code, label) do { \
     if (!(expr)) { \
         ret = err_code; \
         goto label; \
     } \
 } while (0)
 
 /**
  * @brief Convert ESP error code to BESS error code
  */
 #define BESS_ERROR_FROM_ESP(esp_err) ((esp_err) ? BESS_ERROR_UNKNOWN : BESS_OK)
 
 /**
  * @brief Safe version of strncpy that ensures null-termination
  * 
  * @param dest Destination buffer
  * @param src Source string
  * @param size Size of the destination buffer
  * @return char* Pointer to the destination buffer
  */
 char* bess_strncpy(char *dest, const char *src, size_t size);
 
 /**
  * @brief Create a string copy on the heap
  * 
  * @param str String to copy
  * @return char* Pointer to the copied string, or NULL on failure
  */
 char* bess_strdup(const char *str);
 
 #endif /* COMMON_UTILS_H */