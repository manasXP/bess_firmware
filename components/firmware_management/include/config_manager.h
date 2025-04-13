/**
 * @file config_manager.h
 * @brief Configuration management for 100KW/200KWH BESS firmware
 * 
 * This module provides configuration management for the BESS firmware.
 * It handles loading, saving, and accessing configuration parameters
 * for the Battery Energy Storage System with LFP battery modules.
 * 
 * The configuration can be stored in NVS (Non-Volatile Storage),
 * SD card, or loaded from default values. Configuration changes
 * can be made via CLI, API, or remote management interfaces.
 *
 * @copyright Copyright (c) 2025
 */

 #ifndef CONFIG_MANAGER_H
 #define CONFIG_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/semphr.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Configuration storage locations
  */
 typedef enum {
     CONFIG_STORAGE_NVS,      /**< Configuration stored in ESP32 NVS */
     CONFIG_STORAGE_SD_CARD,  /**< Configuration stored on SD card */
     CONFIG_STORAGE_DEFAULT   /**< Use default configuration values */
 } config_storage_t;
 
 /**
  * @brief BESS system operation modes
  */
 typedef enum {
     BESS_MODE_NORMAL,        /**< Normal operation mode */
     BESS_MODE_MAINTENANCE,   /**< Maintenance mode with limited functionality */
     BESS_MODE_CALIBRATION,   /**< Calibration mode for sensors and SoC */
     BESS_MODE_DIAGNOSTIC,    /**< Diagnostic mode for system checks */
     BESS_MODE_EMERGENCY      /**< Emergency mode for critical situations */
 } bess_operation_mode_t;
 
 /**
  * @brief Logging level configuration options
  */
 typedef enum {
     LOG_LEVEL_NONE,          /**< No logging */
     LOG_LEVEL_ERROR,         /**< Error messages only */
     LOG_LEVEL_WARNING,       /**< Errors and warnings */
     LOG_LEVEL_INFO,          /**< Normal operational messages */
     LOG_LEVEL_DEBUG,         /**< Detailed debug information */
     LOG_LEVEL_VERBOSE        /**< Very detailed debug information */
 } log_level_t;
 
 /**
  * @brief Logging destination options
  */
 typedef enum {
     LOG_DEST_NONE = 0,               /**< No logging */
     LOG_DEST_CONSOLE = (1 << 0),     /**< Log to console/UART */
     LOG_DEST_SD_CARD = (1 << 1),     /**< Log to SD card */
     LOG_DEST_CLOUD = (1 << 2),       /**< Log to AWS CloudWatch */
     LOG_DEST_ALL = 0xFF              /**< Log to all available destinations */
 } log_destination_t;
 
 /**
  * @brief Communication interface configuration
  */
 typedef struct {
     struct {
         bool enabled;                 /**< Enable/disable Modbus */
         uint16_t port;               /**< TCP port for Modbus TCP */
         uint8_t slave_address;        /**< Slave address for Modbus RTU */
         uint32_t baud_rate;          /**< Baud rate for serial communication */
         uint8_t uart_num;            /**< UART port number */
         uint8_t parity;              /**< Parity setting (0: none, 1: odd, 2: even) */
         uint8_t stop_bits;           /**< Number of stop bits */
         uint8_t data_bits;           /**< Number of data bits */
     } modbus;
 
     struct {
         bool enabled;                 /**< Enable/disable CANBus */
         uint32_t baud_rate;          /**< CANBus baud rate */
         uint8_t tx_pin;              /**< CANBus TX pin */
         uint8_t rx_pin;              /**< CANBus RX pin */
         bool acceptance_filter;       /**< Enable/disable acceptance filter */
         uint32_t acceptance_code;     /**< Acceptance code for filtering */
         uint32_t acceptance_mask;     /**< Acceptance mask for filtering */
         uint8_t operating_mode;       /**< CAN operating mode (0: normal, 1: listen only) */
     } canbus;
 } comm_config_t;
 
 /**
  * @brief Logging configuration
  */
 typedef struct {
     log_level_t level;                /**< Logging level */
     uint8_t destinations;             /**< Bitmap of log_destination_t values */
     uint32_t max_file_size;           /**< Maximum log file size in KB */
     uint8_t max_files;                /**< Maximum number of log files for rotation */
     bool timestamp_enabled;           /**< Include timestamps in log messages */
     bool level_enabled;               /**< Include log level in messages */
     bool component_enabled;           /**< Include component name in messages */
     
     struct {
         char endpoint[128];           /**< AWS CloudWatch endpoint */
         char region[32];              /**< AWS region */
         char log_group[64];           /**< CloudWatch log group */
         char log_stream[64];          /**< CloudWatch log stream */
         uint16_t batch_size;          /**< Number of logs to batch before sending */
         uint16_t max_buffer;          /**< Maximum buffer size for pending logs */
         uint16_t send_interval_ms;    /**< Interval for sending batched logs (ms) */
     } cloud;
 } logging_config_t;
 
 /**
  * @brief Battery system configuration
  */
 typedef struct {
     uint8_t num_modules;              /**< Number of battery modules in the system */
     float module_nominal_voltage;     /**< Nominal voltage of each module (V) */
     float module_capacity;            /**< Nominal capacity of each module (kWh) */
     uint8_t cells_per_module;         /**< Number of cells per module */
     float cell_nominal_voltage;       /**< Nominal voltage of each cell (V) */
     float system_voltage;             /**< Total system voltage (V) */
     float system_capacity;            /**< Total system capacity (kWh) */
     float max_charge_current;         /**< Maximum charging current (A) */
     float max_discharge_current;      /**< Maximum discharging current (A) */
     float max_power_kw;               /**< Maximum system power (kW) */
 } battery_config_t;
 
 /**
  * @brief Protection thresholds for battery safety
  */
 typedef struct {
     struct {
         float cell_min;               /**< Minimum cell voltage (V) */
         float cell_max;               /**< Maximum cell voltage (V) */
         float module_min;             /**< Minimum module voltage (V) */
         float module_max;             /**< Maximum module voltage (V) */
         float system_min;             /**< Minimum system voltage (V) */
         float system_max;             /**< Maximum system voltage (V) */
         float hysteresis;             /**< Hysteresis for reset (V) */
     } voltage;
     
     struct {
         float min;                    /**< Minimum temperature (°C) */
         float max;                    /**< Maximum temperature (°C) */
         float max_gradient;           /**< Maximum temperature rise rate (°C/min) */
         float hysteresis;             /**< Hysteresis for reset (°C) */
     } temperature;
     
     struct {
         float charge_max;             /**< Maximum charge current (A) */
         float discharge_max;          /**< Maximum discharge current (A) */
         float short_circuit;          /**< Short circuit current threshold (A) */
         float hysteresis;             /**< Hysteresis for reset (A) */
         uint16_t response_time_ms;    /**< Overcurrent response time (ms) */
     } current;
     
     struct {
         uint8_t soc_min;              /**< Minimum State of Charge (%) */
         uint8_t soc_max;              /**< Maximum State of Charge (%) */
         uint8_t hysteresis;           /**< Hysteresis for reset (%) */
     } soc;
 } protection_config_t;
 
 /**
  * @brief SoC calculation configuration
  */
 typedef struct {
     uint8_t algorithm;                /**< SoC algorithm (0: CC, 1: OCV, 2: Kalman, 3: Hybrid) */
     float initial_soc;                /**< Initial SoC at system start (%) */
     float coulomb_efficiency;         /**< Coulomb counting efficiency factor */
     uint16_t ocv_rest_time_s;         /**< Required rest time for OCV measurement (s) */
     uint8_t ocv_rest_current_ma;      /**< Maximum current for rest condition (mA) */
     uint8_t filter_strength;          /**< Kalman filter process noise (1-10) */
     bool temp_compensation;           /**< Enable temperature compensation */
 } soc_config_t;
 
 /**
  * @brief Cell balancing configuration
  */
 typedef struct {
     uint8_t mode;                     /**< Balancing mode (0: none, 1: passive, 2: active, 3: hybrid) */
     float voltage_threshold_mv;       /**< Voltage difference threshold (mV) */
     uint16_t balance_current_ma;      /**< Balancing current (mA) */
     float min_cell_voltage_mv;        /**< Minimum cell voltage for balancing (mV) */
     uint32_t max_balance_time_ms;     /**< Maximum balancing time (ms) */
     uint32_t rest_time_ms;            /**< Rest time between balancing (ms) */
 } balancing_config_t;
 
 /**
  * @brief Thermal management configuration
  */
 typedef struct {
     uint8_t cooling_method;           /**< Cooling method (0: passive, 1: fan, 2: liquid) */
     float elevated_temp;              /**< Elevated temperature threshold (°C) */
     float warning_temp;               /**< Warning temperature threshold (°C) */
     float critical_temp;              /**< Critical temperature threshold (°C) */
     float emergency_temp;             /**< Emergency temperature threshold (°C) */
     float temp_hysteresis;            /**< Temperature hysteresis (°C) */
     float ambient_reference;          /**< Ambient temperature reference (°C) */
     float max_temp_rise_rate;         /**< Maximum temperature rise rate (°C/min) */
 } thermal_config_t;
 
 /**
  * @brief Task configuration for FreeRTOS tasks
  */
 typedef struct {
     struct {
         uint16_t stack_size;          /**< Stack size in words */
         uint8_t priority;             /**< Task priority (1-configMAX_PRIORITIES) */
         uint8_t core_id;              /**< CPU core assignment (-1 for any core) */
         uint16_t interval_ms;         /**< Task execution interval (ms) */
     } bms_monitor;
     
     struct {
         uint16_t stack_size;          /**< Stack size in words */
         uint8_t priority;             /**< Task priority (1-configMAX_PRIORITIES) */
         uint8_t core_id;              /**< CPU core assignment (-1 for any core) */
         uint16_t interval_ms;         /**< Task execution interval (ms) */
     } bms_balancing;
     
     struct {
         uint16_t stack_size;          /**< Stack size in words */
         uint8_t priority;             /**< Task priority (1-configMAX_PRIORITIES) */
         uint8_t core_id;              /**< CPU core assignment (-1 for any core) */
         uint16_t interval_ms;         /**< Task execution interval (ms) */
     } bms_diag;
     
     struct {
         uint16_t stack_size;          /**< Stack size in words */
         uint8_t priority;             /**< Task priority (1-configMAX_PRIORITIES) */
         uint8_t core_id;              /**< CPU core assignment (-1 for any core) */
         uint16_t interval_ms;         /**< Task execution interval (ms) */
     } thermal_monitor;
     
     struct {
         uint16_t stack_size;          /**< Stack size in words */
         uint8_t priority;             /**< Task priority (1-configMAX_PRIORITIES) */
         uint8_t core_id;              /**< CPU core assignment (-1 for any core) */
         uint16_t interval_ms;         /**< Task execution interval (ms) */
     } comms;
     
     struct {
         uint16_t stack_size;          /**< Stack size in words */
         uint8_t priority;             /**< Task priority (1-configMAX_PRIORITIES) */
         uint8_t core_id;              /**< CPU core assignment (-1 for any core) */
         uint16_t interval_ms;         /**< Task execution interval (ms) */
     } logger;
 } task_config_t;
 
 /**
  * @brief System configuration structure
  */
 typedef struct {
     char device_id[32];               /**< Unique device identifier */
     char system_name[64];             /**< System name/description */
     uint8_t firmware_version[3];      /**< Firmware version [major, minor, patch] */
     bess_operation_mode_t mode;       /**< System operation mode */
     
     comm_config_t comm;               /**< Communication configuration */
     logging_config_t logging;         /**< Logging configuration */
     battery_config_t battery;         /**< Battery system configuration */
     protection_config_t protection;   /**< Protection thresholds */
     soc_config_t soc;                 /**< SoC calculation configuration */
     balancing_config_t balancing;     /**< Cell balancing configuration */
     thermal_config_t thermal;         /**< Thermal management configuration */
     task_config_t tasks;              /**< FreeRTOS task configuration */
 } bess_config_t;
 
 /**
  * @brief Callback function type for configuration change notifications
  *
  * @param component_name Name of the component that changed
  * @param user_data User data pointer passed during registration
  */
 typedef void (*config_change_cb_t)(const char *component_name, void *user_data);
 
 /**
  * @brief Initialize the configuration manager
  *
  * @param storage Location to load configuration from
  * @param config_path Path to configuration file (if using SD card)
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_init(config_storage_t storage, const char *config_path);
 
 /**
  * @brief Deinitialize the configuration manager
  *
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_deinit(void);
 
 /**
  * @brief Load configuration from the specified storage
  *
  * @param storage Location to load configuration from
  * @param config_path Path to configuration file (if using SD card)
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_load(config_storage_t storage, const char *config_path);
 
 /**
  * @brief Save current configuration to the specified storage
  *
  * @param storage Location to save configuration to
  * @param config_path Path to configuration file (if using SD card)
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_save(config_storage_t storage, const char *config_path);
 
 /**
  * @brief Get the current configuration
  *
  * @param config Pointer to configuration structure to fill
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_get_config(bess_config_t *config);
 
 /**
  * @brief Update the current configuration
  *
  * @param config Pointer to new configuration
  * @param persist Whether to persist the changes to storage
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_set_config(const bess_config_t *config, bool persist);
 
 /**
  * @brief Register a callback for configuration changes
  *
  * @param component_name Component name to receive notifications for, or NULL for all
  * @param callback Callback function to register
  * @param user_data User data to pass to the callback
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_register_callback(const char *component_name, 
                                           config_change_cb_t callback, 
                                           void *user_data);
 
 /**
  * @brief Unregister a configuration change callback
  *
  * @param component_name Component name the callback was registered for
  * @param callback Callback function to unregister
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_unregister_callback(const char *component_name, 
                                             config_change_cb_t callback);
 
 /**
  * @brief Reset configuration to default values
  *
  * @param persist Whether to persist the default values to storage
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_reset_to_defaults(bool persist);
 
 /**
  * @brief Verify configuration values for validity
  *
  * Checks all configuration parameters against valid ranges
  * and logical constraints.
  *
  * @param config Configuration to verify
  * @return ESP_OK if valid, ESP_ERR_INVALID_ARG if invalid with a log message
  */
 esp_err_t config_manager_verify_config(const bess_config_t *config);
 
 /**
  * @brief Import configuration from JSON string
  *
  * @param json_string JSON string containing configuration
  * @param persist Whether to persist the imported configuration
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_import_json(const char *json_string, bool persist);
 
 /**
  * @brief Export configuration to JSON string
  *
  * @param json_buffer Buffer to store the JSON string
  * @param buffer_size Size of the buffer
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_export_json(char *json_buffer, size_t buffer_size);
 
 /**
  * @brief Get configuration parameter as string
  *
  * Retrieves a configuration parameter by path and converts it to string.
  * Path format: "component.parameter" (e.g., "battery.num_modules")
  *
  * @param param_path Path to the parameter
  * @param value_buffer Buffer to store the string value
  * @param buffer_size Size of the buffer
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_get_param_string(const char *param_path, 
                                          char *value_buffer, 
                                          size_t buffer_size);
 
 /**
  * @brief Set configuration parameter from string
  *
  * Sets a configuration parameter by path from a string value.
  * Path format: "component.parameter" (e.g., "battery.num_modules")
  *
  * @param param_path Path to the parameter
  * @param value String value to set
  * @param persist Whether to persist the change
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t config_manager_set_param_string(const char *param_path, 
                                          const char *value, 
                                          bool persist);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* CONFIG_MANAGER_H */