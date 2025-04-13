/**
 * @file system_manager.h
 * @brief Main system manager for 100KW/200KWH BESS controller
 *
 * This component serves as the central coordinator for the Battery Energy Storage System (BESS),
 * integrating the Battery Management System (BMS), communication interfaces (Modbus, CANBus),
 * and logging subsystems. It manages the overall system state and orchestrates interactions
 * between various subsystems.
 * 
 * Hardware: ESP32-P4
 * RTOS: FreeRTOS
 */

 #ifndef SYSTEM_MANAGER_H
 #define SYSTEM_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/event_groups.h"
 #include "freertos/semphr.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief System operating modes
  */
 typedef enum {
     BESS_MODE_STANDBY = 0,       /*!< System in standby, minimal operations */
     BESS_MODE_CHARGING,          /*!< System in charging mode */
     BESS_MODE_DISCHARGING,       /*!< System in discharging mode */
     BESS_MODE_FAULT,             /*!< System in fault state */
     BESS_MODE_MAINTENANCE,       /*!< System in maintenance mode */
     BESS_MODE_CALIBRATION,       /*!< System in calibration mode */
     BESS_MODE_EMERGENCY_SHUTDOWN /*!< Emergency shutdown active */
 } bess_system_mode_t;
 
 /**
  * @brief System status flags
  */
 typedef enum {
     BESS_STATUS_INITIALIZED      = (1 << 0),  /*!< System initialized */
     BESS_STATUS_BMS_ACTIVE       = (1 << 1),  /*!< BMS subsystem active */
     BESS_STATUS_MODBUS_ACTIVE    = (1 << 2),  /*!< Modbus communication active */
     BESS_STATUS_CANBUS_ACTIVE    = (1 << 3),  /*!< CANBus communication active */
     BESS_STATUS_LOGGING_ACTIVE   = (1 << 4),  /*!< Logging subsystem active */
     BESS_STATUS_GRID_CONNECTED   = (1 << 5),  /*!< Connected to grid */
     BESS_STATUS_BALANCING_ACTIVE = (1 << 6),  /*!< Cell balancing is active */
     BESS_STATUS_COOLING_ACTIVE   = (1 << 7),  /*!< Cooling system active */
     BESS_STATUS_ALARM_ACTIVE     = (1 << 8),  /*!< One or more alarms active */
     BESS_STATUS_SD_CARD_PRESENT  = (1 << 9),  /*!< SD card detected and mounted */
     BESS_STATUS_WIFI_CONNECTED   = (1 << 10), /*!< WiFi connection established */
     BESS_STATUS_AWS_CONNECTED    = (1 << 11), /*!< Connected to AWS IoT */
     BESS_STATUS_TIME_SYNCED      = (1 << 12)  /*!< System time synchronized */
 } bess_system_status_flags_t;
 
 /**
  * @brief System alarm and error types
  */
 typedef enum {
     BESS_ALARM_NONE = 0,                /*!< No alarm */
     BESS_ALARM_OVERCURRENT,             /*!< Overcurrent detected */
     BESS_ALARM_OVERVOLTAGE,             /*!< Overvoltage detected */
     BESS_ALARM_UNDERVOLTAGE,            /*!< Undervoltage detected */
     BESS_ALARM_OVERTEMPERATURE,         /*!< Overtemperature detected */
     BESS_ALARM_COMMUNICATION_ERROR,     /*!< Communication error */
     BESS_ALARM_BMS_ERROR,               /*!< BMS subsystem error */
     BESS_ALARM_THERMAL_RUNAWAY,         /*!< Thermal runaway detected */
     BESS_ALARM_CELL_IMBALANCE,          /*!< Excessive cell imbalance */
     BESS_ALARM_SOC_LOW,                 /*!< State of Charge low */
     BESS_ALARM_SOC_CRITICAL,            /*!< State of Charge critically low */
     BESS_ALARM_GRID_FAULT,              /*!< Grid connection fault */
     BESS_ALARM_INTERNAL_ERROR,          /*!< Internal system error */
     BESS_ALARM_WATCHDOG_RESET,          /*!< System reset by watchdog */
     BESS_ALARM_SENSOR_FAULT,            /*!< Sensor failure detected */
     BESS_ALARM_SD_CARD_ERROR,           /*!< SD card error */
     BESS_ALARM_CLOUD_CONNECTION_ERROR,  /*!< Error connecting to cloud services */
     BESS_ALARM_CONFIG_ERROR,            /*!< Configuration error */
     BESS_ALARM_COOLING_SYSTEM_FAILURE,  /*!< Cooling system failure */
     BESS_ALARM_EMERGENCY_STOP_ACTIVATED /*!< Emergency stop activated */
 } bess_alarm_type_t;
 
 /**
  * @brief System information structure
  */
 typedef struct {
     bess_system_mode_t mode;            /*!< Current system mode */
     uint32_t status_flags;              /*!< System status flags */
     float system_soc;                   /*!< Overall system State of Charge (%) */
     float system_soh;                   /*!< Overall system State of Health (%) */
     float max_temperature;              /*!< Maximum temperature (°C) */
     float min_temperature;              /*!< Minimum temperature (°C) */
     float avg_temperature;              /*!< Average temperature (°C) */
     float power_output;                 /*!< Current power output (kW) */
     float energy_remaining;             /*!< Estimated energy remaining (kWh) */
     uint32_t uptime;                    /*!< System uptime in seconds */
     uint32_t active_alarms;             /*!< Bitmap of active alarms */
     uint16_t module_count;              /*!< Number of active battery modules */
     uint16_t firmware_version;          /*!< Firmware version */
     char system_id[32];                 /*!< System identifier */
 } bess_system_info_t;
 
 /**
  * @brief Log destination options
  */
 typedef enum {
     BESS_LOG_CONSOLE     = (1 << 0), /*!< Log to console */
     BESS_LOG_SD_CARD     = (1 << 1), /*!< Log to SD card */
     BESS_LOG_CLOUDWATCH  = (1 << 2), /*!< Log to AWS CloudWatch */
     BESS_LOG_ALL         = 0xFF      /*!< Log to all available destinations */
 } bess_log_destination_t;
 
 /**
  * @brief System event data
  */
 typedef struct {
     uint32_t event_id;    /*!< Unique event identifier */
     uint32_t timestamp;   /*!< Timestamp of the event */
     uint8_t event_type;   /*!< Type of event */
     uint8_t severity;     /*!< Event severity */
     uint16_t module_id;   /*!< Related module ID, if applicable */
     uint32_t data;        /*!< Event-specific data */
 } bess_event_data_t;
 
 /**
  * @brief System event callback function definition
  */
 typedef void (*bess_event_callback_t)(bess_event_data_t event, void *user_data);
 
 /**
  * @brief Communication statistics
  */
 typedef struct {
     uint32_t modbus_tx_count;      /*!< Modbus transmitted packet count */
     uint32_t modbus_rx_count;      /*!< Modbus received packet count */
     uint32_t modbus_error_count;   /*!< Modbus error count */
     uint32_t canbus_tx_count;      /*!< CANBus transmitted packet count */
     uint32_t canbus_rx_count;      /*!< CANBus received packet count */
     uint32_t canbus_error_count;   /*!< CANBus error count */
     uint32_t wifi_tx_bytes;        /*!< WiFi transmitted bytes */
     uint32_t wifi_rx_bytes;        /*!< WiFi received bytes */
     uint32_t cloud_tx_count;       /*!< Cloud transmitted message count */
     uint32_t cloud_rx_count;       /*!< Cloud received message count */
     uint32_t cloud_error_count;    /*!< Cloud error count */
 } bess_comm_stats_t;
 
 /**
  * @brief System configuration
  */
 typedef struct {
     uint16_t module_count;                /*!< Number of battery modules */
     uint32_t system_capacity_kwh;         /*!< Total system capacity in kWh */
     uint32_t system_power_kw;             /*!< Maximum system power in kW */
     uint16_t max_charging_current;        /*!< Maximum charging current in A */
     uint16_t max_discharging_current;     /*!< Maximum discharging current in A */
     float max_cell_voltage;               /*!< Maximum cell voltage */
     float min_cell_voltage;               /*!< Minimum cell voltage */
     float balance_threshold_mv;           /*!< Cell balancing threshold in mV */
     float temperature_warning_threshold;  /*!< Temperature warning threshold in °C */
     float temperature_critical_threshold; /*!< Temperature critical threshold in °C */
     float temperature_emergency_threshold; /*!< Temperature emergency threshold in °C */
     uint16_t modbus_slave_address;        /*!< Modbus slave address */
     uint32_t modbus_baud_rate;            /*!< Modbus baud rate */
     uint32_t canbus_bitrate;              /*!< CANBus bitrate */
     uint16_t canbus_node_id;              /*!< CANBus node ID */
     uint32_t log_level;                   /*!< Default logging level */
     uint32_t log_destinations;            /*!< Default logging destinations */
     char aws_endpoint[128];               /*!< AWS IoT endpoint */
     char aws_client_id[64];               /*!< AWS IoT client ID */
     char system_name[64];                 /*!< System name/identifier */
 } bess_system_config_t;
 
 /**
  * @brief Initialize the system manager
  * 
  * @param config Pointer to system configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_init(const bess_system_config_t *config);
 
 /**
  * @brief Start the system manager and all subsystems
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_start(void);
 
 /**
  * @brief Stop the system manager and all subsystems
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_stop(void);
 
 /**
  * @brief Set the system operating mode
  * 
  * @param mode Target operating mode
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_set_mode(bess_system_mode_t mode);
 
 /**
  * @brief Get the current system information
  * 
  * @param info Pointer to structure to receive system information
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_info(bess_system_info_t *info);
 
 /**
  * @brief Get communication statistics
  * 
  * @param stats Pointer to structure to receive communication statistics
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_comm_stats(bess_comm_stats_t *stats);
 
 /**
  * @brief Reset communication statistics
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_reset_comm_stats(void);
 
 /**
  * @brief Configure the logging system
  * 
  * @param level Logging level (ESP_LOG_* values)
  * @param destinations Bitmap of destination flags
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_configure_logging(uint32_t level, uint32_t destinations);
 
 /**
  * @brief Log a message to configured destinations
  * 
  * @param tag Log tag
  * @param level Log level
  * @param format Format string
  * @param ... Variable arguments
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_log(const char *tag, uint32_t level, const char *format, ...);
 
 /**
  * @brief Register a callback for system events
  * 
  * @param event_mask Bitmap of events to subscribe to
  * @param callback Callback function
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_register_event_callback(
     uint32_t event_mask,
     bess_event_callback_t callback,
     void *user_data);
 
 /**
  * @brief Unregister a previously registered event callback
  * 
  * @param callback Callback function to unregister
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_unregister_event_callback(bess_event_callback_t callback);
 
 /**
  * @brief Trigger a system alarm
  * 
  * @param alarm_type Type of alarm to trigger
  * @param module_id Module ID related to the alarm (if applicable)
  * @param data Additional alarm-specific data
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_trigger_alarm(
     bess_alarm_type_t alarm_type,
     uint16_t module_id,
     uint32_t data);
 
 /**
  * @brief Clear a system alarm
  * 
  * @param alarm_type Type of alarm to clear
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_clear_alarm(bess_alarm_type_t alarm_type);
 
 /**
  * @brief Get active alarms
  * 
  * @param alarm_bitmap Pointer to receive bitmap of active alarms
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_active_alarms(uint32_t *alarm_bitmap);
 
 /**
  * @brief Run system diagnostics
  * 
  * @param detailed Set to true for detailed diagnostics
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_run_diagnostics(bool detailed);
 
 /**
  * @brief Synchronize system time with an external time source
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_sync_time(void);
 
 /**
  * @brief Update system configuration
  * 
  * @param config Pointer to new configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_update_config(const bess_system_config_t *config);
 
 /**
  * @brief Save current configuration to non-volatile storage
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_save_config(void);
 
 /**
  * @brief Load configuration from non-volatile storage
  * 
  * @param config Pointer to receive loaded configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_load_config(bess_system_config_t *config);
 
 /**
  * @brief Perform a controlled system shutdown
  * 
  * @param reason Shutdown reason
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_shutdown(const char *reason);
 
 /**
  * @brief Perform a controlled system reboot
  * 
  * @param reason Reboot reason
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_reboot(const char *reason);
 
 /**
  * @brief Execute emergency shutdown procedure
  * 
  * @param reason Emergency shutdown reason
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_emergency_shutdown(const char *reason);
 
 /**
  * @brief Get the handler for the BMS subsystem
  * 
  * @param handler Pointer to receive the BMS handler
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_bms_handler(void **handler);
 
 /**
  * @brief Get the handler for the Modbus subsystem
  * 
  * @param handler Pointer to receive the Modbus handler
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_modbus_handler(void **handler);
 
 /**
  * @brief Get the handler for the CANBus subsystem
  * 
  * @param handler Pointer to receive the CANBus handler
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_canbus_handler(void **handler);
 
 /**
  * @brief Send data to the cloud (AWS IoT)
  * 
  * @param topic Topic string
  * @param data Data to send
  * @param data_len Length of data
  * @param qos Quality of Service level
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_send_to_cloud(
     const char *topic,
     const void *data,
     size_t data_len,
     int qos);
 
 /**
  * @brief Get system version information
  * 
  * @param version_str Buffer to receive version string
  * @param max_len Maximum length of version string
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t system_manager_get_version(char *version_str, size_t max_len);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* SYSTEM_MANAGER_H */