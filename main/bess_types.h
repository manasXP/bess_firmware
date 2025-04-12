/**
 * @file bess_types.h
 * @brief Data type definitions for Battery Energy Storage System
 * 
 * This file contains the common data types and enumerations used 
 * throughout the BESS firmware.
 */

 #ifndef BESS_TYPES_H
 #define BESS_TYPES_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include <time.h>
 #include "bess_config.h"
 
 /**
  * @brief System operation modes
  */
 typedef enum {
     BESS_MODE_STANDBY,         /**< System is on standby, not active */
     BESS_MODE_CHARGING,        /**< System is charging batteries */
     BESS_MODE_DISCHARGING,     /**< System is discharging batteries */
     BESS_MODE_BALANCING,       /**< System is balancing cells */
     BESS_MODE_MAINTENANCE,     /**< System is in maintenance mode */
     BESS_MODE_ERROR,           /**< System is in error state */
     BESS_MODE_EMERGENCY_STOP   /**< System is in emergency stop state */
 } bess_operation_mode_t;
 
 /**
  * @brief System state flags
  */
 typedef struct {
     bool grid_connected;       /**< Flag indicating if system is connected to grid */
     bool emergency_stop;       /**< Flag indicating emergency stop is active */
     bool contactor_closed;     /**< Flag indicating main contactor status */
     bool cell_balancing_active; /**< Flag indicating cell balancing is active */
     bool charging_permitted;    /**< Flag indicating charging is permitted */
     bool discharging_permitted; /**< Flag indicating discharging is permitted */
     bool maintenance_mode;      /**< Flag indicating maintenance mode is active */
     bool fault_condition;       /**< Flag indicating fault condition is present */
 } bess_system_flags_t;
 
 /**
  * @brief Error code enumeration
  */
 typedef enum {
     BESS_ERROR_NONE = 0,
     BESS_ERROR_OVERCURRENT,
     BESS_ERROR_UNDERVOLTAGE,
     BESS_ERROR_OVERVOLTAGE,
     BESS_ERROR_UNDERTEMPERATURE,
     BESS_ERROR_OVERTEMPERATURE,
     BESS_ERROR_INTERNAL_COMM,
     BESS_ERROR_CELL_IMBALANCE,
     BESS_ERROR_ISOLATION_FAULT,
     BESS_ERROR_GRID_FAULT,
     BESS_ERROR_CONVERTER_FAULT,
     BESS_ERROR_SMOKE_DETECTED,
     BESS_ERROR_EMERGENCY_STOP,
     BESS_ERROR_BMS_FAULT,
     BESS_ERROR_CONTACTOR_FAULT,
     BESS_ERROR_WATCHDOG_TIMEOUT,
     BESS_ERROR_CONFIGURATION,
     BESS_ERROR_SENSOR_FAULT,
     BESS_ERROR_SD_CARD,
     BESS_ERROR_FIRMWARE_CORRUPTION,
     BESS_ERROR_UNKNOWN
 } bess_error_code_t;
 
 /**
  * @brief Severity levels for logging
  */
 typedef enum {
     BESS_LOG_LEVEL_EMERGENCY = 0,
     BESS_LOG_LEVEL_ALERT,
     BESS_LOG_LEVEL_CRITICAL,
     BESS_LOG_LEVEL_ERROR,
     BESS_LOG_LEVEL_WARNING,
     BESS_LOG_LEVEL_NOTICE,
     BESS_LOG_LEVEL_INFO,
     BESS_LOG_LEVEL_DEBUG
 } bess_log_level_t;
 
 /**
  * @brief Battery cell data structure
  */
 typedef struct {
     float voltage;             /**< Cell voltage in V */
     float temperature;         /**< Cell temperature in °C */
     float internal_resistance; /**< Cell internal resistance in mΩ */
     bool balancing_active;     /**< Flag indicating if balancing is active for this cell */
     uint16_t cycle_count;      /**< Cell cycle count */
     uint8_t health_percentage; /**< Cell health percentage (0-100) */
 } bess_cell_data_t;
 
 /**
  * @brief Battery module data structure
  */
 typedef struct {
     uint8_t id;                                    /**< Module ID */
     float voltage;                                 /**< Module voltage in V */
     float current;                                 /**< Module current in A */
     float temperature[4];                          /**< Module temperature sensors in °C */
     float state_of_charge;                         /**< Module state of charge (0-100) */
     float state_of_health;                         /**< Module state of health (0-100) */
     bess_cell_data_t cells[BESS_CELLS_PER_MODULE]; /**< Cell data */
     uint32_t total_energy_charged;                 /**< Total energy charged in Wh */
     uint32_t total_energy_discharged;              /**< Total energy discharged in Wh */
     uint16_t cycle_count;                          /**< Module cycle count */
     bool online;                                   /**< Flag indicating if module is online */
     bess_error_code_t error_code;                  /**< Module error code */
 } bess_module_data_t;
 
 /**
  * @brief Power conversion data structure
  */
 typedef struct {
     float input_voltage;       /**< Input voltage in V */
     float output_voltage;      /**< Output voltage in V */
     float input_current;       /**< Input current in A */
     float output_current;      /**< Output current in A */
     float input_power;         /**< Input power in W */
     float output_power;        /**< Output power in W */
     float efficiency;          /**< Converter efficiency (0-1) */
     float temperature;         /**< Converter temperature in °C */
     bool online;               /**< Flag indicating if converter is online */
     bess_error_code_t error_code; /**< Converter error code */
 } bess_power_conversion_data_t;
 
 /**
  * @brief Grid connection data structure
  */
 typedef struct {
     float voltage;             /**< Grid voltage in V */
     float frequency;           /**< Grid frequency in Hz */
     float current;             /**< Grid current in A */
     float power_factor;        /**< Grid power factor */
     float active_power;        /**< Active power in W */
     float reactive_power;      /**< Reactive power in VAR */
     bool connected;            /**< Flag indicating if grid is connected */
     bess_error_code_t error_code; /**< Grid error code */
 } bess_grid_data_t;
 
 /**
  * @brief System status data structure
  */
 typedef struct {
     bess_operation_mode_t mode;                /**< System operation mode */
     bess_system_flags_t flags;                 /**< System state flags */
     bess_error_code_t error_code;              /**< System error code */
     float total_voltage;                       /**< Total system voltage in V */
     float total_current;                       /**< Total system current in A */
     float total_power;                         /**< Total system power in W */
     float state_of_charge;                     /**< System state of charge (0-100) */
     float state_of_health;                     /**< System state of health (0-100) */
     float energy_available;                    /**< Available energy in kWh */
     uint32_t total_energy_charged;             /**< Total energy charged in kWh */
     uint32_t total_energy_discharged;          /**< Total energy discharged in kWh */
     uint32_t total_runtime_hours;              /**< Total runtime in hours */
     float ambient_temperature;                 /**< Ambient temperature in °C */
     float highest_cell_temperature;            /**< Highest cell temperature in °C */
     float lowest_cell_temperature;             /**< Lowest cell temperature in °C */
     float highest_cell_voltage;                /**< Highest cell voltage in V */
     float lowest_cell_voltage;                 /**< Lowest cell voltage in V */
     uint8_t active_module_count;               /**< Number of active modules */
     time_t last_maintenance_time;              /**< Timestamp of last maintenance */
     time_t last_error_time;                    /**< Timestamp of last error */
     uint32_t uptime_seconds;                   /**< System uptime in seconds */
 } bess_system_status_t;
 
 /**
  * @brief Log entry structure
  */
 typedef struct {
     time_t timestamp;          /**< Timestamp of log entry */
     bess_log_level_t level;    /**< Log severity level */
     char module[16];           /**< Module name */
     char message[256];         /**< Log message */
     bess_error_code_t error_code; /**< Associated error code if applicable */
 } bess_log_entry_t;
 
 /**
  * @brief Command codes for system control
  */
 typedef enum {
     BESS_CMD_NOP = 0,              /**< No operation */
     BESS_CMD_START,                /**< Start system */
     BESS_CMD_STOP,                 /**< Stop system */
     BESS_CMD_EMERGENCY_STOP,       /**< Emergency stop */
     BESS_CMD_RESET,                /**< Reset system */
     BESS_CMD_ENTER_MAINTENANCE,    /**< Enter maintenance mode */
     BESS_CMD_EXIT_MAINTENANCE,     /**< Exit maintenance mode */
     BESS_CMD_START_CHARGING,       /**< Start charging */
     BESS_CMD_START_DISCHARGING,    /**< Start discharging */
     BESS_CMD_BALANCE_CELLS,        /**< Start cell balancing */
     BESS_CMD_UPDATE_FIRMWARE,      /**< Update firmware */
     BESS_CMD_CALIBRATE_SENSORS,    /**< Calibrate sensors */
     BESS_CMD_CLEAR_ERROR_LOG,      /**< Clear error log */
     BESS_CMD_GET_DIAGNOSTICS,      /**< Get diagnostics information */
     BESS_CMD_TEST_EMERGENCY_STOP,  /**< Test emergency stop circuit */
     BESS_CMD_SET_TIME,             /**< Set system time */
     BESS_CMD_CHANGE_PASSWORD,      /**< Change system password */
     BESS_CMD_FACTORY_RESET         /**< Factory reset */
 } bess_command_code_t;
 
 /**
  * @brief Command structure
  */
 typedef struct {
     bess_command_code_t code;      /**< Command code */
     uint32_t param1;               /**< Command parameter 1 */
     uint32_t param2;               /**< Command parameter 2 */
     char string_param[64];         /**< String parameter if needed */
     uint32_t auth_token;           /**< Authentication token */
 } bess_command_t;
 
 /**
  * @brief Command response structure
  */
 typedef struct {
     bess_command_code_t code;      /**< Command code */
     bool success;                  /**< Success flag */
     bess_error_code_t error_code;  /**< Error code if command failed */
     char message[128];             /**< Response message */
     uint32_t result_data;          /**< Result data if applicable */
 } bess_command_response_t;
 
 /**
  * @brief Event callback function type
  */
 typedef void (*bess_event_callback_t)(void *event_data, void *user_data);
 
 /**
  * @brief Event types enumeration
  */
 typedef enum {
     BESS_EVENT_ERROR,              /**< Error event */
     BESS_EVENT_WARNING,            /**< Warning event */
     BESS_EVENT_STATE_CHANGE,       /**< State change event */
     BESS_EVENT_COMMAND_RECEIVED,   /**< Command received event */
     BESS_EVENT_MODULE_STATUS,      /**< Module status change event */
     BESS_EVENT_GRID_STATUS,        /**< Grid status change event */
     BESS_EVENT_SOC_THRESHOLD,      /**< SoC threshold reached event */
     BESS_EVENT_TEMPERATURE_THRESHOLD, /**< Temperature threshold reached event */
     BESS_EVENT_MAINTENANCE_REQUIRED, /**< Maintenance required event */
     BESS_EVENT_FIRMWARE_UPDATE     /**< Firmware update event */
 } bess_event_type_t;
 
 /**
  * @brief Modbus register map entry
  */
 typedef struct {
     uint16_t address;              /**< Register address */
     uint16_t count;                /**< Number of registers */
     const char *description;       /**< Register description */
     bool read_only;                /**< Read-only flag */
 } bess_modbus_register_map_entry_t;
 
 /**
  * @brief CANBus message ID enumeration
  */
 typedef enum {
     BESS_CAN_ID_HEARTBEAT = 0x100,        /**< Heartbeat message */
     BESS_CAN_ID_MODULE_STATUS = 0x200,    /**< Module status message */
     BESS_CAN_ID_CELL_VOLTAGES = 0x300,    /**< Cell voltages message */
     BESS_CAN_ID_CELL_TEMPS = 0x400,       /**< Cell temperatures message */
     BESS_CAN_ID_SYSTEM_STATUS = 0x500,    /**< System status message */
     BESS_CAN_ID_COMMAND = 0x600,          /**< Command message */
     BESS_CAN_ID_RESPONSE = 0x700          /**< Response message */
 } bess_can_message_id_t;
 
 /**
  * @brief Configuration storage structure
  */
 typedef struct {
     uint32_t crc32;                        /**< CRC32 checksum */
     char system_id[32];                    /**< System ID */
     char wifi_ssid[32];                    /**< WiFi SSID */
     char wifi_password[64];                /**< WiFi password */
     char aws_endpoint[128];                /**< AWS IoT endpoint */
     char aws_region[32];                   /**< AWS region */
     char cloudwatch_log_group[64];         /**< CloudWatch log group */
     char admin_password[64];               /**< Admin password hash */
     uint8_t modbus_slave_id;               /**< Modbus slave ID */
     uint16_t modbus_tcp_port;              /**< Modbus TCP port */
     uint32_t modbus_baud_rate;             /**< Modbus UART baud rate */
     uint32_t can_bitrate;                  /**< CAN bus bitrate */
     float soc_min_threshold;               /**< Minimum SoC threshold */
     float soc_max_threshold;               /**< Maximum SoC threshold */
     float cell_balancing_threshold;        /**< Cell balancing voltage threshold */
     uint32_t logging_interval_ms;          /**< Logging interval in ms */
     bool log_to_console;                   /**< Log to console flag */
     bool log_to_sd_card;                   /**< Log to SD card flag */
     bool log_to_aws;                       /**< Log to AWS flag */
     uint8_t log_level;                     /**< Log level */
     time_t last_maintenance_date;          /**< Last maintenance date */
     uint32_t maintenance_interval_days;    /**< Maintenance interval in days */
     bool auto_firmware_update;             /**< Auto firmware update flag */
     char ota_server_url[128];              /**< OTA server URL */
     uint8_t active_charging_profile;       /**< Active charging profile */
     uint8_t active_discharging_profile;    /**< Active discharging profile */
 } bess_config_storage_t;
 
 /**
  * @brief Charging/discharging profile
  */
 typedef struct {
     uint8_t id;                            /**< Profile ID */
     char name[32];                         /**< Profile name */
     float max_current;                     /**< Maximum current */
     float max_power;                       /**< Maximum power */
     float target_soc;                      /**< Target SoC */
     bool active_hours[24];                 /**< Active hours (one per hour) */
     bool active_days[7];                   /**< Active days (one per day of week) */
     bool enabled;                          /**< Profile enabled flag */
 } bess_power_profile_t;
 
 /**
  * @brief System metrics structure
  */
 typedef struct {
     float energy_efficiency;               /**< System energy efficiency (0-1) */
     float average_power;                   /**< Average power in W */
     float peak_power;                      /**< Peak power in W */
     uint32_t charge_discharge_cycles;      /**< Number of charge/discharge cycles */
     float total_energy_throughput;         /**< Total energy throughput in kWh */
     float average_daily_consumption;       /**< Average daily consumption in kWh */
     float average_daily_generation;        /**< Average daily generation in kWh */
     float carbon_saved;                    /**< Carbon saved in kg CO2 */
     float economic_savings;                /**< Economic savings in currency units */
     uint32_t uptime_percentage;            /**< System uptime percentage */
     uint32_t fault_count;                  /**< Number of faults occurred */
     time_t commission_date;                /**< System commission date */
 } bess_system_metrics_t;
 
 /**
  * @brief Task statistics structure
  */
 typedef struct {
     uint32_t execution_count;              /**< Number of times task executed */
     uint32_t execution_time_us_min;        /**< Minimum execution time in microseconds */
     uint32_t execution_time_us_max;        /**< Maximum execution time in microseconds */
     uint32_t execution_time_us_avg;        /**< Average execution time in microseconds */
     uint32_t stack_watermark;              /**< Stack watermark in bytes */
     uint32_t last_wake_time;               /**< Last wake time */
 } bess_task_stats_t;
 
 /**
  * @brief Network statistics structure
  */
 typedef struct {
     uint32_t bytes_sent;                   /**< Number of bytes sent */
     uint32_t bytes_received;               /**< Number of bytes received */
     uint32_t packets_sent;                 /**< Number of packets sent */
     uint32_t packets_received;             /**< Number of packets received */
     uint32_t connection_errors;            /**< Number of connection errors */
     uint32_t transmission_errors;          /**< Number of transmission errors */
     uint32_t successful_connections;       /**< Number of successful connections */
     uint32_t disconnections;               /**< Number of disconnections */
     bool connected;                        /**< Current connection status */
     int8_t signal_strength;                /**< Signal strength in dBm (WiFi) */
     uint32_t last_connected_time;          /**< Last time connected */
     uint32_t uptime_seconds;               /**< Connection uptime in seconds */
 } bess_network_stats_t;
 
 /**
  * @brief Memory usage statistics
  */
 typedef struct {
     uint32_t free_heap;                    /**< Free heap memory in bytes */
     uint32_t minimum_free_heap;            /**< Minimum free heap ever in bytes */
     uint32_t total_heap;                   /**< Total heap size in bytes */
     uint32_t free_dma;                     /**< Free DMA memory in bytes */
     uint32_t total_dma;                    /**< Total DMA memory in bytes */
     uint32_t free_spiram;                  /**< Free SPI RAM in bytes */
     uint32_t total_spiram;                 /**< Total SPI RAM in bytes */
 } bess_memory_stats_t;
 
 #endif /* BESS_TYPES_H */