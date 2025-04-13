/**
 * @file safety_manager.h
 * @brief Safety Management System for 100KW/200KWH BESS with LFP Battery Modules
 * 
 * This module provides comprehensive safety management for the Battery Energy Storage System,
 * integrating with Battery Manager, Cell Balancer, SoC Calculator, and Thermal Monitor components
 * to ensure safe operation under all conditions. It implements multiple layers of protection,
 * fault detection, emergency response, and system-wide safety coordination.
 * 
 * Features:
 * - Multi-level safety protection against voltage, current, and temperature hazards
 * - Integration with all BMS subsystems for coordinated safety responses
 * - Modbus and CANBus communication for safety status reporting
 * - Comprehensive logging to console, SD Card, and AWS Cloudwatch
 * - FreeRTOS task management for real-time safety monitoring
 * 
 * @note The safety manager is designed for a BESS with 100KW power and 200KWH capacity,
 *       using LFP battery modules (48V, 16KWH) and ESP32-P4 MCU running FreeRTOS.
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef SAFETY_MANAGER_H
 #define SAFETY_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Safety system status codes
  */
 typedef enum {
     SAFETY_STATUS_OK = 0,           /**< All systems operating normally */
     SAFETY_STATUS_WARNING,          /**< Warning conditions detected */
     SAFETY_STATUS_ALARM,            /**< Alarm conditions requiring attention */
     SAFETY_STATUS_CRITICAL,         /**< Critical conditions requiring immediate action */
     SAFETY_STATUS_EMERGENCY,        /**< Emergency conditions requiring shutdown */
     SAFETY_STATUS_FAULT,            /**< System fault detected */
     SAFETY_STATUS_MAINTENANCE,      /**< System in maintenance mode */
     SAFETY_STATUS_UNKNOWN           /**< System status unknown or initializing */
 } safety_status_t;
 
 /**
  * @brief Safety event types
  */
 typedef enum {
     SAFETY_EVENT_VOLTAGE_HIGH,          /**< Module voltage exceeds high threshold */
     SAFETY_EVENT_VOLTAGE_LOW,           /**< Module voltage below low threshold */
     SAFETY_EVENT_CELL_VOLTAGE_HIGH,     /**< Cell voltage exceeds high threshold */
     SAFETY_EVENT_CELL_VOLTAGE_LOW,      /**< Cell voltage below low threshold */
     SAFETY_EVENT_CURRENT_HIGH,          /**< Current exceeds high threshold */
     SAFETY_EVENT_CURRENT_REVERSE,       /**< Reverse current detected */
     SAFETY_EVENT_TEMPERATURE_HIGH,      /**< Temperature exceeds high threshold */
     SAFETY_EVENT_TEMPERATURE_LOW,       /**< Temperature below low threshold */
     SAFETY_EVENT_THERMAL_RUNAWAY,       /**< Thermal runaway condition detected */
     SAFETY_EVENT_SOC_HIGH,              /**< State of Charge exceeds high threshold */
     SAFETY_EVENT_SOC_LOW,               /**< State of Charge below low threshold */
     SAFETY_EVENT_ISOLATION_FAULT,       /**< Isolation fault detected */
     SAFETY_EVENT_COMMUNICATION_ERROR,   /**< Communication error detected */
     SAFETY_EVENT_CONTACTOR_FAULT,       /**< Contactor fault detected */
     SAFETY_EVENT_BALANCING_FAULT,       /**< Cell balancing fault detected */
     SAFETY_EVENT_WATCHDOG_TIMEOUT,      /**< Watchdog timeout detected */
     SAFETY_EVENT_SENSOR_FAULT,          /**< Sensor fault detected */
     SAFETY_EVENT_COOLING_FAULT,         /**< Cooling system fault detected */
     SAFETY_EVENT_SYSTEM_RESET,          /**< System reset detected */
     SAFETY_EVENT_CONFIG_ERROR,          /**< Configuration error detected */
     SAFETY_EVENT_EXTERNAL_COMMAND,      /**< External command received */
     SAFETY_EVENT_COUNT                  /**< Total count of event types */
 } safety_event_t;
 
 /**
  * @brief Safety action types
  */
 typedef enum {
     SAFETY_ACTION_NONE = 0,             /**< No action required */
     SAFETY_ACTION_LOG_ONLY,             /**< Log the event only */
     SAFETY_ACTION_ALERT_OPERATOR,       /**< Alert the operator */
     SAFETY_ACTION_LIMIT_CURRENT,        /**< Limit system current */
     SAFETY_ACTION_INCREASE_COOLING,     /**< Increase cooling system power */
     SAFETY_ACTION_PAUSE_CHARGING,       /**< Pause charging operations */
     SAFETY_ACTION_PAUSE_DISCHARGING,    /**< Pause discharging operations */
     SAFETY_ACTION_PAUSE_BALANCING,      /**< Pause cell balancing */
     SAFETY_ACTION_OPEN_CONTACTORS,      /**< Open system contactors */
     SAFETY_ACTION_EMERGENCY_SHUTDOWN,   /**< Perform emergency system shutdown */
     SAFETY_ACTION_REQUIRE_RESET,        /**< Require system reset to continue */
     SAFETY_ACTION_COUNT                 /**< Total count of action types */
 } safety_action_t;
 
 /**
  * @brief Log destination options for safety events
  */
 typedef enum {
     SAFETY_LOG_CONSOLE = BIT0,          /**< Log to console output */
     SAFETY_LOG_SD_CARD = BIT1,          /**< Log to SD card */
     SAFETY_LOG_CLOUDWATCH = BIT2,       /**< Log to AWS Cloudwatch */
     SAFETY_LOG_ALL = (SAFETY_LOG_CONSOLE | SAFETY_LOG_SD_CARD | SAFETY_LOG_CLOUDWATCH)
 } safety_log_destination_t;
 
 /**
  * @brief System fault data structure
  */
 typedef struct {
     safety_event_t event;               /**< Type of event that caused the fault */
     uint8_t module_id;                  /**< Module ID where fault occurred (0xFF for system-wide) */
     uint32_t timestamp;                 /**< Timestamp when fault was detected */
     float measured_value;               /**< Measured value that triggered the fault */
     float threshold_value;              /**< Threshold value that was exceeded */
     uint32_t duration_ms;               /**< Duration of fault condition in milliseconds */
     safety_action_t action_taken;       /**< Action taken in response to fault */
     bool requires_reset;                /**< Whether fault requires manual reset */
     bool is_active;                     /**< Whether fault is currently active */
 } safety_fault_t;
 
 /**
  * @brief Safety system configuration structure
  */
 typedef struct {
     /* System Configuration */
     uint8_t number_of_modules;          /**< Number of battery modules in the system */
     uint8_t cells_per_module;           /**< Number of cells per battery module */
     
     /* Voltage Limits */
     float module_voltage_high;          /**< High voltage threshold for modules (V) */
     float module_voltage_low;           /**< Low voltage threshold for modules (V) */
     float cell_voltage_high;            /**< High voltage threshold for cells (V) */
     float cell_voltage_low;             /**< Low voltage threshold for cells (V) */
     
     /* Current Limits */
     float current_high_charge;          /**< High current threshold for charging (A) */
     float current_high_discharge;       /**< High current threshold for discharging (A) */
     
     /* Temperature Limits */
     float temperature_high;             /**< High temperature threshold (°C) */
     float temperature_critical;         /**< Critical temperature threshold (°C) */
     float temperature_low;              /**< Low temperature threshold (°C) */
     float temp_rate_runaway;            /**< Temperature rate of change indicating runaway (°C/s) */
     
     /* State of Charge Limits */
     float soc_high;                     /**< High State of Charge threshold (%) */
     float soc_low;                      /**< Low State of Charge threshold (%) */
     
     /* Timing Parameters */
     uint32_t monitoring_interval_ms;    /**< Safety monitoring task interval (ms) */
     uint32_t contactor_timeout_ms;      /**< Contactor operation timeout (ms) */
     uint32_t fault_report_interval_ms;  /**< Fault reporting interval to remote systems (ms) */
     
     /* Response Delays */
     uint32_t voltage_fault_delay_ms;    /**< Delay before acting on voltage faults (ms) */
     uint32_t current_fault_delay_ms;    /**< Delay before acting on current faults (ms) */
     uint32_t temp_fault_delay_ms;       /**< Delay before acting on temperature faults (ms) */
     
     /* Logging Configuration */
     uint32_t log_destination;           /**< Bit mask of log destinations (console, SD, cloud) */
     uint8_t log_level;                  /**< Logging level (0-5, with 5 being most verbose) */
     
     /* Communication Configuration */
     bool enable_modbus_reporting;       /**< Enable fault reporting via Modbus */
     bool enable_canbus_reporting;       /**< Enable fault reporting via CANBus */
     
     /* Fault Management */
     uint8_t max_fault_count;            /**< Maximum number of faults to track */
     bool auto_reset_warnings;           /**< Automatically reset warning-level faults */
     uint32_t auto_reset_timeout_ms;     /**< Timeout for automatic fault reset (ms) */
 } safety_config_t;
 
 /**
  * @brief Event callback function type
  * 
  * @param event The safety event that triggered the callback
  * @param module_id Module ID where the event occurred (0xFF for system-wide)
  * @param measured_value Measured value that triggered the event
  * @param threshold_value Threshold value that was exceeded
  * @param user_data User data pointer provided during callback registration
  */
 typedef void (*safety_event_callback_t)(safety_event_t event, 
                                        uint8_t module_id, 
                                        float measured_value, 
                                        float threshold_value, 
                                        void *user_data);
 
 /**
  * @brief Initialize the safety management system
  * 
  * @param config Pointer to safety system configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_init(const safety_config_t *config);
 
 /**
  * @brief Start the safety management system monitoring tasks
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_start(void);
 
 /**
  * @brief Stop the safety management system monitoring tasks
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_stop(void);
 
 /**
  * @brief Update voltage data for a specific module
  * 
  * @param module_id Module identifier (0 to number_of_modules-1)
  * @param module_voltage Total module voltage
  * @param cell_voltages Array of individual cell voltages
  * @param cell_count Number of cells in the module
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_update_voltage_data(uint8_t module_id, 
                                            float module_voltage,
                                            const float *cell_voltages,
                                            uint8_t cell_count);
 
 /**
  * @brief Update current data for a specific module
  * 
  * @param module_id Module identifier (0 to number_of_modules-1)
  * @param current Current measurement (positive for discharge, negative for charge)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_update_current_data(uint8_t module_id, float current);
 
 /**
  * @brief Update temperature data for a specific module
  * 
  * @param module_id Module identifier (0 to number_of_modules-1)
  * @param temperatures Array of temperature readings
  * @param sensor_count Number of temperature sensors
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_update_temperature_data(uint8_t module_id,
                                                const float *temperatures,
                                                uint8_t sensor_count);
 
 /**
  * @brief Update State of Charge data for a specific module
  * 
  * @param module_id Module identifier (0 to number_of_modules-1)
  * @param soc State of Charge percentage (0-100%)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_update_soc_data(uint8_t module_id, float soc);
 
 /**
  * @brief Register a callback function for safety events
  * 
  * @param event Event type to register for (use SAFETY_EVENT_COUNT for all events)
  * @param callback Callback function pointer
  * @param user_data User data pointer to pass to callback
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_register_callback(safety_event_t event,
                                          safety_event_callback_t callback,
                                          void *user_data);
 
 /**
  * @brief Unregister a callback function for safety events
  * 
  * @param event Event type to unregister
  * @param callback Callback function pointer to unregister
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_unregister_callback(safety_event_t event,
                                            safety_event_callback_t callback);
 
 /**
  * @brief Manual trigger of a safety event for testing or external conditions
  * 
  * @param event Event type to trigger
  * @param module_id Module ID to associate with event (0xFF for system-wide)
  * @param measured_value Measured value to report
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_trigger_event(safety_event_t event,
                                      uint8_t module_id,
                                      float measured_value);
 
 /**
  * @brief Get current system safety status
  * 
  * @param status Pointer to store safety status
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_get_status(safety_status_t *status);
 
 /**
  * @brief Get current fault list
  * 
  * @param faults Array to store fault information
  * @param max_faults Maximum number of faults to retrieve
  * @param fault_count Pointer to store actual number of faults retrieved
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_get_faults(safety_fault_t *faults,
                                   uint8_t max_faults,
                                   uint8_t *fault_count);
 
 /**
  * @brief Clear a specific fault
  * 
  * @param event Event type of fault to clear
  * @param module_id Module ID of fault to clear (0xFF for system-wide)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_clear_fault(safety_event_t event, uint8_t module_id);
 
 /**
  * @brief Clear all faults in the system
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_clear_all_faults(void);
 
 /**
  * @brief Perform safety system self-diagnostic check
  * 
  * @param passed Pointer to store diagnostic result (true = passed)
  * @param error_code Pointer to store error code (if failed)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_self_diagnostic(bool *passed, uint32_t *error_code);
 
 /**
  * @brief Set log destinations for safety events
  * 
  * @param destinations Bit mask of destinations (console, SD card, Cloudwatch)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_set_log_destination(uint32_t destinations);
 
 /**
  * @brief Configure automatic or manual contactor control
  * 
  * @param auto_control If true, safety system can automatically control contactors
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_configure_contactor_control(bool auto_control);
 
 /**
  * @brief Force emergency shutdown of the system
  * 
  * @param reason Text string describing reason for shutdown
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_emergency_shutdown(const char *reason);
 
 /**
  * @brief Dynamically update safety thresholds
  * 
  * @param event Event type threshold to update
  * @param new_threshold New threshold value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_update_threshold(safety_event_t event, float new_threshold);
 
 /**
  * @brief Check if safety system allows charging
  * 
  * @param allowed Pointer to store charging allowed status
  * @param limiting_event If not allowed, pointer to store limiting event type
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_is_charging_allowed(bool *allowed, safety_event_t *limiting_event);
 
 /**
  * @brief Check if safety system allows discharging
  * 
  * @param allowed Pointer to store discharging allowed status
  * @param limiting_event If not allowed, pointer to store limiting event type
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_is_discharging_allowed(bool *allowed, safety_event_t *limiting_event);
 
 /**
  * @brief Configure communication protocols for safety reporting
  * 
  * @param enable_modbus Enable Modbus reporting
  * @param enable_canbus Enable CANBus reporting
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_configure_communication(bool enable_modbus, bool enable_canbus);
 
 /**
  * @brief Check if system is in failsafe mode due to critical issues
  * 
  * @param in_failsafe Pointer to store failsafe status
  * @param critical_event If in failsafe, pointer to store critical event type
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t safety_manager_is_in_failsafe(bool *in_failsafe, safety_event_t *critical_event);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* SAFETY_MANAGER_H */