/**
 * @file fault_detector.h
 * @brief Fault detection system for BESS 100KW/200KWH energy storage system
 * @version 1.0
 * @date 2025-04-13
 * 
 * Provides comprehensive fault detection for the 100KW/200KWH BESS with LFP 
 * battery modules (48V, 16KWH). Detects, logs, and responds to various fault 
 * conditions to ensure safe system operation.
 */

 #ifndef FAULT_DETECTOR_H
 #define FAULT_DETECTOR_H
 
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
  * @brief Fault types that can be detected
  */
 typedef enum {
     FAULT_NONE                      = 0x00000000, /**< No fault detected */
     
     /* Voltage related faults */
     FAULT_CELL_UNDERVOLTAGE         = 0x00000001, /**< Cell voltage below minimum threshold */
     FAULT_CELL_OVERVOLTAGE          = 0x00000002, /**< Cell voltage above maximum threshold */
     FAULT_MODULE_UNDERVOLTAGE       = 0x00000004, /**< Module voltage below minimum threshold */
     FAULT_MODULE_OVERVOLTAGE        = 0x00000008, /**< Module voltage above maximum threshold */
     FAULT_SYSTEM_UNDERVOLTAGE       = 0x00000010, /**< System voltage below minimum threshold */
     FAULT_SYSTEM_OVERVOLTAGE        = 0x00000020, /**< System voltage above maximum threshold */
     FAULT_VOLTAGE_IMBALANCE         = 0x00000040, /**< Excessive voltage difference between cells */
     
     /* Current related faults */
     FAULT_OVERCURRENT_CHARGE        = 0x00000080, /**< Charge current exceeds maximum threshold */
     FAULT_OVERCURRENT_DISCHARGE     = 0x00000100, /**< Discharge current exceeds maximum threshold */
     FAULT_SHORTCIRCUIT              = 0x00000200, /**< Short circuit detected */
     FAULT_CURRENT_IMBALANCE         = 0x00000400, /**< Current imbalance between modules */
     
     /* Temperature related faults */
     FAULT_CELL_UNDERTEMP            = 0x00000800, /**< Cell temperature below minimum threshold */
     FAULT_CELL_OVERTEMP             = 0x00001000, /**< Cell temperature above maximum threshold */
     FAULT_MODULE_UNDERTEMP          = 0x00002000, /**< Module temperature below minimum threshold */
     FAULT_MODULE_OVERTEMP           = 0x00004000, /**< Module temperature above maximum threshold */
     FAULT_THERMAL_RUNAWAY           = 0x00008000, /**< Thermal runaway condition detected */
     FAULT_TEMP_SENSOR_FAILURE       = 0x00010000, /**< Temperature sensor malfunction */
     
     /* System level faults */
     FAULT_INSULATION_FAILURE        = 0x00020000, /**< Insulation resistance below threshold */
     FAULT_CONTACTOR_FAILURE         = 0x00040000, /**< Contactor failed to open/close */
     FAULT_PRECHARGE_FAILURE         = 0x00080000, /**< Precharge circuit malfunction */
     FAULT_BMS_COMMUNICATION         = 0x00100000, /**< Internal BMS communication error */
     FAULT_EXTERNAL_COMMUNICATION    = 0x00200000, /**< External communication error */
     FAULT_WATCHDOG_TIMEOUT          = 0x00400000, /**< Watchdog timer expired */
     FAULT_MEMORY_CORRUPTION         = 0x00800000, /**< Configuration memory corruption */
     
     /* Connectivity faults */
     FAULT_CAN_BUS_ERROR             = 0x01000000, /**< CAN bus error */
     FAULT_MODBUS_ERROR              = 0x02000000, /**< Modbus communication error */
     
     /* Sensor faults */
     FAULT_VOLTAGE_SENSOR            = 0x04000000, /**< Voltage sensor failure */
     FAULT_CURRENT_SENSOR            = 0x08000000, /**< Current sensor failure */
     
     /* Physical faults */
     FAULT_ENCLOSURE_BREACH          = 0x10000000, /**< Enclosure breach detected */
     FAULT_WATER_LEAK                = 0x20000000, /**< Water leak detected */
     FAULT_SMOKE_DETECTED            = 0x40000000, /**< Smoke detected */
     
     FAULT_SYSTEM_CRITICAL           = 0x80000000  /**< Critical system fault requiring immediate shutdown */
 } fault_type_t;
 
 /**
  * @brief Fault severity levels
  */
 typedef enum {
     FAULT_SEVERITY_INFO,        /**< Informational, no immediate action required */
     FAULT_SEVERITY_WARNING,     /**< Warning condition, action recommended */
     FAULT_SEVERITY_ERROR,       /**< Error condition, requires attention */
     FAULT_SEVERITY_CRITICAL,    /**< Critical condition, immediate action required */
     FAULT_SEVERITY_EMERGENCY    /**< Emergency condition, system shutdown needed */
 } fault_severity_t;
 
 /**
  * @brief Fault response actions
  */
 typedef enum {
     FAULT_ACTION_NONE,              /**< No action required */
     FAULT_ACTION_LOG_ONLY,          /**< Only log the fault, no system action */
     FAULT_ACTION_ALERT,             /**< Generate alert but continue operation */
     FAULT_ACTION_LIMIT_CHARGE,      /**< Limit charging capability */
     FAULT_ACTION_LIMIT_DISCHARGE,   /**< Limit discharging capability */
     FAULT_ACTION_PAUSE_OPERATION,   /**< Temporarily pause system operation */
     FAULT_ACTION_OPEN_CONTACTORS,   /**< Open contactors to isolate battery */
     FAULT_ACTION_SHUTDOWN           /**< Complete system shutdown */
 } fault_action_t;
 
 /**
  * @brief Fault event data structure
  */
 typedef struct {
     fault_type_t type;              /**< Type of the detected fault */
     fault_severity_t severity;      /**< Severity of the fault */
     fault_action_t action;          /**< Action taken in response */
     uint32_t timestamp;             /**< Timestamp when fault was detected (seconds since boot) */
     uint8_t module_id;              /**< Module ID or 0xFF for system-level faults */
     uint8_t cell_id;                /**< Cell ID or 0xFF if not applicable */
     float measured_value;           /**< Measured value that triggered the fault */
     float threshold_value;          /**< Threshold value that was exceeded */
     char description[64];           /**< Detailed fault description */
 } fault_event_t;
 
 /**
  * @brief Fault status data structure
  */
 typedef struct {
     uint32_t active_faults;         /**< Bitfield of currently active faults */
     uint32_t latched_faults;        /**< Bitfield of latched faults (remain set until cleared) */
     uint32_t fault_counts[32];      /**< Array of fault occurrence counts indexed by fault type */
     uint32_t last_fault_time;       /**< Timestamp of last fault occurrence */
     fault_type_t last_fault_type;   /**< Type of the last detected fault */
     uint32_t total_faults;          /**< Total number of faults detected since system start */
 } fault_status_t;
 
 /**
  * @brief Configuration structure for fault detector
  */
 typedef struct {
     bool auto_reset;                /**< Automatically reset non-critical faults when condition clears */
     uint32_t monitoring_interval;   /**< Fault detection monitoring interval in milliseconds */
     uint8_t consecutive_readings;   /**< Number of consecutive readings required to trigger a fault */
     uint32_t fault_persistence;     /**< Time in seconds that a fault persists after condition clears */
     uint32_t log_interval;          /**< Minimum interval between logging the same fault (seconds) */
     bool cloud_reporting;           /**< Enable reporting faults to AWS CloudWatch */
     bool enable_watchdog;           /**< Enable watchdog for fault detection task */
 } fault_detector_config_t;
 
 /**
  * @brief Fault event callback function type
  */
 typedef void (*fault_callback_t)(fault_event_t *event, void *user_data);
 
 /**
  * @brief Default configuration for fault detector
  */
 #define FAULT_DETECTOR_DEFAULT_CONFIG() { \
     .auto_reset = true, \
     .monitoring_interval = 100, \
     .consecutive_readings = 3, \
     .fault_persistence = 10, \
     .log_interval = 60, \
     .cloud_reporting = true, \
     .enable_watchdog = true \
 }
 
 /**
  * @brief Initialize the fault detector
  * 
  * @param config Pointer to configuration structure
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_init(const fault_detector_config_t *config);
 
 /**
  * @brief Start the fault detection task
  * 
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_start(void);
 
 /**
  * @brief Stop the fault detection task
  * 
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_stop(void);
 
 /**
  * @brief Reset (clear) all active and latched faults
  * 
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_reset_all(void);
 
 /**
  * @brief Reset (clear) a specific fault
  * 
  * @param fault_type The fault type to clear
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_reset_fault(fault_type_t fault_type);
 
 /**
  * @brief Manually report a fault
  * 
  * @param event Pointer to the fault event structure
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_report_fault(const fault_event_t *event);
 
 /**
  * @brief Set fault threshold values
  * 
  * @param fault_type The fault type to configure
  * @param threshold The threshold value that triggers the fault
  * @param hysteresis Hysteresis value for clearing the fault (must be less than threshold)
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_set_threshold(fault_type_t fault_type, float threshold, float hysteresis);
 
 /**
  * @brief Configure fault response action
  * 
  * @param fault_type The fault type to configure
  * @param severity The severity level of the fault
  * @param action The action to take when fault is detected
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_set_response(fault_type_t fault_type, fault_severity_t severity, fault_action_t action);
 
 /**
  * @brief Get current fault status information
  * 
  * @param status Pointer to fault_status_t structure to receive the status information
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_get_status(fault_status_t *status);
 
 /**
  * @brief Check if a specific fault is active
  * 
  * @param fault_type The fault type to check
  * @param active Pointer to bool to receive the active status
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_is_fault_active(fault_type_t fault_type, bool *active);
 
 /**
  * @brief Check if any fault of a specific severity level or higher is active
  * 
  * @param min_severity The minimum severity level to check for
  * @param active Pointer to bool to receive the active status
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_is_severity_active(fault_severity_t min_severity, bool *active);
 
 /**
  * @brief Register callback for fault events
  * 
  * @param callback The callback function to register
  * @param fault_mask Bitfield of fault types to receive callbacks for (0 for all)
  * @param user_data User data pointer passed to the callback
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_register_callback(fault_callback_t callback, uint32_t fault_mask, void *user_data);
 
 /**
  * @brief Unregister a previously registered callback
  * 
  * @param callback The callback function to unregister
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_unregister_callback(fault_callback_t callback);
 
 /**
  * @brief Get a text description for a fault type
  * 
  * @param fault_type The fault type to get description for
  * @return const char* Pointer to description string
  */
 const char* fault_detector_get_description(fault_type_t fault_type);
 
 /**
  * @brief Run self-test on the fault detection system
  * 
  * @return esp_err_t ESP_OK if self-test passed, or an error code
  */
 esp_err_t fault_detector_self_test(void);
 
 /**
  * @brief Configure fault thresholds from a configuration file
  * 
  * @param config_file Path to the configuration file
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_load_config(const char *config_file);
 
 /**
  * @brief Save current fault thresholds to a configuration file
  * 
  * @param config_file Path to the configuration file
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_save_config(const char *config_file);
 
 /**
  * @brief Enable or disable specific fault detection
  * 
  * @param fault_type The fault type to enable/disable
  * @param enable true to enable, false to disable
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t fault_detector_enable_fault(fault_type_t fault_type, bool enable);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* FAULT_DETECTOR_H */