/**
 * @file emergency_handler.h
 * @brief Emergency handling subsystem for BESS Safety Systems
 *
 * This component provides emergency detection, response, and recovery functionality
 * for the 100KW/200KWH Battery Energy Storage System (BESS) using 48V, 16KWH LFP modules.
 * It integrates with the Battery Management System (BMS) to provide comprehensive
 * protection against electrical, thermal, and operational hazards.
 *
 * @copyright Copyright (c) 2025
 */

 #ifndef EMERGENCY_HANDLER_H
 #define EMERGENCY_HANDLER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/event_groups.h"
 #include "freertos/semphr.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Emergency event types that can be detected by the system
  */
 typedef enum {
     EMERGENCY_NONE = 0,                  /*!< No emergency */
     EMERGENCY_OVER_VOLTAGE = (1 << 0),   /*!< System or module over-voltage */
     EMERGENCY_UNDER_VOLTAGE = (1 << 1),  /*!< System or module under-voltage */
     EMERGENCY_OVER_CURRENT = (1 << 2),   /*!< Excessive current detected */
     EMERGENCY_SHORT_CIRCUIT = (1 << 3),  /*!< Short circuit detected */
     EMERGENCY_OVER_TEMP = (1 << 4),      /*!< Over-temperature condition */
     EMERGENCY_THERMAL_RUNAWAY = (1 << 5),/*!< Thermal runaway detected */
     EMERGENCY_CELL_IMBALANCE = (1 << 6), /*!< Dangerous cell imbalance */
     EMERGENCY_ISOLATION_FAULT = (1 << 7),/*!< Isolation fault detected */
     EMERGENCY_COMMUNICATION = (1 << 8),  /*!< Critical communication failure */
     EMERGENCY_BMS_FAILURE = (1 << 9),    /*!< BMS internal failure */
     EMERGENCY_GRID_FAULT = (1 << 10),    /*!< External grid fault */
     EMERGENCY_MANUAL_TRIGGER = (1 << 11),/*!< Manually triggered emergency */
     EMERGENCY_WATCHDOG = (1 << 12),      /*!< Watchdog timeout */
     EMERGENCY_UNKNOWN = (1 << 15)        /*!< Unknown emergency condition */
 } emergency_type_t;
 
 /**
  * @brief Emergency severity levels
  */
 typedef enum {
     SEVERITY_INFO = 0,      /*!< Informational only, no action required */
     SEVERITY_LOW = 1,       /*!< Low severity, minor action required */
     SEVERITY_MEDIUM = 2,    /*!< Medium severity, significant action required */
     SEVERITY_HIGH = 3,      /*!< High severity, immediate action required */
     SEVERITY_CRITICAL = 4   /*!< Critical severity, system shutdown required */
 } emergency_severity_t;
 
 /**
  * @brief Emergency response actions
  */
 typedef enum {
     RESPONSE_NONE = 0,              /*!< No action (for logging/monitoring only) */
     RESPONSE_LOG_ONLY = (1 << 0),   /*!< Log the event only */
     RESPONSE_ALERT = (1 << 1),      /*!< Generate alert/notification */
     RESPONSE_LIMIT_POWER = (1 << 2),/*!< Reduce power draw/generation */
     RESPONSE_STOP_CHARGE = (1 << 3),/*!< Stop charging operation */
     RESPONSE_STOP_DISCHARGE = (1 << 4), /*!< Stop discharging operation */
     RESPONSE_DISCONNECT = (1 << 5), /*!< Disconnect from grid/load */
     RESPONSE_COOL_SYSTEM = (1 << 6),/*!< Activate cooling at maximum */
     RESPONSE_SHUTDOWN = (1 << 7),   /*!< Complete system shutdown */
     RESPONSE_CONTACTOR_OPEN = (1 << 8), /*!< Open main contactors */
     RESPONSE_ALL = 0xFFFF           /*!< Apply all possible responses */
 } emergency_response_t;
 
 /**
  * @brief System state during/after emergency
  */
 typedef enum {
     EMERGENCY_STATE_NORMAL = 0,     /*!< Normal operation, no emergency */
     EMERGENCY_STATE_ACTIVE = 1,     /*!< Emergency condition is active */
     EMERGENCY_STATE_RECOVERING = 2, /*!< Recovering from emergency */
     EMERGENCY_STATE_LOCKOUT = 3     /*!< Locked out, manual reset required */
 } emergency_state_t;
 
 /**
  * @brief Detailed emergency event information
  */
 typedef struct {
     emergency_type_t type;          /*!< Type of emergency */
     emergency_severity_t severity;  /*!< Severity level */
     uint8_t module_id;              /*!< Module ID (0xFF if system-wide) */
     float measured_value;           /*!< Measured value that triggered (if applicable) */
     float threshold_value;          /*!< Threshold that was exceeded (if applicable) */
     uint32_t timestamp;             /*!< Timestamp when emergency occurred */
     char description[64];           /*!< Human-readable description */
 } emergency_event_t;
 
 /**
  * @brief Emergency handler configuration
  */
 typedef struct {
     uint32_t watchdog_timeout_ms;       /*!< Watchdog timeout in milliseconds */
     bool auto_recover_enabled;          /*!< Enable automatic recovery attempts */
     uint32_t recovery_cooldown_ms;      /*!< Cooldown period before recovery attempt */
     uint8_t max_recovery_attempts;      /*!< Maximum number of recovery attempts */
     emergency_response_t default_resp;  /*!< Default response for unspecified emergencies */
     uint32_t task_stack_size;           /*!< Stack size for emergency handler task */
     uint8_t task_priority;              /*!< Priority for emergency handler task */
     bool log_to_console;                /*!< Enable console logging */
     bool log_to_sd;                     /*!< Enable SD card logging */
     bool log_to_cloud;                  /*!< Enable AWS CloudWatch logging */
 } emergency_handler_config_t;
 
 /**
  * @brief Callback function prototype for emergency notifications
  *
  * @param event Emergency event information
  * @param user_data User data pointer passed during registration
  */
 typedef void (*emergency_callback_t)(emergency_event_t event, void *user_data);
 
 /**
  * @brief Initialize the emergency handler with the specified configuration
  *
  * @param config Pointer to configuration structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_init(const emergency_handler_config_t *config);
 
 /**
  * @brief Start the emergency handler task
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_start(void);
 
 /**
  * @brief Register a callback function for emergency notifications
  *
  * @param callback Function pointer to be called on emergency events
  * @param user_data User data pointer to pass to callback
  * @param mask Bitmask of emergency types to receive (or 0 for all)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_register_callback(emergency_callback_t callback, 
                                              void *user_data,
                                              uint32_t mask);
 
 /**
  * @brief Unregister a previously registered callback function
  *
  * @param callback The callback function to unregister
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_unregister_callback(emergency_callback_t callback);
 
 /**
  * @brief Trigger an emergency response
  *
  * @param type Type of emergency condition
  * @param severity Severity level of the emergency
  * @param module_id Module ID (0xFF for system-wide)
  * @param measured_value Measured value that triggered emergency (optional)
  * @param threshold_value Threshold that was exceeded (optional)
  * @param description Description of the emergency (optional)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_trigger(emergency_type_t type,
                                    emergency_severity_t severity,
                                    uint8_t module_id,
                                    float measured_value,
                                    float threshold_value,
                                    const char *description);
 
 /**
  * @brief Manually trigger an emergency response
  *
  * @param description Description of why the emergency was manually triggered
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_manual_trigger(const char *description);
 
 /**
  * @brief Check if any emergency is currently active
  *
  * @param active Pointer to store the result (true if emergency active)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_is_active(bool *active);
 
 /**
  * @brief Get the current emergency state
  *
  * @param state Pointer to store the current state
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_get_state(emergency_state_t *state);
 
 /**
  * @brief Get details of the most recent emergency event
  *
  * @param event Pointer to store event information
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_get_last_event(emergency_event_t *event);
 
 /**
  * @brief Attempt to clear an active emergency and restore normal operation
  * 
  * @param type The specific emergency type to clear (or EMERGENCY_NONE for all)
  * @param force Force clearing even if conditions still present (use with caution)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_clear(emergency_type_t type, bool force);
 
 /**
  * @brief Set the response action for a specific emergency type
  *
  * @param type Emergency type to configure
  * @param response Response action(s) to take
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_set_response(emergency_type_t type, 
                                         emergency_response_t response);
 
 /**
  * @brief Reset the emergency handler to default state
  * 
  * @param hard_reset If true, clear all history and settings
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_reset(bool hard_reset);
 
 /**
  * @brief Feed the emergency watchdog timer
  * 
  * Must be called periodically to prevent watchdog emergency
  * 
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_feed_watchdog(void);
 
 /**
  * @brief Configure logging destinations
  *
  * @param console Enable/disable console logging
  * @param sd_card Enable/disable SD card logging
  * @param cloud Enable/disable AWS CloudWatch logging
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_configure_logging(bool console, bool sd_card, bool cloud);
 
 /**
  * @brief Get the number of emergency events that have occurred
  *
  * @param count Pointer to store the event count
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_get_event_count(uint32_t *count);
 
 /**
  * @brief Run a comprehensive self-test of the emergency systems
  *
  * Tests detection mechanisms, response systems, and recovery procedures
  *
  * @param test_level Level of testing (0=basic, 1=standard, 2=comprehensive)
  * @param results Pointer to store test results (optional)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_self_test(uint8_t test_level, void *results);
 
 /**
  * @brief Get string representation of emergency type
  *
  * @param type The emergency type
  * @return Constant string representing the emergency type
  */
 const char* emergency_handler_type_to_string(emergency_type_t type);
 
 /**
  * @brief Get string representation of emergency severity
  *
  * @param severity The emergency severity
  * @return Constant string representing the severity
  */
 const char* emergency_handler_severity_to_string(emergency_severity_t severity);
 
 /**
  * @brief Integration hook for Battery Manager module
  *
  * @param event_type Battery manager event type
  * @param module_id Module ID
  * @param data Event-specific data
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_process_battery_event(uint32_t event_type, 
                                                  uint8_t module_id, 
                                                  void *data);
 
 /**
  * @brief Integration hook for Thermal Monitor module
  *
  * @param zone_id Thermal zone ID
  * @param temperature Current temperature
  * @param threshold Threshold temperature
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_process_thermal_event(uint8_t zone_id,
                                                  float temperature,
                                                  float threshold);
 
 /**
  * @brief Integration hook for network/communications events
  *
  * @param comms_type Communication channel type (Modbus, CANBus, etc.)
  * @param device_id Device identifier
  * @param error_code Error code
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t emergency_handler_process_comms_event(uint8_t comms_type,
                                                uint16_t device_id,
                                                int32_t error_code);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* EMERGENCY_HANDLER_H */