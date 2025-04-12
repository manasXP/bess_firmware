/**
 * @file thermal_monitor.h
 * @brief Thermal monitoring system for BESS
 * 
 * Provides interfaces for monitoring and managing the thermal state of
 * battery modules, including temperature measurement, thermal runaway detection,
 * and cooling system control.
 */

 #ifndef THERMAL_MONITOR_H
 #define THERMAL_MONITOR_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 
 /**
  * @brief Thermal zone definitions
  */
 typedef enum {
     THERMAL_ZONE_NORMAL,             /**< Normal operating temperature */
     THERMAL_ZONE_ELEVATED,           /**< Elevated temperature, monitor closely */
     THERMAL_ZONE_WARNING,            /**< Warning temperature level */
     THERMAL_ZONE_CRITICAL,           /**< Critical temperature, immediate action required */
     THERMAL_ZONE_EMERGENCY           /**< Emergency temperature, shutdown required */
 } thermal_zone_t;
 
 /**
  * @brief Cooling method types
  */
 typedef enum {
     COOLING_METHOD_PASSIVE,          /**< Passive cooling (no active components) */
     COOLING_METHOD_FORCED_AIR,       /**< Forced air cooling (fans) */
     COOLING_METHOD_LIQUID,           /**< Liquid cooling system */
     COOLING_METHOD_HYBRID            /**< Hybrid cooling approach */
 } cooling_method_t;
 
 /**
  * @brief Thermal management status
  */
 typedef struct {
     float highest_temperature;       /**< Highest temperature in the system (°C) */
     float lowest_temperature;        /**< Lowest temperature in the system (°C) */
     float average_temperature;       /**< Average temperature across all sensors (°C) */
     float temperature_rate;          /**< Rate of temperature change (°C/min) */
     thermal_zone_t current_zone;     /**< Current thermal zone */
     uint8_t hottest_module;          /**< ID of the module with highest temperature */
     uint8_t coolest_module;          /**< ID of the module with lowest temperature */
     bool cooling_active;             /**< Cooling system is active */
     uint8_t cooling_power;           /**< Cooling power level (0-100%) */
     uint32_t time_in_current_zone;   /**< Time spent in current thermal zone (seconds) */
 } thermal_status_t;
 
 /**
  * @brief Initialize the thermal monitoring system
  * 
  * @param cooling_method Cooling method to use
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_init(cooling_method_t cooling_method);
 
 /**
  * @brief Start the thermal monitoring system
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_start(void);
 
 /**
  * @brief Update temperature data for a module
  * 
  * @param module_id ID of the module
  * @param temperatures Array of temperature sensor readings (°C)
  * @param sensor_count Number of temperature sensors
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_update_module(uint8_t module_id,
                                       const float *temperatures,
                                       uint8_t sensor_count);
 
 /**
  * @brief Get the current thermal status
  * 
  * @param[out] status Pointer to store the thermal status
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_status(thermal_status_t *status);
 
 /**
  * @brief Set temperature thresholds for different thermal zones
  * 
  * @param elevated_temp Threshold for elevated zone (°C)
  * @param warning_temp Threshold for warning zone (°C)
  * @param critical_temp Threshold for critical zone (°C)
  * @param emergency_temp Threshold for emergency zone (°C)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_set_thresholds(float elevated_temp,
                                        float warning_temp,
                                        float critical_temp,
                                        float emergency_temp);
 
 /**
  * @brief Get current temperature thresholds
  * 
  * @param[out] elevated_temp Pointer to store elevated threshold
  * @param[out] warning_temp Pointer to store warning threshold
  * @param[out] critical_temp Pointer to store critical threshold
  * @param[out] emergency_temp Pointer to store emergency threshold
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_thresholds(float *elevated_temp,
                                        float *warning_temp,
                                        float *critical_temp,
                                        float *emergency_temp);
 
 /**
  * @brief Manually activate the cooling system
  * 
  * @param power_level Cooling power level (0-100%)
  * @param duration_seconds Duration to activate cooling (0 for indefinite)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_activate_cooling(uint8_t power_level, uint32_t duration_seconds);
 
 /**
  * @brief Deactivate the cooling system
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_deactivate_cooling(void);
 
 /**
  * @brief Check for thermal runaway condition
  * 
  * @param[out] runaway_detected Pointer to store result (true if runaway detected)
  * @param[out] affected_module Pointer to store ID of affected module (if any)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_check_runaway(bool *runaway_detected, uint8_t *affected_module);
 
 /**
  * @brief Register a callback for thermal events
  * 
  * @param zone Thermal zone to trigger the callback
  * @param callback Function to call when thermal zone is entered
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_register_callback(thermal_zone_t zone,
                                           bess_event_callback_t callback,
                                           void *user_data);
 
 /**
  * @brief Unregister a previously registered callback
  * 
  * @param zone Thermal zone the callback was registered for
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_unregister_callback(thermal_zone_t zone,
                                             bess_event_callback_t callback);
 
 /**
  * @brief Get temperature history for a module
  * 
  * @param module_id ID of the module
  * @param[out] history_buffer Buffer to store temperature history
  * @param buffer_size Size of the history buffer
  * @param[out] samples_returned Pointer to store number of samples returned
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_history(uint8_t module_id,
                                     float *history_buffer,
                                     size_t buffer_size,
                                     size_t *samples_returned);
 
 /**
  * @brief Set the ambient temperature
  * 
  * @param ambient_temp Ambient temperature in °C
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_set_ambient_temp(float ambient_temp);
 
 /**
  * @brief Get the ambient temperature
  * 
  * @param[out] ambient_temp Pointer to store ambient temperature
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t thermal_monitor_get_ambient_temp(float *ambient_temp);
 
 #endif /* THERMAL_MONITOR_H */