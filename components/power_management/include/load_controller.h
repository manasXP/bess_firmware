/**
 * @file load_controller.h
 * @brief Load Controller for BESS Power Management Module
 * 
 * Responsible for monitoring and controlling power flow between the battery system,
 * grid connection, and connected loads. Manages power setpoints, load balancing,
 * and operational mode transitions.
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef LOAD_CONTROLLER_H
 #define LOAD_CONTROLLER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "esp_log.h"
 
 #include "bess_types.h"
 #include "bess_config.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /** 
  * @brief Operation modes for the BESS load controller
  */
 typedef enum {
     LOAD_MODE_STANDBY = 0,      /*!< System in standby, no active power flow */
     LOAD_MODE_DISCHARGE,        /*!< System discharging (supplying power) */
     LOAD_MODE_CHARGE,           /*!< System charging (consuming power) */
     LOAD_MODE_IDLE,             /*!< System connected but not actively charging/discharging */
     LOAD_MODE_ISLAND,           /*!< System operating in island mode (off-grid) */
     LOAD_MODE_EMERGENCY,        /*!< Emergency mode triggered by critical conditions */
     LOAD_MODE_MAINTENANCE,      /*!< Maintenance mode for servicing */
     LOAD_MODE_TEST              /*!< Test mode for system validation */
 } load_controller_mode_t;
 
 /**
  * @brief Power flow directions for the BESS
  */
 typedef enum {
     POWER_FLOW_NONE = 0,        /*!< No power flow */
     POWER_FLOW_TO_GRID,         /*!< Power flowing to the grid */
     POWER_FLOW_FROM_GRID,       /*!< Power flowing from the grid */
     POWER_FLOW_TO_LOAD,         /*!< Power flowing to local loads */
     POWER_FLOW_BIDIRECTIONAL    /*!< Bidirectional power flow */
 } power_flow_direction_t;
 
 /**
  * @brief Configuration parameters for load controller
  */
 typedef struct {
     float max_charge_power_kw;          /*!< Maximum charging power in kW */
     float max_discharge_power_kw;       /*!< Maximum discharging power in kW */
     float min_power_setpoint_kw;        /*!< Minimum power setpoint in kW */
     float ramp_rate_kw_per_sec;         /*!< Ramp rate in kW/s for power changes */
     uint32_t power_update_interval_ms;  /*!< Interval between power updates in ms */
     uint8_t soc_reserve_percent;        /*!< SoC reserve level percentage */
     uint8_t soc_max_percent;            /*!< Maximum SoC level percentage */
     bool island_mode_enabled;           /*!< Whether island mode operation is enabled */
     bool grid_forming_enabled;          /*!< Whether grid-forming capability is enabled */
     bool load_limiting_enabled;         /*!< Whether load limiting is enabled */
 } load_controller_config_t;
 
 /**
  * @brief Status structure for load controller
  */
 typedef struct {
     load_controller_mode_t mode;        /*!< Current operational mode */
     power_flow_direction_t power_flow;  /*!< Current power flow direction */
     float current_power_kw;             /*!< Current power setpoint in kW */
     float measured_power_kw;            /*!< Actual measured power in kW */
     float grid_power_kw;                /*!< Power at grid connection point in kW */
     float load_power_kw;                /*!< Power consumed by loads in kW */
     float target_power_kw;              /*!< Target power setpoint in kW */
     uint32_t uptime_seconds;            /*!< Uptime in seconds */
     bool grid_connected;                /*!< Whether grid connection is established */
     bool converter_enabled;             /*!< Whether power converter is enabled */
     float dc_bus_voltage;               /*!< DC bus voltage */
 } load_controller_status_t;
 
 /**
  * @brief Load controller events that can be subscribed to
  */
 typedef enum {
     LOAD_EVENT_MODE_CHANGE = 0,         /*!< Operational mode changed */
     LOAD_EVENT_GRID_CONNECT,            /*!< Connected to grid */
     LOAD_EVENT_GRID_DISCONNECT,         /*!< Disconnected from grid */
     LOAD_EVENT_POWER_SETPOINT_CHANGE,   /*!< Power setpoint changed */
     LOAD_EVENT_POWER_LIMIT,             /*!< Power limit reached */
     LOAD_EVENT_CONVERTER_ENABLE,        /*!< Converter enabled */
     LOAD_EVENT_CONVERTER_DISABLE,       /*!< Converter disabled */
     LOAD_EVENT_ISLAND_MODE_ENTER,       /*!< Entered island mode */
     LOAD_EVENT_ISLAND_MODE_EXIT,        /*!< Exited island mode */
     LOAD_EVENT_EMERGENCY_CONDITION      /*!< Emergency condition detected */
 } load_controller_event_t;
 
 /**
  * @brief Callback function type for load controller events
  * 
  * @param event Event type
  * @param data Event-specific data
  * @param user_data User data provided during registration
  */
 typedef void (*load_controller_event_callback_t)(
     load_controller_event_t event,
     uint32_t data,
     void *user_data
 );
 
 /**
  * @brief Initialize the load controller with specified configuration
  * 
  * @param config Configuration parameters
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_init(const load_controller_config_t *config);
 
 /**
  * @brief Deinitialize and free resources used by the load controller
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_deinit(void);
 
 /**
  * @brief Start the load controller operation
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_start(void);
 
 /**
  * @brief Stop the load controller operation
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_stop(void);
 
 /**
  * @brief Set the operational mode of the load controller
  * 
  * @param mode Desired operational mode
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_set_mode(load_controller_mode_t mode);
 
 /**
  * @brief Get the current operational mode of the load controller
  * 
  * @param mode Pointer to store the current mode
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_get_mode(load_controller_mode_t *mode);
 
 /**
  * @brief Set power setpoint for the BESS
  * 
  * Positive values indicate discharging (power to grid/loads)
  * Negative values indicate charging (power from grid)
  * 
  * @param power_kw Power setpoint in kW
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_set_power(float power_kw);
 
 /**
  * @brief Get the current status of the load controller
  * 
  * @param status Pointer to store status information
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_get_status(load_controller_status_t *status);
 
 /**
  * @brief Connect to the grid (enable grid connection)
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_connect_grid(void);
 
 /**
  * @brief Disconnect from the grid (disable grid connection)
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_disconnect_grid(void);
 
 /**
  * @brief Enter island mode operation (off-grid)
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_enter_island_mode(void);
 
 /**
  * @brief Exit island mode operation (reconnect to grid)
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_exit_island_mode(void);
 
 /**
  * @brief Enable power converter
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_enable_converter(void);
 
 /**
  * @brief Disable power converter
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_disable_converter(void);
 
 /**
  * @brief Emergency shutdown procedure
  * 
  * @param reason Reason code for the emergency shutdown
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_emergency_shutdown(uint32_t reason);
 
 /**
  * @brief Check if charging is allowed
  * 
  * Validates system conditions to determine if charging is permitted
  * 
  * @param allowed Pointer to store the result
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_is_charging_allowed(bool *allowed);
 
 /**
  * @brief Check if discharging is allowed
  * 
  * Validates system conditions to determine if discharging is permitted
  * 
  * @param allowed Pointer to store the result
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_is_discharging_allowed(bool *allowed);
 
 /**
  * @brief Set maximum power limits
  * 
  * @param max_charge_kw Maximum charging power in kW
  * @param max_discharge_kw Maximum discharging power in kW
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_set_power_limits(float max_charge_kw, float max_discharge_kw);
 
 /**
  * @brief Register a callback for load controller events
  * 
  * @param event Event type to register for
  * @param callback Callback function
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_register_event_callback(
     load_controller_event_t event,
     load_controller_event_callback_t callback,
     void *user_data
 );
 
 /**
  * @brief Unregister a callback for load controller events
  * 
  * @param event Event type to unregister from
  * @param callback Callback function to unregister
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_unregister_event_callback(
     load_controller_event_t event,
     load_controller_event_callback_t callback
 );
 
 /**
  * @brief Run diagnostics on the load controller
  * 
  * @param results Pointer to store diagnostic results
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t load_controller_run_diagnostics(void *results);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* LOAD_CONTROLLER_H */