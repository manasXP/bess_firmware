/**
 * @file power_manager.h
 * @brief Power management system for BESS
 * 
 * Provides interfaces for managing power flow, converter control,
 * charge/discharge operations, and grid interaction for the 
 * 100KW/200KWH Battery Energy Storage System.
 */

 #ifndef POWER_MANAGER_H
 #define POWER_MANAGER_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 
 /**
  * @brief System operation modes
  */
 typedef enum {
     POWER_MODE_STANDBY,          /**< System in standby (no power flow) */
     POWER_MODE_CHARGE,           /**< Battery charging from grid/source */
     POWER_MODE_DISCHARGE,        /**< Battery discharging to load/grid */
     POWER_MODE_IDLE,             /**< System on but no active power transfer */
     POWER_MODE_EMERGENCY,        /**< Emergency operation mode */
     POWER_MODE_MAINTENANCE       /**< Maintenance/service mode */
 } power_mode_t;
 
 /**
  * @brief Charging methods
  */
 typedef enum {
     CHARGE_METHOD_CC_CV,         /**< Constant current followed by constant voltage */
     CHARGE_METHOD_PULSED,        /**< Pulsed charging */
     CHARGE_METHOD_MULTI_STAGE,   /**< Multi-stage adaptive charging */
     CHARGE_METHOD_TRICKLE        /**< Trickle charging (low current) */
 } charge_method_t;
 
 /**
  * @brief Grid connection types
  */
 typedef enum {
     GRID_CONNECTION_ISOLATED,    /**< No grid connection (island mode) */
     GRID_CONNECTION_BACKUP,      /**< Grid backup only */
     GRID_CONNECTION_INTERACTIVE, /**< Interactive with grid (bi-directional) */
     GRID_CONNECTION_EXPORT       /**< Export to grid only */
 } grid_connection_t;
 
 /**
  * @brief Power Flow Direction
  */
 typedef enum {
     POWER_FLOW_NONE,             /**< No power flow */
     POWER_FLOW_GRID_TO_BATTERY,  /**< Grid to battery (charging) */
     POWER_FLOW_BATTERY_TO_GRID,  /**< Battery to grid (discharging) */
     POWER_FLOW_GRID_TO_LOAD,     /**< Grid directly to load */
     POWER_FLOW_BATTERY_TO_LOAD,  /**< Battery to load */
     POWER_FLOW_MIXED             /**< Mixed power flow */
 } power_flow_t;
 
 /**
  * @brief Charging parameters
  */
 typedef struct {
     float charge_current_max;    /**< Maximum charging current (A) */
     float charge_voltage_max;    /**< Maximum charging voltage (V) */
     float charge_power_max;      /**< Maximum charging power (kW) */
     charge_method_t method;      /**< Charging method to use */
     bool enable_balancing;       /**< Enable cell balancing during charging */
     uint32_t timeout_minutes;    /**< Timeout for charging operation (0 for no timeout) */
     float target_soc;            /**< Target state of charge (0-100%) */
     bool scheduled;              /**< Is this a scheduled charge */
 } charge_params_t;
 
 /**
  * @brief Discharging parameters
  */
 typedef struct {
     float discharge_current_max; /**< Maximum discharging current (A) */
     float discharge_power_max;   /**< Maximum discharging power (kW) */
     float min_voltage;           /**< Minimum battery voltage (V) */
     float min_soc;               /**< Minimum state of charge (0-100%) */
     uint32_t timeout_minutes;    /**< Timeout for discharging operation (0 for no timeout) */
     bool scheduled;              /**< Is this a scheduled discharge */
 } discharge_params_t;
 
 /**
  * @brief Power statistics
  */
 typedef struct {
     float instantaneous_power;   /**< Current power in kW (+ charging, - discharging) */
     float voltage;               /**< System voltage (V) */
     float current;               /**< System current (A) */
     float grid_frequency;        /**< Grid frequency (Hz) */
     float energy_charged_kwh;    /**< Total energy charged (kWh) */
     float energy_discharged_kwh; /**< Total energy discharged (kWh) */
     float efficiency;            /**< System efficiency (%) */
     float grid_voltage;          /**< Grid voltage (V) */
     float grid_current;          /**< Grid current (A) */
     float load_power;            /**< Load power (kW) */
     power_flow_t flow_direction; /**< Current power flow direction */
     power_mode_t current_mode;   /**< Current operation mode */
     float dc_bus_voltage;        /**< DC bus voltage (V) */
     uint32_t uptime_seconds;     /**< System uptime in seconds */
 } power_stats_t;
 
 /**
  * @brief Power schedule entry
  */
 typedef struct {
     bool enabled;                /**< Whether this entry is enabled */
     uint8_t day_of_week;         /**< Day of week (0=Sunday, 6=Saturday, 7=All days) */
     uint8_t hour;                /**< Hour (0-23) */
     uint8_t minute;              /**< Minute (0-59) */
     power_mode_t mode;           /**< Operation mode for this schedule */
     float power_setpoint;        /**< Power setpoint (+charging, -discharging) in kW */
     uint16_t duration_minutes;   /**< Duration in minutes */
     bool repeat;                 /**< Whether to repeat weekly */
 } power_schedule_entry_t;
 
 /**
  * @brief Power manager configuration
  */
 typedef struct {
     grid_connection_t grid_connection; /**< Grid connection type */
     uint8_t module_count;             /**< Number of battery modules */
     float system_voltage_nominal;     /**< Nominal system voltage (V) */
     float max_charge_power;           /**< Maximum charging power (kW) */
     float max_discharge_power;        /**< Maximum discharging power (kW) */
     float max_grid_export_power;      /**< Maximum power to export to grid (kW) */
     float min_soc;                    /**< Minimum allowed state of charge (%) */
     float max_soc;                    /**< Maximum allowed state of charge (%) */
     bool allow_grid_charging;         /**< Allow charging from grid */
     bool allow_grid_export;           /**< Allow exporting to grid */
     float inverter_efficiency;        /**< Inverter efficiency (%) */
     bool auto_restart;                /**< Auto restart after fault */
 } power_manager_config_t;
 
 /**
  * @brief Power manager event types
  */
 typedef enum {
     POWER_EVENT_MODE_CHANGE,     /**< Operation mode changed */
     POWER_EVENT_FLOW_CHANGE,     /**< Power flow direction changed */
     POWER_EVENT_CHARGE_COMPLETE, /**< Charging complete */
     POWER_EVENT_DISCHARGE_COMPLETE, /**< Discharging complete */
     POWER_EVENT_GRID_CONNECTED,  /**< Grid connection established */
     POWER_EVENT_GRID_DISCONNECTED, /**< Grid connection lost */
     POWER_EVENT_SCHEDULE_STARTED, /**< Scheduled operation started */
     POWER_EVENT_SCHEDULE_ENDED,  /**< Scheduled operation ended */
     POWER_EVENT_OVERPOWER,       /**< Over power condition */
     POWER_EVENT_EMERGENCY        /**< Emergency event */
 } power_event_t;
 
 /**
  * @brief Power manager event callback function type
  */
 typedef void (*power_event_callback_t)(power_event_t event, 
                                      void *event_data, 
                                      void *user_data);
 
 /**
  * @brief Initialize the power manager
  * 
  * @param config Configuration parameters
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_init(const power_manager_config_t *config);
 
 /**
  * @brief Start the power manager
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_start(void);
 
 /**
  * @brief Stop the power manager
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_stop(void);
 
 /**
  * @brief Set the operation mode
  * 
  * @param mode Operation mode to set
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_set_mode(power_mode_t mode);
 
 /**
  * @brief Get the current operation mode
  * 
  * @param[out] mode Pointer to store current mode
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_get_mode(power_mode_t *mode);
 
 /**
  * @brief Set charging parameters
  * 
  * @param params Charging parameters
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_set_charge_params(const charge_params_t *params);
 
 /**
  * @brief Set discharging parameters
  * 
  * @param params Discharging parameters
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_set_discharge_params(const discharge_params_t *params);
 
 /**
  * @brief Start charging operation
  * 
  * @param target_soc Target state of charge (0-100%, 0 for default)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_start_charging(float target_soc);
 
 /**
  * @brief Start discharging operation
  * 
  * @param power_kw Discharge power in kW (0 for default)
  * @param duration_minutes Duration in minutes (0 for unlimited)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_start_discharging(float power_kw, uint32_t duration_minutes);
 
 /**
  * @brief Stop the current operation (charging or discharging)
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_stop_operation(void);
 
 /**
  * @brief Get power statistics
  * 
  * @param[out] stats Pointer to store power statistics
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_get_stats(power_stats_t *stats);
 
 /**
  * @brief Register a callback for power events
  * 
  * @param event_type Event type to register for (or -1 for all events)
  * @param callback Function to call when event occurs
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_register_callback(int32_t event_type, 
                                         power_event_callback_t callback,
                                         void *user_data);
 
 /**
  * @brief Unregister a previously registered callback
  * 
  * @param event_type Event type the callback was registered for
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_unregister_callback(int32_t event_type,
                                           power_event_callback_t callback);
 
 /**
  * @brief Add a power schedule entry
  * 
  * @param entry Schedule entry to add
  * @param[out] entry_id Pointer to store assigned entry ID
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_add_schedule(const power_schedule_entry_t *entry, uint8_t *entry_id);
 
 /**
  * @brief Update a power schedule entry
  * 
  * @param entry_id ID of the entry to update
  * @param entry New schedule settings
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_update_schedule(uint8_t entry_id, const power_schedule_entry_t *entry);
 
 /**
  * @brief Delete a power schedule entry
  * 
  * @param entry_id ID of the entry to delete
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_delete_schedule(uint8_t entry_id);
 
 /**
  * @brief Get a power schedule entry
  * 
  * @param entry_id ID of the entry to get
  * @param[out] entry Pointer to store schedule entry
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_get_schedule(uint8_t entry_id, power_schedule_entry_t *entry);
 
 /**
  * @brief Get all schedule entries
  * 
  * @param[out] entries Buffer to store entries
  * @param max_entries Maximum number of entries to retrieve
  * @param[out] num_entries Pointer to store number of entries returned
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_get_all_schedules(power_schedule_entry_t *entries, 
                                         uint8_t max_entries,
                                         uint8_t *num_entries);
 
 /**
  * @brief Enable or disable all scheduling
  * 
  * @param enable Whether to enable scheduling
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_enable_scheduling(bool enable);
 
 /**
  * @brief Check if scheduling is enabled
  * 
  * @param[out] enabled Pointer to store result
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_is_scheduling_enabled(bool *enabled);
 
 /**
  * @brief Set constant power output/input
  * 
  * @param power_kw Power in kW (positive for discharge, negative for charge)
  * @param duration_minutes Duration in minutes (0 for unlimited)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_set_constant_power(float power_kw, uint32_t duration_minutes);
 
 /**
  * @brief Emergency shutdown
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_emergency_shutdown(void);
 
 /**
  * @brief Reset after emergency
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_reset_emergency(void);
 
 /**
  * @brief Check if system is in emergency state
  * 
  * @param[out] is_emergency Pointer to store result
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_is_emergency(bool *is_emergency);
 
 /**
  * @brief Set grid connection mode
  * 
  * @param mode Grid connection mode
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_set_grid_connection(grid_connection_t mode);
 
 /**
  * @brief Get current grid connection mode
  * 
  * @param[out] mode Pointer to store grid connection mode
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_get_grid_connection(grid_connection_t *mode);
 
 /**
  * @brief Update current power limits based on battery state
  * 
  * @param soc Current state of charge (0-100%)
  * @param max_charge_current Maximum allowed charge current (A)
  * @param max_discharge_current Maximum allowed discharge current (A)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t power_manager_update_power_limits(float soc, 
                                           float max_charge_current,
                                           float max_discharge_current);
 
 #endif /* POWER_MANAGER_H */