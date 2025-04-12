/**
 * @file converter_interface.h
 * @brief Interface for power converters in BESS
 * 
 * Provides abstraction layer for controlling DC-DC and DC-AC converters
 * in the 100KW/200KWH Battery Energy Storage System.
 */

 #ifndef CONVERTER_INTERFACE_H
 #define CONVERTER_INTERFACE_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 
 /**
  * @brief Converter types
  */
 typedef enum {
     CONVERTER_TYPE_DC_DC,      /**< DC-DC converter */
     CONVERTER_TYPE_DC_AC,      /**< DC-AC converter (inverter) */
     CONVERTER_TYPE_BIDIRECTIONAL /**< Bidirectional converter */
 } converter_type_t;
 
 /**
  * @brief Converter operation modes
  */
 typedef enum {
     CONVERTER_MODE_OFF,           /**< Converter is off */
     CONVERTER_MODE_STANDBY,       /**< Converter in standby */
     CONVERTER_MODE_RECTIFIER,     /**< AC to DC conversion (charging) */
     CONVERTER_MODE_INVERTER,      /**< DC to AC conversion (discharging) */
     CONVERTER_MODE_BUCK,          /**< DC-DC step-down */
     CONVERTER_MODE_BOOST,         /**< DC-DC step-up */
     CONVERTER_MODE_MPPT,          /**< Maximum Power Point Tracking (for PV) */
     CONVERTER_MODE_GRID_FORMING,  /**< Create grid reference */
     CONVERTER_MODE_GRID_FOLLOWING /**< Follow grid reference */
 } converter_mode_t;
 
 /**
  * @brief Converter status information
  */
 typedef struct {
     bool active;               /**< Converter is active */
     converter_mode_t mode;     /**< Current operation mode */
     float input_voltage;       /**< Input voltage (V) */
     float input_current;       /**< Input current (A) */
     float output_voltage;      /**< Output voltage (V) */
     float output_current;      /**< Output current (A) */
     float input_power;         /**< Input power (kW) */
     float output_power;        /**< Output power (kW) */
     float efficiency;          /**< Current efficiency (%) */
     float temperature;         /**< Converter temperature (°C) */
     uint32_t uptime;           /**< Uptime in seconds */
     uint32_t fault_code;       /**< Fault code if any */
     bool grid_connected;       /**< Grid connection status (inverters only) */
     float grid_frequency;      /**< Grid frequency (Hz) (inverters only) */
     uint32_t total_energy_in;  /**< Total energy input (Wh) */
     uint32_t total_energy_out; /**< Total energy output (Wh) */
 } converter_status_t;
 
 /**
  * @brief DC-DC converter configuration
  */
 typedef struct {
     float input_voltage_min;   /**< Minimum input voltage (V) */
     float input_voltage_max;   /**< Maximum input voltage (V) */
     float output_voltage_min;  /**< Minimum output voltage (V) */
     float output_voltage_max;  /**< Maximum output voltage (V) */
     float current_limit;       /**< Current limit (A) */
     float power_limit;         /**< Power limit (kW) */
     bool soft_start;           /**< Enable soft start */
     uint32_t soft_start_ms;    /**< Soft start duration (ms) */
 } dcdc_converter_config_t;
 
 /**
  * @brief DC-AC converter (inverter) configuration
  */
 typedef struct {
     float dc_voltage_min;      /**< Minimum DC voltage (V) */
     float dc_voltage_max;      /**< Maximum DC voltage (V) */
     float ac_voltage_nominal;  /**< Nominal AC voltage (V) */
     float ac_frequency_nominal; /**< Nominal AC frequency (Hz) */
     float power_limit;         /**< Power limit (kW) */
     bool grid_tie_enabled;     /**< Enable grid-tie operation */
     bool island_mode_enabled;  /**< Enable island mode operation */
     float power_factor;        /**< Target power factor (0.0-1.0) */
     bool reactive_power_enabled; /**< Enable reactive power control */
     float reactive_power_setpoint; /**< Reactive power setpoint (kVAR) */
 } inverter_config_t;
 
 /**
  * @brief Converter event types
  */
 typedef enum {
     CONVERTER_EVENT_STARTUP,      /**< Converter started up */
     CONVERTER_EVENT_SHUTDOWN,     /**< Converter shut down */
     CONVERTER_EVENT_MODE_CHANGE,  /**< Operation mode changed */
     CONVERTER_EVENT_FAULT,        /**< Fault occurred */
     CONVERTER_EVENT_WARNING,      /**< Warning condition */
     CONVERTER_EVENT_GRID_CONNECT, /**< Connected to grid */
     CONVERTER_EVENT_GRID_DISCONNECT /**< Disconnected from grid */
 } converter_event_t;
 
 /**
  * @brief Converter event callback function type
  */
 typedef void (*converter_event_callback_t)(uint8_t converter_id,
                                          converter_event_t event,
                                          void *event_data,
                                          void *user_data);
 
 /**
  * @brief Initialize the converter interface
  * 
  * @param converter_count Number of converters in the system
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_interface_init(uint8_t converter_count);
 
 /**
  * @brief Register a new converter in the system
  * 
  * @param converter_type Type of converter to register
  * @param[out] converter_id Pointer to store assigned converter ID
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_register(converter_type_t converter_type, uint8_t *converter_id);
 
 /**
  * @brief Configure a DC-DC converter
  * 
  * @param converter_id ID of the converter to configure
  * @param config Configuration parameters
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_configure_dcdc(uint8_t converter_id, 
                                  const dcdc_converter_config_t *config);
 
 /**
  * @brief Configure a DC-AC converter (inverter)
  * 
  * @param converter_id ID of the converter to configure
  * @param config Configuration parameters
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_configure_inverter(uint8_t converter_id, 
                                      const inverter_config_t *config);
 
 /**
  * @brief Set operation mode for a converter
  * 
  * @param converter_id ID of the converter
  * @param mode Operation mode to set
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_mode(uint8_t converter_id, converter_mode_t mode);
 
 /**
  * @brief Get current operation mode
  * 
  * @param converter_id ID of the converter
  * @param[out] mode Pointer to store current mode
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_get_mode(uint8_t converter_id, converter_mode_t *mode);
 
 /**
  * @brief Start a converter
  * 
  * @param converter_id ID of the converter
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_start(uint8_t converter_id);
 
 /**
  * @brief Stop a converter
  * 
  * @param converter_id ID of the converter
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_stop(uint8_t converter_id);
 
 /**
  * @brief Set output voltage for a DC-DC converter
  * 
  * @param converter_id ID of the converter
  * @param voltage_v Output voltage in volts
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_voltage(uint8_t converter_id, float voltage_v);
 
 /**
  * @brief Set current limit for a converter
  * 
  * @param converter_id ID of the converter
  * @param current_a Current limit in amps
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_current_limit(uint8_t converter_id, float current_a);
 
 /**
  * @brief Set power setpoint for a converter
  * 
  * @param converter_id ID of the converter
  * @param power_kw Power setpoint in kilowatts (+ for discharge, - for charge)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_power(uint8_t converter_id, float power_kw);
 
 /**
  * @brief Get current status of a converter
  * 
  * @param converter_id ID of the converter
  * @param[out] status Pointer to store status information
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_get_status(uint8_t converter_id, converter_status_t *status);
 
 /**
  * @brief Set AC output frequency for an inverter
  * 
  * @param converter_id ID of the inverter
  * @param frequency_hz Frequency in hertz
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_frequency(uint8_t converter_id, float frequency_hz);
 
 /**
  * @brief Set power factor for an inverter
  * 
  * @param converter_id ID of the inverter
  * @param power_factor Power factor (0.0-1.0)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_power_factor(uint8_t converter_id, float power_factor);
 
 /**
  * @brief Set reactive power for an inverter
  * 
  * @param converter_id ID of the inverter
  * @param reactive_power_kvar Reactive power in kVAR
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_reactive_power(uint8_t converter_id, float reactive_power_kvar);
 
 /**
  * @brief Connect inverter to grid
  * 
  * @param converter_id ID of the inverter
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_connect_grid(uint8_t converter_id);
 
 /**
  * @brief Disconnect inverter from grid
  * 
  * @param converter_id ID of the inverter
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_disconnect_grid(uint8_t converter_id);
 
 /**
  * @brief Reset a converter after a fault
  * 
  * @param converter_id ID of the converter
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_reset_fault(uint8_t converter_id);
 
 /**
  * @brief Register a callback for converter events
  * 
  * @param converter_id ID of the converter (or 0xFF for all converters)
  * @param event_type Event type to register for (or 0xFF for all events)
  * @param callback Function to call when event occurs
  * @param user_data User data to pass to callback
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_register_callback(uint8_t converter_id,
                                     uint8_t event_type,
                                     converter_event_callback_t callback,
                                     void *user_data);
 
 /**
  * @brief Unregister a previously registered callback
  * 
  * @param converter_id ID of the converter
  * @param event_type Event type the callback was registered for
  * @param callback Function that was previously registered
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_unregister_callback(uint8_t converter_id,
                                       uint8_t event_type,
                                       converter_event_callback_t callback);
 
 /**
  * @brief Perform self-test on a converter
  * 
  * @param converter_id ID of the converter
  * @param[out] result Pointer to store test result (0 = pass)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_self_test(uint8_t converter_id, uint32_t *result);
 
 /**
  * @brief Update firmware on a converter (if supported)
  * 
  * @param converter_id ID of the converter
  * @param firmware_data Pointer to firmware data
  * @param firmware_size Size of firmware data in bytes
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_update_firmware(uint8_t converter_id,
                                   const uint8_t *firmware_data,
                                   size_t firmware_size);
 
 /**
  * @brief Get firmware version from a converter
  * 
  * @param converter_id ID of the converter
  * @param[out] version_major Pointer to store major version
  * @param[out] version_minor Pointer to store minor version
  * @param[out] version_patch Pointer to store patch version
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_get_firmware_version(uint8_t converter_id,
                                        uint8_t *version_major,
                                        uint8_t *version_minor,
                                        uint8_t *version_patch);
 
 /**
  * @brief Get efficiency at current operating point
  * 
  * @param converter_id ID of the converter
  * @param[out] efficiency Pointer to store efficiency (%)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_get_efficiency(uint8_t converter_id, float *efficiency);
 
 /**
  * @brief Set maximum temperature threshold for a converter
  * 
  * @param converter_id ID of the converter
  * @param temp_c Maximum temperature in Celsius
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t converter_set_temperature_limit(uint8_t converter_id, float temp_c);
 
 #endif /* CONVERTER_INTERFACE_H */