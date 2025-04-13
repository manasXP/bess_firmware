/**
 * @file sensor_interface.h
 * @brief Sensor interface definitions for BESS 100KW/200KWH system
 * 
 * This file defines the standardized interface for interacting with
 * various sensors in the BESS (Battery Energy Storage System).
 * It abstracts the hardware-specific details of different sensor types
 * to provide a unified API for the BMS (Battery Management System).
 * 
 * @note Designed for ESP32-P4 with FreeRTOS
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef SENSOR_INTERFACE_H
 #define SENSOR_INTERFACE_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Sensor types supported by the interface
  */
 typedef enum {
     SENSOR_TYPE_VOLTAGE,           /**< Voltage sensor */
     SENSOR_TYPE_CURRENT,           /**< Current sensor */
     SENSOR_TYPE_TEMPERATURE,       /**< Temperature sensor */
     SENSOR_TYPE_HUMIDITY,          /**< Humidity sensor (environment) */
     SENSOR_TYPE_PRESSURE,          /**< Pressure sensor */
     SENSOR_TYPE_FLOW,              /**< Flow sensor (for liquid cooling) */
     SENSOR_TYPE_DOOR_SWITCH,       /**< Door/enclosure access sensor */
     SENSOR_TYPE_SMOKE,             /**< Smoke detector */
     SENSOR_TYPE_CUSTOM,            /**< Custom sensor type */
     SENSOR_TYPE_MAX                /**< Max enum value for iteration */
 } sensor_type_t;
 
 /**
  * @brief Communication protocol used by sensor
  */
 typedef enum {
     SENSOR_PROTOCOL_I2C,           /**< I2C protocol */
     SENSOR_PROTOCOL_SPI,           /**< SPI protocol */
     SENSOR_PROTOCOL_ONEWIRE,       /**< 1-Wire protocol (Dallas/Maxim) */
     SENSOR_PROTOCOL_ANALOG,        /**< Analog input */
     SENSOR_PROTOCOL_DIGITAL,       /**< Digital input */
     SENSOR_PROTOCOL_MODBUS,        /**< Modbus RTU */
     SENSOR_PROTOCOL_CANBUS,        /**< CANBus */
     SENSOR_PROTOCOL_CUSTOM,        /**< Custom protocol */
     SENSOR_PROTOCOL_MAX            /**< Max enum value for iteration */
 } sensor_protocol_t;
 
 /**
  * @brief Sensor resolution/precision 
  */
 typedef enum {
     SENSOR_RESOLUTION_LOW,         /**< Low resolution */
     SENSOR_RESOLUTION_MEDIUM,      /**< Medium resolution */
     SENSOR_RESOLUTION_HIGH,        /**< High resolution */
     SENSOR_RESOLUTION_CUSTOM,      /**< Custom resolution */
     SENSOR_RESOLUTION_MAX          /**< Max enum value for iteration */
 } sensor_resolution_t;
 
 /**
  * @brief Sensor update frequency 
  */
 typedef enum {
     SENSOR_UPDATE_LOW,             /**< Low update frequency (>1s) */
     SENSOR_UPDATE_MEDIUM,          /**< Medium update frequency (100ms-1s) */
     SENSOR_UPDATE_HIGH,            /**< High update frequency (10-100ms) */
     SENSOR_UPDATE_VERY_HIGH,       /**< Very high update frequency (<10ms) */
     SENSOR_UPDATE_CUSTOM,          /**< Custom update frequency */
     SENSOR_UPDATE_MAX              /**< Max enum value for iteration */
 } sensor_update_frequency_t;
 
 /**
  * @brief Sensor value structure
  */
 typedef struct {
     union {
         float float_value;         /**< Floating point value */
         int32_t int_value;         /**< Integer value */
         bool bool_value;           /**< Boolean value */
         uint8_t *raw_data;         /**< Raw data pointer */
     } value;                       /**< Value union */
     
     uint32_t timestamp_ms;         /**< Timestamp in milliseconds */
     uint8_t data_len;              /**< Data length (for raw_data) */
     uint8_t valid;                 /**< Validity flag */
     uint8_t accuracy;              /**< Accuracy indicator (0-100%) */
 } sensor_value_t;
 
 /**
  * @brief Sensor configuration structure
  */
 typedef struct {
     sensor_type_t type;                    /**< Sensor type */
     sensor_protocol_t protocol;            /**< Communication protocol */
     sensor_resolution_t resolution;        /**< Sensor resolution */
     sensor_update_frequency_t update_freq; /**< Update frequency */
     
     union {
         struct {
             uint8_t bus;                   /**< I2C bus number */
             uint8_t address;               /**< I2C device address */
             uint32_t clock_speed;          /**< I2C clock speed in Hz */
         } i2c;                             /**< I2C configuration */
         
         struct {
             uint8_t host;                  /**< SPI host */
             uint8_t cs_pin;                /**< SPI chip select pin */
             uint32_t clock_speed;          /**< SPI clock speed in Hz */
         } spi;                             /**< SPI configuration */
         
         struct {
             uint8_t pin;                   /**< OneWire pin */
             uint8_t rom_code[8];           /**< Device ROM code */
         } onewire;                         /**< OneWire configuration */
         
         struct {
             uint8_t adc_channel;           /**< ADC channel */
             float conversion_factor;       /**< Conversion factor */
             float offset;                  /**< Offset value */
         } analog;                          /**< Analog configuration */
         
         struct {
             uint8_t pin;                   /**< Digital pin */
             uint8_t active_high;           /**< Active high flag */
         } digital;                         /**< Digital configuration */
         
         struct {
             uint8_t uart_num;              /**< UART port number */
             uint8_t device_addr;           /**< Modbus device address */
             uint16_t register_addr;        /**< Register address */
             uint8_t register_count;        /**< Number of registers */
         } modbus;                          /**< Modbus configuration */
         
         struct {
             uint8_t can_interface;         /**< CAN interface index */
             uint32_t can_id;               /**< CAN message ID */
             uint8_t data_byte_pos;         /**< Position in data bytes */
             uint8_t data_byte_len;         /**< Length in data bytes */
         } canbus;                          /**< CANBus configuration */
         
         void *custom_config;               /**< Custom configuration */
     } config;                              /**< Protocol-specific configuration */
     
     float min_value;                       /**< Minimum valid value */
     float max_value;                       /**< Maximum valid value */
     uint32_t sampling_interval_ms;         /**< Sampling interval in ms */
     float warning_threshold;               /**< Warning threshold */
     float critical_threshold;              /**< Critical threshold */
     float emergency_threshold;             /**< Emergency threshold */
     
     char name[32];                         /**< Sensor name */
     char location[32];                     /**< Sensor physical location */
     uint8_t module_id;                     /**< Associated battery module ID */
     uint8_t enabled;                       /**< Enabled flag */
 } sensor_config_t;
 
 /**
  * @brief Sensor handle
  */
 typedef void* sensor_handle_t;
 
 /**
  * @brief Sensor event types
  */
 typedef enum {
     SENSOR_EVENT_DATA_READY,       /**< New data is available */
     SENSOR_EVENT_THRESHOLD_WARNING, /**< Warning threshold exceeded */
     SENSOR_EVENT_THRESHOLD_CRITICAL, /**< Critical threshold exceeded */
     SENSOR_EVENT_THRESHOLD_EMERGENCY, /**< Emergency threshold exceeded */
     SENSOR_EVENT_ERROR,            /**< Sensor error */
     SENSOR_EVENT_CUSTOM,           /**< Custom event */
     SENSOR_EVENT_MAX               /**< Max enum value for iteration */
 } sensor_event_type_t;
 
 /**
  * @brief Sensor event data
  */
 typedef struct {
     sensor_event_type_t type;      /**< Event type */
     sensor_handle_t sensor;        /**< Sensor handle */
     sensor_value_t value;          /**< Sensor value */
     void *user_data;               /**< User data pointer */
 } sensor_event_t;
 
 /**
  * @brief Sensor event callback
  */
 typedef void (*sensor_event_cb_t)(sensor_event_t *event);
 
 /**
  * @brief Initialize the sensor subsystem
  *
  * This function initializes the sensor subsystem and prepares it for
  * registering and managing sensors.
  *
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_interface_init(void);
 
 /**
  * @brief Deinitialize the sensor subsystem
  *
  * This function cleans up resources used by the sensor subsystem.
  *
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_interface_deinit(void);
 
 /**
  * @brief Create a new sensor instance
  *
  * @param[in] config Sensor configuration
  * @param[out] handle Pointer to store the sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_create(const sensor_config_t *config, sensor_handle_t *handle);
 
 /**
  * @brief Delete a sensor instance
  *
  * @param[in] handle Sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_delete(sensor_handle_t handle);
 
 /**
  * @brief Start a sensor
  *
  * This function starts the sensor's data acquisition process.
  *
  * @param[in] handle Sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_start(sensor_handle_t handle);
 
 /**
  * @brief Stop a sensor
  *
  * This function stops the sensor's data acquisition process.
  *
  * @param[in] handle Sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_stop(sensor_handle_t handle);
 
 /**
  * @brief Read a sensor value
  *
  * This function reads the current value from a sensor.
  *
  * @param[in] handle Sensor handle
  * @param[out] value Pointer to store the sensor value
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_read(sensor_handle_t handle, sensor_value_t *value);
 
 /**
  * @brief Read multiple sensors in batch
  *
  * This function reads values from multiple sensors in a single operation.
  *
  * @param[in] handles Array of sensor handles
  * @param[out] values Array to store sensor values
  * @param[in] count Number of sensors to read
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_read_batch(const sensor_handle_t *handles, sensor_value_t *values, size_t count);
 
 /**
  * @brief Register a callback for sensor events
  *
  * @param[in] handle Sensor handle
  * @param[in] event_type Event type to register for
  * @param[in] callback Callback function
  * @param[in] user_data User data to pass to callback
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_register_event_callback(sensor_handle_t handle, 
                                          sensor_event_type_t event_type,
                                          sensor_event_cb_t callback,
                                          void *user_data);
 
 /**
  * @brief Unregister a callback for sensor events
  *
  * @param[in] handle Sensor handle
  * @param[in] event_type Event type to unregister
  * @param[in] callback Callback function to unregister
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_unregister_event_callback(sensor_handle_t handle,
                                            sensor_event_type_t event_type,
                                            sensor_event_cb_t callback);
 
 /**
  * @brief Set sensor thresholds
  *
  * This function sets the warning, critical, and emergency thresholds for a sensor.
  *
  * @param[in] handle Sensor handle
  * @param[in] warning_threshold Warning threshold value
  * @param[in] critical_threshold Critical threshold value
  * @param[in] emergency_threshold Emergency threshold value
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_set_thresholds(sensor_handle_t handle,
                                 float warning_threshold,
                                 float critical_threshold,
                                 float emergency_threshold);
 
 /**
  * @brief Get sensor configuration
  *
  * @param[in] handle Sensor handle
  * @param[out] config Pointer to store sensor configuration
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_get_config(sensor_handle_t handle, sensor_config_t *config);
 
 /**
  * @brief Update sensor configuration
  *
  * @param[in] handle Sensor handle
  * @param[in] config New sensor configuration
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_update_config(sensor_handle_t handle, const sensor_config_t *config);
 
 /**
  * @brief Enable a sensor
  *
  * @param[in] handle Sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_enable(sensor_handle_t handle);
 
 /**
  * @brief Disable a sensor
  *
  * @param[in] handle Sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_disable(sensor_handle_t handle);
 
 /**
  * @brief Run diagnostics on a sensor
  *
  * This function runs a diagnostics check on the sensor to verify its functionality.
  *
  * @param[in] handle Sensor handle
  * @param[out] diagnostics_passed Set to true if diagnostics passed
  * @param[out] error_code Optional error code if diagnostics failed
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_run_diagnostics(sensor_handle_t handle, 
                                 bool *diagnostics_passed, 
                                 uint32_t *error_code);
 
 /**
  * @brief Get a string representation of a sensor type
  *
  * @param[in] type Sensor type
  * @return String representation
  */
 const char* sensor_type_to_str(sensor_type_t type);
 
 /**
  * @brief Get a string representation of a sensor protocol
  *
  * @param[in] protocol Sensor protocol
  * @return String representation
  */
 const char* sensor_protocol_to_str(sensor_protocol_t protocol);
 
 /**
  * @brief Find sensors by type
  * 
  * This function finds all sensors of a specific type.
  * 
  * @param[in] type Sensor type to search for
  * @param[out] handles Array to store found sensor handles
  * @param[in] max_count Maximum number of handles to return
  * @param[out] count Actual number of handles found
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_find_by_type(sensor_type_t type, 
                              sensor_handle_t *handles, 
                              size_t max_count, 
                              size_t *count);
 
 /**
  * @brief Find sensors by module ID
  * 
  * This function finds all sensors associated with a specific battery module.
  * 
  * @param[in] module_id Module ID to search for
  * @param[out] handles Array to store found sensor handles
  * @param[in] max_count Maximum number of handles to return
  * @param[out] count Actual number of handles found
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_find_by_module(uint8_t module_id, 
                                sensor_handle_t *handles, 
                                size_t max_count, 
                                size_t *count);
 
 /**
  * @brief Set update interval for a sensor
  * 
  * @param[in] handle Sensor handle
  * @param[in] interval_ms Update interval in milliseconds
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_set_update_interval(sensor_handle_t handle, uint32_t interval_ms);
 
 /**
  * @brief Get the last error for a sensor
  * 
  * @param[in] handle Sensor handle
  * @param[out] error_code Error code
  * @param[out] error_msg Error message buffer
  * @param[in] error_msg_size Size of error message buffer
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_get_last_error(sensor_handle_t handle, 
                                uint32_t *error_code,
                                char *error_msg,
                                size_t error_msg_size);
 
 /**
  * @brief Check if a sensor is connected and responding
  * 
  * @param[in] handle Sensor handle
  * @param[out] is_connected Set to true if sensor is connected
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_is_connected(sensor_handle_t handle, bool *is_connected);
 
 /**
  * @brief Calibrate a sensor
  * 
  * This function performs calibration on a sensor.
  * 
  * @param[in] handle Sensor handle
  * @param[in] reference_value Reference value for calibration
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_calibrate(sensor_handle_t handle, float reference_value);
 
 /**
  * @brief Reset a sensor to default settings
  * 
  * @param[in] handle Sensor handle
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_reset(sensor_handle_t handle);
 
 /**
  * @brief Apply a custom command to a sensor
  * 
  * This function allows sending custom commands to sensors that support extended functionality.
  * 
  * @param[in] handle Sensor handle
  * @param[in] command Command identifier
  * @param[in] in_data Input data buffer
  * @param[in] in_size Size of input data
  * @param[out] out_data Output data buffer
  * @param[in,out] out_size Size of output buffer / actual output size
  * @return ESP_OK on success, or an error code
  */
 esp_err_t sensor_custom_command(sensor_handle_t handle,
                                uint32_t command,
                                const void *in_data,
                                size_t in_size,
                                void *out_data,
                                size_t *out_size);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* SENSOR_INTERFACE_H */