/**
 * @file gpio_controller.h
 * @brief GPIO controller interface for BESS 100KW/200KWH system
 *
 * This module provides GPIO control functionality for the Battery Energy Storage System
 * main controller, handling digital I/O operations for:
 * - Safety circuits
 * - Status indicators
 * - Relay controls
 * - Communication interfaces
 * - Sensor inputs
 *
 * The controller is designed for the ESP32-P4 MCU platform using FreeRTOS.
 *
 * @copyright Copyright (c) 2025
 */

 #ifndef GPIO_CONTROLLER_H
 #define GPIO_CONTROLLER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "driver/gpio.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief GPIO pin purposes in BESS system
  */
 typedef enum {
     GPIO_BESS_RELAY_MAIN = 0,        /**< Main power relay control */
     GPIO_BESS_RELAY_PRECHARGE,       /**< Precharge relay control */
     GPIO_BESS_RELAY_AUXILIARY,       /**< Auxiliary relay control */
     GPIO_BESS_LED_FAULT,             /**< Fault indicator LED */
     GPIO_BESS_LED_RUNNING,           /**< Running indicator LED */
     GPIO_BESS_LED_CHARGING,          /**< Charging indicator LED */
     GPIO_BESS_LED_DISCHARGING,       /**< Discharging indicator LED */
     GPIO_BESS_FAN_CONTROL,           /**< Cooling fan control */
     GPIO_BESS_SD_CARD_DETECT,        /**< SD card detection pin */
     GPIO_BESS_ESTOP_INPUT,           /**< Emergency stop input */
     GPIO_BESS_EXT_ALARM_OUTPUT,      /**< External alarm output */
     GPIO_BESS_MODBUS_DE_RE,          /**< MODBUS driver enable/receiver enable */
     GPIO_BESS_CANBUS_STANDBY,        /**< CAN bus standby control */
     GPIO_BESS_BMS_ALERT,             /**< BMS alert input */
     GPIO_BESS_INVERTER_FAULT,        /**< Inverter fault input */
     GPIO_BESS_GRID_DETECT,           /**< Grid detection input */
     GPIO_BESS_AUX_IO_1,              /**< Auxiliary IO 1 */
     GPIO_BESS_AUX_IO_2,              /**< Auxiliary IO 2 */
     GPIO_BESS_AUX_IO_3,              /**< Auxiliary IO 3 */
     GPIO_BESS_AUX_IO_4,              /**< Auxiliary IO 4 */
     GPIO_BESS_COUNT                  /**< Total count of defined GPIOs */
 } gpio_bess_pin_t;
 
 /**
  * @brief GPIO pin configuration structure
  */
 typedef struct {
     gpio_num_t pin_num;              /**< ESP32 GPIO pin number */
     gpio_mode_t mode;                /**< Input, output, etc. */
     bool pull_up_en;                 /**< Enable pull-up resistor */
     bool pull_down_en;               /**< Enable pull-down resistor */
     bool invert_logic;               /**< Invert logical value (HIGH=0, LOW=1) */
     int interrupt_type;              /**< Interrupt type (if applicable) */
 } gpio_bess_config_t;
 
 /**
  * @brief GPIO controller error codes
  */
 typedef enum {
     GPIO_BESS_OK = 0,                /**< No error */
     GPIO_BESS_INVALID_PIN,           /**< Invalid pin identifier */
     GPIO_BESS_CONFIG_ERROR,          /**< Configuration error */
     GPIO_BESS_DRIVER_ERROR,          /**< ESP-IDF driver error */
     GPIO_BESS_NOT_INITIALIZED,       /**< Controller not initialized */
     GPIO_BESS_ALREADY_INITIALIZED    /**< Controller already initialized */
 } gpio_bess_error_t;
 
 /**
  * @brief GPIO interrupt callback function type
  */
 typedef void (*gpio_bess_interrupt_cb_t)(gpio_bess_pin_t pin, void* user_data);
 
 /**
  * @brief Initialize the GPIO controller
  *
  * This function initializes all defined GPIO pins with their default
  * configurations for the BESS system.
  *
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_init(void);
 
 /**
  * @brief Deinitialize the GPIO controller
  *
  * Releases all resources associated with the GPIO controller.
  *
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_deinit(void);
 
 /**
  * @brief Set the state of an output pin
  *
  * @param pin The GPIO pin identifier
  * @param level The level to set (0 = LOW, 1 = HIGH)
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_level(gpio_bess_pin_t pin, uint32_t level);
 
 /**
  * @brief Get the state of a pin
  *
  * @param pin The GPIO pin identifier
  * @param level Pointer to store the level (0 = LOW, 1 = HIGH)
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_get_level(gpio_bess_pin_t pin, uint32_t* level);
 
 /**
  * @brief Set custom configuration for a GPIO pin
  *
  * @param pin The GPIO pin identifier
  * @param config Pointer to the configuration structure
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_configure_pin(gpio_bess_pin_t pin, const gpio_bess_config_t* config);
 
 /**
  * @brief Register an interrupt handler for a GPIO pin
  *
  * @param pin The GPIO pin identifier
  * @param int_type Interrupt type (rising edge, falling edge, etc.)
  * @param callback The callback function to be called when interrupt occurs
  * @param user_data User data to be passed to the callback
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_register_interrupt(gpio_bess_pin_t pin, 
                                            gpio_int_type_t int_type,
                                            gpio_bess_interrupt_cb_t callback,
                                            void* user_data);
 
 /**
  * @brief Unregister an interrupt handler for a GPIO pin
  *
  * @param pin The GPIO pin identifier
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_unregister_interrupt(gpio_bess_pin_t pin);
 
 /**
  * @brief Enable an interrupt for a GPIO pin
  *
  * @param pin The GPIO pin identifier
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_enable_interrupt(gpio_bess_pin_t pin);
 
 /**
  * @brief Disable an interrupt for a GPIO pin
  *
  * @param pin The GPIO pin identifier
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_disable_interrupt(gpio_bess_pin_t pin);
 
 /**
  * @brief Check if a pin is configured as an input
  *
  * @param pin The GPIO pin identifier
  * @param is_input Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_input(gpio_bess_pin_t pin, bool* is_input);
 
 /**
  * @brief Check if a pin is configured as an output
  *
  * @param pin The GPIO pin identifier
  * @param is_output Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_output(gpio_bess_pin_t pin, bool* is_output);
 
 /**
  * @brief Safely control the main power relay
  * 
  * This function implements safety checks before activating the main power relay
  * to prevent dangerous conditions.
  *
  * @param enable TRUE to enable the relay, FALSE to disable
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_main_relay(bool enable);
 
 /**
  * @brief Control the precharge relay
  *
  * @param enable TRUE to enable the relay, FALSE to disable
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_precharge_relay(bool enable);
 
 /**
  * @brief Set the cooling fan duty cycle
  *
  * @param duty_percent Duty cycle percentage (0-100)
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_fan_duty(uint8_t duty_percent);
 
 /**
  * @brief Check if emergency stop is active
  *
  * @param is_active Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_estop_active(bool* is_active);
 
 /**
  * @brief Trigger external alarm
  *
  * @param enable TRUE to enable the alarm, FALSE to disable
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_external_alarm(bool enable);
 
 /**
  * @brief Set status LED states
  *
  * @param fault TRUE to enable fault LED
  * @param running TRUE to enable running LED
  * @param charging TRUE to enable charging LED
  * @param discharging TRUE to enable discharging LED
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_status_leds(bool fault, bool running, 
                                         bool charging, bool discharging);
 
 /**
  * @brief Check if an SD card is inserted
  *
  * @param present Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_sd_card_present(bool* present);
 
 /**
  * @brief Control the MODBUS DE/RE pin for RS-485 communication
  *
  * @param transmit_mode TRUE for transmit mode, FALSE for receive mode
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_modbus_mode(bool transmit_mode);
 
 /**
  * @brief Control the CAN bus standby mode
  *
  * @param standby TRUE for standby mode, FALSE for normal operation
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_set_canbus_standby(bool standby);
 
 /**
  * @brief Check if the BMS has triggered an alert
  *
  * @param alert_active Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_bms_alert_active(bool* alert_active);
 
 /**
  * @brief Check if the inverter has triggered a fault
  *
  * @param fault_active Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_inverter_fault_active(bool* fault_active);
 
 /**
  * @brief Check if grid power is detected
  *
  * @param grid_present Pointer to store the result
  * @return ESP_OK on success, or an error code
  */
 esp_err_t gpio_controller_is_grid_present(bool* grid_present);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* GPIO_CONTROLLER_H */