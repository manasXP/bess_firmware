/**
 * @file gpio_controller.c
 * @brief Implementation of GPIO controller for BESS 100KW/200KWH system
 *
 * This module implements the GPIO control functionality for the Battery Energy Storage System
 * main controller, handling all digital I/O operations.
 *
 * @copyright Copyright (c) 2025
 */

 #include "gpio_controller.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "driver/ledc.h"
 #include "esp_timer.h"
 
 #define TAG "GPIO_CTRL"
 
 /**
  * @brief Default GPIO pin mappings for BESS system
  * These can be overridden via the configuration API
  */
 static const gpio_num_t DEFAULT_PIN_MAP[GPIO_BESS_COUNT] = {
     [GPIO_BESS_RELAY_MAIN]       = GPIO_NUM_4,
     [GPIO_BESS_RELAY_PRECHARGE]  = GPIO_NUM_5,
     [GPIO_BESS_RELAY_AUXILIARY]  = GPIO_NUM_6,
     [GPIO_BESS_LED_FAULT]        = GPIO_NUM_7,
     [GPIO_BESS_LED_RUNNING]      = GPIO_NUM_15,
     [GPIO_BESS_LED_CHARGING]     = GPIO_NUM_16,
     [GPIO_BESS_LED_DISCHARGING]  = GPIO_NUM_17,
     [GPIO_BESS_FAN_CONTROL]      = GPIO_NUM_18,  // PWM capable pin
     [GPIO_BESS_SD_CARD_DETECT]   = GPIO_NUM_19,
     [GPIO_BESS_ESTOP_INPUT]      = GPIO_NUM_34,  // Input only
     [GPIO_BESS_EXT_ALARM_OUTPUT] = GPIO_NUM_21,
     [GPIO_BESS_MODBUS_DE_RE]     = GPIO_NUM_22,
     [GPIO_BESS_CANBUS_STANDBY]   = GPIO_NUM_23,
     [GPIO_BESS_BMS_ALERT]        = GPIO_NUM_35,  // Input only
     [GPIO_BESS_INVERTER_FAULT]   = GPIO_NUM_36,  // Input only
     [GPIO_BESS_GRID_DETECT]      = GPIO_NUM_39,  // Input only
     [GPIO_BESS_AUX_IO_1]         = GPIO_NUM_25,
     [GPIO_BESS_AUX_IO_2]         = GPIO_NUM_26,
     [GPIO_BESS_AUX_IO_3]         = GPIO_NUM_27,
     [GPIO_BESS_AUX_IO_4]         = GPIO_NUM_32
 
 
 esp_err_t gpio_controller_is_input(gpio_bess_pin_t pin, bool* is_input)
 {
     // Validate parameters
     if (!is_valid_pin(pin) || is_input == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Check if pin is configured as input
     *is_input = (s_gpio_state.pins[pin].config.mode == GPIO_MODE_INPUT ||
                 s_gpio_state.pins[pin].config.mode == GPIO_MODE_INPUT_OUTPUT);
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_is_output(gpio_bess_pin_t pin, bool* is_output)
 {
     // Validate parameters
     if (!is_valid_pin(pin) || is_output == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Check if pin is configured as output
     *is_output = (s_gpio_state.pins[pin].config.mode == GPIO_MODE_OUTPUT ||
                  s_gpio_state.pins[pin].config.mode == GPIO_MODE_INPUT_OUTPUT);
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_set_main_relay(bool enable)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     esp_err_t ret = ESP_OK;
     
     // Implement safety checks when enabling the main relay
     if (enable) {
         // Check if E-Stop is active
         bool estop_active = false;
         ret = gpio_controller_is_estop_active(&estop_active);
         if (ret != ESP_OK || estop_active) {
             ESP_LOGE(TAG, "Cannot enable main relay: E-STOP active or error checking E-STOP");
             xSemaphoreGive(s_gpio_state.mutex);
             return ret != ESP_OK ? ret : ESP_FAIL;
         }
         
         // Check if precharge is complete
         if (!s_gpio_state.precharge_active) {
             ESP_LOGE(TAG, "Cannot enable main relay: Precharge not active");
             xSemaphoreGive(s_gpio_state.mutex);
             return ESP_FAIL;
         }
         
         // Check precharge time
         int64_t current_time = esp_timer_get_time();
         int64_t precharge_time_ms = (current_time - s_gpio_state.precharge_start_time) / 1000;
         
         if (precharge_time_ms < PRECHARGE_MIN_TIME_MS) {
             ESP_LOGE(TAG, "Cannot enable main relay: Precharge time too short (%lld ms)", precharge_time_ms);
             xSemaphoreGive(s_gpio_state.mutex);
             return ESP_FAIL;
         }
         
         // All safety checks passed, enable the main relay
         ret = gpio_controller_set_level(GPIO_BESS_RELAY_MAIN, 1);
         
         // Log main relay activation
         if (ret == ESP_OK) {
             ESP_LOGI(TAG, "Main relay enabled after %lld ms precharge", precharge_time_ms);
         }
     } else {
         // When disabling, just do it (safety first - always allow turning off)
         ret = gpio_controller_set_level(GPIO_BESS_RELAY_MAIN, 0);
         
         // Also disable precharge relay if main relay is disabled
         if (ret == ESP_OK) {
             ret = gpio_controller_set_level(GPIO_BESS_RELAY_PRECHARGE, 0);
             s_gpio_state.precharge_active = false;
             ESP_LOGI(TAG, "Main and precharge relays disabled");
         }
     }
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_set_precharge_relay(bool enable)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     esp_err_t ret = ESP_OK;
     
     // Implement safety logic for precharge
     if (enable) {
         // Check if main relay is off before activating precharge
         uint32_t main_relay_level = 0;
         ret = gpio_controller_get_level(GPIO_BESS_RELAY_MAIN, &main_relay_level);
         
         if (ret != ESP_OK || main_relay_level != 0) {
             ESP_LOGE(TAG, "Cannot enable precharge: Main relay is active or error checking main relay");
             xSemaphoreGive(s_gpio_state.mutex);
             return ret != ESP_OK ? ret : ESP_FAIL;
         }
         
         // Check if E-Stop is active
         bool estop_active = false;
         ret = gpio_controller_is_estop_active(&estop_active);
         if (ret != ESP_OK || estop_active) {
             ESP_LOGE(TAG, "Cannot enable precharge: E-STOP active or error checking E-STOP");
             xSemaphoreGive(s_gpio_state.mutex);
             return ret != ESP_OK ? ret : ESP_FAIL;
         }
         
         // Enable precharge relay
         ret = gpio_controller_set_level(GPIO_BESS_RELAY_PRECHARGE, 1);
         
         // Record precharge start time
         if (ret == ESP_OK) {
             s_gpio_state.precharge_active = true;
             s_gpio_state.precharge_start_time = esp_timer_get_time();
             ESP_LOGI(TAG, "Precharge relay enabled");
         }
     } else {
         // When disabling, just do it
         ret = gpio_controller_set_level(GPIO_BESS_RELAY_PRECHARGE, 0);
         s_gpio_state.precharge_active = false;
         ESP_LOGI(TAG, "Precharge relay disabled");
     }
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_set_fan_duty(uint8_t duty_percent)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate duty cycle range
     if (duty_percent > 100) {
         duty_percent = 100;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Calculate duty value for LEDC (10-bit resolution: 0-1023)
     uint32_t duty = (duty_percent * 1023) / 100;
     
     // Set duty cycle
     esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, s_gpio_state.fan_ledc_channel, duty);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set fan duty cycle: %d", ret);
         xSemaphoreGive(s_gpio_state.mutex);
         return ret;
     }
     
     // Update duty
     ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, s_gpio_state.fan_ledc_channel);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to update fan duty cycle: %d", ret);
     } else {
         ESP_LOGI(TAG, "Fan duty cycle set to %d%%", duty_percent);
     }
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_is_estop_active(bool* is_active)
 {
     // Validate parameters
     if (is_active == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Get ESTOP input level
     uint32_t level = 0;
     esp_err_t ret = gpio_controller_get_level(GPIO_BESS_ESTOP_INPUT, &level);
     
     // ESTOP is active when level is HIGH (after any configured inversion)
     if (ret == ESP_OK) {
         *is_active = (level == 1);
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_set_external_alarm(bool enable)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set alarm output level
     esp_err_t ret = gpio_controller_set_level(GPIO_BESS_EXT_ALARM_OUTPUT, enable ? 1 : 0);
     
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "External alarm %s", enable ? "activated" : "deactivated");
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_set_status_leds(bool fault, bool running, bool charging, bool discharging)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Set all status LEDs
     esp_err_t ret;
     
     ret = gpio_controller_set_level(GPIO_BESS_LED_FAULT, fault ? 1 : 0);
     if (ret != ESP_OK) {
         xSemaphoreGive(s_gpio_state.mutex);
         return ret;
     }
     
     ret = gpio_controller_set_level(GPIO_BESS_LED_RUNNING, running ? 1 : 0);
     if (ret != ESP_OK) {
         xSemaphoreGive(s_gpio_state.mutex);
         return ret;
     }
     
     ret = gpio_controller_set_level(GPIO_BESS_LED_CHARGING, charging ? 1 : 0);
     if (ret != ESP_OK) {
         xSemaphoreGive(s_gpio_state.mutex);
         return ret;
     }
     
     ret = gpio_controller_set_level(GPIO_BESS_LED_DISCHARGING, discharging ? 1 : 0);
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_is_sd_card_present(bool* present)
 {
     // Validate parameters
     if (present == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Get SD card detect pin level
     uint32_t level = 0;
     esp_err_t ret = gpio_controller_get_level(GPIO_BESS_SD_CARD_DETECT, &level);
     
     // Card is present when level is HIGH (after any configured inversion)
     if (ret == ESP_OK) {
         *present = (level == 1);
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_set_modbus_mode(bool transmit_mode)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set MODBUS DE/RE pin level
     // HIGH = transmit mode, LOW = receive mode
     esp_err_t ret = gpio_controller_set_level(GPIO_BESS_MODBUS_DE_RE, transmit_mode ? 1 : 0);
     
     if (ret == ESP_OK) {
         ESP_LOGD(TAG, "MODBUS set to %s mode", transmit_mode ? "transmit" : "receive");
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_set_canbus_standby(bool standby)
 {
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Set CAN bus standby pin level
     // HIGH = standby mode, LOW = normal operation
     esp_err_t ret = gpio_controller_set_level(GPIO_BESS_CANBUS_STANDBY, standby ? 1 : 0);
     
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "CAN bus set to %s mode", standby ? "standby" : "normal operation");
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_is_bms_alert_active(bool* alert_active)
 {
     // Validate parameters
     if (alert_active == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Get BMS alert pin level
     uint32_t level = 0;
     esp_err_t ret = gpio_controller_get_level(GPIO_BESS_BMS_ALERT, &level);
     
     // Alert is active when level is HIGH (after any configured inversion)
     if (ret == ESP_OK) {
         *alert_active = (level == 1);
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_is_inverter_fault_active(bool* fault_active)
 {
     // Validate parameters
     if (fault_active == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Get inverter fault pin level
     uint32_t level = 0;
     esp_err_t ret = gpio_controller_get_level(GPIO_BESS_INVERTER_FAULT, &level);
     
     // Fault is active when level is HIGH (after any configured inversion)
     if (ret == ESP_OK) {
         *fault_active = (level == 1);
     }
     
     return ret;
 }
 
 esp_err_t gpio_controller_is_grid_present(bool* grid_present)
 {
     // Validate parameters
     if (grid_present == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Get grid detect pin level
     uint32_t level = 0;
     esp_err_t ret = gpio_controller_get_level(GPIO_BESS_GRID_DETECT, &level);
     
     // Grid is present when level is HIGH (after any configured inversion)
     if (ret == ESP_OK) {
         *grid_present = (level == 1);
     }
     
     return ret;
 }
 };
 
 /**
  * @brief Default configuration for each GPIO pin
  */
 static const gpio_bess_config_t DEFAULT_PIN_CONFIG[GPIO_BESS_COUNT] = {
     [GPIO_BESS_RELAY_MAIN] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_RELAY_PRECHARGE] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_RELAY_AUXILIARY] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_LED_FAULT] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_LED_RUNNING] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_LED_CHARGING] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_LED_DISCHARGING] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_FAN_CONTROL] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_SD_CARD_DETECT] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = true,  // Usually active low
         .interrupt_type = GPIO_INTR_ANYEDGE
     },
     [GPIO_BESS_ESTOP_INPUT] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = true,  // Active low (NC contact)
         .interrupt_type = GPIO_INTR_ANYEDGE
     },
     [GPIO_BESS_EXT_ALARM_OUTPUT] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_MODBUS_DE_RE] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_CANBUS_STANDBY] = {
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_BMS_ALERT] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = true,  // Active low
         .interrupt_type = GPIO_INTR_NEGEDGE
     },
     [GPIO_BESS_INVERTER_FAULT] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = true,  // Active low
         .interrupt_type = GPIO_INTR_NEGEDGE
     },
     [GPIO_BESS_GRID_DETECT] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = false,
         .pull_down_en = true,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_ANYEDGE
     },
     [GPIO_BESS_AUX_IO_1] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_AUX_IO_2] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_AUX_IO_3] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     },
     [GPIO_BESS_AUX_IO_4] = {
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = true,
         .pull_down_en = false,
         .invert_logic = false,
         .interrupt_type = GPIO_INTR_DISABLE
     }
 };
 
 /**
  * @brief Structure to hold current pin configurations
  */
 typedef struct {
     gpio_num_t pin_num;                    /**< ESP32 GPIO pin number */
     gpio_bess_config_t config;             /**< Current configuration */
     bool initialized;                      /**< Whether pin is initialized */
     gpio_bess_interrupt_cb_t callback;     /**< Interrupt callback function */
     void* user_data;                       /**< User data for callback */
 } gpio_pin_state_t;
 
 /**
  * @brief Structure for GPIO controller state
  */
 typedef struct {
     gpio_pin_state_t pins[GPIO_BESS_COUNT];  /**< Pin states */
     bool initialized;                       /**< Whether controller is initialized */
     SemaphoreHandle_t mutex;               /**< Mutex for thread safety */
     ledc_channel_t fan_ledc_channel;       /**< LEDC channel for fan control */
     ledc_timer_t fan_ledc_timer;           /**< LEDC timer for fan control */
     bool precharge_active;                 /**< Whether precharge is active */
     int64_t precharge_start_time;          /**< When precharge started (us) */
 } gpio_controller_state_t;
 
 // Global state for the GPIO controller
 static gpio_controller_state_t s_gpio_state = {
     .initialized = false,
     .mutex = NULL,
     .fan_ledc_channel = LEDC_CHANNEL_0,
     .fan_ledc_timer = LEDC_TIMER_0,
     .precharge_active = false,
     .precharge_start_time = 0
 };
 
 // ISR service handle
 static intr_handle_t s_gpio_isr_handle = NULL;
 
 // LEDC configuration for fan PWM control
 #define LEDC_TIMER_RESOLUTION LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023)
 #define LEDC_BASE_FREQ        25000              // 25 kHz for fan PWM
 
 // Safety parameters
 #define PRECHARGE_MIN_TIME_MS 1000               // Minimum precharge time (ms)
 #define PRECHARGE_MAX_TIME_MS 10000              // Maximum precharge time (ms)
 
 /**
  * @brief GPIO ISR handler
  * 
  * @param arg Argument passed to ISR handler (pin number)
  */
 static void IRAM_ATTR gpio_isr_handler(void* arg)
 {
     uint32_t gpio_num = (uint32_t)arg;
     
     // Find which logical pin this corresponds to
     for (int i = 0; i < GPIO_BESS_COUNT; i++) {
         if (s_gpio_state.pins[i].pin_num == gpio_num && 
             s_gpio_state.pins[i].callback != NULL) {
             
             // Call the registered callback
             s_gpio_state.pins[i].callback((gpio_bess_pin_t)i, s_gpio_state.pins[i].user_data);
             break;
         }
     }
 }
 
 /**
  * @brief Validate a pin identifier
  * 
  * @param pin The GPIO pin identifier to validate
  * @return true if valid, false otherwise
  */
 static bool is_valid_pin(gpio_bess_pin_t pin)
 {
     return (pin >= 0 && pin < GPIO_BESS_COUNT);
 }
 
 /**
  * @brief Apply logic inversion if configured
  * 
  * @param pin The GPIO pin identifier
  * @param value The logical value
  * @return The physical value after applying inversion
  */
 static uint32_t apply_logic_inversion(gpio_bess_pin_t pin, uint32_t value)
 {
     if (s_gpio_state.pins[pin].config.invert_logic) {
         return value ? 0 : 1;
     }
     return value;
 }
 
 /**
  * @brief Initialize LEDC for fan PWM control
  * 
  * @return ESP_OK on success, or error code
  */
 static esp_err_t init_fan_pwm(void)
 {
     // Configure timer
     ledc_timer_config_t ledc_timer = {
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .timer_num = s_gpio_state.fan_ledc_timer,
         .duty_resolution = LEDC_TIMER_RESOLUTION,
         .freq_hz = LEDC_BASE_FREQ,
         .clk_cfg = LEDC_AUTO_CLK
     };
     
     esp_err_t ret = ledc_timer_config(&ledc_timer);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure LEDC timer: %d", ret);
         return ret;
     }
     
     // Configure channel
     ledc_channel_config_t ledc_channel = {
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel = s_gpio_state.fan_ledc_channel,
         .timer_sel = s_gpio_state.fan_ledc_timer,
         .intr_type = LEDC_INTR_DISABLE,
         .gpio_num = s_gpio_state.pins[GPIO_BESS_FAN_CONTROL].pin_num,
         .duty = 0, // Initially off
         .hpoint = 0
     };
     
     ret = ledc_channel_config(&ledc_channel);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure LEDC channel: %d", ret);
         return ret;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Configure a GPIO pin
  * 
  * @param pin The logical pin identifier
  * @param config The pin configuration
  * @return ESP_OK on success, or an error code
  */
 static esp_err_t configure_gpio_pin(gpio_bess_pin_t pin, const gpio_bess_config_t* config)
 {
     if (!is_valid_pin(pin)) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (pin == GPIO_BESS_FAN_CONTROL) {
         // Fan is controlled via LEDC (PWM), so we don't configure it as a regular GPIO
         s_gpio_state.pins[pin].config = *config;
         s_gpio_state.pins[pin].initialized = true;
         return ESP_OK;
     }
     
     gpio_config_t gpio_conf = {
         .pin_bit_mask = (1ULL << s_gpio_state.pins[pin].pin_num),
         .mode = config->mode,
         .pull_up_en = config->pull_up_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
         .pull_down_en = config->pull_down_en ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
         .intr_type = config->interrupt_type
     };
     
     esp_err_t ret = gpio_config(&gpio_conf);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure pin %d: %d", pin, ret);
         return ret;
     }
     
     // Store the configuration
     s_gpio_state.pins[pin].config = *config;
     s_gpio_state.pins[pin].initialized = true;
     
     // If this is an output, set initial level to 0
     if (config->mode == GPIO_MODE_OUTPUT) {
         uint32_t level = apply_logic_inversion(pin, 0);
         gpio_set_level(s_gpio_state.pins[pin].pin_num, level);
     }
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_init(void)
 {
     esp_err_t ret;
     
     // Check if already initialized
     if (s_gpio_state.initialized) {
         ESP_LOGW(TAG, "GPIO controller already initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create mutex for thread safety
     s_gpio_state.mutex = xSemaphoreCreateMutex();
     if (s_gpio_state.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize GPIO ISR handler
     ret = gpio_install_isr_service(0);
     if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
         // ESP_ERR_INVALID_STATE is returned if already installed, which is fine
         ESP_LOGE(TAG, "Failed to install GPIO ISR service: %d", ret);
         vSemaphoreDelete(s_gpio_state.mutex);
         s_gpio_state.mutex = NULL;
         return ret;
     }
     
     // Initialize all pins with default configuration
     for (int i = 0; i < GPIO_BESS_COUNT; i++) {
         // Set pin number from default mapping
         s_gpio_state.pins[i].pin_num = DEFAULT_PIN_MAP[i];
         s_gpio_state.pins[i].callback = NULL;
         s_gpio_state.pins[i].user_data = NULL;
         s_gpio_state.pins[i].initialized = false;
         
         // Apply default configuration
         ret = configure_gpio_pin((gpio_bess_pin_t)i, &DEFAULT_PIN_CONFIG[i]);
         if (ret != ESP_OK) {
             ESP_LOGE(TAG, "Failed to configure pin %d: %d", i, ret);
             gpio_controller_deinit();
             return ret;
         }
     }
     
     // Initialize PWM for fan control
     ret = init_fan_pwm();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize fan PWM: %d", ret);
         gpio_controller_deinit();
         return ret;
     }
     
     // Mark the controller as initialized
     s_gpio_state.initialized = true;
     ESP_LOGI(TAG, "GPIO controller initialized successfully");
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_deinit(void)
 {
     // Check if already deinitialized
     if (!s_gpio_state.initialized) {
         ESP_LOGW(TAG, "GPIO controller already deinitialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Set all outputs to safe state
     for (int i = 0; i < GPIO_BESS_COUNT; i++) {
         if (s_gpio_state.pins[i].initialized &&
             s_gpio_state.pins[i].config.mode == GPIO_MODE_OUTPUT) {
             
             // Set all outputs low (considering logic inversion)
             uint32_t level = apply_logic_inversion((gpio_bess_pin_t)i, 0);
             gpio_set_level(s_gpio_state.pins[i].pin_num, level);
         }
     }
     
     // Stop fan PWM
     ledc_stop(LEDC_LOW_SPEED_MODE, s_gpio_state.fan_ledc_channel, 0);
     
     // Uninstall GPIO ISR service
     gpio_uninstall_isr_service();
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     vSemaphoreDelete(s_gpio_state.mutex);
     s_gpio_state.mutex = NULL;
     
     // Mark as deinitialized
     s_gpio_state.initialized = false;
     ESP_LOGI(TAG, "GPIO controller deinitialized");
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_set_level(gpio_bess_pin_t pin, uint32_t level)
 {
     // Validate parameters
     if (!is_valid_pin(pin)) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if pin is configured as output
     if (s_gpio_state.pins[pin].config.mode != GPIO_MODE_OUTPUT) {
         ESP_LOGW(TAG, "Pin %d is not configured as output", pin);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Apply logic inversion if configured
     uint32_t physical_level = apply_logic_inversion(pin, level);
     
     // Set the pin level
     esp_err_t ret = gpio_set_level(s_gpio_state.pins[pin].pin_num, physical_level);
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_get_level(gpio_bess_pin_t pin, uint32_t* level)
 {
     // Validate parameters
     if (!is_valid_pin(pin) || level == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Get the pin level
     int physical_level = gpio_get_level(s_gpio_state.pins[pin].pin_num);
     
     // Apply logic inversion if configured
     if (s_gpio_state.pins[pin].config.invert_logic) {
         *level = physical_level ? 0 : 1;
     } else {
         *level = physical_level;
     }
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_configure_pin(gpio_bess_pin_t pin, const gpio_bess_config_t* config)
 {
     // Validate parameters
     if (!is_valid_pin(pin) || config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // If this pin has an interrupt, disable it first
     if (s_gpio_state.pins[pin].initialized && 
         s_gpio_state.pins[pin].config.interrupt_type != GPIO_INTR_DISABLE) {
         gpio_isr_handler_remove(s_gpio_state.pins[pin].pin_num);
     }
     
     // Apply the new configuration
     esp_err_t ret = configure_gpio_pin(pin, config);
     
     // If this pin has an interrupt, re-enable it
     if (ret == ESP_OK && 
         config->interrupt_type != GPIO_INTR_DISABLE &&
         s_gpio_state.pins[pin].callback != NULL) {
         
         gpio_isr_handler_add(s_gpio_state.pins[pin].pin_num, 
                            gpio_isr_handler, 
                            (void*)s_gpio_state.pins[pin].pin_num);
     }
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_register_interrupt(gpio_bess_pin_t pin, 
                                           gpio_int_type_t int_type,
                                           gpio_bess_interrupt_cb_t callback,
                                           void* user_data)
 {
     // Validate parameters
     if (!is_valid_pin(pin) || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Remove any existing ISR handler
     if (s_gpio_state.pins[pin].config.interrupt_type != GPIO_INTR_DISABLE) {
         gpio_isr_handler_remove(s_gpio_state.pins[pin].pin_num);
     }
     
     // Set the new callback and user data
     s_gpio_state.pins[pin].callback = callback;
     s_gpio_state.pins[pin].user_data = user_data;
     
     // Update interrupt type if needed
     if (s_gpio_state.pins[pin].config.interrupt_type != int_type) {
         gpio_bess_config_t new_config = s_gpio_state.pins[pin].config;
         new_config.interrupt_type = int_type;
         
         esp_err_t ret = configure_gpio_pin(pin, &new_config);
         if (ret != ESP_OK) {
             s_gpio_state.pins[pin].callback = NULL;
             s_gpio_state.pins[pin].user_data = NULL;
             xSemaphoreGive(s_gpio_state.mutex);
             return ret;
         }
     }
     
     // Add ISR handler
     esp_err_t ret = gpio_isr_handler_add(s_gpio_state.pins[pin].pin_num, 
                                       gpio_isr_handler, 
                                       (void*)(uintptr_t)s_gpio_state.pins[pin].pin_num);
     
     if (ret != ESP_OK) {
         s_gpio_state.pins[pin].callback = NULL;
         s_gpio_state.pins[pin].user_data = NULL;
     }
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ret;
 }
 
 esp_err_t gpio_controller_unregister_interrupt(gpio_bess_pin_t pin)
 {
     // Validate parameters
     if (!is_valid_pin(pin)) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Take the mutex
     if (xSemaphoreTake(s_gpio_state.mutex, portMAX_DELAY) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_FAIL;
     }
     
     // Remove ISR handler if configured
     if (s_gpio_state.pins[pin].config.interrupt_type != GPIO_INTR_DISABLE) {
         gpio_isr_handler_remove(s_gpio_state.pins[pin].pin_num);
     }
     
     // Clear callback and user data
     s_gpio_state.pins[pin].callback = NULL;
     s_gpio_state.pins[pin].user_data = NULL;
     
     // Release the mutex
     xSemaphoreGive(s_gpio_state.mutex);
     
     return ESP_OK;
 }
 
 esp_err_t gpio_controller_enable_interrupt(gpio_bess_pin_t pin)
 {
     // Validate parameters
     if (!is_valid_pin(pin)) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if interrupt is configured
     if (s_gpio_state.pins[pin].config.interrupt_type == GPIO_INTR_DISABLE ||
         s_gpio_state.pins[pin].callback == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Enable the interrupt
     return gpio_intr_enable(s_gpio_state.pins[pin].pin_num);
 }
 
 esp_err_t gpio_controller_disable_interrupt(gpio_bess_pin_t pin)
 {
     // Validate parameters
     if (!is_valid_pin(pin)) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if initialized
     if (!s_gpio_state.initialized || !s_gpio_state.pins[pin].initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if interrupt is configured
     if (s_gpio_state.pins[pin].config.interrupt_type == GPIO_INTR_DISABLE) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Disable the interrupt
     return gpio_intr_disable(s_gpio_state.pins[pin].pin_num);
 }