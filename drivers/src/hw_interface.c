/**
 * @file hw_interface.c
 * @brief Implementation of hardware interfaces for BESS 100KW/200KWH system
 *
 * This file implements interfaces for communicating with hardware components of the
 * Battery Energy Storage System as defined in hw_interface.h.
 * 
 * Hardware: ESP32-P4 MCU
 * RTOS: FreeRTOS
 */

 #include "hw_interface.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "esp_system.h"
 #include "driver/gpio.h"
 #include "driver/uart.h"
 #include "driver/twai.h"  // ESP-IDF CAN driver
 #include "driver/i2c.h"
 #include "driver/spi_master.h"
 #include "esp_vfs_fat.h"
 #include "sdmmc_cmd.h"
 #include "esp_netif.h"
 #include "esp_wifi.h"
 #include "esp_eth.h"
 #include "esp_http_client.h"
 #include "esp_sntp.h"
 #include "nvs_flash.h"
 #include "freertos/event_groups.h"
 #include <string.h>
 #include "esp_timer.h"
 
 #define TAG "HW_INTERFACE"
 
 // Forward declarations for event handlers
 static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
 static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
 static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
 
 /*
  * Battery Module Interface Implementation
  */
 
 // Static variables for battery hardware
 static uint8_t s_module_count = 0;
 static SemaphoreHandle_t s_battery_mutex = NULL;
 static bool s_emergency_stop_active = false;
 
 esp_err_t battery_hw_init(uint8_t module_count) {
     if (module_count > MAX_BATTERY_MODULES) {
         ESP_LOGE(TAG, "Module count %d exceeds maximum supported (%d)", 
                  module_count, MAX_BATTERY_MODULES);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for thread safety
     s_battery_mutex = xSemaphoreCreateMutex();
     if (s_battery_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create battery mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Store module count
     s_module_count = module_count;
     
     // Configure GPIO pins for communication with battery modules
     // This implementation assumes a dedicated communication bus (SPI, I2C, or custom)
     // to each battery module, actual implementation would depend on hardware design
     
     // Example initialization (replace with actual hardware-specific code)
     gpio_config_t io_conf = {
         .pin_bit_mask = (1ULL << CONFIG_BESS_BAT_COMM_ENABLE_PIN),
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type = GPIO_INTR_DISABLE
     };
     gpio_config(&io_conf);
     
     // Configure emergency stop pin
     io_conf.pin_bit_mask = (1ULL << CONFIG_BESS_BAT_EMSTOP_PIN);
     gpio_config(&io_conf);
     
     // Default emergency stop to inactive
     gpio_set_level(CONFIG_BESS_BAT_EMSTOP_PIN, 0);
     s_emergency_stop_active = false;
     
     ESP_LOGI(TAG, "Battery hardware interface initialized with %d modules", module_count);
     return ESP_OK;
 }
 
 esp_err_t battery_hw_read_module(uint8_t module_id, bess_battery_data_t *data) {
     if (module_id >= s_module_count || data == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_battery_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take battery mutex for module %d", module_id);
         return ESP_ERR_TIMEOUT;
     }
     
     // Clear data structure
     memset(data, 0, sizeof(bess_battery_data_t));
     data->module_id = module_id;
     data->timestamp_ms = esp_timer_get_time() / 1000; // Convert µs to ms
     
     // This implementation would typically involve:
     // 1. Selecting the appropriate module via multiplexer/GPIO
     // 2. Sending command(s) to request data
     // 3. Reading response(s) with appropriate timing
     // 4. Parsing response data into the data structure
     
     // Example communication (replace with actual protocol)
     // For this example, we'll simulate a module with random data
     #ifdef CONFIG_BESS_SIMULATION_MODE
         // Simulate module data for testing
         data->module_voltage = 48.0f + ((float)esp_random() / UINT32_MAX - 0.5f) * 2.0f;
         data->module_current = ((float)esp_random() / UINT32_MAX - 0.5f) * 20.0f;
         data->active_cells = MAX_CELLS_PER_MODULE;
         data->active_temp_sensors = MAX_TEMP_SENSORS_PER_MODULE;
         
         // Generate cell voltages (around 3.2V per cell for LFP)
         for (int i = 0; i < data->active_cells; i++) {
             data->cell_voltages[i] = 3.2f + ((float)esp_random() / UINT32_MAX - 0.5f) * 0.2f;
         }
         
         // Generate temperature readings (around 25°C)
         for (int i = 0; i < data->active_temp_sensors; i++) {
             data->temperatures[i] = 25.0f + ((float)esp_random() / UINT32_MAX - 0.5f) * 5.0f;
         }
         
         // Randomly set alarms (low probability)
         data->overvoltage_alarm = (esp_random() % 100 < 2);
         data->undervoltage_alarm = (esp_random() % 100 < 2);
         data->overcurrent_alarm = (esp_random() % 100 < 2);
         data->overtemperature_alarm = (esp_random() % 100 < 2);
         data->comm_error = false;
     #else
         // Actual hardware communication would go here
         // Example SPI communication (pseudocode, actual implementation would vary)
         
         // Select module
         gpio_set_level(CONFIG_BESS_BAT_COMM_ENABLE_PIN, 1);
         
         // Send command to request all data
         uint8_t cmd_buffer[4] = {0x01, 0x02, 0x03, module_id};
         uint8_t resp_buffer[128] = {0};
         
         // SPI transaction would happen here
         // spi_device_transmit(...);
         
         // Parse response data
         // This is highly dependent on the actual protocol and hardware
         
         // For now, mark as communication error
         data->comm_error = true;
     #endif
     
     xSemaphoreGive(s_battery_mutex);
     return ESP_OK;
 }
 
 esp_err_t battery_hw_set_balancing(uint8_t module_id, uint32_t balance_mask) {
     if (module_id >= s_module_count) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (s_emergency_stop_active) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_battery_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take battery mutex for balancing module %d", module_id);
         return ESP_ERR_TIMEOUT;
     }
     
     // This implementation would send commands to the battery module to
     // activate cell balancing according to the balance_mask bitmap
     
     // Example implementation (replace with actual protocol)
     #ifdef CONFIG_BESS_SIMULATION_MODE
         // In simulation mode, just log the balancing request
         ESP_LOGI(TAG, "Module %d balance mask set to 0x%08lx", module_id, balance_mask);
     #else
         // Actual hardware implementation
         // Example SPI command to set balancing (pseudocode)
         uint8_t cmd_buffer[8] = {
             0x05, 0x06, module_id,
             (uint8_t)(balance_mask & 0xFF),
             (uint8_t)((balance_mask >> 8) & 0xFF),
             (uint8_t)((balance_mask >> 16) & 0xFF),
             (uint8_t)((balance_mask >> 24) & 0xFF),
             0
         };
         
         // Calculate checksum
         for (int i = 0; i < 7; i++) {
             cmd_buffer[7] ^= cmd_buffer[i];
         }
         
         // SPI transaction would happen here
         // spi_device_transmit(...);
     #endif
     
     xSemaphoreGive(s_battery_mutex);
     return ESP_OK;
 }
 
 esp_err_t battery_hw_set_emergency_stop(bool emergency_stop) {
     if (xSemaphoreTake(s_battery_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take battery mutex for emergency stop");
         return ESP_ERR_TIMEOUT;
     }
     
     // Set emergency stop state
     s_emergency_stop_active = emergency_stop;
     
     // Set the emergency stop GPIO pin
     gpio_set_level(CONFIG_BESS_BAT_EMSTOP_PIN, emergency_stop ? 1 : 0);
     
     if (emergency_stop) {
         ESP_LOGW(TAG, "Emergency stop activated");
     } else {
         ESP_LOGI(TAG, "Emergency stop deactivated");
     }
     
     xSemaphoreGive(s_battery_mutex);
     return ESP_OK;
 }
 
 /*
  * Thermal Management Interface Implementation
  */
 
 static SemaphoreHandle_t s_thermal_mutex = NULL;
 static bess_cooling_mode_t s_current_cooling_mode = COOLING_MODE_OFF;
 static uint8_t s_current_cooling_power = 0;
 
 esp_err_t thermal_hw_init(void) {
     // Create mutex for thread safety
     s_thermal_mutex = xSemaphoreCreateMutex();
     if (s_thermal_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create thermal mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Configure cooling control pins
     // This implementation assumes PWM control for fans or pumps
     gpio_config_t io_conf = {
         .pin_bit_mask = (1ULL << CONFIG_BESS_COOLING_CONTROL_PIN),
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type = GPIO_INTR_DISABLE
     };
     gpio_config(&io_conf);
     
     // Initialize PWM for cooling control
     // ledc_timer_config_t timer_conf = {...};
     // ledc_channel_config_t channel_conf = {...};
     // ledc_timer_config(&timer_conf);
     // ledc_channel_config(&channel_conf);
     
     // Initialize with cooling off
     thermal_hw_set_cooling_mode(COOLING_MODE_OFF);
     
     ESP_LOGI(TAG, "Thermal hardware interface initialized");
     return ESP_OK;
 }
 
 esp_err_t thermal_hw_set_cooling_mode(bess_cooling_mode_t mode) {
     if (mode > COOLING_MODE_MAX) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_thermal_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take thermal mutex for setting cooling mode");
         return ESP_ERR_TIMEOUT;
     }
     
     // Map cooling mode to power percentage
     uint8_t power = 0;
     switch (mode) {
         case COOLING_MODE_OFF:
             power = 0;
             break;
         case COOLING_MODE_PASSIVE:
             power = 0;  // No active cooling
             break;
         case COOLING_MODE_LOW:
             power = 30;
             break;
         case COOLING_MODE_MEDIUM:
             power = 60;
             break;
         case COOLING_MODE_HIGH:
             power = 85;
             break;
         case COOLING_MODE_MAX:
             power = 100;
             break;
     }
     
     // Set cooling power
     thermal_hw_set_cooling_power(power);
     
     // Update current mode
     s_current_cooling_mode = mode;
     
     ESP_LOGI(TAG, "Cooling mode set to %d, power %d%%", mode, power);
     
     xSemaphoreGive(s_thermal_mutex);
     return ESP_OK;
 }
 
 esp_err_t thermal_hw_set_cooling_power(uint8_t power_percent) {
     if (power_percent > 100) {
         power_percent = 100;
     }
     
     if (xSemaphoreTake(s_thermal_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take thermal mutex for setting cooling power");
         return ESP_ERR_TIMEOUT;
     }
     
     // Store current power level
     s_current_cooling_power = power_percent;
     
     // Set PWM duty cycle for cooling fans/pumps
     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 
     //              (power_percent * 8191) / 100);  // 8191 = 2^13 - 1
     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
     
     // For simulation, just log the power level
     ESP_LOGI(TAG, "Cooling power set to %d%%", power_percent);
     
     xSemaphoreGive(s_thermal_mutex);
     return ESP_OK;
 }
 
 esp_err_t thermal_hw_read_ambient_temp(float *temperature) {
     if (temperature == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_thermal_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take thermal mutex for reading ambient temperature");
         return ESP_ERR_TIMEOUT;
     }
     
     // Read ambient temperature from sensor
     // This implementation would typically use I2C or OneWire to communicate with
     // a temperature sensor (e.g., DS18B20, BME280, etc.)
     
     // Example implementation (replace with actual protocol)
     #ifdef CONFIG_BESS_SIMULATION_MODE
         // Simulate ambient temperature around 25°C
         *temperature = 25.0f + ((float)esp_random() / UINT32_MAX - 0.5f) * 5.0f;
     #else
         // Actual hardware implementation would go here
         // Example I2C read from temperature sensor (pseudocode)
         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
         i2c_master_start(cmd);
         i2c_master_write_byte(cmd, (CONFIG_BESS_TEMP_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
         i2c_master_write_byte(cmd, CONFIG_BESS_TEMP_SENSOR_REG, true);
         i2c_master_stop(cmd);
         i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
         i2c_cmd_link_delete(cmd);
         
         cmd = i2c_cmd_link_create();
         i2c_master_start(cmd);
         i2c_master_write_byte(cmd, (CONFIG_BESS_TEMP_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
         uint8_t data[2];
         i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
         i2c_master_stop(cmd);
         i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
         i2c_cmd_link_delete(cmd);
         
         // Convert raw data to temperature
         // This conversion depends on the specific sensor used
         int16_t raw_temp = (data[0] << 8) | data[1];
         *temperature = (float)raw_temp * 0.0625f;  // For example, for a MCP9808
     #endif
     
     xSemaphoreGive(s_thermal_mutex);
     return ESP_OK;
 }
 
 /*
  * Modbus RTU Interface Implementation
  */
 
 static SemaphoreHandle_t s_modbus_rtu_mutex = NULL;
 static uart_port_t s_modbus_uart_port = UART_NUM_MAX;
 
 esp_err_t modbus_rtu_init(const modbus_rtu_config_t *config) {
     if (config == NULL || config->uart_num >= UART_NUM_MAX) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for thread safety
     s_modbus_rtu_mutex = xSemaphoreCreateMutex();
     if (s_modbus_rtu_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create Modbus RTU mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Store UART port number
     s_modbus_uart_port = config->uart_num;
     
     // Configure UART for Modbus RTU
     uart_config_t uart_config = {
         .baud_rate = config->baud_rate,
         .data_bits = config->data_bits,
         .parity = (config->parity == 'E') ? UART_PARITY_EVEN : 
                  (config->parity == 'O') ? UART_PARITY_ODD : UART_PARITY_DISABLE,
         .stop_bits = config->stop_bits,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .rx_flow_ctrl_thresh = 0,
         .source_clk = UART_SCLK_APB,
     };
     
     ESP_ERROR_CHECK(uart_param_config(config->uart_num, &uart_config));
     ESP_ERROR_CHECK(uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin, 
                                  config->rts_pin, UART_PIN_NO_CHANGE));
     ESP_ERROR_CHECK(uart_driver_install(config->uart_num, 256, 256, 0, NULL, 0));
     
     // RS485 half-duplex mode if RTS pin is configured
     if (config->rts_pin >= 0) {
         ESP_ERROR_CHECK(uart_set_mode(config->uart_num, UART_MODE_RS485_HALF_DUPLEX));
     }
     
     ESP_LOGI(TAG, "Modbus RTU initialized on UART%d at %d baud", 
              config->uart_num, config->baud_rate);
     return ESP_OK;
 }
 
 esp_err_t modbus_rtu_process(uint32_t timeout_ms) {
     if (s_modbus_uart_port == UART_NUM_MAX) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_modbus_rtu_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take Modbus RTU mutex for processing");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if data is available
     size_t available_bytes;
     ESP_ERROR_CHECK(uart_get_buffered_data_len(s_modbus_uart_port, &available_bytes));
     
     if (available_bytes > 0) {
         // Read available data
         uint8_t buffer[256];
         int len = uart_read_bytes(s_modbus_uart_port, buffer, 
                                   available_bytes > sizeof(buffer) ? sizeof(buffer) : available_bytes,
                                   pdMS_TO_TICKS(timeout_ms));
         
         if (len > 0) {
             // Process Modbus RTU frame
             // This would involve parsing the frame, executing the function code,
             // and sending a response
             
             // Example implementation for function code 0x03 (Read Holding Registers)
             // if (len >= 8 && buffer[1] == 0x03) {
             //     uint8_t addr = buffer[0];
             //     uint16_t start_addr = (buffer[2] << 8) | buffer[3];
             //     uint16_t reg_count = (buffer[4] << 8) | buffer[5];
             //     
             //     // Prepare response (simplified example)
             //     uint8_t response[256];
             //     response[0] = addr;
             //     response[1] = 0x03;
             //     response[2] = reg_count * 2;  // Byte count
             //     
             //     // Fill with register values (example)
             //     for (int i = 0; i < reg_count; i++) {
             //         uint16_t value = get_modbus_register(start_addr + i);
             //         response[3 + i*2] = (value >> 8) & 0xFF;
             //         response[4 + i*2] = value & 0xFF;
             //     }
             //     
             //     // Calculate CRC
             //     uint16_t crc = calculate_crc16(response, 3 + reg_count * 2);
             //     response[3 + reg_count * 2] = crc & 0xFF;
             //     response[4 + reg_count * 2] = (crc >> 8) & 0xFF;
             //     
             //     // Send response
             //     uart_write_bytes(s_modbus_uart_port, response, 5 + reg_count * 2);
             // }
             
             ESP_LOGI(TAG, "Processed Modbus RTU frame, %d bytes", len);
         }
     }
     
     xSemaphoreGive(s_modbus_rtu_mutex);
     return ESP_OK;
 }
 
 /*
  * Modbus TCP Interface Implementation
  */
 
 static SemaphoreHandle_t s_modbus_tcp_mutex = NULL;
 static bool s_modbus_tcp_running = false;
 static modbus_tcp_config_t s_modbus_tcp_config;
 
 esp_err_t modbus_tcp_init(const modbus_tcp_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for thread safety
     s_modbus_tcp_mutex = xSemaphoreCreateMutex();
     if (s_modbus_tcp_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create Modbus TCP mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Store configuration
     memcpy(&s_modbus_tcp_config, config, sizeof(modbus_tcp_config_t));
     
     ESP_LOGI(TAG, "Modbus TCP initialized on port %d, slave address %d", 
              config->port, config->slave_address);
     return ESP_OK;
 }
 
 esp_err_t modbus_tcp_start(void) {
     if (xSemaphoreTake(s_modbus_tcp_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take Modbus TCP mutex for starting server");
         return ESP_ERR_TIMEOUT;
     }
     
     if (s_modbus_tcp_running) {
         xSemaphoreGive(s_modbus_tcp_mutex);
         return ESP_OK;  // Already running
     }
     
     // This implementation would typically:
     // 1. Create a TCP server socket
     // 2. Bind to the configured port
     // 3. Listen for connections
     // 4. Create a task to accept connections and handle Modbus TCP frames
     
     // Example implementation (just the outline, not full implementation)
     // xTaskCreate(modbus_tcp_server_task, "modbus_tcp", 4096, NULL, 5, NULL);
     
     s_modbus_tcp_running = true;
     ESP_LOGI(TAG, "Modbus TCP server started on port %d", s_modbus_tcp_config.port);
     
     xSemaphoreGive(s_modbus_tcp_mutex);
     return ESP_OK;
 }
 
 esp_err_t modbus_tcp_stop(void) {
     if (xSemaphoreTake(s_modbus_tcp_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take Modbus TCP mutex for stopping server");
         return ESP_ERR_TIMEOUT;
     }
     
     if (!s_modbus_tcp_running) {
         xSemaphoreGive(s_modbus_tcp_mutex);
         return ESP_OK;  // Already stopped
     }
     
     // This implementation would typically:
     // 1. Signal the server task to stop (through a flag or event group)
     // 2. Close all open connections
     // 3. Close the server socket
     
     s_modbus_tcp_running = false;
     ESP_LOGI(TAG, "Modbus TCP server stopped");
     
     xSemaphoreGive(s_modbus_tcp_mutex);
     return ESP_OK;
 }
 
 /*
  * CANBus Interface Implementation
  */
 
 static SemaphoreHandle_t s_canbus_mutex = NULL;
 static bool s_canbus_initialized = false;
 
 typedef struct {
     uint32_t id;
     void (*callback)(canbus_message_t *message, void *user_data);
     void *user_data;
 } canbus_callback_t;
 
 #define MAX_CAN_CALLBACKS 16
 static canbus_callback_t s_can_callbacks[MAX_CAN_CALLBACKS] = {0};
 static int s_can_callback_count = 0;
 
 esp_err_t canbus_init(const canbus_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for thread safety
     s_canbus_mutex = xSemaphoreCreateMutex();
     if (s_canbus_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create CANBus mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Configure CAN controller (ESP32's TWAI driver)
     twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(config->tx_pin, config->rx_pin, TWAI_MODE_NORMAL);
     twai_timing_config_t t_config;
     
     // Select timing configuration based on baud rate
     switch (config->baud_rate) {
         case 1000000:
             t_config = TWAI_TIMING_CONFIG_1MBITS();
             break;
         case 800000:
             t_config = TWAI_TIMING_CONFIG_800KBITS();
             break;
         case 500000:
             t_config = TWAI_TIMING_CONFIG_500KBITS();
             break;
         case 250000:
             t_config = TWAI_TIMING_CONFIG_250KBITS();
             break;
         case 125000:
             t_config = TWAI_TIMING_CONFIG_125KBITS();
             break;
         case 100000:
             t_config = TWAI_TIMING_CONFIG_100KBITS();
             break;
         default:
             ESP_LOGE(TAG, "Unsupported CAN baud rate: %lu", config->baud_rate);
             xSemaphoreGive(s_canbus_mutex);
             return ESP_ERR_INVALID_ARG;
     }
     
     // Filter configuration
     twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
     if (!config->accept_all_frames) {
         // Custom filter configuration would go here
         // This depends on the specific IDs that the system needs to accept
     }
     
     // Install and start CAN driver
     ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
     ESP_ERROR_CHECK(twai_start());
     
     // Reset callback array
     memset(s_can_callbacks, 0, sizeof(s_can_callbacks));
     s_can_callback_count = 0;
     
     s_canbus_initialized = true;
     ESP_LOGI(TAG, "CANBus initialized at %lu baud", config->baud_rate);
     
     // Create task for receiving CAN messages and dispatching callbacks
     // xTaskCreate(can_rx_task, "can_rx", 2048, NULL, 10, NULL);
     
     return ESP_OK;
 }
 
 esp_err_t canbus_send_message(const canbus_message_t *message) {
     if (message == NULL || !s_canbus_initialized) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_canbus_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take CANBus mutex for sending message");
         return ESP_ERR_TIMEOUT;
     }
     
     // Convert from our message structure to TWAI message structure
     twai_message_t twai_msg = {
         .identifier = message->identifier,
         .extd = message->extended_frame ? 1 : 0,
         .rtr = message->remote_frame ? 1 : 0,
         .data_length_code = message->data_length,
     };
     
     // Copy data
     for (int i = 0; i < message->data_length; i++) {
         twai_msg.data[i] = message->data[i];
     }
     
     // Send message
     esp_err_t ret = twai_transmit(&twai_msg, pdMS_TO_TICKS(100));
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to send CAN message, error: %d", ret);
     }
     
     xSemaphoreGive(s_canbus_mutex);
     return ret;
 }
 
 esp_err_t canbus_receive_message(canbus_message_t *message, uint32_t timeout_ms) {
     if (message == NULL || !s_canbus_initialized) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // No need to take mutex for receiving, as TWAI driver handles this internally
     
     // Receive message using TWAI driver
     twai_message_t twai_msg;
     esp_err_t ret = twai_receive(&twai_msg, pdMS_TO_TICKS(timeout_ms));
     
     if (ret == ESP_OK) {
         // Convert from TWAI message to our message structure
         message->identifier = twai_msg.identifier;
         message->extended_frame = twai_msg.extd ? true : false;
         message->remote_frame = twai_msg.rtr ? true : false;
         message->data_length = twai_msg.data_length_code;
         
         // Copy data
         for (int i = 0; i < twai_msg.data_length_code; i++) {
             message->data[i] = twai_msg.data[i];
         }
     } else if (ret == ESP_ERR_TIMEOUT) {
         // Timeout is a normal condition, not an error worth logging
         return ESP_ERR_TIMEOUT;
     } else {
         ESP_LOGW(TAG, "Failed to receive CAN message, error: %d", ret);
     }
     
     return ret;
 }
 
 /*
  * SD Card Interface Implementation
  */
 
 static SemaphoreHandle_t s_sd_card_mutex = NULL;
 static bool s_sd_card_mounted = false;
 static char s_sd_mount_point[64] = {0};
 static sdmmc_card_t *s_sd_card = NULL;
 
 esp_err_t sd_card_init(const char *mount_point, size_t max_files) {
     if (mount_point == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for thread safety
     s_sd_card_mutex = xSemaphoreCreateMutex();
     if (s_sd_card_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create SD card mutex");
         return ESP_ERR_NO_MEM;
     }
     
     if (xSemaphoreTake(s_sd_card_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take SD card mutex for initialization");
         return ESP_ERR_TIMEOUT;
     }
     
     // Store mount point
     strlcpy(s_sd_mount_point, mount_point, sizeof(s_sd_mount_point));
     
     // Configure SPI for SD card
     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
     host.max_freq_khz = SDMMC_FREQ_DEFAULT;  // Use default frequency
     
     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
     slot_config.gpio_cs = CONFIG_BESS_SD_CS_PIN;
     slot_config.host_id = host.slot;
     
     // Mount options for FAT filesystem
     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
         .format_if_mount_failed = false,
         .max_files = max_files,
         .allocation_unit_size = 16 * 1024  // 16K cluster size
     };
     
     // Try to mount the filesystem
     ESP_LOGI(TAG, "Mounting SD card on %s", mount_point);
     esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &s_sd_card);
     
     if (ret != ESP_OK) {
         if (ret == ESP_FAIL) {
             ESP_LOGE(TAG, "Failed to mount FAT filesystem on SD card");
         } else {
             ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
         }
         s_sd_card_mounted = false;
     } else {
         s_sd_card_mounted = true;
         ESP_LOGI(TAG, "SD card mounted successfully at %s", mount_point);
         
         // Print card info
         sdmmc_card_print_info(stdout, s_sd_card);
     }
     
     xSemaphoreGive(s_sd_card_mutex);
     return ret;
 }
 
 bool sd_card_is_mounted(void) {
     bool mounted = false;
     
     if (xSemaphoreTake(s_sd_card_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         mounted = s_sd_card_mounted;
         xSemaphoreGive(s_sd_card_mutex);
     }
     
     return mounted;
 }
 
 esp_err_t sd_card_get_free_space(uint64_t *bytes_free) {
     if (bytes_free == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_sd_card_mounted) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_sd_card_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take SD card mutex for getting free space");
         return ESP_ERR_TIMEOUT;
     }
     
     FATFS *fs;
     DWORD free_clusters;
     
     // Get volume information
     FRESULT res = f_getfree(s_sd_mount_point, &free_clusters, &fs);
     if (res != FR_OK) {
         ESP_LOGE(TAG, "Failed to get free space: %d", res);
         xSemaphoreGive(s_sd_card_mutex);
         return ESP_FAIL;
     }
     
     // Calculate free space in bytes
     *bytes_free = (uint64_t)free_clusters * (uint64_t)fs->csize * (uint64_t)512;
     
     xSemaphoreGive(s_sd_card_mutex);
     return ESP_OK;
 }
 
 /*
  * RTC Interface Implementation
  */
 
 static SemaphoreHandle_t s_rtc_mutex = NULL;
 static bool s_rtc_initialized = false;
 
 esp_err_t rtc_init(void) {
     // Create mutex for thread safety
     s_rtc_mutex = xSemaphoreCreateMutex();
     if (s_rtc_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create RTC mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize I2C for external RTC if used
     #ifdef CONFIG_BESS_USE_EXTERNAL_RTC
         // Configure I2C
         i2c_config_t i2c_config = {
             .mode = I2C_MODE_MASTER,
             .sda_io_num = CONFIG_BESS_RTC_SDA_PIN,
             .scl_io_num = CONFIG_BESS_RTC_SCL_PIN,
             .sda_pullup_en = GPIO_PULLUP_ENABLE,
             .scl_pullup_en = GPIO_PULLUP_ENABLE,
             .master.clk_speed = 100000  // 100 KHz
         };
         
         ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_config));
         ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));
     #endif
     
     // Initialize SNTP for automatic time synchronization when network is available
     sntp_setoperatingmode(SNTP_OPMODE_POLL);
     sntp_setservername(0, "pool.ntp.org");
     sntp_init();
     
     s_rtc_initialized = true;
     ESP_LOGI(TAG, "RTC interface initialized");
     
     return ESP_OK;
 }
 
 esp_err_t rtc_get_time(rtc_time_t *time) {
     if (time == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_rtc_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_rtc_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take RTC mutex for getting time");
         return ESP_ERR_TIMEOUT;
     }
     
     #ifdef CONFIG_BESS_USE_EXTERNAL_RTC
         // Read time from external RTC chip
         // This implementation would depend on the specific RTC chip used
         // Example for DS3231:
         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
         i2c_master_start(cmd);
         i2c_master_write_byte(cmd, (CONFIG_BESS_RTC_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
         i2c_master_write_byte(cmd, 0x00, true);  // Start at register 0
         i2c_master_stop(cmd);
         ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(100)));
         i2c_cmd_link_delete(cmd);
         
         uint8_t data[7];
         cmd = i2c_cmd_link_create();
         i2c_master_start(cmd);
         i2c_master_write_byte(cmd, (CONFIG_BESS_RTC_I2C_ADDR << 1) | I2C_MASTER_READ, true);
         i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
         i2c_master_stop(cmd);
         ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(100)));
         i2c_cmd_link_delete(cmd);
         
         // Convert BCD values to binary (example for DS3231)
         time->second = ((data[0] >> 4) * 10) + (data[0] & 0x0F);
         time->minute = ((data[1] >> 4) * 10) + (data[1] & 0x0F);
         time->hour = ((data[2] >> 4) * 10) + (data[2] & 0x0F);
         time->day = ((data[4] >> 4) * 10) + (data[4] & 0x0F);
         time->month = ((data[5] >> 4) * 10) + (data[5] & 0x0F);
         time->year = 2000 + ((data[6] >> 4) * 10) + (data[6] & 0x0F);
         time->ms = 0;  // Most RTCs don't track milliseconds
     #else
         // Use ESP32 internal RTC
         struct tm timeinfo;
         time_t now;
         time(&now);
         localtime_r(&now, &timeinfo);
         
         time->year = timeinfo.tm_year + 1900;
         time->month = timeinfo.tm_mon + 1;
         time->day = timeinfo.tm_mday;
         time->hour = timeinfo.tm_hour;
         time->minute = timeinfo.tm_min;
         time->second = timeinfo.tm_sec;
         
         // Get milliseconds from high-resolution timer
         uint64_t us = esp_timer_get_time();
         time->ms = (us / 1000) % 1000;
     #endif
     
     xSemaphoreGive(s_rtc_mutex);
     return ESP_OK;
 }
 
 esp_err_t rtc_set_time(const rtc_time_t *time) {
     if (time == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_rtc_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate time values
     if (time->year < 2000 || time->year > 2099 ||
         time->month < 1 || time->month > 12 ||
         time->day < 1 || time->day > 31 ||
         time->hour > 23 || time->minute > 59 || time->second > 59 || time->ms > 999) {
         ESP_LOGE(TAG, "Invalid time values");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_rtc_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take RTC mutex for setting time");
         return ESP_ERR_TIMEOUT;
     }
     
     #ifdef CONFIG_BESS_USE_EXTERNAL_RTC
         // Set time in external RTC chip
         // Example for DS3231:
         uint8_t data[7];
         // Convert to BCD
         data[0] = ((time->second / 10) << 4) | (time->second % 10);  // Seconds
         data[1] = ((time->minute / 10) << 4) | (time->minute % 10);  // Minutes
         data[2] = ((time->hour / 10) << 4) | (time->hour % 10);      // Hours
         data[3] = 1;  // Day of week (not used)
         data[4] = ((time->day / 10) << 4) | (time->day % 10);        // Day
         data[5] = ((time->month / 10) << 4) | (time->month % 10);    // Month
         data[6] = (((time->year - 2000) / 10) << 4) | ((time->year - 2000) % 10);  // Year
         
         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
         i2c_master_start(cmd);
         i2c_master_write_byte(cmd, (CONFIG_BESS_RTC_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
         i2c_master_write_byte(cmd, 0x00, true);  // Start at register 0
         i2c_master_write(cmd, data, 7, true);
         i2c_master_stop(cmd);
         ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(100)));
         i2c_cmd_link_delete(cmd);
     #endif
     
     // Set system time
     struct tm timeinfo = {0};
     timeinfo.tm_year = time->year - 1900;
     timeinfo.tm_mon = time->month - 1;
     timeinfo.tm_mday = time->day;
     timeinfo.tm_hour = time->hour;
     timeinfo.tm_min = time->minute;
     timeinfo.tm_sec = time->second;
     
     time_t t = mktime(&timeinfo);
     struct timeval tv = {.tv_sec = t, .tv_usec = time->ms * 1000};
     settimeofday(&tv, NULL);
     
     ESP_LOGI(TAG, "System time set to: %04d-%02d-%02d %02d:%02d:%02d.%03d",
              time->year, time->month, time->day,
              time->hour, time->minute, time->second, time->ms);
     
     xSemaphoreGive(s_rtc_mutex);
     return ESP_OK;
 }
 
 esp_err_t rtc_get_timestamp(uint32_t *timestamp) {
     if (timestamp == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_rtc_initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     time_t now;
     time(&now);
     *timestamp = (uint32_t)now;
     
     return ESP_OK;
 }
 
 /*
  * System Health Monitoring Implementation
  */
 
 static SemaphoreHandle_t s_health_mutex = NULL;
 static uint32_t s_system_start_time = 0;
 
 esp_err_t system_health_init(void) {
     // Create mutex for thread safety
     s_health_mutex = xSemaphoreCreateMutex();
     if (s_health_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create system health mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Record system start time
     time_t now;
     time(&now);
     s_system_start_time = (uint32_t)now;
     
     ESP_LOGI(TAG, "System health monitoring initialized");
     return ESP_OK;
 }
 
 esp_err_t system_health_get_data(system_health_t *health) {
     if (health == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take system health mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Get MCU temperature (depends on ESP32 variant)
     #ifdef CONFIG_IDF_TARGET_ESP32
         // ESP32 temperature sensor
         health->mcu_temperature = temprature_sens_read();  // Intentional typo in ESP-IDF API
     #else
         // For other ESP32 variants or if sensor not available
         health->mcu_temperature = 0.0f;
     #endif
     
     // Get system uptime
     time_t now;
     time(&now);
     health->uptime_seconds = (uint32_t)now - s_system_start_time;
     
     // Get heap statistics
     health->free_heap = esp_get_free_heap_size();
     health->min_free_heap = esp_get_minimum_free_heap_size();
     
     // Get CPU usage (simplified version, actual implementation would be more complex)
     // This is a placeholder - actual CPU usage measurement requires periodic sampling
     health->cpu_usage = 0;  // Not implemented
     
     // Get voltage and current (would depend on external sensors)
     health->input_voltage = 0.0f;  // Not implemented
     health->system_current = 0.0f;  // Not implemented
     
     xSemaphoreGive(s_health_mutex);
     return ESP_OK;
 }
 
 /*
  * Network Interface Implementation
  */
 
 static EventGroupHandle_t s_network_event_group = NULL;
 static network_status_t s_network_status = NETWORK_STATUS_DISCONNECTED;
 
 #define WIFI_CONNECTED_BIT      BIT0
 #define WIFI_FAIL_BIT           BIT1
 #define ETH_CONNECTED_BIT       BIT2
 #define ETH_DISCONNECTED_BIT    BIT3
 #define INTERNET_CONNECTED_BIT  BIT4
 
 esp_err_t wifi_hw_init(const wifi_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Initialize network event group if not already done
     if (s_network_event_group == NULL) {
         s_network_event_group = xEventGroupCreate();
         if (s_network_event_group == NULL) {
             ESP_LOGE(TAG, "Failed to create network event group");
             return ESP_ERR_NO_MEM;
         }
     }
     
     // Initialize NVS for WiFi storage
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     
     // Initialize TCP/IP stack
     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     
     // Create default WiFi station netif
     esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
     assert(sta_netif);
     
     // Configure static IP if needed
     if (config->static_ip) {
         esp_netif_dhcpc_stop(sta_netif);
         
         esp_netif_ip_info_t ip_info;
         memset(&ip_info, 0, sizeof(ip_info));
         
         ip_info.ip.addr = ipaddr_addr(config->ip_address);
         ip_info.netmask.addr = ipaddr_addr(config->subnet_mask);
         ip_info.gw.addr = ipaddr_addr(config->gateway);
         
         ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
         
         // Configure DNS
         ip_addr_t dns_server;
         dns_server.type = IPADDR_TYPE_V4;
         dns_server.u_addr.ip4.addr = ipaddr_addr(config->dns_server);
         ESP_ERROR_CHECK(esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, (esp_netif_dns_info_t *)&dns_server));
     }
     
     // Initialize WiFi
     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
     
     // Register event handlers
     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
     
     // Configure WiFi station
     wifi_config_t wifi_cfg = {0};
     strncpy((char *)wifi_cfg.sta.ssid, config->ssid, sizeof(wifi_cfg.sta.ssid));
     strncpy((char *)wifi_cfg.sta.password, config->password, sizeof(wifi_cfg.sta.password));
     
     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
     ESP_ERROR_CHECK(esp_wifi_start());
     
     s_network_status = NETWORK_STATUS_CONNECTING;
     ESP_LOGI(TAG, "WiFi initialized, connecting to SSID: %s", config->ssid);
     
     return ESP_OK;
 }
 
 esp_err_t ethernet_hw_init(const ethernet_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Initialize network event group if not already done
     if (s_network_event_group == NULL) {
         s_network_event_group = xEventGroupCreate();
         if (s_network_event_group == NULL) {
             ESP_LOGE(TAG, "Failed to create network event group");
             return ESP_ERR_NO_MEM;
         }
     }
     
     // Initialize NVS
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     
     // Initialize TCP/IP stack if not already done
     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     
     // Create default Ethernet netif
     esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
     esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
     
     // Configure static IP if needed
     if (config->static_ip) {
         esp_netif_dhcpc_stop(eth_netif);
         
         esp_netif_ip_info_t ip_info;
         memset(&ip_info, 0, sizeof(ip_info));
         
         ip_info.ip.addr = ipaddr_addr(config->ip_address);
         ip_info.netmask.addr = ipaddr_addr(config->subnet_mask);
         ip_info.gw.addr = ipaddr_addr(config->gateway);
         
         ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));
         
         // Configure DNS
         ip_addr_t dns_server;
         dns_server.type = IPADDR_TYPE_V4;
         dns_server.u_addr.ip4.addr = ipaddr_addr(config->dns_server);
         ESP_ERROR_CHECK(esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_MAIN, (esp_netif_dns_info_t *)&dns_server));
     }
     
     // Register event handlers
     ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler, NULL));
     
     // Initialize MAC and PHY configurations
     eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
     eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
     
     // Set PHY address
     phy_config.phy_addr = CONFIG_BESS_ETH_PHY_ADDR;
     phy_config.reset_gpio_num = CONFIG_BESS_ETH_PHY_RST_GPIO;
     
     // Install Ethernet driver
     esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);
     esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
     
     esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
     esp_eth_handle_t eth_handle = NULL;
     ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
     
     // Set MAC address if needed
     if (!config->use_mac_eeprom) {
         ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, config->mac_address));
     }
     
     // Attach Ethernet driver to TCP/IP stack
     ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
     
     // Start Ethernet driver
     ESP_ERROR_CHECK(esp_eth_start(eth_handle));
     
     s_network_status = NETWORK_STATUS_CONNECTING;
     ESP_LOGI(TAG, "Ethernet initialized");
     
     return ESP_OK;
 }
 
 esp_err_t network_get_status(network_status_t *status) {
     if (status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     *status = s_network_status;
     return ESP_OK;
 }
 
 esp_err_t network_test_connection(const char *host, uint32_t timeout_ms) {
     if (host == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (s_network_status < NETWORK_STATUS_CONNECTED) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create HTTP client configuration
     esp_http_client_config_t config = {
         .url = host,
         .timeout_ms = timeout_ms,
         .method = HTTP_METHOD_HEAD,  // Just request headers, not the full content
     };
     
     // Initialize HTTP client
     esp_http_client_handle_t client = esp_http_client_init(&config);
     if (client == NULL) {
         ESP_LOGE(TAG, "Failed to initialize HTTP client");
         return ESP_FAIL;
     }
     
     // Perform HTTP request
     esp_err_t err = esp_http_client_perform(client);
     if (err == ESP_OK) {
         int status_code = esp_http_client_get_status_code(client);
         ESP_LOGI(TAG, "HTTP request status = %d", status_code);
         
         // Any HTTP response means we have connectivity
         if (status_code >= 200) {
             // Update network status
             s_network_status = NETWORK_STATUS_INTERNET;
             xEventGroupSetBits(s_network_event_group, INTERNET_CONNECTED_BIT);
         }
     } else {
         ESP_LOGW(TAG, "HTTP request failed: %s", esp_err_to_name(err));
     }
     
     // Clean up
     esp_http_client_cleanup(client);
     
     return err;
 }
 
 /*
  * Private functions and event handlers
  */
 
 static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
     if (event_base == WIFI_EVENT) {
         switch (event_id) {
             case WIFI_EVENT_STA_START:
                 ESP_LOGI(TAG, "WiFi started, connecting...");
                 esp_wifi_connect();
                 break;
                 
             case WIFI_EVENT_STA_CONNECTED:
                 ESP_LOGI(TAG, "WiFi connected");
                 s_network_status = NETWORK_STATUS_CONNECTED;
                 break;
                 
             case WIFI_EVENT_STA_DISCONNECTED:
                 ESP_LOGI(TAG, "WiFi disconnected, trying to reconnect...");
                 s_network_status = NETWORK_STATUS_DISCONNECTED;
                 xEventGroupClearBits(s_network_event_group, WIFI_CONNECTED_BIT);
                 xEventGroupClearBits(s_network_event_group, INTERNET_CONNECTED_BIT);
                 esp_wifi_connect();
                 break;
                 
             default:
                 break;
         }
     }
 }
 
 static void eth_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
     if (event_base == ETH_EVENT) {
         switch (event_id) {
             case ETHERNET_EVENT_CONNECTED:
                 ESP_LOGI(TAG, "Ethernet connected");
                 s_network_status = NETWORK_STATUS_CONNECTED;
                 xEventGroupSetBits(s_network_event_group, ETH_CONNECTED_BIT);
                 xEventGroupClearBits(s_network_event_group, ETH_DISCONNECTED_BIT);
                 break;
                 
             case ETHERNET_EVENT_DISCONNECTED:
                 ESP_LOGI(TAG, "Ethernet disconnected");
                 s_network_status = NETWORK_STATUS_DISCONNECTED;
                 xEventGroupClearBits(s_network_event_group, ETH_CONNECTED_BIT);
                 xEventGroupSetBits(s_network_event_group, ETH_DISCONNECTED_BIT);
                 xEventGroupClearBits(s_network_event_group, INTERNET_CONNECTED_BIT);
                 break;
                 
             default:
                 break;
         }
     }
 }
 
 static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data) {
     if (event_base == IP_EVENT) {
         if (event_id == IP_EVENT_STA_GOT_IP) {
             ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
             ESP_LOGI(TAG, "WiFi got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
             s_network_status = NETWORK_STATUS_CONNECTED;
             xEventGroupSetBits(s_network_event_group, WIFI_CONNECTED_BIT);
             xEventGroupClearBits(s_network_event_group, WIFI_FAIL_BIT);
         } else if (event_id == IP_EVENT_ETH_GOT_IP) {
             ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
             ESP_LOGI(TAG, "Ethernet got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
             s_network_status = NETWORK_STATUS_CONNECTED;
         }
     }
 
     if (callback == NULL || !s_canbus_initialized) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_canbus_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take CANBus mutex for registering callback");
         return ESP_ERR_TIMEOUT;
     }
     
     if (s_can_callback_count >= MAX_CAN_CALLBACKS) {
         ESP_LOGW(TAG, "Maximum number of CAN callbacks reached");
         xSemaphoreGive(s_canbus_mutex);
         return ESP_ERR_NO_MEM;
     }
     
     // Add callback to array
     s_can_callbacks[s_can_callback_count].id = id;
     s_can_callbacks[s_can_callback_count].callback = callback;
     s_can_callbacks[s_can_callback_count].user_data = user_data;
     s_can_callback_count++;
     
     ESP_LOGI(TAG, "Registered CAN callback for ID 0x%lx, total callbacks: %d", 
              id, s_can_callback_count);
     
     xSemaphoreGive(s_canbus_mutex);
     return ESP_OK;
 }