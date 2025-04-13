/**
 * @file hw_interface.h
 * @brief Hardware interface definitions for BESS 100KW/200KWH system
 *
 * This file defines interfaces for communicating with hardware components of the
 * Battery Energy Storage System, including LFP battery modules (48V, 16KWH),
 * communication interfaces (Modbus, CANBus), and peripheral devices.
 *
 * Hardware: ESP32-P4 MCU
 * RTOS: FreeRTOS
 */

 #ifndef HW_INTERFACE_H
 #define HW_INTERFACE_H
 
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
  * @brief Battery module hardware interface
  */
 #define MAX_BATTERY_MODULES        16    /**< Maximum number of battery modules supported */
 #define MAX_CELLS_PER_MODULE       16    /**< Maximum number of cells per module */
 #define MAX_TEMP_SENSORS_PER_MODULE 8    /**< Maximum temperature sensors per module */
 
 /**
  * @brief Battery module hardware data structure
  */
 typedef struct {
     uint8_t module_id;                                 /**< Module ID (0-15) */
     float cell_voltages[MAX_CELLS_PER_MODULE];         /**< Cell voltages in V */
     float module_voltage;                              /**< Total module voltage in V */
     float module_current;                              /**< Module current in A, positive=charging, negative=discharging */
     float temperatures[MAX_TEMP_SENSORS_PER_MODULE];   /**< Temperature readings in °C */
     uint8_t active_cells;                              /**< Number of active cells in module */
     uint8_t active_temp_sensors;                       /**< Number of active temperature sensors */
     uint32_t balancing_status;                         /**< Bitmap of cells currently balancing */
     uint32_t timestamp_ms;                             /**< Timestamp of last update in ms */
     bool comm_error;                                   /**< Communication error flag */
     bool overvoltage_alarm;                            /**< Overvoltage alarm flag */
     bool undervoltage_alarm;                           /**< Undervoltage alarm flag */
     bool overcurrent_alarm;                            /**< Overcurrent alarm flag */
     bool overtemperature_alarm;                        /**< Overtemperature alarm flag */
 } bess_battery_data_t;
 
 /**
  * @brief Initialize battery module hardware interface
  *
  * @param module_count Number of battery modules to initialize
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t battery_hw_init(uint8_t module_count);
 
 /**
  * @brief Read data from a battery module
  *
  * @param module_id Module ID to read (0-15)
  * @param data Pointer to store the module data
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t battery_hw_read_module(uint8_t module_id, bess_battery_data_t *data);
 
 /**
  * @brief Set balancing state for a specific module
  *
  * @param module_id Module ID (0-15)
  * @param balance_mask Bitmap indicating which cells to balance
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t battery_hw_set_balancing(uint8_t module_id, uint32_t balance_mask);
 
 /**
  * @brief Set the emergency stop state for battery modules
  *
  * @param emergency_stop true to enable emergency stop, false to disable
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t battery_hw_set_emergency_stop(bool emergency_stop);
 
 /**
  * @brief Thermal management hardware interface
  */
 typedef enum {
     COOLING_MODE_OFF = 0,           /**< Cooling system off */
     COOLING_MODE_PASSIVE = 1,       /**< Passive cooling mode */
     COOLING_MODE_LOW = 2,           /**< Low speed active cooling */
     COOLING_MODE_MEDIUM = 3,        /**< Medium speed active cooling */
     COOLING_MODE_HIGH = 4,          /**< High speed active cooling */
     COOLING_MODE_MAX = 5            /**< Maximum cooling power */
 } bess_cooling_mode_t;
 
 /**
  * @brief Initialize thermal management hardware
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t thermal_hw_init(void);
 
 /**
  * @brief Set cooling system mode
  *
  * @param mode Cooling mode to set
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t thermal_hw_set_cooling_mode(bess_cooling_mode_t mode);
 
 /**
  * @brief Set cooling system power (fan speed or pump speed)
  *
  * @param power_percent Power level in percent (0-100)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t thermal_hw_set_cooling_power(uint8_t power_percent);
 
 /**
  * @brief Read ambient temperature sensor
  *
  * @param temperature Pointer to store temperature in °C
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t thermal_hw_read_ambient_temp(float *temperature);
 
 /**
  * @brief Communication interfaces
  */
 
 /**
  * @brief Modbus RTU configuration
  */
 typedef struct {
     uint32_t baud_rate;        /**< Baud rate (typically 9600, 19200, etc.) */
     uint8_t data_bits;         /**< Data bits (typically 8) */
     uint8_t stop_bits;         /**< Stop bits (1 or 2) */
     char parity;               /**< Parity ('N'=none, 'E'=even, 'O'=odd) */
     uint8_t slave_address;     /**< This device's Modbus slave address */
     uint8_t uart_num;          /**< UART port number */
     int tx_pin;                /**< TX GPIO pin number */
     int rx_pin;                /**< RX GPIO pin number */
     int rts_pin;               /**< RTS GPIO pin number for RS485 direction control */
 } modbus_rtu_config_t;
 
 /**
  * @brief Initialize Modbus RTU interface
  *
  * @param config Pointer to Modbus configuration structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t modbus_rtu_init(const modbus_rtu_config_t *config);
 
 /**
  * @brief Process incoming Modbus RTU messages
  *
  * This function should be called periodically to handle Modbus communication
  *
  * @param timeout_ms Maximum time to wait for incoming messages in ms
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t modbus_rtu_process(uint32_t timeout_ms);
 
 /**
  * @brief Modbus TCP configuration
  */
 typedef struct {
     uint16_t port;             /**< TCP port (typically 502) */
     uint8_t slave_address;     /**< This device's Modbus slave address */
     uint16_t max_connections;  /**< Maximum number of simultaneous connections */
 } modbus_tcp_config_t;
 
 /**
  * @brief Initialize Modbus TCP interface
  *
  * @param config Pointer to Modbus TCP configuration structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t modbus_tcp_init(const modbus_tcp_config_t *config);
 
 /**
  * @brief Start Modbus TCP server
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t modbus_tcp_start(void);
 
 /**
  * @brief Stop Modbus TCP server
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t modbus_tcp_stop(void);
 
 /**
  * @brief CANBus configuration
  */
 typedef struct {
     uint32_t baud_rate;        /**< Baud rate (typically 250000, 500000, etc.) */
     int tx_pin;                /**< TX GPIO pin number */
     int rx_pin;                /**< RX GPIO pin number */
     bool accept_all_frames;    /**< Whether to accept all frames or only specific IDs */
 } canbus_config_t;
 
 /**
  * @brief CAN message structure
  */
 typedef struct {
     uint32_t identifier;        /**< Message identifier (11-bit or 29-bit) */
     bool extended_frame;        /**< True for 29-bit identifier, false for 11-bit */
     bool remote_frame;          /**< True for remote frame, false for data frame */
     uint8_t data_length;        /**< Length of data (0-8 bytes) */
     uint8_t data[8];            /**< Message data bytes */
 } canbus_message_t;
 
 /**
  * @brief Initialize CANBus interface
  *
  * @param config Pointer to CANBus configuration structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t canbus_init(const canbus_config_t *config);
 
 /**
  * @brief Send a message over CANBus
  *
  * @param message Pointer to message structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t canbus_send_message(const canbus_message_t *message);
 
 /**
  * @brief Receive a message from CANBus
  *
  * @param message Pointer to store received message
  * @param timeout_ms Maximum time to wait for message in ms
  * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, other error code otherwise
  */
 esp_err_t canbus_receive_message(canbus_message_t *message, uint32_t timeout_ms);
 
 /**
  * @brief Register a callback function for CANBus message reception
  *
  * @param id Message ID to register for (or 0 for all messages)
  * @param callback Function pointer to call when message is received
  * @param user_data User data to pass to callback
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t canbus_register_callback(uint32_t id, 
                                    void (*callback)(canbus_message_t *message, void *user_data),
                                    void *user_data);
 
 /**
  * @brief Storage interface for SD card
  */
 
 /**
  * @brief Initialize SD card hardware
  *
  * @param mount_point Mount point path (e.g., "/sdcard")
  * @param max_files Maximum number of files to keep open
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t sd_card_init(const char *mount_point, size_t max_files);
 
 /**
  * @brief Check if SD card is present and mounted
  *
  * @return true if SD card is present and mounted, false otherwise
  */
 bool sd_card_is_mounted(void);
 
 /**
  * @brief Get free space on SD card
  *
  * @param bytes_free Pointer to store free space in bytes
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t sd_card_get_free_space(uint64_t *bytes_free);
 
 /**
  * @brief Real-time clock (RTC) interface
  */
 
 /**
  * @brief RTC time structure
  */
 typedef struct {
     uint16_t year;     /**< Year (e.g., 2023) */
     uint8_t month;     /**< Month (1-12) */
     uint8_t day;       /**< Day (1-31) */
     uint8_t hour;      /**< Hour (0-23) */
     uint8_t minute;    /**< Minute (0-59) */
     uint8_t second;    /**< Second (0-59) */
     uint16_t ms;       /**< Milliseconds (0-999) */
 } rtc_time_t;
 
 /**
  * @brief Initialize RTC hardware
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t rtc_init(void);
 
 /**
  * @brief Get current time from RTC
  *
  * @param time Pointer to store current time
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t rtc_get_time(rtc_time_t *time);
 
 /**
  * @brief Set current time in RTC
  *
  * @param time Pointer to time structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t rtc_set_time(const rtc_time_t *time);
 
 /**
  * @brief Get Unix timestamp (seconds since January 1, 1970)
  *
  * @param timestamp Pointer to store timestamp
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t rtc_get_timestamp(uint32_t *timestamp);
 
 /**
  * @brief System health monitoring interface
  */
 
 /**
  * @brief System health data structure
  */
 typedef struct {
     float mcu_temperature;     /**< MCU temperature in °C */
     float input_voltage;       /**< Input voltage to MCU in V */
     float system_current;      /**< System current consumption in mA */
     uint32_t free_heap;        /**< Free heap memory in bytes */
     uint32_t min_free_heap;    /**< Minimum free heap memory since boot in bytes */
     uint32_t cpu_usage;        /**< CPU usage in percent (0-100) */
     uint32_t uptime_seconds;   /**< System uptime in seconds */
 } system_health_t;
 
 /**
  * @brief Initialize system health monitoring
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t system_health_init(void);
 
 /**
  * @brief Get system health data
  *
  * @param health Pointer to store health data
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t system_health_get_data(system_health_t *health);
 
 /**
  * @brief Internet connectivity hardware interface
  */
 
 /**
  * @brief Network connection status
  */
 typedef enum {
     NETWORK_STATUS_DISCONNECTED = 0,    /**< Not connected to network */
     NETWORK_STATUS_CONNECTING = 1,       /**< Connection in progress */
     NETWORK_STATUS_CONNECTED = 2,        /**< Connected to network but no internet */
     NETWORK_STATUS_INTERNET = 3          /**< Connected with internet access */
 } network_status_t;
 
 /**
  * @brief WiFi configuration structure
  */
 typedef struct {
     char ssid[32];             /**< WiFi SSID */
     char password[64];         /**< WiFi password */
     bool static_ip;            /**< Whether to use static IP */
     char ip_address[16];       /**< Static IP address (if used) */
     char gateway[16];          /**< Gateway address (if static IP) */
     char subnet_mask[16];      /**< Subnet mask (if static IP) */
     char dns_server[16];       /**< DNS server (if static IP) */
 } wifi_config_t;
 
 /**
  * @brief Ethernet configuration structure
  */
 typedef struct {
     bool static_ip;            /**< Whether to use static IP */
     char ip_address[16];       /**< Static IP address (if used) */
     char gateway[16];          /**< Gateway address (if static IP) */
     char subnet_mask[16];      /**< Subnet mask (if static IP) */
     char dns_server[16];       /**< DNS server (if static IP) */
     bool use_mac_eeprom;       /**< Whether to use MAC from EEPROM */
     uint8_t mac_address[6];    /**< MAC address (if not using EEPROM) */
 } ethernet_config_t;
 
 /**
  * @brief Initialize WiFi hardware
  *
  * @param config Pointer to WiFi configuration
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t wifi_hw_init(const wifi_config_t *config);
 
 /**
  * @brief Initialize Ethernet hardware
  *
  * @param config Pointer to Ethernet configuration
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ethernet_hw_init(const ethernet_config_t *config);
 
 /**
  * @brief Get current network status
  *
  * @param status Pointer to store network status
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t network_get_status(network_status_t *status);
 
 /**
  * @brief Test connection to a remote host
  *
  * @param host Hostname or IP address to test
  * @param timeout_ms Timeout in milliseconds
  * @return ESP_OK if reachable, error code otherwise
  */
 esp_err_t network_test_connection(const char *host, uint32_t timeout_ms);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* HW_INTERFACE_H */