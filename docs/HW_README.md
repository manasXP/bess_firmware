# BESS Hardware Interface Layer

## Overview

The Hardware Interface Layer for the 100KW/200KWH Battery Energy Storage System (BESS) provides a comprehensive abstraction layer between the physical hardware components and the higher-level firmware. This layer handles all direct hardware interactions, providing clean APIs for the BMS (Battery Management System) components to use without needing to understand hardware specifics.

## System Architecture

The BESS hardware interface is designed for an ESP32-P4 MCU running FreeRTOS. It interfaces with:

- LFP battery modules (48V, 16KWH capacity each)
- Cooling system components
- Communication buses (Modbus RTU, Modbus TCP, CANBus)
- Storage (SD Card)
- Real-time clock
- Network connectivity (WiFi, Ethernet)

## Components

### Battery Module Interface

Provides functions to interact with up to 16 battery modules, each containing up to 16 cells.

**Features:**
- Read battery module data (cell voltages, module voltage, current, temperature)
- Control cell balancing
- Emergency stop functionality
- Alarm monitoring (overvoltage, undervoltage, overcurrent, overtemperature)

**Key Functions:**
```c
esp_err_t battery_hw_init(uint8_t module_count);
esp_err_t battery_hw_read_module(uint8_t module_id, bess_battery_data_t *data);
esp_err_t battery_hw_set_balancing(uint8_t module_id, uint32_t balance_mask);
esp_err_t battery_hw_set_emergency_stop(bool emergency_stop);
```

### Thermal Management Interface

Manages cooling systems to maintain optimal operating temperatures for battery modules.

**Features:**
- Multiple cooling modes (Off, Passive, Low, Medium, High, Max)
- PWM-based cooling power control
- Ambient temperature monitoring

**Key Functions:**
```c
esp_err_t thermal_hw_init(void);
esp_err_t thermal_hw_set_cooling_mode(bess_cooling_mode_t mode);
esp_err_t thermal_hw_set_cooling_power(uint8_t power_percent);
esp_err_t thermal_hw_read_ambient_temp(float *temperature);
```

### Communication Interfaces

#### Modbus RTU

Serial-based Modbus implementation with RS485 support.

**Features:**
- Configurable UART parameters (baud rate, data bits, parity, stop bits)
- RS485 direction control support
- Standard Modbus function code handling

**Key Functions:**
```c
esp_err_t modbus_rtu_init(const modbus_rtu_config_t *config);
esp_err_t modbus_rtu_process(uint32_t timeout_ms);
```

#### Modbus TCP

Ethernet-based Modbus server implementation.

**Features:**
- Configurable TCP port and connections
- Standard Modbus function code handling over TCP/IP

**Key Functions:**
```c
esp_err_t modbus_tcp_init(const modbus_tcp_config_t *config);
esp_err_t modbus_tcp_start(void);
esp_err_t modbus_tcp_stop(void);
```

#### CANBus Interface

Provides access to the CAN (Controller Area Network) bus for high-reliability communication.

**Features:**
- Support for standard and extended frame IDs
- Message filtering
- Callback-based message handling
- Multiple baud rate support

**Key Functions:**
```c
esp_err_t canbus_init(const canbus_config_t *config);
esp_err_t canbus_send_message(const canbus_message_t *message);
esp_err_t canbus_receive_message(canbus_message_t *message, uint32_t timeout_ms);
esp_err_t canbus_register_callback(uint32_t id, void (*callback)(canbus_message_t*, void*), void *user_data);
```

### Storage Interface

Provides access to an SD card for data logging and configuration storage.

**Features:**
- FAT filesystem support
- Mount/unmount functionality
- Free space monitoring

**Key Functions:**
```c
esp_err_t sd_card_init(const char *mount_point, size_t max_files);
bool sd_card_is_mounted(void);
esp_err_t sd_card_get_free_space(uint64_t *bytes_free);
```

### Real-Time Clock Interface

Manages system time with optional external RTC support.

**Features:**
- Internal RTC usage
- Optional external RTC chip support
- SNTP synchronization when network available
- Timestamp conversion

**Key Functions:**
```c
esp_err_t rtc_init(void);
esp_err_t rtc_get_time(rtc_time_t *time);
esp_err_t rtc_set_time(const rtc_time_t *time);
esp_err_t rtc_get_timestamp(uint32_t *timestamp);
```

### System Health Monitoring

Monitors various system parameters to ensure reliable operation.

**Features:**
- MCU temperature monitoring
- Input voltage and system current monitoring
- Memory usage tracking
- CPU usage estimation
- System uptime tracking

**Key Functions:**
```c
esp_err_t system_health_init(void);
esp_err_t system_health_get_data(system_health_t *health);
```

### Network Connectivity

Provides WiFi and Ethernet connectivity for remote monitoring and control.

**Features:**
- WiFi station mode support
- Ethernet support with multiple PHY options
- Static and dynamic IP configuration
- Internet connectivity testing

**Key Functions:**
```c
esp_err_t wifi_hw_init(const wifi_config_t *config);
esp_err_t ethernet_hw_init(const ethernet_config_t *config);
esp_err_t network_get_status(network_status_t *status);
esp_err_t network_test_connection(const char *host, uint32_t timeout_ms);
```

## Implementation Details

### Thread Safety

All functions are designed to be thread-safe, using mutexes to protect shared resources. This ensures reliable operation in a multi-tasking FreeRTOS environment where multiple tasks may access hardware simultaneously.

### Error Handling

The interface follows ESP-IDF error handling conventions:
- All functions return `esp_err_t` 
- `ESP_OK` indicates success
- Various error codes indicate specific failure modes
- Detailed error logging helps diagnose issues

### Configuration

The hardware interface relies on several configuration parameters that should be defined in a project configuration file:

**Key Configuration Parameters:**
- `CONFIG_BESS_BAT_COMM_ENABLE_PIN`: GPIO pin for battery communication enable
- `CONFIG_BESS_BAT_EMSTOP_PIN`: GPIO pin for emergency stop
- `CONFIG_BESS_COOLING_CONTROL_PIN`: GPIO pin for cooling system control
- `CONFIG_BESS_SD_CS_PIN`: GPIO pin for SD card chip select
- `CONFIG_BESS_RTC_SDA_PIN`: GPIO pin for RTC I2C SDA
- `CONFIG_BESS_RTC_SCL_PIN`: GPIO pin for RTC I2C SCL
- `CONFIG_BESS_ETH_PHY_ADDR`: Ethernet PHY address
- `CONFIG_BESS_ETH_PHY_RST_GPIO`: GPIO pin for Ethernet PHY reset
- `CONFIG_BESS_USE_EXTERNAL_RTC`: Flag to enable external RTC
- `CONFIG_BESS_SIMULATION_MODE`: Flag to enable simulation mode for testing

### Simulation Mode

The hardware interface includes a simulation mode for testing without actual hardware. When `CONFIG_BESS_SIMULATION_MODE` is defined, the interface generates realistic simulated data for battery modules, temperatures, and other sensor readings.

## Code Structure

The hardware interface is implemented in two main files:

1. **hw_interface.h**: Contains all the interface definitions, data structures, and function declarations.
2. **hw_interface.c**: Contains the implementation of all interface functions.

Example implementations of the header and source files are included in this repository:

### hw_interface.h

The header file defines all interfaces for communicating with hardware components of the BESS.

```c
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

// ... (Detailed interface definitions)

#endif /* HW_INTERFACE_H */
```

### hw_interface.c

The implementation file contains the actual code for interacting with hardware.

```c
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
// ... (Other includes)

#define TAG "HW_INTERFACE"

// Forward declarations for event handlers
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

// ... (Interface implementations)
```

## Usage Examples

### Battery Module Monitoring

```c
// Initialize battery hardware with 12 modules
esp_err_t ret = battery_hw_init(12);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize battery hardware: %s", esp_err_to_name(ret));
    return;
}

// Read data from module 0
bess_battery_data_t module_data;
ret = battery_hw_read_module(0, &module_data);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Module 0: %.2fV, %.2fA, %.2f°C", 
             module_data.module_voltage,
             module_data.module_current,
             module_data.temperatures[0]);
}
```

### Setting Up Cell Balancing

```c
// Start balancing on cells 0, 2, and 5 of module 3
uint32_t balance_mask = (1 << 0) | (1 << 2) | (1 << 5);
esp_err_t ret = battery_hw_set_balancing(3, balance_mask);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Started balancing on module 3");
}
```

### Controlling Cooling System

```c
// Initialize thermal hardware
thermal_hw_init();

// Set cooling to medium mode
thermal_hw_set_cooling_mode(COOLING_MODE_MEDIUM);

// Or set specific power level
thermal_hw_set_cooling_power(75);  // 75% power
```

### CANBus Communication

```c
// Initialize CAN bus at 500 kbps
canbus_config_t can_config = {
    .baud_rate = 500000,
    .tx_pin = CONFIG_BESS_CAN_TX_PIN,
    .rx_pin = CONFIG_BESS_CAN_RX_PIN,
    .accept_all_frames = true
};
canbus_init(&can_config);

// Send a CAN message
canbus_message_t msg = {
    .identifier = 0x123,
    .extended_frame = false,
    .remote_frame = false,
    .data_length = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
};
canbus_send_message(&msg);

// Register a callback for specific message ID
canbus_register_callback(0x456, can_message_handler, NULL);
```

### Setting Up WiFi

```c
// Configure WiFi
wifi_config_t wifi_config = {
    .ssid = "BESS_Network",
    .password = "secure_password",
    .static_ip = true,
    .ip_address = "192.168.1.100",
    .gateway = "192.168.1.1",
    .subnet_mask = "255.255.255.0",
    .dns_server = "8.8.8.8"
};

// Initialize WiFi
wifi_hw_init(&wifi_config);

// Test internet connectivity
network_test_connection("http://example.com", 5000);
```

## Future Enhancements

Potential improvements to the hardware interface layer:

1. **Power Management**: Add support for low-power modes and wake-up sources
2. **Additional Peripherals**: Support for displays, keypads, etc.
3. **Expanded Diagnostics**: More detailed hardware diagnostics and testing tools
4. **Hot-Swap Support**: Improved detection and handling of module hot-swapping
5. **Remote Firmware Updates**: Support for OTA updates
6. **Enhanced Security**: Hardware-based authentication and encryption

## Conclusion

The Hardware Interface Layer provides a robust foundation for the BESS firmware, abstracting hardware complexities and providing clean APIs for higher-level components. This modular design allows for easier testing, maintenance, and future hardware changes without affecting the core application logic.
