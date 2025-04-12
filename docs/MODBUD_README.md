# Modbus Interface for BESS Controller

## Overview

The Modbus Interface provides communication capabilities for the 100KW/200KWH Battery Energy Storage System (BESS) controller. It enables the controller to communicate with various system components including inverters, battery modules, and SCADA systems using the industry-standard Modbus protocol.

## Features

### Transport Layer Support
- **Modbus RTU**: Serial communication over RS-232/RS-485
- **Modbus TCP**: Ethernet-based communication

### Operation Modes
- **Master Mode**: For controlling and querying slave devices
- **Slave Mode**: For responding to external control systems

### Function Code Support
- **Data Reading**:
  - Read Coils (0x01)
  - Read Discrete Inputs (0x02)
  - Read Holding Registers (0x03)
  - Read Input Registers (0x04)
  
- **Data Writing**:
  - Write Single Coil (0x05)
  - Write Single Register (0x06)
  - Write Multiple Coils (0x0F)
  - Write Multiple Registers (0x10)

### System Integration
- Thread-safe design for FreeRTOS environment
- Callback system for register access customization
- Internal register map for standalone operation
- Comprehensive error handling

## Architecture

The Modbus interface consists of the following components:

1. **Configuration System**: Allows customization of transport, mode, and addressing
2. **Protocol Handler**: Processes Modbus PDUs and executes appropriate operations
3. **Transport Layer**: Manages RTU (UART) or TCP communication
4. **Register Maps**: Internal storage for Modbus registers
5. **Callback System**: Allows application integration with register operations

## API Reference

### Initialization and Configuration

```c
esp_err_t modbus_init(const modbus_config_t *config);
esp_err_t modbus_deinit(void);
```

### Register Callbacks

```c
esp_err_t modbus_register_callback(modbus_register_type_t reg_type, 
                                  modbus_register_callback_t callback, void *arg);
```

### Register Access (Internal Maps)

```c
esp_err_t modbus_set_register(modbus_register_type_t reg_type, uint16_t address, uint16_t value);
esp_err_t modbus_get_register(modbus_register_type_t reg_type, uint16_t address, uint16_t *value);
```

### Master Mode Operations

```c
// Reading Functions
esp_err_t modbus_master_read_coils(uint8_t slave_addr, uint16_t start_address, 
                                 uint16_t quantity, uint8_t *coil_status);
esp_err_t modbus_master_read_discrete_inputs(uint8_t slave_addr, uint16_t start_address, 
                                          uint16_t quantity, uint8_t *input_status);
esp_err_t modbus_master_read_holding_registers(uint8_t slave_addr, uint16_t start_address, 
                                            uint16_t quantity, uint16_t *holding_registers);
esp_err_t modbus_master_read_input_registers(uint8_t slave_addr, uint16_t start_address, 
                                          uint16_t quantity, uint16_t *input_registers);

// Writing Functions
esp_err_t modbus_master_write_single_coil(uint8_t slave_addr, uint16_t coil_address, 
                                        uint8_t coil_value);
esp_err_t modbus_master_write_single_register(uint8_t slave_addr, uint16_t register_address, 
                                           uint16_t register_value);
esp_err_t modbus_master_write_multiple_coils(uint8_t slave_addr, uint16_t start_address, 
                                          uint16_t quantity, const uint8_t *coil_values);
esp_err_t modbus_master_write_multiple_registers(uint8_t slave_addr, uint16_t start_address, 
                                             uint16_t quantity, const uint16_t *register_values);
```

### Slave Mode Operations

```c
esp_err_t modbus_slave_set_register_map(modbus_register_type_t reg_type, 
                                      uint16_t start_address, uint16_t count, 
                                      const uint16_t *values);
esp_err_t modbus_slave_get_register_map(modbus_register_type_t reg_type, 
                                      uint16_t start_address, uint16_t count, 
                                      uint16_t *values);
```

## Configuration

### Modbus RTU Configuration

```c
modbus_config_t config = {
    .mode = MODBUS_MODE_MASTER,  // or MODBUS_MODE_SLAVE
    .transport = MODBUS_TRANSPORT_RTU,
    .slave_address = 1,          // Used in slave mode
    .rtu_config = {
        .uart_port = UART_NUM_1,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_16,
        .rts_pin = GPIO_NUM_18,
        .de_pin = GPIO_NUM_NC,   // Optional driver enable pin
        .baudrate = 9600,
        .parity = UART_PARITY_EVEN,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .is_rs485 = true
    }
};
```

### Modbus TCP Configuration

```c
modbus_config_t config = {
    .mode = MODBUS_MODE_SLAVE,   // or MODBUS_MODE_MASTER
    .transport = MODBUS_TRANSPORT_TCP,
    .slave_address = 1,
    .tcp_config = {
        .port = 502,            // Standard Modbus TCP port
        .task_priority = 10,
        .core_id = 0
    }
};
```

## Register Map Organization

The BESS controller organizes its Modbus register map as follows:

### Input Registers (Read-Only)

| Address Range | Description                     |
|--------------|---------------------------------|
| 30001-30100  | Battery measurements            |
| 30101-30200  | Inverter measurements           |
| 30201-30300  | System status indicators        |
| 30301-30400  | Error and alarm status          |
| 30401-30500  | Performance metrics             |

### Holding Registers (Read-Write)

| Address Range | Description                     |
|--------------|---------------------------------|
| 40001-40100  | System control parameters       |
| 40101-40200  | Battery control settings        |
| 40201-40300  | Inverter control settings       |
| 40301-40400  | Protection thresholds           |
| 40401-40500  | System configuration            |

### Coils (Read-Write)

| Address Range | Description                     |
|--------------|---------------------------------|
| 00001-00100  | Control flags                   |
| 00101-00200  | Operation mode selection        |
| 00201-00300  | Enable/disable features         |

### Discrete Inputs (Read-Only)

| Address Range | Description                     |
|--------------|---------------------------------|
| 10001-10100  | System status flags             |
| 10101-10200  | Alarm indicators                |
| 10201-10300  | System state indicators         |

## Integration with BESS Components

### Battery Module Communication

The Modbus interface allows the main controller to communicate with the Battery Management System (BMS) to:

- Read battery cell voltages, temperatures, and currents
- Monitor State of Charge (SoC) and State of Health (SoH)
- Control cell balancing operations
- Set charging and discharging parameters
- Retrieve battery status and fault information

### Inverter Communication

The controller communicates with power conversion systems to:

- Control power output and input levels
- Adjust operating modes (grid-tied, island, hybrid)
- Monitor conversion efficiency
- Retrieve power quality metrics
- Set protective parameters

### SCADA Integration

For external monitoring and control systems, the interface provides:

- Real-time system status reporting
- Historical data access
- Remote configuration capabilities
- Alarm and event notification
- Control command processing

## Error Handling

The Modbus interface provides comprehensive error reporting:

- Communication errors (timeouts, CRC failures)
- Protocol errors (invalid function codes, addresses)
- System errors (resource unavailability)
- Implementation-specific errors

All functions return ESP-IDF standard error codes for consistent error handling.

## Thread Safety

All Modbus interface functions are thread-safe for use in the multitasking FreeRTOS environment. Internal mutexes protect shared data structures from concurrent access issues.

## Future Enhancements

- Modbus ASCII transport support
- Gateway functionality for protocol translation
- Extended diagnostic functions
- Automatic device discovery
- Redundant communication paths

## Dependencies

- ESP-IDF (ESP32-P4 SDK)
- FreeRTOS
- LWIP (for TCP transport)
