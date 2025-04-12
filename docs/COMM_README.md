# Communication Manager Module

## Overview

The Communication Manager is a critical component of the BESS (Battery Energy Storage System) firmware that provides central management of all communication interfaces. It handles Modbus RTU, Modbus TCP, CANbus, and network communications like MQTT, enabling the 100KW/200KWH BESS to interact with other system components, monitoring systems, and cloud services.

## Architecture

The Communication Manager is implemented using a modular, task-based architecture on the ESP32-P4 platform using FreeRTOS. Each communication interface operates in a separate task, with thread-safe data structures and event-based coordination.

### Core Components

1. **Interface Manager**
   - Centralized control of all communication interfaces
   - Handles interface enabling/disabling
   - Tracks interface status and statistics
   - Provides unified API for other system components

2. **Modbus RTU Implementation**
   - Serial-based communication using UART
   - Supports standard Modbus function codes
   - Configurable baud rate, parity, and timeout settings
   - Thread-safe operation in dedicated task

3. **Modbus TCP Implementation**
   - Network-based Modbus communication
   - Support for multiple client connections
   - Standard TCP/IP stack integration
   - Modbus register mapping for system data

4. **CANbus Interface**
   - Implements industrial CANbus protocols
   - Uses ESP32's TWAI driver (Two-Wire Automotive Interface)
   - Support for standard and extended CAN frames
   - Configurable message filtering

5. **Network Communication**
   - MQTT client for cloud connectivity
   - HTTP server/client for monitoring and control
   - Optional BLE interface for local configuration
   - Secure communication options

## Key Features

- **Multi-Protocol Support**: Simultaneously manages multiple communication protocols
- **Thread-Safe Design**: All operations protected with mutex for concurrent access safety
- **Dynamic Interface Management**: Interfaces can be enabled/disabled at runtime
- **Comprehensive Statistics**: Detailed tracking of message counts, bytes, and errors
- **Event-Based Notification**: Callback system for signaling communication events
- **Configurable Parameters**: Adjustable timeouts, buffer sizes, and protocol settings
- **Error Handling**: Robust error detection and recovery mechanisms
- **Resource Management**: Efficient memory usage and proper resource cleanup

## Implementation Details

### Task Structure

The Communication Manager uses multiple FreeRTOS tasks:

1. **Modbus RTU Task**: Manages serial Modbus communication
   - Runs with priority 5 on core 1
   - Handles UART configuration and data exchange
   - Processes Modbus RTU frames according to specification

2. **Modbus TCP Task**: Handles network Modbus communication
   - Processes TCP socket connections
   - Manages multiple client sessions
   - Implements Modbus TCP protocol

3. **CANbus Task**: Manages CAN communications
   - Configures and manages the TWAI controller
   - Handles message transmission and reception
   - Provides filtering capabilities

4. **MQTT Task**: Handles cloud connectivity
   - Manages connection to MQTT broker
   - Publishes system status and alerts
   - Subscribes to control commands

### Data Synchronization

The Communication Manager implements several synchronization mechanisms:

- **Mutex (`s_comm_mutex`)**: Protects shared interface data structures
- **Event Group (`s_comm_event_group`)**: Signals when data is available from interfaces
- **Task Notifications**: Used for task-specific signaling

### Statistics Tracking

For each interface, the system tracks:

- Message counts (TX/RX)
- Byte counts (TX/RX)
- Error counts
- Timestamps for last activities
- Connection status

### Error Handling

The module implements comprehensive error detection and handling:

- Communication timeouts
- Protocol violations
- Buffer overflows
- Hardware errors
- Connection failures

## API Interface

The Communication Manager exposes a comprehensive API for other system components:

### Initialization and Control

- `comm_manager_init()`: Initialize the communication subsystem
- `comm_manager_start()`: Start communication tasks
- `comm_manager_enable_interface()`: Enable a specific interface
- `comm_manager_disable_interface()`: Disable a specific interface

### Status and Statistics

- `comm_manager_get_interface_status()`: Get detailed status for an interface
- `comm_manager_is_interface_enabled()`: Check if an interface is enabled
- `comm_manager_is_interface_connected()`: Check if an interface is connected
- `comm_manager_get_statistics()`: Get statistics for all interfaces
- `comm_manager_reset_statistics()`: Reset statistics for an interface

### Event Management

- `comm_manager_register_callback()`: Register for interface events
- `comm_manager_unregister_callback()`: Unregister from interface events
- `comm_manager_process_data()`: Process data from all interfaces

## Usage Examples

### Initializing Communication Manager

```c
// Initialize the communication manager
esp_err_t result = comm_manager_init();
if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize communication manager");
    return result;
}

// Enable specific interfaces
comm_manager_enable_interface(COMM_INTERFACE_MODBUS_RTU);
comm_manager_enable_interface(COMM_INTERFACE_CANBUS);

// Start the communication manager tasks
result = comm_manager_start();
if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start communication manager");
    return result;
}
```

### Registering for Communication Events

```c
// Event callback function
void communication_event_handler(uint32_t event_data, void *user_data) {
    ESP_LOGI(TAG, "Communication event received: %lu", event_data);
    
    // Handle the event based on the interface and data
    switch (*(comm_interface_type_t*)user_data) {
        case COMM_INTERFACE_MODBUS_RTU:
            // Handle Modbus RTU event
            break;
            
        case COMM_INTERFACE_CANBUS:
            // Handle CANbus event
            break;
            
        // Handle other interface events...
    }
}

// Register callback for Modbus RTU events
comm_interface_type_t interface_type = COMM_INTERFACE_MODBUS_RTU;
comm_manager_register_callback(interface_type, communication_event_handler, &interface_type);
```

### Checking Interface Status

```c
// Get detailed status for CANbus interface
comm_interface_status_t status;
esp_err_t result = comm_manager_get_interface_status(COMM_INTERFACE_CANBUS, &status);

if (result == ESP_OK) {
    printf("CANbus interface status:\n");
    printf("  Enabled:     %s\n", status.enabled ? "Yes" : "No");
    printf("  Connected:   %s\n", status.connected ? "Yes" : "No");
    printf("  Messages TX: %lu\n", status.tx_count);
    printf("  Messages RX: %lu\n", status.rx_count);
    printf("  Bytes TX:    %lu\n", status.bytes_tx);
    printf("  Bytes RX:    %lu\n", status.bytes_rx);
    printf("  Errors:      %lu\n", status.error_count);
}
```

## Integration with BESS System

The Communication Manager interacts with other BESS components:

- **Battery Manager**: Provides battery data for external monitoring
- **System Controller**: Receives control commands from external sources
- **Data Logger**: Publishes system logs and data to remote servers
- **User Interface**: Provides data for local or remote user interfaces

## Security Considerations

The Communication Manager implements several security features:

- **Input Validation**: All received data is validated before processing
- **Buffer Management**: Proper buffer sizing to prevent overflows
- **Error Detection**: CRC and checksum validation for data integrity
- **Authentication**: Support for secure authentication where applicable
- **Rate Limiting**: Protection against communication flooding

## Future Enhancements

Potential improvements for future firmware versions:

1. **Encryption**: Add support for encrypted communications
2. **Protocol Extensions**: Support for additional industrial protocols
3. **Advanced Diagnostics**: Enhanced self-diagnostic capabilities
4. **Load Balancing**: Optimized task scheduling based on system load
5. **Redundancy**: Support for redundant communication channels
6. **Auto-Discovery**: Dynamic discovery of compatible devices
7. **Compression**: Data compression for bandwidth-limited channels
