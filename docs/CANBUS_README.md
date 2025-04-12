# CANbus Interface Implementation

## Overview

This document describes the CANbus communication interface implementation for the Battery Energy Storage System (BESS) with 100KW power and 200KWH capacity. The implementation is designed for the ESP32-P4 MCU using FreeRTOS and leverages the ESP-IDF TWAI driver (ESP's CAN driver, renamed from "CAN" to "TWAI").

## Key Features

- **Complete Thread Safety**: Uses FreeRTOS synchronization primitives (mutexes, queues) to ensure thread-safe operation in a multitasking environment.
- **Dedicated TX/RX Tasks**: Separate high-priority tasks handle message transmission and reception, ensuring responsive communication.
- **Flexible Message Filtering**: Supports registering multiple callbacks with ID and mask-based filtering to process specific message types.
- **Comprehensive Event System**: Monitors bus status and provides callbacks for important events like bus-off conditions, error states, and recovery.
- **Robust Error Handling**: Includes timeout handling, error recovery, and message retransmission strategies.
- **Configurable Bitrate**: Supports standard CAN bitrates from 100kbps to 1Mbps.
- **Module Communication Support**: Includes utility functions for communicating with battery modules, including requesting data and sending commands.
- **Message Parsing and Creation**: Provides functions to parse module status messages and create various message types like heartbeat and system status.

## Implementation Details

### Architecture

The CANbus interface is implemented with the following components:

1. **State Management Structure**:
   - Maintains the state of the CANbus interface
   - Tracks initialization and running status
   - Stores registered callbacks
   - Manages synchronization primitives

2. **Task Structure**:
   - RX Task: Continuously monitors the bus for incoming messages and bus status changes
   - TX Task: Handles message transmission from a queue

3. **Callback System**:
   - Message Callbacks: Registered for specific message IDs with filtering masks
   - Event Callbacks: Triggered for bus status changes and error conditions

4. **Synchronization Mechanism**:
   - Mutex for protecting callback registrations
   - Mutex for protecting status information
   - Queue for transmit messages

### Message ID Structure

The implementation uses the following message ID structure (example):

- **0x100-0x1FF**: System status messages
- **0x500-0x5FF**: Module status messages (module ID in lower 8 bits)
- **0x600-0x6FF**: Data request messages (module ID in lower 8 bits)
- **0x700-0x7FF**: Command messages (module ID in lower 8 bits)

### Module Communication

The interface provides specific functions for module communication:

- **Request Module Data**: Request specific data from a module
- **Send Module Command**: Send a command to a module
- **Parse Module Status**: Extract module status information from a CAN message

## Usage Examples

### Initialization and Configuration

```c
// Initialize with default settings
esp_err_t ret = canbus_interface_init();
if (ret != ESP_OK) {
    // Handle initialization error
}

// Set bitrate to 500 kbps (optional, default is 500 kbps)
canbus_interface_set_bitrate(500000);

// Start the interface
ret = canbus_interface_start();
if (ret != ESP_OK) {
    // Handle start error
}
```

### Registering Message Callbacks

```c
// Callback function for module status messages
void module_status_callback(twai_message_t *message, void *user_data) {
    canbus_module_status_t status;
    
    if (canbus_interface_parse_module_status(message, &status) == ESP_OK) {
        printf("Module %d: %.2fV, %.1fA, %.1f°C, SoC: %d%%\n",
               status.module_id, status.voltage, status.current,
               status.temperature, status.state_of_charge);
    }
}

// Register callback for all module status messages (0x500-0x5FF)
canbus_interface_register_message_callback(
    0x500,                  // Base ID
    0xF00,                  // Mask: match top 4 bits
    module_status_callback,
    NULL                    // No user data
);
```

### Registering Event Callbacks

```c
// Callback function for bus-off events
void bus_off_callback(canbus_event_type_t event, void *user_data) {
    printf("CAN bus entered bus-off state!\n");
    // Take appropriate action
}

// Register callback for bus-off events
canbus_interface_register_event_callback(
    CANBUS_EVENT_BUS_OFF,
    bus_off_callback,
    NULL                    // No user data
);
```

### Sending Messages

```c
// Request voltage data from module 3
canbus_interface_request_module_data(3, 0x01); // 0x01 = voltage data type

// Send balance command to module 5
uint8_t balance_data[2] = {0x01, 0x00}; // Enable balancing
canbus_interface_send_module_command(5, 0x10, balance_data, 2); // 0x10 = balance command

// Send system heartbeat
twai_message_t heartbeat;
canbus_interface_create_heartbeat_message(&heartbeat);
canbus_interface_send_message(&heartbeat);
```

### Cleanup

```c
// Stop the interface when done
canbus_interface_stop();
```

## Error Handling

The interface provides comprehensive error handling:

- All functions return ESP error codes (ESP_OK or error code)
- The interface monitors bus status and triggers appropriate callbacks
- Automatic recovery from bus-off state
- Message retransmission for certain error types

## Performance Considerations

- RX and TX tasks run at priority 10 (configurable)
- Stack size is 4096 bytes per task (configurable)
- Transmit queue can hold up to 32 messages (configurable)
- Message callbacks are executed in the context of the RX task

## Advanced Features

### Manual Filter Configuration

For more complex filtering needs, the TWAI driver's filter can be reconfigured after initialization:

```c
twai_filter_config_t f_config = {
    .acceptance_code = 0x500,
    .acceptance_mask = 0x700,
    .single_filter = true
};
twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
```

### Bus Monitoring

The interface provides functions to monitor bus status:

```c
twai_status_info_t status;
canbus_interface_get_status(&status);
printf("TX errors: %d, RX errors: %d, msgs in TX queue: %d\n",
       status.tx_error_counter, status.rx_error_counter, status.msgs_to_tx);
```

