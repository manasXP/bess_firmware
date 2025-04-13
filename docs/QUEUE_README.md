# Queue Manager Module

## Overview

The Queue Manager is a centralized message passing system for the 100KW/200KWH BESS controller firmware. It provides a robust, thread-safe mechanism for different subsystems to communicate with each other using FreeRTOS queue primitives, with support for priority messaging, timeout handling, and queue monitoring.

## Key Features

### Thread-Safe Operations
- All operations are protected by a mutex to ensure safety in a multi-threaded FreeRTOS environment
- Careful management of lock acquisition and release to prevent deadlocks
- Timeout mechanisms for all blocking operations

### Multiple Queue Types
- Support for various predefined queue types:
  - Battery data
  - Thermal monitoring data
  - Control commands
  - Modbus messages
  - CANBus messages
  - Logging messages
  - Alarm notifications
  - Cloud data
  - User interface messages
  - Custom queue types

### Priority Messaging
- Four levels of message priority:
  - LOW: Standard, non-urgent messages
  - NORMAL: Default priority for routine operations
  - HIGH: Important messages that should be processed quickly
  - CRITICAL: Emergency messages requiring immediate attention

### Comprehensive Statistics
- Detailed tracking for each queue:
  - Messages sent
  - Messages received
  - Messages dropped (when queue is full)
  - High watermark (maximum usage)
  - Current usage
  - Operation timeouts

### Dynamic Queue Management
- Create and delete queues at runtime
- Configurable queue size and item size
- Queue lookup by name or type
- System-wide queue status monitoring

### Event Notification System
- Register callbacks for queue events
- Receive notifications when messages are processed
- User-defined context data for callbacks
- Thread-safe callback invocation

### Message Metadata
- Envelope structure for rich message information:
  - Timestamp
  - Source and target component IDs
  - Priority level
  - Unique message identifier
  - Data size information

## Integration Guide

### Initialization

Initialize the queue manager at system startup:

```c
// Initialize the queue manager
esp_err_t result = queue_manager_init();
if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize queue manager");
    // Handle error
}
```

### Creating Queues

Create queues for different message types:

```c
// Create a queue for battery data
queue_handle_t* battery_queue;
result = queue_manager_create_queue(
    "battery_data",              // Queue name
    QUEUE_TYPE_BATTERY_DATA,     // Queue type
    20,                          // Queue size (number of items)
    sizeof(bess_battery_data_t), // Item size in bytes
    &battery_queue               // Output: queue handle
);

// Create a queue for thermal data
queue_handle_t* thermal_queue;
result = queue_manager_create_queue(
    "thermal_data",              // Queue name
    QUEUE_TYPE_THERMAL_DATA,     // Queue type
    10,                          // Queue size (number of items)
    sizeof(bess_thermal_data_t), // Item size in bytes
    &thermal_queue               // Output: queue handle
);
```

### Simple Message Passing

Basic send and receive operations:

```c
// Send data to a queue
bess_battery_data_t battery_data = {/* initialize data */};
result = queue_manager_send(battery_queue, &battery_data, 100); // 100ms timeout

// Receive data from a queue
bess_battery_data_t received_data;
result = queue_manager_receive(battery_queue, &received_data, portMAX_DELAY); // Wait indefinitely
```

### Priority Message Passing

Sending messages with priority and metadata:

```c
// Create a message with priority
queue_message_t message;
bess_alarm_t alarm_data = {/* initialize alarm data */};

queue_manager_create_message(
    &alarm_data,                // Message data
    sizeof(bess_alarm_t),       // Data size
    QUEUE_PRIORITY_CRITICAL,    // Priority
    COMPONENT_ID_BATTERY_MGR,   // Source component
    COMPONENT_ID_SYSTEM_CTRL,   // Target component
    &message                    // Output: message envelope
);

// Send the priority message
queue_handle_t* alarm_queue;
queue_manager_get_queue_by_type(QUEUE_TYPE_ALARM, &alarm_queue);
queue_manager_send_with_priority(alarm_queue, &message, 0); // Use default timeout
```

### Receiving Messages with Metadata

```c
// Receive a message with metadata
queue_message_t received_message;
result = queue_manager_receive_with_metadata(
    alarm_queue,
    &received_message,
    1000  // 1 second timeout
);

if (result == ESP_OK) {
    // Access message metadata
    printf("Message ID: %u\n", received_message.message_id);
    printf("Source: %u\n", received_message.source_id);
    printf("Timestamp: %llu\n", received_message.timestamp);
    
    // Process message data
    bess_alarm_t* alarm = (bess_alarm_t*)received_message.data;
    // Handle alarm...
}
```

### Registering for Events

```c
// Callback function for queue events
void alarm_event_handler(queue_type_t queue_type, void* data, void* context) {
    if (queue_type == QUEUE_TYPE_ALARM) {
        bess_alarm_t* alarm = (bess_alarm_t*)data;
        // Handle alarm event
        printf("Alarm received: type=%d, level=%d\n", alarm->type, alarm->level);
    }
}

// Register the callback
queue_manager_register_callback(
    QUEUE_TYPE_ALARM,           // Queue type to monitor
    alarm_event_handler,        // Callback function
    NULL                        // User context (optional)
);
```

### Queue Statistics

Monitoring queue performance:

```c
// Get statistics for a specific queue
queue_stats_t stats;
queue_manager_get_stats(battery_queue, &stats);

printf("Battery queue usage: %u/%u\n", stats.current_usage, battery_queue->size);
printf("Messages sent: %u, received: %u\n", stats.messages_sent, stats.messages_received);
printf("High watermark: %u\n", stats.high_watermark);

// Print statistics for all queues
queue_manager_print_stats(NULL);
```

### Error Handling

```c
// Example of proper error handling
result = queue_manager_send(battery_queue, &battery_data, 100);
if (result == ESP_ERR_TIMEOUT) {
    // Handle timeout - queue might be full
    ESP_LOGW(TAG, "Queue full, message dropped");
    // Implement recovery logic
} else if (result != ESP_OK) {
    // Handle other errors
    ESP_LOGE(TAG, "Failed to send message, error %d", result);
}
```

### Cleanup

```c
// Delete a queue when no longer needed
queue_manager_delete_queue(battery_queue);

// When shutting down, deinitialize the queue manager
queue_manager_deinit();
```

## Advanced Usage

### System Suspension

```c
// Suspend all queue operations during critical system events
queue_manager_suspend();

// Perform critical operations...

// Resume normal queue operations
queue_manager_resume();
```

### Queue Flushing

```c
// Clear all pending messages from a queue
queue_manager_flush(alarm_queue);
```

### Custom Timeout Configuration

```c
// Set a new default timeout for queue operations
queue_manager_set_default_timeout(200); // 200ms
```

## Thread Safety Considerations

- All Queue Manager functions are thread-safe and can be called from any FreeRTOS task
- Callbacks are invoked outside of mutex locks to prevent potential deadlocks
- The implementation handles case where the queue or its handle might be deleted while being accessed

## Performance Considerations

- Use appropriate queue sizes to prevent overflow and message drops
- Monitor high watermarks to identify potential bottlenecks
- Consider using separate queues for different priority levels of the same message type
- Be cautious with timeouts and blocking calls in ISRs or high-priority tasks

## Integration with BESS Components

The Queue Manager is designed to work seamlessly with other BESS firmware components:

- **Battery Manager**: Send battery status updates and receive control commands
- **Thermal Monitor**: Distribute temperature readings and receive cooling controls
- **System Controller**: Coordinate system-wide operations
- **Communications**: Interface with Modbus and CANBus modules
- **Logging System**: Centralized message collection for logging
- **Cloud Interface**: Queue data for transmission to AWS CloudWatch

## Error Codes

The Queue Manager uses standard ESP-IDF error codes:

- **ESP_OK**: Operation completed successfully
- **ESP_FAIL**: Generic failure
- **ESP_ERR_TIMEOUT**: Operation timed out, typically when a queue is full or empty
- **ESP_ERR_INVALID_ARG**: Invalid arguments provided to a function
- **ESP_ERR_INVALID_STATE**: Queue manager in invalid state for the operation

## Future Enhancements

Potential improvements for future firmware versions:

1. **Queue Set Support**: Enhanced support for FreeRTOS queue sets for monitoring multiple queues
2. **Message Filtering**: Ability to filter messages at the queue level based on content
3. **Persistent Queues**: Option to store critical messages in non-volatile storage
4. **Queue Monitoring Task**: Dedicated task for real-time monitoring and statistics collection
5. **Dynamic Priority Adjustment**: Adapt message priority based on system state
6. **Message Batching**: Support for sending/receiving multiple messages in a single operation
7. **Inter-Core Communication**: Optimized queues for communication between CPU cores
8. **Queue Visualization**: Debug interface for visualizing queue activity
