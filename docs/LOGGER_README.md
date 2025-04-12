# Logging Subsystem for BESS 100KW/200KWH Firmware

## Overview

The Logging Subsystem is a critical component of the BESS (Battery Energy Storage System) firmware that provides a unified logging interface with multiple output destinations. This module is specifically designed for the 100KW/200KWH BESS with LFP battery modules (48V, 16KWH) running on ESP32-P4 with FreeRTOS.

## Key Features

### Multiple Output Destinations

- **Console Logging**: Real-time display with color-coded output based on log level
- **SD Card Logging**: Persistent storage with automatic file rotation and management
- **AWS CloudWatch**: Remote cloud-based logging for monitoring and analytics

### Configurable Logging Levels

The system supports multiple log levels with independent filtering for each destination:

- **ERROR**: Critical errors that prevent system operation
- **WARN**: Warning conditions that should be addressed
- **INFO**: Informational messages about system operation
- **DEBUG**: Detailed debugging information
- **VERBOSE**: Very detailed debugging information
- **EVENT**: System events for monitoring
- **AUDIT**: Security and access-related events for compliance tracking

### Thread-Safe Design

- Mutex protection for all shared data structures
- Queue-based architecture for non-blocking logging operations
- Dedicated FreeRTOS task for asynchronous message processing
- Event groups for coordinating operations like flushing and shutdown

### SD Card Management

- Automatic log file rotation based on configurable maximum file size
- Timestamped log filenames for easy identification
- Limit on total number of log files with automatic cleanup of oldest files
- Robust error handling for SD card mounting and file operations

### CloudWatch Integration

- Configurable AWS region, log group, and log stream
- Efficient batched uploading to reduce network overhead
- Automatic sequence token management
- Timeout-based flushing for timely delivery of low-volume logs

### Comprehensive Statistics

- Messages queued, processed, and dropped
- Per-destination message counts
- Error counts for SD card and CloudWatch operations
- Queue high watermark tracking

## Implementation Details

### Architecture

The logging subsystem consists of these main components:

1. **Public API**: Functions and macros for submitting log messages
2. **Message Queue**: Thread-safe buffer for storing pending log messages
3. **Logger Task**: Background task that processes queued messages
4. **Destination Handlers**: Specialized code for each output destination
5. **Configuration System**: Settings for controlling logger behavior

### Technical Features

- **Failsafe Operation**: Continues functioning even if some destinations fail
- **Memory Efficiency**: Optimized data structures with minimal overhead
- **Dynamic Configuration**: Runtime adjustment of logging parameters
- **Robust Error Handling**: Comprehensive validation and error recovery

## Integration Guide

### Initialization

Initialize the logging subsystem at system startup:

```c
// Use default configuration
bess_logger_init(NULL);

// Or with custom configuration
bess_logger_config_t config = BESS_LOGGER_DEFAULT_CONFIG;
config.console_level = BESS_LOG_INFO;
config.sd_card_level = BESS_LOG_DEBUG;
config.cloud_level = BESS_LOG_ERROR;
bess_logger_init(&config);
```

### Basic Logging

Use the convenience macros for standard logging:

```c
// Log at different levels
BESS_LOGE(BESS_MODULE_BATTERY_MANAGER, "Critical error: %s", error_message);
BESS_LOGW(BESS_MODULE_THERMAL_MONITOR, "Temperature high: %.1f°C", temperature);
BESS_LOGI(BESS_MODULE_SYSTEM, "System started successfully");
BESS_LOGD(BESS_MODULE_CANBUS, "CAN message received: ID=0x%X, Data=%s", id, data);
BESS_LOGV(BESS_MODULE_MODBUS, "Register read: addr=0x%04X, value=0x%04X", addr, value);

// Log events and audit records
BESS_LOG_EVENT(BESS_MODULE_PROTECTION, "Overvoltage protection activated");
BESS_LOG_AUDIT(BESS_MODULE_USER_INTERFACE, "Configuration changed by user %s", username);
```

### Destination-Specific Logging

Send logs to specific destinations:

```c
// Log only to console
BESS_LOG_CONSOLE(BESS_LOG_DEBUG, BESS_MODULE_DIAGNOSTICS, "Running self-test...");

// Log only to SD card
BESS_LOG_SD(BESS_LOG_AUDIT, BESS_MODULE_POWER_CONTROL, "Power setpoint changed to %d kW", setpoint);

// Log only to CloudWatch
BESS_LOG_CLOUD(BESS_LOG_ERROR, BESS_MODULE_PROTECTION, "Emergency shutdown triggered");
```

### Configuration Management

Adjust logging behavior at runtime:

```c
// Change log levels
bess_logger_set_level(BESS_LOG_DEST_CONSOLE, BESS_LOG_DEBUG);
bess_logger_set_level(BESS_LOG_DEST_SD_CARD, BESS_LOG_INFO);
bess_logger_set_level(BESS_LOG_DEST_CLOUD, BESS_LOG_ERROR);

// Enable/disable destinations
bess_logger_enable_destination(BESS_LOG_DEST_SD_CARD, true);
bess_logger_enable_destination(BESS_LOG_DEST_CLOUD, wifi_connected);

// Configure SD card parameters
bess_logger_configure_sd("/sdcard/bess_logs", 2 * 1024 * 1024, 20);

// Configure CloudWatch parameters
bess_logger_configure_cloud("us-west-2", "BESS-Production", "Unit-123", 50, 30000);
```

### Memory Management

The logging system automatically manages memory for log messages. However, applications should be careful to:

- Avoid excessively long or frequent messages that could fill the queue
- Consider lower log levels for high-volume message sources
- Flush logs before system shutdown for critical information

```c
// Flush all pending logs with a timeout
bess_logger_flush(5000);  // 5-second timeout
```

### Statistics Monitoring

Track logging system performance:

```c
bess_logger_stats_t stats;
bess_logger_get_stats(&stats);

printf("Messages processed: %u\n", stats.messages_processed);
printf("Messages dropped: %u\n", stats.messages_dropped);
printf("Console messages: %u\n", stats.console_messages);
printf("SD card messages: %u\n", stats.sd_card_messages);
printf("CloudWatch messages: %u\n", stats.cloud_messages);
printf("CloudWatch failures: %u\n", stats.cloud_failures);
```

## Code Structure

The logging subsystem consists of these main files:

- **logger.h**: Public API declarations and documentation
- **logger.c**: Core implementation including task and destination handlers

## Future Enhancements

Potential improvements for future firmware versions:

1. **Remote Configuration**: Ability to adjust logging parameters via remote interface
2. **Log Filtering**: Advanced filtering based on module and content patterns
3. **Compression**: Log file compression to reduce storage requirements
4. **Encryption**: Encrypted log files for sensitive information
5. **Advanced Analytics**: On-device log analysis for predictive maintenance
6. **MQTT Support**: Additional destination for MQTT-based IoT platforms

## Technical Notes

1. **AWS Authentication**: The CloudWatch implementation requires proper AWS credentials
2. **SD Card Performance**: Log rotation may cause brief pauses; adjust parameters accordingly
3. **Memory Usage**: Each queued message requires memory until processed
4. **Task Priority**: The logger task runs at lower priority than critical control tasks
5. **Timestamp Accuracy**: Timestamps require correct system time configuration