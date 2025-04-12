# MQTT Communication Module

## Overview

The MQTT Communication Module is a critical component of the Battery Energy Storage System (BESS) firmware, responsible for establishing reliable communication with AWS IoT Core and other MQTT brokers. This module enables telemetry data transmission, remote control operations, system status reporting, and integration with AWS CloudWatch for logging.

## Features

- **Secure Connectivity**: Supports TLS/SSL with certificate-based authentication
- **Reliable Communication**: Implements QoS levels 0, 1, and 2 for message delivery guarantees
- **Automatic Reconnection**: Built-in reconnection mechanism with configurable retry intervals
- **Telemetry Publishing**: Standardized format for battery system metrics
- **Command Reception**: Processing of remote control commands
- **Alarm Publishing**: Severity-based alarm distribution
- **CloudWatch Integration**: Structured log format compatible with AWS CloudWatch
- **Thread Safety**: Complete thread safety for multi-task operation
- **Robust Error Handling**: Comprehensive error detection and reporting

## Architecture

The MQTT module follows a layered architecture:

1. **Transport Layer**: Handles the underlying TCP/TLS connection
2. **MQTT Protocol Layer**: Implements the MQTT 3.1.1 protocol
3. **Application Layer**: Provides domain-specific APIs for battery system operations

The module is built on top of the ESP-IDF MQTT client library, adding BESS-specific functionality, error handling, and thread safety.

## Topic Structure

The MQTT module uses a standardized topic structure:

| Topic Pattern | Description | QoS |
|---------------|-------------|-----|
| `bess/telemetry` | Real-time telemetry data | 1 |
| `bess/status/info` | Normal system status updates | 1 |
| `bess/status/error` | Error status notifications | 1 |
| `bess/status/connection` | Connection status with retain flag | 1 |
| `bess/alarms/info` | Informational alarms | 1 |
| `bess/alarms/warning` | Warning alarms | 1 |
| `bess/alarms/critical` | Critical alarms | 1 |
| `bess/logs/info` | Normal logs for CloudWatch | 0 |
| `bess/logs/warning` | Warning logs for CloudWatch | 0 |
| `bess/logs/error` | Error logs for CloudWatch | 0 |
| `bess/control/commands` | Incoming control commands | 1 |
| `bess/control/config` | Configuration updates | 1 |
| `bess/system/reset` | System reset commands | 2 |
| `bess/firmware/update` | Firmware update notifications | 2 |

## Message Formats

### Telemetry Data

```json
{
  "timestamp": 1713012345,
  "device_id": "bess_controller_001",
  "electrical": {
    "voltage": 48.5,
    "current": 10.2,
    "power_kw": 0.495
  },
  "battery": {
    "soc": 75.8,
    "soh": 98.2
  },
  "thermal": {
    "max_temp": 35.2,
    "min_temp": 28.7,
    "avg_temp": 31.4
  }
}
```

### Status Updates

```json
{
  "timestamp": 1713012400,
  "device_id": "bess_controller_001",
  "code": 1001,
  "message": "Cell balancing started",
  "is_error": false
}
```

### Alarms

```json
{
  "timestamp": 1713012500,
  "device_id": "bess_controller_001",
  "code": 3002,
  "message": "Module temperature high",
  "severity": 3,
  "module_id": 2
}
```

### CloudWatch Logs

```json
{
  "timestamp": 1713012600000,
  "device_id": "bess_controller_001",
  "level": 2,
  "component": "battery_manager",
  "message": "Cell voltage deviation detected in module 3"
}
```

### Control Commands

```json
{
  "command": "set_mode",
  "mode": "discharge",
  "params": {
    "power_limit_kw": 25,
    "soc_min": 20
  },
  "request_id": "req-12345"
}
```

## Usage Examples

### Initialization and Connection

```c
#include "mqtt_client.h"

// Create configuration with default values
bess_mqtt_config_t mqtt_config = BESS_MQTT_DEFAULT_CONFIG();

// Customize configuration
strcpy(mqtt_config.broker_uri, "mqtts://example-iot.amazonaws.com:8883");
strcpy(mqtt_config.client_id, "bess_controller_001");
mqtt_config.cert_file = "/spiffs/cert.pem";
mqtt_config.key_file = "/spiffs/key.pem";
mqtt_config.ca_file = "/spiffs/ca.pem";
mqtt_config.event_callback = mqtt_event_handler;
mqtt_config.reconnect_interval_ms = 10000;
mqtt_config.max_reconnect_attempts = 0;  // Infinite retry

// Initialize MQTT client
esp_err_t ret = bess_mqtt_init(&mqtt_config);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize MQTT client: %s", esp_err_to_name(ret));
    return;
}

// Connect to broker
ret = bess_mqtt_connect();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect to MQTT broker: %s", esp_err_to_name(ret));
    return;
}

// Register standard topics
ret = bess_mqtt_register_standard_topics();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register standard topics: %s", esp_err_to_name(ret));
}
```

### Event Handling

```c
// Event handler callback
void mqtt_event_handler(const bess_mqtt_event_data_t *event_data, void *user_context) {
    switch (event_data->event_id) {
        case BESS_MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker");
            // Initialize system after connection
            break;
            
        case BESS_MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from MQTT broker");
            // Handle disconnection
            break;
            
        case BESS_MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Received data on topic: %s", event_data->message.topic);
            // Process received data
            if (strcmp(event_data->message.topic, "bess/control/config") == 0) {
                // Handle configuration update
                process_config_update(event_data->message.data, event_data->message.data_len);
            }
            break;
            
        case BESS_MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error: %s", event_data->error.error_msg);
            break;
            
        default:
            break;
    }
}
```

### Publishing Telemetry Data

```c
void publish_telemetry_task(void *pvParameters) {
    while (1) {
        // Get current system status
        bess_system_status_t status;
        esp_err_t ret = battery_manager_get_system_status(&status);
        
        if (ret == ESP_OK) {
            // Get thermal data
            float max_temp, min_temp, avg_temp;
            thermal_monitor_get_temperatures(&max_temp, &min_temp, &avg_temp);
            
            // Publish telemetry
            ret = bess_mqtt_publish_telemetry(
                status.system_voltage,
                status.system_current,
                status.system_soc,
                status.system_soh,
                status.system_power,
                max_temp,
                min_temp,
                avg_temp
            );
            
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to publish telemetry: %s", esp_err_to_name(ret));
            }
        }
        
        // Publish every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
```

### Publishing Alarms

```c
// Example of publishing an alarm when a thermal issue is detected
void thermal_alert_handler(uint8_t module_id, float temperature) {
    if (temperature > TEMPERATURE_WARNING_THRESHOLD) {
        // Publish a warning alarm
        bess_mqtt_publish_alarm(
            3001,                                   // Alarm code
            "Module temperature above warning threshold",  // Message
            2,                                      // Severity (warning)
            module_id                               // Affected module
        );
    }
    
    if (temperature > TEMPERATURE_CRITICAL_THRESHOLD) {
        // Publish a critical alarm
        bess_mqtt_publish_alarm(
            3002,                                   // Alarm code
            "Module temperature above critical threshold", // Message
            4,                                      // Severity (critical)
            module_id                               // Affected module
        );
        
        // Take protective action
        emergency_thermal_shutdown(module_id);
    }
}
```

### CloudWatch Logging

```c
// Custom logging macro
#define BESS_LOG_CLOUDWATCH(level, component, format, ...) do { \
    char log_message[256]; \
    snprintf(log_message, sizeof(log_message), format, ##__VA_ARGS__); \
    esp_log_write(level, component, "%s", log_message); \
    if (bess_mqtt_is_connected()) { \
        bess_mqtt_publish_log(level, component, log_message); \
    } \
} while(0)

// Usage example
BESS_LOG_CLOUDWATCH(2, "cell_balancer", "Balancing completed for module %d in %d minutes", 
                   module_id, (int)(balancing_time / 60));
```

## Integration with AWS IoT Core and CloudWatch

### AWS IoT Core Setup

1. Create a Thing in AWS IoT Core for each BESS controller
2. Generate certificates for each Thing
3. Create policies to allow publishing and subscribing to relevant topics
4. Configure device certificates in the BESS firmware

### CloudWatch Integration

1. Create an AWS IoT Rule to forward logs to CloudWatch:

```sql
SELECT 
  timestamp as timestamp,
  device_id as device_id,
  level as level,
  component as component,
  message as message
FROM 'bess/logs/#'
```

2. Configure the rule action to send data to CloudWatch Logs
3. Set up CloudWatch Dashboards and Alarms based on the logs

## Error Handling

The MQTT module provides detailed error codes to help diagnose communication issues:

| Error Code | Description | Recommended Action |
|------------|-------------|-------------------|
| `BESS_MQTT_ERR_INIT_FAILED` | Initialization failed | Check memory and configuration |
| `BESS_MQTT_ERR_CONNECT_FAILED` | Connection to broker failed | Verify network connectivity and credentials |
| `BESS_MQTT_ERR_SUBSCRIBE_FAILED` | Subscription failed | Check topic permissions and broker connectivity |
| `BESS_MQTT_ERR_PUBLISH_FAILED` | Message publishing failed | Verify connectivity and QoS settings |
| `BESS_MQTT_ERR_DISCONNECTED` | Client disconnected | Reconnect or wait for automatic reconnection |
| `BESS_MQTT_ERR_TIMEOUT` | Operation timed out | Adjust timeout or check network latency |
| `BESS_MQTT_ERR_AUTHENTICATION_FAILED` | Authentication failed | Verify certificates and credentials |
| `BESS_MQTT_ERR_TLS_FAILED` | TLS/SSL error | Check certificate validity and TLS configuration |

## Resource Requirements

The MQTT module has the following resource requirements:

- **RAM**: Approximately 16KB for client context and buffers
- **Task Stack**: 4KB for reconnect task
- **Storage**: Certificate storage (typically 2-4KB per certificate)
- **Network**: Regular small data packets, peaks during telemetry transmission
- **CPU**: Minimal when idle, moderate during publish/receive operations

## Thread Safety and Performance Considerations

The MQTT client is designed for thread safety:

- All public APIs are protected with mutexes for concurrent access
- Long-running operations have configurable timeouts
- The reconnect task runs at a lower priority to avoid interfering with critical tasks
- Message buffer size is configurable to balance memory usage and performance

Performance considerations:

- Use appropriate QoS levels:
  - QoS 0 for frequent, non-critical telemetry
  - QoS 1 for important status updates
  - QoS 2 for critical commands and firmware updates
- Consider bandwidth limitations when setting telemetry publishing frequency
- Be aware of certificate expiration and renewal requirements

## Security Best Practices

1. **Use TLS for all connections**: Never use unencrypted MQTT in production
2. **Rotate certificates**: Implement certificate rotation policies
3. **Restrictive policies**: Use least-privilege IoT Core policies
4. **Unique client IDs**: Ensure each device has a unique client ID
5. **Protect credentials**: Store certificates securely using ESP32's secure storage if possible
6. **Enable LWT**: Always enable Last Will and Testament for unexpected disconnections
7. **Topic authorization**: Implement topic-based access control in AWS IoT Core

## Future Enhancements

- MQTT 5.0 protocol support
- Message persistence during disconnections
- Enhanced message queuing with priority
- Support for alternative cloud providers (Azure IoT, Google Cloud IoT)
- Binary payload optimization for telemetry data
- Remote firmware update via MQTT