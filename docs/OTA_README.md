# OTA Manager Module

## Overview

The OTA (Over-The-Air) Manager is a critical component of the BESS firmware management subsystem, responsible for secure and reliable firmware updates for the 100KW/200KWH Battery Energy Storage System. It enables remote firmware updates through multiple interfaces while ensuring system safety and reliability.

## Key Features

### Multiple Update Sources

The OTA Manager supports firmware updates from various sources:

- **HTTPS Server**: Secure downloads with TLS encryption and certificate validation
- **AWS IoT**: Integration with AWS IoT Core for cloud-managed updates
- **Modbus**: Updates through Modbus industrial protocol
- **CANBus**: Updates via CAN network interface
- **SD Card**: Local updates from SD card files
- **USB Drive**: Local updates from USB storage devices

### Security Features

- **Signature Verification**: Cryptographic validation of firmware authenticity
- **Hash Verification**: MD5 and SHA256 hash verification to ensure integrity
- **TLS Encryption**: Secure communication for remote updates
- **Authentication**: Basic auth and other authentication mechanisms

### Safety Integration

The OTA Manager integrates with the Battery Management System (BMS) to ensure updates only occur under safe conditions:

- **Battery SoC Checks**: Ensures sufficient battery level before updating
- **Temperature Monitoring**: Prevents updates during unsafe temperature conditions
- **Current Monitoring**: Ensures updates only occur during appropriate load conditions
- **Custom Safety Callback**: Extensible safety checks via callback mechanism

### Reliability Features

- **Automatic Rollback**: Recovery from failed updates
- **Update Validation**: Post-update validation with timeout-based verification
- **Atomic Updates**: Ensures updates are applied completely or not at all
- **Multi-stage Process**: Downloading, verification, and application stages with appropriate checkpoints

### Comprehensive Logging

The OTA Manager implements a robust logging system with multiple outputs:

- **Console Logging**: Real-time logs to serial console
- **SD Card Logging**: Persistent logs saved to SD card
- **AWS CloudWatch**: Remote logging to AWS CloudWatch service

## Architecture

The OTA Manager is implemented with a task-based architecture on FreeRTOS, providing non-blocking operation and integration with the real-time system.

### Components

1. **OTA Task**: Main OTA operations run in a dedicated FreeRTOS task
2. **Command Queue**: Thread-safe mechanism for update commands
3. **Event System**: Status notifications and callback mechanism
4. **Timer Service**: Automatic update checking at configurable intervals

### State Machine

The OTA update process follows a well-defined state machine:

- `IDLE`: No update activity
- `CHECKING`: Checking for available updates
- `UPDATE_AVAILABLE`: Update found but not yet started
- `DOWNLOADING`: Update is being downloaded
- `VERIFYING`: Update integrity is being verified
- `READY_TO_APPLY`: Update is ready to be applied
- `APPLYING`: Update is being applied
- `COMPLETE`: Update successfully applied
- `REBOOTING`: System is rebooting to new firmware
- `FAILED`: Update process failed
- `ABORTED`: Update was manually aborted

## Integration with BESS

The OTA Manager is designed specifically for the BESS 100KW/200KWH system with LFP battery modules (48V, 16KWH), ensuring all updates maintain system safety and reliability.

### ESP32-P4 Integration

- Optimized for ESP32-P4 MCU architecture
- Uses ESP-IDF OTA partition scheme
- Implements ESP-IDF logging system
- Leverages hardware acceleration for cryptographic operations

### FreeRTOS Integration

- Task scheduling for non-blocking operations
- Mutex protection for thread safety
- Event groups for state signaling
- Timers for periodic operations

## Usage Examples

### Basic Initialization

```c
/* Configure OTA manager for HTTPS updates */
ota_manager_config_t config = {
    .source = OTA_SOURCE_HTTPS,
    .config.https = {
        .url = "https://firmware.example.com/bess/latest.bin",
        .cert_pem = server_cert_pem,
        .basic_auth_user = "bess_device",
        .basic_auth_pass = "password123"
    },
    .check_interval_ms = 24 * 60 * 60 * 1000,  /* Daily checks */
    .automatic_update = false,
    .verify_signature = true,
    .public_key_pem = update_public_key_pem,
    .task_priority = 5,
    .task_stack_size = 8192,
    .core_id = 0
};

/* Initialize OTA manager */
esp_err_t ret = ota_manager_init(&config);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize OTA manager: %s", esp_err_to_name(ret));
    return;
}

/* Start OTA task */
ota_manager_start();
```

### Setting BMS Safety Thresholds

```c
/* Set safety thresholds for OTA operations */
ota_manager_set_bms_safety_thresholds(
    30,       /* Minimum 30% SoC */
    45.0f,    /* Maximum 45°C temperature */
    0.0f,     /* Minimum 0°C temperature */
    10.0f     /* Maximum 10A current */
);
```

### Registering for Event Notifications

```c
/* Event callback function */
void ota_event_handler(ota_status_t status, ota_error_t error, void *user_data) {
    switch (status) {
        case OTA_STATUS_UPDATE_AVAILABLE:
            ESP_LOGI(TAG, "New update available!");
            break;
            
        case OTA_STATUS_COMPLETE:
            ESP_LOGI(TAG, "Update complete, rebooting...");
            break;
            
        case OTA_STATUS_FAILED:
            ESP_LOGW(TAG, "Update failed: %s", ota_manager_error_to_string(error));
            break;
            
        /* Handle other states... */
    }
}

/* Register for events */
config.event_cb = ota_event_handler;
config.event_cb_data = NULL;
```

### Progress Tracking

```c
/* Progress callback function */
void ota_progress_handler(size_t received, size_t total, void *user_data) {
    int percentage = (total > 0) ? (received * 100 / total) : 0;
    ESP_LOGI(TAG, "Download progress: %d%% (%u/%u bytes)", percentage, received, total);
    
    /* Update LED or display to show progress */
    update_progress_indicator(percentage);
}

/* Register progress callback */
config.progress_cb = ota_progress_handler;
config.progress_cb_data = NULL;
```

### Manual Update Check

```c
/* Check for updates manually */
esp_err_t ret = ota_manager_check_for_update();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to check for updates: %s", esp_err_to_name(ret));
}
```

### SD Card Updates

```c
/* Configure for SD card updates */
ota_manager_config_t sd_config = {
    .source = OTA_SOURCE_SD_CARD,
    .config.sd_card = {
        .filepath = "/sdcard/bess_update.bin"
    },
    .verify_signature = true,
    .public_key_pem = update_public_key_pem
};

ota_manager_init(&sd_config);
ota_manager_start();

/* Check for update file on SD card */
ota_manager_check_for_update();
```

## Safety Considerations

The OTA Manager implements multiple safety features to ensure the BESS system remains operational during and after updates:

1. **Pre-update Safety Checks**: Validates battery conditions before starting updates
2. **Redundant Partitions**: Maintains previous firmware for rollback
3. **Integrity Verification**: Ensures firmware is not corrupted during transmission
4. **Signature Verification**: Prevents installation of unauthorized firmware
5. **Automatic Rollback**: Returns to known-good firmware if update fails

## Configuration Parameters

Key configuration parameters are defined in `ota_manager.h`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `check_interval_ms` | Interval for automatic update checks | 0 (disabled) |
| `automatic_update` | Apply updates automatically | false |
| `verify_signature` | Verify firmware signature | true |
| `allow_rollback` | Enable automatic rollback | true |
| `rollback_timeout_ms` | Timeout for rollback verification | 300000 (5 min) |
| `min_soc` | Minimum battery SoC for updates | 30% |
| `max_temperature` | Maximum battery temperature | 45°C |
| `min_temperature` | Minimum battery temperature | 0°C |
| `max_current` | Maximum battery current | 10A |

## Error Handling

The OTA Manager provides comprehensive error codes and handling:

| Error Code | Description |
|------------|-------------|
| `OTA_ERR_NONE` | No error |
| `OTA_ERR_NO_UPDATES` | No updates available |
| `OTA_ERR_CONNECTIVITY` | Cannot connect to update server |
| `OTA_ERR_SERVER` | Server error |
| `OTA_ERR_AUTHENTICATION` | Authentication failed |
| `OTA_ERR_INSUFFICIENT_SPACE` | Not enough space for update |
| `OTA_ERR_VALIDATION_FAILED` | Signature/checksum validation failed |
| `OTA_ERR_FLASH_WRITE` | Error writing to flash |
| `OTA_ERR_INCOMPATIBLE_VERSION` | Incompatible firmware version |
| `OTA_ERR_BATTERY_LOW` | Battery too low for update |
| `OTA_ERR_BMS_UNSAFE` | BMS reports unsafe condition |
| `OTA_ERR_INTERNAL` | Internal error |

## Future Enhancements

Planned future enhancements for the OTA Manager include:

1. **Delta Updates**: Support for downloading only changed parts of firmware
2. **Multi-Component Updates**: Support for updating different system components independently
3. **Scheduled Updates**: Time-based update scheduling
4. **Enhanced Security**: Additional security mechanisms including device attestation
5. **Update Metrics**: Detailed statistics and performance metrics for update operations
6. **Remote Configuration**: Remote configuration of OTA parameters

## Dependencies

The OTA Manager depends on the following components:

- ESP-IDF OTA APIs
- FreeRTOS kernel
- mbedTLS for cryptographic operations
- ESP HTTP client
- AWS IoT SDK (optional, for AWS IoT integration)
- SD card driver
- USB driver
