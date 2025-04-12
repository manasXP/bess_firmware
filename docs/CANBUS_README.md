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

## Implementation Source Code

Here's the core implementation of the CANbus interface:

```c
/**
 * @file canbus_interface.c
 * @brief CANbus communication interface implementation for BESS
 *
 * Implements the CANbus communication interface for the Battery Energy
 * Storage System (BESS) 100KW/200KWH using ESP32-P4 and FreeRTOS.
 */

#include "canbus_interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "bess_types.h"
#include <string.h>

#define TAG "CANBUS"

#define MAX_CALLBACKS_PER_EVENT   5
#define MAX_MESSAGE_CALLBACKS     10
#define CANBUS_TASK_STACK_SIZE    4096
#define CANBUS_TASK_PRIORITY      10
#define CANBUS_QUEUE_SIZE         32
#define CANBUS_TX_GPIO            4  // Example GPIO pin - modify based on hardware
#define CANBUS_RX_GPIO            5  // Example GPIO pin - modify based on hardware

/**
 * @brief CANbus message callback registration entry
 */
typedef struct {
    uint32_t id;                      /**< Message ID */
    uint32_t mask;                    /**< ID mask for filtering */
    canbus_message_callback_t callback; /**< Callback function */
    void *user_data;                  /**< User data passed to callback */
    bool active;                      /**< Whether this entry is active */
} canbus_message_callback_entry_t;

/**
 * @brief CANbus event callback registration entry
 */
typedef struct {
    canbus_event_callback_t callback; /**< Callback function */
    void *user_data;                  /**< User data passed to callback */
    bool active;                      /**< Whether this entry is active */
} canbus_event_callback_entry_t;

/**
 * @brief CANbus interface state
 */
typedef struct {
    bool initialized;                 /**< Whether the interface is initialized */
    bool running;                     /**< Whether the interface is running */
    TaskHandle_t rx_task_handle;      /**< Handle for the receive task */
    TaskHandle_t tx_task_handle;      /**< Handle for the transmit task */
    QueueHandle_t tx_queue;           /**< Queue for outgoing messages */
    SemaphoreHandle_t callback_mutex; /**< Mutex protecting callback registrations */
    SemaphoreHandle_t status_mutex;   /**< Mutex protecting interface status */
    canbus_message_callback_entry_t message_callbacks[MAX_MESSAGE_CALLBACKS]; /**< Message callback registrations */
    canbus_event_callback_entry_t event_callbacks[CANBUS_EVENT_BUS_RECOVERED + 1][MAX_CALLBACKS_PER_EVENT]; /**< Event callback registrations */
    uint32_t bitrate;                 /**< Current bitrate in bits per second */
} canbus_state_t;

/**
 * CAN message structure for TX queue
 */
typedef struct {
    twai_message_t message;           /**< CAN message */
} canbus_tx_item_t;

/* Static module state */
static canbus_state_t s_canbus_state = {
    .initialized = false,
    .running = false,
    .rx_task_handle = NULL,
    .tx_task_handle = NULL,
    .tx_queue = NULL,
    .callback_mutex = NULL,
    .status_mutex = NULL,
    .bitrate = 500000,                /** Default to 500 kbps */
};

/* Forward declarations for internal functions */
static void canbus_rx_task(void *arg);
static void canbus_tx_task(void *arg);
static void handle_canbus_event(canbus_event_type_t event);
static esp_err_t process_received_message(twai_message_t *message);

/**
 * @brief Initialize the CANbus interface
 */
esp_err_t canbus_interface_init(void) {
    ESP_LOGI(TAG, "Initializing CANbus interface");
    
    if (s_canbus_state.initialized) {
        ESP_LOGW(TAG, "CANbus interface already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Create synchronization primitives */
    s_canbus_state.callback_mutex = xSemaphoreCreateMutex();
    if (s_canbus_state.callback_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create callback mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_canbus_state.status_mutex = xSemaphoreCreateMutex();
    if (s_canbus_state.status_mutex == NULL) {
        vSemaphoreDelete(s_canbus_state.callback_mutex);
        ESP_LOGE(TAG, "Failed to create status mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_canbus_state.tx_queue = xQueueCreate(CANBUS_QUEUE_SIZE, sizeof(canbus_tx_item_t));
    if (s_canbus_state.tx_queue == NULL) {
        vSemaphoreDelete(s_canbus_state.callback_mutex);
        vSemaphoreDelete(s_canbus_state.status_mutex);
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }
    
    /* Initialize callback arrays */
    memset(s_canbus_state.message_callbacks, 0, sizeof(s_canbus_state.message_callbacks));
    memset(s_canbus_state.event_callbacks, 0, sizeof(s_canbus_state.event_callbacks));
    
    /* Configure TWAI (CAN) driver */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CANBUS_TX_GPIO, CANBUS_RX_GPIO, TWAI_MODE_NORMAL);
    
    /* Select timing based on bitrate */
    twai_timing_config_t t_config;
    if (s_canbus_state.bitrate == 1000000) {
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    } else if (s_canbus_state.bitrate == 800000) {
        t_config = TWAI_TIMING_CONFIG_800KBITS();
    } else if (s_canbus_state.bitrate == 250000) {
        t_config = TWAI_TIMING_CONFIG_250KBITS();
    } else if (s_canbus_state.bitrate == 125000) {
        t_config = TWAI_TIMING_CONFIG_125KBITS();
    } else if (s_canbus_state.bitrate == 100000) {
        t_config = TWAI_TIMING_CONFIG_100KBITS();
    } else {
        /* Default to 500kbps */
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    }
    
    /* Use acceptance filter to accept all messages initially */
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    /* Install TWAI driver */
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        vQueueDelete(s_canbus_state.tx_queue);
        vSemaphoreDelete(s_canbus_state.callback_mutex);
        vSemaphoreDelete(s_canbus_state.status_mutex);
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    s_canbus_state.initialized = true;
    ESP_LOGI(TAG, "CANbus interface initialized");
    return ESP_OK;
}

/**
 * @brief Start the CANbus interface
 */
esp_err_t canbus_interface_start(void) {
    ESP_LOGI(TAG, "Starting CANbus interface");
    
    if (!s_canbus_state.initialized) {
        ESP_LOGE(TAG, "CANbus interface not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_canbus_state.running) {
        ESP_LOGW(TAG, "CANbus interface already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Start TWAI driver */
    esp_err_t ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }
    