/**
 * @file comm_manager.c
 * @brief Implementation of the Communication Manager for BESS
 *
 * Provides central management of all communication interfaces
 * including Modbus, CANbus, and network communications.
 */

 #include "comm_manager.h"
 #include "esp_log.h"
 #include "esp_system.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "esp_err.h"
 #include "driver/uart.h"
 #include "driver/gpio.h"
 #include "driver/twai.h" // ESP32 CAN driver
 
 // Logging tag
 static const char *TAG = "COMM_MANAGER";
 
 // Task handles
 static TaskHandle_t s_modbus_rtu_task_handle = NULL;
 static TaskHandle_t s_modbus_tcp_task_handle = NULL;
 static TaskHandle_t s_canbus_task_handle = NULL;
 static TaskHandle_t s_mqtt_task_handle = NULL;
 
 // Mutexes
 static SemaphoreHandle_t s_comm_mutex = NULL;
 
 // Event groups
 static EventGroupHandle_t s_comm_event_group = NULL;
 
 // Event bits
 #define COMM_EVENT_MODBUS_RTU_DATA    (1 << 0)
 #define COMM_EVENT_MODBUS_TCP_DATA    (1 << 1)
 #define COMM_EVENT_CANBUS_DATA        (1 << 2)
 #define COMM_EVENT_MQTT_DATA          (1 << 3)
 #define COMM_EVENT_HTTP_DATA          (1 << 4)
 #define COMM_EVENT_BLE_DATA           (1 << 5)
 
 // Structure to hold interface callbacks
 typedef struct {
     bess_event_callback_t callback;
     void *user_data;
 } comm_callback_t;
 
 // Communication interface data
 typedef struct {
     bool initialized;
     bool enabled;
     bool connected;
     uint32_t tx_count;
     uint32_t rx_count;
     uint32_t error_count;
     uint32_t last_tx_time;
     uint32_t last_rx_time;
     uint32_t bytes_tx;
     uint32_t bytes_rx;
     // Interface specific configurations could be added here
     comm_callback_t callbacks[5]; // Support up to 5 callbacks per interface
     uint8_t callback_count;
 } comm_interface_data_t;
 
 // Array to store data for all interfaces
 static comm_interface_data_t s_interfaces[COMM_INTERFACE_COUNT];
 
 // Forward declarations for interface-specific functions
 static void modbus_rtu_task(void *arg);
 static void modbus_tcp_task(void *arg);
 static void canbus_task(void *arg);
 static void mqtt_task(void *arg);
 
 // Helper function to notify callbacks
 static void notify_callbacks(comm_interface_type_t interface_type, uint32_t event_data) {
     if (interface_type >= COMM_INTERFACE_COUNT) {
         return;
     }
 
     // Take mutex to ensure thread safety
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         comm_interface_data_t *interface = &s_interfaces[interface_type];
         
         // Call all registered callbacks
         for (uint8_t i = 0; i < interface->callback_count; i++) {
             if (interface->callbacks[i].callback != NULL) {
                 interface->callbacks[i].callback(event_data, interface->callbacks[i].user_data);
             }
         }
         
         xSemaphoreGive(s_comm_mutex);
     }
 }
 
 esp_err_t comm_manager_init(void) {
     ESP_LOGI(TAG, "Initializing communication manager");
     
     // Create mutex for thread safety
     s_comm_mutex = xSemaphoreCreateMutex();
     if (s_comm_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create communication mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Create event group for signaling between tasks
     s_comm_event_group = xEventGroupCreate();
     if (s_comm_event_group == NULL) {
         vSemaphoreDelete(s_comm_mutex);
         ESP_LOGE(TAG, "Failed to create communication event group");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize all interfaces
     memset(s_interfaces, 0, sizeof(s_interfaces));
     
     // Initialize Modbus RTU interface
     s_interfaces[COMM_INTERFACE_MODBUS_RTU].initialized = true;
     
     // Initialize Modbus TCP interface
     s_interfaces[COMM_INTERFACE_MODBUS_TCP].initialized = true;
     
     // Initialize CANbus interface
     s_interfaces[COMM_INTERFACE_CANBUS].initialized = true;
     
     // Initialize MQTT interface
     s_interfaces[COMM_INTERFACE_MQTT].initialized = true;
     
     ESP_LOGI(TAG, "Communication manager initialized successfully");
     return ESP_OK;
 }
 
 esp_err_t comm_manager_start(void) {
     ESP_LOGI(TAG, "Starting communication manager tasks");
     
     // Start Modbus RTU task if enabled
     if (s_interfaces[COMM_INTERFACE_MODBUS_RTU].enabled) {
         BaseType_t ret = xTaskCreatePinnedToCore(
             modbus_rtu_task,
             "modbus_rtu_task",
             4096,
             NULL,
             5,
             &s_modbus_rtu_task_handle,
             1
         );
         
         if (ret != pdPASS) {
             ESP_LOGE(TAG, "Failed to create Modbus RTU task");
             return ESP_FAIL;
         }
     }
     
     // Start Modbus TCP task if enabled
     if (s_interfaces[COMM_INTERFACE_MODBUS_TCP].enabled) {
         BaseType_t ret = xTaskCreatePinnedToCore(
             modbus_tcp_task,
             "modbus_tcp_task",
             4096,
             NULL,
             5,
             &s_modbus_tcp_task_handle,
             1
         );
         
         if (ret != pdPASS) {
             ESP_LOGE(TAG, "Failed to create Modbus TCP task");
             return ESP_FAIL;
         }
     }
     
     // Start CANbus task if enabled
     if (s_interfaces[COMM_INTERFACE_CANBUS].enabled) {
         BaseType_t ret = xTaskCreatePinnedToCore(
             canbus_task,
             "canbus_task",
             4096,
             NULL,
             5,
             &s_canbus_task_handle,
             1
         );
         
         if (ret != pdPASS) {
             ESP_LOGE(TAG, "Failed to create CANbus task");
             return ESP_FAIL;
         }
     }
     
     // Start MQTT task if enabled
     if (s_interfaces[COMM_INTERFACE_MQTT].enabled) {
         BaseType_t ret = xTaskCreatePinnedToCore(
             mqtt_task,
             "mqtt_task",
             4096,
             NULL,
             5,
             &s_mqtt_task_handle,
             1
         );
         
         if (ret != pdPASS) {
             ESP_LOGE(TAG, "Failed to create MQTT task");
             return ESP_FAIL;
         }
     }
     
     ESP_LOGI(TAG, "Communication manager tasks started successfully");
     return ESP_OK;
 }
 
 esp_err_t comm_manager_get_interface_status(comm_interface_type_t interface_type, 
                                            comm_interface_status_t *status) {
     if (interface_type >= COMM_INTERFACE_COUNT || status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         comm_interface_data_t *interface = &s_interfaces[interface_type];
         status->enabled = interface->enabled;
         status->connected = interface->connected;
         status->tx_count = interface->tx_count;
         status->rx_count = interface->rx_count;
         status->error_count = interface->error_count;
         status->last_tx_time = interface->last_tx_time;
         status->last_rx_time = interface->last_rx_time;
         status->bytes_tx = interface->bytes_tx;
         status->bytes_rx = interface->bytes_rx;
         
         xSemaphoreGive(s_comm_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_enable_interface(comm_interface_type_t interface_type) {
     if (interface_type >= COMM_INTERFACE_COUNT) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         if (!s_interfaces[interface_type].initialized) {
             xSemaphoreGive(s_comm_mutex);
             ESP_LOGW(TAG, "Interface %d not initialized", interface_type);
             return ESP_ERR_NOT_FOUND;
         }
         
         s_interfaces[interface_type].enabled = true;
         xSemaphoreGive(s_comm_mutex);
         
         ESP_LOGI(TAG, "Interface %d enabled", interface_type);
         
         // If already started, start the appropriate task
         switch (interface_type) {
             case COMM_INTERFACE_MODBUS_RTU:
                 if (s_modbus_rtu_task_handle == NULL) {
                     BaseType_t ret = xTaskCreatePinnedToCore(
                         modbus_rtu_task,
                         "modbus_rtu_task",
                         4096,
                         NULL,
                         5,
                         &s_modbus_rtu_task_handle,
                         1
                     );
                     
                     if (ret != pdPASS) {
                         ESP_LOGE(TAG, "Failed to create Modbus RTU task");
                         return ESP_FAIL;
                     }
                 }
                 break;
                 
             case COMM_INTERFACE_MODBUS_TCP:
                 if (s_modbus_tcp_task_handle == NULL) {
                     BaseType_t ret = xTaskCreatePinnedToCore(
                         modbus_tcp_task,
                         "modbus_tcp_task",
                         4096,
                         NULL,
                         5,
                         &s_modbus_tcp_task_handle,
                         1
                     );
                     
                     if (ret != pdPASS) {
                         ESP_LOGE(TAG, "Failed to create Modbus TCP task");
                         return ESP_FAIL;
                     }
                 }
                 break;
                 
             case COMM_INTERFACE_CANBUS:
                 if (s_canbus_task_handle == NULL) {
                     BaseType_t ret = xTaskCreatePinnedToCore(
                         canbus_task,
                         "canbus_task",
                         4096,
                         NULL,
                         5,
                         &s_canbus_task_handle,
                         1
                     );
                     
                     if (ret != pdPASS) {
                         ESP_LOGE(TAG, "Failed to create CANbus task");
                         return ESP_FAIL;
                     }
                 }
                 break;
                 
             case COMM_INTERFACE_MQTT:
                 if (s_mqtt_task_handle == NULL) {
                     BaseType_t ret = xTaskCreatePinnedToCore(
                         mqtt_task,
                         "mqtt_task",
                         4096,
                         NULL,
                         5,
                         &s_mqtt_task_handle,
                         1
                     );
                     
                     if (ret != pdPASS) {
                         ESP_LOGE(TAG, "Failed to create MQTT task");
                         return ESP_FAIL;
                     }
                 }
                 break;
                 
             default:
                 // Other interfaces not implemented yet
                 break;
         }
         
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_disable_interface(comm_interface_type_t interface_type) {
     if (interface_type >= COMM_INTERFACE_COUNT) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         if (!s_interfaces[interface_type].initialized) {
             xSemaphoreGive(s_comm_mutex);
             ESP_LOGW(TAG, "Interface %d not initialized", interface_type);
             return ESP_ERR_NOT_FOUND;
         }
         
         s_interfaces[interface_type].enabled = false;
         xSemaphoreGive(s_comm_mutex);
         
         // Stop the appropriate task
         switch (interface_type) {
             case COMM_INTERFACE_MODBUS_RTU:
                 if (s_modbus_rtu_task_handle != NULL) {
                     vTaskDelete(s_modbus_rtu_task_handle);
                     s_modbus_rtu_task_handle = NULL;
                 }
                 break;
                 
             case COMM_INTERFACE_MODBUS_TCP:
                 if (s_modbus_tcp_task_handle != NULL) {
                     vTaskDelete(s_modbus_tcp_task_handle);
                     s_modbus_tcp_task_handle = NULL;
                 }
                 break;
                 
             case COMM_INTERFACE_CANBUS:
                 if (s_canbus_task_handle != NULL) {
                     vTaskDelete(s_canbus_task_handle);
                     s_canbus_task_handle = NULL;
                 }
                 break;
                 
             case COMM_INTERFACE_MQTT:
                 if (s_mqtt_task_handle != NULL) {
                     vTaskDelete(s_mqtt_task_handle);
                     s_mqtt_task_handle = NULL;
                 }
                 break;
                 
             default:
                 // Other interfaces not implemented yet
                 break;
         }
         
         ESP_LOGI(TAG, "Interface %d disabled", interface_type);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_register_callback(comm_interface_type_t interface_type,
                                          bess_event_callback_t callback,
                                          void *user_data) {
     if (interface_type >= COMM_INTERFACE_COUNT || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         comm_interface_data_t *interface = &s_interfaces[interface_type];
         
         // Check if we have space for another callback
         if (interface->callback_count >= 5) {
             xSemaphoreGive(s_comm_mutex);
             ESP_LOGW(TAG, "Maximum callbacks reached for interface %d", interface_type);
             return ESP_ERR_NO_MEM;
         }
         
         // Add the callback
         interface->callbacks[interface->callback_count].callback = callback;
         interface->callbacks[interface->callback_count].user_data = user_data;
         interface->callback_count++;
         
         xSemaphoreGive(s_comm_mutex);
         ESP_LOGI(TAG, "Callback registered for interface %d", interface_type);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_unregister_callback(comm_interface_type_t interface_type,
                                           bess_event_callback_t callback) {
     if (interface_type >= COMM_INTERFACE_COUNT || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         comm_interface_data_t *interface = &s_interfaces[interface_type];
         bool found = false;
         
         // Find and remove the callback
         for (uint8_t i = 0; i < interface->callback_count; i++) {
             if (interface->callbacks[i].callback == callback) {
                 // Shift remaining callbacks down
                 for (uint8_t j = i; j < interface->callback_count - 1; j++) {
                     interface->callbacks[j] = interface->callbacks[j + 1];
                 }
                 interface->callback_count--;
                 found = true;
                 break;
             }
         }
         
         xSemaphoreGive(s_comm_mutex);
         
         if (!found) {
             ESP_LOGW(TAG, "Callback not found for interface %d", interface_type);
             return ESP_ERR_NOT_FOUND;
         }
         
         ESP_LOGI(TAG, "Callback unregistered for interface %d", interface_type);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_process_data(void) {
     // This function will be called periodically to process data from all interfaces
     // It checks the event group to see which interfaces have data
     
     EventBits_t bits = xEventGroupGetBits(s_comm_event_group);
     
     if (bits & COMM_EVENT_MODBUS_RTU_DATA) {
         // Process Modbus RTU data
         xEventGroupClearBits(s_comm_event_group, COMM_EVENT_MODBUS_RTU_DATA);
     }
     
     if (bits & COMM_EVENT_MODBUS_TCP_DATA) {
         // Process Modbus TCP data
         xEventGroupClearBits(s_comm_event_group, COMM_EVENT_MODBUS_TCP_DATA);
     }
     
     if (bits & COMM_EVENT_CANBUS_DATA) {
         // Process CANbus data
         xEventGroupClearBits(s_comm_event_group, COMM_EVENT_CANBUS_DATA);
     }
     
     if (bits & COMM_EVENT_MQTT_DATA) {
         // Process MQTT data
         xEventGroupClearBits(s_comm_event_group, COMM_EVENT_MQTT_DATA);
     }
     
     return ESP_OK;
 }
 
 esp_err_t comm_manager_get_statistics(comm_interface_status_t *stats, size_t *count) {
     if (stats == NULL || count == NULL || *count == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         size_t num_interfaces = COMM_INTERFACE_COUNT;
         if (*count < num_interfaces) {
             num_interfaces = *count;
         }
         
         for (size_t i = 0; i < num_interfaces; i++) {
             stats[i].enabled = s_interfaces[i].enabled;
             stats[i].connected = s_interfaces[i].connected;
             stats[i].tx_count = s_interfaces[i].tx_count;
             stats[i].rx_count = s_interfaces[i].rx_count;
             stats[i].error_count = s_interfaces[i].error_count;
             stats[i].last_tx_time = s_interfaces[i].last_tx_time;
             stats[i].last_rx_time = s_interfaces[i].last_rx_time;
             stats[i].bytes_tx = s_interfaces[i].bytes_tx;
             stats[i].bytes_rx = s_interfaces[i].bytes_rx;
         }
         
         *count = num_interfaces;
         xSemaphoreGive(s_comm_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_reset_statistics(comm_interface_type_t interface_type) {
     if (interface_type >= COMM_INTERFACE_COUNT) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         s_interfaces[interface_type].tx_count = 0;
         s_interfaces[interface_type].rx_count = 0;
         s_interfaces[interface_type].error_count = 0;
         s_interfaces[interface_type].bytes_tx = 0;
         s_interfaces[interface_type].bytes_rx = 0;
         
         xSemaphoreGive(s_comm_mutex);
         ESP_LOGI(TAG, "Statistics reset for interface %d", interface_type);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_is_interface_enabled(comm_interface_type_t interface_type, bool *enabled) {
     if (interface_type >= COMM_INTERFACE_COUNT || enabled == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         *enabled = s_interfaces[interface_type].enabled;
         xSemaphoreGive(s_comm_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 esp_err_t comm_manager_is_interface_connected(comm_interface_type_t interface_type, bool *connected) {
     if (interface_type >= COMM_INTERFACE_COUNT || connected == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         *connected = s_interfaces[interface_type].connected;
         xSemaphoreGive(s_comm_mutex);
         return ESP_OK;
     }
     
     return ESP_ERR_TIMEOUT;
 }
 
 /*
  * Interface-specific task implementations
  */
 
 static void modbus_rtu_task(void *arg) {
    ESP_LOGI(TAG, "Modbus RTU task started");
    
    // UART configuration for Modbus RTU
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };
    
    // Configure UART1 (example) for Modbus RTU
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 17, 16, -1, -1)); // TX: GPIO17, RX: GPIO16
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0));
    
    // Update connection status
    if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_interfaces[COMM_INTERFACE_MODBUS_RTU].connected = true;
        xSemaphoreGive(s_comm_mutex);
    }
    
    uint8_t rx_buffer[256];
    
    // Main task loop
    while (1) {
        // Check if interface is still enabled
        bool enabled = false;
        esp_err_t err = comm_manager_is_interface_enabled(COMM_INTERFACE_MODBUS_RTU, &enabled);
        
        if (err != ESP_OK || !enabled) {
            ESP_LOGI(TAG, "Modbus RTU interface disabled, stopping task");
            
            // Update connection status before exiting
            if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_interfaces[COMM_INTERFACE_MODBUS_RTU].connected = false;
                xSemaphoreGive(s_comm_mutex);
            }
            
            // Clean up and exit task
            uart_driver_delete(UART_NUM_1);
            s_modbus_rtu_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
        
        // Read data from UART
        int len = uart_read_bytes(UART_NUM_1, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes from Modbus RTU", len);
            
            // Update statistics
            if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_interfaces[COMM_INTERFACE_MODBUS_RTU].rx_count++;
                s_interfaces[COMM_INTERFACE_MODBUS_RTU].last_rx_time = esp_timer_get_time() / 1000;
                s_interfaces[COMM_INTERFACE_MODBUS_RTU].bytes_rx += len;
                xSemaphoreGive(s_comm_mutex);
            }
            
            // Set event bit to signal data available
            xEventGroupSetBits(s_comm_event_group, COMM_EVENT_MODBUS_RTU_DATA);
            
            // Notify callbacks
            notify_callbacks(COMM_INTERFACE_MODBUS_RTU, len);
        }
        
        // Simulate occasional transmission
        if (esp_random() % 20 == 0) {
            uint8_t tx_buffer[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // Example Modbus query
            int bytes_sent = uart_write_bytes(UART_NUM_1, (const char*)tx_buffer, sizeof(tx_buffer));
            
            if (bytes_sent > 0) {
                // Update statistics
                if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    s_interfaces[COMM_INTERFACE_MODBUS_RTU].tx_count++;
                    s_interfaces[COMM_INTERFACE_MODBUS_RTU].last_tx_time = esp_timer_get_time() / 1000;
                    s_interfaces[COMM_INTERFACE_MODBUS_RTU].bytes_tx += bytes_sent;
                    xSemaphoreGive(s_comm_mutex);
                }
            } else {
                // Update error statistics
                if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    s_interfaces[COMM_INTERFACE_MODBUS_RTU].error_count++;
                    xSemaphoreGive(s_comm_mutex);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent CPU hogging
    }
}

 static void mqtt_task(void *arg) {
     ESP_LOGI(TAG, "MQTT task started");
     
     // Update connection status
     if (xSemaphoreTake(s_comm_mutex,INTERFACE_MODBUS_RTU].connected = true;
         xSemaphoreGive(s_comm_mutex);
     }
     
     uint8_t rx_buffer[256];
     
     // Main task loop
     while (1) {
         // Check if interface is still enabled
         bool enabled = false;
         esp_err_t err = comm_manager_is_interface_enabled(COMM_INTERFACE_MODBUS_RTU, &enabled);
         
         if (err != ESP_OK || !enabled) {
             ESP_LOGI(TAG, "Modbus RTU interface disabled, stopping task");
             
             // Update connection status before exiting
             if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 s_interfaces[COMM_INTERFACE_MODBUS_RTU].connected = false;
                 xSemaphoreGive(s_comm_mutex);
             }
             
             // Clean up and exit task
             uart_driver_delete(UART_NUM_1);
             s_modbus_rtu_task_handle = NULL;
             vTaskDelete(NULL);
             return;
         }
         
         // Read data from UART
         int len = uart_read_bytes(UART_NUM_1, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(100));
         
         if (len > 0) {
             ESP_LOGI(TAG, "Received %d bytes from Modbus RTU", len);
             
             // Update statistics
             if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 s_interfaces[COMM_INTERFACE_MODBUS_RTU].rx_count++;
                 s_interfaces[COMM_INTERFACE_MODBUS_RTU].last_rx_time = esp_timer_get_time() / 1000;
                 s_interfaces[COMM_INTERFACE_MODBUS_RTU].bytes_rx += len;
                 xSemaphoreGive(s_comm_mutex);
             }
             
             // Set event bit to signal data available
             xEventGroupSetBits(s_comm_event_group, COMM_EVENT_MODBUS_RTU_DATA);
             
             // Notify callbacks
             notify_callbacks(COMM_INTERFACE_MODBUS_RTU, len);
         }
         
         // Simulate occasional transmission
         if (esp_random() % 20 == 0) {
             uint8_t tx_buffer[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // Example Modbus query
             int bytes_sent = uart_write_bytes(UART_NUM_1, (const char*)tx_buffer, sizeof(tx_buffer));
             
             if (bytes_sent > 0) {
                 // Update statistics
                 if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                     s_interfaces[COMM_INTERFACE_MODBUS_RTU].tx_count++;
                     s_interfaces[COMM_INTERFACE_MODBUS_RTU].last_tx_time = esp_timer_get_time() / 1000;
                     s_interfaces[COMM_INTERFACE_MODBUS_RTU].bytes_tx += bytes_sent;
                     xSemaphoreGive(s_comm_mutex);
                 }
             } else {
                 // Update error statistics
                 if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                     s_interfaces[COMM_INTERFACE_MODBUS_RTU].error_count++;
                     xSemaphoreGive(s_comm_mutex);
                 }
             }
         }
         
         vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent CPU hogging
     }
 }
 
 static void modbus_tcp_task(void *arg) {
     ESP_LOGI(TAG, "Modbus TCP task started");
     
     // Update connection status
     if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         s_interfaces[COMM_INTERFACE_MODBUS_TCP].connected = true;
         xSemaphoreGive(s_comm_mutex);
     }
     
     // Main task loop
     while (1) {
         // Check if interface is still enabled
         bool enabled = false;
         esp_err_t err = comm_manager_is_interface_enabled(COMM_INTERFACE_MODBUS_TCP, &enabled);
         
         if (err != ESP_OK || !enabled) {
             ESP_LOGI(TAG, "Modbus TCP interface disabled, stopping task");
             
             // Update connection status before exiting
             if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 s_interfaces[COMM_INTERFACE_MODBUS_TCP].connected = false;
                 xSemaphoreGive(s_comm_mutex);
             }
             
             s_modbus_tcp_task_handle = NULL;
             vTaskDelete(NULL);
             return;
         }
         
         // Modbus TCP implementation would go here
         // For now, just update statistics periodically for simulation
         
         vTaskDelay(pdMS_TO_TICKS(1000));
     }
 }
 
 static void canbus_task(void *arg) {
    ESP_LOGI(TAG, "CANbus task started");
    
    // Configure TWAI (Two-Wire Automotive Interface - CAN driver for ESP32)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbit/s
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver");
        s_canbus_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    // Start TWAI driver
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver");
        twai_driver_uninstall();
        s_canbus_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    // Update connection status
    if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_interfaces[COMM_INTERFACE_CANBUS].connected = true;
        xSemaphoreGive(s_comm_mutex);
    }
    
    twai_message_t rx_message;
    
    // Main task loop
    while (1) {
        // Check if interface is still enabled
        bool enabled = false;
        esp_err_t err = comm_manager_is_interface_enabled(COMM_INTERFACE_CANBUS, &enabled);
        
        if (err != ESP_OK || !enabled) {
            ESP_LOGI(TAG, "CANbus interface disabled, stopping task");
            
            // Update connection status before exiting
            if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_interfaces[COMM_INTERFACE_CANBUS].connected = false;
                xSemaphoreGive(s_comm_mutex);
            }
            
            // Clean up and exit task
            twai_stop();
            twai_driver_uninstall();
            s_canbus_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
        
        // Receive message from CAN bus with timeout
        esp_err_t result = twai_receive(&rx_message, pdMS_TO_TICKS(100));
        
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Received CAN message, ID: 0x%lx, DLC: %d", rx_message.identifier, rx_message.data_length_code);
            
            // Update statistics
            if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_interfaces[COMM_INTERFACE_CANBUS].rx_count++;
                s_interfaces[COMM_INTERFACE_CANBUS].last_rx_time = esp_timer_get_time() / 1000;
                s_interfaces[COMM_INTERFACE_CANBUS].bytes_rx += rx_message.data_length_code;
                xSemaphoreGive(s_comm_mutex);
            }
            
            // Set event bit to signal data available
            xEventGroupSetBits(s_comm_event_group, COMM_EVENT_CANBUS_DATA);
            
            // Notify callbacks
            notify_callbacks(COMM_INTERFACE_CANBUS, rx_message.identifier);
        } else if (result != ESP_ERR_TIMEOUT) {
            // Update error statistics if not just a timeout
            if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_interfaces[COMM_INTERFACE_CANBUS].error_count++;
                xSemaphoreGive(s_comm_mutex);
            }
        }
        
        // Simulate occasional transmission
        if (esp_random() % 20 == 0) {
            twai_message_t tx_message;
            tx_message.identifier = 0x0123;               // Example CAN ID
            tx_message.data_length_code = 8;              // 8 bytes
            tx_message.flags = TWAI_MSG_FLAG_NONE;        // Standard message
            
            // Fill with dummy data
            for (int i = 0; i < 8; i++) {
                tx_message.data[i] = i;
            }
            
            // Transmit message
            esp_err_t tx_result = twai_transmit(&tx_message, pdMS_TO_TICKS(100));
            
            if (tx_result == ESP_OK) {
                // Update statistics
                if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    s_interfaces[COMM_INTERFACE_CANBUS].tx_count++;
                    s_interfaces[COMM_INTERFACE_CANBUS].last_tx_time = esp_timer_get_time() / 1000;
                    s_interfaces[COMM_INTERFACE_CANBUS].bytes_tx += tx_message.data_length_code;
                    xSemaphoreGive(s_comm_mutex);
                }
            } else {
                // Update error statistics
                if (xSemaphoreTake(s_comm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    s_interfaces[COMM_INTERFACE_CANBUS].error_count++;
                    xSemaphoreGive(s_comm_mutex);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent CPU hogging
    }
}