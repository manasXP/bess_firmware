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
     
     /* Create RX and TX tasks */
     BaseType_t xReturned = xTaskCreate(
         canbus_rx_task,
         "canbus_rx",
         CANBUS_TASK_STACK_SIZE,
         NULL,
         CANBUS_TASK_PRIORITY,
         &s_canbus_state.rx_task_handle
     );
     
     if (xReturned != pdPASS) {
         twai_stop();
         ESP_LOGE(TAG, "Failed to create RX task");
         return ESP_ERR_NO_MEM;
     }
     
     xReturned = xTaskCreate(
         canbus_tx_task,
         "canbus_tx",
         CANBUS_TASK_STACK_SIZE,
         NULL,
         CANBUS_TASK_PRIORITY,
         &s_canbus_state.tx_task_handle
     );
     
     if (xReturned != pdPASS) {
         vTaskDelete(s_canbus_state.rx_task_handle);
         twai_stop();
         ESP_LOGE(TAG, "Failed to create TX task");
         return ESP_ERR_NO_MEM;
     }
     
     s_canbus_state.running = true;
     ESP_LOGI(TAG, "CANbus interface started");
     return ESP_OK;
 }
 
 /**
  * @brief Stop the CANbus interface
  */
 esp_err_t canbus_interface_stop(void) {
     ESP_LOGI(TAG, "Stopping CANbus interface");
     
     if (!s_canbus_state.running) {
         ESP_LOGW(TAG, "CANbus interface not running");
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Delete tasks */
     if (s_canbus_state.rx_task_handle != NULL) {
         vTaskDelete(s_canbus_state.rx_task_handle);
         s_canbus_state.rx_task_handle = NULL;
     }
     
     if (s_canbus_state.tx_task_handle != NULL) {
         vTaskDelete(s_canbus_state.tx_task_handle);
         s_canbus_state.tx_task_handle = NULL;
     }
     
     /* Stop TWAI driver */
     esp_err_t ret = twai_stop();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(ret));
         return ret;
     }
     
     /* Clear TX queue */
     xQueueReset(s_canbus_state.tx_queue);
     
     s_canbus_state.running = false;
     ESP_LOGI(TAG, "CANbus interface stopped");
     return ESP_OK;
 }
 
 /**
  * @brief Send a CAN message
  */
 esp_err_t canbus_interface_send_message(const twai_message_t *message) {
     if (!s_canbus_state.running) {
         ESP_LOGW(TAG, "CANbus interface not running");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (message == NULL) {
         ESP_LOGE(TAG, "Invalid message pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     canbus_tx_item_t tx_item;
     memcpy(&tx_item.message, message, sizeof(twai_message_t));
     
     if (xQueueSend(s_canbus_state.tx_queue, &tx_item, pdMS_TO_TICKS(50)) != pdTRUE) {
         ESP_LOGW(TAG, "TX queue full, message dropped");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for receiving CAN messages
  */
 esp_err_t canbus_interface_register_message_callback(uint32_t id,
                                                    uint32_t mask,
                                                    canbus_message_callback_t callback,
                                                    void *user_data) {
     if (!s_canbus_state.initialized) {
         ESP_LOGE(TAG, "CANbus interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
     
     int free_slot = -1;
     for (int i = 0; i < MAX_MESSAGE_CALLBACKS; i++) {
         if (!s_canbus_state.message_callbacks[i].active) {
             free_slot = i;
             break;
         }
     }
     
     if (free_slot == -1) {
         xSemaphoreGive(s_canbus_state.callback_mutex);
         ESP_LOGE(TAG, "No free slots for message callback registration");
         return ESP_ERR_NO_MEM;
     }
     
     s_canbus_state.message_callbacks[free_slot].id = id;
     s_canbus_state.message_callbacks[free_slot].mask = mask;
     s_canbus_state.message_callbacks[free_slot].callback = callback;
     s_canbus_state.message_callbacks[free_slot].user_data = user_data;
     s_canbus_state.message_callbacks[free_slot].active = true;
     
     xSemaphoreGive(s_canbus_state.callback_mutex);
     ESP_LOGI(TAG, "Registered message callback for ID 0x%lx with mask 0x%lx", id, mask);
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a previously registered message callback
  */
 esp_err_t canbus_interface_unregister_message_callback(uint32_t id,
                                                      canbus_message_callback_t callback) {
     if (!s_canbus_state.initialized) {
         ESP_LOGE(TAG, "CANbus interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t ret = ESP_ERR_NOT_FOUND;
     
     xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
     
     for (int i = 0; i < MAX_MESSAGE_CALLBACKS; i++) {
         if (s_canbus_state.message_callbacks[i].active &&
             s_canbus_state.message_callbacks[i].id == id &&
             s_canbus_state.message_callbacks[i].callback == callback) {
             s_canbus_state.message_callbacks[i].active = false;
             ret = ESP_OK;
             ESP_LOGI(TAG, "Unregistered message callback for ID 0x%lx", id);
             break;
         }
     }
     
     xSemaphoreGive(s_canbus_state.callback_mutex);
     return ret;
 }
 
 /**
  * @brief Register a callback for CANbus events
  */
 esp_err_t canbus_interface_register_event_callback(canbus_event_type_t event_type,
                                                  canbus_event_callback_t callback,
                                                  void *user_data) {
     if (!s_canbus_state.initialized) {
         ESP_LOGE(TAG, "CANbus interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (event_type > CANBUS_EVENT_BUS_RECOVERED) {
         ESP_LOGE(TAG, "Invalid event type");
         return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
     
     int free_slot = -1;
     for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
         if (!s_canbus_state.event_callbacks[event_type][i].active) {
             free_slot = i;
             break;
         }
     }
     
     if (free_slot == -1) {
         xSemaphoreGive(s_canbus_state.callback_mutex);
         ESP_LOGE(TAG, "No free slots for event callback registration");
         return ESP_ERR_NO_MEM;
     }
     
     s_canbus_state.event_callbacks[event_type][free_slot].callback = callback;
     s_canbus_state.event_callbacks[event_type][free_slot].user_data = user_data;
     s_canbus_state.event_callbacks[event_type][free_slot].active = true;
     
     xSemaphoreGive(s_canbus_state.callback_mutex);
     ESP_LOGI(TAG, "Registered event callback for event type %d", event_type);
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a previously registered event callback
  */
 esp_err_t canbus_interface_unregister_event_callback(canbus_event_type_t event_type,
                                                    canbus_event_callback_t callback) {
     if (!s_canbus_state.initialized) {
         ESP_LOGE(TAG, "CANbus interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (event_type > CANBUS_EVENT_BUS_RECOVERED) {
         ESP_LOGE(TAG, "Invalid event type");
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t ret = ESP_ERR_NOT_FOUND;
     
     xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
     
     for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
         if (s_canbus_state.event_callbacks[event_type][i].active &&
             s_canbus_state.event_callbacks[event_type][i].callback == callback) {
             s_canbus_state.event_callbacks[event_type][i].active = false;
             ret = ESP_OK;
             ESP_LOGI(TAG, "Unregistered event callback for event type %d", event_type);
             break;
         }
     }
     
     xSemaphoreGive(s_canbus_state.callback_mutex);
     return ret;
 }
 
 /**
  * @brief Get the current status of the CANbus interface
  */
 esp_err_t canbus_interface_get_status(twai_status_info_t *status) {
     if (!s_canbus_state.initialized) {
         ESP_LOGE(TAG, "CANbus interface not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (status == NULL) {
         ESP_LOGE(TAG, "Invalid status pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreTake(s_canbus_state.status_mutex, portMAX_DELAY);
     esp_err_t ret = twai_get_status_info(status);
     xSemaphoreGive(s_canbus_state.status_mutex);
     
     return ret;
 }
 
 /**
  * @brief Set the CANbus bitrate
  */
 esp_err_t canbus_interface_set_bitrate(uint32_t bitrate) {
     if (s_canbus_state.running) {
         ESP_LOGE(TAG, "Cannot change bitrate while CANbus interface is running");
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Validate bitrate */
     if (bitrate != 1000000 && bitrate != 800000 && bitrate != 500000 &&
         bitrate != 250000 && bitrate != 125000 && bitrate != 100000) {
         ESP_LOGE(TAG, "Unsupported bitrate: %lu", bitrate);
         return ESP_ERR_INVALID_ARG;
     }
     
     s_canbus_state.bitrate = bitrate;
     ESP_LOGI(TAG, "CANbus bitrate set to %lu bps", bitrate);
     
     /* If already initialized, we need to re-initialize with new bitrate */
     if (s_canbus_state.initialized) {
         esp_err_t ret = twai_driver_uninstall();
         if (ret != ESP_OK) {
             ESP_LOGE(TAG, "Failed to uninstall TWAI driver: %s", esp_err_to_name(ret));
             return ret;
         }
         
         s_canbus_state.initialized = false;
         return canbus_interface_init();
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Request data from a specific module
  */
 esp_err_t canbus_interface_request_module_data(uint8_t module_id, uint8_t data_type) {
     if (!s_canbus_state.running) {
         ESP_LOGW(TAG, "CANbus interface not running");
         return ESP_ERR_INVALID_STATE;
     }
     
     twai_message_t message;
     message.identifier = 0x600 | module_id;  // Example ID format: 0x6xx where xx is module ID
     message.data_length_code = 2;
     message.data[0] = 0x40;  // Example: 0x40 = data request command
     message.data[1] = data_type;
     message.flags = 0;  // Standard frame
     
     return canbus_interface_send_message(&message);
 }
 
 /**
  * @brief Send a command to a specific module
  */
 esp_err_t canbus_interface_send_module_command(uint8_t module_id,
                                              uint8_t command,
                                              const uint8_t *data,
                                              size_t data_len) {
     if (!s_canbus_state.running) {
         ESP_LOGW(TAG, "CANbus interface not running");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (data_len > 7) {
         ESP_LOGE(TAG, "Data too long for single CAN frame");
         return ESP_ERR_INVALID_ARG;
     }
     
     twai_message_t message;
     message.identifier = 0x700 | module_id;  // Example ID format: 0x7xx where xx is module ID
     message.data_length_code = data_len + 1;
     message.data[0] = command;
     
     if (data_len > 0 && data != NULL) {
         memcpy(&message.data[1], data, data_len);
     }
     
     message.flags = 0;  // Standard frame
     
     return canbus_interface_send_message(&message);
 }
 
 /**
  * @brief Parse a CANbus message containing module status
  */
 esp_err_t canbus_interface_parse_module_status(const twai_message_t *message,
                                              canbus_module_status_t *status) {
     if (message == NULL || status == NULL) {
         ESP_LOGE(TAG, "Invalid pointer arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Check if this is a module status message based on ID range */
     if ((message->identifier & 0xF00) != 0x500) {  // Example: 0x5xx for status messages
         ESP_LOGW(TAG, "Message ID 0x%lx is not a module status message", message->identifier);
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Check minimum required length */
     if (message->data_length_code < 8) {
         ESP_LOGW(TAG, "Message too short for module status");
         return ESP_ERR_INVALID_SIZE;
     }
     
     /* Parse module ID from identifier */
     status->module_id = message->identifier & 0xFF;
     
     /* Parse voltage - example: 2 bytes, scaling factor 0.01V */
     uint16_t raw_voltage = (message->data[0] << 8) | message->data[1];
     status->voltage = raw_voltage * 0.01f;
     
     /* Parse current - example: 2 bytes, signed value, scaling factor 0.1A */
     int16_t raw_current = (message->data[2] << 8) | message->data[3];
     status->current = raw_current * 0.1f;
     
     /* Parse temperature - example: 1 byte, offset -40°C */
     status->temperature = message->data[4] - 40.0f;
     
     /* Parse SoC - example: 1 byte, 0-100% */
     status->state_of_charge = message->data[5];
     
     /* Parse status flags */
     status->status_flags = message->data[6];
     
     /* Parse error code */
     status->error_code = message->data[7];
     
     return ESP_OK;
 }
 
 /**
  * @brief Create a CANbus message for heartbeat
  */
 esp_err_t canbus_interface_create_heartbeat_message(twai_message_t *message) {
     if (message == NULL) {
         ESP_LOGE(TAG, "Invalid message pointer");
         return ESP_ERR_INVALID_ARG;
     }
     
     message->identifier = 0x700;  // Example: 0x700 for heartbeat
     message->data_length_code = 1;
     message->data[0] = 0x01;  // Heartbeat with status "running"
     message->flags = 0;  // Standard frame
     
     return ESP_OK;
 }
 
 /**
  * @brief Create a CANbus message for system status
  */
 esp_err_t canbus_interface_create_system_status_message(const bess_system_status_t *status,
                                                       twai_message_t *message) {
     if (status == NULL || message == NULL) {
         ESP_LOGE(TAG, "Invalid pointer arguments");
         return ESP_ERR_INVALID_ARG;
     }
     
     message->identifier = 0x100;  // Example: 0x100 for system status
     message->data_length_code = 8;
     message->flags = 0;  // Standard frame
     
     /* Pack system status into CAN frame data - this is just an example */
     /* Byte 0: System state */
     message->data[0] = status->state;
     
     /* Bytes 1-2: System SoC (0-1000, representing 0-100.0%) */
     uint16_t soc = (uint16_t)(status->state_of_charge * 10.0f);
     message->data[1] = (soc >> 8) & 0xFF;
     message->data[2] = soc & 0xFF;
     
     /* Byte 3: System health (0-100%) */
     message->data[3] = status->state_of_health;
     
     /* Byte 4: Error flags */
     message->data[4] = status->error_flags & 0xFF;
     
     /* Byte 5: Warning flags */
     message->data[5] = status->warning_flags & 0xFF;
     
     /* Byte 6: Module active bitmap, low byte */
     message->data[6] = status->active_modules & 0xFF;
     
     /* Byte 7: Module active bitmap, high byte */
     message->data[7] = (status->active_modules >> 8) & 0xFF;
     
     return ESP_OK;
 }
 
 /**
  * @brief CAN receive task function
  */
 static void canbus_rx_task(void *arg) {
     ESP_LOGI(TAG, "RX task started");
     
     twai_status_info_t status_info;
     canbus_event_type_t previous_event = CANBUS_EVENT_ERROR_ACTIVE;
     
     while (s_canbus_state.running) {
         /* Check bus status every iteration */
         if (twai_get_status_info(&status_info) == ESP_OK) {
             canbus_event_type_t current_event;
             
             if (status_info.state == TWAI_STATE_BUS_OFF) {
                 current_event = CANBUS_EVENT_BUS_OFF;
             } else if (status_info.state == TWAI_STATE_RECOVERING) {
                 current_event = CANBUS_EVENT_BUS_RECOVERED;
             } else if (status_info.tx_error_counter >= 128 || status_info.rx_error_counter >= 128) {
                 current_event = CANBUS_EVENT_ERROR_PASSIVE;
             } else if (status_info.tx_error_counter >= 96 || status_info.rx_error_counter >= 96) {
                 current_event = CANBUS_EVENT_ERROR_WARNING;
             } else {
                 current_event = CANBUS_EVENT_ERROR_ACTIVE;
             }
             
             if (current_event != previous_event) {
                 handle_canbus_event(current_event);
                 previous_event = current_event;
             }
         }
         
         /* Receive message */
         twai_message_t rx_message;
         esp_err_t ret = twai_receive(&rx_message, pdMS_TO_TICKS(100));
         
         if (ret == ESP_OK) {
             process_received_message(&rx_message);
         } else if (ret != ESP_ERR_TIMEOUT) {
             ESP_LOGW(TAG, "Failed to receive message: %s", esp_err_to_name(ret));
         }
     }
     
     ESP_LOGI(TAG, "RX task stopped");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief CAN transmit task function
  */
 static void canbus_tx_task(void *arg) {
     ESP_LOGI(TAG, "TX task started");
     
     canbus_tx_item_t tx_item;
     
     while (s_canbus_state.running) {
         /* Wait for message to send */
         if (xQueueReceive(s_canbus_state.tx_queue, &tx_item, pdMS_TO_TICKS(100)) == pdTRUE) {
             /* Send message */
             esp_err_t ret = twai_transmit(&tx_item.message, pdMS_TO_TICKS(100));
             
             if (ret != ESP_OK) {
                 ESP_LOGW(TAG, "Failed to transmit message: %s", esp_err_to_name(ret));
                 
                 /* Check if we should retry */
                 if (ret != ESP_ERR_TIMEOUT && ret != ESP_ERR_INVALID_STATE) {
                     /* Put back in queue for retry */
                     if (xQueueSendToFront(s_canbus_state.tx_queue, &tx_item, 0) != pdTRUE) {
                         ESP_LOGW(TAG, "TX queue full, message dropped");
                     }
                 }
             }
         }
     }
     
     ESP_LOGI(TAG, "TX task stopped");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Process a received CAN message
  */
 static esp_err_t process_received_message(twai_message_t *message) {
     bool message_handled = false;
     
     xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
     
     /* Check message against all registered callbacks */
     for (int i = 0; i < MAX_MESSAGE_CALLBACKS; i++) {
         if (s_canbus_state.message_callbacks[i].active) {
             /* Apply mask for filtering */
             uint32_t masked_id = message->identifier & s_canbus_state.message_callbacks[i].mask;
             uint32_t masked_callback_id = s_canbus_state.message_callbacks[i].id & s_canbus_state.message_callbacks[i].mask;
             
             /* If IDs match after masking, invoke callback */
             if (masked_id == masked_callback_id) {
                 canbus_message_callback_t callback = s_canbus_state.message_callbacks[i].callback;
                 void *user_data = s_canbus_state.message_callbacks[i].user_data;
                 
                 /* Release mutex before calling callback to prevent deadlocks */
                 xSemaphoreGive(s_canbus_state.callback_mutex);
                 
                 callback(message, user_data);
                 message_handled = true;
                 
                 /* Re-acquire mutex to continue iteration */
                 xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
             }
         }
     }
     
     xSemaphoreGive(s_canbus_state.callback_mutex);
     
     /* Special handling for unhandled messages if needed */
     if (!message_handled) {
         ESP_LOGD(TAG, "Received unhandled message: ID=0x%lx, DLC=%d", 
                  message->identifier, message->data_length_code);
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Handle CANbus events and notify registered callbacks
  */
 static void handle_canbus_event(canbus_event_type_t event) {
     ESP_LOGI(TAG, "CANbus event: %d", event);
     
     xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
     
     /* Call all registered callbacks for this event */
     for (int i = 0; i < MAX_CALLBACKS_PER_EVENT; i++) {
         if (s_canbus_state.event_callbacks[event][i].active) {
             canbus_event_callback_t callback = s_canbus_state.event_callbacks[event][i].callback;
             void *user_data = s_canbus_state.event_callbacks[event][i].user_data;
             
             /* Release mutex before calling callback to prevent deadlocks */
             xSemaphoreGive(s_canbus_state.callback_mutex);
             
             callback(event, user_data);
             
             /* Re-acquire mutex to continue iteration */
             xSemaphoreTake(s_canbus_state.callback_mutex, portMAX_DELAY);
         }
     }
     
     xSemaphoreGive(s_canbus_state.callback_mutex);
     
     /* Special handling for bus-off event */
     if (event == CANBUS_EVENT_BUS_OFF) {
         ESP_LOGW(TAG, "CAN bus is in bus-off state, initiating recovery");
         twai_initiate_recovery();
     }
 }