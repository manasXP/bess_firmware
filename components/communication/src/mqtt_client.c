/**
 * @file mqtt_client.c
 * @brief Implementation of MQTT Client for the BESS Main Controller
 */

 #include "mqtt_client.h"
 #include "esp_log.h"
 #include "esp_system.h"
 #include "freertos/semphr.h"
 #include "string.h"
 #include "cJSON.h"
 #include "esp_tls.h"
 #include <time.h>
 
 #define TAG "BESS_MQTT"
 
 /**
  * @brief Internal structure to track subscription information
  */
 typedef struct {
     char topic[BESS_MQTT_MAX_TOPIC_LENGTH];
     bess_mqtt_qos_t qos;
     bool active;
 } bess_mqtt_subscription_t;
 
 /**
  * @brief Internal MQTT client context
  */
 typedef struct {
     esp_mqtt_client_handle_t client;
     bess_mqtt_state_t state;
     bess_mqtt_config_t config;
     bess_mqtt_subscription_t subscriptions[BESS_MQTT_MAX_SUBSCRIPTIONS];
     uint8_t subscription_count;
     SemaphoreHandle_t mutex;
     EventGroupHandle_t event_group;
     TaskHandle_t reconnect_task;
     uint16_t reconnect_attempts;
     bool initialized;
 } bess_mqtt_context_t;
 
 // Event group bits
 #define MQTT_CONNECTED_BIT      BIT0
 #define MQTT_DISCONNECTED_BIT   BIT1
 #define MQTT_ERROR_BIT          BIT2
 
 // Static client context
 static bess_mqtt_context_t s_mqtt_context = {
     .client = NULL,
     .state = BESS_MQTT_STATE_DISCONNECTED,
     .subscription_count = 0,
     .mutex = NULL,
     .event_group = NULL,
     .reconnect_task = NULL,
     .reconnect_attempts = 0,
     .initialized = false
 };
 
 // Forward declarations of internal functions
 static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
 static void mqtt_reconnect_task(void *pvParameters);
 static esp_err_t mqtt_subscribe_internal(const char *topic, bess_mqtt_qos_t qos);
 static esp_err_t mqtt_unsubscribe_internal(const char *topic);
 static void mqtt_handle_event_connected(void);
 static void mqtt_handle_event_disconnected(void);
 static void mqtt_handle_event_subscribed(const char *topic, int qos);
 static void mqtt_handle_event_unsubscribed(const char *topic);
 static void mqtt_handle_event_published(int msg_id);
 static void mqtt_handle_event_data(const char *topic, const uint8_t *data, int data_len, int qos, bool retain);
 static void mqtt_handle_event_error(bess_mqtt_err_t error_code, const char *error_message);
 
 /**
  * @brief Main MQTT event handler
  */
 static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
     esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
 
     switch (event_id) {
         case MQTT_EVENT_CONNECTED:
             mqtt_handle_event_connected();
             break;
 
         case MQTT_EVENT_DISCONNECTED:
             mqtt_handle_event_disconnected();
             break;
 
         case MQTT_EVENT_SUBSCRIBED:
             mqtt_handle_event_subscribed(event->topic, event->qos);
             break;
 
         case MQTT_EVENT_UNSUBSCRIBED:
             mqtt_handle_event_unsubscribed(event->topic);
             break;
 
         case MQTT_EVENT_PUBLISHED:
             mqtt_handle_event_published(event->msg_id);
             break;
 
         case MQTT_EVENT_DATA:
             mqtt_handle_event_data(event->topic, event->data, event->data_len, event->qos, event->retain);
             break;
 
         case MQTT_EVENT_ERROR:
             {
                 // Handle different types of errors
                 if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                     if (event->error_handle->esp_tls_last_esp_err) {
                         mqtt_handle_event_error(BESS_MQTT_ERR_CONNECT_FAILED, 
                                             esp_err_to_name(event->error_handle->esp_tls_last_esp_err));
                     } else if (event->error_handle->esp_tls_stack_err) {
                         mqtt_handle_event_error(BESS_MQTT_ERR_TLS_FAILED, 
                                             "TLS error");
                     } else if (event->error_handle->esp_transport_sock_errno) {
                         mqtt_handle_event_error(BESS_MQTT_ERR_CONNECT_FAILED, 
                                             strerror(event->error_handle->esp_transport_sock_errno));
                     }
                 } else {
                     mqtt_handle_event_error(BESS_MQTT_ERR_UNKNOWN, 
                                         "Unknown MQTT error");
                 }
             }
             break;
 
         default:
             ESP_LOGD(TAG, "Other MQTT event: %d", event_id);
             break;
     }
 }
 
 /**
  * @brief Reconnect task that attempts to reestablish MQTT connection
  */
 static void mqtt_reconnect_task(void *pvParameters) {
     while (1) {
         // Only attempt reconnection if we're disconnected but initialized
         if (s_mqtt_context.initialized && 
             (s_mqtt_context.state == BESS_MQTT_STATE_DISCONNECTED || 
              s_mqtt_context.state == BESS_MQTT_STATE_ERROR)) {
             
             // Check if we've reached the maximum reconnect attempts
             if (s_mqtt_context.config.max_reconnect_attempts > 0 && 
                 s_mqtt_context.reconnect_attempts >= s_mqtt_context.config.max_reconnect_attempts) {
                 
                 ESP_LOGW(TAG, "Maximum reconnect attempts reached (%d)", 
                         s_mqtt_context.reconnect_attempts);
                 
                 // Sleep for a while and continue monitoring
                 vTaskDelay(pdMS_TO_TICKS(s_mqtt_context.config.reconnect_interval_ms * 5));
                 continue;
             }
             
             ESP_LOGI(TAG, "Attempting MQTT reconnection (%d/%d)", 
                     s_mqtt_context.reconnect_attempts + 1,
                     s_mqtt_context.config.max_reconnect_attempts > 0 ? 
                         s_mqtt_context.config.max_reconnect_attempts : INT_MAX);
             
             // Try to connect
             esp_err_t ret = bess_mqtt_connect();
             
             if (ret == ESP_OK) {
                 ESP_LOGI(TAG, "MQTT reconnection successful");
                 s_mqtt_context.reconnect_attempts = 0;
                 
                 // Try to resubscribe to all active topics
                 if (xSemaphoreTake(s_mqtt_context.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                     for (int i = 0; i < s_mqtt_context.subscription_count; i++) {
                         if (s_mqtt_context.subscriptions[i].active) {
                             ESP_LOGI(TAG, "Resubscribing to topic: %s", 
                                     s_mqtt_context.subscriptions[i].topic);
                             
                             // We don't check the result as we'll keep trying on the next iteration
                             esp_mqtt_client_subscribe(s_mqtt_context.client, 
                                                     s_mqtt_context.subscriptions[i].topic, 
                                                     s_mqtt_context.subscriptions[i].qos);
                         }
                     }
                     xSemaphoreGive(s_mqtt_context.mutex);
                 }
             } else {
                 s_mqtt_context.reconnect_attempts++;
                 ESP_LOGW(TAG, "MQTT reconnection failed: %s", esp_err_to_name(ret));
             }
         }
         
         // Sleep for the configured interval
         vTaskDelay(pdMS_TO_TICKS(s_mqtt_context.config.reconnect_interval_ms));
     }
 }
 
 /**
  * @brief Subscribe to an MQTT topic internally
  */
 static esp_err_t mqtt_subscribe_internal(const char *topic, bess_mqtt_qos_t qos) {
     if (xSemaphoreTake(s_mqtt_context.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to acquire mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     // Check if we're already subscribed to this topic
     for (int i = 0; i < s_mqtt_context.subscription_count; i++) {
         if (strcmp(s_mqtt_context.subscriptions[i].topic, topic) == 0) {
             // Already subscribed, update QoS if different
             if (s_mqtt_context.subscriptions[i].qos != qos) {
                 s_mqtt_context.subscriptions[i].qos = qos;
                 xSemaphoreGive(s_mqtt_context.mutex);
                 
                 // Resubscribe with new QoS
                 int msg_id = esp_mqtt_client_subscribe(s_mqtt_context.client, topic, qos);
                 if (msg_id < 0) {
                     ESP_LOGE(TAG, "Failed to update subscription for topic %s", topic);
                     return BESS_MQTT_ERR_SUBSCRIBE_FAILED;
                 }
                 
                 ESP_LOGI(TAG, "Updated subscription QoS for topic %s to %d", topic, qos);
                 return ESP_OK;
             }
             
             xSemaphoreGive(s_mqtt_context.mutex);
             ESP_LOGW(TAG, "Already subscribed to topic %s with same QoS", topic);
             return BESS_MQTT_ERR_TOPIC_EXISTS;
         }
     }
 
     // Check if we've reached the maximum number of subscriptions
     if (s_mqtt_context.subscription_count >= BESS_MQTT_MAX_SUBSCRIPTIONS) {
         xSemaphoreGive(s_mqtt_context.mutex);
         ESP_LOGE(TAG, "Maximum number of subscriptions reached");
         return BESS_MQTT_ERR_MAX_SUBSCRIPTIONS;
     }
 
     // Subscribe to topic
     int msg_id = esp_mqtt_client_subscribe(s_mqtt_context.client, topic, qos);
     if (msg_id < 0) {
         xSemaphoreGive(s_mqtt_context.mutex);
         ESP_LOGE(TAG, "Failed to subscribe to topic %s", topic);
         return BESS_MQTT_ERR_SUBSCRIBE_FAILED;
     }
 
     // Add to subscription list
     strncpy(s_mqtt_context.subscriptions[s_mqtt_context.subscription_count].topic, 
            topic, 
            BESS_MQTT_MAX_TOPIC_LENGTH - 1);
            
     s_mqtt_context.subscriptions[s_mqtt_context.subscription_count].qos = qos;
     s_mqtt_context.subscriptions[s_mqtt_context.subscription_count].active = true;
     s_mqtt_context.subscription_count++;
 
     xSemaphoreGive(s_mqtt_context.mutex);
     ESP_LOGI(TAG, "Subscribed to topic %s with QoS %d", topic, qos);
     return ESP_OK;
 }
 
 /**
  * @brief Unsubscribe from an MQTT topic internally
  */
 static esp_err_t mqtt_unsubscribe_internal(const char *topic) {
     if (xSemaphoreTake(s_mqtt_context.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to acquire mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     // Find the topic in our subscription list
     int index = -1;
     for (int i = 0; i < s_mqtt_context.subscription_count; i++) {
         if (strcmp(s_mqtt_context.subscriptions[i].topic, topic) == 0) {
             index = i;
             break;
         }
     }
 
     if (index == -1) {
         xSemaphoreGive(s_mqtt_context.mutex);
         ESP_LOGW(TAG, "Not subscribed to topic %s", topic);
         return BESS_MQTT_ERR_TOPIC_NOT_FOUND;
     }
 
     // Unsubscribe from topic
     int msg_id = esp_mqtt_client_unsubscribe(s_mqtt_context.client, topic);
     if (msg_id < 0) {
         xSemaphoreGive(s_mqtt_context.mutex);
         ESP_LOGE(TAG, "Failed to unsubscribe from topic %s", topic);
         return BESS_MQTT_ERR_SUBSCRIBE_FAILED;
     }
 
     // Mark subscription as inactive
     s_mqtt_context.subscriptions[index].active = false;
 
     // Compact subscription list if this was the last item
     if (index == s_mqtt_context.subscription_count - 1) {
         s_mqtt_context.subscription_count--;
     }
 
     xSemaphoreGive(s_mqtt_context.mutex);
     ESP_LOGI(TAG, "Unsubscribed from topic %s", topic);
     return ESP_OK;
 }
 
 /**
  * @brief Handle MQTT connected event
  */
 static void mqtt_handle_event_connected(void) {
     ESP_LOGI(TAG, "MQTT client connected to broker");
     s_mqtt_context.state = BESS_MQTT_STATE_CONNECTED;
     s_mqtt_context.reconnect_attempts = 0;
     
     // Signal connected event
     xEventGroupSetBits(s_mqtt_context.event_group, MQTT_CONNECTED_BIT);
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_CONNECTED
         };
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
     
     // Publish connection status
     cJSON *status = cJSON_CreateObject();
     if (status != NULL) {
         cJSON_AddStringToObject(status, "status", "connected");
         cJSON_AddNumberToObject(status, "timestamp", (double)time(NULL));
         bess_mqtt_publish_json("bess/status/connection", status, BESS_MQTT_QOS_1, true);
         cJSON_Delete(status);
     }
 }
 
 /**
  * @brief Handle MQTT disconnected event
  */
 static void mqtt_handle_event_disconnected(void) {
     ESP_LOGW(TAG, "MQTT client disconnected from broker");
     s_mqtt_context.state = BESS_MQTT_STATE_DISCONNECTED;
     
     // Signal disconnected event
     xEventGroupSetBits(s_mqtt_context.event_group, MQTT_DISCONNECTED_BIT);
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_DISCONNECTED
         };
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 }
 
 /**
  * @brief Handle MQTT subscribed event
  */
 static void mqtt_handle_event_subscribed(const char *topic, int qos) {
     ESP_LOGI(TAG, "MQTT client subscribed to topic %s with QoS %d", topic, qos);
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_SUBSCRIBED
         };
         
         strncpy(event_data.subscribed.topic, topic, BESS_MQTT_MAX_TOPIC_LENGTH - 1);
         event_data.subscribed.qos = (bess_mqtt_qos_t)qos;
         
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 }
 
 /**
  * @brief Handle MQTT unsubscribed event
  */
 static void mqtt_handle_event_unsubscribed(const char *topic) {
     ESP_LOGI(TAG, "MQTT client unsubscribed from topic %s", topic);
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_UNSUBSCRIBED
         };
         
         strncpy(event_data.unsubscribed.topic, topic, BESS_MQTT_MAX_TOPIC_LENGTH - 1);
         
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 }
 
 /**
  * @brief Handle MQTT published event
  */
 static void mqtt_handle_event_published(int msg_id) {
     ESP_LOGD(TAG, "MQTT message published successfully, msg_id=%d", msg_id);
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_PUBLISHED
         };
         
         event_data.published.message_id = msg_id;
         
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 }
 
 /**
  * @brief Handle MQTT data event (message received)
  */
 static void mqtt_handle_event_data(const char *topic, const uint8_t *data, int data_len, int qos, bool retain) {
     ESP_LOGI(TAG, "MQTT message received on topic %s (QoS %d, retain %d, len %d)", 
              topic, qos, retain, data_len);
     
     // Special handling for control topics
     if (strcmp(topic, "bess/control/commands") == 0) {
         bess_mqtt_message_t message = {0};
         strncpy(message.topic, topic, BESS_MQTT_MAX_TOPIC_LENGTH - 1);
         message.data = (uint8_t *)data;
         message.data_len = data_len;
         message.qos = (bess_mqtt_qos_t)qos;
         message.retain = retain;
         
         // Handle control command directly
         bess_mqtt_handle_control_command(&message);
     }
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_DATA
         };
         
         strncpy(event_data.message.topic, topic, BESS_MQTT_MAX_TOPIC_LENGTH - 1);
         event_data.message.data = (uint8_t *)data;
         event_data.message.data_len = data_len;
         event_data.message.qos = (bess_mqtt_qos_t)qos;
         event_data.message.retain = retain;
         
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 }
 
 /**
  * @brief Handle MQTT error event
  */
 static void mqtt_handle_event_error(bess_mqtt_err_t error_code, const char *error_message) {
     ESP_LOGE(TAG, "MQTT error: %s", error_message);
     
     // Set error state and signal error event
     s_mqtt_context.state = BESS_MQTT_STATE_ERROR;
     xEventGroupSetBits(s_mqtt_context.event_group, MQTT_ERROR_BIT);
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_ERROR
         };
         
         event_data.error.error_code = error_code;
         strncpy(event_data.error.error_msg, error_message, sizeof(event_data.error.error_msg) - 1);
         
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 }
 
 /**
  * @brief Initialize MQTT client
  */
 esp_err_t bess_mqtt_init(const bess_mqtt_config_t *config) {
     if (config == NULL) {
         ESP_LOGE(TAG, "Invalid configuration");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (s_mqtt_context.initialized) {
         ESP_LOGW(TAG, "MQTT client already initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     // Create mutex and event group
     s_mqtt_context.mutex = xSemaphoreCreateMutex();
     if (s_mqtt_context.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_ERR_NO_MEM;
     }
 
     s_mqtt_context.event_group = xEventGroupCreate();
     if (s_mqtt_context.event_group == NULL) {
         vSemaphoreDelete(s_mqtt_context.mutex);
         ESP_LOGE(TAG, "Failed to create event group");
         return ESP_ERR_NO_MEM;
     }
 
     // Copy configuration
     memcpy(&s_mqtt_context.config, config, sizeof(bess_mqtt_config_t));
 
     // Initialize MQTT client
     esp_mqtt_client_config_t mqtt_cfg = {
         .broker.address.uri = config->broker_uri,
         .credentials.username = config->username,
         .credentials.authentication.password = config->password,
         .credentials.client_id = config->client_id,
         .session.keepalive = config->keepalive,
         .session.disable_clean_session = !config->clean_session,
         .network.timeout_ms = config->timeout * 1000,
     };
 
     // Setup TLS if certificates are provided
     if (config->cert_file != NULL && config->key_file != NULL) {
         mqtt_cfg.broker.verification.certificate = config->cert_file;
         mqtt_cfg.credentials.authentication.certificate = config->cert_file;
         mqtt_cfg.credentials.authentication.key = config->key_file;
     }
 
     if (config->ca_file != NULL) {
         mqtt_cfg.broker.verification.certificate = config->ca_file;
     }
 
     // Setup LWT if enabled
     if (config->lwt.enabled) {
         mqtt_cfg.session.last_will.topic = config->lwt.topic;
         mqtt_cfg.session.last_will.msg = config->lwt.message;
         mqtt_cfg.session.last_will.qos = config->lwt.qos;
         mqtt_cfg.session.last_will.retain = config->lwt.retain;
     }
 
     s_mqtt_context.client = esp_mqtt_client_init(&mqtt_cfg);
     if (s_mqtt_context.client == NULL) {
         vEventGroupDelete(s_mqtt_context.event_group);
         vSemaphoreDelete(s_mqtt_context.mutex);
         ESP_LOGE(TAG, "Failed to initialize MQTT client");
         return ESP_FAIL;
     }
 
     // Register event handler
     esp_err_t ret = esp_mqtt_client_register_event(s_mqtt_context.client, 
                                                 ESP_EVENT_ANY_ID, 
                                                 mqtt_event_handler, 
                                                 NULL);
     if (ret != ESP_OK) {
         esp_mqtt_client_destroy(s_mqtt_context.client);
         vEventGroupDelete(s_mqtt_context.event_group);
         vSemaphoreDelete(s_mqtt_context.mutex);
         ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(ret));
         return ret;
     }
 
     s_mqtt_context.initialized = true;
     ESP_LOGI(TAG, "MQTT client initialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Connect to MQTT broker
  */
 esp_err_t bess_mqtt_connect(void) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state == BESS_MQTT_STATE_CONNECTED ||
         s_mqtt_context.state == BESS_MQTT_STATE_CONNECTING) {
         ESP_LOGW(TAG, "MQTT client already connected or connecting");
         return ESP_ERR_INVALID_STATE;
     }
 
     ESP_LOGI(TAG, "Connecting to MQTT broker: %s", s_mqtt_context.config.broker_uri);
     s_mqtt_context.state = BESS_MQTT_STATE_CONNECTING;
     
     // Call event callback if registered
     if (s_mqtt_context.config.event_callback != NULL) {
         bess_mqtt_event_data_t event_data = {
             .event_id = BESS_MQTT_EVENT_BEFORE_CONNECT
         };
         s_mqtt_context.config.event_callback(&event_data, s_mqtt_context.config.user_context);
     }
 
     esp_err_t ret = esp_mqtt_client_start(s_mqtt_context.client);
     if (ret != ESP_OK) {
         s_mqtt_context.state = BESS_MQTT_STATE_ERROR;
         ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
         return ret;
     }
 
     // Wait for connected event with timeout
     EventBits_t bits = xEventGroupWaitBits(s_mqtt_context.event_group,
                                           MQTT_CONNECTED_BIT | MQTT_ERROR_BIT,
                                           pdTRUE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(s_mqtt_context.config.timeout * 1000));
 
     if (bits & MQTT_ERROR_BIT) {
         ESP_LOGE(TAG, "Failed to connect to MQTT broker");
         return ESP_FAIL;
     }
 
     if (!(bits & MQTT_CONNECTED_BIT)) {
         ESP_LOGE(TAG, "Connection to MQTT broker timed out");
         s_mqtt_context.state = BESS_MQTT_STATE_ERROR;
         return BESS_MQTT_ERR_TIMEOUT;
     }
 
     // Create reconnect task if reconnection is enabled
     if (s_mqtt_context.config.max_reconnect_attempts != 0 && s_mqtt_context.reconnect_task == NULL) {
         BaseType_t ret = xTaskCreate(mqtt_reconnect_task, "mqtt_reconnect", 4096, NULL, 5, &s_mqtt_context.reconnect_task);
         if (ret != pdPASS) {
             ESP_LOGW(TAG, "Failed to create reconnect task");
         }
     }
 
     ESP_LOGI(TAG, "Connected to MQTT broker");
     return ESP_OK;
 }
 
 /**
  * @brief Disconnect from MQTT broker
  */
 esp_err_t bess_mqtt_disconnect(void) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state == BESS_MQTT_STATE_DISCONNECTED ||
         s_mqtt_context.state == BESS_MQTT_STATE_DISCONNECTING) {
         ESP_LOGW(TAG, "MQTT client already disconnected or disconnecting");
         return ESP_ERR_INVALID_STATE;
     }
 
     ESP_LOGI(TAG, "Disconnecting from MQTT broker");
     s_mqtt_context.state = BESS_MQTT_STATE_DISCONNECTING;
 
     esp_err_t ret = esp_mqtt_client_stop(s_mqtt_context.client);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(ret));
         return ret;
     }
 
     // Wait for disconnected event with timeout
     EventBits_t bits = xEventGroupWaitBits(s_mqtt_context.event_group,
                                           MQTT_DISCONNECTED_BIT,
                                           pdTRUE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(s_mqtt_context.config.timeout * 1000));
 
     if (!(bits & MQTT_DISCONNECTED_BIT)) {
         ESP_LOGE(TAG, "Disconnection from MQTT broker timed out");
         return BESS_MQTT_ERR_TIMEOUT;
     }
 
     ESP_LOGI(TAG, "Disconnected from MQTT broker");
     return ESP_OK;
 }
 
 /**
  * @brief Get current MQTT connection state
  */
 bess_mqtt_state_t bess_mqtt_get_state(void) {
     return s_mqtt_context.state;
 }
 
 /**
  * @brief Check if MQTT client is connected
  */
 bool bess_mqtt_is_connected(void) {
     return (s_mqtt_context.state == BESS_MQTT_STATE_CONNECTED);
 }
 
 /**
  * @brief Subscribe to MQTT topic
  */
 esp_err_t bess_mqtt_subscribe(const char *topic, bess_mqtt_qos_t qos) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (topic == NULL || strlen(topic) == 0 || strlen(topic) >= BESS_MQTT_MAX_TOPIC_LENGTH) {
         ESP_LOGE(TAG, "Invalid topic");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     return mqtt_subscribe_internal(topic, qos);
 }
 
 /**
  * @brief Unsubscribe from MQTT topic
  */
 esp_err_t bess_mqtt_unsubscribe(const char *topic) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (topic == NULL || strlen(topic) == 0 || strlen(topic) >= BESS_MQTT_MAX_TOPIC_LENGTH) {
         ESP_LOGE(TAG, "Invalid topic");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     return mqtt_unsubscribe_internal(topic);
 }
 
 /**
  * @brief Publish message to MQTT topic
  */
 esp_err_t bess_mqtt_publish(const char *topic, const void *data, size_t len, 
                           bess_mqtt_qos_t qos, bool retain) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (topic == NULL || strlen(topic) == 0 || strlen(topic) >= BESS_MQTT_MAX_TOPIC_LENGTH) {
         ESP_LOGE(TAG, "Invalid topic");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (data == NULL && len > 0) {
         ESP_LOGE(TAG, "Invalid data pointer");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (len > BESS_MQTT_MAX_MESSAGE_SIZE) {
         ESP_LOGE(TAG, "Message too large");
         return BESS_MQTT_ERR_BUFFER_OVERFLOW;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     ESP_LOGD(TAG, "Publishing message to %s (QoS %d, retain %d, len %d)", 
              topic, qos, retain, len);
 
     int msg_id = esp_mqtt_client_publish(s_mqtt_context.client, topic, data, len, qos, retain);
     if (msg_id < 0) {
         ESP_LOGE(TAG, "Failed to publish message to %s", topic);
         return BESS_MQTT_ERR_PUBLISH_FAILED;
     }
 
     ESP_LOGD(TAG, "Message published successfully, msg_id=%d", msg_id);
     return ESP_OK;
 }
 
 /**
  * @brief Publish string message to MQTT topic
  */
 esp_err_t bess_mqtt_publish_string(const char *topic, const char *str, 
                                  bess_mqtt_qos_t qos, bool retain) {
     if (str == NULL) {
         ESP_LOGE(TAG, "Invalid string pointer");
         return ESP_ERR_INVALID_ARG;
     }
 
     return bess_mqtt_publish(topic, str, strlen(str), qos, retain);
 }
 
 /**
  * @brief Publish JSON message to MQTT topic
  */
 esp_err_t bess_mqtt_publish_json(const char *topic, const cJSON *json_root, 
                                bess_mqtt_qos_t qos, bool retain) {
     if (json_root == NULL) {
         ESP_LOGE(TAG, "Invalid JSON object");
         return ESP_ERR_INVALID_ARG;
     }
 
     char *json_str = cJSON_Print(json_root);
     if (json_str == NULL) {
         ESP_LOGE(TAG, "Failed to serialize JSON");
         return ESP_FAIL;
     }
 
     esp_err_t result = bess_mqtt_publish_string(topic, json_str, qos, retain);
     free(json_str);
 
     return result;
 }
 
 /**
  * @brief Set event callback function
  */
 esp_err_t bess_mqtt_set_callback(bess_mqtt_event_callback_t event_callback, void *user_context) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (event_callback == NULL) {
         ESP_LOGE(TAG, "Invalid callback function");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (xSemaphoreTake(s_mqtt_context.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to acquire mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     s_mqtt_context.config.event_callback = event_callback;
     s_mqtt_context.config.user_context = user_context;
 
     xSemaphoreGive(s_mqtt_context.mutex);
     ESP_LOGI(TAG, "Event callback set successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Set LWT (Last Will and Testament) message
  */
 esp_err_t bess_mqtt_set_lwt(const char *topic, const char *message, 
                           bess_mqtt_qos_t qos, bool retain) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (topic == NULL || strlen(topic) == 0 || strlen(topic) >= BESS_MQTT_MAX_TOPIC_LENGTH) {
         ESP_LOGE(TAG, "Invalid topic");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (message == NULL || strlen(message) == 0 || strlen(message) >= 128) {
         ESP_LOGE(TAG, "Invalid message");
         return ESP_ERR_INVALID_ARG;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_DISCONNECTED) {
         ESP_LOGE(TAG, "Cannot set LWT while connected");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (xSemaphoreTake(s_mqtt_context.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to acquire mutex");
         return ESP_ERR_TIMEOUT;
     }
 
     s_mqtt_context.config.lwt.enabled = true;
     strncpy(s_mqtt_context.config.lwt.topic, topic, BESS_MQTT_MAX_TOPIC_LENGTH - 1);
     strncpy(s_mqtt_context.config.lwt.message, message, 127);
     s_mqtt_context.config.lwt.qos = qos;
     s_mqtt_context.config.lwt.retain = retain;
 
     xSemaphoreGive(s_mqtt_context.mutex);
     ESP_LOGI(TAG, "LWT message set successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Publish BESS telemetry data to MQTT
  */
 esp_err_t bess_mqtt_publish_telemetry(float system_voltage, float system_current,
                                     float system_soc, float system_soh,
                                     float power_kw, float max_temp,
                                     float min_temp, float avg_temp) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     cJSON *telemetry = cJSON_CreateObject();
     if (telemetry == NULL) {
         ESP_LOGE(TAG, "Failed to create JSON object");
         return ESP_FAIL;
     }
 
     // Add timestamp
     cJSON_AddNumberToObject(telemetry, "timestamp", (double)time(NULL));
     cJSON_AddStringToObject(telemetry, "device_id", s_mqtt_context.config.client_id);
     
     // Add electrical parameters
     cJSON *electrical = cJSON_CreateObject();
     cJSON_AddNumberToObject(electrical, "voltage", system_voltage);
     cJSON_AddNumberToObject(electrical, "current", system_current);
     cJSON_AddNumberToObject(electrical, "power_kw", power_kw);
     cJSON_AddItemToObject(telemetry, "electrical", electrical);
 
     // Add battery state
     cJSON *battery = cJSON_CreateObject();
     cJSON_AddNumberToObject(battery, "soc", system_soc);
     cJSON_AddNumberToObject(battery, "soh", system_soh);
     cJSON_AddItemToObject(telemetry, "battery", battery);
 
     // Add thermal data
     cJSON *thermal = cJSON_CreateObject();
     cJSON_AddNumberToObject(thermal, "max_temp", max_temp);
     cJSON_AddNumberToObject(thermal, "min_temp", min_temp);
     cJSON_AddNumberToObject(thermal, "avg_temp", avg_temp);
     cJSON_AddItemToObject(telemetry, "thermal", thermal);
 
     // Publish to telemetry topic
     const char *topic = "bess/telemetry";
     esp_err_t result = bess_mqtt_publish_json(topic, telemetry, BESS_MQTT_QOS_1, false);
     
     cJSON_Delete(telemetry);
     return result;
 }
 
 /**
  * @brief Publish system status update to MQTT
  */
 esp_err_t bess_mqtt_publish_status(uint32_t status_code, const char *status_message, bool is_error) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     if (status_message == NULL) {
         ESP_LOGE(TAG, "Invalid status message");
         return ESP_ERR_INVALID_ARG;
     }
 
     cJSON *status = cJSON_CreateObject();
     if (status == NULL) {
         ESP_LOGE(TAG, "Failed to create JSON object");
         return ESP_FAIL;
     }
 
     cJSON_AddNumberToObject(status, "timestamp", (double)time(NULL));
     cJSON_AddStringToObject(status, "device_id", s_mqtt_context.config.client_id);
     cJSON_AddNumberToObject(status, "code", status_code);
     cJSON_AddStringToObject(status, "message", status_message);
     cJSON_AddBoolToObject(status, "is_error", is_error);
 
     // Publish to status topic
     const char *topic = is_error ? "bess/status/error" : "bess/status/info";
     esp_err_t result = bess_mqtt_publish_json(topic, status, BESS_MQTT_QOS_1, false);
     
     cJSON_Delete(status);
     return result;
 }
 
 /**
  * @brief Publish alarm or event to MQTT
  */
 esp_err_t bess_mqtt_publish_alarm(uint32_t alarm_code, const char *alarm_message, 
                                 uint8_t severity, int16_t module_id) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     if (alarm_message == NULL) {
         ESP_LOGE(TAG, "Invalid alarm message");
         return ESP_ERR_INVALID_ARG;
     }
 
     cJSON *alarm = cJSON_CreateObject();
     if (alarm == NULL) {
         ESP_LOGE(TAG, "Failed to create JSON object");
         return ESP_FAIL;
     }
 
     cJSON_AddNumberToObject(alarm, "timestamp", (double)time(NULL));
     cJSON_AddStringToObject(alarm, "device_id", s_mqtt_context.config.client_id);
     cJSON_AddNumberToObject(alarm, "code", alarm_code);
     cJSON_AddStringToObject(alarm, "message", alarm_message);
     cJSON_AddNumberToObject(alarm, "severity", severity);
     
     if (module_id >= 0) {
         cJSON_AddNumberToObject(alarm, "module_id", module_id);
     }
 
     // Determine topic based on severity
     char topic[BESS_MQTT_MAX_TOPIC_LENGTH];
     if (severity >= 4) {
         snprintf(topic, sizeof(topic), "bess/alarms/critical");
     } else if (severity >= 2) {
         snprintf(topic, sizeof(topic), "bess/alarms/warning");
     } else {
         snprintf(topic, sizeof(topic), "bess/alarms/info");
     }
 
     esp_err_t result = bess_mqtt_publish_json(topic, alarm, BESS_MQTT_QOS_1, false);
     
     cJSON_Delete(alarm);
     return result;
 }
 
 /**
  * @brief Handle control commands received via MQTT
  */
 esp_err_t bess_mqtt_handle_control_command(const bess_mqtt_message_t *message) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (message == NULL || message->data == NULL || message->data_len == 0) {
         ESP_LOGE(TAG, "Invalid message");
         return ESP_ERR_INVALID_ARG;
     }
 
     // Copy data to ensure it's null-terminated
     char *data_str = malloc(message->data_len + 1);
     if (data_str == NULL) {
         ESP_LOGE(TAG, "Memory allocation failed");
         return ESP_ERR_NO_MEM;
     }
 
     memcpy(data_str, message->data, message->data_len);
     data_str[message->data_len] = '\0';
 
     // Parse JSON command
     cJSON *command = cJSON_Parse(data_str);
     free(data_str);
 
     if (command == NULL) {
         ESP_LOGE(TAG, "Failed to parse JSON command");
         return ESP_FAIL;
     }
 
     // Extract command type
     cJSON *cmd_type = cJSON_GetObjectItem(command, "command");
     if (cmd_type == NULL || !cJSON_IsString(cmd_type)) {
         ESP_LOGE(TAG, "Invalid command format: missing 'command' field");
         cJSON_Delete(command);
         return ESP_FAIL;
     }
 
     const char *cmd_str = cmd_type->valuestring;
     ESP_LOGI(TAG, "Received command: %s", cmd_str);
 
     // TODO: Process specific commands and implement control logic
     // This is a placeholder for the actual command processing
     // The implementation depends on the specific BESS control interface
 
     // Example command processing structure
     if (strcmp(cmd_str, "get_status") == 0) {
         // Handle get_status command
         ESP_LOGI(TAG, "Processing get_status command");
         // TODO: Implement status response
     } else if (strcmp(cmd_str, "set_mode") == 0) {
         // Handle set_mode command
         cJSON *mode = cJSON_GetObjectItem(command, "mode");
         if (mode != NULL && cJSON_IsString(mode)) {
             ESP_LOGI(TAG, "Setting mode to: %s", mode->valuestring);
             // TODO: Implement mode setting
         } else {
             ESP_LOGE(TAG, "Invalid set_mode command: missing 'mode' field");
         }
     } else if (strcmp(cmd_str, "start_balancing") == 0) {
         // Handle start_balancing command
         ESP_LOGI(TAG, "Starting cell balancing");
         // TODO: Call battery_manager_start_balancing()
     } else if (strcmp(cmd_str, "stop_balancing") == 0) {
         // Handle stop_balancing command
         ESP_LOGI(TAG, "Stopping cell balancing");
         // TODO: Call battery_manager_stop_balancing()
     } else {
         ESP_LOGW(TAG, "Unknown command: %s", cmd_str);
     }
 
     cJSON_Delete(command);
     return ESP_OK;
 }
 
 /**
  * @brief Register predefined topics for BESS operation
  */
 esp_err_t bess_mqtt_register_standard_topics(void) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     esp_err_t ret;
 
     // Control topics
     ret = bess_mqtt_subscribe("bess/control/commands", BESS_MQTT_QOS_1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to subscribe to control commands topic");
         return ret;
     }
 
     ret = bess_mqtt_subscribe("bess/control/config", BESS_MQTT_QOS_1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to subscribe to control config topic");
         return ret;
     }
 
     // System-wide topics
     ret = bess_mqtt_subscribe("bess/system/reset", BESS_MQTT_QOS_2);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to subscribe to system reset topic");
         return ret;
     }
 
     // Firmware update topics
     ret = bess_mqtt_subscribe("bess/firmware/update", BESS_MQTT_QOS_2);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to subscribe to firmware update topic");
         return ret;
     }
 
     ESP_LOGI(TAG, "Successfully registered all standard topics");
     return ESP_OK;
 }
 
 /**
  * @brief Publish log message to MQTT for CloudWatch integration
  */
 esp_err_t bess_mqtt_publish_log(uint8_t log_level, const char *component, const char *message) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     if (s_mqtt_context.state != BESS_MQTT_STATE_CONNECTED) {
         ESP_LOGE(TAG, "MQTT client not connected");
         return BESS_MQTT_ERR_DISCONNECTED;
     }
 
     if (component == NULL || message == NULL) {
         ESP_LOGE(TAG, "Invalid component or message");
         return ESP_ERR_INVALID_ARG;
     }
 
     // CloudWatch expects a specific format for log messages
     cJSON *log = cJSON_CreateObject();
     if (log == NULL) {
         ESP_LOGE(TAG, "Failed to create JSON object");
         return ESP_FAIL;
     }
 
     // Add common fields
     cJSON_AddNumberToObject(log, "timestamp", (double)time(NULL) * 1000); // CloudWatch expects milliseconds
     cJSON_AddStringToObject(log, "device_id", s_mqtt_context.config.client_id);
     cJSON_AddNumberToObject(log, "level", log_level);
     cJSON_AddStringToObject(log, "component", component);
     cJSON_AddStringToObject(log, "message", message);
 
     // Determine log topic based on level
     char topic[BESS_MQTT_MAX_TOPIC_LENGTH];
     if (log_level >= 4) { // Error and critical
         snprintf(topic, sizeof(topic), "bess/logs/error");
     } else if (log_level >= 2) { // Warning and notice
         snprintf(topic, sizeof(topic), "bess/logs/warning");
     } else { // Info and debug
         snprintf(topic, sizeof(topic), "bess/logs/info");
     }
 
     esp_err_t result = bess_mqtt_publish_json(topic, log, BESS_MQTT_QOS_0, false);
     
     cJSON_Delete(log);
     return result;
 }
 
 /**
  * @brief Clean up MQTT client resources
  */
 esp_err_t bess_mqtt_cleanup(void) {
     if (!s_mqtt_context.initialized) {
         ESP_LOGE(TAG, "MQTT client not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     // Disconnect if connected
     if (s_mqtt_context.state == BESS_MQTT_STATE_CONNECTED) {
         esp_err_t ret = bess_mqtt_disconnect();
         if (ret != ESP_OK) {
             ESP_LOGW(TAG, "Failed to disconnect MQTT client during cleanup");
             // Continue with cleanup anyway
         }
     }
 
     // Delete reconnect task if it exists
     if (s_mqtt_context.reconnect_task != NULL) {
         vTaskDelete(s_mqtt_context.reconnect_task);
         s_mqtt_context.reconnect_task = NULL;
     }
 
     // Destroy MQTT client
     if (s_mqtt_context.client != NULL) {
         esp_mqtt_client_destroy(s_mqtt_context.client);
         s_mqtt_context.client = NULL;
     }
 
     // Delete synchronization primitives
     if (s_mqtt_context.mutex != NULL) {
         vSemaphoreDelete(s_mqtt_context.mutex);
         s_mqtt_context.mutex = NULL;
     }
 
     if (s_mqtt_context.event_group != NULL) {
         vEventGroupDelete(s_mqtt_context.event_group);
         s_mqtt_context.event_group = NULL;
     }
 
     // Reset context
     s_mqtt_context.state = BESS_MQTT_STATE_DISCONNECTED;
     s_mqtt_context.subscription_count = 0;
     s_mqtt_context.reconnect_attempts = 0;
     s_mqtt_context.initialized = false;
 
     ESP_LOGI(TAG, "MQTT client cleaned up successfully");
     return ESP_OK;
 }