/**
 * @file load_controller.c
 * @brief Implementation of Load Controller for BESS Power Management Module
 * 
 * This file implements the power management functions for the BESS system,
 * controlling power flow between battery, grid, and loads.
 * 
 * @copyright Copyright (c) 2025
 */

 #include "load_controller.h"
 #include "esp_timer.h"
 #include "esp_log.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "driver/gpio.h"
 #include "bess_config.h"
 #include "battery_manager.h"  // For battery status and SoC
 
 #define TAG "LOAD_CTRL"
 
 // Maximum number of event callbacks per event type
 #define MAX_EVENT_CALLBACKS 5
 
 // Event bits for the event group
 #define LOAD_CTRL_START_BIT          (1 << 0)
 #define LOAD_CTRL_STOP_BIT           (1 << 1)
 #define LOAD_CTRL_GRID_CONNECT_BIT   (1 << 2)
 #define LOAD_CTRL_GRID_DISCONNECT_BIT (1 << 3)
 #define LOAD_CTRL_EMERGENCY_BIT      (1 << 4)
 #define LOAD_CTRL_UPDATE_POWER_BIT   (1 << 5)
 
 // Command types for the command queue
 typedef enum {
     CMD_SET_MODE = 0,
     CMD_SET_POWER,
     CMD_CONNECT_GRID,
     CMD_DISCONNECT_GRID,
     CMD_ENTER_ISLAND,
     CMD_EXIT_ISLAND,
     CMD_ENABLE_CONVERTER,
     CMD_DISABLE_CONVERTER,
     CMD_EMERGENCY_SHUTDOWN,
     CMD_SET_POWER_LIMITS
 } load_controller_cmd_type_t;
 
 // Command structure for the command queue
 typedef struct {
     load_controller_cmd_type_t type;
     union {
         load_controller_mode_t mode;
         float power;
         uint32_t reason;
         struct {
             float charge;
             float discharge;
         } limits;
     } data;
 } load_controller_cmd_t;
 
 // Callback structure for event notifications
 typedef struct {
     load_controller_event_callback_t callback;
     void *user_data;
     bool in_use;
 } callback_info_t;
 
 // Private state for the load controller
 typedef struct {
     load_controller_config_t config;
     load_controller_status_t status;
     bool initialized;
     bool running;
     
     // Synchronization primitives
     SemaphoreHandle_t mutex;
     EventGroupHandle_t event_group;
     QueueHandle_t cmd_queue;
     TaskHandle_t task_handle;
     
     // Callback arrays for each event type
     callback_info_t callbacks[10][MAX_EVENT_CALLBACKS]; // 10 event types
     
     // Power ramping state
     float power_ramp_target;
     uint64_t last_power_update;
     
     // Safety state
     bool emergency_stop;
     uint32_t emergency_reason;
     
     // Diagnostics data
     uint32_t grid_connect_count;
     uint32_t grid_disconnect_count;
     uint32_t mode_change_count;
     uint32_t error_count;
 } load_controller_state_t;
 
 // Singleton instance of the load controller state
 static load_controller_state_t s_controller = {0};
 
 // Forward declarations for internal functions
 static void load_controller_task(void *arg);
 static esp_err_t load_controller_process_command(load_controller_cmd_t *cmd);
 static void load_controller_update_power(void);
 static void load_controller_notify_event(load_controller_event_t event, uint32_t data);
 static esp_err_t load_controller_validate_config(const load_controller_config_t *config);
 static esp_err_t load_controller_hardware_init(void);
 static void load_controller_hardware_deinit(void);
 static esp_err_t load_controller_hardware_set_power(float power_kw);
 static esp_err_t load_controller_hardware_enable_grid(bool enable);
 static esp_err_t load_controller_hardware_enable_converter(bool enable);
 static void load_controller_update_status_from_hardware(void);
 static bool load_controller_check_safety_conditions(void);
 
 /**
  * @brief Initialize the load controller with specified configuration
  */
 esp_err_t load_controller_init(const load_controller_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Validate configuration parameters
     esp_err_t err = load_controller_validate_config(config);
     if (err != ESP_OK) {
         return err;
     }
     
     // Return if already initialized
     if (s_controller.initialized) {
         ESP_LOGW(TAG, "Load controller already initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Create synchronization primitives
     s_controller.mutex = xSemaphoreCreateMutex();
     if (s_controller.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_ERR_NO_MEM;
     }
     
     s_controller.event_group = xEventGroupCreate();
     if (s_controller.event_group == NULL) {
         vSemaphoreDelete(s_controller.mutex);
         ESP_LOGE(TAG, "Failed to create event group");
         return ESP_ERR_NO_MEM;
     }
     
     s_controller.cmd_queue = xQueueCreate(10, sizeof(load_controller_cmd_t));
     if (s_controller.cmd_queue == NULL) {
         vEventGroupDelete(s_controller.event_group);
         vSemaphoreDelete(s_controller.mutex);
         ESP_LOGE(TAG, "Failed to create command queue");
         return ESP_ERR_NO_MEM;
     }
     
     // Copy configuration
     memcpy(&s_controller.config, config, sizeof(load_controller_config_t));
     
     // Initialize status
     s_controller.status.mode = LOAD_MODE_STANDBY;
     s_controller.status.power_flow = POWER_FLOW_NONE;
     s_controller.status.current_power_kw = 0.0f;
     s_controller.status.measured_power_kw = 0.0f;
     s_controller.status.grid_power_kw = 0.0f;
     s_controller.status.load_power_kw = 0.0f;
     s_controller.status.target_power_kw = 0.0f;
     s_controller.status.uptime_seconds = 0;
     s_controller.status.grid_connected = false;
     s_controller.status.converter_enabled = false;
     
     // Initialize other state variables
     s_controller.initialized = true;
     s_controller.running = false;
     s_controller.power_ramp_target = 0.0f;
     s_controller.last_power_update = 0;
     s_controller.emergency_stop = false;
     s_controller.emergency_reason = 0;
     
     // Clear all callback registrations
     memset(s_controller.callbacks, 0, sizeof(s_controller.callbacks));
     
     // Initialize hardware
     err = load_controller_hardware_init();
     if (err != ESP_OK) {
         load_controller_deinit();
         ESP_LOGE(TAG, "Failed to initialize hardware");
         return err;
     }
     
     ESP_LOGI(TAG, "Load controller initialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Deinitialize and free resources used by the load controller
  */
 esp_err_t load_controller_deinit(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Stop the task if running
     if (s_controller.running) {
         esp_err_t err = load_controller_stop();
         if (err != ESP_OK) {
             ESP_LOGW(TAG, "Failed to stop load controller");
             // Continue with deinit anyway
         }
     }
     
     // Clean up hardware resources
     load_controller_hardware_deinit();
     
     // Free synchronization primitives
     if (s_controller.mutex != NULL) {
         vSemaphoreDelete(s_controller.mutex);
         s_controller.mutex = NULL;
     }
     
     if (s_controller.event_group != NULL) {
         vEventGroupDelete(s_controller.event_group);
         s_controller.event_group = NULL;
     }
     
     if (s_controller.cmd_queue != NULL) {
         vQueueDelete(s_controller.cmd_queue);
         s_controller.cmd_queue = NULL;
     }
     
     // Reset state
     s_controller.initialized = false;
     
     ESP_LOGI(TAG, "Load controller deinitialized");
     return ESP_OK;
 }
 
 /**
  * @brief Start the load controller operation
  */
 esp_err_t load_controller_start(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (s_controller.running) {
         ESP_LOGW(TAG, "Load controller already running");
         return ESP_OK;
     }
     
     // Create the controller task
     BaseType_t result = xTaskCreatePinnedToCore(
         load_controller_task,
         "load_ctrl",
         4096,  // Stack size
         NULL,  // Task parameters
         5,     // Priority
         &s_controller.task_handle,
         0      // Core ID
     );
     
     if (result != pdPASS) {
         ESP_LOGE(TAG, "Failed to create load controller task");
         return ESP_ERR_NO_MEM;
     }
     
     // Set the start bit to signal the task
     xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_START_BIT);
     
     s_controller.running = true;
     ESP_LOGI(TAG, "Load controller started");
     
     // Notify that the mode changed to standby
     load_controller_notify_event(LOAD_EVENT_MODE_CHANGE, LOAD_MODE_STANDBY);
     
     return ESP_OK;
 }
 
 /**
  * @brief Stop the load controller operation
  */
 esp_err_t load_controller_stop(void) {
     if (!s_controller.initialized || !s_controller.running) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Signal task to stop
     xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_STOP_BIT);
     
     // Wait for task to terminate
     vTaskDelay(pdMS_TO_TICKS(100));
     
     // Delete task if still running
     if (s_controller.task_handle != NULL) {
         vTaskDelete(s_controller.task_handle);
         s_controller.task_handle = NULL;
     }
     
     // Reset running flag
     s_controller.running = false;
     
     ESP_LOGI(TAG, "Load controller stopped");
     return ESP_OK;
 }
 
 /**
  * @brief Set the operational mode of the load controller
  */
 esp_err_t load_controller_set_mode(load_controller_mode_t mode) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (mode < LOAD_MODE_STANDBY || mode > LOAD_MODE_TEST) {
         return ESP_ERR_INVALID_ARG;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_SET_MODE,
         .data.mode = mode
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send set mode command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Get the current operational mode of the load controller
  */
 esp_err_t load_controller_get_mode(load_controller_mode_t *mode) {
     if (!s_controller.initialized || mode == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for get_mode");
         return ESP_ERR_TIMEOUT;
     }
     
     *mode = s_controller.status.mode;
     
     xSemaphoreGive(s_controller.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set power setpoint for the BESS
  */
 esp_err_t load_controller_set_power(float power_kw) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate power setpoint
     if (power_kw > s_controller.config.max_discharge_power_kw) {
         ESP_LOGW(TAG, "Power setpoint %.2f exceeds maximum discharge limit %.2f, clamping",
                 power_kw, s_controller.config.max_discharge_power_kw);
         power_kw = s_controller.config.max_discharge_power_kw;
     } else if (power_kw < -s_controller.config.max_charge_power_kw) {
         ESP_LOGW(TAG, "Power setpoint %.2f exceeds maximum charge limit %.2f, clamping",
                 power_kw, -s_controller.config.max_charge_power_kw);
         power_kw = -s_controller.config.max_charge_power_kw;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_SET_POWER,
         .data.power = power_kw
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send set power command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Get the current status of the load controller
  */
 esp_err_t load_controller_get_status(load_controller_status_t *status) {
     if (!s_controller.initialized || status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for get_status");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy status structure
     memcpy(status, &s_controller.status, sizeof(load_controller_status_t));
     
     xSemaphoreGive(s_controller.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Connect to the grid (enable grid connection)
  */
 esp_err_t load_controller_connect_grid(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_CONNECT_GRID
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send connect grid command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Disconnect from the grid (disable grid connection)
  */
 esp_err_t load_controller_disconnect_grid(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_DISCONNECT_GRID
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send disconnect grid command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Enter island mode operation (off-grid)
  */
 esp_err_t load_controller_enter_island_mode(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Check if island mode is enabled in configuration
     if (!s_controller.config.island_mode_enabled) {
         ESP_LOGE(TAG, "Island mode not enabled in configuration");
         return ESP_ERR_NOT_SUPPORTED;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_ENTER_ISLAND
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send enter island mode command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Exit island mode operation (reconnect to grid)
  */
 esp_err_t load_controller_exit_island_mode(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_EXIT_ISLAND
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send exit island mode command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Enable power converter
  */
 esp_err_t load_controller_enable_converter(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_ENABLE_CONVERTER
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send enable converter command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Disable power converter
  */
 esp_err_t load_controller_disable_converter(void) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_DISABLE_CONVERTER
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send disable converter command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Emergency shutdown procedure
  */
 esp_err_t load_controller_emergency_shutdown(uint32_t reason) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_EMERGENCY_SHUTDOWN,
         .data.reason = reason
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send emergency shutdown command to queue");
         
         // Set emergency bit directly since this is critical
         xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_EMERGENCY_BIT);
         
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Check if charging is allowed
  */
 esp_err_t load_controller_is_charging_allowed(bool *allowed) {
     if (!s_controller.initialized || allowed == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for is_charging_allowed");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if we're in an emergency state
     if (s_controller.emergency_stop) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if converter is enabled
     if (!s_controller.status.converter_enabled) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if we're in a mode that allows charging
     if (s_controller.status.mode != LOAD_MODE_CHARGE && 
         s_controller.status.mode != LOAD_MODE_IDLE) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if grid is connected (required for charging)
     if (!s_controller.status.grid_connected) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check battery SoC
     float soc = 0.0f;
     esp_err_t result = battery_manager_get_soc(&soc);
     if (result != ESP_OK) {
         ESP_LOGW(TAG, "Failed to get SoC for charge check");
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if battery is already full
     if (soc >= s_controller.config.soc_max_percent) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // All checks passed
     *allowed = true;
     xSemaphoreGive(s_controller.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Check if discharging is allowed
  */
 esp_err_t load_controller_is_discharging_allowed(bool *allowed) {
     if (!s_controller.initialized || allowed == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for is_discharging_allowed");
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if we're in an emergency state
     if (s_controller.emergency_stop) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if converter is enabled
     if (!s_controller.status.converter_enabled) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if we're in a mode that allows discharging
     if (s_controller.status.mode != LOAD_MODE_DISCHARGE && 
         s_controller.status.mode != LOAD_MODE_IDLE &&
         s_controller.status.mode != LOAD_MODE_ISLAND) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check battery SoC
     float soc = 0.0f;
     esp_err_t result = battery_manager_get_soc(&soc);
     if (result != ESP_OK) {
         ESP_LOGW(TAG, "Failed to get SoC for discharge check");
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // Check if battery SoC is above reserve level
     if (soc <= s_controller.config.soc_reserve_percent) {
         *allowed = false;
         xSemaphoreGive(s_controller.mutex);
         return ESP_OK;
     }
     
     // All checks passed
     *allowed = true;
     xSemaphoreGive(s_controller.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set maximum power limits
  */
 esp_err_t load_controller_set_power_limits(float max_charge_kw, float max_discharge_kw) {
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Validate power limits
     if (max_charge_kw <= 0 || max_discharge_kw <= 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     load_controller_cmd_t cmd = {
         .type = CMD_SET_POWER_LIMITS,
         .data.limits = {
             .charge = max_charge_kw,
             .discharge = max_discharge_kw
         }
     };
     
     if (xQueueSend(s_controller.cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to send set power limits command to queue");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for load controller events
  */
 esp_err_t load_controller_register_event_callback(
     load_controller_event_t event,
     load_controller_event_callback_t callback,
     void *user_data) {
     
     if (!s_controller.initialized || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (event >= LOAD_EVENT_EMERGENCY_CONDITION) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for register_event_callback");
         return ESP_ERR_TIMEOUT;
     }
     
     // Find an empty slot
     bool found = false;
     for (int i = 0; i < MAX_EVENT_CALLBACKS; i++) {
         if (!s_controller.callbacks[event][i].in_use) {
             s_controller.callbacks[event][i].callback = callback;
             s_controller.callbacks[event][i].user_data = user_data;
             s_controller.callbacks[event][i].in_use = true;
             found = true;
             break;
         }
     }
     
     xSemaphoreGive(s_controller.mutex);
     
     if (!found) {
         ESP_LOGE(TAG, "No available callback slots for event %d", event);
         return ESP_ERR_NO_MEM;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a callback for load controller events
  */
 esp_err_t load_controller_unregister_event_callback(
     load_controller_event_t event,
     load_controller_event_callback_t callback) {
     
     if (!s_controller.initialized || callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (event >= LOAD_EVENT_EMERGENCY_CONDITION) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for unregister_event_callback");
         return ESP_ERR_TIMEOUT;
     }
     
     // Find the callback
     bool found = false;
     for (int i = 0; i < MAX_EVENT_CALLBACKS; i++) {
         if (s_controller.callbacks[event][i].in_use && 
             s_controller.callbacks[event][i].callback == callback) {
             s_controller.callbacks[event][i].in_use = false;
             found = true;
             break;
         }
     }
     
     xSemaphoreGive(s_controller.mutex);
     
     if (!found) {
         ESP_LOGW(TAG, "Callback not found for event %d", event);
         return ESP_ERR_NOT_FOUND;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Main task for the load controller
  */
 static void load_controller_task(void *arg) {
     ESP_LOGI(TAG, "Load controller task started");
     
     // Wait for the start signal
     EventBits_t bits = xEventGroupWaitBits(
         s_controller.event_group, 
         LOAD_CTRL_START_BIT,
         pdTRUE,  // Clear the bits after
         pdFALSE, // Wait for any bit
         portMAX_DELAY
     );
     
     if (!(bits & LOAD_CTRL_START_BIT)) {
         ESP_LOGE(TAG, "Unexpected event bits in load controller task startup");
         vTaskDelete(NULL);
         return;
     }
     
     // Initialize uptime tracking
     uint64_t last_update_time = esp_timer_get_time() / 1000;  // ms
     uint32_t update_interval = s_controller.config.power_update_interval_ms;
     
     // Main control loop
     load_controller_cmd_t cmd;
     while (1) {
         // Process any pending commands
         while (xQueueReceive(s_controller.cmd_queue, &cmd, 0) == pdTRUE) {
             load_controller_process_command(&cmd);
         }
         
         // Check for stop signal
         bits = xEventGroupGetBits(s_controller.event_group);
         if (bits & LOAD_CTRL_STOP_BIT) {
             ESP_LOGI(TAG, "Received stop signal, exiting load controller task");
             break;
         }
         
         // Check for emergency condition
         if (bits & LOAD_CTRL_EMERGENCY_BIT) {
             ESP_LOGW(TAG, "Emergency condition detected, handling");
             
             // Handle emergency by disabling converter and disconnecting from grid
             load_controller_hardware_enable_converter(false);
             load_controller_hardware_enable_grid(false);
             
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 s_controller.status.converter_enabled = false;
                 s_controller.status.grid_connected = false;
                 s_controller.status.mode = LOAD_MODE_EMERGENCY;
                 s_controller.status.power_flow = POWER_FLOW_NONE;
                 s_controller.status.current_power_kw = 0.0f;
                 
                 xSemaphoreGive(s_controller.mutex);
             }
             
             // Notify about mode change and emergency
             load_controller_notify_event(LOAD_EVENT_MODE_CHANGE, LOAD_MODE_EMERGENCY);
             load_controller_notify_event(LOAD_EVENT_EMERGENCY_CONDITION, s_controller.emergency_reason);
             
             // Clear the emergency bit
             xEventGroupClearBits(s_controller.event_group, LOAD_CTRL_EMERGENCY_BIT);
         }
         
         // Update power if needed
         uint64_t current_time = esp_timer_get_time() / 1000;  // ms
         if (current_time - last_update_time >= update_interval) {
             // Update power setpoint based on ramp rate
             if (bits & LOAD_CTRL_UPDATE_POWER_BIT) {
                 load_controller_update_power();
             }
             
             // Update status from hardware measurements
             load_controller_update_status_from_hardware();
             
             // Check safety conditions
             if (!load_controller_check_safety_conditions()) {
                 ESP_LOGW(TAG, "Safety condition check failed, triggering emergency");
                 s_controller.emergency_reason = 0x1000; // Safety check failed
                 xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_EMERGENCY_BIT);
             }
             
             // Update uptime
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                 s_controller.status.uptime_seconds = current_time / 1000;
                 xSemaphoreGive(s_controller.mutex);
             }
             
             last_update_time = current_time;
         }
         
         // Yield to other tasks
         vTaskDelay(pdMS_TO_TICKS(10));
     }
     
     // Cleanup before task exit
     ESP_LOGI(TAG, "Load controller task exiting");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Process a command from the command queue
  */
 static esp_err_t load_controller_process_command(load_controller_cmd_t *cmd) {
     if (cmd == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     ESP_LOGD(TAG, "Processing command type %d", cmd->type);
     
     switch (cmd->type) {
         case CMD_SET_MODE: {
             load_controller_mode_t new_mode = cmd->data.mode;
             load_controller_mode_t old_mode;
             
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for set_mode command");
                 return ESP_ERR_TIMEOUT;
             }
             
             old_mode = s_controller.status.mode;
             
             // Handle mode transition
             switch (new_mode) {
                 case LOAD_MODE_STANDBY:
                     // Stop all power flow and disable converter
                     s_controller.power_ramp_target = 0.0f;
                     s_controller.status.target_power_kw = 0.0f;
                     s_controller.status.current_power_kw = 0.0f;
                     s_controller.status.power_flow = POWER_FLOW_NONE;
                     s_controller.status.converter_enabled = false;
                     load_controller_hardware_set_power(0.0f);
                     load_controller_hardware_enable_converter(false);
                     break;
                     
                 case LOAD_MODE_DISCHARGE:
                     // Enable converter for discharge
                     if (!s_controller.status.converter_enabled) {
                         load_controller_hardware_enable_converter(true);
                         s_controller.status.converter_enabled = true;
                     }
                     s_controller.status.power_flow = POWER_FLOW_TO_GRID;
                     break;
                     
                 case LOAD_MODE_CHARGE:
                     // Enable converter and grid connection for charging
                     if (!s_controller.status.converter_enabled) {
                         load_controller_hardware_enable_converter(true);
                         s_controller.status.converter_enabled = true;
                     }
                     if (!s_controller.status.grid_connected) {
                         load_controller_hardware_enable_grid(true);
                         s_controller.status.grid_connected = true;
                     }
                     s_controller.status.power_flow = POWER_FLOW_FROM_GRID;
                     break;
                     
                 case LOAD_MODE_IDLE:
                     // Enable converter but set power to zero
                     if (!s_controller.status.converter_enabled) {
                         load_controller_hardware_enable_converter(true);
                         s_controller.status.converter_enabled = true;
                     }
                     s_controller.power_ramp_target = 0.0f;
                     s_controller.status.target_power_kw = 0.0f;
                     load_controller_hardware_set_power(0.0f);
                     s_controller.status.power_flow = POWER_FLOW_NONE;
                     break;
                     
                 case LOAD_MODE_ISLAND:
                     // Enter island mode (off-grid)
                     if (!s_controller.status.converter_enabled) {
                         load_controller_hardware_enable_converter(true);
                         s_controller.status.converter_enabled = true;
                     }
                     if (s_controller.status.grid_connected) {
                         load_controller_hardware_enable_grid(false);
                         s_controller.status.grid_connected = false;
                     }
                     s_controller.status.power_flow = POWER_FLOW_TO_LOAD;
                     break;
                     
                 case LOAD_MODE_EMERGENCY:
                     // Emergency shutdown
                     s_controller.power_ramp_target = 0.0f;
                     s_controller.status.target_power_kw = 0.0f;
                     s_controller.status.current_power_kw = 0.0f;
                     s_controller.status.power_flow = POWER_FLOW_NONE;
                     s_controller.status.converter_enabled = false;
                     load_controller_hardware_set_power(0.0f);
                     load_controller_hardware_enable_converter(false);
                     load_controller_hardware_enable_grid(false);
                     s_controller.status.grid_connected = false;
                     break;
                     
                 case LOAD_MODE_MAINTENANCE:
                     // Maintenance mode - similar to standby but may keep certain
                     // systems powered for service
                     s_controller.power_ramp_target = 0.0f;
                     s_controller.status.target_power_kw = 0.0f;
                     s_controller.status.current_power_kw = 0.0f;
                     s_controller.status.power_flow = POWER_FLOW_NONE;
                     s_controller.status.converter_enabled = false;
                     load_controller_hardware_set_power(0.0f);
                     load_controller_hardware_enable_converter(false);
                     break;
                     
                 case LOAD_MODE_TEST:
                     // Test mode - specific behavior depends on test requirements
                     break;
                     
                 default:
                     ESP_LOGW(TAG, "Unhandled mode transition to %d", new_mode);
                     xSemaphoreGive(s_controller.mutex);
                     return ESP_ERR_INVALID_ARG;
             }
             
             // Update mode and stats
             s_controller.status.mode = new_mode;
             s_controller.mode_change_count++;
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about mode change if it actually changed
             if (old_mode != new_mode) {
                 load_controller_notify_event(LOAD_EVENT_MODE_CHANGE, new_mode);
                 ESP_LOGI(TAG, "Mode changed from %d to %d", old_mode, new_mode);
             }
             
             break;
         }
         
         case CMD_SET_POWER: {
             float power = cmd->data.power;
             
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for set_power command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Update power targets
             s_controller.power_ramp_target = power;
             s_controller.status.target_power_kw = power;
             s_controller.last_power_update = esp_timer_get_time() / 1000;
             
             // Signal to update power in the main task loop
             xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_UPDATE_POWER_BIT);
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about power setpoint change
             load_controller_notify_event(LOAD_EVENT_POWER_SETPOINT_CHANGE, *((uint32_t*)&power));
             ESP_LOGI(TAG, "Power setpoint changed to %.2f kW", power);
             
             break;
         }
         
         case CMD_CONNECT_GRID: {
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for connect_grid command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Only connect if not already connected
             if (!s_controller.status.grid_connected) {
                 esp_err_t result = load_controller_hardware_enable_grid(true);
                 if (result == ESP_OK) {
                     s_controller.status.grid_connected = true;
                     s_controller.grid_connect_count++;
                     ESP_LOGI(TAG, "Grid connected");
                 } else {
                     ESP_LOGE(TAG, "Failed to connect grid: %s", esp_err_to_name(result));
                     s_controller.error_count++;
                     xSemaphoreGive(s_controller.mutex);
                     return result;
                 }
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about grid connection
             load_controller_notify_event(LOAD_EVENT_GRID_CONNECT, 0);
             
             break;
         }
         
         case CMD_DISCONNECT_GRID: {
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for disconnect_grid command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Only disconnect if currently connected
             if (s_controller.status.grid_connected) {
                 // First ensure power is zero
                 load_controller_hardware_set_power(0.0f);
                 s_controller.status.current_power_kw = 0.0f;
                 
                 esp_err_t result = load_controller_hardware_enable_grid(false);
                 if (result == ESP_OK) {
                     s_controller.status.grid_connected = false;
                     s_controller.grid_disconnect_count++;
                     ESP_LOGI(TAG, "Grid disconnected");
                 } else {
                     ESP_LOGE(TAG, "Failed to disconnect grid: %s", esp_err_to_name(result));
                     s_controller.error_count++;
                     xSemaphoreGive(s_controller.mutex);
                     return result;
                 }
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about grid disconnection
             load_controller_notify_event(LOAD_EVENT_GRID_DISCONNECT, 0);
             
             break;
         }
         
         case CMD_ENTER_ISLAND: {
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for enter_island command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // First ensure we're in the right mode and state
             if (s_controller.status.mode != LOAD_MODE_ISLAND) {
                 // Set island mode
                 s_controller.status.mode = LOAD_MODE_ISLAND;
                 s_controller.mode_change_count++;
                 
                 // Ensure converter is enabled for island mode
                 if (!s_controller.status.converter_enabled) {
                     load_controller_hardware_enable_converter(true);
                     s_controller.status.converter_enabled = true;
                 }
                 
                 // Disconnect from grid
                 if (s_controller.status.grid_connected) {
                     // First ensure power is zero
                     load_controller_hardware_set_power(0.0f);
                     s_controller.status.current_power_kw = 0.0f;
                     
                     load_controller_hardware_enable_grid(false);
                     s_controller.status.grid_connected = false;
                     s_controller.grid_disconnect_count++;
                 }
                 
                 s_controller.status.power_flow = POWER_FLOW_TO_LOAD;
                 
                 ESP_LOGI(TAG, "Entered island mode");
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about island mode entry
             load_controller_notify_event(LOAD_EVENT_ISLAND_MODE_ENTER, 0);
             load_controller_notify_event(LOAD_EVENT_MODE_CHANGE, LOAD_MODE_ISLAND);
             
             break;
         }
         
         case CMD_EXIT_ISLAND: {
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for exit_island command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Check if we're in island mode
             if (s_controller.status.mode == LOAD_MODE_ISLAND) {
                 // Connect to grid
                 esp_err_t result = load_controller_hardware_enable_grid(true);
                 if (result == ESP_OK) {
                     s_controller.status.grid_connected = true;
                     s_controller.grid_connect_count++;
                     
                     // Change to idle mode
                     s_controller.status.mode = LOAD_MODE_IDLE;
                     s_controller.mode_change_count++;
                     s_controller.status.power_flow = POWER_FLOW_NONE;
                     
                     ESP_LOGI(TAG, "Exited island mode");
                 } else {
                     ESP_LOGE(TAG, "Failed to connect grid when exiting island mode: %s", 
                              esp_err_to_name(result));
                     s_controller.error_count++;
                     xSemaphoreGive(s_controller.mutex);
                     return result;
                 }
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about island mode exit
             load_controller_notify_event(LOAD_EVENT_ISLAND_MODE_EXIT, 0);
             load_controller_notify_event(LOAD_EVENT_MODE_CHANGE, LOAD_MODE_IDLE);
             
             break;
         }
         
         case CMD_ENABLE_CONVERTER: {
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for enable_converter command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Only enable if not already enabled
             if (!s_controller.status.converter_enabled) {
                 esp_err_t result = load_controller_hardware_enable_converter(true);
                 if (result == ESP_OK) {
                     s_controller.status.converter_enabled = true;
                     ESP_LOGI(TAG, "Converter enabled");
                 } else {
                     ESP_LOGE(TAG, "Failed to enable converter: %s", esp_err_to_name(result));
                     s_controller.error_count++;
                     xSemaphoreGive(s_controller.mutex);
                     return result;
                 }
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about converter enable
             load_controller_notify_event(LOAD_EVENT_CONVERTER_ENABLE, 0);
             
             break;
         }
         
         case CMD_DISABLE_CONVERTER: {
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for disable_converter command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Only disable if currently enabled
             if (s_controller.status.converter_enabled) {
                 // First ensure power is zero
                 load_controller_hardware_set_power(0.0f);
                 s_controller.status.current_power_kw = 0.0f;
                 
                 esp_err_t result = load_controller_hardware_enable_converter(false);
                 if (result == ESP_OK) {
                     s_controller.status.converter_enabled = false;
                     ESP_LOGI(TAG, "Converter disabled");
                 } else {
                     ESP_LOGE(TAG, "Failed to disable converter: %s", esp_err_to_name(result));
                     s_controller.error_count++;
                     xSemaphoreGive(s_controller.mutex);
                     return result;
                 }
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about converter disable
             load_controller_notify_event(LOAD_EVENT_CONVERTER_DISABLE, 0);
             
             break;
         }
         
         case CMD_EMERGENCY_SHUTDOWN: {
             uint32_t reason = cmd->data.reason;
             
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for emergency_shutdown command");
                 // Continue with emergency shutdown even if mutex fails
             } else {
                 s_controller.emergency_stop = true;
                 s_controller.emergency_reason = reason;
                 xSemaphoreGive(s_controller.mutex);
             }
             
             // Set emergency bit to trigger immediate handling in main task
             xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_EMERGENCY_BIT);
             
             ESP_LOGW(TAG, "Emergency shutdown initiated, reason: 0x%08x", reason);
             break;
         }
         
         case CMD_SET_POWER_LIMITS: {
             float max_charge = cmd->data.limits.charge;
             float max_discharge = cmd->data.limits.discharge;
             
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to take mutex for set_power_limits command");
                 return ESP_ERR_TIMEOUT;
             }
             
             // Update configuration
             s_controller.config.max_charge_power_kw = max_charge;
             s_controller.config.max_discharge_power_kw = max_discharge;
             
             ESP_LOGI(TAG, "Power limits updated: max charge=%.2f kW, max discharge=%.2f kW",
                      max_charge, max_discharge);
             
             // Check if current power setpoint exceeds new limits
             if (s_controller.status.target_power_kw > max_discharge) {
                 s_controller.power_ramp_target = max_discharge;
                 s_controller.status.target_power_kw = max_discharge;
                 xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_UPDATE_POWER_BIT);
                 ESP_LOGW(TAG, "Current power setpoint exceeds new discharge limit, reducing to %.2f kW",
                          max_discharge);
             } else if (s_controller.status.target_power_kw < -max_charge) {
                 s_controller.power_ramp_target = -max_charge;
                 s_controller.status.target_power_kw = -max_charge;
                 xEventGroupSetBits(s_controller.event_group, LOAD_CTRL_UPDATE_POWER_BIT);
                 ESP_LOGW(TAG, "Current power setpoint exceeds new charge limit, reducing to %.2f kW",
                          -max_charge);
             }
             
             xSemaphoreGive(s_controller.mutex);
             
             // Notify about power limit change using a dummy data value
             uint32_t limit_data = (uint32_t)((max_discharge * 10) + (max_charge / 10));
             load_controller_notify_event(LOAD_EVENT_POWER_LIMIT, limit_data);
             
             break;
         }
         
         default:
             ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
             return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Update power setpoint based on ramp rate
  */
 static void load_controller_update_power(void) {
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for update_power");
         return;
     }
     
     // Get current time in milliseconds
     uint64_t now = esp_timer_get_time() / 1000;
     uint64_t elapsed_ms = now - s_controller.last_power_update;
     s_controller.last_power_update = now;
     
     // Calculate maximum change based on ramp rate and elapsed time
     float max_delta = s_controller.config.ramp_rate_kw_per_sec * (elapsed_ms / 1000.0f);
     
     // Calculate needed change to reach target
     float current = s_controller.status.current_power_kw;
     float target = s_controller.power_ramp_target;
     float delta = target - current;
     
     // Limit change to maximum ramp rate
     if (fabs(delta) > max_delta) {
         delta = (delta > 0) ? max_delta : -max_delta;
     }
     
     // Update power
     float new_power = current + delta;
     
     // Apply the new power setpoint to hardware
     esp_err_t result = load_controller_hardware_set_power(new_power);
     
     if (result == ESP_OK) {
         s_controller.status.current_power_kw = new_power;
         
         // Update power flow direction
         if (new_power > 0.0001f) {
             // Discharging
             if (s_controller.status.grid_connected) {
                 s_controller.status.power_flow = POWER_FLOW_TO_GRID;
             } else {
                 s_controller.status.power_flow = POWER_FLOW_TO_LOAD;
             }
         } else if (new_power < -0.0001f) {
             // Charging
             s_controller.status.power_flow = POWER_FLOW_FROM_GRID;
         } else {
             // No significant power flow
             s_controller.status.power_flow = POWER_FLOW_NONE;
         }
         
         // Clear the update bit if we've reached the target
         if (fabs(new_power - target) < 0.01f) {
             xEventGroupClearBits(s_controller.event_group, LOAD_CTRL_UPDATE_POWER_BIT);
             ESP_LOGI(TAG, "Power ramping complete, reached %.2f kW", new_power);
         }
     } else {
         ESP_LOGE(TAG, "Failed to set hardware power: %s", esp_err_to_name(result));
         s_controller.error_count++;
     }
     
     xSemaphoreGive(s_controller.mutex);
 }
 
 /**
  * @brief Update status from hardware measurements
  */
 static void load_controller_update_status_from_hardware(void) {
     // In a real implementation, this would read values from sensors and hardware
     // For this placeholder, we'll just simulate some values
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
         return;  // Skip this update if mutex is busy
     }
     
     // Simulate reading from power meter
     float simulated_power_delta = ((float)esp_random() / UINT32_MAX - 0.5f) * 0.5f;
     s_controller.status.measured_power_kw = s_controller.status.current_power_kw + simulated_power_delta;
     
     // Simulate grid and load power
     if (s_controller.status.grid_connected) {
         if (s_controller.status.power_flow == POWER_FLOW_TO_GRID) {
             s_controller.status.grid_power_kw = -s_controller.status.measured_power_kw;
             s_controller.status.load_power_kw = 0.0f;
         } else if (s_controller.status.power_flow == POWER_FLOW_FROM_GRID) {
             s_controller.status.grid_power_kw = -s_controller.status.measured_power_kw;
             s_controller.status.load_power_kw = 0.0f;
         } else {
             s_controller.status.grid_power_kw = 0.0f;
             s_controller.status.load_power_kw = 0.0f;
         }
     } else {
         s_controller.status.grid_power_kw = 0.0f;
         s_controller.status.load_power_kw = s_controller.status.measured_power_kw;
     }
     
     // Read DC bus voltage
     s_controller.status.dc_bus_voltage = 750.0f + ((float)esp_random() / UINT32_MAX - 0.5f) * 10.0f;
     
     xSemaphoreGive(s_controller.mutex);
 }
 
 /**
  * @brief Check safety conditions for continued operation
  * 
  * @return true Safety conditions are met
  * @return false Safety conditions are violated
  */
 static bool load_controller_check_safety_conditions(void) {
     bool safe = true;
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
         // If we can't check safety, assume unsafe
         return false;
     }
     
     // Check DC bus voltage
     if (s_controller.status.dc_bus_voltage > 850.0f || 
         s_controller.status.dc_bus_voltage < 650.0f) {
         ESP_LOGW(TAG, "DC bus voltage out of range: %.2f V", 
                  s_controller.status.dc_bus_voltage);
         safe = false;
     }
     
     // Check for excessive power deviation
     float power_deviation = fabs(s_controller.status.measured_power_kw - 
                                 s_controller.status.current_power_kw);
     if (power_deviation > 5.0f) {
         ESP_LOGW(TAG, "Excessive power deviation: %.2f kW vs %.2f kW", 
                  s_controller.status.measured_power_kw,
                  s_controller.status.current_power_kw);
         safe = false;
     }
     
     xSemaphoreGive(s_controller.mutex);
     
     return safe;
 }
 
 /**
  * @brief Notify registered callbacks about an event
  */
 static void load_controller_notify_event(load_controller_event_t event, uint32_t data) {
     if (event >= LOAD_EVENT_EMERGENCY_CONDITION) {
         ESP_LOGW(TAG, "Invalid event type %d for notification", event);
         return;
     }
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for notify_event");
         return;
     }
     
     // Call all registered callbacks for this event
     for (int i = 0; i < MAX_EVENT_CALLBACKS; i++) {
         if (s_controller.callbacks[event][i].in_use) {
             // Get callback info
             load_controller_event_callback_t callback = s_controller.callbacks[event][i].callback;
             void *user_data = s_controller.callbacks[event][i].user_data;
             
             // Release mutex before calling callback to avoid deadlocks
             xSemaphoreGive(s_controller.mutex);
             
             // Call the callback
             callback(event, data, user_data);
             
             // Retake mutex to continue iteration
             if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                 ESP_LOGE(TAG, "Failed to retake mutex after callback");
                 return;
             }
         }
     }
     
     xSemaphoreGive(s_controller.mutex);
 }
 
 /**
  * @brief Validate configuration parameters for load controller
  */
 static esp_err_t load_controller_validate_config(const load_controller_config_t *config) {
     // Check power limits
     if (config->max_charge_power_kw <= 0 || config->max_discharge_power_kw <= 0) {
         ESP_LOGE(TAG, "Invalid power limits: charge=%.2f, discharge=%.2f",
                  config->max_charge_power_kw, config->max_discharge_power_kw);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check minimum power setpoint
     if (config->min_power_setpoint_kw < 0) {
         ESP_LOGE(TAG, "Invalid minimum power setpoint: %.2f", config->min_power_setpoint_kw);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check ramp rate
     if (config->ramp_rate_kw_per_sec <= 0) {
         ESP_LOGE(TAG, "Invalid ramp rate: %.2f", config->ramp_rate_kw_per_sec);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check update interval
     if (config->power_update_interval_ms < 10 || config->power_update_interval_ms > 10000) {
         ESP_LOGE(TAG, "Invalid power update interval: %lu", config->power_update_interval_ms);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check SoC limits
     if (config->soc_reserve_percent >= config->soc_max_percent ||
         config->soc_max_percent > 100) {
         ESP_LOGE(TAG, "Invalid SoC limits: reserve=%u%%, max=%u%%",
                  config->soc_reserve_percent, config->soc_max_percent);
         return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Initialize hardware interfaces for the load controller
  */
 static esp_err_t load_controller_hardware_init(void) {
     ESP_LOGI(TAG, "Initializing load controller hardware");
     
     // In a real implementation, this would:
     // 1. Set up GPIO pins for control signals
     // 2. Initialize SPI/I2C/UART interfaces for power converter communication
     // 3. Configure ADCs for power/voltage measurement
     // 4. Initialize contactors for grid connection
     
     // For this placeholder implementation, we'll just simulate success
     
     // Set up virtual GPIO for grid connection contactor
     // gpio_config_t io_conf = {
     //     .pin_bit_mask = (1ULL << GRID_CONTACTOR_PIN),
     //     .mode = GPIO_MODE_OUTPUT,
     //     .pull_up_en = GPIO_PULLUP_DISABLE,
     //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
     //     .intr_type = GPIO_INTR_DISABLE
     // };
     // gpio_config(&io_conf);
     // gpio_set_level(GRID_CONTACTOR_PIN, 0);  // Start with grid disconnected
     
     // Simulate hardware initialization delay
     vTaskDelay(pdMS_TO_TICKS(100));
     
     ESP_LOGI(TAG, "Hardware initialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Clean up hardware resources used by the load controller
  */
 static void load_controller_hardware_deinit(void) {
     ESP_LOGI(TAG, "Deinitializing load controller hardware");
     
     // Ensure converter is disabled
     load_controller_hardware_enable_converter(false);
     
     // Ensure grid is disconnected
     load_controller_hardware_enable_grid(false);
     
     // In a real implementation, this would:
     // 1. Release GPIO pins
     // 2. Close SPI/I2C/UART interfaces
     // 3. Disable ADCs
     
     ESP_LOGI(TAG, "Hardware deinitialized");
 }
 
 /**
  * @brief Set power setpoint in hardware
  */
 static esp_err_t load_controller_hardware_set_power(float power_kw) {
     // In a real implementation, this would:
     // 1. Convert power setpoint to appropriate format for converter
     // 2. Send command to converter via communication interface
     // 3. Verify command was accepted
     
     // For this placeholder, we'll just log the action
     ESP_LOGD(TAG, "Hardware: Setting power to %.2f kW", power_kw);
     
     // Simulate communication delay
     vTaskDelay(pdMS_TO_TICKS(5));
     
     // Simulate success
     return ESP_OK;
 }
 
 /**
  * @brief Enable or disable grid connection in hardware
  */
 static esp_err_t load_controller_hardware_enable_grid(bool enable) {
     // In a real implementation, this would:
     // 1. Control grid connection contactor
     // 2. Verify contactor state
     
     // For this placeholder, we'll just log the action
     ESP_LOGI(TAG, "Hardware: %s grid connection", enable ? "Enabling" : "Disabling");
     
     // Simulate contactor operation
     // gpio_set_level(GRID_CONTACTOR_PIN, enable ? 1 : 0);
     
     // Simulate contactor operation delay
     vTaskDelay(pdMS_TO_TICKS(100));
     
     // Simulate success
     return ESP_OK;
 }
 
 /**
  * @brief Enable or disable power converter in hardware
  */
 static esp_err_t load_controller_hardware_enable_converter(bool enable) {
     // In a real implementation, this would:
     // 1. Send enable/disable command to converter
     // 2. Verify converter state
     
     // For this placeholder, we'll just log the action
     ESP_LOGI(TAG, "Hardware: %s power converter", enable ? "Enabling" : "Disabling");
     
     // Simulate communication and operation delay
     vTaskDelay(pdMS_TO_TICKS(50));
     
     // Simulate success
     return ESP_OK;
 }
 
     if (!s_controller.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // For now, this function doesn't use the results parameter,
     // but it could be extended to populate diagnostic information
     
     ESP_LOGI(TAG, "Running load controller diagnostics");
     
     if (xSemaphoreTake(s_controller.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex for run_diagnostics");
         return ESP_ERR_TIMEOUT;
     }
     
     // Print diagnostic information
     ESP_LOGI(TAG, "Status: Mode=%d, Power=%.2fkW, Grid=%s, Converter=%s",
             s_controller.status.mode,
             s_controller.status.current_power_kw,
             s_controller.status.grid_connected ? "Connected" : "Disconnected",
             s_controller.status.converter_enabled ? "Enabled" : "Disabled");
     
     ESP_LOGI(TAG, "Stats: Grid connects=%lu, disconnects=%lu, mode changes=%lu, errors=%lu",
             s_controller.grid_connect_count,
             s_controller.grid_disconnect_count,
             s_controller.mode_change_count,
             s_controller.error_count);
     
     xSemaphoreGive(s_controller.mutex);
     
     // Run hardware self-test
     // This would involve checking communication with converters, 
     // validating sensor readings, and testing control paths
     
     // Run converter communication check
     bool converter_ok = true;  // Placeholder for actual check
     if (!converter_ok) {
         ESP_LOGW(TAG, "Converter communication check failed");
         return ESP_ERR_INVALID_RESPONSE;
     }
     
     // Test grid connection circuitry
     bool grid_ctrl_ok = true;  // Placeholder for actual check
     if (!grid_ctrl_ok) {
         ESP_LOGW(TAG, "Grid connection control check failed");
         return ESP_ERR_INVALID_RESPONSE;
     }
     
     // For a full implementation, we would run more comprehensive tests here
     
     ESP_LOGI(TAG, "Diagnostics completed successfully");
     return ESP_OK;
 }