/**
 * @file power_manager.c
 * @brief Implementation of power management system for BESS
 * 
 * Implements power flow control, converter operations, charge/discharge
 * management, and grid interaction for the 100KW/200KWH BESS.
 */

 #include "power_manager.h"
 #include "esp_log.h"
 #include "esp_timer.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "bess_types.h"
 #include "battery_manager.h"
 #include "logger.h"
 
 static const char *TAG = "power_manager";
 
 /* Constants */
 #define MAX_SCHEDULE_ENTRIES 16
 #define POWER_MONITOR_TASK_STACK_SIZE 4096
 #define POWER_MONITOR_TASK_PRIORITY 5
 #define POWER_MONITOR_INTERVAL_MS 500
 #define MAX_EVENT_CALLBACKS 10
 #define POWER_TASK_DELAY_MS 10
 #define POWER_RAMP_RATE_KW_PER_S 10.0f  // Power ramp rate in kW/s
 #define POWER_ERROR_MARGIN 0.5f          // Error margin for power control (kW)
 
 /* FreeRTOS event bits */
 #define EVENT_EMERGENCY        (1 << 0)
 #define EVENT_GRID_CONNECTED   (1 << 1)
 #define EVENT_MODE_CHANGED     (1 << 2)
 #define EVENT_SCHEDULE_ACTIVE  (1 << 3)
 #define EVENT_STOP_REQUEST     (1 << 4)
 #define EVENT_ALL              (0x1F)
 
 /* Power manager state structure */
 typedef struct {
     power_manager_config_t config;
     power_mode_t current_mode;
     power_flow_t current_flow;
     charge_params_t charge_params;
     discharge_params_t discharge_params;
     power_stats_t stats;
     grid_connection_t grid_connection;
     bool is_running;
     bool scheduling_enabled;
     bool emergency_state;
     uint32_t mode_transition_time;
     
     // For scheduling
     power_schedule_entry_t schedules[MAX_SCHEDULE_ENTRIES];
     uint8_t active_schedule_id;
     bool schedule_active;
     
     // Target power settings
     float target_power_kw;
     uint32_t operation_end_time;
     
     // Callback storage
     struct {
         int32_t event_type;
         power_event_callback_t callback;
         void *user_data;
     } callbacks[MAX_EVENT_CALLBACKS];
     uint8_t callback_count;
     
     // Task and synchronization
     TaskHandle_t monitor_task_handle;
     SemaphoreHandle_t mutex;
     EventGroupHandle_t event_group;
 } power_manager_state_t;
 
 /* Static variables */
 static power_manager_state_t s_state = {0};
 
 /* Function prototypes for internal functions */
 static void power_monitor_task(void *pvParameters);
 static void handle_emergency(void);
 static void trigger_event(power_event_t event, void *event_data);
 static esp_err_t set_power_level(float power_kw);
 static esp_err_t validate_power_level(float power_kw);
 static esp_err_t update_power_flow(void);
 static esp_err_t process_schedules(void);
 static bool is_schedule_active(const power_schedule_entry_t *schedule);
 static void activate_schedule(uint8_t schedule_id);
 static void deactivate_schedule(void);
 
 /**
  * @brief Initialize the power manager
  */
 esp_err_t power_manager_init(const power_manager_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex and event group
     s_state.mutex = xSemaphoreCreateMutex();
     if (s_state.mutex == NULL) {
         LOG_ERROR("Failed to create power manager mutex");
         return ESP_ERR_NO_MEM;
     }
     
     s_state.event_group = xEventGroupCreate();
     if (s_state.event_group == NULL) {
         vSemaphoreDelete(s_state.mutex);
         LOG_ERROR("Failed to create power manager event group");
         return ESP_ERR_NO_MEM;
     }
     
     // Initialize the state
     memcpy(&s_state.config, config, sizeof(power_manager_config_t));
     s_state.current_mode = POWER_MODE_STANDBY;
     s_state.current_flow = POWER_FLOW_NONE;
     s_state.grid_connection = config->grid_connection;
     s_state.is_running = false;
     s_state.scheduling_enabled = false;
     s_state.emergency_state = false;
     s_state.mode_transition_time = 0;
     s_state.target_power_kw = 0.0f;
     s_state.operation_end_time = 0;
     s_state.active_schedule_id = 0xFF;
     s_state.schedule_active = false;
     s_state.callback_count = 0;
     
     // Initialize schedules
     memset(s_state.schedules, 0, sizeof(s_state.schedules));
     
     // Set default charge parameters
     s_state.charge_params.charge_current_max = 100.0f;    // Default to 100A
     s_state.charge_params.charge_voltage_max = 54.0f * config->module_count;  // 54V per module max
     s_state.charge_params.charge_power_max = config->max_charge_power;
     s_state.charge_params.method = CHARGE_METHOD_CC_CV;
     s_state.charge_params.enable_balancing = true;
     s_state.charge_params.timeout_minutes = 0;           // No timeout
     s_state.charge_params.target_soc = 100.0f;           // Default to 100%
     s_state.charge_params.scheduled = false;
     
     // Set default discharge parameters
     s_state.discharge_params.discharge_current_max = 100.0f;    // Default to 100A
     s_state.discharge_params.discharge_power_max = config->max_discharge_power;
     s_state.discharge_params.min_voltage = 44.0f * config->module_count;  // 44V per module min
     s_state.discharge_params.min_soc = config->min_soc;
     s_state.discharge_params.timeout_minutes = 0;        // No timeout
     s_state.discharge_params.scheduled = false;
     
     // Initialize stats
     memset(&s_state.stats, 0, sizeof(power_stats_t));
     s_state.stats.current_mode = POWER_MODE_STANDBY;
     s_state.stats.flow_direction = POWER_FLOW_NONE;
     
     LOG_INFO("Power manager initialized successfully");
     
     return ESP_OK;
 }
 
 /* Internal function implementations */
 
 /**
  * @brief Power monitoring task
  */
 static void power_monitor_task(void *pvParameters) {
     uint32_t last_update_time = 0;
     uint32_t last_uptime_update = 0;
     
     LOG_INFO("Power monitor task started");
     
     while (1) {
         // Check for stop request
         EventBits_t bits = xEventGroupGetBits(s_state.event_group);
         if ((bits & EVENT_STOP_REQUEST) != 0) {
             LOG_INFO("Power monitor task stopping due to stop request");
             break;
         }
         
         // Get current time
         uint32_t current_time = esp_timer_get_time() / 1000;
         
         // Update statistics
         if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             // Update uptime every second
             if (current_time - last_uptime_update >= 1000) {
                 s_state.stats.uptime_seconds++;
                 last_uptime_update = current_time;
             }
             
             // Check for operation timeout
             if (s_state.operation_end_time > 0 && current_time >= s_state.operation_end_time) {
                 // Operation has timed out
                 LOG_INFO("Operation timeout reached");
                 
                 // Save the current mode
                 power_mode_t current_mode = s_state.current_mode;
                 
                 // Set to IDLE mode
                 s_state.current_mode = POWER_MODE_IDLE;
                 s_state.stats.current_mode = POWER_MODE_IDLE;
                 s_state.target_power_kw = 0.0f;
                 s_state.operation_end_time = 0;
                 
                 // Determine which event to trigger
                 power_event_t timeout_event = POWER_EVENT_MODE_CHANGE;
                 if (current_mode == POWER_MODE_CHARGE) {
                     timeout_event = POWER_EVENT_CHARGE_COMPLETE;
                 } else if (current_mode == POWER_MODE_DISCHARGE) {
                     timeout_event = POWER_EVENT_DISCHARGE_COMPLETE;
                 }
                 
                 // Set mode changed event
                 xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
                 
                 xSemaphoreGive(s_state.mutex);
                 
                 // Update power flow
                 update_power_flow();
                 
                 // Set power level to zero
                 set_power_level(0.0f);
                 
                 // Trigger events
                 trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_IDLE);
                 trigger_event(timeout_event, NULL);
                 
                 // Skip to next iteration
                 vTaskDelay(pdMS_TO_TICKS(POWER_MONITOR_INTERVAL_MS));
                 continue;
             }
             
             // Process schedules
             if (s_state.scheduling_enabled && (bits & EVENT_EMERGENCY) == 0) {
                 if (process_schedules() != ESP_OK) {
                     LOG_WARN("Error processing schedules");
                 }
             }
             
             xSemaphoreGive(s_state.mutex);
         }
         
         // Check if we need to update the power level
         if (current_time - last_update_time >= POWER_MONITOR_INTERVAL_MS) {
             last_update_time = current_time;
             
             // Get current target power
             float target_power = 0.0f;
             if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 target_power = s_state.target_power_kw;
                 xSemaphoreGive(s_state.mutex);
             }
             
             // Only update if not in emergency state
             if ((bits & EVENT_EMERGENCY) == 0) {
                 set_power_level(target_power);
             }
         }
         
         // Delay until next iteration
         vTaskDelay(pdMS_TO_TICKS(POWER_TASK_DELAY_MS));
     }
     
     // Task is ending, clear the event flag
     xEventGroupClearBits(s_state.event_group, EVENT_STOP_REQUEST);
     
     // Task cleanup
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) == pdTRUE) {
         s_state.monitor_task_handle = NULL;
         s_state.is_running = false;
         xSemaphoreGive(s_state.mutex);
     }
     
     LOG_INFO("Power monitor task terminated");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Handle emergency conditions
  */
 static void handle_emergency(void) {
     LOG_ERROR("Emergency handler activated");
     
     // Immediately stop power flow
     set_power_level(0.0f);
     
     // Set emergency mode
     power_manager_emergency_shutdown();
 }
 
 /**
  * @brief Trigger event callbacks
  */
 static void trigger_event(power_event_t event, void *event_data) {
     // Make a local copy of callbacks to avoid holding the mutex during callback execution
     struct {
         int32_t event_type;
         power_event_callback_t callback;
         void *user_data;
     } callbacks[MAX_EVENT_CALLBACKS];
     uint8_t callback_count = 0;
     
     // Get callbacks under mutex protection
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         callback_count = s_state.callback_count;
         memcpy(callbacks, s_state.callbacks, sizeof(callbacks));
         xSemaphoreGive(s_state.mutex);
     }
     
     // Call each registered callback
     for (int i = 0; i < callback_count; i++) {
         // Call if this callback is for all events or this specific event
         if (callbacks[i].event_type == -1 || callbacks[i].event_type == event) {
             callbacks[i].callback(event, event_data, callbacks[i].user_data);
         }
     }
 }
 
 /**
  * @brief Set power level with ramping if needed
  */
 static esp_err_t set_power_level(float power_kw) {
     // Simple validation
     if (isnan(power_kw) || isinf(power_kw)) {
         LOG_ERROR("Invalid power level: %.1f", power_kw);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Get current stats and target
     power_stats_t current_stats;
     float target_power = power_kw;
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         current_stats = s_state.stats;
         
         // Check emergency state
         if (s_state.emergency_state) {
             target_power = 0.0f;
         }
         
         // Apply power limits based on mode
         if (target_power < 0.0f) {
             // Charging (negative power)
             if (-target_power > s_state.charge_params.charge_power_max) {
                 target_power = -s_state.charge_params.charge_power_max;
                 LOG_WARN("Limiting charge power to %.1f kW", s_state.charge_params.charge_power_max);
             }
         } else if (target_power > 0.0f) {
             // Discharging (positive power)
             if (target_power > s_state.discharge_params.discharge_power_max) {
                 target_power = s_state.discharge_params.discharge_power_max;
                 LOG_WARN("Limiting discharge power to %.1f kW", s_state.discharge_params.discharge_power_max);
             }
         }
         
         xSemaphoreGive(s_state.mutex);
     } else {
         return ESP_ERR_TIMEOUT;
     }
     
     // Calculate ramp rate
     float current_power = current_stats.instantaneous_power;
     float power_delta = target_power - current_power;
     
     // If power change is small, apply directly
     if (fabs(power_delta) < POWER_ERROR_MARGIN) {
         // Update the actual power setting in the hardware
         // This would involve commands to inverter, converters, etc.
         // For now, we'll just update the stats
         
         if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             s_state.stats.instantaneous_power = target_power;
             xSemaphoreGive(s_state.mutex);
         }
         
         // Only log if there's a meaningful change
         if (fabs(power_delta) > 0.1f) {
             LOG_INFO("Power set to %.1f kW", target_power);
         }
     } else {
         // Apply ramp rate limit
         float max_delta = POWER_RAMP_RATE_KW_PER_S * (POWER_MONITOR_INTERVAL_MS / 1000.0f);
         
         if (fabs(power_delta) > max_delta) {
             if (power_delta > 0) {
                 target_power = current_power + max_delta;
             } else {
                 target_power = current_power - max_delta;
             }
             LOG_INFO("Ramping power from %.1f to %.1f kW (towards %.1f kW)", 
                      current_power, target_power, power_kw);
         } else {
             LOG_INFO("Power set to %.1f kW", target_power);
         }
         
         // Update the actual power setting in the hardware
         // This would involve commands to inverter, converters, etc.
         // For now, we'll just update the stats
         
         if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             s_state.stats.instantaneous_power = target_power;
             xSemaphoreGive(s_state.mutex);
         }
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Validate power level against system limits
  */
 static esp_err_t validate_power_level(float power_kw) {
     if (isnan(power_kw) || isinf(power_kw)) {
         LOG_ERROR("Invalid power level: not a number");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check against system limits
     if (power_kw < 0.0f) {
         // Charging (negative power)
         if (-power_kw > s_state.config.max_charge_power) {
             xSemaphoreGive(s_state.mutex);
             LOG_ERROR("Requested charge power %.1f kW exceeds system limit %.1f kW", 
                      -power_kw, s_state.config.max_charge_power);
             return ESP_ERR_INVALID_ARG;
         }
     } else if (power_kw > 0.0f) {
         // Discharging (positive power)
         if (power_kw > s_state.config.max_discharge_power) {
             xSemaphoreGive(s_state.mutex);
             LOG_ERROR("Requested discharge power %.1f kW exceeds system limit %.1f kW", 
                      power_kw, s_state.config.max_discharge_power);
             return ESP_ERR_INVALID_ARG;
         }
     }
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Update power flow based on current state
  */
 static esp_err_t update_power_flow(void) {
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     power_flow_t old_flow = s_state.current_flow;
     power_flow_t new_flow = POWER_FLOW_NONE;
     
     // Determine flow based on mode and grid connection
     switch (s_state.current_mode) {
         case POWER_MODE_CHARGE:
             if (s_state.grid_connection != GRID_CONNECTION_ISOLATED) {
                 new_flow = POWER_FLOW_GRID_TO_BATTERY;
             }
             break;
             
         case POWER_MODE_DISCHARGE:
             if (s_state.stats.load_power > 0.0f) {
                 if (s_state.stats.load_power >= s_state.stats.instantaneous_power) {
                     // Load consumes all battery output
                     new_flow = POWER_FLOW_BATTERY_TO_LOAD;
                 } else {
                     // Excess power goes to grid if allowed
                     if (s_state.grid_connection != GRID_CONNECTION_ISOLATED && 
                         s_state.config.allow_grid_export) {
                         new_flow = POWER_FLOW_BATTERY_TO_GRID;
                     } else {
                         new_flow = POWER_FLOW_BATTERY_TO_LOAD;
                     }
                 }
             } else if (s_state.grid_connection != GRID_CONNECTION_ISOLATED && 
                        s_state.config.allow_grid_export) {
                 // No local load, all to grid
                 new_flow = POWER_FLOW_BATTERY_TO_GRID;
             }
             break;
             
         case POWER_MODE_STANDBY:
         case POWER_MODE_IDLE:
             if (s_state.stats.load_power > 0.0f && s_state.grid_connection != GRID_CONNECTION_ISOLATED) {
                 new_flow = POWER_FLOW_GRID_TO_LOAD;
             } else {
                 new_flow = POWER_FLOW_NONE;
             }
             break;
             
         case POWER_MODE_EMERGENCY:
         case POWER_MODE_MAINTENANCE:
             new_flow = POWER_FLOW_NONE;
             break;
     }
     
     // Update the flow if changed
     if (new_flow != old_flow) {
         s_state.current_flow = new_flow;
         s_state.stats.flow_direction = new_flow;
         
         LOG_INFO("Power flow changed from %d to %d", old_flow, new_flow);
         
         // Save a local copy of the new flow
         power_flow_t flow_for_event = new_flow;
         
         xSemaphoreGive(s_state.mutex);
         
         // Trigger flow change event
         trigger_event(POWER_EVENT_FLOW_CHANGE, (void*)(intptr_t)flow_for_event);
     } else {
         xSemaphoreGive(s_state.mutex);
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Process schedules and activate if needed
  */
 static esp_err_t process_schedules(void) {
     // Get current time components
     time_t now;
     struct tm timeinfo;
     time(&now);
     localtime_r(&now, &timeinfo);
     
     uint8_t current_day = timeinfo.tm_wday;  // 0-6, Sunday=0
     uint8_t current_hour = timeinfo.tm_hour; // 0-23
     uint8_t current_minute = timeinfo.tm_min; // 0-59
     
     // Check if we need to deactivate current schedule
     if (s_state.schedule_active) {
         const power_schedule_entry_t *current_schedule = &s_state.schedules[s_state.active_schedule_id];
         
         // Calculate end time
         uint16_t end_hour = current_schedule->hour;
         uint16_t end_minute = current_schedule->minute + current_schedule->duration_minutes;
         
         // Adjust for overflow
         end_hour += end_minute / 60;
         end_minute %= 60;
         end_hour %= 24;
         
         // Check if schedule period has ended
         bool schedule_ended = false;
         
         if (current_hour > end_hour || 
             (current_hour == end_hour && current_minute >= end_minute)) {
             schedule_ended = true;
         }
         
         if (schedule_ended) {
             // Deactivate the schedule
             deactivate_schedule();
         }
     }
     
     // Don't check for new schedules if we have an active one
     if (s_state.schedule_active) {
         return ESP_OK;
     }
     
     // Check all schedules
     for (int i = 0; i < MAX_SCHEDULE_ENTRIES; i++) {
         if (!s_state.schedules[i].enabled) {
             continue;
         }
         
         // Check if this schedule is currently active
         if (is_schedule_active(&s_state.schedules[i])) {
             // Activate this schedule
             activate_schedule(i);
             break;
         }
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Check if a schedule is currently active
  */
 static bool is_schedule_active(const power_schedule_entry_t *schedule) {
     if (!schedule->enabled) {
         return false;
     }
     
     // Get current time
     time_t now;
     struct tm timeinfo;
     time(&now);
     localtime_r(&now, &timeinfo);
     
     uint8_t current_day = timeinfo.tm_wday;  // 0-6, Sunday=0
     uint8_t current_hour = timeinfo.tm_hour; // 0-23
     uint8_t current_minute = timeinfo.tm_min; // 0-59
     
     // Check day match
     bool day_match = (schedule->day_of_week == 7) || (schedule->day_of_week == current_day);
     if (!day_match) {
         return false;
     }
     
     // Calculate start and end times
     uint16_t start_hour = schedule->hour;
     uint16_t start_minute = schedule->minute;
     
     uint16_t end_hour = start_hour;
     uint16_t end_minute = start_minute + schedule->duration_minutes;
     
     // Adjust for overflow
     end_hour += end_minute / 60;
     end_minute %= 60;
     end_hour %= 24;
     
     // Check if current time is within schedule period
     if (current_hour < start_hour || 
         (current_hour == start_hour && current_minute < start_minute)) {
         return false;
     }
     
     if (current_hour > end_hour || 
         (current_hour == end_hour && current_minute >= end_minute)) {
         return false;
     }
     
     return true;
 }
 
 /**
  * @brief Activate a schedule
  */
 static void activate_schedule(uint8_t schedule_id) {
     if (schedule_id >= MAX_SCHEDULE_ENTRIES || !s_state.schedules[schedule_id].enabled) {
         LOG_ERROR("Invalid schedule ID: %u", schedule_id);
         return;
     }
     
     const power_schedule_entry_t *schedule = &s_state.schedules[schedule_id];
     
     LOG_INFO("Activating schedule %u: mode=%d, power=%.1f kW, duration=%u min", 
              schedule_id, schedule->mode, schedule->power_setpoint, 
              schedule->duration_minutes);
     
     // Set the active schedule
     s_state.active_schedule_id = schedule_id;
     s_state.schedule_active = true;
     
     // Set schedule active event bit
     xEventGroupSetBits(s_state.event_group, EVENT_SCHEDULE_ACTIVE);
     
     // Set mode based on schedule
     power_mode_t old_mode = s_state.current_mode;
     s_state.current_mode = schedule->mode;
     s_state.stats.current_mode = schedule->mode;
     
     // Calculate end time
     s_state.operation_end_time = esp_timer_get_time() / 1000 + 
                                (schedule->duration_minutes * 60 * 1000);
     
     // Set target power based on mode
     if (schedule->mode == POWER_MODE_CHARGE) {
         s_state.target_power_kw = -1.0f * fabs(schedule->power_setpoint);
         s_state.charge_params.scheduled = true;
     } else if (schedule->mode == POWER_MODE_DISCHARGE) {
         s_state.target_power_kw = fabs(schedule->power_setpoint);
         s_state.discharge_params.scheduled = true;
     } else {
         s_state.target_power_kw = 0.0f;
     }
     
     // Set mode changed event if needed
     if (old_mode != schedule->mode) {
         xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
     }
     
     // Trigger events
     trigger_event(POWER_EVENT_SCHEDULE_STARTED, (void*)(intptr_t)schedule_id);
     if (old_mode != schedule->mode) {
         trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)schedule->mode);
     }
     
     // Update power flow
     update_power_flow();
 }
 
 /**
  * @brief Deactivate the current schedule
  */
 static void deactivate_schedule(void) {
     if (!s_state.schedule_active) {
         return;
     }
     
     uint8_t schedule_id = s_state.active_schedule_id;
     LOG_INFO("Deactivating schedule %u", schedule_id);
     
     // Clear schedule active
     s_state.schedule_active = false;
     s_state.active_schedule_id = 0xFF;
     
     // Clear schedule active event bit
     xEventGroupClearBits(s_state.event_group, EVENT_SCHEDULE_ACTIVE);
     
     // Set to IDLE mode
     power_mode_t old_mode = s_state.current_mode;
     s_state.current_mode = POWER_MODE_IDLE;
     s_state.stats.current_mode = POWER_MODE_IDLE;
     
     // Reset target power and timeouts
     s_state.target_power_kw = 0.0f;
     s_state.operation_end_time = 0;
     
     // Reset scheduled flags
     s_state.charge_params.scheduled = false;
     s_state.discharge_params.scheduled = false;
     
     // Set mode changed event if needed
     if (old_mode != POWER_MODE_IDLE) {
         xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
     }
     
     // Trigger events
     trigger_event(POWER_EVENT_SCHEDULE_ENDED, (void*)(intptr_t)schedule_id);
     if (old_mode != POWER_MODE_IDLE) {
         trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_IDLE);
     }
     
     // Update power flow
     update_power_flow();
 }
 
 /* Internal function implementations */
 
 /**
  * @brief Power monitoring task
  */
 static void power_monitor_task(void *pvParameters) {
     uint32_t last_update_time = 0;
     uint32_t last_uptime_update = 0;
     
     LOG_INFO("Power monitor task started");
     
     while (1) {
         // Check for stop request
         EventBits_t bits = xEventGroupGetBits(s_state.event_group);
         if ((bits & EVENT_STOP_REQUEST) != 0) {
             LOG_INFO("Power monitor task stopping due to stop request");
             break;
         }
         
         // Get current time
         uint32_t current_time = esp_timer_get_time() / 1000;
         
         // Update statistics
         if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             // Update uptime every second
             if (current_time - last_uptime_update >= 1000) {
                 s_state.stats.uptime_seconds++;
                 last_uptime_update = current_time;
             }
             
             // Check for operation timeout
             if (s_state.operation_end_time > 0 && current_time >= s_state.operation_end_time) {
                 // Operation has timed out
                 LOG_INFO("Operation timeout reached");
                 
                 // Save the current mode
                 power_mode_t current_mode = s_state.current_mode;
                 
                 // Set to IDLE mode
                 s_state.current_mode = POWER_MODE_IDLE;
                 s_state.stats.current_mode = POWER_MODE_IDLE;
                 s_state.target_power_kw = 0.0f;
                 s_state.operation_end_time = 0;
                 
                 // Determine which event to trigger
                 power_event_t timeout_event = POWER_EVENT_MODE_CHANGE;
                 if (current_mode == POWER_MODE_CHARGE) {
                     timeout_event = POWER_EVENT_CHARGE_COMPLETE;
                 } else if (current_mode == POWER_MODE_DISCHARGE) {
                     timeout_event = POWER_EVENT_DISCHARGE_COMPLETE;
                 }
                 
                 // Set mode changed event
                 xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
                 
                 xSemaphoreGive(s_state.mutex);
                 
                 // Update power flow
                 update_power_flow();
                 
                 // Set power level to zero
                 set_power_level(0.0f);
                 
                 // Trigger events
                 trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_IDLE);
                 trigger_event(timeout_event, NULL);
                 
                 // Skip to next iteration
                 vTaskDelay(pdMS_TO_TICKS(POWER_MONITOR_INTERVAL_MS));
                 continue;
             }
             
             // Process schedules
             if (s_state.scheduling_enabled && (bits & EVENT_EMERGENCY) == 0) {
                 if (process_schedules() != ESP_OK) {
                     LOG_WARN("Error processing schedules");
                 }
             }
             
             xSemaphoreGive(s_state.mutex);
         }
         
         // Check if we need to update the power level
         if (current_time - last_update_time >= POWER_MONITOR_INTERVAL_MS) {
             last_update_time = current_time;
             
             // Get current target power
             float target_power = 0.0f;
             if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 target_power = s_state.target_power_kw;
                 xSemaphoreGive(s_state.mutex);
             }
             
             // Only update if not in emergency state
             if ((bits & EVENT_EMERGENCY) == 0) {
                 set_power_level(target_power);
             }
         }
         
         // Delay until next iteration
         vTaskDelay(pdMS_TO_TICKS(POWER_TASK_DELAY_MS));
     }
     
     // Task is ending, clear the event flag
     xEventGroupClearBits(s_state.event_group, EVENT_STOP_REQUEST);
     
     // Task cleanup
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) == pdTRUE) {
         s_state.monitor_task_handle = NULL;
         s_state.is_running = false;
         xSemaphoreGive(s_state.mutex);
     }
     
     LOG_INFO("Power monitor task terminated");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Handle emergency conditions
  */
 static void handle_emergency(void) {
     LOG_ERROR("Emergency handler activated");
     
     // Immediately stop power flow
     set_power_level(0.0f);
     
     // Set emergency mode
     power_manager_emergency_shutdown();
 }
 
 /**
  * @brief Trigger event callbacks
  */
 static void trigger_event(power_event_t event, void *event_data) {
     // Make a local copy of callbacks to avoid holding the mutex during callback execution
     struct {
         int32_t event_type;
         power_event_callback_t callback;
         void *user_data;
     } callbacks[MAX_EVENT_CALLBACKS];
     uint8_t callback_count = 0;
     
     // Get callbacks under mutex protection
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         callback_count = s_state.callback_count;
         memcpy(callbacks, s_state.callbacks, sizeof(callbacks));
         xSemaphoreGive(s_state.mutex);
     }
     
     // Call each registered callback
     for (int i = 0; i < callback_count; i++) {
         // Call if this callback is for all events or this specific event
         if (callbacks[i].event_type == -1 || callbacks[i].event_type == event) {
             callbacks[i].callback(event, event_data, callbacks[i].user_data);
         }
     }
 }
 
 /**
  * @brief Set power level with ramping if needed
  */
 static esp_err_t set_power_level(float power_kw) {
     // Simple validation
     if (isnan(power_kw) || isinf(power_kw)) {
         LOG_ERROR("Invalid power level: %.1f", power_kw);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Get current stats and target
     power_stats_t current_stats;
     float target_power = power_kw;
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         current_stats = s_state.stats;
         
         // Check emergency state
         if (s_state.emergency_state) {
             target_power = 0.0f;
         }
         
         // Apply power limits based on mode
         if (target_power < 0.0f) {
             // Charging (negative power)
             if (-target_power > s_state.charge_params.charge_power_max) {
                 target_power = -s_state.charge_params.charge_power_max;
                 LOG_WARN("Limiting charge power to %.1f kW", s_state.charge_params.charge_power_max);
             }
         } else if (target_power > 0.0f) {
             // Discharging (positive power)
             if (target_power > s_state.discharge_params.discharge_power_max) {
                 target_power = s_state.discharge_params.discharge_power_max;
                 LOG_WARN("Limiting discharge power to %.1f kW", s_state.discharge_params.discharge_power_max);
             }
         }
         
         xSemaphoreGive(s_state.mutex);
     } else {
         return ESP_ERR_TIMEOUT;
     }
     
     // Calculate ramp rate
     float current_power = current_stats.instantaneous_power;
     float power_delta = target_power - current_power;
     
     // If power change is small, apply directly
     if (fabs(power_delta) < POWER_ERROR_MARGIN) {
         // Update the actual power setting in the hardware
         // This would involve commands to inverter, converters, etc.
         // For now, we'll just update the stats
         
         if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             s_state.stats.instantaneous_power = target_power;
             xSemaphoreGive(s_state.mutex);
         }
         
         // Only log if there's a meaningful change
         if (fabs(power_delta) > 0.1f) {
             LOG_INFO("Power set to %.1f kW", target_power);
         }
     } else {
         // Apply ramp rate limit
         float max_delta = POWER_RAMP_RATE_KW_PER_S * (POWER_MONITOR_INTERVAL_MS / 1000.0f);
         
         if (fabs(power_delta) > max_delta) {
             if (power_delta > 0) {
                 target_power = current_power + max_delta;
             } else {
                 target_power = current_power - max_delta;
             }
             LOG_INFO("Ramping power from %.1f to %.1f kW (towards %.1f kW)", 
                      current_power, target_power, power_kw);
         } else {
             LOG_INFO("Power set to %.1f kW", target_power);
         }
         
         // Update the actual power setting in the hardware
         // This would involve commands to inverter, converters, etc.
         // For now, we'll just update the stats
         
         if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             s_state.stats.instantaneous_power = target_power;
             xSemaphoreGive(s_state.mutex);
         }
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Validate power level against system limits
  */
 static esp_err_t validate_power_level(float power_kw) {
     if (isnan(power_kw) || isinf(power_kw)) {
         LOG_ERROR("Invalid power level: not a number");
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check against system limits
     if (power_kw < 0.0f) {
         // Charging (negative power)
         if (-power_kw > s_state.config.max_charge_power) {
             xSemaphoreGive(s_state
 
 /**
  * @brief Delete a power schedule entry
  */
 esp_err_t power_manager_delete_schedule(uint8_t entry_id) {
     if (entry_id >= MAX_SCHEDULE_ENTRIES) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if the entry exists
     if (!s_state.schedules[entry_id].enabled) {
         xSemaphoreGive(s_state.mutex);
         return ESP_ERR_NOT_FOUND;
     }
     
     // Check if this is the active schedule
     if (s_state.schedule_active && s_state.active_schedule_id == entry_id) {
         deactivate_schedule();
     }
     
     // Mark as disabled
     s_state.schedules[entry_id].enabled = false;
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Deleted schedule entry %u", entry_id);
     
     return ESP_OK;
 }
 
 /**
  * @brief Get a power schedule entry
  */
 esp_err_t power_manager_get_schedule(uint8_t entry_id, power_schedule_entry_t *entry) {
     if (entry == NULL || entry_id >= MAX_SCHEDULE_ENTRIES) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if the entry exists
     if (!s_state.schedules[entry_id].enabled) {
         xSemaphoreGive(s_state.mutex);
         return ESP_ERR_NOT_FOUND;
     }
     
     // Copy the schedule
     memcpy(entry, &s_state.schedules[entry_id], sizeof(power_schedule_entry_t));
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get all schedule entries
  */
 esp_err_t power_manager_get_all_schedules(power_schedule_entry_t *entries, 
                                        uint8_t max_entries,
                                        uint8_t *num_entries) {
     if (entries == NULL || num_entries == NULL || max_entries == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     uint8_t count = 0;
     
     // Copy enabled entries
     for (int i = 0; i < MAX_SCHEDULE_ENTRIES && count < max_entries; i++) {
         if (s_state.schedules[i].enabled) {
             memcpy(&entries[count], &s_state.schedules[i], sizeof(power_schedule_entry_t));
             count++;
         }
     }
     
     *num_entries = count;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Enable or disable all scheduling
  */
 esp_err_t power_manager_enable_scheduling(bool enable) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     bool old_state = s_state.scheduling_enabled;
     s_state.scheduling_enabled = enable;
     
     // If disabling and we have an active schedule, deactivate it
     if (!enable && s_state.schedule_active) {
         deactivate_schedule();
     }
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Scheduling %s", enable ? "enabled" : "disabled");
     
     return ESP_OK;
 }
 
 /**
  * @brief Check if scheduling is enabled
  */
 esp_err_t power_manager_is_scheduling_enabled(bool *enabled) {
     if (enabled == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     *enabled = s_state.scheduling_enabled;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set constant power output/input
  */
 esp_err_t power_manager_set_constant_power(float power_kw, uint32_t duration_minutes) {
     // Validate power level
     esp_err_t err = validate_power_level(power_kw);
     if (err != ESP_OK) {
         return err;
     }
     
     bool is_emergency = false;
     if (power_manager_is_emergency(&is_emergency) == ESP_OK && is_emergency) {
         LOG_ERROR("Cannot set power level in emergency state");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Set target power
     s_state.target_power_kw = power_kw;
     
     // Set operation end time if duration specified
     if (duration_minutes > 0) {
         s_state.operation_end_time = (esp_timer_get_time() / 1000) + 
                                    (duration_minutes * 60 * 1000);
     } else {
         s_state.operation_end_time = 0;  // No timeout
     }
     
     // Set appropriate mode based on power direction
     power_mode_t new_mode;
     if (power_kw < -POWER_ERROR_MARGIN) {
         // Negative power means charging
         new_mode = POWER_MODE_CHARGE;
         s_state.charge_params.scheduled = false;
     } else if (power_kw > POWER_ERROR_MARGIN) {
         // Positive power means discharging
         new_mode = POWER_MODE_DISCHARGE;
         s_state.discharge_params.scheduled = false;
     } else {
         // Zero or near-zero power means idle
         new_mode = POWER_MODE_IDLE;
     }
     
     // Change mode if needed
     if (s_state.current_mode != new_mode) {
         power_mode_t old_mode = s_state.current_mode;
         s_state.current_mode = new_mode;
         s_state.stats.current_mode = new_mode;
         s_state.mode_transition_time = esp_timer_get_time() / 1000;
         
         // Set mode changed event
         xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
         
         LOG_INFO("Power mode changed from %d to %d due to constant power command", 
                 old_mode, new_mode);
     }
     
     xSemaphoreGive(s_state.mutex);
     
     // Update power flow
     update_power_flow();
     
     // Set the power level
     set_power_level(power_kw);
     
     LOG_INFO("Set constant power to %.1f kW for %u minutes", 
              power_kw, duration_minutes);
     
     return ESP_OK;
 }
 
 /**
  * @brief Emergency shutdown
  */
 esp_err_t power_manager_emergency_shutdown(void) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Only proceed if not already in emergency state
     if (!s_state.emergency_state) {
         s_state.emergency_state = true;
         
         // Save previous mode
         power_mode_t old_mode = s_state.current_mode;
         
         // Set emergency mode
         s_state.current_mode = POWER_MODE_EMERGENCY;
         s_state.stats.current_mode = POWER_MODE_EMERGENCY;
         
         // Stop power flow
         s_state.target_power_kw = 0.0f;
         
         // Set emergency event
         xEventGroupSetBits(s_state.event_group, EVENT_EMERGENCY);
         
         xSemaphoreGive(s_state.mutex);
         
         // Immediately set power to zero
         set_power_level(0.0f);
         
         LOG_ERROR("!!! EMERGENCY SHUTDOWN INITIATED !!!");
         
         // Trigger events
         trigger_event(POWER_EVENT_EMERGENCY, NULL);
         if (old_mode != POWER_MODE_EMERGENCY) {
             trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_EMERGENCY);
         }
     } else {
         xSemaphoreGive(s_state.mutex);
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Reset after emergency
  */
 esp_err_t power_manager_reset_emergency(void) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Only proceed if in emergency state
     if (s_state.emergency_state) {
         s_state.emergency_state = false;
         
         // Clear emergency event
         xEventGroupClearBits(s_state.event_group, EVENT_EMERGENCY);
         
         // Set to idle mode
         s_state.current_mode = POWER_MODE_IDLE;
         s_state.stats.current_mode = POWER_MODE_IDLE;
         s_state.mode_transition_time = esp_timer_get_time() / 1000;
         
         // Set mode changed event
         xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
         
         xSemaphoreGive(s_state.mutex);
         
         LOG_INFO("Emergency state reset, system in idle mode");
         
         // Trigger mode change event
         trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_IDLE);
     } else {
         xSemaphoreGive(s_state.mutex);
         LOG_WARN("System not in emergency state");
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Check if system is in emergency state
  */
 esp_err_t power_manager_is_emergency(bool *is_emergency) {
     if (is_emergency == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     *is_emergency = s_state.emergency_state;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set grid connection mode
  */
 esp_err_t power_manager_set_grid_connection(grid_connection_t mode) {
     if (mode < GRID_CONNECTION_ISOLATED || mode > GRID_CONNECTION_EXPORT) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     grid_connection_t old_mode = s_state.grid_connection;
     s_state.grid_connection = mode;
     s_state.config.grid_connection = mode;
     
     // Update grid connected event bit
     if (mode == GRID_CONNECTION_ISOLATED) {
         xEventGroupClearBits(s_state.event_group, EVENT_GRID_CONNECTED);
     } else {
         xEventGroupSetBits(s_state.event_group, EVENT_GRID_CONNECTED);
     }
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Grid connection mode changed from %d to %d", old_mode, mode);
     
     // Trigger appropriate events
     if (old_mode == GRID_CONNECTION_ISOLATED && mode != GRID_CONNECTION_ISOLATED) {
         trigger_event(POWER_EVENT_GRID_CONNECTED, NULL);
     } else if (old_mode != GRID_CONNECTION_ISOLATED && mode == GRID_CONNECTION_ISOLATED) {
         trigger_event(POWER_EVENT_GRID_DISCONNECTED, NULL);
     }
     
     // Update power flow based on new grid connection mode
     update_power_flow();
     
     return ESP_OK;
 }
 
 /**
  * @brief Get current grid connection mode
  */
 esp_err_t power_manager_get_grid_connection(grid_connection_t *mode) {
     if (mode == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     *mode = s_state.grid_connection;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Update current power limits based on battery state
  */
 esp_err_t power_manager_update_power_limits(float soc, 
                                          float max_charge_current,
                                          float max_discharge_current) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Update charge parameters
     s_state.charge_params.charge_current_max = max_charge_current;
     
     // Update discharge parameters
     s_state.discharge_params.discharge_current_max = max_discharge_current;
     
     // Calculate power limits based on nominal system voltage
     float system_voltage = s_state.config.system_voltage_nominal;
     float max_charge_power = (max_charge_current * system_voltage) / 1000.0f;  // Convert to kW
     float max_discharge_power = (max_discharge_current * system_voltage) / 1000.0f;  // Convert to kW
     
     // Limit to system maximums
     if (max_charge_power > s_state.config.max_charge_power) {
         max_charge_power = s_state.config.max_charge_power;
     }
     if (max_discharge_power > s_state.config.max_discharge_power) {
         max_discharge_power = s_state.config.max_discharge_power;
     }
     
     // Apply SoC limitations
     // Reduce charge power as we approach max SoC
     if (soc > s_state.config.max_soc - 5.0f) {
         float factor = (s_state.config.max_soc - soc) / 5.0f;
         if (factor < 0.0f) factor = 0.0f;
         max_charge_power *= factor;
     }
     
     // Reduce discharge power as we approach min SoC
     if (soc < s_state.config.min_soc + 5.0f) {
         float factor = (soc - s_state.config.min_soc) / 5.0f;
         if (factor < 0.0f) factor = 0.0f;
         max_discharge_power *= factor;
     }
     
     // Update the power limits
     s_state.charge_params.charge_power_max = max_charge_power;
     s_state.discharge_params.discharge_power_max = max_discharge_power;
     
     // If we are already charging or discharging, we may need to adjust power
     if (s_state.current_mode == POWER_MODE_CHARGE) {
         float target_power = -1.0f * max_charge_power;
         if (s_state.target_power_kw < target_power) {
             s_state.target_power_kw = target_power;
         }
     } else if (s_state.current_mode == POWER_MODE_DISCHARGE) {
         if (s_state.target_power_kw > max_discharge_power) {
             s_state.target_power_kw = max_discharge_power;
         }
     }
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Updated power limits based on SoC=%.1f%%: charge=%.1f kW, discharge=%.1f kW", 
              soc, max_charge_power, max_discharge_power);
     
     return ESP_OK;
 }
 
 /**
  * @brief Start the power manager
  */
 esp_err_t power_manager_start(void) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     if (s_state.is_running) {
         xSemaphoreGive(s_state.mutex);
         return ESP_OK;  // Already running
     }
     
     // Clear all event bits
     xEventGroupClearBits(s_state.event_group, EVENT_ALL);
     
     // Create monitor task
     BaseType_t res = xTaskCreate(power_monitor_task, 
                                 "power_monitor", 
                                 POWER_MONITOR_TASK_STACK_SIZE, 
                                 NULL, 
                                 POWER_MONITOR_TASK_PRIORITY, 
                                 &s_state.monitor_task_handle);
     
     if (res != pdPASS) {
         xSemaphoreGive(s_state.mutex);
         LOG_ERROR("Failed to create power monitor task");
         return ESP_ERR_NO_MEM;
     }
     
     s_state.is_running = true;
     s_state.stats.uptime_seconds = 0;
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Power manager started");
     
     return ESP_OK;
 }
 
 /**
  * @brief Stop the power manager
  */
 esp_err_t power_manager_stop(void) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     if (!s_state.is_running) {
         xSemaphoreGive(s_state.mutex);
         return ESP_OK;  // Already stopped
     }
     
     // Set stop request bit
     xEventGroupSetBits(s_state.event_group, EVENT_STOP_REQUEST);
     
     // Wait for task to stop
     xSemaphoreGive(s_state.mutex);
     
     // Wait for task to acknowledge stop request
     EventBits_t bits = xEventGroupWaitBits(s_state.event_group, 
                                          EVENT_STOP_REQUEST, 
                                          pdFALSE, 
                                          pdTRUE, 
                                          pdMS_TO_TICKS(5000));
     
     if ((bits & EVENT_STOP_REQUEST) == 0) {
         LOG_WARN("Timeout waiting for power monitor task to stop");
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Delete task if it's still running
     if (s_state.monitor_task_handle != NULL) {
         vTaskDelete(s_state.monitor_task_handle);
         s_state.monitor_task_handle = NULL;
     }
     
     s_state.is_running = false;
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Power manager stopped");
     
     return ESP_OK;
 }
 
 /**
  * @brief Set the operation mode
  */
 esp_err_t power_manager_set_mode(power_mode_t mode) {
     if (mode < POWER_MODE_STANDBY || mode > POWER_MODE_MAINTENANCE) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Can't change mode if in emergency state
     if (s_state.emergency_state && mode != POWER_MODE_EMERGENCY) {
         xSemaphoreGive(s_state.mutex);
         LOG_WARN("Cannot change mode while in emergency state");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (s_state.current_mode != mode) {
         power_mode_t old_mode = s_state.current_mode;
         s_state.current_mode = mode;
         s_state.stats.current_mode = mode;
         s_state.mode_transition_time = esp_timer_get_time() / 1000;
         
         // Set mode changed event
         xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
         
         LOG_INFO("Power mode changed from %d to %d", old_mode, mode);
         
         // Trigger event
         power_event_t event = POWER_EVENT_MODE_CHANGE;
         void *event_data = (void*)(intptr_t)mode;
         
         xSemaphoreGive(s_state.mutex);
         trigger_event(event, event_data);
         
         // Update power flow based on new mode
         update_power_flow();
         
         return ESP_OK;
     }
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get the current operation mode
  */
 esp_err_t power_manager_get_mode(power_mode_t *mode) {
     if (mode == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     *mode = s_state.current_mode;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set charging parameters
  */
 esp_err_t power_manager_set_charge_params(const charge_params_t *params) {
     if (params == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     memcpy(&s_state.charge_params, params, sizeof(charge_params_t));
     
     // Ensure limits are within system capabilities
     if (s_state.charge_params.charge_power_max > s_state.config.max_charge_power) {
         s_state.charge_params.charge_power_max = s_state.config.max_charge_power;
         LOG_WARN("Charge power limited to system maximum of %.1f kW", s_state.config.max_charge_power);
     }
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Charge parameters updated: max_current=%.1fA, max_voltage=%.1fV, max_power=%.1fkW, target_soc=%.1f%%",
              params->charge_current_max, params->charge_voltage_max, 
              params->charge_power_max, params->target_soc);
     
     return ESP_OK;
 }
 
 /**
  * @brief Set discharging parameters
  */
 esp_err_t power_manager_set_discharge_params(const discharge_params_t *params) {
     if (params == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     memcpy(&s_state.discharge_params, params, sizeof(discharge_params_t));
     
     // Ensure limits are within system capabilities
     if (s_state.discharge_params.discharge_power_max > s_state.config.max_discharge_power) {
         s_state.discharge_params.discharge_power_max = s_state.config.max_discharge_power;
         LOG_WARN("Discharge power limited to system maximum of %.1f kW", s_state.config.max_discharge_power);
     }
     
     // Ensure minimum SoC is respected
     if (s_state.discharge_params.min_soc < s_state.config.min_soc) {
         s_state.discharge_params.min_soc = s_state.config.min_soc;
         LOG_WARN("Minimum SoC limited to system limit of %.1f%%", s_state.config.min_soc);
     }
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Discharge parameters updated: max_current=%.1fA, max_power=%.1fkW, min_voltage=%.1fV, min_soc=%.1f%%",
              params->discharge_current_max, params->discharge_power_max, 
              params->min_voltage, params->min_soc);
     
     return ESP_OK;
 }
 
 /**
  * @brief Start charging operation
  */
 esp_err_t power_manager_start_charging(float target_soc) {
     bool is_emergency = false;
     if (power_manager_is_emergency(&is_emergency) == ESP_OK && is_emergency) {
         LOG_ERROR("Cannot start charging in emergency state");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if grid charging is allowed
     if (s_state.grid_connection == GRID_CONNECTION_ISOLATED || 
         (s_state.grid_connection == GRID_CONNECTION_EXPORT && !s_state.config.allow_grid_charging)) {
         xSemaphoreGive(s_state.mutex);
         LOG_ERROR("Grid charging not allowed in current configuration");
         return ESP_ERR_NOT_SUPPORTED;
     }
     
     // Update target SoC if specified
     if (target_soc > 0.0f) {
         if (target_soc > 100.0f) {
             target_soc = 100.0f;
         } else if (target_soc < s_state.config.min_soc) {
             target_soc = s_state.config.min_soc;
         }
         s_state.charge_params.target_soc = target_soc;
     }
     
     // Set operation end time if timeout is specified
     if (s_state.charge_params.timeout_minutes > 0) {
         s_state.operation_end_time = (esp_timer_get_time() / 1000) + 
                                     (s_state.charge_params.timeout_minutes * 60 * 1000);
     } else {
         s_state.operation_end_time = 0;  // No timeout
     }
     
     // Calculate appropriate power level based on parameters
     float power_kw = -1.0f * s_state.charge_params.charge_power_max;  // Negative for charging
     
     // Set target power
     s_state.target_power_kw = power_kw;
     
     // Switch to charge mode
     s_state.current_mode = POWER_MODE_CHARGE;
     s_state.stats.current_mode = POWER_MODE_CHARGE;
     s_state.mode_transition_time = esp_timer_get_time() / 1000;
     
     // Mark as non-scheduled if not already set
     if (!s_state.charge_params.scheduled) {
         s_state.charge_params.scheduled = false;
     }
     
     // Set mode changed event
     xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
     
     xSemaphoreGive(s_state.mutex);
     
     // Update power flow
     update_power_flow();
     
     LOG_INFO("Started charging operation with target SoC=%.1f%%, power=%.1fkW", 
              s_state.charge_params.target_soc, fabs(power_kw));
     
     // Trigger event
     trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_CHARGE);
     
     return ESP_OK;
 }
 
 /**
  * @brief Start discharging operation
  */
 esp_err_t power_manager_start_discharging(float power_kw, uint32_t duration_minutes) {
     bool is_emergency = false;
     if (power_manager_is_emergency(&is_emergency) == ESP_OK && is_emergency) {
         LOG_ERROR("Cannot start discharging in emergency state");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Verify we can export to grid if needed
     if (s_state.grid_connection == GRID_CONNECTION_ISOLATED || 
         (s_state.grid_connection == GRID_CONNECTION_BACKUP && !s_state.config.allow_grid_export)) {
         // Only allow if we have a local load
         if (s_state.stats.load_power < power_kw) {
             xSemaphoreGive(s_state.mutex);
             LOG_ERROR("Grid export not allowed in current configuration");
             return ESP_ERR_NOT_SUPPORTED;
         }
     }
     
     // Use default power if not specified
     if (power_kw <= 0.0f) {
         power_kw = s_state.discharge_params.discharge_power_max;
     }
     
     // Limit to maximum discharge power
     if (power_kw > s_state.discharge_params.discharge_power_max) {
         power_kw = s_state.discharge_params.discharge_power_max;
         LOG_WARN("Discharge power limited to maximum of %.1f kW", power_kw);
     }
     
     // Use specified duration or default
     if (duration_minutes > 0) {
         s_state.operation_end_time = (esp_timer_get_time() / 1000) + 
                                    (duration_minutes * 60 * 1000);
         s_state.discharge_params.timeout_minutes = duration_minutes;
     } else if (s_state.discharge_params.timeout_minutes > 0) {
         s_state.operation_end_time = (esp_timer_get_time() / 1000) + 
                                    (s_state.discharge_params.timeout_minutes * 60 * 1000);
     } else {
         s_state.operation_end_time = 0;  // No timeout
     }
     
     // Set target power (positive for discharging)
     s_state.target_power_kw = power_kw;
     
     // Switch to discharge mode
     s_state.current_mode = POWER_MODE_DISCHARGE;
     s_state.stats.current_mode = POWER_MODE_DISCHARGE;
     s_state.mode_transition_time = esp_timer_get_time() / 1000;
     
     // Mark as non-scheduled if not already set
     if (!s_state.discharge_params.scheduled) {
         s_state.discharge_params.scheduled = false;
     }
     
     // Set mode changed event
     xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
     
     xSemaphoreGive(s_state.mutex);
     
     // Update power flow
     update_power_flow();
     
     LOG_INFO("Started discharging operation with power=%.1fkW, duration=%u minutes", 
              power_kw, duration_minutes > 0 ? duration_minutes : s_state.discharge_params.timeout_minutes);
     
     // Trigger event
     trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_DISCHARGE);
     
     return ESP_OK;
 }
 
 /**
  * @brief Stop the current operation (charging or discharging)
  */
 esp_err_t power_manager_stop_operation(void) {
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     power_mode_t old_mode = s_state.current_mode;
     
     // Only need to change if we're in charge or discharge mode
     if (old_mode == POWER_MODE_CHARGE || old_mode == POWER_MODE_DISCHARGE) {
         // Set to idle mode
         s_state.current_mode = POWER_MODE_IDLE;
         s_state.stats.current_mode = POWER_MODE_IDLE;
         s_state.mode_transition_time = esp_timer_get_time() / 1000;
         
         // Reset target power and timeouts
         s_state.target_power_kw = 0.0f;
         s_state.operation_end_time = 0;
         
         // Clear any active schedule
         if (s_state.schedule_active) {
             deactivate_schedule();
         }
         
         // Set mode changed event
         xEventGroupSetBits(s_state.event_group, EVENT_MODE_CHANGED);
         
         xSemaphoreGive(s_state.mutex);
         
         // Update power flow
         update_power_flow();
         
         LOG_INFO("Stopped %s operation", 
                 old_mode == POWER_MODE_CHARGE ? "charging" : "discharging");
         
         // Trigger appropriate events
         trigger_event(POWER_EVENT_MODE_CHANGE, (void*)(intptr_t)POWER_MODE_IDLE);
         
         if (old_mode == POWER_MODE_CHARGE) {
             trigger_event(POWER_EVENT_CHARGE_COMPLETE, NULL);
         } else if (old_mode == POWER_MODE_DISCHARGE) {
             trigger_event(POWER_EVENT_DISCHARGE_COMPLETE, NULL);
         }
         
         return ESP_OK;
     }
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;  // No operation was running
 }
 
 /**
  * @brief Get power statistics
  */
 esp_err_t power_manager_get_stats(power_stats_t *stats) {
     if (stats == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     memcpy(stats, &s_state.stats, sizeof(power_stats_t));
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for power events
  */
 esp_err_t power_manager_register_callback(int32_t event_type, 
                                        power_event_callback_t callback,
                                        void *user_data) {
     if (callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if we have room for another callback
     if (s_state.callback_count >= MAX_EVENT_CALLBACKS) {
         xSemaphoreGive(s_state.mutex);
         LOG_ERROR("Maximum number of callbacks already registered");
         return ESP_ERR_NO_MEM;
     }
     
     // Add the callback
     s_state.callbacks[s_state.callback_count].event_type = event_type;
     s_state.callbacks[s_state.callback_count].callback = callback;
     s_state.callbacks[s_state.callback_count].user_data = user_data;
     s_state.callback_count++;
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Registered callback for event type %d", event_type);
     
     return ESP_OK;
 }
 
 /**
  * @brief Unregister a previously registered callback
  */
 esp_err_t power_manager_unregister_callback(int32_t event_type,
                                          power_event_callback_t callback) {
     if (callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     bool found = false;
     
     // Find and remove the callback
     for (int i = 0; i < s_state.callback_count; i++) {
         // Check if this is the callback we want to remove
         if (s_state.callbacks[i].event_type == event_type && 
             s_state.callbacks[i].callback == callback) {
             found = true;
             
             // Remove by shifting remaining elements
             for (int j = i; j < s_state.callback_count - 1; j++) {
                 s_state.callbacks[j] = s_state.callbacks[j + 1];
             }
             
             s_state.callback_count--;
             break;
         }
     }
     
     xSemaphoreGive(s_state.mutex);
     
     if (!found) {
         LOG_WARN("Callback for event type %d not found", event_type);
         return ESP_ERR_NOT_FOUND;
     }
     
     LOG_INFO("Unregistered callback for event type %d", event_type);
     
     return ESP_OK;
 }
 
 /**
  * @brief Add a power schedule entry
  */
 esp_err_t power_manager_add_schedule(const power_schedule_entry_t *entry, uint8_t *entry_id) {
     if (entry == NULL || entry_id == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Find an empty slot
     uint8_t id = 0xFF;
     for (int i = 0; i < MAX_SCHEDULE_ENTRIES; i++) {
         if (!s_state.schedules[i].enabled) {
             id = i;
             break;
         }
     }
     
     if (id == 0xFF) {
         xSemaphoreGive(s_state.mutex);
         LOG_ERROR("No empty slots for new schedule");
         return ESP_ERR_NO_MEM;
     }
     
     // Store the schedule
     memcpy(&s_state.schedules[id], entry, sizeof(power_schedule_entry_t));
     s_state.schedules[id].enabled = true;
     *entry_id = id;
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Added schedule entry %u: day=%u, time=%02u:%02u, mode=%d, power=%.1f kW", 
              id, entry->day_of_week, entry->hour, entry->minute, 
              entry->mode, entry->power_setpoint);
     
     return ESP_OK;
 }
 
 /**
  * @brief Update a power schedule entry
  */
 esp_err_t power_manager_update_schedule(uint8_t entry_id, const power_schedule_entry_t *entry) {
     if (entry == NULL || entry_id >= MAX_SCHEDULE_ENTRIES) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE) {
         return ESP_ERR_TIMEOUT;
     }
     
     // Check if the entry exists
     if (!s_state.schedules[entry_id].enabled) {
         xSemaphoreGive(s_state.mutex);
         LOG_ERROR("Schedule entry %u not found", entry_id);
         return ESP_ERR_NOT_FOUND;
     }
     
     // Check if this is the active schedule
     bool was_active = (s_state.schedule_active && s_state.active_schedule_id == entry_id);
     
     // Update the schedule
     memcpy(&s_state.schedules[entry_id], entry, sizeof(power_schedule_entry_t));
     s_state.schedules[entry_id].enabled = true;
     
     // If this was the active schedule, we may need to deactivate it
     if (was_active && !is_schedule_active(&s_state.schedules[entry_id])) {
         deactivate_schedule();
     }
     
     xSemaphoreGive(s_state.mutex);
     
     LOG_INFO("Updated schedule entry %u: day=%u, time=%02u:%02u, mode=%d, power=%.1f kW", 
              entry_id, entry->day_of_week, entry->hour, entry->minute, 
              entry->mode, entry->power_setpoint);
     
     return ESP_OK;
 }