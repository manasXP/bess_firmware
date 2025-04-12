/**
 * @file battery_manager.c
 * @brief Implementation of battery management system for BESS
 * The implementation includes several key features:
 *
 * 1. Real-time battery data monitoring
 * 2. Cell voltage and temperature limit checking
 * 3. Automatic cell balancing
 * 4. Thermal management
 * 5. State of Charge/Health estimation
 * 6. Event notification system
 * 7. Advanced diagnostics and fault detection
 */

 #include <string.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 
 #include "esp_log.h"
 #include "esp_err.h"
 #include "driver/gpio.h"
 #include "driver/adc.h"
 #include "driver/i2c.h"
 #include "esp_adc_cal.h"
 #include "esp_timer.h"
 
 #include "battery_manager.h"
 #include "soc_calculator.h"
 #include "cell_balancer.h"
 #include "thermal_monitor.h"
 #include "bess_config.h"
 #include "bess_types.h"
 
 /* Event bit definitions */
 #define BMS_EVENT_INIT_COMPLETE      BIT0
 #define BMS_EVENT_START_MONITORING   BIT1
 #define BMS_EVENT_STOP_MONITORING    BIT2
 #define BMS_EVENT_FAULT_DETECTED     BIT3
 #define BMS_EVENT_BALANCING_ACTIVE   BIT4
 #define BMS_EVENT_DATA_UPDATED       BIT5
 
 /* Define tag for logging */
 static const char* TAG = "Battery-Manager";
 
 /* Module data storage */
 static bess_module_data_t s_module_data[BESS_MODULE_COUNT];
 
 /* Battery system status */
 static bess_system_status_t s_battery_status;
 
 /* Current charging and discharging parameters */
 static struct {
     float max_charging_voltage;
     float max_charging_current;
     float target_soc;
     float min_discharging_voltage;
     float max_discharging_current;
     float min_soc;
 } s_battery_params;
 
 /* Event group for BMS state */
 static EventGroupHandle_t s_bms_event_group = NULL;
 
 /* Mutex for data access */
 static SemaphoreHandle_t s_data_mutex = NULL;
 
 /* Tasks */
 static TaskHandle_t s_monitoring_task_handle = NULL;
 static TaskHandle_t s_balancing_task_handle = NULL;
 static TaskHandle_t s_diagnostics_task_handle = NULL;
 
 /* Event callback registry */
 #define MAX_CALLBACKS_PER_EVENT 5
 typedef struct {
     bess_event_callback_t callbacks[MAX_CALLBACKS_PER_EVENT];
     void *user_data[MAX_CALLBACKS_PER_EVENT];
     uint8_t count;
 } event_callback_registry_t;
 
 static event_callback_registry_t s_event_callbacks[10];
 
 /* Forward declarations for internal functions */
 static void bms_monitoring_task(void *pvParameters);
 static void bms_balancing_task(void *pvParameters);
 static void bms_diagnostics_task(void *pvParameters);
 static esp_err_t update_module_data(uint8_t module_id);
 static esp_err_t update_system_status(void);
 static void notify_event(bess_event_type_t event_type, void *event_data);
 static void check_battery_limits(void);
 
 /**
  * @brief Initialize the battery management system
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t battery_manager_init(void)
 {
     esp_err_t ret;
     
     ESP_LOGI(TAG, "Initializing Battery Management System");
     
     /* Initialize module data */
     memset(s_module_data, 0, sizeof(s_module_data));
     memset(&s_battery_status, 0, sizeof(s_battery_status));
     
     /* Initialize default parameters */
     s_battery_params.max_charging_voltage = BESS_MODULE_NOMINAL_VOLTAGE * BESS_MODULE_COUNT * 1.05f; // 5% above nominal
     s_battery_params.max_charging_current = BESS_SAFETY_MAX_CURRENT * 0.8f; // 80% of max current
     s_battery_params.target_soc = BESS_BMS_SOC_MAX;
     s_battery_params.min_discharging_voltage = BESS_MODULE_NOMINAL_VOLTAGE * BESS_MODULE_COUNT * 0.9f; // 90% of nominal
     s_battery_params.max_discharging_current = BESS_SAFETY_MAX_CURRENT * 0.8f; // 80% of max current
     s_battery_params.min_soc = BESS_BMS_SOC_MIN;
     
     /* Create event group and mutex */
     s_bms_event_group = xEventGroupCreate();
     if (s_bms_event_group == NULL) {
         ESP_LOGE(TAG, "Failed to create BMS event group");
         return ESP_FAIL;
     }
     
     s_data_mutex = xSemaphoreCreateMutex();
     if (s_data_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create data mutex");
         vEventGroupDelete(s_bms_event_group);
         return ESP_FAIL;
     }
     
     /* Initialize sub-components */
     
     // Initialize SoC calculator
     ret = soc_calculator_init(SOC_METHOD_HYBRID);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize SoC calculator: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     // Initialize cell balancer
     ret = cell_balancer_init(CELL_BALANCING_PASSIVE);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize cell balancer: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     // Initialize thermal monitor
     ret = thermal_monitor_init(COOLING_METHOD_FORCED_AIR);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize thermal monitor: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     // Set thermal thresholds
     ret = thermal_monitor_set_thresholds(
         BESS_SAFETY_WARN_TEMP_C - 5,  // Elevated
         BESS_SAFETY_WARN_TEMP_C,      // Warning
         BESS_SAFETY_MAX_TEMP_C - 5,   // Critical
         BESS_SAFETY_MAX_TEMP_C        // Emergency
     );
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set thermal thresholds: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     // Set cell balancing parameters
     ret = cell_balancer_set_threshold(BESS_BMS_CELL_BALANCING_THRESHOLD);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set cell balancing threshold: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     // Clear event callback registry
     memset(s_event_callbacks, 0, sizeof(s_event_callbacks));
     
     // Initialization complete
     xEventGroupSetBits(s_bms_event_group, BMS_EVENT_INIT_COMPLETE);
     ESP_LOGI(TAG, "Battery Management System initialized successfully");
     
     return ESP_OK;
     
 cleanup:
     if (s_data_mutex != NULL) {
         vSemaphoreDelete(s_data_mutex);
         s_data_mutex = NULL;
     }
     
     if (s_bms_event_group != NULL) {
         vEventGroupDelete(s_bms_event_group);
         s_bms_event_group = NULL;
     }
     
     return ret;
 }
 
 /**
  * @brief Start the battery management system tasks
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t battery_manager_start(void)
 {
     ESP_LOGI(TAG, "Starting Battery Management System");
     
     // Wait for initialization to complete
     EventBits_t bits = xEventGroupWaitBits(
         s_bms_event_group,
         BMS_EVENT_INIT_COMPLETE,
         pdFALSE,
         pdTRUE,
         pdMS_TO_TICKS(5000)
     );
     
     if ((bits & BMS_EVENT_INIT_COMPLETE) == 0) {
         ESP_LOGE(TAG, "BMS initialization not complete");
         return ESP_ERR_NOT_FINISHED;
     }
     
     // Create monitoring task
     BaseType_t ret = xTaskCreatePinnedToCore(
         bms_monitoring_task,
         "bms_monitoring",
         BESS_BMS_TASK_STACK_SIZE,
         NULL,
         BESS_BMS_TASK_PRIORITY,
         &s_monitoring_task_handle,
         BESS_BMS_TASK_CORE
     );
     
     if (ret != pdPASS) {
         ESP_LOGE(TAG, "Failed to create BMS monitoring task");
         return ESP_FAIL;
     }
     
     // Create balancing task
     ret = xTaskCreatePinnedToCore(
         bms_balancing_task,
         "bms_balancing",
         BESS_BMS_TASK_STACK_SIZE,
         NULL,
         BESS_BMS_TASK_PRIORITY - 1, // Lower priority than monitoring
         &s_balancing_task_handle,
         BESS_BMS_TASK_CORE
     );
     
     if (ret != pdPASS) {
         ESP_LOGE(TAG, "Failed to create BMS balancing task");
         vTaskDelete(s_monitoring_task_handle);
         s_monitoring_task_handle = NULL;
         return ESP_FAIL;
     }
     
     // Create diagnostics task
     ret = xTaskCreatePinnedToCore(
         bms_diagnostics_task,
         "bms_diagnostics",
         BESS_BMS_TASK_STACK_SIZE,
         NULL,
         BESS_BMS_TASK_PRIORITY - 2, // Lower priority than balancing
         &s_diagnostics_task_handle,
         BESS_BMS_TASK_CORE
     );
     
     if (ret != pdPASS) {
         ESP_LOGE(TAG, "Failed to create BMS diagnostics task");
         vTaskDelete(s_monitoring_task_handle);
         vTaskDelete(s_balancing_task_handle);
         s_monitoring_task_handle = NULL;
         s_balancing_task_handle = NULL;
         return ESP_FAIL;
     }
     
     // Start thermal monitoring
     esp_err_t err = thermal_monitor_start();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start thermal monitor: %s", esp_err_to_name(err));
         vTaskDelete(s_monitoring_task_handle);
         vTaskDelete(s_balancing_task_handle);
         vTaskDelete(s_diagnostics_task_handle);
         s_monitoring_task_handle = NULL;
         s_balancing_task_handle = NULL;
         s_diagnostics_task_handle = NULL;
         return err;
     }
     
     // Signal tasks to start monitoring
     xEventGroupSetBits(s_bms_event_group, BMS_EVENT_START_MONITORING);
     
     ESP_LOGI(TAG, "Battery Management System started successfully");
     
     return ESP_OK;
 }
 
 /**
  * @brief Battery monitoring task function
  * 
  * @param pvParameters Task parameters (unused)
  */
 static void bms_monitoring_task(void *pvParameters)
 {
     ESP_LOGI(TAG, "BMS monitoring task started");
     
     TickType_t last_wake_time = xTaskGetTickCount();
     uint8_t update_counter = 0;
     bool fault_detected = false;
     
     // Wait for start signal
     xEventGroupWaitBits(
         s_bms_event_group,
         BMS_EVENT_START_MONITORING,
         pdFALSE,
         pdTRUE,
         portMAX_DELAY
     );
     
     while (1) {
         // Check if we should stop monitoring
         EventBits_t bits = xEventGroupGetBits(s_bms_event_group);
         if ((bits & BMS_EVENT_STOP_MONITORING) != 0) {
             ESP_LOGI(TAG, "Stopping BMS monitoring task");
             break;
         }
         
         // Update all module data
         for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
             esp_err_t ret = update_module_data(i);
             if (ret != ESP_OK) {
                 ESP_LOGW(TAG, "Failed to update module %u data: %s", i, esp_err_to_name(ret));
                 
                 if (s_module_data[i].online) {
                     // Module was online but now communication failed
                     ESP_LOGW(TAG, "Module %u communication lost", i);
                     
                     // Take mutex to update module data
                     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                         s_module_data[i].online = false;
                         s_module_data[i].error_code = BESS_ERROR_INTERNAL_COMM;
                         xSemaphoreGive(s_data_mutex);
                     }
                 }
             } else {
                 // Module communication successful
                 if (!s_module_data[i].online) {
                     ESP_LOGI(TAG, "Module %u communication established", i);
                     
                     // Take mutex to update module data
                     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                         s_module_data[i].online = true;
                         s_module_data[i].error_code = BESS_ERROR_NONE;
                         xSemaphoreGive(s_data_mutex);
                     }
                 }
             }
         }
         
         // Update system status based on module data
         update_system_status();
         
         // Check battery limits and safety thresholds
         check_battery_limits();
         
         // Notify that data has been updated
         xEventGroupSetBits(s_bms_event_group, BMS_EVENT_DATA_UPDATED);
         
         // Every 10 cycles, perform additional tasks
         if (++update_counter >= 10) {
             update_counter = 0;
             
             // Perform periodic health check
             thermal_status_t thermal_status;
             esp_err_t ret = thermal_monitor_get_status(&thermal_status);
             if (ret == ESP_OK) {
                 // Update battery status with thermal information
                 if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                     s_battery_status.highest_cell_temperature = thermal_status.highest_temperature;
                     s_battery_status.lowest_cell_temperature = thermal_status.lowest_temperature;
                     xSemaphoreGive(s_data_mutex);
                 }
                 
                 // Check for thermal runaway
                 bool runaway_detected = false;
                 uint8_t affected_module = 0;
                 ret = thermal_monitor_check_runaway(&runaway_detected, &affected_module);
                 if (ret == ESP_OK && runaway_detected) {
                     ESP_LOGE(TAG, "THERMAL RUNAWAY DETECTED in module %u!", affected_module);
                     fault_detected = true;
                     
                     // Set fault flag and error code
                     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                         s_battery_status.flags.fault_condition = true;
                         s_battery_status.error_code = BESS_ERROR_OVERTEMPERATURE;
                         xSemaphoreGive(s_data_mutex);
                     }
                     
                     // Signal fault to other tasks
                     xEventGroupSetBits(s_bms_event_group, BMS_EVENT_FAULT_DETECTED);
                     
                     // Notify event listeners
                     notify_event(BESS_EVENT_ERROR, &s_battery_status);
                 }
             }
         }
         
         // Wait for next cycle
         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BESS_BMS_TASK_INTERVAL_MS));
     }
     
     // Task cleanup
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Battery balancing task function
  * 
  * @param pvParameters Task parameters (unused)
  */
 static void bms_balancing_task(void *pvParameters)
 {
     ESP_LOGI(TAG, "BMS balancing task started");
     
     TickType_t last_wake_time = xTaskGetTickCount();
     
     // Wait for start signal
     xEventGroupWaitBits(
         s_bms_event_group,
         BMS_EVENT_START_MONITORING,
         pdFALSE,
         pdTRUE,
         portMAX_DELAY
     );
     
     while (1) {
         // Check if we should stop
         EventBits_t bits = xEventGroupGetBits(s_bms_event_group);
         if ((bits & BMS_EVENT_STOP_MONITORING) != 0) {
             ESP_LOGI(TAG, "Stopping BMS balancing task");
             break;
         }
         
         // Wait for data to be updated
         bits = xEventGroupWaitBits(
             s_bms_event_group,
             BMS_EVENT_DATA_UPDATED,
             pdTRUE,  // Clear the bit after waiting
             pdFALSE,
             pdMS_TO_TICKS(BESS_BMS_TASK_INTERVAL_MS * 2)
         );
         
         if ((bits & BMS_EVENT_DATA_UPDATED) != 0) {
             // Data updated, check if balancing is needed for each module
             for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
                 // Skip offline modules
                 if (!s_module_data[i].online) {
                     continue;
                 }
                 
                 // Check if balancing is needed
                 bool balancing_needed = false;
                 esp_err_t ret = cell_balancer_is_needed(s_module_data[i].cells, BESS_CELLS_PER_MODULE, &balancing_needed);
                 
                 if (ret == ESP_OK && balancing_needed) {
                     // Start balancing this module
                     ret = cell_balancer_start(i, s_module_data[i].cells, BESS_CELLS_PER_MODULE, false);
                     if (ret == ESP_OK) {
                         ESP_LOGI(TAG, "Started cell balancing for module %u", i);
                         xEventGroupSetBits(s_bms_event_group, BMS_EVENT_BALANCING_ACTIVE);
                         
                         // Update module status
                         if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                             s_battery_status.flags.cell_balancing_active = true;
                             xSemaphoreGive(s_data_mutex);
                         }
                     } else {
                         ESP_LOGW(TAG, "Failed to start balancing for module %u: %s", i, esp_err_to_name(ret));
                     }
                 } else if (ret != ESP_OK) {
                     ESP_LOGW(TAG, "Failed to check if balancing needed for module %u: %s", i, esp_err_to_name(ret));
                 }
                 
                 // Update balancing status if active
                 cell_balancing_status_t balance_status;
                 ret = cell_balancer_get_status(i, &balance_status);
                 if (ret == ESP_OK && balance_status.active) {
                     // Update balancer with latest cell data
                     ret = cell_balancer_update(i, s_module_data[i].cells, BESS_CELLS_PER_MODULE);
                     if (ret != ESP_OK) {
                         ESP_LOGW(TAG, "Failed to update balancing for module %u: %s", i, esp_err_to_name(ret));
                     }
                     
                     // Check if balancing completed
                     if (balance_status.current_voltage_delta < BESS_BMS_CELL_BALANCING_THRESHOLD * 0.5f) {
                         ESP_LOGI(TAG, "Cell balancing completed for module %u", i);
                         ret = cell_balancer_stop(i);
                         if (ret != ESP_OK) {
                             ESP_LOGW(TAG, "Failed to stop balancing for module %u: %s", i, esp_err_to_name(ret));
                         }
                     }
                 }
             }
             
             // Check if any module is still balancing
             bool any_balancing = false;
             for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
                 cell_balancing_status_t balance_status;
                 esp_err_t ret = cell_balancer_get_status(i, &balance_status);
                 if (ret == ESP_OK && balance_status.active) {
                     any_balancing = true;
                     break;
                 }
             }
             
             // Update system flag
             if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 s_battery_status.flags.cell_balancing_active = any_balancing;
                 xSemaphoreGive(s_data_mutex);
             }
         }
         
         // Wait for next cycle
         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BESS_BMS_TASK_INTERVAL_MS * 5)); // Run at 1/5 the rate of monitoring
     }
     
     // Task cleanup
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Battery diagnostics task function
  * 
  * @param pvParameters Task parameters (unused)
  */
 static void bms_diagnostics_task(void *pvParameters)
 {
     ESP_LOGI(TAG, "BMS diagnostics task started");
     
     TickType_t last_wake_time = xTaskGetTickCount();
     uint32_t diagnostic_interval_ms = 5 * 60 * 1000; // 5 minutes
     
     // Wait for start signal
     xEventGroupWaitBits(
         s_bms_event_group,
         BMS_EVENT_START_MONITORING,
         pdFALSE,
         pdTRUE,
         portMAX_DELAY
     );
     
     while (1) {
         // Check if we should stop
         EventBits_t bits = xEventGroupGetBits(s_bms_event_group);
         if ((bits & BMS_EVENT_STOP_MONITORING) != 0) {
             ESP_LOGI(TAG, "Stopping BMS diagnostics task");
             break;
         }
         
         // Run diagnostics checks
         ESP_LOGI(TAG, "Running BMS diagnostics");
         
         // Check capacity consistency across modules
         float min_soc = 100.0f;
         float max_soc = 0.0f;
         float avg_soc = 0.0f;
         uint8_t active_modules = 0;
         
         if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
                 if (s_module_data[i].online) {
                     if (s_module_data[i].state_of_charge < min_soc) {
                         min_soc = s_module_data[i].state_of_charge;
                     }
                     if (s_module_data[i].state_of_charge > max_soc) {
                         max_soc = s_module_data[i].state_of_charge;
                     }
                     avg_soc += s_module_data[i].state_of_charge;
                     active_modules++;
                 }
             }
             xSemaphoreGive(s_data_mutex);
         }
         
         if (active_modules > 0) {
             avg_soc /= active_modules;
             
             ESP_LOGI(TAG, "SoC statistics: min=%.1f%%, max=%.1f%%, avg=%.1f%%, delta=%.1f%%",
                     min_soc, max_soc, avg_soc, max_soc - min_soc);
             
             // Check for imbalance
             if (max_soc - min_soc > 5.0f) {
                 ESP_LOGW(TAG, "Significant SoC imbalance detected between modules");
                 // TODO: Trigger balancing or recalibration
             }
         }
         
         // Check for aging patterns
         float min_soh = 100.0f;
         float max_soh = 0.0f;
         float avg_soh = 0.0f;
         
         if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
                 if (s_module_data[i].online) {
                     if (s_module_data[i].state_of_health < min_soh) {
                         min_soh = s_module_data[i].state_of_health;
                     }
                     if (s_module_data[i].state_of_health > max_soh) {
                         max_soh = s_module_data[i].state_of_health;
                     }
                     avg_soh += s_module_data[i].state_of_health;
                 }
             }
             xSemaphoreGive(s_data_mutex);
         }
         
         if (active_modules > 0) {
             avg_soh /= active_modules;
             
             ESP_LOGI(TAG, "SoH statistics: min=%.1f%%, max=%.1f%%, avg=%.1f%%, delta=%.1f%%",
                     min_soh, max_soh, avg_soh, max_soh - min_soh);
             
             // Check for aging
             if (avg_soh < 80.0f) {
                 ESP_LOGW(TAG, "Battery system is showing signs of aging (avg SoH: %.1f%%)", avg_soh);
             }
             
             if (min_soh < 70.0f) {
                 ESP_LOGW(TAG, "Some modules have significantly degraded (min SoH: %.1f%%)", min_soh);
             }
         }
         
         // Wait for next diagnostic cycle
         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(diagnostic_interval_ms));
     }
     
     // Task cleanup
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Update data for a specific battery module
  * 
  * @param module_id ID of the module to update
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t update_module_data(uint8_t module_id)
 {
     if (module_id >= BESS_MODULE_COUNT) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // TODO: Implement actual communication with battery modules via CAN or other protocol
     // For now, we'll simulate module data for testing
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for module %u update", module_id);
         return ESP_ERR_TIMEOUT;
     }
     
     // Update module data (simulation only - replace with actual module communication)
     s_module_data[module_id].id = module_id;
     
     // Simulate module voltage (48V nominal with some variance)
     s_module_data[module_id].voltage = BESS_MODULE_NOMINAL_VOLTAGE + ((esp_random() % 100) / 100.0f - 0.5f) * 2.0f;
     
     // Simulate module current
     s_module_data[module_id].current = ((esp_random() % 2000) / 10.0f - 100.0f);
     
     // Simulate temperature sensors (around 25°C with some variance)
     for (int i = 0; i < 4; i++) {
         s_module_data[module_id].temperature[i] = 25.0f + ((esp_random() % 100) / 100.0f - 0.5f) * 5.0f;
     }
     
     // Simulate SoC and SoH (these would normally be calculated)
     s_module_data[module_id].state_of_charge = 50.0f + ((esp_random() % 100) / 100.0f - 0.5f) * 10.0f;
     s_module_data[module_id].state_of_health = 90.0f + ((esp_random() % 100) / 100.0f - 0.5f) * 5.0f;
     
     // Simulate cell data
     for (int i = 0; i < BESS_CELLS_PER_MODULE; i++) {
         // Cell voltage (3.2V nominal with some variance)
         s_module_data[module_id].cells[i].voltage = 
             BESS_CELL_NOMINAL_VOLTAGE + ((esp_random() % 100) / 100.0f - 0.5f) * 0.2f;
         
         // Cell temperature (same as module temperature with minor variance)
         s_module_data[module_id].cells[i].temperature = 
             s_module_data[module_id].temperature[i % 4] + ((esp_random() % 100) / 100.0f - 0.5f) * 2.0f;
         
         // Other cell parameters
         s_module_data[module_id].cells[i].internal_resistance = 5.0f + ((esp_random() % 100) / 100.0f) * 2.0f;
         s_module_data[module_id].cells[i].balancing_active = false;
         s_module_data[module_id].cells[i].cycle_count = 100 + (esp_random() % 50);
         s_module_data[module_id].cells[i].health_percentage = 90 + (esp_random() % 10);
     }
     
     // Update module energy counters
     s_module_data[module_id].total_energy_charged += 
         (s_module_data[module_id].current > 0) ? 
         (s_module_data[module_id].current * s_module_data[module_id].voltage * (BESS_BMS_TASK_INTERVAL_MS / 3600000.0f)) : 0;
     
     s_module_data[module_id].total_energy_discharged += 
         (s_module_data[module_id].current < 0) ? 
         (-s_module_data[module_id].current * s_module_data[module_id].voltage * (BESS_BMS_TASK_INTERVAL_MS / 3600000.0f)) : 0;
     
     // Module is online
     s_module_data[module_id].online = true;
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     // Update thermal monitoring data
     esp_err_t ret = thermal_monitor_update_module(module_id, s_module_data[module_id].temperature, 4);
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to update thermal monitoring for module %u: %s", module_id, esp_err_to_name(ret));
     }
     
     // Calculate SoC for this module
     float module_soc;
     ret = soc_calculator_calculate_module_soc(module_id, &s_module_data[module_id], &module_soc);
     if (ret == ESP_OK) {
         // Take the data mutex again to update SoC
         if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             s_module_data[module_id].state_of_charge = module_soc;
             xSemaphoreGive(s_data_mutex);
         }
     } else {
         ESP_LOGW(TAG, "Failed to calculate SoC for module %u: %s", module_id, esp_err_to_name(ret));
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Update the overall system status based on module data
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t update_system_status(void)
 {
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for system status update");
         return ESP_ERR_TIMEOUT;
     }
     
     // Initialize totals
     float total_voltage = 0.0f;
     float total_current = 0.0f;
     float total_power = 0.0f;
     float highest_temp = -100.0f;
     float lowest_temp = 200.0f;
     float highest_cell_voltage = 0.0f;
     float lowest_cell_voltage = 10.0f; // Unrealistically high starting point
     float total_soc = 0.0f;
     float total_soh = 0.0f;
     uint8_t active_count = 0;
     
     // Collect data from modules
     for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
         if (s_module_data[i].online) {
             active_count++;
             
             // Add module voltage and current
             total_voltage += s_module_data[i].voltage;
             total_current += s_module_data[i].current;
             
             // Sum state values for averaging later
             total_soc += s_module_data[i].state_of_charge;
             total_soh += s_module_data[i].state_of_health;
             
             // Find extreme temperatures
             for (int t = 0; t < 4; t++) {
                 if (s_module_data[i].temperature[t] > highest_temp) {
                     highest_temp = s_module_data[i].temperature[t];
                 }
                 if (s_module_data[i].temperature[t] < lowest_temp) {
                     lowest_temp = s_module_data[i].temperature[t];
                 }
             }
             
             // Find extreme cell voltages
             for (int c = 0; c < BESS_CELLS_PER_MODULE; c++) {
                 if (s_module_data[i].cells[c].voltage > highest_cell_voltage) {
                     highest_cell_voltage = s_module_data[i].cells[c].voltage;
                 }
                 if (s_module_data[i].cells[c].voltage < lowest_cell_voltage) {
                     lowest_cell_voltage = s_module_data[i].cells[c].voltage;
                 }
             }
         }
     }
     
     // Calculate power
     total_power = total_voltage * total_current;
     
     // Update system status
     s_battery_status.total_voltage = total_voltage;
     s_battery_status.total_current = total_current;
     s_battery_status.total_power = total_power;
     s_battery_status.highest_cell_temperature = highest_temp;
     s_battery_status.lowest_cell_temperature = lowest_temp;
     s_battery_status.highest_cell_voltage = highest_cell_voltage;
     s_battery_status.lowest_cell_voltage = lowest_cell_voltage;
     s_battery_status.active_module_count = active_count;
     
     // Calculate average SoC and SoH if any modules are active
     if (active_count > 0) {
         s_battery_status.state_of_charge = total_soc / active_count;
         s_battery_status.state_of_health = total_soh / active_count;
         
         // Calculate available energy in kWh
         s_battery_status.energy_available = 
             (s_battery_status.state_of_charge / 100.0f) * 
             BESS_MODULE_CAPACITY_KWH * active_count;
     } else {
         // No active modules
         s_battery_status.state_of_charge = 0.0f;
         s_battery_status.state_of_health = 0.0f;
         s_battery_status.energy_available = 0.0f;
     }
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Check battery limits and set appropriate flags/alarms
  */
 static void check_battery_limits(void)
 {
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for battery limits check");
         return;
     }
     
     bool fault_detected = false;
     bess_error_code_t error_code = BESS_ERROR_NONE;
     
     // Check voltage limits
     if (s_battery_status.total_voltage > BESS_SAFETY_MAX_VOLTAGE) {
         ESP_LOGW(TAG, "System voltage exceeds maximum: %.2fV", s_battery_status.total_voltage);
         fault_detected = true;
         error_code = BESS_ERROR_OVERVOLTAGE;
     } else if (s_battery_status.total_voltage < BESS_SAFETY_MIN_VOLTAGE && 
                s_battery_status.active_module_count > 0) {
         ESP_LOGW(TAG, "System voltage below minimum: %.2fV", s_battery_status.total_voltage);
         fault_detected = true;
         error_code = BESS_ERROR_UNDERVOLTAGE;
     }
     
     // Check cell voltage limits
     if (s_battery_status.highest_cell_voltage > BESS_CELL_MAX_VOLTAGE) {
         ESP_LOGW(TAG, "Cell voltage exceeds maximum: %.2fV", s_battery_status.highest_cell_voltage);
         fault_detected = true;
         error_code = BESS_ERROR_OVERVOLTAGE;
     } else if (s_battery_status.lowest_cell_voltage < BESS_CELL_MIN_VOLTAGE && 
                s_battery_status.active_module_count > 0) {
         ESP_LOGW(TAG, "Cell voltage below minimum: %.2fV", s_battery_status.lowest_cell_voltage);
         fault_detected = true;
         error_code = BESS_ERROR_UNDERVOLTAGE;
     }
     
     // Check current limits
     if (fabs(s_battery_status.total_current) > BESS_SAFETY_MAX_CURRENT) {
         ESP_LOGW(TAG, "System current exceeds maximum: %.2fA", fabs(s_battery_status.total_current));
         fault_detected = true;
         error_code = BESS_ERROR_OVERCURRENT;
     }
     
     // Check temperature limits
     if (s_battery_status.highest_cell_temperature > BESS_SAFETY_MAX_TEMP_C) {
         ESP_LOGW(TAG, "Cell temperature exceeds maximum: %.2f°C", s_battery_status.highest_cell_temperature);
         fault_detected = true;
         error_code = BESS_ERROR_OVERTEMPERATURE;
     } else if (s_battery_status.lowest_cell_temperature < BESS_SAFETY_MIN_TEMP_C && 
                s_battery_status.active_module_count > 0) {
         ESP_LOGW(TAG, "Cell temperature below minimum: %.2f°C", s_battery_status.lowest_cell_temperature);
         fault_detected = true;
         error_code = BESS_ERROR_UNDERTEMPERATURE;
     }
     
     // Check balance limits
     float cell_voltage_delta = s_battery_status.highest_cell_voltage - s_battery_status.lowest_cell_voltage;
     if (cell_voltage_delta > BESS_BMS_VOLTAGE_DELTA_MAX) {
         ESP_LOGW(TAG, "Cell voltage imbalance detected: %.3fV", cell_voltage_delta);
         
         // Only set an error if severely imbalanced (2x the threshold)
         if (cell_voltage_delta > BESS_BMS_VOLTAGE_DELTA_MAX * 2.0f) {
             fault_detected = true;
             error_code = BESS_ERROR_CELL_IMBALANCE;
         }
     }
     
     // Update fault status
     if (fault_detected && s_battery_status.error_code == BESS_ERROR_NONE) {
         // New fault detected
         ESP_LOGW(TAG, "Battery fault detected: %d", error_code);
         s_battery_status.flags.fault_condition = true;
         s_battery_status.error_code = error_code;
         s_battery_status.last_error_time = time(NULL);
         
         // Update charging/discharging permissions
         if (error_code == BESS_ERROR_OVERVOLTAGE || 
             error_code == BESS_ERROR_OVERTEMPERATURE) {
             s_battery_status.flags.charging_permitted = false;
         }
         
         if (error_code == BESS_ERROR_UNDERVOLTAGE || 
             error_code == BESS_ERROR_UNDERTEMPERATURE) {
             s_battery_status.flags.discharging_permitted = false;
         }
         
         if (error_code == BESS_ERROR_OVERCURRENT || 
             error_code == BESS_ERROR_CELL_IMBALANCE) {
             s_battery_status.flags.charging_permitted = false;
             s_battery_status.flags.discharging_permitted = false;
         }
         
         // Signal fault to other tasks
         xEventGroupSetBits(s_bms_event_group, BMS_EVENT_FAULT_DETECTED);
         
         // Notify event listeners
         xSemaphoreGive(s_data_mutex); // Release mutex before notification
         notify_event(BESS_EVENT_ERROR, &s_battery_status);
         return; // Mutex already released
     } else if (!fault_detected && s_battery_status.error_code != BESS_ERROR_NONE) {
         // Fault condition cleared
         ESP_LOGI(TAG, "Battery fault condition cleared");
         s_battery_status.flags.fault_condition = false;
         s_battery_status.error_code = BESS_ERROR_NONE;
         
         // Reset permissions based on current state
         update_charging_discharging_permissions();
         
         // Notify event listeners
         xSemaphoreGive(s_data_mutex); // Release mutex before notification
         notify_event(BESS_EVENT_STATE_CHANGE, &s_battery_status);
         return; // Mutex already released
     }
     
     // No change in fault status, but still update permissions
     update_charging_discharging_permissions();
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
 }
 
 /**
  * @brief Update charging and discharging permissions based on system state
  * 
  * Note: This function assumes the data mutex is already held
  */
 static void update_charging_discharging_permissions(void)
 {
     // Default permissions
     bool charging_permitted = true;
     bool discharging_permitted = true;
     
     // Check if system is in fault condition
     if (s_battery_status.flags.fault_condition) {
         // Permissions already set in check_battery_limits
         return;
     }
     
     // Check SoC limits
     if (s_battery_status.state_of_charge >= s_battery_params.target_soc) {
         charging_permitted = false;
     }
     
     if (s_battery_status.state_of_charge <= s_battery_params.min_soc) {
         discharging_permitted = false;
     }
     
     // Check temperature limits
     if (s_battery_status.highest_cell_temperature >= BESS_SAFETY_WARN_TEMP_C) {
         charging_permitted = false;
     }
     
     if (s_battery_status.lowest_cell_temperature <= BESS_SAFETY_WARN_MIN_TEMP_C) {
         discharging_permitted = false;
     }
     
     // Update flags
     s_battery_status.flags.charging_permitted = charging_permitted;
     s_battery_status.flags.discharging_permitted = discharging_permitted;
 }
 
 /**
  * @brief Notify registered callbacks about an event
  * 
  * @param event_type Type of event
  * @param event_data Event data to pass to callbacks
  */
 static void notify_event(bess_event_type_t event_type, void *event_data)
 {
     if (event_type >= sizeof(s_event_callbacks) / sizeof(s_event_callbacks[0])) {
         ESP_LOGW(TAG, "Invalid event type: %d", event_type);
         return;
     }
     
     event_callback_registry_t *registry = &s_event_callbacks[event_type];
     
     for (uint8_t i = 0; i < registry->count; i++) {
         if (registry->callbacks[i] != NULL) {
             registry->callbacks[i](event_data, registry->user_data[i]);
         }
     }
 }
 
 /* External API implementation */
 
 esp_err_t battery_manager_get_module_data(bess_module_data_t *module_data)
 {
     if (module_data == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for get_module_data");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy module data
     memcpy(module_data, s_module_data, sizeof(s_module_data));
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_get_system_status(bess_system_status_t *battery_status)
 {
     if (battery_status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for get_system_status");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy system status
     memcpy(battery_status, &s_battery_status, sizeof(s_battery_status));
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_start_balancing(bool force_balance)
 {
     ESP_LOGI(TAG, "Starting cell balancing for all modules");
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for start_balancing");
         return ESP_ERR_TIMEOUT;
     }
     
     // Start balancing for all online modules
     esp_err_t ret = ESP_OK;
     for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
         if (s_module_data[i].online) {
             esp_err_t module_ret = cell_balancer_start(i, s_module_data[i].cells, BESS_CELLS_PER_MODULE, force_balance);
             if (module_ret != ESP_OK) {
                 ESP_LOGW(TAG, "Failed to start balancing for module %u: %s", i, esp_err_to_name(module_ret));
                 ret = module_ret; // Save last error
             }
         }
     }
     
     // Set balancing flag
     s_battery_status.flags.cell_balancing_active = true;
     xEventGroupSetBits(s_bms_event_group, BMS_EVENT_BALANCING_ACTIVE);
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ret;
 }
 
 esp_err_t battery_manager_stop_balancing(void)
 {
     ESP_LOGI(TAG, "Stopping cell balancing for all modules");
     
     // Stop balancing for all modules
     esp_err_t ret = ESP_OK;
     for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
         esp_err_t module_ret = cell_balancer_stop(i);
         if (module_ret != ESP_OK) {
             ESP_LOGW(TAG, "Failed to stop balancing for module %u: %s", i, esp_err_to_name(module_ret));
             ret = module_ret; // Save last error
         }
     }
     
     // Clear balancing flag
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         s_battery_status.flags.cell_balancing_active = false;
         xSemaphoreGive(s_data_mutex);
     }
     
     return ret;
 }
 
 esp_err_t battery_manager_get_soc(float *soc)
 {
     if (soc == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for get_soc");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy SoC value
     *soc = s_battery_status.state_of_charge;
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_get_soh(float *soh)
 {
     if (soh == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for get_soh");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy SoH value
     *soh = s_battery_status.state_of_health;
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_set_charging_params(float max_voltage, float max_current, float target_soc)
 {
     // Validate parameters
     if (max_voltage <= 0.0f || max_current <= 0.0f || 
         target_soc < 0.0f || target_soc > 100.0f) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for set_charging_params");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update parameters
     s_battery_params.max_charging_voltage = max_voltage;
     s_battery_params.max_charging_current = max_current;
     s_battery_params.target_soc = target_soc;
     
     // Update permissions based on new parameters
     update_charging_discharging_permissions();
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     ESP_LOGI(TAG, "Charging parameters updated: max_voltage=%.2fV, max_current=%.2fA, target_soc=%.1f%%",
              max_voltage, max_current, target_soc);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_set_discharging_params(float min_voltage, float max_current, float min_soc)
 {
     // Validate parameters
     if (min_voltage <= 0.0f || max_current <= 0.0f || 
         min_soc < 0.0f || min_soc > 100.0f) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for set_discharging_params");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update parameters
     s_battery_params.min_discharging_voltage = min_voltage;
     s_battery_params.max_discharging_current = max_current;
     s_battery_params.min_soc = min_soc;
     
     // Update permissions based on new parameters
     update_charging_discharging_permissions();
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     ESP_LOGI(TAG, "Discharging parameters updated: min_voltage=%.2fV, max_current=%.2fA, min_soc=%.1f%%",
              min_voltage, max_current, min_soc);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_is_charging_allowed(bool *allowed)
 {
     if (allowed == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for is_charging_allowed");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy permission flag
     *allowed = s_battery_status.flags.charging_permitted;
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_is_discharging_allowed(bool *allowed)
 {
     if (allowed == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for is_discharging_allowed");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy permission flag
     *allowed = s_battery_status.flags.discharging_permitted;
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_register_event_callback(bess_event_type_t event_type, 
                                                  bess_event_callback_t callback,
                                                  void *user_data)
 {
     if (callback == NULL || 
         event_type >= sizeof(s_event_callbacks) / sizeof(s_event_callbacks[0])) {
         return ESP_ERR_INVALID_ARG;
     }
     
     event_callback_registry_t *registry = &s_event_callbacks[event_type];
     
     if (registry->count >= MAX_CALLBACKS_PER_EVENT) {
         ESP_LOGW(TAG, "Max callbacks reached for event type %d", event_type);
         return ESP_ERR_NO_MEM;
     }
     
     // Register callback
     registry->callbacks[registry->count] = callback;
     registry->user_data[registry->count] = user_data;
     registry->count++;
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_unregister_event_callback(bess_event_type_t event_type,
                                                    bess_event_callback_t callback)
 {
     if (callback == NULL || 
         event_type >= sizeof(s_event_callbacks) / sizeof(s_event_callbacks[0])) {
         return ESP_ERR_INVALID_ARG;
     }
     
     event_callback_registry_t *registry = &s_event_callbacks[event_type];
     
     // Find and remove callback
     for (uint8_t i = 0; i < registry->count; i++) {
         if (registry->callbacks[i] == callback) {
             // Shift remaining callbacks down
             for (uint8_t j = i; j < registry->count - 1; j++) {
                 registry->callbacks[j] = registry->callbacks[j + 1];
                 registry->user_data[j] = registry->user_data[j + 1];
             }
             
             registry->count--;
             return ESP_OK;
         }
     }
     
     // Callback not found
     return ESP_ERR_NOT_FOUND;
 }
 
 esp_err_t battery_manager_run_diagnostics(bool full_diagnostics, char *diagnostics_result)
 {
     ESP_LOGI(TAG, "Running battery diagnostics (full=%d)", full_diagnostics);
     
     // Basic diagnostics are performed continuously in the diagnostics task
     // Here we can trigger additional tests
     
     // TODO: Implement comprehensive diagnostics
     
     if (diagnostics_result != NULL) {
         // For now, provide basic system status
         snprintf(diagnostics_result, 256, 
                 "Battery System Diagnostics:\n"
                 "- Active modules: %u/%u\n"
                 "- System voltage: %.2f V\n"
                 "- System current: %.2f A\n"
                 "- State of Charge: %.1f%%\n"
                 "- State of Health: %.1f%%\n"
                 "- Cell voltage range: %.3f - %.3f V\n"
                 "- Cell temperature range: %.1f - %.1f °C\n",
                 s_battery_status.active_module_count, BESS_MODULE_COUNT,
                 s_battery_status.total_voltage,
                 s_battery_status.total_current,
                 s_battery_status.state_of_charge,
                 s_battery_status.state_of_health,
                 s_battery_status.lowest_cell_voltage, s_battery_status.highest_cell_voltage,
                 s_battery_status.lowest_cell_temperature, s_battery_status.highest_cell_temperature);
     }
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_standby(void)
 {
     ESP_LOGI(TAG, "Putting battery management system in standby mode");
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for standby");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update system mode
     s_battery_status.mode = BESS_MODE_STANDBY;
     
     // Disable charging and discharging
     s_battery_status.flags.charging_permitted = false;
     s_battery_status.flags.discharging_permitted = false;
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     // Signal mode change
     notify_event(BESS_EVENT_STATE_CHANGE, &s_battery_status);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_resume(void)
 {
     ESP_LOGI(TAG, "Resuming battery management system from standby mode");
     
     // Take the data mutex
     if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take data mutex for resume");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update system mode
     s_battery_status.mode = BESS_MODE_STANDBY; // Will be updated based on actual operation
     
     // Re-evaluate charging and discharging permissions
     update_charging_discharging_permissions();
     
     // Release the data mutex
     xSemaphoreGive(s_data_mutex);
     
     // Signal mode change
     notify_event(BESS_EVENT_STATE_CHANGE, &s_battery_status);
     
     return ESP_OK;
 }
 
 esp_err_t battery_manager_calibrate(void)
 {
     ESP_LOGI(TAG, "Calibrating battery sensors");
     
     // TODO: Implement actual calibration procedures
     
     // For now, just reset SoC calculator
     for (uint8_t i = 0; i < BESS_MODULE_COUNT; i++) {
         if (s_module_data[i].online) {
             // Reset SoC based on OCV
             soc_calculator_calibrate_from_ocv(i, s_module_data[i].voltage / BESS_CELLS_PER_MODULE);
         }
     }
     
     ESP_LOGI(TAG, "Battery calibration completed");
     
     return ESP_OK;
 }