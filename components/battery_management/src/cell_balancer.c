/**
 * @file cell_balancer.c
 * @brief Cell balancing implementation for LFP battery packs
 * 
 * This file implements the cell balancing functionality for LFP battery modules
 * in the BESS system. It handles both passive and active balancing methods,
 * schedules balancing operations, and tracks balancing status.
 */

 #include "cell_balancer.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include <string.h>
 #include <math.h>
 
 #define TAG "CELL_BALANCER"
 
 // Event group bits
 #define BALANCING_ACTIVE_BIT        (1 << 0)
 #define BALANCING_PAUSED_BIT        (1 << 1)
 #define BALANCING_EMERGENCY_STOP_BIT (1 << 2)
 
 // Default balancing parameters
 #define DEFAULT_VOLTAGE_THRESHOLD_MV  50    // 50mV difference to trigger balancing
 #define DEFAULT_BALANCE_CURRENT_MA    100   // 100mA balancing current (passive)
 #define DEFAULT_MIN_CELL_VOLTAGE_MV   2500  // 2.5V minimum cell voltage for balancing
 #define DEFAULT_MAX_BALANCE_TIME_MS   3600000 // 1 hour maximum balancing time
 #define DEFAULT_REST_TIME_MS          300000  // 5 minutes rest between balancing cycles
 #define DEFAULT_CELLS_PER_MODULE      16      // Typical cells per 48V LFP module
 #define DEFAULT_MODULES_COUNT         12      // Default module count for 200kWh system
 #define MAX_MODULES_COUNT             16      // Maximum supported modules
 
 // Cell equalization modes
 typedef enum {
     CELL_BALANCING_MODE_NONE,         // No balancing
     CELL_BALANCING_MODE_PASSIVE,      // Passive balancing (resistive)
     CELL_BALANCING_MODE_ACTIVE,       // Active balancing (energy transfer)
     CELL_BALANCING_MODE_HYBRID        // Hybrid approach
 } cell_balancing_mode_t;
 
 // Balancing status for a module
 typedef struct {
     bool is_balancing;                // Whether module is currently being balanced
     bool is_initialized;              // Whether module has been initialized
     uint32_t balance_start_time;      // When balancing started (ms)
     uint32_t total_balance_time;      // Total time spent balancing (ms)
     uint8_t balancing_cells_bitmap[4]; // Bitmap of cells being balanced (up to 32 cells)
     uint8_t num_cells_balancing;      // Number of cells currently being balanced
     float highest_cell_voltage;       // Current highest cell voltage (V)
     float lowest_cell_voltage;        // Current lowest cell voltage (V)
     uint8_t highest_cell_idx;         // Index of highest voltage cell
     uint8_t lowest_cell_idx;          // Index of lowest voltage cell
 } module_balance_status_t;
 
 // Configuration parameters
 typedef struct {
     uint32_t voltage_threshold_mv;    // Voltage difference threshold to trigger balancing (mV)
     uint32_t balance_current_ma;      // Balancing current (mA)
     uint32_t min_cell_voltage_mv;     // Minimum cell voltage for balancing (mV)
     uint32_t max_balance_time_ms;     // Maximum time to balance continuously (ms)
     uint32_t rest_time_ms;            // Rest time between balancing cycles (ms)
     uint8_t cells_per_module;         // Number of cells per module
     cell_balancing_mode_t mode;       // Balancing mode
 } cell_balancer_config_t;
 
 // Global state
 typedef struct {
     cell_balancer_config_t config;
     module_balance_status_t module_status[MAX_MODULES_COUNT];
     uint8_t module_count;
     EventGroupHandle_t event_group;
     SemaphoreHandle_t mutex;
     cell_balancer_callback_t event_callback;
     void *callback_arg;
     bool is_initialized;
 } cell_balancer_state_t;
 
 // Static (global) variables
 static cell_balancer_state_t s_state = {
     .config = {
         .voltage_threshold_mv = DEFAULT_VOLTAGE_THRESHOLD_MV,
         .balance_current_ma = DEFAULT_BALANCE_CURRENT_MA,
         .min_cell_voltage_mv = DEFAULT_MIN_CELL_VOLTAGE_MV,
         .max_balance_time_ms = DEFAULT_MAX_BALANCE_TIME_MS,
         .rest_time_ms = DEFAULT_REST_TIME_MS,
         .cells_per_module = DEFAULT_CELLS_PER_MODULE,
         .mode = CELL_BALANCING_MODE_PASSIVE
     },
     .module_count = DEFAULT_MODULES_COUNT,
     .is_initialized = false
 };
 
 // Forward declarations for internal functions
 static esp_err_t update_cell_status(uint8_t module_id, const bess_cell_data_t *cell_data, uint8_t cell_count);
 static esp_err_t start_cell_balancing(uint8_t module_id);
 static esp_err_t stop_cell_balancing(uint8_t module_id);
 static bool check_balancing_needed(uint8_t module_id);
 static void notify_event(cell_balancer_event_t event, uint8_t module_id);
 static void update_balancing_bitmap(uint8_t module_id, const bess_cell_data_t *cell_data, uint8_t cell_count);
 static float convert_mv_to_v(uint32_t mv);
 static uint32_t convert_v_to_mv(float v);
 
 /**
  * @brief Initialize the cell balancer
  * 
  * @param config Pointer to configuration parameters (NULL for defaults)
  * @param module_count Number of battery modules in the system
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_init(const cell_balancer_config_t *config, uint8_t module_count) {
     // Validate parameters
     if (module_count == 0 || module_count > MAX_MODULES_COUNT) {
         ESP_LOGE(TAG, "Invalid module count: %d", module_count);
         return ESP_ERR_INVALID_ARG;
     }
     
     // Create mutex for thread safety
     s_state.mutex = xSemaphoreCreateMutex();
     if (s_state.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_ERR_NO_MEM;
     }
     
     // Create event group
     s_state.event_group = xEventGroupCreate();
     if (s_state.event_group == NULL) {
         vSemaphoreDelete(s_state.mutex);
         ESP_LOGE(TAG, "Failed to create event group");
         return ESP_ERR_NO_MEM;
     }
     
     // Apply configuration if provided
     if (config != NULL) {
         memcpy(&s_state.config, config, sizeof(cell_balancer_config_t));
     }
     
     // Set module count
     s_state.module_count = module_count;
     
     // Initialize module statuses
     for (uint8_t i = 0; i < module_count; i++) {
         s_state.module_status[i].is_balancing = false;
         s_state.module_status[i].is_initialized = false;
         s_state.module_status[i].balance_start_time = 0;
         s_state.module_status[i].total_balance_time = 0;
         memset(s_state.module_status[i].balancing_cells_bitmap, 0, sizeof(s_state.module_status[i].balancing_cells_bitmap));
         s_state.module_status[i].num_cells_balancing = 0;
         s_state.module_status[i].highest_cell_voltage = 0.0f;
         s_state.module_status[i].lowest_cell_voltage = 5.0f; // Initialize higher than any expected cell voltage
         s_state.module_status[i].highest_cell_idx = 0;
         s_state.module_status[i].lowest_cell_idx = 0;
     }
     
     // Reset callback
     s_state.event_callback = NULL;
     s_state.callback_arg = NULL;
     
     // Mark as initialized
     s_state.is_initialized = true;
     
     ESP_LOGI(TAG, "Cell balancer initialized with %d modules", module_count);
     
     return ESP_OK;
 }
 
 /**
  * @brief Configure the cell balancer parameters
  * 
  * @param config Pointer to configuration parameters
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_configure(const cell_balancer_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy configuration
     memcpy(&s_state.config, config, sizeof(cell_balancer_config_t));
     
     xSemaphoreGive(s_state.mutex);
     
     ESP_LOGI(TAG, "Cell balancer reconfigured: threshold=%dmV, current=%dmA, mode=%d",
              config->voltage_threshold_mv, config->balance_current_ma, config->mode);
     
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for cell balancing events
  * 
  * @param callback Function to call when events occur
  * @param arg User argument to pass to the callback
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_register_callback(cell_balancer_callback_t callback, void *arg) {
     if (callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     s_state.event_callback = callback;
     s_state.callback_arg = arg;
     
     xSemaphoreGive(s_state.mutex);
     
     ESP_LOGI(TAG, "Event callback registered");
     
     return ESP_OK;
 }
 
 /**
  * @brief Update cell data and evaluate if balancing is needed
  * 
  * @param module_id ID of the module (0-based)
  * @param cell_data Array of cell data
  * @param cell_count Number of cells in the module
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_update_cell_data(uint8_t module_id, 
                                        const bess_cell_data_t *cell_data, 
                                        uint8_t cell_count) {
     if (module_id >= s_state.module_count || cell_data == NULL || cell_count == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Update cell status
     esp_err_t result = update_cell_status(module_id, cell_data, cell_count);
     
     // Check if balancing is needed and not already active
     bool balancing_needed = check_balancing_needed(module_id);
     bool emergency_stop = false;
     
     // Check for emergency stop conditions (module too hot, etc.)
     // This would come from thermal monitoring system
     EventBits_t bits = xEventGroupGetBits(s_state.event_group);
     if (bits & BALANCING_EMERGENCY_STOP_BIT) {
         emergency_stop = true;
         
         // If we're currently balancing, stop
         if (s_state.module_status[module_id].is_balancing) {
             ESP_LOGW(TAG, "Emergency stop triggered for module %d", module_id);
             stop_cell_balancing(module_id);
             notify_event(CELL_BALANCER_EVENT_EMERGENCY_STOP, module_id);
         }
     }
     
     // If we're not in emergency stop, handle regular balancing logic
     if (!emergency_stop) {
         if (balancing_needed && !s_state.module_status[module_id].is_balancing) {
             // Start balancing
             ESP_LOGI(TAG, "Starting balancing for module %d: voltage difference %.1fmV", 
                      module_id, 
                      (s_state.module_status[module_id].highest_cell_voltage - 
                       s_state.module_status[module_id].lowest_cell_voltage) * 1000);
             
             start_cell_balancing(module_id);
             notify_event(CELL_BALANCER_EVENT_STARTED, module_id);
         } 
         else if (!balancing_needed && s_state.module_status[module_id].is_balancing) {
             // Stop balancing if cells are now balanced
             ESP_LOGI(TAG, "Stopping balancing for module %d: cells balanced", module_id);
             stop_cell_balancing(module_id);
             notify_event(CELL_BALANCER_EVENT_COMPLETED, module_id);
         }
         else if (s_state.module_status[module_id].is_balancing) {
             // Check if we've been balancing too long
             uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
             uint32_t balance_time = current_time - s_state.module_status[module_id].balance_start_time;
             
             if (balance_time >= s_state.config.max_balance_time_ms) {
                 ESP_LOGW(TAG, "Maximum balance time reached for module %d", module_id);
                 stop_cell_balancing(module_id);
                 notify_event(CELL_BALANCER_EVENT_TIMEOUT, module_id);
             }
             else {
                 // We're still balancing, update the balancing bitmap
                 if (cell_count > 0) {
                     update_balancing_bitmap(module_id, cell_data, cell_count);
                 }
             }
         }
     }
     
     xSemaphoreGive(s_state.mutex);
     return result;
 }
 
 /**
  * @brief Start cell balancing for all modules
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_start_all(void) {
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Clear pause bit if set
     xEventGroupClearBits(s_state.event_group, BALANCING_PAUSED_BIT);
     
     // Set active bit
     xEventGroupSetBits(s_state.event_group, BALANCING_ACTIVE_BIT);
     
     ESP_LOGI(TAG, "Cell balancing enabled for all modules");
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Stop cell balancing for all modules
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_stop_all(void) {
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Clear active bit
     xEventGroupClearBits(s_state.event_group, BALANCING_ACTIVE_BIT);
     
     // Stop all active balancing
     for (uint8_t i = 0; i < s_state.module_count; i++) {
         if (s_state.module_status[i].is_balancing) {
             stop_cell_balancing(i);
             notify_event(CELL_BALANCER_EVENT_STOPPED, i);
         }
     }
     
     ESP_LOGI(TAG, "Cell balancing disabled for all modules");
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Pause cell balancing temporarily
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_pause(void) {
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Set pause bit
     xEventGroupSetBits(s_state.event_group, BALANCING_PAUSED_BIT);
     
     // Temporarily stop all active balancing
     for (uint8_t i = 0; i < s_state.module_count; i++) {
         if (s_state.module_status[i].is_balancing) {
             stop_cell_balancing(i);
             notify_event(CELL_BALANCER_EVENT_PAUSED, i);
         }
     }
     
     ESP_LOGI(TAG, "Cell balancing paused");
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Resume cell balancing after pause
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_resume(void) {
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Clear pause bit
     xEventGroupClearBits(s_state.event_group, BALANCING_PAUSED_BIT);
     
     ESP_LOGI(TAG, "Cell balancing resumed");
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Set emergency stop condition (e.g., due to thermal issue)
  * 
  * @param emergency_stop true to set emergency stop, false to clear
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_set_emergency_stop(bool emergency_stop) {
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     if (emergency_stop) {
         // Set emergency stop bit
         xEventGroupSetBits(s_state.event_group, BALANCING_EMERGENCY_STOP_BIT);
         
         // Stop all active balancing
         for (uint8_t i = 0; i < s_state.module_count; i++) {
             if (s_state.module_status[i].is_balancing) {
                 stop_cell_balancing(i);
                 notify_event(CELL_BALANCER_EVENT_EMERGENCY_STOP, i);
             }
         }
         
         ESP_LOGW(TAG, "Emergency stop activated");
     } else {
         // Clear emergency stop bit
         xEventGroupClearBits(s_state.event_group, BALANCING_EMERGENCY_STOP_BIT);
         ESP_LOGI(TAG, "Emergency stop cleared");
     }
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get the balancing status for a specific module
  * 
  * @param module_id ID of the module (0-based)
  * @param[out] status Pointer to store the status information
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_get_status(uint8_t module_id, cell_balancer_status_t *status) {
     if (module_id >= s_state.module_count || status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (!s_state.is_initialized) {
         ESP_LOGE(TAG, "Cell balancer not initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Copy status information
     status->is_balancing = s_state.module_status[module_id].is_balancing;
     status->is_balanced = !check_balancing_needed(module_id);
     status->num_cells_balancing = s_state.module_status[module_id].num_cells_balancing;
     status->highest_cell_voltage = s_state.module_status[module_id].highest_cell_voltage;
     status->lowest_cell_voltage = s_state.module_status[module_id].lowest_cell_voltage;
     status->highest_cell_idx = s_state.module_status[module_id].highest_cell_idx;
     status->lowest_cell_idx = s_state.module_status[module_id].lowest_cell_idx;
     
     // Calculate voltage difference
     status->voltage_difference = status->highest_cell_voltage - status->lowest_cell_voltage;
     
     // Calculate balance progress (rough estimate based on voltage difference)
     uint32_t threshold_v = convert_mv_to_v(s_state.config.voltage_threshold_mv);
     if (status->voltage_difference <= threshold_v) {
         status->balance_progress = 100.0f;  // Fully balanced
     } else {
         // Calculate based on initial difference vs current difference
         // This is a rough estimate and could be improved with cell modeling
         status->balance_progress = 100.0f - ((status->voltage_difference / threshold_v) * 100.0f);
         if (status->balance_progress < 0.0f) {
             status->balance_progress = 0.0f;
         }
     }
     
     // Copy balancing cells bitmap
     memcpy(status->balancing_cells_bitmap, 
            s_state.module_status[module_id].balancing_cells_bitmap, 
            sizeof(status->balancing_cells_bitmap));
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /*
  * Internal helper functions
  */
 
 /**
  * Update cell status information for a module
  */
 static esp_err_t update_cell_status(uint8_t module_id, const bess_cell_data_t *cell_data, uint8_t cell_count) {
     if (cell_count > s_state.config.cells_per_module) {
         ESP_LOGW(TAG, "Cell count %d exceeds configured cells per module %d", 
                  cell_count, s_state.config.cells_per_module);
         cell_count = s_state.config.cells_per_module;
     }
     
     // Find highest and lowest cell voltages
     float highest_voltage = 0.0f;
     float lowest_voltage = 5.0f; // Initialize higher than any expected cell voltage
     uint8_t highest_idx = 0;
     uint8_t lowest_idx = 0;
     
     for (uint8_t i = 0; i < cell_count; i++) {
         float voltage = cell_data[i].voltage;
         
         // Validate voltage reading (basic sanity check)
         if (voltage < 1.5f || voltage > 4.5f) {
             ESP_LOGW(TAG, "Invalid cell voltage reading: %.2fV for cell %d in module %d", 
                      voltage, i, module_id);
             continue;
         }
         
         if (voltage > highest_voltage) {
             highest_voltage = voltage;
             highest_idx = i;
         }
         
         if (voltage < lowest_voltage) {
             lowest_voltage = voltage;
             lowest_idx = i;
         }
     }
     
     // Update module status
     s_state.module_status[module_id].highest_cell_voltage = highest_voltage;
     s_state.module_status[module_id].lowest_cell_voltage = lowest_voltage;
     s_state.module_status[module_id].highest_cell_idx = highest_idx;
     s_state.module_status[module_id].lowest_cell_idx = lowest_idx;
     s_state.module_status[module_id].is_initialized = true;
     
     return ESP_OK;
 }
 
 /**
  * Start cell balancing for a specific module
  */
 static esp_err_t start_cell_balancing(uint8_t module_id) {
     if (module_id >= s_state.module_count) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check system state
     EventBits_t bits = xEventGroupGetBits(s_state.event_group);
     if ((bits & BALANCING_ACTIVE_BIT) == 0 || 
         (bits & BALANCING_PAUSED_BIT) != 0 ||
         (bits & BALANCING_EMERGENCY_STOP_BIT) != 0) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Record start time
     s_state.module_status[module_id].balance_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
     s_state.module_status[module_id].is_balancing = true;
     
     ESP_LOGI(TAG, "Balancing started for module %d", module_id);
     
     return ESP_OK;
 }
 
 /**
  * Stop cell balancing for a specific module
  */
 static esp_err_t stop_cell_balancing(uint8_t module_id) {
     if (module_id >= s_state.module_count) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Only update if currently balancing
     if (s_state.module_status[module_id].is_balancing) {
         // Record total balance time
         uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
         uint32_t balance_time = current_time - s_state.module_status[module_id].balance_start_time;
         s_state.module_status[module_id].total_balance_time += balance_time;
         
         // Clear balancing flag
         s_state.module_status[module_id].is_balancing = false;
         
         // Clear balancing cells bitmap
         memset(s_state.module_status[module_id].balancing_cells_bitmap, 0, 
                sizeof(s_state.module_status[module_id].balancing_cells_bitmap));
         s_state.module_status[module_id].num_cells_balancing = 0;
         
         ESP_LOGI(TAG, "Balancing stopped for module %d, duration: %lu ms", 
                  module_id, balance_time);
     }
     
     return ESP_OK;
 }
 
 /**
  * Check if balancing is needed for a module
  */
 static bool check_balancing_needed(uint8_t module_id) {
     if (module_id >= s_state.module_count || !s_state.module_status[module_id].is_initialized) {
         return false;
     }
     
     // Calculate voltage difference
     float voltage_diff = s_state.module_status[module_id].highest_cell_voltage - 
                          s_state.module_status[module_id].lowest_cell_voltage;
     
     // Convert threshold from mV to V
     float threshold_v = convert_mv_to_v(s_state.config.voltage_threshold_mv);
     
     // Check if the difference exceeds the threshold
     return (voltage_diff > threshold_v);
 }
 
 /**
  * Notify a balancing event via callback
  */
 static void notify_event(cell_balancer_event_t event, uint8_t module_id) {
     if (s_state.event_callback != NULL) {
         s_state.event_callback(event, module_id, s_state.callback_arg);
     }
 }
 
 /**
  * Update the bitmap of cells being balanced
  */
 static void update_balancing_bitmap(uint8_t module_id, const bess_cell_data_t *cell_data, uint8_t cell_count) {
     if (module_id >= s_state.module_count || cell_data == NULL || cell_count == 0) {
         return;
     }
     
     // Clear the bitmap
     memset(s_state.module_status[module_id].balancing_cells_bitmap, 0, 
            sizeof(s_state.module_status[module_id].balancing_cells_bitmap));
     
     uint8_t num_cells_balancing = 0;
     
     // Apply balancing logic based on mode
     switch (s_state.config.mode) {
         case CELL_BALANCING_MODE_PASSIVE:
             // In passive balancing, we discharge higher voltage cells to match lower ones
             for (uint8_t i = 0; i < cell_count; i++) {
                 // Check if cell voltage is above lowest cell by threshold
                 float voltage_diff = cell_data[i].voltage - s_state.module_status[module_id].lowest_cell_voltage;
                 float threshold_v = convert_mv_to_v(s_state.config.voltage_threshold_mv);
                 
                 if (voltage_diff > threshold_v / 2) {  // Use half threshold for hysteresis
                     // Set the bit for this cell (enable balancing)
                     uint8_t byte_idx = i / 8;
                     uint8_t bit_pos = i % 8;
                     
                     if (byte_idx < sizeof(s_state.module_status[module_id].balancing_cells_bitmap)) {
                         s_state.module_status[module_id].balancing_cells_bitmap[byte_idx] |= (1 << bit_pos);
                         num_cells_balancing++;
                     }
                 }
             }
             break;
             
         case CELL_BALANCING_MODE_ACTIVE:
             // In active balancing, we transfer charge from higher to lower voltage cells
             // This is a simplified model - actual implementation would depend on hardware
             
             // Enable balancing for the highest cell (source)
             {
                 uint8_t high_idx = s_state.module_status[module_id].highest_cell_idx;
                 uint8_t byte_idx = high_idx / 8;
                 uint8_t bit_pos = high_idx % 8;
                 
                 if (byte_idx < sizeof(s_state.module_status[module_id].balancing_cells_bitmap)) {
                     s_state.module_status[module_id].balancing_cells_bitmap[byte_idx] |= (1 << bit_pos);
                     num_cells_balancing++;
                 }
             }
             
             // Enable balancing for the lowest cell (destination)
             {
                 uint8_t low_idx = s_state.module_status[module_id].lowest_cell_idx;
                 uint8_t byte_idx = low_idx / 8;
                 uint8_t bit_pos = low_idx % 8;
                 
                 if (byte_idx < sizeof(s_state.module_status[module_id].balancing_cells_bitmap)) {
                     s_state.module_status[module_id].balancing_cells_bitmap[byte_idx] |= (1 << bit_pos);
                     num_cells_balancing++;
                 }
             }
             break;
             
         case CELL_BALANCING_MODE_HYBRID:
             // Hybrid mode - combine passive and active approaches
             // For cells with significant imbalance, use active balancing
             // For minor imbalances, use passive balancing
             
             float high_threshold_v = convert_mv_to_v(s_state.config.voltage_threshold_mv * 2);
             float low_threshold_v = convert_mv_to_v(s_state.config.voltage_threshold_mv);
             
             for (uint8_t i = 0; i < cell_count; i++) {
                 float voltage_diff = cell_data[i].voltage - s_state.module_status[module_id].lowest_cell_voltage;
                 
                 if (voltage_diff > low_threshold_v) {
                     // Set the bit for this cell (enable balancing)
                     uint8_t byte_idx = i / 8;
                     uint8_t bit_pos = i % 8;
                     
                     if (byte_idx < sizeof(s_state.module_status[module_id].balancing_cells_bitmap)) {
                         s_state.module_status[module_id].balancing_cells_bitmap[byte_idx] |= (1 << bit_pos);
                         num_cells_balancing++;
                     }
                 }
             }
             break;
             
         case CELL_BALANCING_MODE_NONE:
         default:
             // No balancing, clear all bits
             break;
     }
     
     // Update count of cells being balanced
     s_state.module_status[module_id].num_cells_balancing = num_cells_balancing;
     
     ESP_LOGD(TAG, "Module %d: %d cells being balanced", module_id, num_cells_balancing);
 }
 
 /**
  * Convert millivolts to volts
  */
 static float convert_mv_to_v(uint32_t mv) {
     return (float)mv / 1000.0f;
 }
 
 /**
  * Convert volts to millivolts
  */
 static uint32_t convert_v_to_mv(float v) {
     return (uint32_t)(v * 1000.0f);
 }