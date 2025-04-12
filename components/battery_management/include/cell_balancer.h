/**
 * @file cell_balancer.h
 * @brief Cell balancing functionality for LFP battery packs
 * 
 * Provides functions to manage cell balancing, ensuring uniform voltage levels
 * across all cells to maximize capacity and lifespan of LFP battery modules.
 */

 #ifndef CELL_BALANCER_H
 #define CELL_BALANCER_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 
 /**
  * @brief Cell balancing configuration parameters
  */
 typedef struct {
     uint32_t voltage_threshold_mv;    // Voltage difference threshold to trigger balancing (mV)
     uint32_t balance_current_ma;      // Balancing current (mA)
     uint32_t min_cell_voltage_mv;     // Minimum cell voltage for balancing (mV)
     uint32_t max_balance_time_ms;     // Maximum time to balance continuously (ms)
     uint32_t rest_time_ms;            // Rest time between balancing cycles (ms)
     uint8_t cells_per_module;         // Number of cells per module
     uint8_t mode;                     // Balancing mode (0=none, 1=passive, 2=active, 3=hybrid)
 } cell_balancer_config_t;
 
 /**
  * @brief Cell balancer status information
  */
 typedef struct {
     bool is_balancing;                // Whether module is currently being balanced
     bool is_balanced;                 // Whether module cells are balanced
     uint8_t num_cells_balancing;      // Number of cells currently being balanced
     float highest_cell_voltage;       // Current highest cell voltage (V)
     float lowest_cell_voltage;        // Current lowest cell voltage (V)
     float voltage_difference;         // Difference between highest and lowest cell (V)
     uint8_t highest_cell_idx;         // Index of highest voltage cell
     uint8_t lowest_cell_idx;          // Index of lowest voltage cell
     float balance_progress;           // Estimated balance progress (0-100%)
     uint8_t balancing_cells_bitmap[4]; // Bitmap of cells being balanced (up to 32 cells)
 } cell_balancer_status_t;
 
 /**
  * @brief Cell balancer events
  */
 typedef enum {
     CELL_BALANCER_EVENT_STARTED,       // Balancing started
     CELL_BALANCER_EVENT_COMPLETED,     // Balancing completed successfully
     CELL_BALANCER_EVENT_STOPPED,       // Balancing stopped manually
     CELL_BALANCER_EVENT_PAUSED,        // Balancing paused
     CELL_BALANCER_EVENT_RESUMED,       // Balancing resumed
     CELL_BALANCER_EVENT_TIMEOUT,       // Maximum balance time reached
     CELL_BALANCER_EVENT_EMERGENCY_STOP // Emergency stop triggered
 } cell_balancer_event_t;
 
 /**
  * @brief Cell balancer event callback function type
  */
 typedef void (*cell_balancer_callback_t)(cell_balancer_event_t event, 
                                       uint8_t module_id, 
                                       void *arg);
 
 /**
  * @brief Initialize the cell balancer
  * 
  * @param config Pointer to configuration parameters (NULL for defaults)
  * @param module_count Number of battery modules in the system
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_init(const cell_balancer_config_t *config, uint8_t module_count);
 
 /**
  * @brief Configure the cell balancer parameters
  * 
  * @param config Pointer to configuration parameters
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_configure(const cell_balancer_config_t *config);
 
 /**
  * @brief Register a callback for cell balancing events
  * 
  * @param callback Function to call when events occur
  * @param arg User argument to pass to the callback
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_register_callback(cell_balancer_callback_t callback, void *arg);
 
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
                                        uint8_t cell_count);
 
 /**
  * @brief Start cell balancing for all modules
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_start_all(void);
 
 /**
  * @brief Stop cell balancing for all modules
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_stop_all(void);
 
 /**
  * @brief Pause cell balancing temporarily
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_pause(void);
 
 /**
  * @brief Resume cell balancing after pause
  * 
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_resume(void);
 
 /**
  * @brief Set emergency stop condition (e.g., due to thermal issue)
  * 
  * @param emergency_stop true to set emergency stop, false to clear
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_set_emergency_stop(bool emergency_stop);
 
 /**
  * @brief Get the balancing status for a specific module
  * 
  * @param module_id ID of the module (0-based)
  * @param[out] status Pointer to store the status information
  * @return esp_err_t ESP_OK if successful, error code otherwise
  */
 esp_err_t cell_balancer_get_status(uint8_t module_id, cell_balancer_status_t *status);
 
 #endif /* CELL_BALANCER_H */