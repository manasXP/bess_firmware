/**
 * @file soc_calculator.c
 * @brief Implementation of State of Charge (SoC) estimation algorithms
 * 
 * This file implements various methods for calculating battery State of Charge
 * including Coulomb counting, OCV-based estimation, Kalman filter, and hybrid approaches.
 */

 #include "soc_calculator.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/semphr.h"
 #include <math.h>
 #include <string.h>
 
 #define TAG "SOC_CALC"
 
 // Constants for LFP battery characteristics
 #define MAX_MODULE_COUNT            16      // Maximum number of battery modules
 #define NOMINAL_MODULE_CAPACITY     16000.0 // 16 kWh in Wh
 #define FULLY_CHARGED_VOLTAGE       54.0    // Fully charged LFP module voltage
 #define FULLY_DISCHARGED_VOLTAGE    40.0    // Fully discharged LFP module voltage
 #define CAPACITY_TEMPERATURE_FACTOR 0.005   // 0.5% capacity loss per °C below optimal
 
 // Lookup table for LFP OCV-SoC relationship (Voltage to SoC mapping)
 // LFP batteries have a relatively flat discharge curve
 static const float OCV_SOC_TABLE[][2] = {
     {54.0, 100.0}, // 100% SoC
     {53.4, 95.0},  // 95% SoC
     {52.8, 90.0},  // 90% SoC
     {52.2, 85.0},  // 85% SoC
     {51.6, 80.0},  // 80% SoC
     {51.0, 70.0},  // 70% SoC
     {50.4, 60.0},  // 60% SoC
     {49.8, 50.0},  // 50% SoC
     {49.2, 40.0},  // 40% SoC
     {48.6, 30.0},  // 30% SoC
     {48.0, 20.0},  // 20% SoC
     {47.0, 15.0},  // 15% SoC
     {46.0, 10.0},  // 10% SoC
     {44.0, 5.0},   // 5% SoC
     {40.0, 0.0},   // 0% SoC
 };
 #define OCV_TABLE_SIZE (sizeof(OCV_SOC_TABLE) / sizeof(OCV_SOC_TABLE[0]))
 
 // Temperature compensation table (°C, factor)
 static const float TEMP_COMP_TABLE[][2] = {
     {-20.0, 0.70}, // At -20°C, available capacity is 70% of nominal
     {-10.0, 0.80}, // At -10°C, available capacity is 80% of nominal
     {0.0,   0.90}, // At 0°C, available capacity is 90% of nominal
     {10.0,  0.95}, // At 10°C, available capacity is 95% of nominal
     {25.0,  1.00}, // At 25°C, available capacity is 100% of nominal (optimal)
     {40.0,  0.98}, // At 40°C, available capacity is 98% of nominal
     {50.0,  0.95}, // At 50°C, available capacity is 95% of nominal
     {60.0,  0.90}  // At 60°C, available capacity is 90% of nominal
 };
 #define TEMP_COMP_TABLE_SIZE (sizeof(TEMP_COMP_TABLE) / sizeof(TEMP_COMP_TABLE[0]))
 
 // Module data structure
 typedef struct {
     float current_soc;           // Current SoC value (0-100%)
     float accumulated_charge;    // Accumulated charge in Ah
     float nominal_capacity;      // Nominal capacity in Ah
     float kalman_soc;            // Kalman filter SoC estimate
     float kalman_p;              // Kalman filter error covariance
     float last_ocv;              // Last measured open circuit voltage
     bool  is_initialized;        // Flag indicating if module is initialized
     float current_estimation_error; // Current estimation error
 } module_soc_data_t;
 
 // SOC calculator state
 typedef struct {
     soc_calculation_method_t method;     // Current calculation method
     module_soc_data_t modules[MAX_MODULE_COUNT]; // Per-module SoC data
     SemaphoreHandle_t mutex;            // Mutex for thread safety
     
     // Kalman filter parameters (shared across modules)
     float process_noise;        // Process noise covariance (Q)
     float measurement_noise;    // Measurement noise covariance (R)
 } soc_calculator_state_t;
 
 // Global state
 static soc_calculator_state_t s_state = {
     .method = SOC_METHOD_HYBRID,
     .process_noise = 0.01,      // Default process noise
     .measurement_noise = 1.0    // Default measurement noise
 };
 
 // Forward declarations for internal functions
 static float calculate_soc_from_ocv(float ocv);
 static float interpolate_ocv_soc(float ocv);
 static float get_temperature_factor(float temperature);
 static void update_kalman_filter(uint8_t module_id, float measured_soc);
 static float limit_soc_value(float soc);
 
 /**
  * @brief Initialize the SoC calculator
  * 
  * @param method Calculation method to use
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_init(soc_calculation_method_t method) {
     // Create mutex for thread safety
     s_state.mutex = xSemaphoreCreateMutex();
     if (s_state.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_ERR_NO_MEM;
     }
     
     s_state.method = method;
     
     // Initialize module data
     for (int i = 0; i < MAX_MODULE_COUNT; i++) {
         s_state.modules[i].current_soc = 0.0;
         s_state.modules[i].accumulated_charge = 0.0;
         s_state.modules[i].nominal_capacity = NOMINAL_MODULE_CAPACITY / 48.0; // Wh to Ah conversion (nominal 48V)
         s_state.modules[i].kalman_soc = 0.0;
         s_state.modules[i].kalman_p = 1.0;  // Initial error covariance
         s_state.modules[i].last_ocv = 0.0;
         s_state.modules[i].is_initialized = false;
         s_state.modules[i].current_estimation_error = 5.0; // Initial 5% error estimate
     }
     
     ESP_LOGI(TAG, "SoC calculator initialized with method: %d", method);
     return ESP_OK;
 }
 
 /**
  * @brief Calculate SoC for a specific module
  * 
  * @param module_id ID of the module to calculate SoC for
  * @param module_data Current module data
  * @param[out] soc Pointer to store the calculated SoC (0-100%)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_calculate_module_soc(uint8_t module_id, 
                                              const bess_module_data_t *module_data,
                                              float *soc) {
     if (module_id >= MAX_MODULE_COUNT || module_data == NULL || soc == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // If module not initialized, initialize with OCV-based SoC
     if (!s_state.modules[module_id].is_initialized) {
         float initial_soc = calculate_soc_from_ocv(module_data->voltage);
         s_state.modules[module_id].current_soc = initial_soc;
         s_state.modules[module_id].kalman_soc = initial_soc;
         s_state.modules[module_id].accumulated_charge = 
             (initial_soc / 100.0) * s_state.modules[module_id].nominal_capacity;
         s_state.modules[module_id].last_ocv = module_data->voltage;
         s_state.modules[module_id].is_initialized = true;
     }
     
     float calculated_soc = 0.0;
     
     switch (s_state.method) {
         case SOC_METHOD_COULOMB_COUNTING:
             // SoC is directly taken from the accumulated charge
             calculated_soc = (s_state.modules[module_id].accumulated_charge / 
                              s_state.modules[module_id].nominal_capacity) * 100.0;
             break;
             
         case SOC_METHOD_OCV:
             // Only use OCV if current is low (battery at rest)
             if (fabs(module_data->current) < 0.1) { // Less than 100mA
                 calculated_soc = calculate_soc_from_ocv(module_data->voltage);
                 s_state.modules[module_id].last_ocv = module_data->voltage;
             } else {
                 // If current is significant, use last calculated value
                 calculated_soc = s_state.modules[module_id].current_soc;
             }
             break;
             
         case SOC_METHOD_KALMAN:
             // If current is low, update Kalman with OCV measurement
             if (fabs(module_data->current) < 0.1) { // Less than 100mA
                 float ocv_soc = calculate_soc_from_ocv(module_data->voltage);
                 update_kalman_filter(module_id, ocv_soc);
                 s_state.modules[module_id].last_ocv = module_data->voltage;
             }
             calculated_soc = s_state.modules[module_id].kalman_soc;
             break;
             
         case SOC_METHOD_HYBRID:
         default:
             // Use Coulomb counting but periodically calibrate with OCV
             calculated_soc = (s_state.modules[module_id].accumulated_charge / 
                              s_state.modules[module_id].nominal_capacity) * 100.0;
             
             // If current is low, calibrate using OCV
             if (fabs(module_data->current) < 0.1) { // Less than 100mA
                 float ocv_soc = calculate_soc_from_ocv(module_data->voltage);
                 // Weighted average between Coulomb counting and OCV
                 calculated_soc = calculated_soc * 0.7 + ocv_soc * 0.3;
                 
                 // Update accumulated charge for better Coulomb counting in future
                 s_state.modules[module_id].accumulated_charge = 
                     (calculated_soc / 100.0) * s_state.modules[module_id].nominal_capacity;
                 
                 s_state.modules[module_id].last_ocv = module_data->voltage;
             }
             break;
     }
     
     // Apply temperature compensation if temperature data is available
     if (module_data->temperature > -50.0f) { // Valid temperature reading
         float temp_factor = get_temperature_factor(module_data->temperature);
         // Adjust the available capacity based on temperature
         float temp_adjusted_soc = calculated_soc / temp_factor;
         
         // Don't allow temperature to cause SoC to exceed 100%
         if (temp_adjusted_soc > 100.0f && calculated_soc <= 100.0f) {
             temp_adjusted_soc = 100.0f;
         }
         
         calculated_soc = temp_adjusted_soc;
     }
     
     // Ensure SoC stays within valid range
     calculated_soc = limit_soc_value(calculated_soc);
     
     // Update module data
     s_state.modules[module_id].current_soc = calculated_soc;
     
     // Update error estimation
     float current_abs = fabs(module_data->current);
     if (current_abs < 0.1) { // At rest
         s_state.modules[module_id].current_estimation_error = 2.0; // Assume 2% error at rest
     } else if (current_abs > 10.0) { // High current
         s_state.modules[module_id].current_estimation_error = 5.0; // Assume 5% error at high current
     }
     
     // Copy the result
     *soc = calculated_soc;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Calculate the overall system SoC
  * 
  * @param module_data Array of module data for all modules
  * @param module_count Number of modules
  * @param[out] system_soc Pointer to store the calculated system SoC (0-100%)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_calculate_system_soc(const bess_module_data_t *module_data,
                                              uint8_t module_count,
                                              float *system_soc) {
     if (module_data == NULL || system_soc == NULL || module_count == 0 || 
         module_count > MAX_MODULE_COUNT) {
         return ESP_ERR_INVALID_ARG;
     }
     
     float soc_sum = 0.0f;
     float energy_sum = 0.0f;
     float total_nominal_energy = 0.0f;
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     for (uint8_t i = 0; i < module_count; i++) {
         // Get individual module SoC
         float module_soc = s_state.modules[i].current_soc;
         
         // Calculate energy weights for weighted average
         float module_nominal_energy = s_state.modules[i].nominal_capacity * 48.0; // Ah * nominal V
         float module_energy = module_nominal_energy * (module_soc / 100.0);
         
         soc_sum += module_soc;
         energy_sum += module_energy;
         total_nominal_energy += module_nominal_energy;
     }
     
     // Calculate system SoC as weighted average
     if (total_nominal_energy > 0) {
         *system_soc = (energy_sum / total_nominal_energy) * 100.0;
     } else {
         // Fallback to simple average if nominal energies are not available
         *system_soc = soc_sum / module_count;
     }
     
     // Ensure system SoC is within valid range
     *system_soc = limit_soc_value(*system_soc);
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Update the coulomb counting with new current measurement
  * 
  * @param module_id ID of the module
  * @param current Current in amperes (positive for charging, negative for discharging)
  * @param time_delta Time since last update in seconds
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_update_coulomb_counting(uint8_t module_id, 
                                                float current, 
                                                float time_delta) {
     if (module_id >= MAX_MODULE_COUNT || time_delta <= 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // If not initialized, initialize with 50% SoC (moderate assumption)
     if (!s_state.modules[module_id].is_initialized) {
         s_state.modules[module_id].current_soc = 50.0;
         s_state.modules[module_id].accumulated_charge = 
             0.5 * s_state.modules[module_id].nominal_capacity;
         s_state.modules[module_id].is_initialized = true;
     }
     
     // Calculate charge change (Ah) = current (A) * time (h)
     float charge_delta = current * (time_delta / 3600.0);
     
     // Account for charging efficiency (typically 95-98% for LFP batteries)
     if (current > 0) { // Charging
         charge_delta *= 0.97; // 97% charging efficiency
     }
     
     // Update accumulated charge
     s_state.modules[module_id].accumulated_charge += charge_delta;
     
     // Ensure accumulated charge doesn't exceed bounds
     if (s_state.modules[module_id].accumulated_charge < 0) {
         s_state.modules[module_id].accumulated_charge = 0;
     } else if (s_state.modules[module_id].accumulated_charge > 
                s_state.modules[module_id].nominal_capacity) {
         s_state.modules[module_id].accumulated_charge = 
             s_state.modules[module_id].nominal_capacity;
     }
     
     // Update SoC percentage
     s_state.modules[module_id].current_soc = 
         (s_state.modules[module_id].accumulated_charge / 
          s_state.modules[module_id].nominal_capacity) * 100.0;
     
     // Update Kalman filter state prediction if using Kalman
     if (s_state.method == SOC_METHOD_KALMAN) {
         // State prediction step
         s_state.modules[module_id].kalman_soc += 
             (charge_delta / s_state.modules[module_id].nominal_capacity) * 100.0;
         
         // Error covariance prediction step
         s_state.modules[module_id].kalman_p += s_state.process_noise;
         
         // Ensure Kalman SoC is within bounds
         s_state.modules[module_id].kalman_soc = 
             limit_soc_value(s_state.modules[module_id].kalman_soc);
     }
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Reset the SoC calculation for a specific module
  * 
  * @param module_id ID of the module
  * @param initial_soc Initial SoC value (0-100%)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_reset(uint8_t module_id, float initial_soc) {
     if (module_id >= MAX_MODULE_COUNT || initial_soc < 0 || initial_soc > 100) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Reset module data
     s_state.modules[module_id].current_soc = initial_soc;
     s_state.modules[module_id].accumulated_charge = 
         (initial_soc / 100.0) * s_state.modules[module_id].nominal_capacity;
     s_state.modules[module_id].kalman_soc = initial_soc;
     s_state.modules[module_id].kalman_p = 1.0;  // Reset error covariance
     s_state.modules[module_id].is_initialized = true;
     s_state.modules[module_id].current_estimation_error = 2.0; // Assume 2% error initially
     
     ESP_LOGI(TAG, "Reset module %d SoC to %.1f%%", module_id, initial_soc);
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Calibrate the SoC calculation based on OCV measurements
  * 
  * @param module_id ID of the module
  * @param rest_voltage Measured voltage after battery has been at rest
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_calibrate_from_ocv(uint8_t module_id, float rest_voltage) {
     if (module_id >= MAX_MODULE_COUNT || 
         rest_voltage < FULLY_DISCHARGED_VOLTAGE || 
         rest_voltage > FULLY_CHARGED_VOLTAGE) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     // Calculate SoC from OCV
     float ocv_soc = calculate_soc_from_ocv(rest_voltage);
     
     ESP_LOGI(TAG, "Calibrating module %d from OCV: %.2fV -> %.1f%% SoC", 
              module_id, rest_voltage, ocv_soc);
     
     // Update module data based on method
     switch (s_state.method) {
         case SOC_METHOD_COULOMB_COUNTING:
             // Reset accumulated charge based on OCV SoC
             s_state.modules[module_id].accumulated_charge = 
                 (ocv_soc / 100.0) * s_state.modules[module_id].nominal_capacity;
             s_state.modules[module_id].current_soc = ocv_soc;
             break;
             
         case SOC_METHOD_OCV:
             // Direct assignment for OCV method
             s_state.modules[module_id].current_soc = ocv_soc;
             break;
             
         case SOC_METHOD_KALMAN:
             // Update Kalman with OCV measurement
             update_kalman_filter(module_id, ocv_soc);
             break;
             
         case SOC_METHOD_HYBRID:
         default:
             // For hybrid, blend current estimate with OCV
             float current_soc = s_state.modules[module_id].current_soc;
             float blended_soc = current_soc * 0.3 + ocv_soc * 0.7; // Higher weight to OCV during calibration
             
             s_state.modules[module_id].current_soc = blended_soc;
             s_state.modules[module_id].accumulated_charge = 
                 (blended_soc / 100.0) * s_state.modules[module_id].nominal_capacity;
             break;
     }
     
     s_state.modules[module_id].last_ocv = rest_voltage;
     s_state.modules[module_id].is_initialized = true;
     
     // Reset estimation error after calibration
     s_state.modules[module_id].current_estimation_error = 2.0;
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get the SoC-OCV table for a specific cell type
  * 
  * @param[out] ocv_table Pointer to array to store the OCV table values
  * @param[out] soc_points Pointer to array to store the SoC points (0-100%)
  * @param[out] table_size Pointer to store the size of the table
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_get_ocv_table(float **ocv_table, 
                                      float **soc_points, 
                                      size_t *table_size) {
     if (ocv_table == NULL || soc_points == NULL || table_size == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Allocate memory for tables
     float *ocv_values = (float *)malloc(OCV_TABLE_SIZE * sizeof(float));
     float *soc_values = (float *)malloc(OCV_TABLE_SIZE * sizeof(float));
     
     if (ocv_values == NULL || soc_values == NULL) {
         free(ocv_values);
         free(soc_values);
         return ESP_ERR_NO_MEM;
     }
     
     // Copy values from the static table
     for (size_t i = 0; i < OCV_TABLE_SIZE; i++) {
         ocv_values[i] = OCV_SOC_TABLE[i][0];
         soc_values[i] = OCV_SOC_TABLE[i][1];
     }
     
     *ocv_table = ocv_values;
     *soc_points = soc_values;
     *table_size = OCV_TABLE_SIZE;
     
     return ESP_OK;
 }
 
 /**
  * @brief Set the Kalman filter parameters
  * 
  * @param process_noise Process noise covariance
  * @param measurement_noise Measurement noise covariance
  * @param error_covariance Initial error covariance
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_set_kalman_params(float process_noise,
                                          float measurement_noise,
                                          float error_covariance) {
     if (process_noise <= 0 || measurement_noise <= 0 || error_covariance <= 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     s_state.process_noise = process_noise;
     s_state.measurement_noise = measurement_noise;
     
     // Update initial error covariance for all modules
     for (int i = 0; i < MAX_MODULE_COUNT; i++) {
         s_state.modules[i].kalman_p = error_covariance;
     }
     
     ESP_LOGI(TAG, "Kalman parameters updated: Q=%.4f, R=%.4f, P0=%.4f", 
              process_noise, measurement_noise, error_covariance);
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get the SoC estimation error
  * 
  * @param module_id ID of the module
  * @param[out] error Pointer to store the estimated error (percentage points)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_get_estimation_error(uint8_t module_id, float *error) {
     if (module_id >= MAX_MODULE_COUNT || error == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     if (!s_state.modules[module_id].is_initialized) {
         *error = 10.0; // High error for uninitialized modules
     } else {
         *error = s_state.modules[module_id].current_estimation_error;
     }
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Adjust SoC based on temperature compensation
  * 
  * @param module_id ID of the module
  * @param temperature Current temperature in Celsius
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_temperature_compensation(uint8_t module_id, float temperature) {
     if (module_id >= MAX_MODULE_COUNT || temperature < -30.0 || temperature > 70.0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to take mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     if (!s_state.modules[module_id].is_initialized) {
         xSemaphoreGive(s_state.mutex);
         return ESP_ERR_INVALID_STATE;
     }
     
     // Get temperature compensation factor
     float temp_factor = get_temperature_factor(temperature);
     
     // Apply compensation to SoC
     float current_soc = s_state.modules[module_id].current_soc;
     float compensated_soc = current_soc / temp_factor;
     
     // Ensure compensated SoC stays within valid range
     compensated_soc = limit_soc_value(compensated_soc);
     
     // Update module SoC
     s_state.modules[module_id].current_soc = compensated_soc;
     
     ESP_LOGD(TAG, "Module %d temperature compensation: %.1f°C, factor=%.2f, SoC: %.1f%% -> %.1f%%",
              module_id, temperature, temp_factor, current_soc, compensated_soc);
     
     xSemaphoreGive(s_state.mutex);
     return ESP_OK;
 }
 
 /*
  * Internal helper functions
  */
 
 /**
  * Calculate SoC from Open Circuit Voltage using lookup table
  */
 static float calculate_soc_from_ocv(float ocv) {
     return interpolate_ocv_soc(ocv);
 }
 
 /**
  * Interpolate SoC from OCV using the lookup table
  */
 static float interpolate_ocv_soc(float ocv) {
     // Ensure voltage is within bounds
     if (ocv >= FULLY_CHARGED_VOLTAGE) {
         return 100.0;
     } else if (ocv <= FULLY_DISCHARGED_VOLTAGE) {
         return 0.0;
     }
     
     // Linear interpolation between table points
     for (size_t i = 0; i < OCV_TABLE_SIZE - 1; i++) {
         if (ocv <= OCV_SOC_TABLE[i][0] && ocv >= OCV_SOC_TABLE[i+1][0]) {
             float voltage_range = OCV_SOC_TABLE[i][0] - OCV_SOC_TABLE[i+1][0];
             float soc_range = OCV_SOC_TABLE[i][1] - OCV_SOC_TABLE[i+1][1];
             float voltage_diff = OCV_SOC_TABLE[i][0] - ocv;
             
             return OCV_SOC_TABLE[i][1] - (voltage_diff / voltage_range) * soc_range;
         }
     }
     
     // If we get here, there's something wrong with the table or input
     ESP_LOGW(TAG, "OCV outside expected range: %.2fV", ocv);
     
     // Fallback linear interpolation across entire range
     float voltage_range = FULLY_CHARGED_VOLTAGE - FULLY_DISCHARGED_VOLTAGE;
     float normalized_voltage = (ocv - FULLY_DISCHARGED_VOLTAGE) / voltage_range;
     return normalized_voltage * 100.0;
 }
 
 /**
  * Get temperature compensation factor based on current temperature
  */
 static float get_temperature_factor(float temperature) {
     // Ensure temperature is within bounds
     if (temperature <= TEMP_COMP_TABLE[0][0]) {
         return TEMP_COMP_TABLE[0][1];
     } else if (temperature >= TEMP_COMP_TABLE[TEMP_COMP_TABLE_SIZE-1][0]) {
         return TEMP_COMP_TABLE[TEMP_COMP_TABLE_SIZE-1][1];
     }
     
     // Linear interpolation between table points
     for (size_t i = 0; i < TEMP_COMP_TABLE_SIZE - 1; i++) {
         if (temperature >= TEMP_COMP_TABLE[i][0] && temperature <= TEMP_COMP_TABLE[i+1][0]) {
             float temp_range = TEMP_COMP_TABLE[i+1][0] - TEMP_COMP_TABLE[i][0];
             float factor_range = TEMP_COMP_TABLE[i+1][1] - TEMP_COMP_TABLE[i][1];
             float temp_diff = temperature - TEMP_COMP_TABLE[i][0];
             
             return TEMP_COMP_TABLE[i][1] + (temp_diff / temp_range) * factor_range;
         }
     }
     
     // Fallback value (should never reach here)
     return 1.0;
 }
 
 /**
  * Update Kalman filter with a new measurement
  */
 static void update_kalman_filter(uint8_t module_id, float measured_soc) {
     // Skip update if module is not initialized
     if (!s_state.modules[module_id].is_initialized) {
         return;
     }
     
     // Kalman gain calculation
     float kalman_gain = s_state.modules[module_id].kalman_p / 
                          (s_state.modules[module_id].kalman_p + s_state.measurement_noise);
     
     // State update
     s_state.modules[module_id].kalman_soc += 
         kalman_gain * (measured_soc - s_state.modules[module_id].kalman_soc);
     
     // Error covariance update
     s_state.modules[module_id].kalman_p = 
         (1 - kalman_gain) * s_state.modules[module_id].kalman_p;
     
     // Ensure Kalman SoC is within bounds
     s_state.modules[module_id].kalman_soc = 
         limit_soc_value(s_state.modules[module_id].kalman_soc);
     
     ESP_LOGD(TAG, "Module %d Kalman update: measurement=%.1f%%, K=%.3f, updated=%.1f%%", 
              module_id, measured_soc, kalman_gain, s_state.modules[module_id].kalman_soc);
 }
 
 /**
  * Ensure SoC value stays within valid range (0-100%)
  */
 static float limit_soc_value(float soc) {
     if (soc < 0.0f) {
         return 0.0f;
     } else if (soc > 100.0f) {
         return 100.0f;
     }
     return soc;
 }