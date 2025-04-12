/**
 * @file soc_calculator.h
 * @brief State of Charge (SoC) estimation algorithms
 * 
 * Provides functions to calculate battery State of Charge using
 * various methods including Coulomb counting, OCV-based estimation,
 * and combined/Kalman filter approaches.
 */

 #ifndef SOC_CALCULATOR_H
 #define SOC_CALCULATOR_H
 
 #include "esp_err.h"
 #include "bess_types.h"
 
 /**
  * @brief SoC calculation methods
  */
 typedef enum {
     SOC_METHOD_COULOMB_COUNTING,    /**< Coulomb counting method (current integration) */
     SOC_METHOD_OCV,                 /**< Open Circuit Voltage based method */
     SOC_METHOD_KALMAN,              /**< Kalman filter based fusion */
     SOC_METHOD_HYBRID               /**< Hybrid approach */
 } soc_calculation_method_t;
 
 /**
  * @brief Initialize the SoC calculator
  * 
  * @param method Calculation method to use
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_init(soc_calculation_method_t method);
 
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
                                               float *soc);
 
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
                                               float *system_soc);
 
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
                                                 float time_delta);
 
 /**
  * @brief Reset the SoC calculation for a specific module
  * 
  * @param module_id ID of the module
  * @param initial_soc Initial SoC value (0-100%)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_reset(uint8_t module_id, float initial_soc);
 
 /**
  * @brief Calibrate the SoC calculation based on OCV measurements
  * 
  * @param module_id ID of the module
  * @param rest_voltage Measured voltage after battery has been at rest
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_calibrate_from_ocv(uint8_t module_id, float rest_voltage);
 
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
                                       size_t *table_size);
 
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
                                           float error_covariance);
 
 /**
  * @brief Get the SoC estimation error
  * 
  * @param module_id ID of the module
  * @param[out] error Pointer to store the estimated error (percentage points)
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_get_estimation_error(uint8_t module_id, float *error);
 
 /**
  * @brief Adjust SoC based on temperature compensation
  * 
  * @param module_id ID of the module
  * @param temperature Current temperature in Celsius
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 esp_err_t soc_calculator_temperature_compensation(uint8_t module_id, float temperature);
 
 #endif /* SOC_CALCULATOR_H */