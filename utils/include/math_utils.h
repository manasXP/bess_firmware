/**
 * @file math_utils.h
 * @brief Mathematical utilities for the BESS firmware
 * 
 * This file contains mathematical utilities, functions, and macros
 * used throughout the BESS (Battery Energy Storage System) firmware.
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef MATH_UTILS_H
 #define MATH_UTILS_H
 
 #include <math.h>
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 
 /**
  * @brief Floating point comparison epsilon
  */
 #define MATH_FLOAT_EPSILON 0.000001f
 
 /**
  * @brief Get the minimum of two values
  */
 #define MATH_MIN(a, b) ((a) < (b) ? (a) : (b))
 
 /**
  * @brief Get the maximum of two values
  */
 #define MATH_MAX(a, b) ((a) > (b) ? (a) : (b))
 
 /**
  * @brief Clamp a value between a minimum and maximum
  */
 #define MATH_CLAMP(value, min, max) (MATH_MIN(MATH_MAX(value, min), max))
 
 /**
  * @brief Check if two floating point values are equal
  */
 #define MATH_FLOAT_EQUAL(a, b) (fabsf((a) - (b)) < MATH_FLOAT_EPSILON)
 
 /**
  * @brief Convert a value from one range to another
  */
 #define MATH_MAP(x, in_min, in_max, out_min, out_max) \
     ((float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
 
 /**
  * @brief Linear interpolation between two values
  * 
  * @param a First value
  * @param b Second value
  * @param t Interpolation factor (0.0 - 1.0)
  * @return float Interpolated value
  */
 static inline float math_lerp(float a, float b, float t) {
     return a + t * (b - a);
 }
 
 /**
  * @brief Structure for a 2D point
  */
 typedef struct {
     float x;
     float y;
 } math_point_2d_t;
 
 /**
  * @brief Structure for a 3D point
  */
 typedef struct {
     float x;
     float y;
     float z;
 } math_point_3d_t;
 
 /**
  * @brief Structure for an OCV-SoC mapping point
  */
 typedef struct {
     float ocv;   // Open Circuit Voltage (V)
     float soc;   // State of Charge (%)
 } math_ocv_soc_point_t;
 
 /**
  * @brief Structure for a moving average filter
  */
 typedef struct {
     float *buffer;       // Buffer for samples
     uint16_t size;       // Size of the buffer
     uint16_t index;      // Current index in the buffer
     uint16_t count;      // Number of samples in the buffer
     float sum;           // Sum of all samples in the buffer
 } math_moving_avg_t;
 
 /**
  * @brief Structure for a Kalman filter
  */
 typedef struct {
     float value;         // Current estimated value
     float error;         // Current estimated error
     float q;             // Process noise
     float r;             // Measurement noise
     float k;             // Kalman gain
     bool initialized;    // Whether the filter is initialized
 } math_kalman_t;
 
 /**
  * @brief Initialize a moving average filter
  * 
  * @param filter Pointer to the filter structure
  * @param buffer Buffer for samples, allocated by the caller
  * @param size Size of the buffer
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t math_moving_avg_init(math_moving_avg_t *filter, float *buffer, uint16_t size);
 
 /**
  * @brief Add a sample to a moving average filter
  * 
  * @param filter Pointer to the filter structure
  * @param sample New sample value
  * @return float Updated average value
  */
 float math_moving_avg_update(math_moving_avg_t *filter, float sample);
 
 /**
  * @brief Reset a moving average filter
  * 
  * @param filter Pointer to the filter structure
  */
 void math_moving_avg_reset(math_moving_avg_t *filter);
 
 /**
  * @brief Get the current average value
  * 
  * @param filter Pointer to the filter structure
  * @return float Current average value
  */
 float math_moving_avg_get(const math_moving_avg_t *filter);
 
 /**
  * @brief Initialize a Kalman filter
  * 
  * @param filter Pointer to the filter structure
  * @param initial_value Initial estimated value
  * @param initial_error Initial estimated error
  * @param process_noise Process noise (Q)
  * @param measurement_noise Measurement noise (R)
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t math_kalman_init(math_kalman_t *filter, float initial_value, float initial_error, 
                           float process_noise, float measurement_noise);
 
 /**
  * @brief Update a Kalman filter with a new measurement
  * 
  * @param filter Pointer to the filter structure
  * @param measurement New measurement value
  * @return float Updated estimated value
  */
 float math_kalman_update(math_kalman_t *filter, float measurement);
 
 /**
  * @brief Get the current estimated value
  * 
  * @param filter Pointer to the filter structure
  * @return float Current estimated value
  */
 float math_kalman_get_value(const math_kalman_t *filter);
 
 /**
  * @brief Get the current estimated error
  * 
  * @param filter Pointer to the filter structure
  * @return float Current estimated error
  */
 float math_kalman_get_error(const math_kalman_t *filter);
 
 /**
  * @brief Perform linear interpolation on a lookup table
  * 
  * @param x_value The x value to interpolate
  * @param points Array of x,y points
  * @param num_points Number of points in the array
  * @param y_out Pointer to store the interpolated y value
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t math_interpolate(float x_value, const math_point_2d_t *points, 
                           uint16_t num_points, float *y_out);
 
 /**
  * @brief Perform OCV-SoC interpolation
  * 
  * @param ocv The OCV value (V)
  * @param curve The OCV-SoC curve
  * @param num_points Number of points in the curve
  * @param soc_out Pointer to store the interpolated SoC value
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t math_ocv_to_soc(float ocv, const math_ocv_soc_point_t *curve, 
                          uint16_t num_points, float *soc_out);
 
 /**
  * @brief Calculate the sum of an array
  * 
  * @param array Array of values
  * @param length Length of the array
  * @return float Sum of all elements
  */
 float math_sum(const float *array, uint16_t length);
 
 /**
  * @brief Calculate the average of an array
  * 
  * @param array Array of values
  * @param length Length of the array
  * @return float Average of all elements
  */
 float math_average(const float *array, uint16_t length);
 
 /**
  * @brief Calculate the standard deviation of an array
  * 
  * @param array Array of values
  * @param length Length of the array
  * @param avg Pre-calculated average (pass 0 to calculate internally)
  * @return float Standard deviation
  */
 float math_std_dev(const float *array, uint16_t length, float avg);
 
 /**
  * @brief Find the minimum value in an array
  * 
  * @param array Array of values
  * @param length Length of the array
  * @param index Optional pointer to store the index of the minimum value
  * @return float Minimum value
  */
 float math_min(const float *array, uint16_t length, uint16_t *index);
 
 /**
  * @brief Find the maximum value in an array
  * 
  * @param array Array of values
  * @param length Length of the array
  * @param index Optional pointer to store the index of the maximum value
  * @return float Maximum value
  */
 float math_max(const float *array, uint16_t length, uint16_t *index);
 
 /**
  * @brief Calculate the rate of change
  * 
  * @param current_value Current value
  * @param previous_value Previous value
  * @param time_delta Time difference in seconds
  * @return float Rate of change per second
  */
 float math_rate_of_change(float current_value, float previous_value, float time_delta);
 
 /**
  * @brief Calculate the weighted average of an array
  * 
  * @param values Array of values
  * @param weights Array of weights
  * @param length Length of the arrays
  * @return float Weighted average
  */
 float math_weighted_average(const float *values, const float *weights, uint16_t length);
 
 /**
  * @brief Calculate the power from voltage and current
  * 
  * @param voltage Voltage in volts
  * @param current Current in amperes
  * @return float Power in watts
  */
 static inline float math_calculate_power(float voltage, float current) {
     return voltage * current;
 }
 
 /**
  * @brief Calculate energy from power and time
  * 
  * @param power Power in watts
  * @param hours Time in hours
  * @return float Energy in watt-hours
  */
 static inline float math_calculate_energy(float power, float hours) {
     return power * hours;
 }
 
 /**
  * @brief Calculate the temperature-compensated capacity
  * 
  * @param nominal_capacity Nominal capacity at reference temperature
  * @param temperature Current temperature in °C
  * @param reference_temperature Reference temperature in °C (typically 25°C)
  * @param temp_coefficient Temperature coefficient (% per °C)
  * @return float Temperature-compensated capacity
  */
 float math_temperature_compensated_capacity(float nominal_capacity, float temperature, 
                                           float reference_temperature, float temp_coefficient);
 
 /**
  * @brief Calculate capacity degradation based on cycles
  * 
  * @param initial_capacity Initial capacity (100%)
  * @param cycle_count Number of charge/discharge cycles
  * @param cycle_coefficient Degradation coefficient per cycle
  * @return float Remaining capacity percentage
  */
 float math_cycle_degradation(float initial_capacity, uint32_t cycle_count, float cycle_coefficient);
 
 #endif /* MATH_UTILS_H */