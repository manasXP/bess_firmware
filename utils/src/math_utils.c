/**
 * @file math_utils.c
 * @brief Implementation of mathematical utilities for the BESS firmware
 * 
 * This file implements the mathematical utilities, functions, and algorithms
 * defined in math_utils.h used throughout the BESS firmware.
 * 
 * @copyright Copyright (c) 2025
 */

 #include "math_utils.h"
 #include <string.h>
 #include <stdlib.h>
 #include <stdio.h>
 
 // Implementation of moving average filter functions
 
 esp_err_t math_moving_avg_init(math_moving_avg_t *filter, float *buffer, uint16_t size) {
     if (filter == NULL || buffer == NULL || size == 0) {
         return ESP_ERR_INVALID_ARG;
     }
 
     filter->buffer = buffer;
     filter->size = size;
     filter->index = 0;
     filter->count = 0;
     filter->sum = 0.0f;
 
     // Initialize buffer to zeros
     memset(buffer, 0, sizeof(float) * size);
 
     return ESP_OK;
 }
 
 float math_moving_avg_update(math_moving_avg_t *filter, float sample) {
     if (filter == NULL || filter->buffer == NULL) {
         return 0.0f;
     }
 
     // If buffer is full, subtract the oldest sample
     if (filter->count == filter->size) {
         filter->sum -= filter->buffer[filter->index];
     } else {
         // Increment count until buffer is full
         filter->count++;
     }
 
     // Add new sample to the buffer
     filter->buffer[filter->index] = sample;
     filter->sum += sample;
 
     // Update index for next time
     filter->index = (filter->index + 1) % filter->size;
 
     // Return the current average
     return filter->sum / filter->count;
 }
 
 void math_moving_avg_reset(math_moving_avg_t *filter) {
     if (filter == NULL || filter->buffer == NULL) {
         return;
     }
 
     filter->index = 0;
     filter->count = 0;
     filter->sum = 0.0f;
 
     // Clear buffer
     memset(filter->buffer, 0, sizeof(float) * filter->size);
 }
 
 float math_moving_avg_get(const math_moving_avg_t *filter) {
     if (filter == NULL || filter->buffer == NULL || filter->count == 0) {
         return 0.0f;
     }
 
     return filter->sum / filter->count;
 }
 
 // Implementation of Kalman filter functions
 
 esp_err_t math_kalman_init(math_kalman_t *filter, float initial_value, float initial_error, 
                            float process_noise, float measurement_noise) {
     if (filter == NULL || process_noise < 0.0f || measurement_noise < 0.0f) {
         return ESP_ERR_INVALID_ARG;
     }
 
     filter->value = initial_value;
     filter->error = initial_error;
     filter->q = process_noise;
     filter->r = measurement_noise;
     filter->k = 0.0f;
     filter->initialized = true;
 
     return ESP_OK;
 }
 
 float math_kalman_update(math_kalman_t *filter, float measurement) {
     if (filter == NULL || !filter->initialized) {
         return measurement;
     }
 
     // Prediction update - Project the state ahead
     // Note: For a simple 1D Kalman filter, the state transition is just identity
     // so we don't change the state value in prediction
 
     // Update the error covariance
     filter->error = filter->error + filter->q;
 
     // Measurement update - Compute Kalman gain
     filter->k = filter->error / (filter->error + filter->r);
 
     // Update estimate with measurement
     filter->value = filter->value + filter->k * (measurement - filter->value);
 
     // Update error covariance
     filter->error = (1.0f - filter->k) * filter->error;
 
     return filter->value;
 }
 
 float math_kalman_get_value(const math_kalman_t *filter) {
     if (filter == NULL || !filter->initialized) {
         return 0.0f;
     }
 
     return filter->value;
 }
 
 float math_kalman_get_error(const math_kalman_t *filter) {
     if (filter == NULL || !filter->initialized) {
         return 0.0f;
     }
 
     return filter->error;
 }
 
 // Implementation of interpolation functions
 
 esp_err_t math_interpolate(float x_value, const math_point_2d_t *points, 
                            uint16_t num_points, float *y_out) {
     if (points == NULL || num_points < 2 || y_out == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Check if x_value is below the lowest point
     if (x_value <= points[0].x) {
         *y_out = points[0].y;
         return ESP_OK;
     }
 
     // Check if x_value is above the highest point
     if (x_value >= points[num_points - 1].x) {
         *y_out = points[num_points - 1].y;
         return ESP_OK;
     }
 
     // Find the two points for interpolation
     uint16_t i;
     for (i = 0; i < num_points - 1; i++) {
         if (x_value >= points[i].x && x_value < points[i+1].x) {
             break;
         }
     }
 
     // Calculate interpolation factor
     float t = (x_value - points[i].x) / (points[i+1].x - points[i].x);
 
     // Linear interpolation
     *y_out = math_lerp(points[i].y, points[i+1].y, t);
 
     return ESP_OK;
 }
 
 esp_err_t math_ocv_to_soc(float ocv, const math_ocv_soc_point_t *curve, 
                           uint16_t num_points, float *soc_out) {
     if (curve == NULL || num_points < 2 || soc_out == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Check if OCV is below the lowest point
     if (ocv <= curve[0].ocv) {
         *soc_out = curve[0].soc;
         return ESP_OK;
     }
 
     // Check if OCV is above the highest point
     if (ocv >= curve[num_points - 1].ocv) {
         *soc_out = curve[num_points - 1].soc;
         return ESP_OK;
     }
 
     // Find the two points for interpolation
     uint16_t i;
     for (i = 0; i < num_points - 1; i++) {
         if (ocv >= curve[i].ocv && ocv < curve[i+1].ocv) {
             break;
         }
     }
 
     // Calculate interpolation factor
     float t = (ocv - curve[i].ocv) / (curve[i+1].ocv - curve[i].ocv);
 
     // Linear interpolation
     *soc_out = math_lerp(curve[i].soc, curve[i+1].soc, t);
 
     return ESP_OK;
 }
 
 // Implementation of statistical functions
 
 float math_sum(const float *array, uint16_t length) {
     if (array == NULL || length == 0) {
         return 0.0f;
     }
 
     float sum = 0.0f;
     for (uint16_t i = 0; i < length; i++) {
         sum += array[i];
     }
 
     return sum;
 }
 
 float math_average(const float *array, uint16_t length) {
     if (array == NULL || length == 0) {
         return 0.0f;
     }
 
     return math_sum(array, length) / length;
 }
 
 float math_std_dev(const float *array, uint16_t length, float avg) {
     if (array == NULL || length <= 1) {
         return 0.0f;
     }
 
     // Calculate average if not provided
     if (MATH_FLOAT_EQUAL(avg, 0.0f)) {
         avg = math_average(array, length);
     }
 
     // Calculate sum of squared differences
     float sum_sq_diff = 0.0f;
     for (uint16_t i = 0; i < length; i++) {
         float diff = array[i] - avg;
         sum_sq_diff += diff * diff;
     }
 
     // Calculate variance and standard deviation
     float variance = sum_sq_diff / (length - 1); // Use n-1 for sample standard deviation
     return sqrtf(variance);
 }
 
 float math_min(const float *array, uint16_t length, uint16_t *index) {
     if (array == NULL || length == 0) {
         if (index != NULL) {
             *index = 0;
         }
         return 0.0f;
     }
 
     float min_val = array[0];
     uint16_t min_idx = 0;
 
     for (uint16_t i = 1; i < length; i++) {
         if (array[i] < min_val) {
             min_val = array[i];
             min_idx = i;
         }
     }
 
     if (index != NULL) {
         *index = min_idx;
     }
 
     return min_val;
 }
 
 float math_max(const float *array, uint16_t length, uint16_t *index) {
     if (array == NULL || length == 0) {
         if (index != NULL) {
             *index = 0;
         }
         return 0.0f;
     }
 
     float max_val = array[0];
     uint16_t max_idx = 0;
 
     for (uint16_t i = 1; i < length; i++) {
         if (array[i] > max_val) {
             max_val = array[i];
             max_idx = i;
         }
     }
 
     if (index != NULL) {
         *index = max_idx;
     }
 
     return max_val;
 }
 
 float math_rate_of_change(float current_value, float previous_value, float time_delta) {
     if (time_delta <= 0.0f) {
         return 0.0f;
     }
 
     return (current_value - previous_value) / time_delta;
 }
 
 float math_weighted_average(const float *values, const float *weights, uint16_t length) {
     if (values == NULL || weights == NULL || length == 0) {
         return 0.0f;
     }
 
     float sum_weighted = 0.0f;
     float sum_weights = 0.0f;
 
     for (uint16_t i = 0; i < length; i++) {
         sum_weighted += values[i] * weights[i];
         sum_weights += weights[i];
     }
 
     if (sum_weights > 0.0f) {
         return sum_weighted / sum_weights;
     } else {
         return 0.0f;
     }
 }
 
 float math_temperature_compensated_capacity(float nominal_capacity, float temperature, 
                                            float reference_temperature, float temp_coefficient) {
     // Calculate temperature difference
     float temp_diff = temperature - reference_temperature;
     
     // Calculate compensation factor
     float compensation_factor = 1.0f + (temp_coefficient * temp_diff / 100.0f);
     
     // Apply compensation factor to nominal capacity
     return nominal_capacity * compensation_factor;
 }
 
 float math_cycle_degradation(float initial_capacity, uint32_t cycle_count, float cycle_coefficient) {
     // Calculate remaining capacity percentage based on cycle count
     float degradation = 1.0f - (cycle_coefficient * cycle_count / 100.0f);
     
     // Ensure degradation doesn't go below 0
     degradation = MATH_MAX(degradation, 0.0f);
     
     // Apply degradation factor to initial capacity
     return initial_capacity * degradation;
 }