/**
 * @file time_utils.h
 * @brief Time-related utilities for the BESS firmware
 * 
 * This file contains time-related utilities, functions, and macros
 * used throughout the BESS (Battery Energy Storage System) firmware.
 * 
 * @copyright Copyright (c) 2025
 */

 #ifndef TIME_UTILS_H
 #define TIME_UTILS_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include <time.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_timer.h"
 #include "esp_err.h"
 
 /**
  * @brief Structure for representing a timestamp
  */
 typedef struct {
     uint16_t year;       // Year (e.g., 2025)
     uint8_t month;       // Month (1-12)
     uint8_t day;         // Day (1-31)
     uint8_t hour;        // Hour (0-23)
     uint8_t minute;      // Minute (0-59)
     uint8_t second;      // Second (0-59)
     uint16_t millisecond; // Millisecond (0-999)
 } timestamp_t;
 
 /**
  * @brief Structure for timing statistics
  */
 typedef struct {
     uint64_t min_time;   // Minimum execution time (microseconds)
     uint64_t max_time;   // Maximum execution time (microseconds)
     uint64_t total_time; // Total execution time (microseconds)
     uint32_t count;      // Number of executions
 } timing_stats_t;
 
 /**
  * @brief Periods for recurring tasks
  */
 typedef enum {
     TIME_PERIOD_100MS = 100,
     TIME_PERIOD_500MS = 500,
     TIME_PERIOD_1S = 1000,
     TIME_PERIOD_5S = 5000,
     TIME_PERIOD_10S = 10000,
     TIME_PERIOD_30S = 30000,
     TIME_PERIOD_1M = 60000,
     TIME_PERIOD_5M = 300000
 } time_period_t;
 
 /**
  * @brief Get current timestamp from system time
  * 
  * @param timestamp Pointer to store the current timestamp
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t time_get_timestamp(timestamp_t *timestamp);
 
 /**
  * @brief Convert a timestamp to Unix time (seconds since epoch)
  * 
  * @param timestamp The timestamp to convert
  * @return time_t Unix time
  */
 time_t time_timestamp_to_unix(const timestamp_t *timestamp);
 
 /**
  * @brief Convert Unix time to a timestamp
  * 
  * @param unix_time Unix time
  * @param timestamp Pointer to store the converted timestamp
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t time_unix_to_timestamp(time_t unix_time, timestamp_t *timestamp);
 
 /**
  * @brief Get the system uptime in seconds
  * 
  * @return uint32_t System uptime in seconds
  */
 uint32_t time_get_uptime_seconds(void);
 
 /**
  * @brief Get the system uptime in milliseconds
  * 
  * @return uint64_t System uptime in milliseconds
  */
 uint64_t time_get_uptime_ms(void);
 
 /**
  * @brief Format a timestamp as a string
  * 
  * @param timestamp The timestamp to format
  * @param format The format string (strftime-compatible)
  * @param buffer Output buffer
  * @param buffer_size Size of the output buffer
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t time_format_timestamp(const timestamp_t *timestamp, const char *format, 
                               char *buffer, size_t buffer_size);
 
 /**
  * @brief Parse a string into a timestamp
  * 
  * @param str The string to parse
  * @param format The format string (strftime-compatible)
  * @param timestamp Pointer to store the parsed timestamp
  * @return esp_err_t ESP_OK on success, or an error code
  */
 esp_err_t time_parse_timestamp(const char *str, const char *format, timestamp_t *timestamp);
 
 /**
  * @brief Compare two timestamps
  * 
  * @param t1 First timestamp
  * @param t2 Second timestamp
  * @return int Negative if t1 < t2, 0 if t1 == t2, positive if t1 > t2
  */
 int time_compare_timestamps(const timestamp_t *t1, const timestamp_t *t2);
 
 /**
  * @brief Calculate time difference between two timestamps in seconds
  * 
  * @param t1 First timestamp
  * @param t2 Second timestamp
  * @return double Time difference in seconds (t2 - t1)
  */
 double time_difference_seconds(const timestamp_t *t1, const timestamp_t *t2);
 
 /**
  * @brief High-resolution timer for performance measurements
  */
 typedef uint64_t hr_timer_t;
 
 /**
  * @brief Start a high-resolution timer
  * 
  * @return hr_timer_t Timer handle
  */
 static inline hr_timer_t time_hr_timer_start(void) {
     return esp_timer_get_time();
 }
 
 /**
  * @brief Stop a high-resolution timer and get elapsed time in microseconds
  * 
  * @param start Timer handle from time_hr_timer_start
  * @return uint64_t Elapsed time in microseconds
  */
 static inline uint64_t time_hr_timer_stop(hr_timer_t start) {
     return esp_timer_get_time() - start;
 }
 
 /**
  * @brief Initialize timing statistics
  * 
  * @param stats Pointer to timing statistics structure
  */
 void time_stats_init(timing_stats_t *stats);
 
 /**
  * @brief Update timing statistics with a new measurement
  * 
  * @param stats Pointer to timing statistics structure
  * @param time_us Execution time in microseconds
  */
 void time_stats_update(timing_stats_t *stats, uint64_t time_us);
 
 /**
  * @brief Reset timing statistics
  * 
  * @param stats Pointer to timing statistics structure
  */
 void time_stats_reset(timing_stats_t *stats);
 
 /**
  * @brief Get average execution time from timing statistics
  * 
  * @param stats Pointer to timing statistics structure
  * @return double Average execution time in microseconds
  */
 double time_stats_get_average(const timing_stats_t *stats);
 
 /**
  * @brief Task timing utility for FreeRTOS tasks
  * 
  * This macro measures the execution time of a block of code
  * and updates timing statistics.
  * 
  * Usage:
  * TIME_MEASURE_TASK_EXECUTION(stats) {
  *     // Code to measure