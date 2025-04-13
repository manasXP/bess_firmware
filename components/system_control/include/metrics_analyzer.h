/**
 * @file metrics_analyzer.h
 * @brief Header file for BESS metrics analyzer component
 * 
 * This module provides functionality for analyzing operational metrics 
 * for the 100KW/200KWH Battery Energy Storage System (BESS).
 * It tracks, analyzes, and provides insights and trends for various
 * system metrics to support decision-making and optimization.
 *
 * @copyright (c) 2025 BESS Firmware Team
 */

 #ifndef METRICS_ANALYZER_H
 #define METRICS_ANALYZER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "esp_log.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Metric data types that can be analyzed
  */
 typedef enum {
     METRIC_TYPE_SOC,               /**< State of Charge */
     METRIC_TYPE_SOH,               /**< State of Health */
     METRIC_TYPE_VOLTAGE,           /**< System voltage */
     METRIC_TYPE_CURRENT,           /**< System current */
     METRIC_TYPE_POWER,             /**< Instantaneous power */
     METRIC_TYPE_TEMPERATURE,       /**< System temperature */
     METRIC_TYPE_ENERGY_IN,         /**< Energy input (charging) */
     METRIC_TYPE_ENERGY_OUT,        /**< Energy output (discharging) */
     METRIC_TYPE_EFFICIENCY,        /**< System efficiency */
     METRIC_TYPE_UPTIME,            /**< System uptime */
     METRIC_TYPE_CYCLES,            /**< Charge/discharge cycles */
     METRIC_TYPE_GRID_FREQUENCY,    /**< Grid frequency if connected */
     METRIC_TYPE_MAX                /**< Maximum metric type value (for array sizing) */
 } metrics_type_t;
 
 /**
  * @brief Time period for metric analysis
  */
 typedef enum {
     METRICS_PERIOD_REALTIME,       /**< Real-time data (last reading) */
     METRICS_PERIOD_MINUTE,         /**< Last minute average */
     METRICS_PERIOD_HOUR,           /**< Last hour average */
     METRICS_PERIOD_DAY,            /**< Last 24 hours average */
     METRICS_PERIOD_WEEK,           /**< Last 7 days average */
     METRICS_PERIOD_MONTH,          /**< Last 30 days average */
     METRICS_PERIOD_MAX             /**< Maximum period value (for array sizing) */
 } metrics_period_t;
 
 /**
  * @brief Statistical analysis types
  */
 typedef enum {
     METRICS_STAT_MIN,              /**< Minimum value */
     METRICS_STAT_MAX,              /**< Maximum value */
     METRICS_STAT_AVG,              /**< Average value */
     METRICS_STAT_MEDIAN,           /**< Median value */
     METRICS_STAT_STDDEV,           /**< Standard deviation */
     METRICS_STAT_TREND,            /**< Trend direction (positive/negative) */
     METRICS_STAT_RATE_OF_CHANGE,   /**< Rate of change */
     METRICS_STAT_MAX               /**< Maximum stat type value (for array sizing) */
 } metrics_stat_t;
 
 /**
  * @brief Trend direction for metric values
  */
 typedef enum {
     TREND_STABLE,                  /**< No significant change */
     TREND_INCREASING,              /**< Value is increasing */
     TREND_DECREASING,              /**< Value is decreasing */
     TREND_FLUCTUATING              /**< Value is fluctuating without clear direction */
 } metrics_trend_t;
 
 /**
  * @brief Metric threshold severity levels
  */
 typedef enum {
     THRESHOLD_NORMAL,              /**< Within normal operating range */
     THRESHOLD_WARNING,             /**< Approaching limits, action recommended */
     THRESHOLD_CRITICAL,            /**< Outside safe range, immediate action required */
     THRESHOLD_EMERGENCY            /**< Dangerous condition, emergency procedures needed */
 } threshold_severity_t;
 
 /**
  * @brief Configuration structure for metrics analyzer
  */
 typedef struct {
     uint32_t sampling_interval_ms;         /**< Metric sampling interval in milliseconds */
     uint32_t storage_capacity_per_metric;  /**< How many samples to store per metric */
     bool enable_cloud_upload;              /**< Whether to upload metrics to AWS CloudWatch */
     uint32_t cloud_upload_interval_s;      /**< How often to upload metrics to cloud (seconds) */
     char *metrics_log_path;                /**< Path to store metrics logs on SD card */
     uint8_t task_priority;                 /**< Priority for the metrics analyzer task */
     uint8_t core_id;                       /**< CPU core to run the analyzer task on */
 } metrics_analyzer_config_t;
 
 /**
  * @brief Metrics threshold configuration
  */
 typedef struct {
     metrics_type_t metric_type;           /**< Type of metric */
     float warning_low;                    /**< Lower warning threshold */
     float warning_high;                   /**< Upper warning threshold */
     float critical_low;                   /**< Lower critical threshold */
     float critical_high;                  /**< Upper critical threshold */
     float emergency_low;                  /**< Lower emergency threshold */
     float emergency_high;                 /**< Upper emergency threshold */
 } metrics_threshold_t;
 
 /**
  * @brief Statistical summary of a metric
  */
 typedef struct {
     float min;                           /**< Minimum value recorded */
     float max;                           /**< Maximum value recorded */
     float avg;                           /**< Average value */
     float median;                        /**< Median value */
     float stddev;                        /**< Standard deviation */
     metrics_trend_t trend;               /**< Current trend */
     float rate_of_change;                /**< Rate of change (units per hour) */
     uint32_t sample_count;               /**< Number of samples this summary is based on */
     int64_t timestamp;                   /**< Timestamp of this summary (microseconds since boot) */
 } metrics_statistics_t;
 
 /**
  * @brief Callback function for metric threshold crossing events
  * 
  * @param metric_type The type of metric that crossed a threshold
  * @param severity The severity level of the threshold crossing
  * @param value The current value that triggered the event
  * @param threshold The threshold value that was crossed
  * @param user_data User-provided context pointer
  */
 typedef void (*metrics_threshold_callback_t)(
     metrics_type_t metric_type,
     threshold_severity_t severity,
     float value,
     float threshold,
     void *user_data
 );
 
 /**
  * @brief Initialize the metrics analyzer component
  * 
  * @param config Configuration parameters for the metrics analyzer
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_init(const metrics_analyzer_config_t *config);
 
 /**
  * @brief Deinitialize the metrics analyzer component
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_deinit(void);
 
 /**
  * @brief Start the metrics analyzer
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_start(void);
 
 /**
  * @brief Stop the metrics analyzer
  * 
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_stop(void);
 
 /**
  * @brief Add a new metric data point
  * 
  * @param type The type of metric
  * @param value The metric value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_add_data_point(metrics_type_t type, float value);
 
 /**
  * @brief Add a batch of metric data points at once (more efficient)
  * 
  * @param types Array of metric types
  * @param values Array of metric values
  * @param count Number of metrics in the arrays
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_add_data_points(const metrics_type_t *types, 
                                           const float *values, 
                                           uint8_t count);
 
 /**
  * @brief Get the latest value for a specific metric
  * 
  * @param type The type of metric
  * @param value Pointer to store the metric value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_get_latest(metrics_type_t type, float *value);
 
 /**
  * @brief Get statistical analysis for a metric over a specified time period
  * 
  * @param type The type of metric
  * @param period The time period for analysis
  * @param stats Pointer to store the statistics
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_get_statistics(metrics_type_t type, 
                                          metrics_period_t period,
                                          metrics_statistics_t *stats);
 
 /**
  * @brief Get specific statistic for a metric
  * 
  * @param type The type of metric
  * @param period The time period for analysis
  * @param stat_type The specific statistic requested
  * @param value Pointer to store the statistic value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_get_single_statistic(metrics_type_t type,
                                                metrics_period_t period,
                                                metrics_stat_t stat_type,
                                                float *value);
 
 /**
  * @brief Configure threshold settings for a metric
  * 
  * @param threshold Threshold configuration structure
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_set_thresholds(const metrics_threshold_t *threshold);
 
 /**
  * @brief Get the current threshold settings for a metric
  * 
  * @param type The type of metric
  * @param threshold Pointer to store the threshold configuration
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_get_thresholds(metrics_type_t type, 
                                          metrics_threshold_t *threshold);
 
 /**
  * @brief Register a callback for threshold crossing events
  * 
  * @param type The type of metric to monitor (or METRIC_TYPE_MAX for all metrics)
  * @param severity Minimum severity level to trigger callback
  * @param callback Function to call when threshold is crossed
  * @param user_data User context pointer to pass to the callback
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_register_threshold_callback(
     metrics_type_t type,
     threshold_severity_t severity,
     metrics_threshold_callback_t callback,
     void *user_data
 );
 
 /**
  * @brief Unregister a threshold crossing callback
  * 
  * @param callback The callback function to unregister
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_unregister_threshold_callback(
     metrics_threshold_callback_t callback
 );
 
 /**
  * @brief Export metrics to a JSON file on the SD card
  * 
  * @param type The type of metric to export (or METRIC_TYPE_MAX for all metrics)
  * @param period The time period to export
  * @param filename The filename to export to
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_export_to_json(metrics_type_t type,
                                          metrics_period_t period,
                                          const char *filename);
 
 /**
  * @brief Force an immediate upload of metrics to AWS CloudWatch
  * 
  * @param type The type of metric to upload (or METRIC_TYPE_MAX for all metrics)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_force_cloud_upload(metrics_type_t type);
 
 /**
  * @brief Clear stored metric history
  * 
  * @param type The type of metric to clear (or METRIC_TYPE_MAX for all metrics)
  * @param period The time period to clear (or METRICS_PERIOD_MAX for all periods)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_clear_history(metrics_type_t type, 
                                         metrics_period_t period);
 
 /**
  * @brief Get system performance metrics report
  * 
  * @param output_buffer Buffer to write the report to
  * @param buffer_size Size of the output buffer
  * @param written_size Pointer to store the number of bytes written
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_generate_performance_report(char *output_buffer,
                                                      size_t buffer_size,
                                                      size_t *written_size);
 
 /**
  * @brief Get current trend analysis for a specific metric
  * 
  * @param type The type of metric
  * @param trend Pointer to store the trend value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_get_trend(metrics_type_t type, metrics_trend_t *trend);
 
 /**
  * @brief Check if a metric is currently in an alarm state
  * 
  * @param type The type of metric
  * @param severity Pointer to store the current severity level
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t metrics_analyzer_check_alarm_state(metrics_type_t type, 
                                             threshold_severity_t *severity);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* METRICS_ANALYZER_H */