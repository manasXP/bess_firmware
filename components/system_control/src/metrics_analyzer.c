/**
 * @file metrics_analyzer.c
 * @brief Implementation of the BESS metrics analyzer component
 * 
 * This module provides functionality for analyzing operational metrics 
 * for the 100KW/200KWH Battery Energy Storage System (BESS).
 * It tracks, analyzes, and provides insights and trends for various
 * system metrics to support decision-making and optimization.
 *
 * @copyright (c) 2025 BESS Firmware Team
 */

 #include <string.h>
 #include <math.h>
 #include <stdlib.h>
 #include <float.h>
 #include "metrics_analyzer.h"
 #include "sdmmc_cmd.h"
 #include "esp_system.h"
 #include "cJSON.h"
 #include "esp_aws_iot.h"
 #include "esp_timer.h"
 
 #define TAG "METRICS_ANALYZER"
 
 /**
  * @brief Maximum number of threshold callbacks that can be registered
  */
 #define MAX_THRESHOLD_CALLBACKS 10
 
 /**
  * @brief Structure to store a single data point
  */
 typedef struct {
     float value;
     int64_t timestamp;
 } metric_data_point_t;
 
 /**
  * @brief Structure to store data points for a specific metric
  */
 typedef struct {
     metric_data_point_t *data_points;
     uint32_t capacity;
     uint32_t count;
     uint32_t head;
     SemaphoreHandle_t mutex;
 } metric_storage_t;
 
 /**
  * @brief Structure for threshold callback registration
  */
 typedef struct {
     metrics_type_t type;
     threshold_severity_t severity;
     metrics_threshold_callback_t callback;
     void *user_data;
     bool in_use;
 } threshold_callback_entry_t;
 
 /**
  * @brief Global state for the metrics analyzer
  */
 typedef struct {
     metrics_analyzer_config_t config;
     metric_storage_t storages[METRIC_TYPE_MAX];
     metrics_threshold_t thresholds[METRIC_TYPE_MAX];
     threshold_callback_entry_t callbacks[MAX_THRESHOLD_CALLBACKS];
     TaskHandle_t task_handle;
     SemaphoreHandle_t global_mutex;
     EventGroupHandle_t event_group;
     bool initialized;
     bool running;
 } metrics_analyzer_state_t;
 
 /* Event bits for the analyzer task */
 #define METRICS_EVENT_STOP       (1 << 0)
 #define METRICS_EVENT_UPLOAD     (1 << 1)
 
 /* Static variables */
 static metrics_analyzer_state_t s_state = {0};
 
 /* Forward declarations for internal functions */
 static void metrics_analyzer_task(void *arg);
 static esp_err_t process_metrics_for_thresholds(metrics_type_t type, float value);
 static esp_err_t calculate_statistics(metrics_type_t type, metrics_period_t period, metrics_statistics_t *stats);
 static int compare_float(const void *a, const void *b);
 static metrics_trend_t calculate_trend(const metric_data_point_t *data, uint32_t count);
 static esp_err_t upload_metrics_to_cloud(metrics_type_t type);
 static esp_err_t save_metrics_to_sd(metrics_type_t type, metrics_period_t period);
 static float get_rate_of_change(const metric_data_point_t *data, uint32_t count);
 static uint32_t get_data_points_for_period(metrics_type_t type, metrics_period_t period, metric_data_point_t **data_out);
 
 /**
  * @brief Initialize the metrics analyzer component
  */
 esp_err_t metrics_analyzer_init(const metrics_analyzer_config_t *config) {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     if (s_state.initialized) {
         ESP_LOGW(TAG, "Metrics analyzer already initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     /* Initialize state */
     memset(&s_state, 0, sizeof(metrics_analyzer_state_t));
     memcpy(&s_state.config, config, sizeof(metrics_analyzer_config_t));
 
     /* Create global mutex */
     s_state.global_mutex = xSemaphoreCreateMutex();
     if (s_state.global_mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create global mutex");
         return ESP_ERR_NO_MEM;
     }
 
     /* Create event group */
     s_state.event_group = xEventGroupCreate();
     if (s_state.event_group == NULL) {
         vSemaphoreDelete(s_state.global_mutex);
         ESP_LOGE(TAG, "Failed to create event group");
         return ESP_ERR_NO_MEM;
     }
 
     /* Initialize storage for each metric type */
     for (int i = 0; i < METRIC_TYPE_MAX; i++) {
         s_state.storages[i].data_points = calloc(config->storage_capacity_per_metric, sizeof(metric_data_point_t));
         if (s_state.storages[i].data_points == NULL) {
             ESP_LOGE(TAG, "Failed to allocate memory for metric type %d", i);
             metrics_analyzer_deinit();
             return ESP_ERR_NO_MEM;
         }
 
         s_state.storages[i].capacity = config->storage_capacity_per_metric;
         s_state.storages[i].count = 0;
         s_state.storages[i].head = 0;
         s_state.storages[i].mutex = xSemaphoreCreateMutex();
         if (s_state.storages[i].mutex == NULL) {
             ESP_LOGE(TAG, "Failed to create mutex for metric type %d", i);
             metrics_analyzer_deinit();
             return ESP_ERR_NO_MEM;
         }
     }
 
     /* Initialize default thresholds */
     for (int i = 0; i < METRIC_TYPE_MAX; i++) {
         s_state.thresholds[i].metric_type = i;
         
         /* Set default thresholds based on metric type - these are just examples */
         switch (i) {
             case METRIC_TYPE_SOC:
                 s_state.thresholds[i].warning_low = 20.0f;
                 s_state.thresholds[i].warning_high = 90.0f;
                 s_state.thresholds[i].critical_low = 10.0f;
                 s_state.thresholds[i].critical_high = 95.0f;
                 s_state.thresholds[i].emergency_low = 5.0f;
                 s_state.thresholds[i].emergency_high = 100.0f;
                 break;
                 
             case METRIC_TYPE_TEMPERATURE:
                 s_state.thresholds[i].warning_low = 5.0f;
                 s_state.thresholds[i].warning_high = 40.0f;
                 s_state.thresholds[i].critical_low = 0.0f;
                 s_state.thresholds[i].critical_high = 50.0f;
                 s_state.thresholds[i].emergency_low = -10.0f;
                 s_state.thresholds[i].emergency_high = 60.0f;
                 break;
                 
             default:
                 /* Initialize with wide range for other metrics */
                 s_state.thresholds[i].warning_low = -1000.0f;
                 s_state.thresholds[i].warning_high = 1000.0f;
                 s_state.thresholds[i].critical_low = -2000.0f;
                 s_state.thresholds[i].critical_high = 2000.0f;
                 s_state.thresholds[i].emergency_low = -5000.0f;
                 s_state.thresholds[i].emergency_high = 5000.0f;
                 break;
         }
     }
 
     /* Initialize callback registry */
     for (int i = 0; i < MAX_THRESHOLD_CALLBACKS; i++) {
         s_state.callbacks[i].in_use = false;
     }
 
     s_state.initialized = true;
     ESP_LOGI(TAG, "Metrics analyzer initialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Deinitialize the metrics analyzer component
  */
 esp_err_t metrics_analyzer_deinit(void) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     /* Stop the task if running */
     if (s_state.running) {
         metrics_analyzer_stop();
     }
 
     /* Free allocated resources */
     for (int i = 0; i < METRIC_TYPE_MAX; i++) {
         if (s_state.storages[i].data_points != NULL) {
             free(s_state.storages[i].data_points);
             s_state.storages[i].data_points = NULL;
         }
         
         if (s_state.storages[i].mutex != NULL) {
             vSemaphoreDelete(s_state.storages[i].mutex);
             s_state.storages[i].mutex = NULL;
         }
     }
 
     if (s_state.global_mutex != NULL) {
         vSemaphoreDelete(s_state.global_mutex);
         s_state.global_mutex = NULL;
     }
 
     if (s_state.event_group != NULL) {
         vEventGroupDelete(s_state.event_group);
         s_state.event_group = NULL;
     }
 
     s_state.initialized = false;
     ESP_LOGI(TAG, "Metrics analyzer deinitialized");
     return ESP_OK;
 }
 
 /**
  * @brief Main task for metrics analyzer
  */
 static void metrics_analyzer_task(void *arg) {
     ESP_LOGI(TAG, "Metrics analyzer task started");
     TickType_t last_upload_time = xTaskGetTickCount();
     
     while (1) {
         /* Check if we should stop */
         EventBits_t bits = xEventGroupWaitBits(
             s_state.event_group,
             METRICS_EVENT_STOP | METRICS_EVENT_UPLOAD,
             pdTRUE,  /* Clear on exit */
             pdFALSE, /* Don't wait for all bits */
             s_state.config.sampling_interval_ms / portTICK_PERIOD_MS
         );
         
         if ((bits & METRICS_EVENT_STOP) != 0) {
             break;
         }
         
         /* Check if it's time to upload metrics to cloud */
         if (s_state.config.enable_cloud_upload) {
             TickType_t current_time = xTaskGetTickCount();
             if (((bits & METRICS_EVENT_UPLOAD) != 0) || 
                 (current_time - last_upload_time >= (s_state.config.cloud_upload_interval_s * 1000 / portTICK_PERIOD_MS))) {
                 
                 upload_metrics_to_cloud(METRIC_TYPE_MAX);
                 last_upload_time = current_time;
             }
         }
         
         /* Periodically save metrics to SD card */
         static uint32_t sd_save_counter = 0;
         sd_save_counter++;
         if (sd_save_counter >= 60) { /* Save to SD card every ~minute */
             sd_save_counter = 0;
             save_metrics_to_sd(METRIC_TYPE_MAX, METRICS_PERIOD_REALTIME);
         }
     }
     
     ESP_LOGI(TAG, "Metrics analyzer task stopped");
     s_state.task_handle = NULL;
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Start the metrics analyzer
  */
 esp_err_t metrics_analyzer_start(void) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (s_state.running) {
         ESP_LOGW(TAG, "Metrics analyzer already running");
         return ESP_OK;
     }
     
     /* Clear event bits */
     xEventGroupClearBits(s_state.event_group, METRICS_EVENT_STOP | METRICS_EVENT_UPLOAD);
     
     /* Create task */
     BaseType_t res = xTaskCreatePinnedToCore(
         metrics_analyzer_task,
         "metrics_analyzer",
         4096,
         NULL,
         s_state.config.task_priority,
         &s_state.task_handle,
         s_state.config.core_id
     );
     
     if (res != pdPASS) {
         ESP_LOGE(TAG, "Failed to create metrics analyzer task");
         return ESP_FAIL;
     }
     
     s_state.running = true;
     ESP_LOGI(TAG, "Metrics analyzer started");
     return ESP_OK;
 }
 
 /**
  * @brief Stop the metrics analyzer
  */
 esp_err_t metrics_analyzer_stop(void) {
     if (!s_state.initialized || !s_state.running) {
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Signal the task to stop */
     xEventGroupSetBits(s_state.event_group, METRICS_EVENT_STOP);
     
     /* Wait for task to finish (with timeout) */
     for (int i = 0; i < 10; i++) {
         if (s_state.task_handle == NULL) {
             break;
         }
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     
     /* Force delete if still running */
     if (s_state.task_handle != NULL) {
         vTaskDelete(s_state.task_handle);
         s_state.task_handle = NULL;
         ESP_LOGW(TAG, "Had to force delete metrics analyzer task");
     }
     
     s_state.running = false;
     ESP_LOGI(TAG, "Metrics analyzer stopped");
     return ESP_OK;
 }
 
 /**
  * @brief Add a new metric data point
  */
 esp_err_t metrics_analyzer_add_data_point(metrics_type_t type, float value) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (type >= METRIC_TYPE_MAX) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Lock the specific metric storage */
     if (xSemaphoreTake(s_state.storages[type].mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take mutex for metric type %d", type);
         return ESP_ERR_TIMEOUT;
     }
     
     /* Add data point to circular buffer */
     uint32_t index = s_state.storages[type].head;
     s_state.storages[type].data_points[index].value = value;
     s_state.storages[type].data_points[index].timestamp = esp_timer_get_time();
     
     /* Update head and count */
     s_state.storages[type].head = (s_state.storages[type].head + 1) % s_state.storages[type].capacity;
     if (s_state.storages[type].count < s_state.storages[type].capacity) {
         s_state.storages[type].count++;
     }
     
     xSemaphoreGive(s_state.storages[type].mutex);
     
     /* Process thresholds */
     return process_metrics_for_thresholds(type, value);
 }
 
 /**
  * @brief Add a batch of metric data points at once
  */
 esp_err_t metrics_analyzer_add_data_points(const metrics_type_t *types, 
                                           const float *values, 
                                           uint8_t count) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (types == NULL || values == NULL || count == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     esp_err_t result = ESP_OK;
     int64_t now = esp_timer_get_time();
     
     /* Process each metric */
     for (uint8_t i = 0; i < count; i++) {
         metrics_type_t type = types[i];
         float value = values[i];
         
         if (type >= METRIC_TYPE_MAX) {
             ESP_LOGW(TAG, "Invalid metric type: %d", type);
             result = ESP_ERR_INVALID_ARG;
             continue;
         }
         
         /* Lock the specific metric storage */
         if (xSemaphoreTake(s_state.storages[type].mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
             ESP_LOGW(TAG, "Failed to take mutex for metric type %d", type);
             result = ESP_ERR_TIMEOUT;
             continue;
         }
         
         /* Add data point to circular buffer */
         uint32_t index = s_state.storages[type].head;
         s_state.storages[type].data_points[index].value = value;
         s_state.storages[type].data_points[index].timestamp = now;
         
         /* Update head and count */
         s_state.storages[type].head = (s_state.storages[type].head + 1) % s_state.storages[type].capacity;
         if (s_state.storages[type].count < s_state.storages[type].capacity) {
             s_state.storages[type].count++;
         }
         
         xSemaphoreGive(s_state.storages[type].mutex);
         
         /* Process thresholds */
         esp_err_t threshold_result = process_metrics_for_thresholds(type, value);
         if (threshold_result != ESP_OK && result == ESP_OK) {
             result = threshold_result;
         }
     }
     
     return result;
 }
 
 /**
  * @brief Get the latest value for a specific metric
  */
 esp_err_t metrics_analyzer_get_latest(metrics_type_t type, float *value) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (type >= METRIC_TYPE_MAX || value == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Lock the specific metric storage */
     if (xSemaphoreTake(s_state.storages[type].mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take mutex for metric type %d", type);
         return ESP_ERR_TIMEOUT;
     }
     
     if (s_state.storages[type].count == 0) {
         xSemaphoreGive(s_state.storages[type].mutex);
         return ESP_ERR_NOT_FOUND;
     }
     
     /* Get the most recent value */
     uint32_t latest_index = (s_state.storages[type].head + s_state.storages[type].capacity - 1) % s_state.storages[type].capacity;
     *value = s_state.storages[type].data_points[latest_index].value;
     
     xSemaphoreGive(s_state.storages[type].mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Function to compare two floats for qsort
  */
 static int compare_float(const void *a, const void *b) {
     float fa = *(const float*)a;
     float fb = *(const float*)b;
     return (fa > fb) - (fa < fb);
 }
 
 /**
  * @brief Get statistical analysis for a metric over a specified time period
  */
 esp_err_t metrics_analyzer_get_statistics(metrics_type_t type, 
                                          metrics_period_t period,
                                          metrics_statistics_t *stats) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (type >= METRIC_TYPE_MAX || period >= METRICS_PERIOD_MAX || stats == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     return calculate_statistics(type, period, stats);
 }
 
 /**
  * @brief Calculate statistics for the given metric and period
  */
 static esp_err_t calculate_statistics(metrics_type_t type, metrics_period_t period, metrics_statistics_t *stats) {
     metric_data_point_t *data = NULL;
     uint32_t count = get_data_points_for_period(type, period, &data);
     
     if (count == 0 || data == NULL) {
         return ESP_ERR_NOT_FOUND;
     }
     
     /* Calculate min, max, avg */
     float min = FLT_MAX;
     float max = -FLT_MAX;
     float sum = 0.0f;
     float *sorted_values = malloc(count * sizeof(float));
     
     if (sorted_values == NULL) {
         free(data);
         return ESP_ERR_NO_MEM;
     }
     
     for (uint32_t i = 0; i < count; i++) {
         float val = data[i].value;
         
         if (val < min) min = val;
         if (val > max) max = val;
         sum += val;
         
         sorted_values[i] = val;
     }
     
     /* Sort values for median calculation */
     qsort(sorted_values, count, sizeof(float), compare_float);
     
     /* Calculate median */
     float median;
     if (count % 2 == 0) {
         median = (sorted_values[count/2 - 1] + sorted_values[count/2]) / 2.0f;
     } else {
         median = sorted_values[count/2];
     }
     
     /* Calculate standard deviation */
     float avg = sum / count;
     float variance_sum = 0.0f;
     
     for (uint32_t i = 0; i < count; i++) {
         float diff = data[i].value - avg;
         variance_sum += diff * diff;
     }
     
     float stddev = sqrtf(variance_sum / count);
     
     /* Determine trend */
     metrics_trend_t trend = calculate_trend(data, count);
     
     /* Calculate rate of change */
     float rate_of_change = get_rate_of_change(data, count);
     
     /* Fill in the statistics structure */
     stats->min = min;
     stats->max = max;
     stats->avg = avg;
     stats->median = median;
     stats->stddev = stddev;
     stats->trend = trend;
     stats->rate_of_change = rate_of_change;
     stats->sample_count = count;
     stats->timestamp = esp_timer_get_time();
     
     free(sorted_values);
     free(data);
     
     return ESP_OK;
 }
 
 /**
  * @brief Get and copy data points for a specific time period
  */
 static uint32_t get_data_points_for_period(metrics_type_t type, metrics_period_t period, metric_data_point_t **data_out) {
     /* Take mutex for this metric type */
     if (xSemaphoreTake(s_state.storages[type].mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to take mutex for metric type %d", type);
         *data_out = NULL;
         return 0;
     }
     
     uint32_t count = s_state.storages[type].count;
     if (count == 0) {
         xSemaphoreGive(s_state.storages[type].mutex);
         *data_out = NULL;
         return 0;
     }
     
     /* Determine the time window based on period */
     int64_t now = esp_timer_get_time();
     int64_t period_us;
     
     switch (period) {
         case METRICS_PERIOD_REALTIME:
             period_us = 0;  /* Just get the latest value */
             break;
         case METRICS_PERIOD_MINUTE:
             period_us = 60 * 1000000LL;
             break;
         case METRICS_PERIOD_HOUR:
             period_us = 60 * 60 * 1000000LL;
             break;
         case METRICS_PERIOD_DAY:
             period_us = 24 * 60 * 60 * 1000000LL;
             break;
         case METRICS_PERIOD_WEEK:
             period_us = 7 * 24 * 60 * 60 * 1000000LL;
             break;
         case METRICS_PERIOD_MONTH:
             period_us = 30 * 24 * 60 * 60 * 1000000LL;
             break;
         default:
             xSemaphoreGive(s_state.storages[type].mutex);
             *data_out = NULL;
             return 0;
     }
     
     /* Special case for realtime - just return the latest value */
     if (period == METRICS_PERIOD_REALTIME) {
         *data_out = malloc(sizeof(metric_data_point_t));
         if (*data_out == NULL) {
             xSemaphoreGive(s_state.storages[type].mutex);
             return 0;
         }
         
         uint32_t latest_idx = (s_state.storages[type].head + s_state.storages[type].capacity - 1) % s_state.storages[type].capacity;
         (*data_out)[0] = s_state.storages[type].data_points[latest_idx];
         
         xSemaphoreGive(s_state.storages[type].mutex);
         return 1;
     }
     
     /* Count how many data points are within the period */
     uint32_t period_count = 0;
     for (uint32_t i = 0; i < count; i++) {
         uint32_t idx = (s_state.storages[type].head + s_state.storages[type].capacity - 1 - i) % s_state.storages[type].capacity;
         if (now - s_state.storages[type].data_points[idx].timestamp <= period_us) {
             period_count++;
         } else {
             break;  /* Data is ordered by time, so we can stop */
         }
     }
     
     if (period_count == 0) {
         xSemaphoreGive(s_state.storages[type].mutex);
         *data_out = NULL;
         return 0;
     }
     
     /* Allocate memory and copy data points */
     *data_out = malloc(period_count * sizeof(metric_data_point_t));
     if (*data_out == NULL) {
         xSemaphoreGive(s_state.storages[type].mutex);
         return 0;
     }
     
     for (uint32_t i = 0; i < period_count; i++) {
         uint32_t idx = (s_state.storages[type].head + s_state.storages[type].capacity - 1 - i) % s_state.storages[type].capacity;
         (*data_out)[i] = s_state.storages[type].data_points[idx];
     }
     
     xSemaphoreGive(s_state.storages[type].mutex);
     return period_count;
 }
 
 /**
  * @brief Calculate trend from data points
  */
 static metrics_trend_t calculate_trend(const metric_data_point_t *data, uint32_t count) {
     if (count < 3) {
         return TREND_STABLE;
     }
     
     /* Simple linear regression to determine trend */
     double sum_x = 0.0;
     double sum_y = 0.0;
     double sum_xy = 0.0;
     double sum_x2 = 0.0;
     
     /* Normalize timestamps to avoid large numbers */
     int64_t base_time = data[0].timestamp;
     
     for (uint32_t i = 0; i < count; i++) {
         double x = (data[i].timestamp - base_time) / 1000000.0;  /* Convert to seconds */
         double y = data[i].value;
         
         sum_x += x;
         sum_y += y;
         sum_xy += x * y;
         sum_x2 += x * x;
     }
     
     double n = (double)count;
     double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
     
     /* Calculate variance to determine if fluctuating */
     double mean = sum_y / n;
     double variance = 0.0;
     
     for (uint32_t i = 0; i < count; i++) {
         double diff = data[i].value - mean;
         variance += diff * diff;
     }
     
     variance /= n;
     
     /* Determine trend based on slope and variance */
     double threshold = 0.001;
     double fluctuation_threshold = 0.05 * mean;  /* 5% of mean */
     
     if (variance > fluctuation_threshold * fluctuation_threshold) {
         return TREND_FLUCTUATING;
     } else if (slope > threshold) {
         return TREND_INCREASING;
     } else if (slope < -threshold) {
         return TREND_DECREASING;
     } else {
         return TREND_STABLE;
     }
 }
 
 /**
  * @brief Calculate rate of change in units per hour
  */
 static float get_rate_of_change(const metric_data_point_t *data, uint32_t count) {
     if (count < 2) {
         return 0.0f;
     }
     
     /* We'll use the first and last points to calculate rate of change */
     float first_value = data[count - 1].value;  /* Oldest point */
     float last_value = data[0].value;           /* Newest point */
     int64_t time_diff_us = data[0].timestamp - data[count - 1].timestamp;
     
     /* Convert to hours */
     float time_diff_hours = (float)time_diff_us / (3600.0f * 1000000.0f);
     
     if (time_diff_hours < 0.0001f) {
         return 0.0f;  /* Avoid division by very small numbers */
     }
     
     return (last_value - first_value) / time_diff_hours;
 }
 
 /**
  * @brief Get specific statistic for a metric
  */
 esp_err_t metrics_analyzer_get_single_statistic(metrics_type_t type,
                                                metrics_period_t period,
                                                metrics_stat_t stat_type,
                                                float *value) {
     if (!s_state.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (type >= METRIC_TYPE_MAX || period >= METRICS_PERIOD_MAX || 
         stat_type >= METRICS_STAT_MAX || value == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Get full statistics and extract the requested one */
     metrics_statistics_t stats;
     esp_err_t result = calculate_statistics(type, period, &stats);
     
     if (result != ESP_OK) {
         return result;
     }
     
     /* Extract the requested statistic */
     switch (stat_type) {
         case METRICS_STAT_MIN:
             *value = stats.min;
             break;
         case METRICS_STAT_MAX:
             *value = stats.max;
             break;
         case METRICS_STAT_AVG:
             *value = stats.avg;
             break;
         case METRICS_STAT_MEDIAN:
             *value = stats.median;
             break;
         case METRICS_STAT_STDDEV:
             *value = stats.stddev;
             break;
         case METRICS_STAT_TREND:
             *value = (float)stats.trend;
             break;
         case METRICS_STAT_RATE_OF_CHANGE:
             *value = stats.rate_of_change;
             break;
         default:
             return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }