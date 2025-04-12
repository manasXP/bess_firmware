/**
 * @file time_utils.c
 * @brief Implementation of time-related utilities for the BESS firmware
 * 
 * This file implements the time-related utilities and functions defined in
 * time_utils.h used throughout the BESS firmware.
 * 
 * @copyright Copyright (c) 2025
 */

 #include "time_utils.h"
 #include <string.h>
 #include <stdio.h>
 
 esp_err_t time_get_timestamp(timestamp_t *timestamp) {
     if (timestamp == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Get current time
     time_t now;
     struct tm timeinfo;
     time(&now);
     localtime_r(&now, &timeinfo);
 
     // Fill timestamp structure
     timestamp->year = timeinfo.tm_year + 1900;  // tm_year is years since 1900
     timestamp->month = timeinfo.tm_mon + 1;     // tm_mon is 0-11
     timestamp->day = timeinfo.tm_mday;
     timestamp->hour = timeinfo.tm_hour;
     timestamp->minute = timeinfo.tm_min;
     timestamp->second = timeinfo.tm_sec;
     
     // Get milliseconds from high-resolution timer
     // This is an approximation as POSIX time doesn't include ms
     uint64_t current_time_ms = esp_timer_get_time() / 1000;
     timestamp->millisecond = current_time_ms % 1000;
 
     return ESP_OK;
 }
 
 time_t time_timestamp_to_unix(const timestamp_t *timestamp) {
     if (timestamp == NULL) {
         return 0;
     }
 
     struct tm timeinfo;
     memset(&timeinfo, 0, sizeof(struct tm));
     
     timeinfo.tm_year = timestamp->year - 1900;  // tm_year is years since 1900
     timeinfo.tm_mon = timestamp->month - 1;     // tm_mon is 0-11
     timeinfo.tm_mday = timestamp->day;
     timeinfo.tm_hour = timestamp->hour;
     timeinfo.tm_min = timestamp->minute;
     timeinfo.tm_sec = timestamp->second;
     
     // Convert to Unix time (seconds since epoch)
     return mktime(&timeinfo);
 }
 
 esp_err_t time_unix_to_timestamp(time_t unix_time, timestamp_t *timestamp) {
     if (timestamp == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     struct tm timeinfo;
     localtime_r(&unix_time, &timeinfo);
     
     timestamp->year = timeinfo.tm_year + 1900;  // tm_year is years since 1900
     timestamp->month = timeinfo.tm_mon + 1;     // tm_mon is 0-11
     timestamp->day = timeinfo.tm_mday;
     timestamp->hour = timeinfo.tm_hour;
     timestamp->minute = timeinfo.tm_min;
     timestamp->second = timeinfo.tm_sec;
     timestamp->millisecond = 0;  // Unix time doesn't include milliseconds
     
     return ESP_OK;
 }
 
 uint32_t time_get_uptime_seconds(void) {
     // FreeRTOS tick count / tick rate = seconds
     return (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
 }
 
 uint64_t time_get_uptime_ms(void) {
     // Return system uptime in milliseconds using ESP timer
     return esp_timer_get_time() / 1000;
 }
 
 esp_err_t time_format_timestamp(const timestamp_t *timestamp, const char *format, 
                                char *buffer, size_t buffer_size) {
     if (timestamp == NULL || format == NULL || buffer == NULL || buffer_size == 0) {
         return ESP_ERR_INVALID_ARG;
     }
 
     struct tm timeinfo;
     memset(&timeinfo, 0, sizeof(struct tm));
     
     timeinfo.tm_year = timestamp->year - 1900;
     timeinfo.tm_mon = timestamp->month - 1;
     timeinfo.tm_mday = timestamp->day;
     timeinfo.tm_hour = timestamp->hour;
     timeinfo.tm_min = timestamp->minute;
     timeinfo.tm_sec = timestamp->second;
     
     size_t len = strftime(buffer, buffer_size, format, &timeinfo);
     if (len == 0) {
         return ESP_ERR_INVALID_SIZE;
     }
     
     // If format contains milliseconds placeholder (e.g., %ms), replace it
     // This is a custom extension as strftime doesn't support milliseconds
     char *ms_pos = strstr(buffer, "%ms");
     if (ms_pos != NULL) {
         char ms_str[5];
         snprintf(ms_str, sizeof(ms_str), "%03u", timestamp->millisecond);
         
         // Shift the rest of the string right to make room for milliseconds
         size_t ms_pos_idx = ms_pos - buffer;
         size_t shift_size = strlen(ms_str) - 3; // 3 is the length of "%ms"
         
         if (ms_pos_idx + strlen(ms_pos) + shift_size < buffer_size) {
             memmove(ms_pos + shift_size, ms_pos + 3, strlen(ms_pos + 3) + 1);
             memcpy(ms_pos, ms_str, strlen(ms_str));
         } else {
             return ESP_ERR_INVALID_SIZE;
         }
     }
     
     return ESP_OK;
 }
 
 esp_err_t time_parse_timestamp(const char *str, const char *format, timestamp_t *timestamp) {
     if (str == NULL || format == NULL || timestamp == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     struct tm timeinfo;
     memset(&timeinfo, 0, sizeof(struct tm));
     
     // Parse using strptime
     if (strptime(str, format, &timeinfo) == NULL) {
         return ESP_FAIL;
     }
     
     timestamp->year = timeinfo.tm_year + 1900;
     timestamp->month = timeinfo.tm_mon + 1;
     timestamp->day = timeinfo.tm_mday;
     timestamp->hour = timeinfo.tm_hour;
     timestamp->minute = timeinfo.tm_min;
     timestamp->second = timeinfo.tm_sec;
     
     // Look for milliseconds in custom format (e.g., "...%ms...")
     // This is a custom extension to handle milliseconds
     char *ms_format = strstr(format, "%ms");
     if (ms_format != NULL) {
         // Calculate offset to where milliseconds should be in the input string
         size_t format_ms_pos = ms_format - format;
         size_t format_before_ms_len = format_ms_pos;
         
         char format_before_ms[256];
         strncpy(format_before_ms, format, format_before_ms_len);
         format_before_ms[format_before_ms_len] = '\0';
         
         char temp_str[256];
         strftime(temp_str, sizeof(temp_str), format_before_ms, &timeinfo);
         
         size_t str_ms_pos = strlen(temp_str);
         if (str_ms_pos < strlen(str)) {
             sscanf(str + str_ms_pos, "%3hu", &timestamp->millisecond);
         }
     } else {
         timestamp->millisecond = 0;
     }
     
     return ESP_OK;
 }
 
 int time_compare_timestamps(const timestamp_t *t1, const timestamp_t *t2) {
     if (t1 == NULL || t2 == NULL) {
         return 0;
     }
 
     if (t1->year != t2->year) return t1->year - t2->year;
     if (t1->month != t2->month) return t1->month - t2->month;
     if (t1->day != t2->day) return t1->day - t2->day;
     if (t1->hour != t2->hour) return t1->hour - t2->hour;
     if (t1->minute != t2->minute) return t1->minute - t2->minute;
     if (t1->second != t2->second) return t1->second - t2->second;
     return t1->millisecond - t2->millisecond;
 }
 
 double time_difference_seconds(const timestamp_t *t1, const timestamp_t *t2) {
     if (t1 == NULL || t2 == NULL) {
         return 0.0;
     }
 
     // Convert both timestamps to Unix time
     time_t time1 = time_timestamp_to_unix(t1);
     time_t time2 = time_timestamp_to_unix(t2);
     
     // Calculate the difference in seconds
     double diff_seconds = difftime(time2, time1);
     
     // Add millisecond precision
     diff_seconds += (t2->millisecond - t1->millisecond) / 1000.0;
     
     return diff_seconds;
 }
 
 void time_stats_init(timing_stats_t *stats) {
     if (stats == NULL) {
         return;
     }
     
     stats->min_time = UINT64_MAX;  // Initialize to maximum possible value
     stats->max_time = 0;
     stats->total_time = 0;
     stats->count = 0;
 }
 
 void time_stats_update(timing_stats_t *stats, uint64_t time_us) {
     if (stats == NULL) {
         return;
     }
     
     // Update minimum time
     if (time_us < stats->min_time) {
         stats->min_time = time_us;
     }
     
     // Update maximum time
     if (time_us > stats->max_time) {
         stats->max_time = time_us;
     }
     
     // Update total time and count
     stats->total_time += time_us;
     stats->count++;
 }
 
 void time_stats_reset(timing_stats_t *stats) {
     time_stats_init(stats);
 }
 
 double time_stats_get_average(const timing_stats_t *stats) {
     if (stats == NULL || stats->count == 0) {
         return 0.0;
     }
     
     return (double)stats->total_time / stats->count;
 }