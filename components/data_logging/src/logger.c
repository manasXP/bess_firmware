/**
 * @file logger.c
 * @brief Implementation of the logging subsystem for BESS 100KW/200KWH Firmware
 * 
 * This module provides a unified logging interface that supports multiple
 * output destinations including console, SD card, and AWS CloudWatch.
 * 
 * @copyright (c) 2025 BESS Firmware Team
 * @version 1.0.0
 */

 #include "logger.h"
 #include "esp_system.h"
 #include "esp_wifi.h"
 #include "freertos/event_groups.h"
 #include "esp_http_client.h"
 #include "esp_crt_bundle.h"
 #include "esp_sntp.h"
 #include "sdmmc_cmd.h"
 #include "driver/sdmmc_host.h"
 #include "esp_vfs_fat.h"
 #include "cJSON.h"
 #include <string.h>
 #include <time.h>
 #include <sys/time.h>
 #include <sys/stat.h>
 #include <dirent.h>
 
 /* Private definitions */
 #define BESS_LOG_TASK_NAME           "bess_logger"
 #define BESS_LOG_DEFAULT_STACK_SIZE  4096
 #define BESS_LOG_DEFAULT_PRIORITY    5
 #define BESS_LOG_DEFAULT_QUEUE_SIZE  100
 #define BESS_LOG_MAX_MSG_LENGTH      512
 #define BESS_LOG_DEFAULT_SD_PATH     "/sdcard/logs"
 #define BESS_LOG_DEFAULT_FILE_SIZE   (1024 * 1024)  /* 1MB */
 #define BESS_LOG_DEFAULT_MAX_FILES   10
 #define BESS_LOG_AWS_ENDPOINT        ".amazonaws.com"
 #define BESS_LOG_AWS_SERVICE         "logs"
 #define BESS_LOG_AWS_API_VERSION     "2014-03-28"
 #define BESS_LOG_AWS_TIMEOUT_MS      5000
 #define BESS_LOG_FILE_EXT            ".log"
 #define BESS_LOG_DEFAULT_BATCH_SIZE  20
 #define BESS_LOG_DEFAULT_BATCH_TIMEOUT 10000  /* 10 seconds */
 #define BESS_LOG_SD_MOUNT_POINT      "/sdcard"
 #define BESS_LOG_FILENAME_BUF_SIZE   64
 #define BESS_LOG_TIME_BUF_SIZE       32
 #define BESS_LOG_AWS_URL_SIZE        256
 #define BESS_LOG_SEQUENCE_TOKEN_SIZE 128
 
 /* Color codes for console output */
 #define BESS_LOG_COLOR_BLACK   "30"
 #define BESS_LOG_COLOR_RED     "31"
 #define BESS_LOG_COLOR_GREEN   "32"
 #define BESS_LOG_COLOR_YELLOW  "33"
 #define BESS_LOG_COLOR_BLUE    "34"
 #define BESS_LOG_COLOR_MAGENTA "35"
 #define BESS_LOG_COLOR_CYAN    "36"
 #define BESS_LOG_COLOR_WHITE   "37"
 #define BESS_LOG_COLOR_RESET   "\033[0m"
 
 /* Module name strings for logging */
 static const char *s_module_names[BESS_MODULE_MAX] = {
     "SYSTEM",               /* BESS_MODULE_SYSTEM */
     "BATTERY_MANAGER",      /* BESS_MODULE_BATTERY_MANAGER */
     "SOC_CALCULATOR",       /* BESS_MODULE_SOC_CALCULATOR */
     "CELL_BALANCER",        /* BESS_MODULE_CELL_BALANCER */
     "THERMAL_MONITOR",      /* BESS_MODULE_THERMAL_MONITOR */
     "MODBUS",               /* BESS_MODULE_MODBUS */
     "CANBUS",               /* BESS_MODULE_CANBUS */
     "DATA_LOGGER",          /* BESS_MODULE_DATA_LOGGER */
     "CLOUD_SYNC",           /* BESS_MODULE_CLOUD_SYNC */
     "POWER_CONTROL",        /* BESS_MODULE_POWER_CONTROL */
     "USER_INTERFACE",       /* BESS_MODULE_USER_INTERFACE */
     "SCHEDULER",            /* BESS_MODULE_SCHEDULER */
     "PROTECTION",           /* BESS_MODULE_PROTECTION */
     "DIAGNOSTICS"           /* BESS_MODULE_DIAGNOSTICS */
 };
 
 /* Level name strings for logging */
 static const char *s_level_names[] = {
     "NONE",                 /* BESS_LOG_NONE */
     "ERROR",                /* BESS_LOG_ERROR */
     "WARN",                 /* BESS_LOG_WARN */
     "INFO",                 /* BESS_LOG_INFO */
     "DEBUG",                /* BESS_LOG_DEBUG */
     "VERBOSE",              /* BESS_LOG_VERBOSE */
     "EVENT",                /* BESS_LOG_EVENT */
     "AUDIT"                 /* BESS_LOG_AUDIT */
 };
 
 /* Level colors for console output */
 static const char *s_level_colors[] = {
     BESS_LOG_COLOR_WHITE,   /* BESS_LOG_NONE */
     BESS_LOG_COLOR_RED,     /* BESS_LOG_ERROR */
     BESS_LOG_COLOR_YELLOW,  /* BESS_LOG_WARN */
     BESS_LOG_COLOR_GREEN,   /* BESS_LOG_INFO */
     BESS_LOG_COLOR_CYAN,    /* BESS_LOG_DEBUG */
     BESS_LOG_COLOR_BLUE,    /* BESS_LOG_VERBOSE */
     BESS_LOG_COLOR_MAGENTA, /* BESS_LOG_EVENT */
     BESS_LOG_COLOR_WHITE    /* BESS_LOG_AUDIT */
 };
 
 /* Logger configuration structure */
 typedef struct {
     bess_logger_config_t config;             /* User-provided configuration */
     QueueHandle_t msg_queue;                 /* Message queue handle */
     TaskHandle_t task_handle;                /* Logger task handle */
     SemaphoreHandle_t mutex;                 /* Mutex for thread safety */
     bool initialized;                        /* Initialization flag */
     bool sd_mounted;                         /* SD card mount status */
     bool destinations[3];                    /* Enabled destinations */
     bess_log_level_t levels[3];              /* Level for each destination */
     FILE *current_file;                      /* Current log file handle */
     char current_filename[BESS_LOG_FILENAME_BUF_SIZE]; /* Current log filename */
     uint32_t current_file_size;              /* Current log file size */
     bess_logger_stats_t stats;               /* Statistics */
     char *cloud_batch;                       /* CloudWatch batch buffer */
     uint16_t cloud_batch_count;              /* Current batch message count */
     uint64_t last_batch_time;                /* Last batch upload time */
     char sequence_token[BESS_LOG_SEQUENCE_TOKEN_SIZE]; /* CloudWatch sequence token */
     EventGroupHandle_t event_group;          /* Event group for sync */
 } bess_logger_context_t;
 
 /* Event bits for the logger event group */
 #define LOGGER_EVENT_FLUSH     BIT0
 #define LOGGER_EVENT_FLUSHED   BIT1
 #define LOGGER_EVENT_SHUTDOWN  BIT2
 
 /* Static context instance */
 static bess_logger_context_t s_logger_ctx;
 
 /* Default logger configuration */
 const bess_logger_config_t BESS_LOGGER_DEFAULT_CONFIG = {
     .console_level = BESS_LOG_INFO,
     .sd_card_level = BESS_LOG_DEBUG,
     .cloud_level = BESS_LOG_ERROR,
     .sd_card_path = BESS_LOG_DEFAULT_SD_PATH,
     .aws_region = "us-east-1",
     .aws_log_group = "BESS-Logs",
     .aws_log_stream = "default-stream",
     .queue_size = BESS_LOG_DEFAULT_QUEUE_SIZE,
     .task_priority = BESS_LOG_DEFAULT_PRIORITY,
     .task_stack_size = BESS_LOG_DEFAULT_STACK_SIZE,
     .max_msg_length = BESS_LOG_MAX_MSG_LENGTH,
     .enable_timestamp = true,
     .enable_module_name = true,
     .enable_log_colors = true,
     .enable_cloud_batch = true,
     .cloud_batch_size = BESS_LOG_DEFAULT_BATCH_SIZE,
     .cloud_batch_timeout_ms = BESS_LOG_DEFAULT_BATCH_TIMEOUT,
     .sd_max_file_size = BESS_LOG_DEFAULT_FILE_SIZE,
     .sd_max_files = BESS_LOG_DEFAULT_MAX_FILES
 };
 
 /* Forward declarations for internal functions */
 static void logger_task(void *pvParameter);
 static esp_err_t write_to_console(bess_log_message_t *message);
 static esp_err_t write_to_sd_card(bess_log_message_t *message);
 static esp_err_t write_to_cloud(bess_log_message_t *message);
 static esp_err_t flush_cloud_batch(bool force);
 static esp_err_t rotate_log_file(void);
 static esp_err_t open_log_file(void);
 static void free_log_message(bess_log_message_t *message);
 static char *format_log_message(bess_log_message_t *message, bool include_metadata);
 static uint64_t get_timestamp_ms(void);
 static esp_err_t mount_sd_card(void);
 static esp_err_t clean_old_log_files(void);
 static int compare_log_files(const void *a, const void *b);
 
 /**
  * @brief Get timestamp in milliseconds since boot
  * 
  * @return uint64_t Timestamp in milliseconds
  */
 static uint64_t get_timestamp_ms(void) {
     return esp_timer_get_time() / 1000;
 }
 
 /**
  * @brief Free log message resources
  * 
  * @param message Message to free
  */
 static void free_log_message(bess_log_message_t *message) {
     if (message != NULL) {
         if (message->message != NULL) {
             free(message->message);
         }
         free(message);
     }
 }
 
 /**
  * @brief Format log message with metadata
  * 
  * @param message The log message
  * @param include_metadata Whether to include timestamp, level and module
  * @return char* Formatted message string (must be freed by caller)
  */
 static char *format_log_message(bess_log_message_t *message, bool include_metadata) {
     if (message == NULL) {
         return NULL;
     }
     
     char *formatted_message = NULL;
     
     if (include_metadata) {
         const char *level_name = (message->level < 8) ? s_level_names[message->level] : "UNKNOWN";
         const char *module_name = (message->module < BESS_MODULE_MAX) ? 
                                  s_module_names[message->module] : "UNKNOWN";
         
         /* Format timestamp */
         char timestamp_str[BESS_LOG_TIME_BUF_SIZE] = {0};
         struct timeval tv;
         gettimeofday(&tv, NULL);
         struct tm timeinfo;
         localtime_r(&tv.tv_sec, &timeinfo);
         
         snprintf(timestamp_str, BESS_LOG_TIME_BUF_SIZE, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000);
         
         asprintf(&formatted_message, "[%s][%s][%s] %s", 
                 timestamp_str, level_name, module_name, message->message);
     } else {
         formatted_message = strdup(message->message);
     }
     
     return formatted_message;
 }
 
 /**
  * @brief Compare log files for sorting
  * 
  * Comparison function for qsort to sort log files by name (timestamp).
  * 
  * @param a First filename
  * @param b Second filename
  * @return int Comparison result (-1, 0, 1)
  */
 static int compare_log_files(const void *a, const void *b) {
     return strcmp(*(const char **)a, *(const char **)b);
 }
 
 /**
  * @brief Open a new log file
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t open_log_file(void) {
     if (!s_logger_ctx.sd_mounted) {
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Create filename with timestamp */
     char timestamp[32];
     struct timeval tv;
     gettimeofday(&tv, NULL);
     struct tm timeinfo;
     localtime_r(&tv.tv_sec, &timeinfo);
     
     strftime(timestamp, sizeof(timestamp), "%Y%m%d-%H%M%S", &timeinfo);
     
     /* Format full path */
     snprintf(s_logger_ctx.current_filename, BESS_LOG_FILENAME_BUF_SIZE,
             "%s/bess_%s%s", s_logger_ctx.config.sd_card_path, timestamp, BESS_LOG_FILE_EXT);
     
     /* Open file */
     s_logger_ctx.current_file = fopen(s_logger_ctx.current_filename, "w");
     if (s_logger_ctx.current_file == NULL) {
         /* Failed to open file */
         xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
         s_logger_ctx.stats.messages_dropped++;
         xSemaphoreGive(s_logger_ctx.mutex);
         return ESP_ERR_NO_MEM;
     }
     
     /* Format the message with vsprintf */
     vsnprintf(message->message, s_logger_ctx.config.max_msg_length, format, args);
     
     /* Send message to the queue */
     if (xQueueSend(s_logger_ctx.msg_queue, &message, 0) != pdTRUE) {
         /* Queue is full, free message and increment dropped counter */
         free(message->message);
         free(message);
         
         xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
         s_logger_ctx.stats.messages_dropped++;
         xSemaphoreGive(s_logger_ctx.mutex);
         
         return ESP_ERR_NO_MEM;
     }
     
     /* Update queued message counter */
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     s_logger_ctx.stats.messages_queued++;
     if (s_logger_ctx.stats.messages_queued > s_logger_ctx.stats.queue_high_watermark) {
         s_logger_ctx.stats.queue_high_watermark = s_logger_ctx.stats.messages_queued;
     }
     xSemaphoreGive(s_logger_ctx.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Configure SD card logging parameters
  */
 esp_err_t bess_logger_configure_sd(const char *path, uint32_t max_file_size, uint32_t max_files) {
     if (!s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (path == NULL || max_file_size == 0 || max_files == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     
     /* Update configuration */
     s_logger_ctx.config.sd_card_path = path;
     s_logger_ctx.config.sd_max_file_size = max_file_size;
     s_logger_ctx.config.sd_max_files = max_files;
     
     /* Create log directory if it doesn't exist */
     if (s_logger_ctx.sd_mounted) {
         struct stat st;
         if (stat(path, &st) != 0) {
             if (mkdir(path, 0755) != 0) {
                 ESP_LOGW("LOGGER", "Failed to create log directory: %s", path);
             }
         }
         
         /* Close current file and open a new one with updated settings */
         if (s_logger_ctx.current_file != NULL) {
             fclose(s_logger_ctx.current_file);
             s_logger_ctx.current_file = NULL;
         }
         
         open_log_file();
         clean_old_log_files();
     }
     
     xSemaphoreGive(s_logger_ctx.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Configure AWS CloudWatch parameters
  */
 esp_err_t bess_logger_configure_cloud(const char *region, const char *log_group, 
                                       const char *log_stream, uint16_t batch_size,
                                       uint32_t batch_timeout_ms) {
     if (!s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (region == NULL || log_group == NULL || log_stream == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     
     /* Update configuration */
     s_logger_ctx.config.aws_region = region;
     s_logger_ctx.config.aws_log_group = log_group;
     s_logger_ctx.config.aws_log_stream = log_stream;
     
     /* Update batch settings if batching is enabled */
     if (s_logger_ctx.config.enable_cloud_batch) {
         /* If batch size is changing, reallocate the batch buffer */
         if (batch_size > 0 && batch_size != s_logger_ctx.config.cloud_batch_size) {
             s_logger_ctx.config.cloud_batch_size = batch_size;
             
             /* Force flush the current batch */
             if (s_logger_ctx.cloud_batch_count > 0) {
                 flush_cloud_batch(true);
             }
             
             /* Reallocate batch buffer */
             if (s_logger_ctx.cloud_batch != NULL) {
                 free(s_logger_ctx.cloud_batch);
             }
             
             size_t batch_buffer_size = s_logger_ctx.config.cloud_batch_size * 
                                       (s_logger_ctx.config.max_msg_length + 200);
             s_logger_ctx.cloud_batch = (char *)malloc(batch_buffer_size);
             if (s_logger_ctx.cloud_batch == NULL) {
                 s_logger_ctx.config.enable_cloud_batch = false;
             } else {
                 s_logger_ctx.cloud_batch[0] = '\0';
                 s_logger_ctx.cloud_batch_count = 0;
             }
         }
         
         /* Update batch timeout */
         if (batch_timeout_ms > 0) {
             s_logger_ctx.config.cloud_batch_timeout_ms = batch_timeout_ms;
         }
     }
     
     /* Reset sequence token since we're changing log stream */
     s_logger_ctx.sequence_token[0] = '\0';
     
     xSemaphoreGive(s_logger_ctx.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get statistics for the logging subsystem
  * 
  * @param stats Pointer to statistics structure
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 esp_err_t bess_logger_get_stats(bess_logger_stats_t *stats) {
     if (!s_logger_ctx.initialized || stats == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     memcpy(stats, &s_logger_ctx.stats, sizeof(bess_logger_stats_t));
     xSemaphoreGive(s_logger_ctx.mutex);
     
     return ESP_OK;
 }_logger_ctx.stats.sd_card_errors++;
         xSemaphoreGive(s_logger_ctx.mutex);
         return ESP_FAIL;
     }
     
     /* Reset file size counter */
     s_logger_ctx.current_file_size = 0;
     
     /* Write header */
     char header[256];
     snprintf(header, sizeof(header), 
             "--- BESS 100KW/200KWH Firmware Log File ---\n"
             "Started: %04d-%02d-%02d %02d:%02d:%02d\n"
             "-------------------------------------------\n",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
     
     fputs(header, s_logger_ctx.current_file);
     s_logger_ctx.current_file_size = strlen(header);
     
     return ESP_OK;
 }
 
 /**
  * @brief Clean old log files
  * 
  * Removes oldest log files when maximum number of files is exceeded.
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t clean_old_log_files(void) {
     if (!s_logger_ctx.sd_mounted) {
         return ESP_ERR_INVALID_STATE;
     }
     
     DIR *dir = opendir(s_logger_ctx.config.sd_card_path);
     if (dir == NULL) {
         return ESP_FAIL;
     }
     
     /* Count log files and collect names */
     char **log_files = NULL;
     int file_count = 0;
     struct dirent *entry;
     
     /* First pass: count files */
     while ((entry = readdir(dir)) != NULL) {
         if (entry->d_type == DT_REG && 
             strstr(entry->d_name, BESS_LOG_FILE_EXT) != NULL) {
             file_count++;
         }
     }
     
     /* Check if cleanup needed */
     if (file_count <= s_logger_ctx.config.sd_max_files) {
         closedir(dir);
         return ESP_OK;
     }
     
     /* Allocate memory for filenames */
     log_files = (char **)malloc(file_count * sizeof(char *));
     if (log_files == NULL) {
         closedir(dir);
         return ESP_ERR_NO_MEM;
     }
     
     /* Second pass: collect filenames */
     rewinddir(dir);
     file_count = 0;
     while ((entry = readdir(dir)) != NULL) {
         if (entry->d_type == DT_REG && 
             strstr(entry->d_name, BESS_LOG_FILE_EXT) != NULL) {
             log_files[file_count] = strdup(entry->d_name);
             if (log_files[file_count] != NULL) {
                 file_count++;
             }
         }
     }
     
     closedir(dir);
     
     /* Sort filenames */
     qsort(log_files, file_count, sizeof(char *), compare_log_files);
     
     /* Delete oldest files */
     int files_to_delete = file_count - s_logger_ctx.config.sd_max_files;
     for (int i = 0; i < files_to_delete; i++) {
         char full_path[BESS_LOG_FILENAME_BUF_SIZE];
         snprintf(full_path, BESS_LOG_FILENAME_BUF_SIZE, "%s/%s", 
                 s_logger_ctx.config.sd_card_path, log_files[i]);
         
         /* Delete file */
         unlink(full_path);
     }
     
     /* Free memory */
     for (int i = 0; i < file_count; i++) {
         free(log_files[i]);
     }
     free(log_files);
     
     return ESP_OK;
 }
 
 /**
  * @brief Rotate log file
  * 
  * Closes the current log file and opens a new one.
  * Implements log file rotation based on maximum size.
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t rotate_log_file(void) {
     if (!s_logger_ctx.sd_mounted) {
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Close current file if open */
     if (s_logger_ctx.current_file != NULL) {
         fclose(s_logger_ctx.current_file);
         s_logger_ctx.current_file = NULL;
     }
     
     /* Clean old log files */
     clean_old_log_files();
     
     /* Open new log file */
     return open_log_file();
 }
 
 /**
  * @brief Mount SD card
  * 
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t mount_sd_card(void) {
     /* SD card configuration */
     sdmmc_host_t host = SDMMC_HOST_DEFAULT();
     sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
     
     /* Mount configuration */
     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
         .format_if_mount_failed = false,
         .max_files = 5,
         .allocation_unit_size = 16 * 1024
     };
     
     /* Mount */
     sdmmc_card_t *card;
     esp_err_t ret = esp_vfs_fat_sdmmc_mount(BESS_LOG_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
     
     if (ret != ESP_OK) {
         if (ret == ESP_FAIL) {
             ESP_LOGE("LOGGER", "Failed to mount filesystem on SD card");
         } else {
             ESP_LOGE("LOGGER", "Failed to initialize SD card: %s", esp_err_to_name(ret));
         }
         return ret;
     }
     
     /* Card mounted successfully */
     s_logger_ctx.sd_mounted = true;
     
     /* Print card information */
     ESP_LOGI("LOGGER", "SD card mounted, size: %lluMB", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
     
     return ESP_OK;
 }
 
 /**
  * @brief Write log message to console
  * 
  * @param message The log message to write
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t write_to_console(bess_log_message_t *message) {
     if (message == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     char timestamp_str[BESS_LOG_TIME_BUF_SIZE] = {0};
     char *formatted_message;
     
     /* Format timestamp if enabled */
     if (s_logger_ctx.config.enable_timestamp) {
         struct timeval tv;
         gettimeofday(&tv, NULL);
         struct tm timeinfo;
         localtime_r(&tv.tv_sec, &timeinfo);
         
         snprintf(timestamp_str, BESS_LOG_TIME_BUF_SIZE, "%02d:%02d:%02d.%03ld",
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000);
     }
     
     /* Get level and module names */
     const char *level_name = (message->level < 8) ? s_level_names[message->level] : "UNKNOWN";
     const char *module_name = (message->module < BESS_MODULE_MAX) ? 
                              s_module_names[message->module] : "UNKNOWN";
     
     /* Format with colors if enabled */
     if (s_logger_ctx.config.enable_log_colors) {
         const char *color = (message->level < 8) ? s_level_colors[message->level] : BESS_LOG_COLOR_WHITE;
         
         if (s_logger_ctx.config.enable_timestamp && s_logger_ctx.config.enable_module_name) {
             printf("\033[%sm[%s][%s][%s] %s" BESS_LOG_COLOR_RESET "\n", 
                   color, timestamp_str, level_name, module_name, message->message);
         } else if (s_logger_ctx.config.enable_timestamp) {
             printf("\033[%sm[%s][%s] %s" BESS_LOG_COLOR_RESET "\n", 
                   color, timestamp_str, level_name, message->message);
         } else if (s_logger_ctx.config.enable_module_name) {
             printf("\033[%sm[%s][%s] %s" BESS_LOG_COLOR_RESET "\n", 
                   color, level_name, module_name, message->message);
         } else {
             printf("\033[%sm[%s] %s" BESS_LOG_COLOR_RESET "\n", 
                   color, level_name, message->message);
         }
     } else {
         if (s_logger_ctx.config.enable_timestamp && s_logger_ctx.config.enable_module_name) {
             printf("[%s][%s][%s] %s\n", timestamp_str, level_name, module_name, message->message);
         } else if (s_logger_ctx.config.enable_timestamp) {
             printf("[%s][%s] %s\n", timestamp_str, level_name, message->message);
         } else if (s_logger_ctx.config.enable_module_name) {
             printf("[%s][%s] %s\n", level_name, module_name, message->message);
         } else {
             printf("[%s] %s\n", level_name, message->message);
         }
     }
     
     /* Update statistics */
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     s_logger_ctx.stats.console_messages++;
     xSemaphoreGive(s_logger_ctx.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Write log message to SD card
  * 
  * @param message The log message to write
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t write_to_sd_card(bess_log_message_t *message) {
     if (message == NULL || !s_logger_ctx.sd_mounted || s_logger_ctx.current_file == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     char timestamp_str[BESS_LOG_TIME_BUF_SIZE] = {0};
     struct timeval tv;
     gettimeofday(&tv, NULL);
     struct tm timeinfo;
     localtime_r(&tv.tv_sec, &timeinfo);
     
     /* Format timestamp (always include for file logging) */
     snprintf(timestamp_str, BESS_LOG_TIME_BUF_SIZE, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000);
     
     /* Get level and module names */
     const char *level_name = (message->level < 8) ? s_level_names[message->level] : "UNKNOWN";
     const char *module_name = (message->module < BESS_MODULE_MAX) ? 
                              s_module_names[message->module] : "UNKNOWN";
     
     /* Format message */
     char formatted_message[s_logger_ctx.config.max_msg_length + 100]; /* Add space for metadata */
     int len;
     
     if (s_logger_ctx.config.enable_module_name) {
         len = snprintf(formatted_message, sizeof(formatted_message), 
                       "[%s][%s][%s] %s\n", 
                       timestamp_str, level_name, module_name, message->message);
     } else {
         len = snprintf(formatted_message, sizeof(formatted_message), 
                       "[%s][%s] %s\n", 
                       timestamp_str, level_name, message->message);
     }
     
     /* Write to file */
     if (fputs(formatted_message, s_logger_ctx.current_file) == EOF) {
         /* File error */
         xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
         s_logger_ctx.stats.sd_card_errors++;
         xSemaphoreGive(s_logger_ctx.mutex);
         return ESP_FAIL;
     }
     
     /* Update file size and check for rotation */
     s_logger_ctx.current_file_size += len;
     if (s_logger_ctx.current_file_size >= s_logger_ctx.config.sd_max_file_size) {
         rotate_log_file();
     }
     
     /* Update statistics */
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     s_logger_ctx.stats.sd_card_messages++;
     xSemaphoreGive(s_logger_ctx.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Flush CloudWatch batch
  * 
  * @param force Force flush even if batch is not full
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t flush_cloud_batch(bool force) {
     if (!s_logger_ctx.config.enable_cloud_batch || 
         s_logger_ctx.cloud_batch == NULL ||
         s_logger_ctx.cloud_batch_count == 0) {
         return ESP_OK;  /* Nothing to flush */
     }
     
     /* Complete the JSON array */
     strcat(s_logger_ctx.cloud_batch, "]");
     
     /* Prepare AWS CloudWatch PutLogEvents request */
     /* This is a simplified implementation - in a real application,
        AWS Signature v4 signing would be required */
     char url[BESS_LOG_AWS_URL_SIZE];
     snprintf(url, sizeof(url), "https://logs.%s.%s/%s/resources/logs/actions/PutLogEvents",
              s_logger_ctx.config.aws_region, BESS_LOG_AWS_ENDPOINT, BESS_LOG_AWS_API_VERSION);
     
     /* Create JSON body */
     cJSON *root = cJSON_CreateObject();
     cJSON_AddStringToObject(root, "logGroupName", s_logger_ctx.config.aws_log_group);
     cJSON_AddStringToObject(root, "logStreamName", s_logger_ctx.config.aws_log_stream);
     
     /* Add sequence token if we have one */
     if (s_logger_ctx.sequence_token[0] != '\0') {
         cJSON_AddStringToObject(root, "sequenceToken", s_logger_ctx.sequence_token);
     }
     
     /* Add log events array */
     cJSON *log_events = cJSON_Parse(s_logger_ctx.cloud_batch);
     if (log_events != NULL) {
         cJSON_AddItemToObject(root, "logEvents", log_events);
     } else {
         cJSON_Delete(root);
         
         /* Reset batch */
         s_logger_ctx.cloud_batch[0] = '\0';
         s_logger_ctx.cloud_batch_count = 0;
         
         /* Update statistics */
         xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
         s_logger_ctx.stats.cloud_failures++;
         xSemaphoreGive(s_logger_ctx.mutex);
         
         return ESP_FAIL;
     }
     
     char *request_body = cJSON_Print(root);
     cJSON_Delete(root);
     
     if (request_body == NULL) {
         /* Reset batch */
         s_logger_ctx.cloud_batch[0] = '\0';
         s_logger_ctx.cloud_batch_count = 0;
         
         /* Update statistics */
         xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
         s_logger_ctx.stats.cloud_failures++;
         xSemaphoreGive(s_logger_ctx.mutex);
         
         return ESP_ERR_NO_MEM;
     }
     
     /* Configure HTTP client */
     esp_http_client_config_t config = {
         .url = url,
         .method = HTTP_METHOD_POST,
         .timeout_ms = BESS_LOG_AWS_TIMEOUT_MS,
         .crt_bundle_attach = esp_crt_bundle_attach,
     };
     
     esp_http_client_handle_t client = esp_http_client_init(&config);
     esp_http_client_set_header(client, "Content-Type", "application/json");
     esp_http_client_set_header(client, "X-Amz-Date", "20250101T000000Z");  /* Placeholder */
     
     /* Set request body */
     esp_http_client_set_post_field(client, request_body, strlen(request_body));
     
     /* Perform request */
     esp_err_t result = esp_http_client_perform(client);
     
     /* Process response */
     if (result == ESP_OK) {
         int status_code = esp_http_client_get_status_code(client);
         
         if (status_code == 200) {
             /* Extract new sequence token from response */
             int content_length = esp_http_client_get_content_length(client);
             if (content_length > 0) {
                 char *response_body = (char *)malloc(content_length + 1);
                 if (response_body != NULL) {
                     esp_http_client_read_response(client, response_body, content_length);
                     response_body[content_length] = '\0';
                     
                     /* Parse response to get sequence token */
                     cJSON *response_json = cJSON_Parse(response_body);
                     if (response_json != NULL) {
                         cJSON *token = cJSON_GetObjectItem(response_json, "nextSequenceToken");
                         if (token != NULL && cJSON_IsString(token)) {
                             strncpy(s_logger_ctx.sequence_token, token->valuestring, 
                                    BESS_LOG_SEQUENCE_TOKEN_SIZE - 1);
                             s_logger_ctx.sequence_token[BESS_LOG_SEQUENCE_TOKEN_SIZE - 1] = '\0';
                         }
                         cJSON_Delete(response_json);
                     }
                     
                     free(response_body);
                 }
             }
             
             /* Update statistics */
             xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
             s_logger_ctx.stats.cloud_batches++;
             xSemaphoreGive(s_logger_ctx.mutex);
         } else {
             /* Update statistics */
             xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
             s_logger_ctx.stats.cloud_failures++;
             xSemaphoreGive(s_logger_ctx.mutex);
             
             result = ESP_FAIL;
         }
     } else {
         /* Update statistics */
         xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
         s_logger_ctx.stats.cloud_failures++;
         xSemaphoreGive(s_logger_ctx.mutex);
     }
     
     /* Clean up */
     esp_http_client_cleanup(client);
     free(request_body);
     
     /* Reset batch */
     s_logger_ctx.cloud_batch[0] = '\0';
     s_logger_ctx.cloud_batch_count = 0;
     
     return result;
 }
 
 /**
  * @brief Write log message to AWS CloudWatch
  * 
  * @param message The log message to write
  * @return esp_err_t ESP_OK on success, otherwise an error code
  */
 static esp_err_t write_to_cloud(bess_log_message_t *message) {
     if (message == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Format timestamp (always include for cloud logging) */
     char timestamp_str[BESS_LOG_TIME_BUF_SIZE] = {0};
     struct timeval tv;
     gettimeofday(&tv, NULL);
     struct tm timeinfo;
     localtime_r(&tv.tv_sec, &timeinfo);
     
     snprintf(timestamp_str, BESS_LOG_TIME_BUF_SIZE, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000);
     
     /* Get level and module names */
     const char *level_name = (message->level < 8) ? s_level_names[message->level] : "UNKNOWN";
     const char *module_name = (message->module < BESS_MODULE_MAX) ? 
                              s_module_names[message->module] : "UNKNOWN";
     
     /* Format message */
     char *formatted_message;
     if (s_logger_ctx.config.enable_module_name) {
         asprintf(&formatted_message, "[%s][%s][%s] %s", 
                 timestamp_str, level_name, module_name, message->message);
     } else {
         asprintf(&formatted_message, "[%s][%s] %s", 
                 timestamp_str, level_name, message->message);
     }
     
     if (formatted_message == NULL) {
         return ESP_ERR_NO_MEM;
     }
     
     /* Handle message based on batching configuration */
     esp_err_t result = ESP_OK;
     
     if (s_logger_ctx.config.enable_cloud_batch) {
         /* Create JSON object for the log event */
         cJSON *log_event = cJSON_CreateObject();
         cJSON_AddStringToObject(log_event, "message", formatted_message);
         cJSON_AddNumberToObject(log_event, "timestamp", (double)tv.tv_sec * 1000 + tv.tv_usec / 1000);
         
         /* Add to batch */
         char *event_str = cJSON_Print(log_event);
         cJSON_Delete(log_event);
         
         if (event_str != NULL) {
             /* Append to batch buffer */
             if (s_logger_ctx.cloud_batch_count == 0) {
                 /* First message in batch */
                 strcpy(s_logger_ctx.cloud_batch, "[");
                 strcat(s_logger_ctx.cloud_batch, event_str);
                 s_logger_ctx.cloud_batch_count = 1;
                 s_logger_ctx.last_batch_time = get_timestamp_ms();
             } else {
                 /* Append to existing batch */
                 strcat(s_logger_ctx.cloud_batch, ",");
                 strcat(s_logger_ctx.cloud_batch, event_str);
                 s_logger_ctx.cloud_batch_count++;
             }
             
             free(event_str);
             
             /* Check if batch is full */
             if (s_logger_ctx.cloud_batch_count >= s_logger_ctx.config.cloud_batch_size) {
                 result = flush_cloud_batch(false);
             }
         } else {
             result = ESP_ERR_NO_MEM;
         }
     } else {
         /* Send individually (not implemented) */
         /* This would be similar to flush_cloud_batch but for a single message */
         /* Not implemented in this version */
         result = ESP_ERR_NOT_SUPPORTED;
     }
     
     free(formatted_message);
     
     /* Update statistics */
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     s_logger_ctx.stats.cloud_messages++;
     xSemaphoreGive(s_logger_ctx.mutex);
     
     return result;
 }
 
 /**
  * @brief Logger task that processes queued messages
  * 
  * This task continuously monitors the message queue and processes
  * any log messages by writing them to the configured destinations.
  * 
  * @param pvParameter Task parameters (unused)
  */
 static void logger_task(void *pvParameter) {
     bess_log_message_t *message;
     uint64_t last_check = get_timestamp_ms();
     bool shutdown_requested = false;
     
     while (!shutdown_requested) {
         /* Check for events */
         EventBits_t bits = xEventGroupGetBits(s_logger_ctx.event_group);
         
         /* Handle flush request */
         if (bits & LOGGER_EVENT_FLUSH) {
             /* Process all queued messages */
             while (xQueueReceive(s_logger_ctx.msg_queue, &message, 0) == pdTRUE) {
                 /* Process message */
                 if (message->destinations & BESS_LOG_DEST_CONSOLE) {
                     write_to_console(message);
                 }
                 
                 if (message->destinations & BESS_LOG_DEST_SD_CARD) {
                     write_to_sd_card(message);
                 }
                 
                 if (message->destinations & BESS_LOG_DEST_CLOUD) {
                     write_to_cloud(message);
                 }
                 
                 /* Update statistics */
                 xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
                 s_logger_ctx.stats.messages_processed++;
                 s_logger_ctx.stats.messages_queued--;
                 xSemaphoreGive(s_logger_ctx.mutex);
                 
                 /* Free message */
                 free_log_message(message);
             }
             
             /* Flush CloudWatch batch if there's any pending messages */
             if (s_logger_ctx.cloud_batch_count > 0) {
                 flush_cloud_batch(true);
             }
             
             /* Flush SD card file */
             if (s_logger_ctx.current_file != NULL) {
                 fflush(s_logger_ctx.current_file);
             }
             
             /* Clear flush bit and set flushed bit */
             xEventGroupClearBits(s_logger_ctx.event_group, LOGGER_EVENT_FLUSH);
             xEventGroupSetBits(s_logger_ctx.event_group, LOGGER_EVENT_FLUSHED);
         }
         
         /* Check for shutdown request */
         if (bits & LOGGER_EVENT_SHUTDOWN) {
             shutdown_requested = true;
             continue;
         }
         
         /* Wait for messages with timeout */
         if (xQueueReceive(s_logger_ctx.msg_queue, &message, pdMS_TO_TICKS(100)) == pdTRUE) {
             /* Process message */
             if (message->destinations & BESS_LOG_DEST_CONSOLE) {
                 write_to_console(message);
             }
             
             if (message->destinations & BESS_LOG_DEST_SD_CARD) {
                 write_to_sd_card(message);
             }
             
             if (message->destinations & BESS_LOG_DEST_CLOUD) {
                 write_to_cloud(message);
             }
             
             /* Update statistics */
             xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
             s_logger_ctx.stats.messages_processed++;
             s_logger_ctx.stats.messages_queued--;
             xSemaphoreGive(s_logger_ctx.mutex);
             
             /* Free message */
             free_log_message(message);
         }
         
         /* Check if it's time to flush cloud batch due to timeout */
         uint64_t now = get_timestamp_ms();
         if (s_logger_ctx.config.enable_cloud_batch && 
             s_logger_ctx.cloud_batch_count > 0 &&
             (now - s_logger_ctx.last_batch_time) > s_logger_ctx.config.cloud_batch_timeout_ms) {
             flush_cloud_batch(false);
         }
     }
     
     /* Clean up any remaining messages in the queue */
     while (xQueueReceive(s_logger_ctx.msg_queue, &message, 0) == pdTRUE) {
         free_log_message(message);
     }
     
     /* Flush and close log file */
     if (s_logger_ctx.current_file != NULL) {
         fflush(s_logger_ctx.current_file);
     }
     
     /* Final CloudWatch batch flush */
     if (s_logger_ctx.cloud_batch_count > 0) {
         flush_cloud_batch(true);
     }
     
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Initialize the logging subsystem with specified configuration
  */
 esp_err_t bess_logger_init(const bess_logger_config_t *config) {
     if (s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     /* Initialize context with default values */
     memset(&s_logger_ctx, 0, sizeof(bess_logger_context_t));
     
     /* Copy configuration or use defaults */
     if (config != NULL) {
         memcpy(&s_logger_ctx.config, config, sizeof(bess_logger_config_t));
     } else {
         memcpy(&s_logger_ctx.config, &BESS_LOGGER_DEFAULT_CONFIG, sizeof(bess_logger_config_t));
     }
 
     /* Set initial log levels */
     s_logger_ctx.levels[0] = s_logger_ctx.config.console_level;  /* Console */
     s_logger_ctx.levels[1] = s_logger_ctx.config.sd_card_level;  /* SD Card */
     s_logger_ctx.levels[2] = s_logger_ctx.config.cloud_level;    /* Cloud */
     
     /* Enable all destinations by default */
     s_logger_ctx.destinations[0] = true;  /* Console */
     s_logger_ctx.destinations[1] = true;  /* SD Card */
     s_logger_ctx.destinations[2] = true;  /* Cloud */
 
     /* Create message queue */
     s_logger_ctx.msg_queue = xQueueCreate(s_logger_ctx.config.queue_size, sizeof(bess_log_message_t *));
     if (s_logger_ctx.msg_queue == NULL) {
         return ESP_ERR_NO_MEM;
     }
 
     /* Create mutex for thread safety */
     s_logger_ctx.mutex = xSemaphoreCreateMutex();
     if (s_logger_ctx.mutex == NULL) {
         vQueueDelete(s_logger_ctx.msg_queue);
         return ESP_ERR_NO_MEM;
     }
     
     /* Create event group for synchronization */
     s_logger_ctx.event_group = xEventGroupCreate();
     if (s_logger_ctx.event_group == NULL) {
         vSemaphoreDelete(s_logger_ctx.mutex);
         vQueueDelete(s_logger_ctx.msg_queue);
         return ESP_ERR_NO_MEM;
     }
 
     /* Allocate cloud batch buffer if cloud batching is enabled */
     if (s_logger_ctx.config.enable_cloud_batch) {
         /* Calculate buffer size: batch_size * max_msg_length * estimated JSON overhead */
         size_t batch_buffer_size = s_logger_ctx.config.cloud_batch_size * 
                                   (s_logger_ctx.config.max_msg_length + 200);
         s_logger_ctx.cloud_batch = (char *)malloc(batch_buffer_size);
         if (s_logger_ctx.cloud_batch == NULL) {
             vEventGroupDelete(s_logger_ctx.event_group);
             vSemaphoreDelete(s_logger_ctx.mutex);
             vQueueDelete(s_logger_ctx.msg_queue);
             return ESP_ERR_NO_MEM;
         }
         s_logger_ctx.cloud_batch[0] = '\0';  /* Initialize as empty string */
         s_logger_ctx.cloud_batch_count = 0;
     }
 
     /* Mount SD card if SD card logging is enabled */
     if (s_logger_ctx.destinations[1]) {
         esp_err_t ret = mount_sd_card();
         if (ret != ESP_OK) {
             ESP_LOGW("LOGGER", "Failed to mount SD card, SD logging disabled");
             s_logger_ctx.destinations[1] = false;
         } else {
             s_logger_ctx.sd_mounted = true;
             
             /* Create log directory if it doesn't exist */
             struct stat st;
             if (stat(s_logger_ctx.config.sd_card_path, &st) != 0) {
                 if (mkdir(s_logger_ctx.config.sd_card_path, 0755) != 0) {
                     ESP_LOGW("LOGGER", "Failed to create log directory: %s", s_logger_ctx.config.sd_card_path);
                     s_logger_ctx.destinations[1] = false;
                 }
             } else if (!S_ISDIR(st.st_mode)) {
                 ESP_LOGW("LOGGER", "Log path exists but is not a directory: %s", s_logger_ctx.config.sd_card_path);
                 s_logger_ctx.destinations[1] = false;
             }
             
             /* Open initial log file */
             if (s_logger_ctx.destinations[1]) {
                 if (open_log_file() != ESP_OK) {
                     ESP_LOGW("LOGGER", "Failed to open log file, SD logging disabled");
                     s_logger_ctx.destinations[1] = false;
                 }
                 
                 /* Clean old log files */
                 clean_old_log_files();
             }
         }
     }
 
     /* Create the logger task */
     BaseType_t task_created = xTaskCreate(
         logger_task,
         BESS_LOG_TASK_NAME,
         s_logger_ctx.config.task_stack_size,
         NULL,
         s_logger_ctx.config.task_priority,
         &s_logger_ctx.task_handle
     );
     
     if (task_created != pdPASS) {
         if (s_logger_ctx.cloud_batch != NULL) {
             free(s_logger_ctx.cloud_batch);
         }
         if (s_logger_ctx.current_file != NULL) {
             fclose(s_logger_ctx.current_file);
         }
         vEventGroupDelete(s_logger_ctx.event_group);
         vSemaphoreDelete(s_logger_ctx.mutex);
         vQueueDelete(s_logger_ctx.msg_queue);
         return ESP_ERR_NO_MEM;
     }
     
     s_logger_ctx.initialized = true;
     
     /* Log initialization message */
     bess_logger_log(BESS_LOG_INFO, BESS_MODULE_DATA_LOGGER, BESS_LOG_DEST_CONSOLE,
                    "Logging subsystem initialized");
     
     return ESP_OK;
 }
 
 /**
  * @brief Deinitialize the logging subsystem and free resources
  */
 esp_err_t bess_logger_deinit(void) {
     if (!s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Set shutdown flag */
     xEventGroupSetBits(s_logger_ctx.event_group, LOGGER_EVENT_SHUTDOWN);
     
     /* Wait for task to terminate */
     if (s_logger_ctx.task_handle != NULL) {
         vTaskDelay(pdMS_TO_TICKS(100));  /* Give task time to notice shutdown flag */
         vTaskDelete(s_logger_ctx.task_handle);
         s_logger_ctx.task_handle = NULL;
     }
     
     /* Flush and close log file */
     if (s_logger_ctx.current_file != NULL) {
         fclose(s_logger_ctx.current_file);
         s_logger_ctx.current_file = NULL;
     }
     
     /* Free resources */
     if (s_logger_ctx.cloud_batch != NULL) {
         free(s_logger_ctx.cloud_batch);
         s_logger_ctx.cloud_batch = NULL;
     }
     
     /* Clear message queue */
     bess_log_message_t *message;
     while (xQueueReceive(s_logger_ctx.msg_queue, &message, 0) == pdTRUE) {
         free_log_message(message);
     }
     
     vEventGroupDelete(s_logger_ctx.event_group);
     vSemaphoreDelete(s_logger_ctx.mutex);
     vQueueDelete(s_logger_ctx.msg_queue);
     
     s_logger_ctx.initialized = false;
     return ESP_OK;
 }
 
 /**
  * @brief Set logging level for a specific destination
  */
 esp_err_t bess_logger_set_level(bess_log_destination_t destination, bess_log_level_t level) {
     if (!s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (level > BESS_LOG_ALL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     
     if (destination & BESS_LOG_DEST_CONSOLE) {
         s_logger_ctx.levels[0] = level;
     }
     
     if (destination & BESS_LOG_DEST_SD_CARD) {
         s_logger_ctx.levels[1] = level;
     }
     
     if (destination & BESS_LOG_DEST_CLOUD) {
         s_logger_ctx.levels[2] = level;
     }
     
     xSemaphoreGive(s_logger_ctx.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Get current logging level for a specific destination
  */
 esp_err_t bess_logger_get_level(bess_log_destination_t destination, bess_log_level_t *level) {
     if (!s_logger_ctx.initialized || level == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (destination == BESS_LOG_DEST_CONSOLE) {
         *level = s_logger_ctx.levels[0];
     } else if (destination == BESS_LOG_DEST_SD_CARD) {
         *level = s_logger_ctx.levels[1];
     } else if (destination == BESS_LOG_DEST_CLOUD) {
         *level = s_logger_ctx.levels[2];
     } else {
         return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Enable or disable a logging destination
  */
 esp_err_t bess_logger_enable_destination(bess_log_destination_t destination, bool enable) {
     if (!s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     xSemaphoreTake(s_logger_ctx.mutex, portMAX_DELAY);
     
     if (destination & BESS_LOG_DEST_CONSOLE) {
         s_logger_ctx.destinations[0] = enable;
     }
     
     if (destination & BESS_LOG_DEST_SD_CARD) {
         /* If enabling SD and not already mounted, try to mount */
         if (enable && !s_logger_ctx.sd_mounted) {
             if (mount_sd_card() == ESP_OK) {
                 s_logger_ctx.sd_mounted = true;
                 open_log_file();
             } else {
                 enable = false;  /* Failed to mount, don't enable */
             }
         }
         s_logger_ctx.destinations[1] = enable;
     }
     
     if (destination & BESS_LOG_DEST_CLOUD) {
         /* If enabling cloud batching and buffer not allocated, allocate it */
         if (enable && s_logger_ctx.config.enable_cloud_batch && s_logger_ctx.cloud_batch == NULL) {
             size_t batch_buffer_size = s_logger_ctx.config.cloud_batch_size * 
                                       (s_logger_ctx.config.max_msg_length + 200);
             s_logger_ctx.cloud_batch = (char *)malloc(batch_buffer_size);
             if (s_logger_ctx.cloud_batch == NULL) {
                 enable = false;  /* Failed to allocate, don't enable */
             } else {
                 s_logger_ctx.cloud_batch[0] = '\0';
                 s_logger_ctx.cloud_batch_count = 0;
             }
         }
         s_logger_ctx.destinations[2] = enable;
     }
     
     xSemaphoreGive(s_logger_ctx.mutex);
     return ESP_OK;
 }
 
 /**
  * @brief Check if a specific logging destination is enabled
  */
 esp_err_t bess_logger_is_destination_enabled(bess_log_destination_t destination, bool *enabled) {
     if (!s_logger_ctx.initialized || enabled == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (destination == BESS_LOG_DEST_CONSOLE) {
         *enabled = s_logger_ctx.destinations[0];
     } else if (destination == BESS_LOG_DEST_SD_CARD) {
         *enabled = s_logger_ctx.destinations[1];
     } else if (destination == BESS_LOG_DEST_CLOUD) {
         *enabled = s_logger_ctx.destinations[2];
     } else {
         return ESP_ERR_INVALID_ARG;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Flush all pending log messages
  */
 esp_err_t bess_logger_flush(uint32_t timeout_ms) {
     if (!s_logger_ctx.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Set flush event bit */
     xEventGroupClearBits(s_logger_ctx.event_group, LOGGER_EVENT_FLUSHED);
     xEventGroupSetBits(s_logger_ctx.event_group, LOGGER_EVENT_FLUSH);
     
     /* Wait for flushed bit or timeout */
     TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
     EventBits_t bits = xEventGroupWaitBits(
         s_logger_ctx.event_group,
         LOGGER_EVENT_FLUSHED,
         pdTRUE,  /* Clear on exit */
         pdFALSE, /* Wait for any bit */
         ticks
     );
     
     return (bits & LOGGER_EVENT_FLUSHED) ? ESP_OK : ESP_ERR_TIMEOUT;
 }
 
 /**
  * @brief Log a message with variable arguments
  */
 esp_err_t bess_logger_log(bess_log_level_t level, bess_module_t module, 
                           bess_log_destination_t destinations, 
                           const char *format, ...) {
     va_list args;
     va_start(args, format);
     esp_err_t result = bess_logger_vlog(level, module, destinations, format, args);
     va_end(args);
     return result;
 }
 
 /**
 * @brief Log a message with va_list arguments
 * 
 * This function is responsible for processing log messages with a variable 
 * argument list. It validates inputs, formats the message, and places it in
 * the logging queue for processing by the logger task. The function supports
 * multiple output destinations including console, SD card, and AWS CloudWatch.
 * 
 * @param level Log level of the message
 * @param module Source module identifier
 * @param destinations Bitwise combination of destination flags
 * @param format Printf-style format string
 * @param args Variable argument list
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t bess_logger_vlog(bess_log_level_t level, bess_module_t module, 
    bess_log_destination_t destinations, 
    const char *format, va_list args) {
    
    // Check for valid inputs
    if (format == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (module >= BESS_MODULE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    // Get the logger configuration and state
    extern bess_logger_config_t g_logger_config;
    extern QueueHandle_t g_log_queue;
    extern SemaphoreHandle_t g_logger_mutex;
    extern bess_logger_stats_t g_logger_stats;
    extern bess_log_level_t g_console_level;
    extern bess_log_level_t g_sd_card_level;
    extern bess_log_level_t g_cloud_level;
    extern bool g_destinations_enabled[3]; // Console, SD, Cloud

    // Early return if logger is not initialized
    if (g_log_queue == NULL || g_logger_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check if any enabled destination should receive this message based on log level
    bool should_log = false;

    // For each destination, check if it's enabled and if the log level is appropriate
    if ((destinations & BESS_LOG_DEST_CONSOLE) && 
        g_destinations_enabled[0] && 
        level <= g_console_level) {
        should_log = true;
    } else if ((destinations & BESS_LOG_DEST_SD_CARD) && 
        g_destinations_enabled[1] && 
        level <= g_sd_card_level) {
        should_log = true;
    } else if ((destinations & BESS_LOG_DEST_CLOUD) && 
        g_destinations_enabled[2] && 
        level <= g_cloud_level) {
        should_log = true;
    }

    // If no appropriate destination, return early
    if (!should_log) {
        return ESP_OK;
    }

    // Create a log message structure
    bess_log_message_t* log_message = (bess_log_message_t*)malloc(sizeof(bess_log_message_t));
    if (log_message == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Initialize the log message
    log_message->level = level;
    log_message->module = module;
    log_message->timestamp = pdTICKS_TO_MS(xTaskGetTickCount()); // Current time in ms
    log_message->destinations = destinations;

    // Format the message with variable arguments
    va_list args_copy;
    va_copy(args_copy, args);

    // First, calculate the required buffer size
    int msg_length = vsnprintf(NULL, 0, format, args_copy);
    va_end(args_copy);

    if (msg_length < 0) {
        free(log_message);
        return ESP_FAIL;
    }

    // Ensure the message doesn't exceed the maximum allowed length
    if (msg_length > g_logger_config.max_msg_length - 1) {
        msg_length = g_logger_config.max_msg_length - 1;
    }

    // Allocate memory for the message
    log_message->message = (char*)malloc(msg_length + 1); // +1 for null terminator
    if (log_message->message == NULL) {
        free(log_message);
        return ESP_ERR_NO_MEM;
    }

// Format the message
vsnprintf(log_message->message, msg_length + 1, format, args);

// Attempt to add the message to the queue
if (xQueueSend(g_log_queue, &log_message, 0) != pdPASS) {
    // Queue is full, update stats and free memory
    if (xSemaphoreTake(g_logger_mutex, 0) == pdTRUE) {
        g_logger_stats.messages_dropped++;
        xSemaphoreGive(g_logger_mutex);
    }

    free(log_message->message);
    free(log_message);
    return ESP_ERR_TIMEOUT;
}

    // Update statistics if possible (non-blocking)
    if (xSemaphoreTake(g_logger_mutex, 0) == pdTRUE) {
        // Get current queue state for statistics
        UBaseType_t queue_messages;
        queue_messages = uxQueueMessagesWaiting(g_log_queue);

        // Update high watermark if needed
        if (queue_messages > g_logger_stats.queue_high_watermark) {
            g_logger_stats.queue_high_watermark = queue_messages;
        }

        g_logger_stats.messages_queued = queue_messages;
        xSemaphoreGive(g_logger_mutex);
    }

    return ESP_OK;
}