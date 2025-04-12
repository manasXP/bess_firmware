/**
 * @file sd_logger.c
 * @brief Implementation of SD Card logging component for BESS 100KW/200KWH system
 * 
 * This module provides functions for logging data to an SD card in the 
 * Battery Energy Storage System. It's part of the data_logger subsystem
 * that also includes console and cloud logging capabilities.
 * 
 * @note This component is designed for the ESP32-P4 platform running FreeRTOS
 *
 * @copyright (c) 2025 BESS Systems
 */

 #include "sd_logger.h"
 #include <stdio.h>
 #include <string.h>
 #include <stdarg.h>
 #include <sys/stat.h>
 #include <dirent.h>
 #include <unistd.h>
 #include "esp_vfs_fat.h"
 #include "sdmmc_cmd.h"
 #include "driver/sdspi_host.h"
 #include "driver/gpio.h"
 #include "esp_check.h"
 #include "esp_system.h"
 #include "esp_vfs.h"
 #include "esp_timer.h"
 
 // Tag for ESP log messages
 static const char* TAG = "sd_logger";
 
 // SD card configuration
 #define PIN_NUM_MISO        CONFIG_BESS_SDMMC_MISO_GPIO
 #define PIN_NUM_MOSI        CONFIG_BESS_SDMMC_MOSI_GPIO
 #define PIN_NUM_CLK         CONFIG_BESS_SDMMC_CLK_GPIO
 #define PIN_NUM_CS          CONFIG_BESS_SDMMC_CS_GPIO
 
 // SD card SPI configuration
 #define SD_SPI_HOST         SPI2_HOST
 #define SD_SPI_DMA_CHAN     SPI_DMA_CH_AUTO
 
 // Max path and filename lengths
 #define MAX_PATH_LENGTH     128
 #define MAX_FILENAME_LENGTH 64
 
 // Error retry delays
 #define RETRY_DELAY_MS      100
 
 // Internal state structure
 typedef struct {
     sd_logger_config_t config;
     bool initialized;
     bool mounted;
     SemaphoreHandle_t mutex;
     QueueHandle_t message_queue;
     TaskHandle_t logger_task;
     FILE* log_file;
     char current_filename[MAX_FILENAME_LENGTH];
     uint32_t bytes_written;
     uint32_t messages_logged;
     uint32_t failed_writes;
     uint32_t queue_highwater;
     uint8_t current_file_index;
     time_t start_time;
     sdmmc_card_t* card;
     sd_logger_event_cb_t event_callback;
     void* event_cb_arg;
     esp_timer_handle_t health_check_timer;
 } sd_logger_t;
 
 // Global logger instance
 static sd_logger_t s_logger = {0};
 
 // Forward declarations of internal functions
 static void sd_logger_task(void* arg);
 static esp_err_t create_log_file(void);
 static esp_err_t close_log_file(void);
 static esp_err_t sd_card_mount(void);
 static esp_err_t sd_card_unmount(void);
 static esp_err_t check_and_create_dir(const char* path);
 static esp_err_t cleanup_old_logs(void);
 static esp_err_t format_log_message(sd_log_message_t* message, char* buffer, size_t buffer_size);
 static esp_err_t write_to_file(const char* data, size_t length);
 static void sd_health_check_timer_cb(void* arg);
 static void notify_event(esp_err_t event_code);
 
 /**
  * @brief Initialize the SD card logger
  */
 esp_err_t sd_logger_init(const sd_logger_config_t *config) {
     esp_err_t ret = ESP_OK;
 
     // Check if already initialized
     if (s_logger.initialized) {
         ESP_LOGW(TAG, "SD logger already initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     // Validate parameters
     if (config == NULL) {
         ESP_LOGE(TAG, "NULL config provided");
         return ESP_ERR_INVALID_ARG;
     }
 
     // Check if the mount point is valid
     if (config->mount_point == NULL || strlen(config->mount_point) == 0) {
         ESP_LOGE(TAG, "Invalid mount point");
         return ESP_ERR_INVALID_ARG;
     }
 
     // Initialize state
     memset(&s_logger, 0, sizeof(sd_logger_t));
     memcpy(&s_logger.config, config, sizeof(sd_logger_config_t));
     s_logger.start_time = time(NULL);
 
     // Create mutex
     s_logger.mutex = xSemaphoreCreateMutex();
     if (s_logger.mutex == NULL) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return ESP_ERR_NO_MEM;
     }
 
     // Create message queue
     s_logger.message_queue = xQueueCreate(config->queue_size, sizeof(sd_log_message_t));
     if (s_logger.message_queue == NULL) {
         vSemaphoreDelete(s_logger.mutex);
         ESP_LOGE(TAG, "Failed to create message queue");
         return ESP_ERR_NO_MEM;
     }
 
     // Mount SD card
     ret = sd_card_mount();
     if (ret != ESP_OK) {
         vQueueDelete(s_logger.message_queue);
         vSemaphoreDelete(s_logger.mutex);
         ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
         return ret;
     }
 
     // Create required directories
     ret = check_and_create_dir(config->base_path);
     if (ret != ESP_OK) {
         sd_card_unmount();
         vQueueDelete(s_logger.message_queue);
         vSemaphoreDelete(s_logger.mutex);
         ESP_LOGE(TAG, "Failed to create log directory: %s", esp_err_to_name(ret));
         return ret;
     }
 
     // Clean up old logs to prevent storage overflow
     cleanup_old_logs();
 
     // Create initial log file
     ret = create_log_file();
     if (ret != ESP_OK) {
         sd_card_unmount();
         vQueueDelete(s_logger.message_queue);
         vSemaphoreDelete(s_logger.mutex);
         ESP_LOGE(TAG, "Failed to create log file: %s", esp_err_to_name(ret));
         return ret;
     }
 
     // Set up SD card health check timer if enabled
     if (config->enable_sd_checks) {
         const esp_timer_create_args_t timer_args = {
             .callback = &sd_health_check_timer_cb,
             .name = "sd_health_timer"
         };
         ret = esp_timer_create(&timer_args, &s_logger.health_check_timer);
         if (ret != ESP_OK) {
             ESP_LOGW(TAG, "Failed to create health check timer: %s", esp_err_to_name(ret));
             // Continue anyway, health checks are optional
         } else {
             // Start the timer with the configured interval
             esp_timer_start_periodic(s_logger.health_check_timer, 
                                     config->sd_check_interval_ms * 1000); // Convert to microseconds
         }
     }
 
     // Create logger task
     BaseType_t task_created = xTaskCreate(
         sd_logger_task,
         "sd_logger_task",
         config->task_stack_size,
         NULL,
         config->task_priority,
         &s_logger.logger_task
     );
 
     if (task_created != pdPASS) {
         if (s_logger.health_check_timer) {
             esp_timer_delete(s_logger.health_check_timer);
         }
         close_log_file();
         sd_card_unmount();
         vQueueDelete(s_logger.message_queue);
         vSemaphoreDelete(s_logger.mutex);
         ESP_LOGE(TAG, "Failed to create logger task");
         return ESP_ERR_NO_MEM;
     }
 
     // Mark as initialized
     s_logger.initialized = true;
 
     ESP_LOGI(TAG, "SD Logger initialized successfully");
     
     // Log system startup
     sd_logger_write(SD_LOG_INFO, TAG, "SD Logger started. BESS 100KW/200KWH System");
 
     return ESP_OK;
 }
 
 /**
  * @brief Close the current log file
  * 
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t close_log_file(void) {
     if (s_logger.log_file != NULL) {
         // Write a closing footer
         char footer[128];
         time_t now = time(NULL);
         struct tm timeinfo;
         localtime_r(&now, &timeinfo);
         
         char timestamp[32];
         strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
         
         snprintf(footer, sizeof(footer),
                 "\n------------- LOG FILE CLOSED -------------\n"
                 "Closed at: %s\n"
                 "Messages logged: %u\n"
                 "Bytes written: %u\n"
                 "-------------------------------------------\n",
                 timestamp, s_logger.messages_logged, s_logger.bytes_written);
         
         fwrite(footer, 1, strlen(footer), s_logger.log_file);
         fflush(s_logger.log_file);
         fsync(fileno(s_logger.log_file));
         fclose(s_logger.log_file);
         s_logger.log_file = NULL;
         
         ESP_LOGI(TAG, "Closed log file");
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Mount the SD card
  * 
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t sd_card_mount(void) {
     if (s_logger.mounted) {
         return ESP_OK; // Already mounted
     }
     
     esp_err_t ret = ESP_OK;
     
     // SD card mount point configuration
     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
         .format_if_mount_failed = false,  // Don't format if mount fails
         .max_files = 5,                   // Max open files
         .allocation_unit_size = 16 * 1024 // Allocation unit size
     };
     
     ESP_LOGI(TAG, "Mounting SD card...");
     
     // Configure SPI bus for SD card
     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
     host.slot = SD_SPI_HOST;
     
     spi_bus_config_t bus_cfg = {
         .mosi_io_num = PIN_NUM_MOSI,
         .miso_io_num = PIN_NUM_MISO,
         .sclk_io_num = PIN_NUM_CLK,
         .quadwp_io_num = -1,
         .quadhd_io_num = -1,
         .max_transfer_sz = 4000,
     };
     
     ret = spi_bus_initialize(host.slot, &bus_cfg, SD_SPI_DMA_CHAN);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
         notify_event(ret);
         return ret;
     }
     
     // Configure SPI device for SD card
     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
     slot_config.gpio_cs = PIN_NUM_CS;
     slot_config.host_id = host.slot;
     
     // Try to mount the SD card
     ret = esp_vfs_fat_sdspi_mount(s_logger.config.mount_point, &host, &slot_config, 
                                  &mount_config, &s_logger.card);
     
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
         spi_bus_free(host.slot);
         notify_event(ret);
         return ret;
     }
     
     // Get and log card info
     sdmmc_card_print_info(stdout, s_logger.card);
     
     s_logger.mounted = true;
     ESP_LOGI(TAG, "SD card mounted successfully at %s", s_logger.config.mount_point);
     
     return ESP_OK;
 }
 
 /**
  * @brief Unmount the SD card
  * 
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t sd_card_unmount(void) {
     if (!s_logger.mounted) {
         return ESP_OK; // Already unmounted
     }
     
     ESP_LOGI(TAG, "Unmounting SD card...");
     
     // Close any open log file
     close_log_file();
     
     // Unmount the card
     esp_err_t ret = esp_vfs_fat_sdcard_unmount(s_logger.config.mount_point, s_logger.card);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
         notify_event(ret);
         return ret;
     }
     
     // Free SPI bus
     spi_bus_free(SD_SPI_HOST);
     
     s_logger.card = NULL;
     s_logger.mounted = false;
     
     ESP_LOGI(TAG, "SD card unmounted");
     
     return ESP_OK;
 }
 
 /**
  * @brief Check if directory exists and create it if it doesn't
  * 
  * @param path Directory path to check/create
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t check_and_create_dir(const char* path) {
     if (path == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Check if the directory exists
     struct stat st;
     if (stat(path, &st) == 0) {
         if (S_ISDIR(st.st_mode)) {
             return ESP_OK; // Directory exists
         } else {
             ESP_LOGE(TAG, "Path exists but is not a directory: %s", path);
             return ESP_FAIL;
         }
     }
     
     // Directory doesn't exist, create it
     ESP_LOGI(TAG, "Creating directory: %s", path);
     
     // Check if we need to create parent directories
     char parent_path[MAX_PATH_LENGTH] = {0};
     const char* slash = strrchr(path, '/');
     
     if (slash && slash != path) {
         // Copy parent path
         size_t parent_len = slash - path;
         if (parent_len < sizeof(parent_path)) {
             memcpy(parent_path, path, parent_len);
             parent_path[parent_len] = '\0';
             
             // Recursively create parent directory
             esp_err_t ret = check_and_create_dir(parent_path);
             if (ret != ESP_OK) {
                 return ret;
             }
         }
     }
     
     // Create the directory with read/write/search permissions
     if (mkdir(path, 0755) != 0) {
         ESP_LOGE(TAG, "Failed to create directory: %s (errno: %d)", path, errno);
         return ESP_FAIL;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Clean up old log files to prevent storage overflow
  * 
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t cleanup_old_logs(void) {
     if (!s_logger.mounted) {
         return ESP_ERR_INVALID_STATE;
     }
     
     ESP_LOGI(TAG, "Checking for old log files to clean up");
     
     // Open the log directory
     DIR* dir = opendir(s_logger.config.base_path);
     if (!dir) {
         ESP_LOGW(TAG, "Could not open log directory: %s", s_logger.config.base_path);
         return ESP_ERR_NOT_FOUND;
     }
     
     // Count log files and find the oldest ones
     typedef struct {
         char name[MAX_FILENAME_LENGTH];
         time_t timestamp;
     } log_file_info_t;
     
     log_file_info_t* files = NULL;
     int file_count = 0;
     
     // First, count the log files
     struct dirent* entry;
     while ((entry = readdir(dir)) != NULL) {
         // Check if this is a log file (by prefix and extension)
         if (strncmp(entry->d_name, s_logger.config.filename_prefix, 
                    strlen(s_logger.config.filename_prefix)) == 0 &&
             strstr(entry->d_name, ".log") != NULL) {
             file_count++;
         }
     }
     
     // If we have more files than the maximum allowed, process them
     if (file_count > s_logger.config.max_files) {
         rewinddir(dir);
         
         // Allocate memory for file info
         files = (log_file_info_t*)malloc(file_count * sizeof(log_file_info_t));
         if (!files) {
             closedir(dir);
             ESP_LOGE(TAG, "Failed to allocate memory for file cleanup");
             return ESP_ERR_NO_MEM;
         }
         
         // Collect file information
         int idx = 0;
         while ((entry = readdir(dir)) != NULL && idx < file_count) {
             // Check if this is a log file (by prefix and extension)
             if (strncmp(entry->d_name, s_logger.config.filename_prefix, 
                        strlen(s_logger.config.filename_prefix)) == 0 &&
                 strstr(entry->d_name, ".log") != NULL) {
                 
                 // Get file stats to find creation time
                 char full_path[MAX_PATH_LENGTH];
                 snprintf(full_path, sizeof(full_path), "%s/%s", 
                         s_logger.config.base_path, entry->d_name);
                 
                 struct stat st;
                 if (stat(full_path, &st) == 0) {
                     // Store file info
                     strncpy(files[idx].name, entry->d_name, sizeof(files[idx].name) - 1);
                     files[idx].name[sizeof(files[idx].name) - 1] = '\0';
                     files[idx].timestamp = st.st_mtime;
                     idx++;
                 }
             }
         }
         
         // Sort files by timestamp (oldest first)
         for (int i = 0; i < idx - 1; i++) {
             for (int j = 0; j < idx - i - 1; j++) {
                 if (files[j].timestamp > files[j + 1].timestamp) {
                     // Swap
                     log_file_info_t temp = files[j];
                     files[j] = files[j + 1];
                     files[j + 1] = temp;
                 }
             }
         }
         
         // Delete oldest files to get down to max_files
         int files_to_delete = idx - s_logger.config.max_files;
         for (int i = 0; i < files_to_delete; i++) {
             char full_path[MAX_PATH_LENGTH];
             snprintf(full_path, sizeof(full_path), "%s/%s", 
                     s_logger.config.base_path, files[i].name);
             
             ESP_LOGI(TAG, "Deleting old log file: %s", files[i].name);
             if (unlink(full_path) != 0) {
                 ESP_LOGW(TAG, "Failed to delete file: %s (errno: %d)", full_path, errno);
             }
         }
         
         free(files);
     }
     
     closedir(dir);
     return ESP_OK;
 }
 
 /**
  * @brief Format a log message for writing to the file
  * 
  * @param message Pointer to the log message structure
  * @param buffer Buffer to store the formatted message
  * @param buffer_size Size of the buffer
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t format_log_message(sd_log_message_t* message, char* buffer, size_t buffer_size) {
     if (message == NULL || buffer == NULL || buffer_size == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     // Format timestamp if enabled
     char timestamp[32] = "";
     if (s_logger.config.include_timestamp && message->timestamp > 0) {
         struct tm timeinfo;
         localtime_r(&message->timestamp, &timeinfo);
         strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
     }
     
     // Get level string
     const char* level_str = "NONE";
     switch (message->level) {
         case SD_LOG_ERROR:
             level_str = "ERROR";
             break;
         case SD_LOG_WARN:
             level_str = "WARN";
             break;
         case SD_LOG_INFO:
             level_str = "INFO";
             break;
         case SD_LOG_DEBUG:
             level_str = "DEBUG";
             break;
         case SD_LOG_VERBOSE:
             level_str = "VERBOSE";
             break;
         default:
             level_str = "UNKNOWN";
             break;
     }
     
     // Format the message
     int written;
     if (s_logger.config.include_timestamp) {
         written = snprintf(buffer, buffer_size, "[%s] %5s (%s): %s\n", 
                           timestamp, level_str, message->tag, message->message);
     } else {
         written = snprintf(buffer, buffer_size, "%5s (%s): %s\n", 
                           level_str, message->tag, message->message);
     }
     
     if (written < 0 || written >= buffer_size) {
         return ESP_ERR_BUFFER_OVERFLOW;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Write data to the log file
  * 
  * @param data Pointer to the data to write
  * @param length Length of the data in bytes
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t write_to_file(const char* data, size_t length) {
     if (data == NULL || length == 0) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (s_logger.log_file == NULL) {
         ESP_LOGW(TAG, "No open log file for writing");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Try to write the data
     size_t written = 0;
     uint8_t retry_count = 0;
     
     while (written < length && retry_count < s_logger.config.max_write_retry) {
         size_t result = fwrite(data + written, 1, length - written, s_logger.log_file);
         
         if (result > 0) {
             written += result;
         } else {
             // Check for errors
             if (ferror(s_logger.log_file)) {
                 ESP_LOGW(TAG, "Error writing to log file (retry %d): %d", 
                         retry_count, ferror(s_logger.log_file));
                 clearerr(s_logger.log_file);
                 
                 // Delay before retry
                 vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                 retry_count++;
             } else {
                 // No error but no data written - should not happen
                 break;
             }
         }
     }
     
     // Update bytes written counter
     s_logger.bytes_written += written;
     
     // Check if we wrote everything
     if (written < length) {
         ESP_LOGE(TAG, "Failed to write all data to log file after %d retries", retry_count);
         return ESP_FAIL;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief SD card health check timer callback
  * 
  * @param arg Unused timer argument
  */
 static void sd_health_check_timer_cb(void* arg) {
     // Only perform health check if mounted and initialized
     if (!s_logger.initialized || !s_logger.mounted) {
         return;
     }
     
     // Try to acquire the mutex with a short timeout
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         uint8_t health = 0;
         
         // Perform the health check
         esp_err_t ret = sd_logger_check_health(&health);
         
         // Log the health status periodically
         ESP_LOGI(TAG, "SD card health check: %d%% (status: %s)", 
                 health, (ret == ESP_OK) ? "OK" : "FAIL");
         
         // If health is critical, notify the system
         if (health < 50) {
             ESP_LOGW(TAG, "SD card health is critical: %d%%", health);
             notify_event(ESP_ERR_FLASH_OP_FAIL);
         }
         
         xSemaphoreGive(s_logger.mutex);
     }
 }
 
 /**
  * @brief Send an event notification to the registered callback
  * 
  * @param event_code ESP error code representing the event
  */
 static void notify_event(esp_err_t event_code) {
     if (s_logger.event_callback != NULL) {
         s_logger.event_callback(s_logger.event_cb_arg, event_code);
     }
 }
 
 /**
  * @brief Deinitialize the SD card logger
  */
 esp_err_t sd_logger_deinit(void) {
     esp_err_t ret = ESP_OK;
 
     if (!s_logger.initialized) {
         ESP_LOGW(TAG, "SD logger not initialized");
         return ESP_ERR_INVALID_STATE;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGE(TAG, "Failed to acquire mutex for deinit");
         return ESP_ERR_TIMEOUT;
     }
 
     // Stop the health check timer if it exists
     if (s_logger.health_check_timer) {
         esp_timer_stop(s_logger.health_check_timer);
         esp_timer_delete(s_logger.health_check_timer);
         s_logger.health_check_timer = NULL;
     }
 
     // Log shutdown
     sd_logger_write(SD_LOG_INFO, TAG, "SD Logger shutting down");
     sd_logger_flush();
 
     // Delete the task. This is a bit tricky since we're in the task.
     // We'll set a flag and release the mutex, then delete the task from outside.
     TaskHandle_t task_to_delete = s_logger.logger_task;
     s_logger.logger_task = NULL;
     s_logger.initialized = false;
 
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
 
     // Delete task if it's not the current task
     if (task_to_delete != NULL && task_to_delete != xTaskGetCurrentTaskHandle()) {
         vTaskDelete(task_to_delete);
     }
 
     // Close the log file
     close_log_file();
 
     // Unmount SD card
     sd_card_unmount();
 
     // Clean up resources
     vQueueDelete(s_logger.message_queue);
     vSemaphoreDelete(s_logger.mutex);
 
     ESP_LOGI(TAG, "SD Logger deinitialized");
 
     // If we're in the logger task, delete ourselves
     if (task_to_delete == xTaskGetCurrentTaskHandle()) {
         vTaskDelete(NULL);
     }
 
     return ESP_OK;
 }
 
 /**
  * @brief Write a log message to the SD card
  */
 esp_err_t sd_logger_write(sd_log_level_t level, const char *tag, const char *format, ...) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Check log level
     if (level == SD_LOG_NONE || level > s_logger.config.log_level) {
         return ESP_OK; // Silently ignore messages below the configured level
     }
 
     // Check tag
     if (tag == NULL || format == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Format the message
     va_list args;
     va_start(args, format);
     
     sd_log_message_t message = {0};
     message.timestamp = time(NULL);
     message.level = level;
     
     // Copy tag (truncating if necessary)
     strncpy(message.tag, tag, sizeof(message.tag) - 1);
     message.tag[sizeof(message.tag) - 1] = '\0';
     
     // Format the message (truncating if necessary)
     vsnprintf(message.message, sizeof(message.message), format, args);
     message.message[sizeof(message.message) - 1] = '\0';
     
     va_end(args);
 
     // Send to queue
     if (xQueueSend(s_logger.message_queue, &message, pdMS_TO_TICKS(100)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to queue log message, queue full");
         return ESP_ERR_TIMEOUT;
     }
 
     return ESP_OK;
 }
 
 /**
  * @brief Write a log message with a binary blob
  */
 esp_err_t sd_logger_write_binary(sd_log_level_t level, const char *tag, 
                                const char *message, const void *data, size_t data_len) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Check log level
     if (level == SD_LOG_NONE || level > s_logger.config.log_level) {
         return ESP_OK; // Silently ignore messages below the configured level
     }
 
     // Check parameters
     if (tag == NULL || message == NULL || (data == NULL && data_len > 0)) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Basic text part of the message
     esp_err_t ret = sd_logger_write(level, tag, "%s", message);
     if (ret != ESP_OK) {
         return ret;
     }
 
     // If there's binary data to write
     if (data != NULL && data_len > 0) {
         // Acquire the mutex
         if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
             ESP_LOGW(TAG, "Failed to acquire mutex for binary write");
             return ESP_ERR_TIMEOUT;
         }
 
         // We'll write a hexdump format for binary data
         const uint8_t* bytes = (const uint8_t*)data;
         const size_t BYTES_PER_LINE = 16;
         char line[80]; // Line buffer
 
         for (size_t i = 0; i < data_len; i += BYTES_PER_LINE) {
             // Format address
             int offset = snprintf(line, sizeof(line), "  %04x: ", (unsigned int)i);
             
             // Format hex bytes
             for (size_t j = 0; j < BYTES_PER_LINE; j++) {
                 if (i + j < data_len) {
                     offset += snprintf(line + offset, sizeof(line) - offset, 
                                       "%02x ", bytes[i + j]);
                 } else {
                     offset += snprintf(line + offset, sizeof(line) - offset, "   ");
                 }
                 
                 // Extra space after 8 bytes
                 if (j == 7) {
                     offset += snprintf(line + offset, sizeof(line) - offset, " ");
                 }
             }
             
             // Format ASCII representation
             offset += snprintf(line + offset, sizeof(line) - offset, " |");
             for (size_t j = 0; j < BYTES_PER_LINE; j++) {
                 if (i + j < data_len) {
                     uint8_t c = bytes[i + j];
                     // Only print printable ASCII characters
                     if (c >= 32 && c <= 126) {
                         offset += snprintf(line + offset, sizeof(line) - offset, "%c", c);
                     } else {
                         offset += snprintf(line + offset, sizeof(line) - offset, ".");
                     }
                 } else {
                     offset += snprintf(line + offset, sizeof(line) - offset, " ");
                 }
             }
             offset += snprintf(line + offset, sizeof(line) - offset, "|\n");
             
             // Write the line to the file
             write_to_file(line, strlen(line));
         }
 
         // Release the mutex
         xSemaphoreGive(s_logger.mutex);
     }
 
     return ESP_OK;
 }
 
 /**
  * @brief Set the minimum log level
  */
 esp_err_t sd_logger_set_level(sd_log_level_t level) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (level > SD_LOG_VERBOSE) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for setting log level");
         return ESP_ERR_TIMEOUT;
     }
 
     s_logger.config.log_level = level;
     
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Get the current log level
  */
 esp_err_t sd_logger_get_level(sd_log_level_t *level) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (level == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for getting log level");
         return ESP_ERR_TIMEOUT;
     }
 
     *level = s_logger.config.log_level;
     
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Flush all pending log data to the SD card
  */
 esp_err_t sd_logger_flush(void) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for flush");
         return ESP_ERR_TIMEOUT;
     }
 
     if (s_logger.log_file != NULL) {
         fflush(s_logger.log_file);
         fsync(fileno(s_logger.log_file));
     }
 
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
 
     return ESP_OK;
 }
 
 /**
  * @brief Get the status of the SD logger
  */
 esp_err_t sd_logger_get_status(sd_logger_status_t *status) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (status == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for getting status");
         return ESP_ERR_TIMEOUT;
     }
 
     // Fill in the status structure
     status->is_initialized = s_logger.initialized;
     status->is_mounted = s_logger.mounted;
     status->bytes_written = s_logger.bytes_written;
     status->messages_logged = s_logger.messages_logged;
     status->failed_writes = s_logger.failed_writes;
     status->queue_highwater = s_logger.queue_highwater;
     status->current_file_index = s_logger.current_file_index;
     status->start_time = s_logger.start_time;
     
     // Get free space info
     if (s_logger.mounted) {
         struct statvfs stat;
         if (statvfs(s_logger.config.mount_point, &stat) == 0) {
             status->free_space_bytes = (uint64_t)stat.f_bsize * stat.f_bfree;
         } else {
             status->free_space_bytes = 0;
         }
     } else {
         status->free_space_bytes = 0;
     }
     
     // Get health info
     uint8_t health = 0;
     sd_logger_check_health(&health);
     status->card_health = health;
     
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Check the health of the SD card
  */
 esp_err_t sd_logger_check_health(uint8_t *health_percent) {
     if (!s_logger.initialized || !s_logger.mounted) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (health_percent == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for health check");
         return ESP_ERR_TIMEOUT;
     }
 
     // For now, we'll use a simple check based on write failures and card info
     // In a real implementation, more sophisticated card health checks should be performed
     
     // Base health on failed writes rate (if any writes have been attempted)
     uint8_t health = 100;
     if (s_logger.messages_logged > 0) {
         uint32_t failure_rate = (s_logger.failed_writes * 100) / s_logger.messages_logged;
         if (failure_rate > 0) {
             health -= failure_rate * 2; // Each percent of failures reduces health by 2%
         }
     }
     
     // Check if card is still present and working
     if (s_logger.card == NULL) {
         health = 0;
     }
     
     // Ensure the health value is within range
     if (health > 100) {
         health = 100;
     }
     
     *health_percent = health;
     
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Start a new log file
  */
 esp_err_t sd_logger_rotate_file(void) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for file rotation");
         return ESP_ERR_TIMEOUT;
     }
 
     // Close current file
     esp_err_t ret = close_log_file();
     if (ret != ESP_OK) {
         xSemaphoreGive(s_logger.mutex);
         return ret;
     }
 
     // Create new file
     ret = create_log_file();
     if (ret != ESP_OK) {
         xSemaphoreGive(s_logger.mutex);
         return ret;
     }
 
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Register a callback for SD card events
  */
 esp_err_t sd_logger_register_event_handler(sd_logger_event_cb_t callback, void *arg) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     if (callback == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     // Acquire the mutex
     if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         ESP_LOGW(TAG, "Failed to acquire mutex for event handler registration");
         return ESP_ERR_TIMEOUT;
     }
 
     s_logger.event_callback = callback;
     s_logger.event_cb_arg = arg;
     
     // Release the mutex
     xSemaphoreGive(s_logger.mutex);
     
     return ESP_OK;
 }
 
 /**
  * @brief Log battery-specific events to SD card
  */
 esp_err_t sd_logger_battery_event(uint8_t module_id, 
                                uint32_t event_type, 
                                const void *event_data,
                                size_t event_data_size) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Prefix for battery event logs
     char prefix[64];
     snprintf(prefix, sizeof(prefix), "Battery Event - Module %u, Type 0x%08x:", 
              module_id, event_type);
 
     // Log the event description
     sd_logger_write(SD_LOG_INFO, "BATT_EVT", "%s", prefix);
     
     // If we have event data, also log it as binary
     if (event_data != NULL && event_data_size > 0) {
         sd_logger_write_binary(SD_LOG_INFO, "BATT_EVT", prefix, event_data, event_data_size);
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Create a CSV-formatted log entry for battery telemetry
  */
 esp_err_t sd_logger_battery_telemetry(uint8_t module_id,
                                    uint32_t voltage,
                                    int32_t current,
                                    float temperature,
                                    float soc,
                                    float soh) {
     if (!s_logger.initialized) {
         return ESP_ERR_INVALID_STATE;
     }
 
     // Create CSV-formatted telemetry entry
     time_t now = time(NULL);
     struct tm timeinfo;
     localtime_r(&now, &timeinfo);
     
     char time_str[20];
     strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
     
     // Format as CSV: timestamp,module_id,voltage(mV),current(mA),temp(C),soc(%),soh(%)
     sd_logger_write(SD_LOG_INFO, "BATT_TLM", 
                   "%s,%u,%u,%d,%.2f,%.2f,%.2f", 
                   time_str, module_id, voltage, current, temperature, soc, soh);
     
     return ESP_OK;
 }
 
 /**
  * @brief Main logger task
  * 
  * FreeRTOS task that processes log messages from the queue and writes them to the SD card.
  * Also handles file rotation when the current log file reaches the maximum size.
  * 
  * @param arg Unused task argument
  */
 static void sd_logger_task(void* arg) {
     sd_log_message_t message;
     char buffer[512]; // Buffer for formatted message
     UBaseType_t queue_items;
     const TickType_t check_interval = pdMS_TO_TICKS(1000); // Check file state every second
     TickType_t last_check = xTaskGetTickCount();
     
     ESP_LOGI(TAG, "SD logger task started");
     
     while (s_logger.initialized) {
         // Process messages from the queue
         while (xQueueReceive(s_logger.message_queue, &message, 0) == pdTRUE) {
             // Format the message
             if (format_log_message(&message, buffer, sizeof(buffer)) == ESP_OK) {
                 // Acquire the mutex
                 if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                     // Write to file
                     if (write_to_file(buffer, strlen(buffer)) == ESP_OK) {
                         s_logger.messages_logged++;
                     } else {
                         s_logger.failed_writes++;
                     }
                     
                     // Auto-flush if enabled
                     if (s_logger.config.auto_flush && s_logger.log_file != NULL) {
                         fflush(s_logger.log_file);
                     }
                     
                     // Update queue highwater mark
                     queue_items = uxQueueMessagesWaiting(s_logger.message_queue);
                     if (queue_items > s_logger.queue_highwater) {
                         s_logger.queue_highwater = queue_items;
                     }
                     
                     // Release the mutex
                     xSemaphoreGive(s_logger.mutex);
                 } else {
                     ESP_LOGW(TAG, "Failed to acquire mutex for message write");
                     s_logger.failed_writes++;
                 }
             }
         }
         
         // Periodic file checks
         TickType_t now = xTaskGetTickCount();
         if ((now - last_check) >= check_interval) {
             last_check = now;
             
             if (xSemaphoreTake(s_logger.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                 // Check if current file is too large
                 if (s_logger.log_file != NULL) {
                     long file_size = ftell(s_logger.log_file);
                     if (file_size >= (s_logger.config.max_file_size_kb * 1024)) {
                         ESP_LOGI(TAG, "Log file reached max size, rotating");
                         close_log_file();
                         create_log_file();
                     }
                 } else {
                     // Log file is not open - try to create it
                     if (!s_logger.mounted) {
                         sd_card_mount();
                     }
                     
                     if (s_logger.mounted && s_logger.log_file == NULL) {
                         create_log_file();
                     }
                 }
                 
                 xSemaphoreGive(s_logger.mutex);
             }
         }
         
         // Small delay to prevent task from hogging CPU
         vTaskDelay(pdMS_TO_TICKS(20));
     }
     
     ESP_LOGI(TAG, "SD logger task ending");
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Create a new log file
  * 
  * Creates a new log file with a timestamp and index in the filename.
  * Writes a header with system information to the beginning of the file.
  * 
  * @return ESP_OK on success or an error code on failure
  */
 static esp_err_t create_log_file(void) {
     if (!s_logger.mounted) {
         return ESP_ERR_INVALID_STATE;
     }
     
     // Close any existing file
     if (s_logger.log_file != NULL) {
         fclose(s_logger.log_file);
         s_logger.log_file = NULL;
     }
     
     // Increment the file index
     s_logger.current_file_index = (s_logger.current_file_index + 1) % s_logger.config.max_files;
     
     // Generate filename with timestamp
     time_t now = time(NULL);
     struct tm timeinfo;
     localtime_r(&now, &timeinfo);
     
     char timestamp[32];
     strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
     
     // Create the filename
     char filename[MAX_PATH_LENGTH];
     snprintf(filename, sizeof(filename), "%s/%s_%s_%d.log", 
              s_logger.config.base_path, s_logger.config.filename_prefix,
              timestamp, s_logger.current_file_index);
     
     // Store the current filename
     strncpy(s_logger.current_filename, filename, sizeof(s_logger.current_filename) - 1);
     s_logger.current_filename[sizeof(s_logger.current_filename) - 1] = '\0';
     
     // Open the file
     s_logger.log_file = fopen(filename, "a+");
     if (s_logger.log_file == NULL) {
         ESP_LOGE(TAG, "Failed to open log file: %s", filename);
         return ESP_FAIL;
     }
     
     // Write file header
     char header[256];
     snprintf(header, sizeof(header),
              "------------- BESS LOG FILE -------------\n"
              "System: BESS 100KW/200KWH\n"
              "File created: %s\n"
              "Firmware version: %s\n"
              "-------------------------------------------\n\n",
              timestamp, CONFIG_BESS_FIRMWARE_VERSION);
     
     if (fwrite(header, 1, strlen(header), s_logger.log_file) != strlen(header)) {
         ESP_LOGE(TAG, "Failed to write file header");
         fclose(s_logger.log_file);
         s_logger.log_file = NULL;
         return ESP_FAIL;
     }
     
     s_logger.bytes_written += strlen(header);
     fflush(s_logger.log_file);
     
     ESP_LOGI(TAG, "Created new log file: %s", filename);
     
     return ESP_OK;
 }