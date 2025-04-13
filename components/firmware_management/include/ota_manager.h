/**
 * @file ota_manager.h
 * @brief OTA Manager for BESS 100KW/200KWH firmware updates
 *
 * This module provides OTA (Over-The-Air) update capabilities for the 
 * BESS 100KW/200KWH system, supporting firmware updates via various interfaces.
 * It ensures secure, reliable updates with rollback capabilities.
 *
 * @note This component is part of the firmware_management subsystem
 */

 #ifndef OTA_MANAGER_H
 #define OTA_MANAGER_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_err.h"
 #include "esp_ota_ops.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/event_groups.h"
 #include "freertos/semphr.h"
 #include "esp_http_client.h"
 #include "esp_https_ota.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief OTA update sources
  */
 typedef enum {
     OTA_SOURCE_HTTPS,     /**< Update from HTTPS server */
     OTA_SOURCE_AWS_IOT,   /**< Update via AWS IoT */
     OTA_SOURCE_MODBUS,    /**< Update via Modbus interface */
     OTA_SOURCE_CANBUS,    /**< Update via CANBus interface */
     OTA_SOURCE_SD_CARD,   /**< Update from local SD card */
     OTA_SOURCE_USB,       /**< Update from USB drive */
 } ota_source_t;
 
 /**
  * @brief OTA update status codes
  */
 typedef enum {
     OTA_STATUS_IDLE,              /**< No update in progress */
     OTA_STATUS_CHECKING,          /**< Checking for updates */
     OTA_STATUS_UPDATE_AVAILABLE,  /**< Update is available */
     OTA_STATUS_DOWNLOADING,       /**< Downloading update */
     OTA_STATUS_VERIFYING,         /**< Verifying update integrity */
     OTA_STATUS_READY_TO_APPLY,    /**< Update ready to be applied */
     OTA_STATUS_APPLYING,          /**< Applying update */
     OTA_STATUS_COMPLETE,          /**< Update completed successfully */
     OTA_STATUS_REBOOTING,         /**< Rebooting to apply update */
     OTA_STATUS_FAILED,            /**< Update failed */
     OTA_STATUS_ABORTED,           /**< Update was aborted */
 } ota_status_t;
 
 /**
  * @brief Error codes specific to OTA operations
  */
 typedef enum {
     OTA_ERR_NONE = 0,                    /**< No error */
     OTA_ERR_NO_UPDATES = -1,             /**< No updates available */
     OTA_ERR_CONNECTIVITY = -2,           /**< Cannot connect to update server */
     OTA_ERR_SERVER = -3,                 /**< Server error */
     OTA_ERR_AUTHENTICATION = -4,         /**< Authentication failed */
     OTA_ERR_INSUFFICIENT_SPACE = -5,     /**< Not enough space for update */
     OTA_ERR_VALIDATION_FAILED = -6,      /**< Signature/checksum validation failed */
     OTA_ERR_FLASH_WRITE = -7,            /**< Error writing to flash */
     OTA_ERR_INCOMPATIBLE_VERSION = -8,   /**< Incompatible firmware version */
     OTA_ERR_BATTERY_LOW = -9,            /**< Battery too low for update */
     OTA_ERR_BMS_UNSAFE = -10,            /**< BMS reports unsafe condition */
     OTA_ERR_INTERNAL = -100,             /**< Internal error */
 } ota_error_t;
 
 /**
  * @brief OTA progress callback function type
  * 
  * @param received Number of bytes received
  * @param total Total number of bytes
  * @param user_data User-defined data passed to the callback
  */
 typedef void (*ota_progress_cb_t)(size_t received, size_t total, void *user_data);
 
 /**
  * @brief OTA event callback function type
  * 
  * @param status Current OTA status
  * @param error Error code (if status is OTA_STATUS_FAILED)
  * @param user_data User-defined data passed to the callback
  */
 typedef void (*ota_event_cb_t)(ota_status_t status, ota_error_t error, void *user_data);
 
 /**
  * @brief OTA safety check callback function type
  * 
  * This callback allows the BMS to approve or deny an OTA update
  * based on current system safety conditions.
  * 
  * @param user_data User-defined data passed to the callback
  * @return true if it's safe to proceed with OTA, false otherwise
  */
 typedef bool (*ota_safety_check_cb_t)(void *user_data);
 
 /**
  * @brief OTA configuration structure
  */
 typedef struct {
     ota_source_t source;                   /**< Update source */
     union {
         struct {
             const char *url;               /**< URL for HTTPS updates */
             const char *cert_pem;          /**< Server certificate for HTTPS (can be NULL) */
             const char *basic_auth_user;   /**< Basic auth username (can be NULL) */
             const char *basic_auth_pass;   /**< Basic auth password (can be NULL) */
         } https;
         
         struct {
             const char *topic;             /**< AWS IoT topic for updates */
         } aws_iot;
         
         struct {
             uint8_t device_id;             /**< Modbus device ID */
             uint8_t function_code;         /**< Modbus function code */
             uint16_t register_addr;        /**< Modbus register address */
         } modbus;
         
         struct {
             uint32_t can_id;               /**< CAN identifier */
             uint8_t priority;              /**< Message priority */
         } canbus;
         
         struct {
             const char *filepath;          /**< File path on SD card */
         } sd_card;
         
         struct {
             const char *device_path;       /**< USB device path */
         } usb;
     } config;                              /**< Source-specific configuration */
     
     uint32_t check_interval_ms;            /**< Interval for automatic update checks (0 = disabled) */
     bool automatic_update;                 /**< Whether to apply updates automatically */
     bool verify_signature;                 /**< Whether to verify signature (strongly recommended) */
     const char *public_key_pem;            /**< Public key for signature verification (required if verify_signature is true) */
     
     ota_safety_check_cb_t safety_check;    /**< Safety check callback (can be NULL) */
     void *safety_check_data;               /**< User data for safety check callback */
     
     ota_progress_cb_t progress_cb;         /**< Progress callback (can be NULL) */
     void *progress_cb_data;                /**< User data for progress callback */
     
     ota_event_cb_t event_cb;               /**< Event callback (can be NULL) */
     void *event_cb_data;                   /**< User data for event callback */
     
     bool allow_rollback;                   /**< Whether to enable automatic rollback on failure */
     uint32_t rollback_timeout_ms;          /**< Timeout for rollback verification (applies when allow_rollback is true) */
     
     int task_priority;                     /**< Priority for the OTA task */
     uint32_t task_stack_size;              /**< Stack size for the OTA task */
     int core_id;                           /**< Core ID for the OTA task (-1 for any core) */
 } ota_manager_config_t;
 
 /**
  * @brief OTA update information
  */
 typedef struct {
     char version[32];                      /**< Version string of the update */
     char release_notes[256];               /**< Release notes */
     size_t size;                           /**< Size in bytes */
     char md5_hash[33];                     /**< MD5 hash of the update (hex string) */
     char sha256_hash[65];                  /**< SHA256 hash of the update (hex string) */
     bool is_mandatory;                     /**< Whether this update is mandatory */
     bool requires_reboot;                  /**< Whether the update requires a reboot */
     uint32_t min_battery_level;            /**< Minimum battery level required (0-100%) */
 } ota_update_info_t;
 
 /**
  * @brief Initialize the OTA manager with the given configuration
  *
  * @param config Pointer to configuration structure
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_init(const ota_manager_config_t *config);
 
 /**
  * @brief Deinitialize the OTA manager and free resources
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_deinit(void);
 
 /**
  * @brief Start the OTA task to handle updates
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_start(void);
 
 /**
  * @brief Stop the OTA task
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_stop(void);
 
 /**
  * @brief Check for available updates (non-blocking)
  *
  * @return ESP_OK if check started successfully, error code otherwise
  */
 esp_err_t ota_manager_check_for_update(void);
 
 /**
  * @brief Check for available updates (blocking)
  *
  * @param update_info Pointer to store update information if available (can be NULL)
  * @return ESP_OK if update is available, OTA_ERR_NO_UPDATES if no update available, error code otherwise
  */
 esp_err_t ota_manager_check_for_update_blocking(ota_update_info_t *update_info);
 
 /**
  * @brief Begin downloading and applying an update (non-blocking)
  *
  * @return ESP_OK if update started successfully, error code otherwise
  */
 esp_err_t ota_manager_begin_update(void);
 
 /**
  * @brief Begin downloading and applying an update (blocking)
  *
  * @return ESP_OK if update completed successfully, error code otherwise
  */
 esp_err_t ota_manager_begin_update_blocking(void);
 
 /**
  * @brief Cancel an ongoing update
  *
  * @return ESP_OK if update was cancelled, error code otherwise
  */
 esp_err_t ota_manager_cancel_update(void);
 
 /**
  * @brief Apply a previously downloaded update and reboot
  *
  * @return ESP_OK if reboot was initiated, error code otherwise
  */
 esp_err_t ota_manager_apply_update(void);
 
 /**
  * @brief Get the current OTA status
  *
  * @param status Pointer to store the status
  * @param error Pointer to store error code if status is OTA_STATUS_FAILED (can be NULL)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_get_status(ota_status_t *status, ota_error_t *error);
 
 /**
  * @brief Check if an update is in progress
  *
  * @return true if update is in progress, false otherwise
  */
 bool ota_manager_is_updating(void);
 
 /**
  * @brief Get detailed error message for the specified error code
  *
  * @param error The error code
  * @return String description of the error
  */
 const char *ota_manager_error_to_string(ota_error_t error);
 
 /**
  * @brief Configure logging options for OTA operations
  *
  * @param log_to_console Enable/disable console logging
  * @param log_to_sd Enable/disable SD card logging
  * @param log_to_cloudwatch Enable/disable AWS CloudWatch logging
  * @param log_level Minimum log level to record (uses ESP log levels)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_config_logging(bool log_to_console, bool log_to_sd, 
                                    bool log_to_cloudwatch, esp_log_level_t log_level);
 
 /**
  * @brief Mark the current firmware as valid
  * 
  * This should be called after successful validation of a new firmware
  * to prevent automatic rollback.
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_mark_valid(void);
 
 /**
  * @brief Get the running firmware version
  *
  * @param version Buffer to store version string
  * @param max_len Maximum length of the buffer
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_get_running_version(char *version, size_t max_len);
 
 /**
  * @brief Get the latest firmware version available (if known)
  *
  * @param version Buffer to store version string
  * @param max_len Maximum length of the buffer
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_get_latest_version(char *version, size_t max_len);
 
 /**
  * @brief Configure rollback settings
  *
  * @param allow_rollback Enable/disable automatic rollback
  * @param timeout_ms Timeout for rollback verification
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_config_rollback(bool allow_rollback, uint32_t timeout_ms);
 
 /**
  * @brief Request an immediate rollback to the previous firmware
  *
  * @return ESP_OK if rollback was initiated, error code otherwise
  */
 esp_err_t ota_manager_rollback(void);
 
 /**
  * @brief Check if rollback is available
  *
  * @return true if rollback is available, false otherwise
  */
 bool ota_manager_can_rollback(void);
 
 /**
  * @brief Get information about the update partition
  *
  * @param info Pointer to store update partition information
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_get_update_partition_info(esp_ota_img_states_t *state, 
                                               esp_partition_info_t *info);
 
 /**
  * @brief Report diagnostics to AWS CloudWatch
  *
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_report_diagnostics(void);
 
 /**
  * @brief Set BMS safety thresholds for OTA updates
  * 
  * This function sets safety thresholds that must be met for OTA updates
  * to proceed, based on battery system parameters.
  *
  * @param min_soc Minimum state of charge percentage (0-100)
  * @param max_temperature Maximum allowed temperature (°C)
  * @param min_temperature Minimum allowed temperature (°C)
  * @param max_current Maximum allowed current during update (A)
  * @return ESP_OK on success, error code otherwise
  */
 esp_err_t ota_manager_set_bms_safety_thresholds(uint8_t min_soc, float max_temperature,
                                              float min_temperature, float max_current);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* OTA_MANAGER_H */