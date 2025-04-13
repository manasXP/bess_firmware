/**
 * @file ota_manager.c
 * @brief Implementation of the OTA Manager for BESS 100KW/200KWH firmware updates
 */

#include <string.h>
#include "ota_manager.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_fs.h"
#include "esp_app_format.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_crc.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"
#include "aws_iot_mqtt_client.h"
#include "aws_cloudwatch_metrics.h"
#include "sdmmc_cmd.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"

/* Tag for logging */
static const char *TAG = "ota_manager";

/* OTA Manager state struct */
typedef struct {
    ota_manager_config_t config;           /* Configuration */
    TaskHandle_t task_handle;              /* OTA task handle */
    SemaphoreHandle_t mutex;               /* Access mutex */
    EventGroupHandle_t event_group;        /* Event flags */
    QueueHandle_t command_queue;           /* Command queue */
    TimerHandle_t check_timer;             /* Auto-check timer */
    
    ota_status_t current_status;           /* Current status */
    ota_error_t last_error;                /* Last error */
    
    esp_ota_handle_t update_handle;        /* OTA update handle */
    const esp_partition_t *update_partition; /* Update partition */
    
    ota_update_info_t update_info;         /* Current update info */
    char running_version[32];              /* Current version */
    char latest_version[32];               /* Latest available version */
    
    size_t bytes_received;                 /* Bytes received in current update */
    size_t total_bytes;                    /* Total bytes expected */

    /* BMS safety thresholds */
    uint8_t min_soc;                       /* Minimum state of charge % */
    float max_temperature;                 /* Maximum temperature °C */
    float min_temperature;                 /* Minimum temperature °C */
    float max_current;                     /* Maximum current A */
    
    /* Logging configuration */
    bool log_to_console;                   /* Console logging */
    bool log_to_sd;                        /* SD card logging */
    bool log_to_cloudwatch;                /* AWS CloudWatch logging */
    esp_log_level_t log_level;             /* Log level */
    
    FILE *log_file;                        /* Log file handle */
    
    /* MD5 and SHA256 context for verification */
    mbedtls_md_context_t md5_ctx;
    mbedtls_md_context_t sha256_ctx;
    
    bool initialized;                      /* Initialization state */
} ota_manager_t;

/* Event group bits */
#define OTA_EVENT_CHECK_REQUEST      (1 << 0)
#define OTA_EVENT_START_REQUEST      (1 << 1)
#define OTA_EVENT_CANCEL_REQUEST     (1 << 2)
#define OTA_EVENT_APPLY_REQUEST      (1 << 3)
#define OTA_EVENT_STOP_REQUEST       (1 << 4)
#define OTA_EVENT_TASK_STOPPED       (1 << 5)
#define OTA_EVENT_ALL                (0x3F)

/* Command types for the queue */
typedef enum {
    OTA_CMD_CHECK_UPDATE,
    OTA_CMD_START_UPDATE,
    OTA_CMD_CANCEL_UPDATE,
    OTA_CMD_APPLY_UPDATE,
} ota_cmd_type_t;

/* Command structure */
typedef struct {
    ota_cmd_type_t type;
    void *param;
} ota_cmd_t;

/* Singleton instance */
static ota_manager_t s_ota_manager = {0};

/* Forward declarations of static functions */
static void ota_task(void *pvParameter);
static void check_timer_callback(TimerHandle_t xTimer);
static esp_err_t check_for_update_internal(ota_update_info_t *update_info);
static esp_err_t begin_update_internal(void);
static esp_err_t apply_update_internal(void);
static esp_err_t cancel_update_internal(void);
static esp_err_t verify_update_signature(const esp_partition_t *partition);
static esp_err_t verify_update_hash(const esp_partition_t *partition);
static esp_err_t perform_safety_check(void);
static void update_status(ota_status_t status, ota_error_t error);
static esp_err_t init_hash_contexts(void);
static void deinit_hash_contexts(void);
static void update_hashes(const void *data, size_t len);
static bool verify_md5_hash(const char *expected_hash);
static bool verify_sha256_hash(const char *expected_hash);
static void log_message(esp_log_level_t level, const char *format, ...);
static esp_err_t http_event_handler(esp_http_client_event_t *evt);
static esp_err_t read_firmware_from_sd(const char *filepath);
static esp_err_t read_firmware_from_usb(const char *device_path);
static void report_progress(size_t received, size_t total);
static esp_err_t extract_version_from_firmware(const esp_partition_t *partition, char *version, size_t max_len);

/* OTA manager initialization */
esp_err_t ota_manager_init(const ota_manager_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Default configuration parameters */
    memset(&s_ota_manager, 0, sizeof(s_ota_manager));
    memcpy(&s_ota_manager.config, config, sizeof(ota_manager_config_t));
    
    /* Create synchronization primitives */
    s_ota_manager.mutex = xSemaphoreCreateMutex();
    if (s_ota_manager.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    s_ota_manager.event_group = xEventGroupCreate();
    if (s_ota_manager.event_group == NULL) {
        vSemaphoreDelete(s_ota_manager.mutex);
        return ESP_ERR_NO_MEM;
    }
    
    s_ota_manager.command_queue = xQueueCreate(10, sizeof(ota_cmd_t));
    if (s_ota_manager.command_queue == NULL) {
        vEventGroupDelete(s_ota_manager.event_group);
        vSemaphoreDelete(s_ota_manager.mutex);
        return ESP_ERR_NO_MEM;
    }
    
    /* Initialize default status */
    s_ota_manager.current_status = OTA_STATUS_IDLE;
    s_ota_manager.last_error = OTA_ERR_NONE;
    
    /* Get running firmware version */
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running != NULL) {
        extract_version_from_firmware(running, s_ota_manager.running_version, sizeof(s_ota_manager.running_version));
    } else {
        strncpy(s_ota_manager.running_version, "unknown", sizeof(s_ota_manager.running_version) - 1);
    }
    
    /* Default logging behavior */
    s_ota_manager.log_to_console = true;
    s_ota_manager.log_to_sd = false;
    s_ota_manager.log_to_cloudwatch = false;
    s_ota_manager.log_level = ESP_LOG_INFO;
    
    /* Default BMS safety thresholds */
    s_ota_manager.min_soc = 30;               /* 30% minimum SoC */
    s_ota_manager.max_temperature = 45.0f;    /* 45°C max temperature */
    s_ota_manager.min_temperature = 0.0f;     /* 0°C min temperature */
    s_ota_manager.max_current = 10.0f;        /* 10A max current */
    
    /* Create check timer if auto-check is enabled */
    if (config->check_interval_ms > 0) {
        s_ota_manager.check_timer = xTimerCreate(
            "ota_check_timer",
            pdMS_TO_TICKS(config->check_interval_ms),
            pdTRUE,  /* Auto-reload */
            NULL,
            check_timer_callback
        );
        
        if (s_ota_manager.check_timer == NULL) {
            vQueueDelete(s_ota_manager.command_queue);
            vEventGroupDelete(s_ota_manager.event_group);
            vSemaphoreDelete(s_ota_manager.mutex);
            return ESP_ERR_NO_MEM;
        }
    }
    
    s_ota_manager.initialized = true;
    
    log_message(ESP_LOG_INFO, "OTA Manager initialized, running version: %s", s_ota_manager.running_version);
    
    return ESP_OK;
}

/* Deinitialize OTA manager */
esp_err_t ota_manager_deinit(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Stop the OTA task if running */
    if (s_ota_manager.task_handle != NULL) {
        xEventGroupSetBits(s_ota_manager.event_group, OTA_EVENT_STOP_REQUEST);
        
        /* Wait for task to stop */
        EventBits_t bits = xEventGroupWaitBits(
            s_ota_manager.event_group,
            OTA_EVENT_TASK_STOPPED,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(5000)
        );
        
        if ((bits & OTA_EVENT_TASK_STOPPED) == 0) {
            /* Force delete if task didn't stop gracefully */
            vTaskDelete(s_ota_manager.task_handle);
        }
        
        s_ota_manager.task_handle = NULL;
    }
    
    /* Clean up resources */
    if (s_ota_manager.update_handle != 0) {
        esp_ota_abort(s_ota_manager.update_handle);
        s_ota_manager.update_handle = 0;
    }
    
    if (s_ota_manager.check_timer != NULL) {
        xTimerStop(s_ota_manager.check_timer, 0);
        xTimerDelete(s_ota_manager.check_timer, 0);
        s_ota_manager.check_timer = NULL;
    }
    
    if (s_ota_manager.log_file != NULL) {
        fclose(s_ota_manager.log_file);
        s_ota_manager.log_file = NULL;
    }
    
    deinit_hash_contexts();
    
    vQueueDelete(s_ota_manager.command_queue);
    vEventGroupDelete(s_ota_manager.event_group);
    vSemaphoreDelete(s_ota_manager.mutex);
    
    s_ota_manager.initialized = false;
    
    log_message(ESP_LOG_INFO, "OTA Manager deinitialized");
    
    return ESP_OK;
}

/* Start the OTA task */
esp_err_t ota_manager_start(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_ota_manager.task_handle != NULL) {
        return ESP_OK;  /* Already running */
    }
    
    /* Create OTA task */
    BaseType_t ret = xTaskCreatePinnedToCore(
        ota_task,
        "ota_task",
        s_ota_manager.config.task_stack_size,
        NULL,
        s_ota_manager.config.task_priority,
        &s_ota_manager.task_handle,
        s_ota_manager.config.core_id
    );
    
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    
    /* Start auto-check timer if configured */
    if (s_ota_manager.check_timer != NULL) {
        xTimerStart(s_ota_manager.check_timer, 0);
    }
    
    log_message(ESP_LOG_INFO, "OTA Manager task started");
    
    return ESP_OK;
}

/* Stop the OTA task */
esp_err_t ota_manager_stop(void) {
    if (!s_ota_manager.initialized || s_ota_manager.task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Signal task to stop */
    xEventGroupSetBits(s_ota_manager.event_group, OTA_EVENT_STOP_REQUEST);
    
    /* Wait for task to acknowledge */
    EventBits_t bits = xEventGroupWaitBits(
        s_ota_manager.event_group,
        OTA_EVENT_TASK_STOPPED,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(5000)
    );
    
    /* Check if task stopped properly */
    if ((bits & OTA_EVENT_TASK_STOPPED) == 0) {
        return ESP_ERR_TIMEOUT;
    }
    
    /* Stop timer */
    if (s_ota_manager.check_timer != NULL) {
        xTimerStop(s_ota_manager.check_timer, 0);
    }
    
    s_ota_manager.task_handle = NULL;
    
    log_message(ESP_LOG_INFO, "OTA Manager task stopped");
    
    return ESP_OK;
}

/* Check for available updates (non-blocking) */
esp_err_t ota_manager_check_for_update(void) {
    if (!s_ota_manager.initialized || s_ota_manager.task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Send command to check for update */
    ota_cmd_t cmd = {
        .type = OTA_CMD_CHECK_UPDATE,
        .param = NULL
    };
    
    if (xQueueSend(s_ota_manager.command_queue, &cmd, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

/* Check for available updates (blocking) */
esp_err_t ota_manager_check_for_update_blocking(ota_update_info_t *update_info) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Get mutex to ensure exclusive access */
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t result = check_for_update_internal(update_info);
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    return result;
}

/* Begin downloading and applying an update (non-blocking) */
esp_err_t ota_manager_begin_update(void) {
    if (!s_ota_manager.initialized || s_ota_manager.task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_ota_manager.current_status != OTA_STATUS_UPDATE_AVAILABLE && 
        s_ota_manager.current_status != OTA_STATUS_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Send command to start update */
    ota_cmd_t cmd = {
        .type = OTA_CMD_START_UPDATE,
        .param = NULL
    };
    
    if (xQueueSend(s_ota_manager.command_queue, &cmd, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

/* Begin downloading and applying an update (blocking) */
esp_err_t ota_manager_begin_update_blocking(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_ota_manager.current_status != OTA_STATUS_UPDATE_AVAILABLE && 
        s_ota_manager.current_status != OTA_STATUS_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Get mutex to ensure exclusive access */
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t result = begin_update_internal();
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    return result;
}

/* Cancel an ongoing update */
esp_err_t ota_manager_cancel_update(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Check if there's an update to cancel */
    if (s_ota_manager.current_status != OTA_STATUS_DOWNLOADING &&
        s_ota_manager.current_status != OTA_STATUS_VERIFYING) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_ota_manager.task_handle != NULL) {
        /* Send command to cancel update */
        ota_cmd_t cmd = {
            .type = OTA_CMD_CANCEL_UPDATE,
            .param = NULL
        };
        
        if (xQueueSend(s_ota_manager.command_queue, &cmd, 0) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        
        return ESP_OK;
    } else {
        /* Call directly if no task */
        if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        
        esp_err_t result = cancel_update_internal();
        
        xSemaphoreGive(s_ota_manager.mutex);
        
        return result;
    }
}

/* Apply a downloaded update and reboot */
esp_err_t ota_manager_apply_update(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Check if there's an update to apply */
    if (s_ota_manager.current_status != OTA_STATUS_READY_TO_APPLY) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_ota_manager.task_handle != NULL) {
        /* Send command to apply update */
        ota_cmd_t cmd = {
            .type = OTA_CMD_APPLY_UPDATE,
            .param = NULL
        };
        
        if (xQueueSend(s_ota_manager.command_queue, &cmd, 0) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        
        return ESP_OK;
    } else {
        /* Call directly if no task */
        if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        
        esp_err_t result = apply_update_internal();
        
        xSemaphoreGive(s_ota_manager.mutex);
        
        return result;
    }
}

/* Get current OTA status */
esp_err_t ota_manager_get_status(ota_status_t *status, ota_error_t *error) {
    if (!s_ota_manager.initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    *status = s_ota_manager.current_status;
    if (error != NULL) {
        *error = s_ota_manager.last_error;
    }
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    return ESP_OK;
}

/* Check if an update is in progress */
bool ota_manager_is_updating(void) {
    ota_status_t status;
    
    if (ota_manager_get_status(&status, NULL) != ESP_OK) {
        return false;
    }
    
    return (status == OTA_STATUS_DOWNLOADING || 
            status == OTA_STATUS_VERIFYING || 
            status == OTA_STATUS_READY_TO_APPLY ||
            status == OTA_STATUS_APPLYING);
}

/* Get error string description */
const char *ota_manager_error_to_string(ota_error_t error) {
    switch (error) {
        case OTA_ERR_NONE:
            return "No error";
        case OTA_ERR_NO_UPDATES:
            return "No updates available";
        case OTA_ERR_CONNECTIVITY:
            return "Connectivity error";
        case OTA_ERR_SERVER:
            return "Server error";
        case OTA_ERR_AUTHENTICATION:
            return "Authentication failed";
        case OTA_ERR_INSUFFICIENT_SPACE:
            return "Insufficient space for update";
        case OTA_ERR_VALIDATION_FAILED:
            return "Update validation failed";
        case OTA_ERR_FLASH_WRITE:
            return "Flash write error";
        case OTA_ERR_INCOMPATIBLE_VERSION:
            return "Incompatible firmware version";
        case OTA_ERR_BATTERY_LOW:
            return "Battery level too low";
        case OTA_ERR_BMS_UNSAFE:
            return "BMS reports unsafe condition";
        case OTA_ERR_INTERNAL:
            return "Internal error";
        default:
            return "Unknown error";
    }
}

/* Configure logging options */
esp_err_t ota_manager_config_logging(bool log_to_console, bool log_to_sd, 
                                  bool log_to_cloudwatch, esp_log_level_t log_level) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    s_ota_manager.log_to_console = log_to_console;
    s_ota_manager.log_to_cloudwatch = log_to_cloudwatch;
    s_ota_manager.log_level = log_level;
    
    /* Handle SD card logging changes */
    if (s_ota_manager.log_to_sd != log_to_sd) {
        s_ota_manager.log_to_sd = log_to_sd;
        
        /* Close existing log file if we're disabling SD logging */
        if (!log_to_sd && s_ota_manager.log_file != NULL) {
            fclose(s_ota_manager.log_file);
            s_ota_manager.log_file = NULL;
        }
        
        /* Open new log file if we're enabling SD logging */
        if (log_to_sd && s_ota_manager.log_file == NULL) {
            /* Mount SD card if needed */
            sdmmc_card_t *card = NULL;  /* Would be initialized in actual impl */
            
            if (card != NULL) {
                s_ota_manager.log_file = fopen("/sdcard/ota_manager.log", "a");
                if (s_ota_manager.log_file == NULL) {
                    ESP_LOGE(TAG, "Failed to open OTA log file");
                    /* Continue even if file can't be opened */
                }
            }
        }
    }
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    log_message(ESP_LOG_INFO, "OTA logger configured - console: %d, SD: %d, CloudWatch: %d, level: %d",
               log_to_console, log_to_sd, log_to_cloudwatch, log_level);
    
    return ESP_OK;
}

/* Mark current firmware as valid */
esp_err_t ota_manager_mark_valid(void) {
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    
    if (err == ESP_OK) {
        log_message(ESP_LOG_INFO, "Current firmware marked as valid");
    } else {
        log_message(ESP_LOG_ERROR, "Failed to mark firmware as valid: %d", err);
    }
    
    return err;
}

/* Get running firmware version */
esp_err_t ota_manager_get_running_version(char *version, size_t max_len) {
    if (!s_ota_manager.initialized || version == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    strncpy(version, s_ota_manager.running_version, max_len - 1);
    version[max_len - 1] = '\0';
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    return ESP_OK;
}

/* Get latest available firmware version */
esp_err_t ota_manager_get_latest_version(char *version, size_t max_len) {
    if (!s_ota_manager.initialized || version == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    if (strlen(s_ota_manager.latest_version) == 0) {
        /* No latest version known */
        version[0] = '\0';
        xSemaphoreGive(s_ota_manager.mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    strncpy(version, s_ota_manager.latest_version, max_len - 1);
    version[max_len - 1] = '\0';
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    return ESP_OK;
}

/* Configure rollback settings */
esp_err_t ota_manager_config_rollback(bool allow_rollback, uint32_t timeout_ms) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    s_ota_manager.config.allow_rollback = allow_rollback;
    s_ota_manager.config.rollback_timeout_ms = timeout_ms;
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    log_message(ESP_LOG_INFO, "Rollback settings updated - allow_rollback: %d, timeout: %u ms", 
               allow_rollback, timeout_ms);
    
    return ESP_OK;
}

/* Request a rollback to previous firmware */
esp_err_t ota_manager_rollback(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!ota_manager_can_rollback()) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    log_message(ESP_LOG_WARN, "Initiating firmware rollback");
    
    /* Perform actual rollback */
    esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
    if (err != ESP_OK) {
        log_message(ESP_LOG_ERROR, "Rollback failed: %d", err);
    }
    
    return err;
}

/* Check if rollback is available */
bool ota_manager_can_rollback(void) {
    if (!s_ota_manager.initialized) {
        return false;
    }
    
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *partition = esp_ota_get_last_invalid_partition();
    
    /* Check if we have a previous partition that can be rolled back to */
    if (running == NULL || partition == NULL || running == partition) {
        return false;
    }
    
    return true;
}

/* Get update partition information */
esp_err_t ota_manager_get_update_partition_info(esp_ota_img_states_t *state, 
                                              esp_partition_info_t *info) {
    if (!s_ota_manager.initialized || state == NULL || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Get update partition */
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    /* Get partition state */
    esp_err_t err = esp_ota_get_state_partition(update_partition, state);
    if (err != ESP_OK) {
        return err;
    }
    
    /* Copy partition info */
    memcpy(info, update_partition, sizeof(esp_partition_info_t));
    
    return ESP_OK;
}

/* Report diagnostics to AWS CloudWatch */
esp_err_t ota_manager_report_diagnostics(void) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!s_ota_manager.log_to_cloudwatch) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Get current status */
    ota_status_t status;
    ota_error_t error;
    ota_manager_get_status(&status, &error);
    
    /* Get system info for diagnostics */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    /* Format diagnostic data for CloudWatch */
    /* In a real implementation, this would use the AWS CloudWatch SDK */
    log_message(ESP_LOG_INFO, "Reporting OTA diagnostics to AWS CloudWatch");
    
    /* Example of reporting metrics to CloudWatch (would use AWS SDK) */
    /* aws_cloudwatch_report_metric("OTA Status", status); */
    /* aws_cloudwatch_report_metric("OTA Error", error); */
    
    return ESP_OK;
}

/* Set BMS safety thresholds for OTA updates */
esp_err_t ota_manager_set_bms_safety_thresholds(uint8_t min_soc, float max_temperature,
                                             float min_temperature, float max_current) {
    if (!s_ota_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (min_soc > 100 || max_temperature < min_temperature) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    s_ota_manager.min_soc = min_soc;
    s_ota_manager.max_temperature = max_temperature;
    s_ota_manager.min_temperature = min_temperature;
    s_ota_manager.max_current = max_current;
    
    xSemaphoreGive(s_ota_manager.mutex);
    
    log_message(ESP_LOG_INFO, "BMS safety thresholds updated - min SoC: %u%%, temp range: %.1f-%.1f°C, max current: %.1fA",
               min_soc, min_temperature, max_temperature, max_current);
    
    return ESP_OK;
}

/* ===== STATIC FUNCTION IMPLEMENTATIONS ===== */

/* Main OTA task function */
static void ota_task(void *pvParameter) {
    ota_cmd_t cmd;
    
    log_message(ESP_LOG_INFO, "OTA task started");
    
    while (1) {
        /* Check for stop request */
        EventBits_t bits = xEventGroupGetBits(s_ota_manager.event_group);
        if (bits & OTA_EVENT_STOP_REQUEST) {
            break;
        }
        
        /* Wait for commands with timeout */
        if (xQueueReceive(s_ota_manager.command_queue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            /* Process received command */
            switch (cmd.type) {
                case OTA_CMD_CHECK_UPDATE:
                    log_message(ESP_LOG_INFO, "Processing check update command");
                    
                    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                        check_for_update_internal(NULL);
                        xSemaphoreGive(s_ota_manager.mutex);
                    } else {
                        log_message(ESP_LOG_ERROR, "Failed to get mutex for check update");
                    }
                    break;
                    
                case OTA_CMD_START_UPDATE:
                    log_message(ESP_LOG_INFO, "Processing start update command");
                    
                    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                        begin_update_internal();
                        xSemaphoreGive(s_ota_manager.mutex);
                    } else {
                        log_message(ESP_LOG_ERROR, "Failed to get mutex for start update");
                    }
                    break;
                    
                case OTA_CMD_CANCEL_UPDATE:
                    log_message(ESP_LOG_INFO, "Processing cancel update command");
                    
                    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                        cancel_update_internal();
                        xSemaphoreGive(s_ota_manager.mutex);
                    } else {
                        log_message(ESP_LOG_ERROR, "Failed to get mutex for cancel update");
                    }
                    break;
                    
                case OTA_CMD_APPLY_UPDATE:
                    log_message(ESP_LOG_INFO, "Processing apply update command");
                    
                    if (xSemaphoreTake(s_ota_manager.mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                        apply_update_internal();
                        xSemaphoreGive(s_ota_manager.mutex);
                    } else {
                        log_message(ESP_LOG_ERROR, "Failed to get mutex for apply update");
                    }
                    break;
                    
                default:
                    log_message(ESP_LOG_WARN, "Unknown command received: %d", cmd.type);
                    break;
            }
        }
    }
    
    log_message(ESP_LOG_INFO, "OTA task stopping");
    
    /* Signal that the task is stopping */
    xEventGroupSetBits(s_ota_manager.event_group, OTA_EVENT_TASK_STOPPED);
    
    /* Clean up any resources if needed */
    if (s_ota_manager.update_handle != 0) {
        esp_ota_abort(s_ota_manager.update_handle);
        s_ota_manager.update_handle = 0;
    }
    
    /* Delete the task */
    vTaskDelete(NULL);
}

/* Timer callback for automatic update checks */
static void check_timer_callback(TimerHandle_t xTimer) {
    if (!s_ota_manager.initialized || s_ota_manager.task_handle == NULL) {
        return;
    }
    
    /* Send command to check for update */
    ota_cmd_t cmd = {
        .type = OTA_CMD_CHECK_UPDATE,
        .param = NULL
    };
    
    xQueueSend(s_ota_manager.command_queue, &cmd, 0);
}

/* Check for available updates (internal implementation) */
static esp_err_t check_for_update_internal(ota_update_info_t *update_info) {
    esp_err_t err = ESP_OK;
    
    /* Update status */
    update_status(OTA_STATUS_CHECKING, OTA_ERR_NONE);
    
    /* Get configured source */
    ota_source_t source = s_ota_manager.config.source;
    
    log_message(ESP_LOG_INFO, "Checking for updates from source %d", source);
    
    switch (source) {
        case OTA_SOURCE_HTTPS: {
            /* Configure HTTP client */
            esp_http_client_config_t config = {
                .url = s_ota_manager.config.config.https.url,
                .cert_pem = s_ota_manager.config.config.https.cert_pem,
                .timeout_ms = 10000,
                .event_handler = http_event_handler,
            };
            
            /* Add basic auth if configured */
            if (s_ota_manager.config.config.https.basic_auth_user != NULL) {
                config.username = s_ota_manager.config.config.https.basic_auth_user;
                config.password = s_ota_manager.config.config.https.basic_auth_pass;
            }
            
            /* Initialize HTTP client */
            esp_http_client_handle_t client = esp_http_client_init(&config);
            if (client == NULL) {
                err = ESP_ERR_NO_MEM;
                goto exit;
            }
            
            /* First, make a HEAD request to get metadata */
            esp_http_client_set_method(client, HTTP_METHOD_HEAD);
            err = esp_http_client_perform(client);
            
            if (err != ESP_OK) {
                log_message(ESP_LOG_ERROR, "HTTP request failed: %s", esp_err_to_name(err));
                esp_http_client_cleanup(client);
                err = OTA_ERR_CONNECTIVITY;
                goto exit;
            }
            
            int status_code = esp_http_client_get_status_code(client);
            if (status_code != 200) {
                log_message(ESP_LOG_ERROR, "HTTP server returned status code %d", status_code);
                esp_http_client_cleanup(client);
                err = (status_code >= 500) ? OTA_ERR_SERVER : OTA_ERR_CONNECTIVITY;
                goto exit;
            }
            
            /* Parse version info from headers */
            char version_header[33] = {0};
            esp_http_client_get_header(client, "X-Firmware-Version", version_header, sizeof(version_header) - 1);
            
            /* Only process update if version is different from current */
            if (strcmp(version_header, s_ota_manager.running_version) == 0) {
                log_message(ESP_LOG_INFO, "Already running latest version: %s", version_header);
                esp_http_client_cleanup(client);
                err = OTA_ERR_NO_UPDATES;
                goto exit;
            }
            
            /* Store version */
            strncpy(s_ota_manager.latest_version, version_header, sizeof(s_ota_manager.latest_version) - 1);
            strncpy(s_ota_manager.update_info.version, version_header, sizeof(s_ota_manager.update_info.version) - 1);
            
            /* Get content length */
            int content_length = esp_http_client_get_content_length(client);
            if (content_length <= 0) {
                log_message(ESP_LOG_WARN, "Content length not available");
                /* Continue anyway - we'll get it when downloading */
            }
            s_ota_manager.update_info.size = content_length;
            
            /* Get MD5 hash if available */
            char md5_header[33] = {0};
            esp_http_client_get_header(client, "X-Firmware-MD5", md5_header, sizeof(md5_header) - 1);
            if (strlen(md5_header) > 0) {
                strncpy(s_ota_manager.update_info.md5_hash, md5_header, sizeof(s_ota_manager.update_info.md5_hash) - 1);
            }
            
            /* Get SHA256 hash if available */
            char sha256_header[65] = {0};
            esp_http_client_get_header(client, "X-Firmware-SHA256", sha256_header, sizeof(sha256_header) - 1);
            if (strlen(sha256_header) > 0) {
                strncpy(s_ota_manager.update_info.sha256_hash, sha256_header, sizeof(s_ota_manager.update_info.sha256_hash) - 1);
            }
            
            /* Get release notes if available */
            char notes_header[256] = {0};
            esp_http_client_get_header(client, "X-Firmware-Notes", notes_header, sizeof(notes_header) - 1);
            if (strlen(notes_header) > 0) {
                strncpy(s_ota_manager.update_info.release_notes, notes_header, sizeof(s_ota_manager.update_info.release_notes) - 1);
            }
            
            /* Get other flags */
            char mandatory_header[10] = {0};
            esp_http_client_get_header(client, "X-Firmware-Mandatory", mandatory_header, sizeof(mandatory_header) - 1);
            s_ota_manager.update_info.is_mandatory = (strcmp(mandatory_header, "true") == 0);
            
            char reboot_header[10] = {0};
            esp_http_client_get_header(client, "X-Firmware-Reboot", reboot_header, sizeof(reboot_header) - 1);
            s_ota_manager.update_info.requires_reboot = (strcmp(reboot_header, "false") != 0); /* Default to true */
            
            char min_soc_header[10] = {0};
            esp_http_client_get_header(client, "X-Firmware-MinSoC", min_soc_header, sizeof(min_soc_header) - 1);
            if (strlen(min_soc_header) > 0) {
                s_ota_manager.update_info.min_battery_level = atoi(min_soc_header);
            } else {
                s_ota_manager.update_info.min_battery_level = s_ota_manager.min_soc;
            }
            
            esp_http_client_cleanup(client);
            
            log_message(ESP_LOG_INFO, "Update available: %s, size: %d bytes", 
                      s_ota_manager.update_info.version, s_ota_manager.update_info.size);
            
            /* If caller wants update info, copy it */
            if (update_info != NULL) {
                memcpy(update_info, &s_ota_manager.update_info, sizeof(ota_update_info_t));
            }
            
            /* Update status */
            update_status(OTA_STATUS_UPDATE_AVAILABLE, OTA_ERR_NONE);
            
            /* If auto-update is enabled, start the update */
            if (s_ota_manager.config.automatic_update) {
                begin_update_internal();
            }
            
            break;
        }
            
        case OTA_SOURCE_AWS_IOT:
            /* AWS IoT update source implementation would go here */
            log_message(ESP_LOG_WARN, "AWS IoT update source not fully implemented");
            err = ESP_ERR_NOT_SUPPORTED;
            break;
            
        case OTA_SOURCE_MODBUS:
            /* Modbus update source implementation would go here */
            log_message(ESP_LOG_WARN, "Modbus update source not fully implemented");
            err = ESP_ERR_NOT_SUPPORTED;
            break;
            
        case OTA_SOURCE_CANBUS:
            /* CANBus update source implementation would go here */
            log_message(ESP_LOG_WARN, "CANBus update source not fully implemented");
            err = ESP_ERR_NOT_SUPPORTED;
            break;
            
        case OTA_SOURCE_SD_CARD:
            /* SD Card update source implementation */
            /* This would check for a specific file */
            if (s_ota_manager.config.config.sd_card.filepath != NULL) {
                /* Check if file exists and get metadata */
                FILE *f = fopen(s_ota_manager.config.config.sd_card.filepath, "rb");
                if (f != NULL) {
                    /* Get file size */
                    fseek(f, 0, SEEK_END);
                    long file_size = ftell(f);
                    fseek(f, 0, SEEK_SET);
                    
                    /* Read version information from file header */
                    /* In a real implementation, you would extract version info */
                    /* from the file's app descriptor */
                    fclose(f);
                    
                    /* Update info */
                    s_ota_manager.update_info.size = file_size;
                    strncpy(s_ota_manager.update_info.version, "sd_card_update", sizeof(s_ota_manager.update_info.version) - 1);
                    strncpy(s_ota_manager.latest_version, "sd_card_update", sizeof(s_ota_manager.latest_version) - 1);
                    
                    /* Update status */
                    update_status(OTA_STATUS_UPDATE_AVAILABLE, OTA_ERR_NONE);
                    
                    if (update_info != NULL) {
                        memcpy(update_info, &s_ota_manager.update_info, sizeof(ota_update_info_t));
                    }
                    
                    log_message(ESP_LOG_INFO, "Update file found on SD card: %s, size: %ld bytes", 
                              s_ota_manager.config.config.sd_card.filepath, file_size);
                              
                    /* If auto-update is enabled, start the update */
                    if (s_ota_manager.config.automatic_update) {
                        begin_update_internal();
                    }
                } else {
                    log_message(ESP_LOG_WARN, "Update file not found on SD card: %s", 
                              s_ota_manager.config.config.sd_card.filepath);
                    err = OTA_ERR_NO_UPDATES;
                }
            } else {
                log_message(ESP_LOG_ERROR, "SD card filepath not configured");
                err = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case OTA_SOURCE_USB:
            /* USB update source implementation would go here */
            log_message(ESP_LOG_WARN, "USB update source not fully implemented");
            err = ESP_ERR_NOT_SUPPORTED;
            break;
            
        default:
            log_message(ESP_LOG_ERROR, "Unknown update source: %d", source);
            err = ESP_ERR_INVALID_ARG;
            break;
    }
    
exit:
    if (err != ESP_OK && err != OTA_ERR_NO_UPDATES) {
        /* Update status on error */
        update_status(OTA_STATUS_IDLE, (ota_error_t)err);
    }
    
    return err;
}

/* Initialize hash contexts */
static esp_err_t init_hash_contexts(void) {
    /* Initialize MD5 context */
    mbedtls_md_init(&s_ota_manager.md5_ctx);
    if (mbedtls_md_setup(&s_ota_manager.md5_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_MD5), 0) != 0) {
        mbedtls_md_free(&s_ota_manager.md5_ctx);
        return ESP_FAIL;
    }
    mbedtls_md_starts(&s_ota_manager.md5_ctx);
    
    /* Initialize SHA256 context */
    mbedtls_md_init(&s_ota_manager.sha256_ctx);
    if (mbedtls_md_setup(&s_ota_manager.sha256_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0) != 0) {
        mbedtls_md_free(&s_ota_manager.md5_ctx);
        mbedtls_md_free(&s_ota_manager.sha256_ctx);
        return ESP_FAIL;
    }
    mbedtls_md_starts(&s_ota_manager.sha256_ctx);
    
    return ESP_OK;
}

/* Deinitialize hash contexts */
static void deinit_hash_contexts(void) {
    mbedtls_md_free(&s_ota_manager.md5_ctx);
    mbedtls_md_free(&s_ota_manager.sha256_ctx);
}

/* Update hash values with new data */
static void update_hashes(const void *data, size_t len) {
    mbedtls_md_update(&s_ota_manager.md5_ctx, data, len);
    mbedtls_md_update(&s_ota_manager.sha256_ctx, data, len);
}

/* Verify MD5 hash */
static bool verify_md5_hash(const char *expected_hash) {
    unsigned char md5_result[16];
    char md5_str[33];
    
    mbedtls_md_finish(&s_ota_manager.md5_ctx, md5_result);
    
    /* Convert binary hash to hex string */
    for (int i = 0; i < 16; i++) {
        sprintf(&md5_str[i * 2], "%02x", md5_result[i]);
    }
    md5_str[32] = '\0';
    
    log_message(ESP_LOG_DEBUG, "Calculated MD5: %s, Expected: %s", md5_str, expected_hash);
    
    return (strcmp(md5_str, expected_hash) == 0);
}

/* Verify SHA256 hash */
static bool verify_sha256_hash(const char *expected_hash) {
    unsigned char sha_result[32];
    char sha_str[65];
    
    mbedtls_md_finish(&s_ota_manager.sha256_ctx, sha_result);
    
    /* Convert binary hash to hex string */
    for (int i = 0; i < 32; i++) {
        sprintf(&sha_str[i * 2], "%02x", sha_result[i]);
    }
    sha_str[64] = '\0';
    
    log_message(ESP_LOG_DEBUG, "Calculated SHA256: %s, Expected: %s", sha_str, expected_hash);
    
    return (strcmp(sha_str, expected_hash) == 0);
}

/* Log message to all configured outputs */
static void log_message(esp_log_level_t level, const char *format, ...) {
    if ((int)level < (int)s_ota_manager.log_level) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    
    /* Console logging */
    if (s_ota_manager.log_to_console) {
        char log_buffer[512];
        vsnprintf(log_buffer, sizeof(log_buffer), format, args);
        esp_log_write(level, TAG, "%s", log_buffer);
    }
    
    /* SD card logging */
    if (s_ota_manager.log_to_sd && s_ota_manager.log_file != NULL) {
        char time_buffer[32];
        time_t now;
        time(&now);
        strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
        
        char level_str[10];
        switch (level) {
            case ESP_LOG_ERROR: strcpy(level_str, "ERROR"); break;
            case ESP_LOG_WARN: strcpy(level_str, "WARN"); break;
            case ESP_LOG_INFO: strcpy(level_str, "INFO"); break;
            case ESP_LOG_DEBUG: strcpy(level_str, "DEBUG"); break;
            case ESP_LOG_VERBOSE: strcpy(level_str, "VERBOSE"); break;
            default: strcpy(level_str, "?"); break;
        }
        
        fprintf(s_ota_manager.log_file, "[%s] %s: ", time_buffer, level_str);
        vfprintf(s_ota_manager.log_file, format, args);
        fprintf(s_ota_manager.log_file, "\n");
    }
    
    /* AWS CloudWatch logging */
    if (s_ota_manager.log_to_cloudwatch) {
        /* In a real implementation, this would use the AWS CloudWatch SDK */
        /* aws_cloudwatch_log_event(TAG, level, format, args); */
    }
    
    va_end(args);
}

/* HTTP client event handler */
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            log_message(ESP_LOG_ERROR, "HTTP client error");
            break;
            
        case HTTP_EVENT_ON_CONNECTED:
            log_message(ESP_LOG_DEBUG, "HTTP client connected");
            break;
            
        case HTTP_EVENT_HEADERS_SENT:
            log_message(ESP_LOG_DEBUG, "HTTP headers sent");
            break;
            
        case HTTP_EVENT_ON_HEADER:
            log_message(ESP_LOG_DEBUG, "HTTP header: %s: %s", evt->header_key, evt->header_value);
            break;
            
        case HTTP_EVENT_ON_DATA:
            /* Process received data */
            if (evt->data_len > 0 && s_ota_manager.update_handle != 0) {
                /* Write data to OTA partition */
                esp_err_t err = esp_ota_write(s_ota_manager.update_handle, evt->data, evt->data_len);
                if (err != ESP_OK) {
                    log_message(ESP_LOG_ERROR, "esp_ota_write failed: %s", esp_err_to_name(err));
                    return err;
                }
                
                /* Update hashes */
                update_hashes(evt->data, evt->data_len);
                
                /* Update progress */
                s_ota_manager.bytes_received += evt->data_len;
                report_progress(s_ota_manager.bytes_received, s_ota_manager.total_bytes);
            }
            break;
            
        case HTTP_EVENT_ON_FINISH:
            log_message(ESP_LOG_DEBUG, "HTTP request finished");
            break;
            
        case HTTP_EVENT_DISCONNECTED:
            log_message(ESP_LOG_DEBUG, "HTTP client disconnected");
            break;
    }
    
    return ESP_OK;
}