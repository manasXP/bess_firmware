/**
 * @file system_manager.c
 * @brief Implementation of the system manager for 100KW/200KWH BESS controller
 *
 * This component serves as the central coordinator for the Battery Energy Storage System (BESS),
 * integrating the Battery Management System (BMS), communication interfaces (Modbus, CANBus),
 * and logging subsystems.
 * 
 * Hardware: ESP32-P4
 * RTOS: FreeRTOS
 */

#include "system_manager.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"

/* Include headers for BMS components */
#include "battery_manager.h"
#include "soc_calculator.h"
#include "cell_balancer.h"
#include "thermal_monitor.h"

/* Include headers for communication interfaces */
#include "modbus_interface.h"
#include "canbus_interface.h"
#include "aws_interface.h"

/* Define constants */
#define TAG "SYSTEM_MANAGER"
#define SYSTEM_MANAGER_TASK_STACK_SIZE 8192
#define SYSTEM_MANAGER_TASK_PRIORITY 10
#define SYSTEM_MANAGER_TASK_CORE 0
#define SYSTEM_MANAGER_WATCHDOG_TIMEOUT_SEC 30
#define SYSTEM_MANAGER_MAIN_LOOP_INTERVAL_MS 1000
#define SYSTEM_MANAGER_TIME_SYNC_INTERVAL_SEC 3600
#define SYSTEM_MANAGER_LOG_QUEUE_SIZE 50
#define SYSTEM_MANAGER_MAX_EVENT_CALLBACKS 10
#define SYSTEM_MANAGER_MAX_LOG_SIZE 1024
#define SYSTEM_MANAGER_MODULE_COUNT_MAX 16
#define SYSTEM_MANAGER_SD_MOUNT_POINT "/sdcard"
#define SYSTEM_MANAGER_LOG_FILENAME_FORMAT "/sdcard/logs/bess_%Y%m%d.log"
#define SYSTEM_MANAGER_CONFIG_FILENAME "/sdcard/config/bess_config.bin"
#define SYSTEM_MANAGER_NVS_NAMESPACE "bess_config"
#define SYSTEM_MANAGER_NVS_CONFIG_KEY "sys_config"
#define SYSTEM_MANAGER_FIRMWARE_VERSION 0x0100  /* v1.0 */
#define SYSTEM_MANAGER_VERSION_STRING "BESS Controller v1.0.0"

/* SD Card configuration */
#define SD_PIN_NUM_MISO 2
#define SD_PIN_NUM_MOSI 15
#define SD_PIN_NUM_CLK  14
#define SD_PIN_NUM_CS   13

/* Define task handles */
static TaskHandle_t s_system_manager_task_handle = NULL;
static TaskHandle_t s_log_task_handle = NULL;

/* Define queue handles */
static QueueHandle_t s_log_queue = NULL;
static QueueHandle_t s_event_queue = NULL;

/* Define synchronization primitives */
static SemaphoreHandle_t s_system_data_mutex = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;
static EventGroupHandle_t s_system_event_group = NULL;

/* System event bits */
#define SYSTEM_EVENT_INIT_COMPLETE (1 << 0)
#define SYSTEM_EVENT_BMS_READY     (1 << 1)
#define SYSTEM_EVENT_COMM_READY    (1 << 2)
#define SYSTEM_EVENT_TIME_SYNCED   (1 << 3)
#define SYSTEM_EVENT_SD_READY      (1 << 4)
#define SYSTEM_EVENT_CLOUD_READY   (1 << 5)
#define SYSTEM_EVENT_SHUTDOWN      (1 << 6)
#define SYSTEM_EVENT_REBOOT        (1 << 7)
#define SYSTEM_EVENT_EMERGENCY     (1 << 8)

/* Define structure to store system state */
typedef struct {
    bess_system_info_t info;
    bess_system_config_t config;
    bess_comm_stats_t comm_stats;
    uint64_t last_time_sync;
    bool sd_card_mounted;
    FILE *log_file;
    char log_filename[64];
    bool initialized;
    bool running;
} system_state_t;

/* Define structure for event callbacks */
typedef struct {
    bool active;
    uint32_t event_mask;
    bess_event_callback_t callback;
    void *user_data;
} event_callback_entry_t;

/* Define structure for log queue items */
typedef struct {
    uint32_t timestamp;
    uint32_t level;
    char tag[16];
    char message[SYSTEM_MANAGER_MAX_LOG_SIZE];
} log_queue_item_t;

/* Static variables */
static system_state_t s_system_state = {0};
static event_callback_entry_t s_event_callbacks[SYSTEM_MANAGER_MAX_EVENT_CALLBACKS] = {0};

/* BMS subsystem handler */
static void *s_bms_handler = NULL;

/* Communication interface handlers */
static void *s_modbus_handler = NULL;
static void *s_canbus_handler = NULL;
static void *s_aws_handler = NULL;

/* Function prototypes for internal functions */
static void system_manager_task(void *pvParameters);
static void log_processing_task(void *pvParameters);
static void time_sync_callback(struct timeval *tv);
static esp_err_t init_subsystems(void);
static esp_err_t init_sd_card(void);
static esp_err_t init_logging(void);
static esp_err_t init_nvs(void);
static esp_err_t init_time_sync(void);
static esp_err_t process_events(void);
static esp_err_t write_log_to_sd(const log_queue_item_t *log_item);
static esp_err_t send_log_to_cloudwatch(const log_queue_item_t *log_item);
static void notify_event_callbacks(bess_event_data_t event);
static void emergency_shutdown_handler(void);
static esp_err_t update_system_status(void);
static esp_err_t rotate_log_file(void);
static esp_err_t create_directory(const char *dir);
static esp_err_t init_bms_subsystem(void);
static esp_err_t init_communication_interfaces(void);
static esp_err_t init_aws_interface(void);

/**
 * @brief Initialize the system manager
 */
esp_err_t system_manager_init(const bess_system_config_t *config) {
    ESP_LOGI(TAG, "Initializing system manager");
    
    if (s_system_state.initialized) {
        ESP_LOGW(TAG, "System manager already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Initialize synchronization primitives */
    s_system_data_mutex = xSemaphoreCreateMutex();
    if (s_system_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config mutex");
        vSemaphoreDelete(s_system_data_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    s_system_event_group = xEventGroupCreate();
    if (s_system_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(s_system_data_mutex);
        vSemaphoreDelete(s_config_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    /* Initialize queues */
    s_log_queue = xQueueCreate(SYSTEM_MANAGER_LOG_QUEUE_SIZE, sizeof(log_queue_item_t));
    if (s_log_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create log queue");
        vSemaphoreDelete(s_system_data_mutex);
        vSemaphoreDelete(s_config_mutex);
        vEventGroupDelete(s_system_event_group);
        return ESP_ERR_NO_MEM;
    }
    
    s_event_queue = xQueueCreate(20, sizeof(bess_event_data_t));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        vSemaphoreDelete(s_system_data_mutex);
        vSemaphoreDelete(s_config_mutex);
        vEventGroupDelete(s_system_event_group);
        vQueueDelete(s_log_queue);
        return ESP_ERR_NO_MEM;
    }
    
    /* Initialize NVS for configuration storage */
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize system state with provided configuration */
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&s_system_state.config, config, sizeof(bess_system_config_t));
        
        /* Initialize system info structure */
        s_system_state.info.mode = BESS_MODE_STANDBY;
        s_system_state.info.status_flags = 0;
        s_system_state.info.system_soc = 0.0f;
        s_system_state.info.system_soh = 100.0f;
        s_system_state.info.max_temperature = 0.0f;
        s_system_state.info.min_temperature = 0.0f;
        s_system_state.info.avg_temperature = 0.0f;
        s_system_state.info.power_output = 0.0f;
        s_system_state.info.energy_remaining = 0.0f;
        s_system_state.info.uptime = 0;
        s_system_state.info.active_alarms = 0;
        s_system_state.info.module_count = config->module_count;
        s_system_state.info.firmware_version = SYSTEM_MANAGER_FIRMWARE_VERSION;
        strncpy(s_system_state.info.system_id, config->system_name, sizeof(s_system_state.info.system_id) - 1);
        s_system_state.info.system_id[sizeof(s_system_state.info.system_id) - 1] = '\0';
        
        /* Reset communication statistics */
        memset(&s_system_state.comm_stats, 0, sizeof(bess_comm_stats_t));
        
        /* Initialize other state variables */
        s_system_state.last_time_sync = 0;
        s_system_state.sd_card_mounted = false;
        s_system_state.log_file = NULL;
        s_system_state.initialized = true;
        s_system_state.running = false;
        
        xSemaphoreGive(s_config_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take config mutex during initialization");
        return ESP_FAIL;
    }
    
    /* Create log processing task */
    BaseType_t task_created = xTaskCreatePinnedToCore(
        log_processing_task,
        "log_task",
        4096,  /* Stack size */
        NULL,  /* Parameters */
        tskIDLE_PRIORITY + 3,  /* Priority */
        &s_log_task_handle,
        SYSTEM_MANAGER_TASK_CORE  /* Core */
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create log processing task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "System manager initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start the system manager and all subsystems
 */
esp_err_t system_manager_start(void) {
    ESP_LOGI(TAG, "Starting system manager");
    
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_system_state.running) {
        ESP_LOGW(TAG, "System manager already running");
        return ESP_OK;
    }
    
    /* Create the main system manager task */
    BaseType_t task_created = xTaskCreatePinnedToCore(
        system_manager_task,
        "sys_manager",
        SYSTEM_MANAGER_TASK_STACK_SIZE,
        NULL,  /* Parameters */
        SYSTEM_MANAGER_TASK_PRIORITY,
        &s_system_manager_task_handle,
        SYSTEM_MANAGER_TASK_CORE
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create system manager task");
        return ESP_FAIL;
    }
    
    s_system_state.running = true;
    ESP_LOGI(TAG, "System manager started successfully");
    return ESP_OK;
}

/**
 * @brief Stop the system manager and all subsystems
 */
esp_err_t system_manager_stop(void) {
    ESP_LOGI(TAG, "Stopping system manager");
    
    if (!s_system_state.running) {
        ESP_LOGW(TAG, "System manager not running");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Set flags to signal shutdown to tasks */
    xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_SHUTDOWN);
    
    /* Wait for main task to complete shutdown */
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    /* Clean up any resources if needed */
    if (s_system_state.log_file != NULL) {
        fclose(s_system_state.log_file);
        s_system_state.log_file = NULL;
    }
    
    s_system_state.running = false;
    ESP_LOGI(TAG, "System manager stopped successfully");
    return ESP_OK;
}

/**
 * @brief Set the system operating mode
 */
esp_err_t system_manager_set_mode(bess_system_mode_t mode) {
    ESP_LOGI(TAG, "Setting system mode to %d", mode);
    
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Check mode validity */
    if (mode < BESS_MODE_STANDBY || mode > BESS_MODE_EMERGENCY_SHUTDOWN) {
        ESP_LOGE(TAG, "Invalid mode: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Don't allow mode changes when in emergency shutdown or fault, 
       except to transition back to standby after fault resolution */
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        if (s_system_state.info.mode == BESS_MODE_EMERGENCY_SHUTDOWN &&
            mode != BESS_MODE_STANDBY) {
            xSemaphoreGive(s_system_data_mutex);
            ESP_LOGE(TAG, "System is in emergency shutdown, cannot change mode");
            return ESP_ERR_INVALID_STATE;
        }
        
        if (s_system_state.info.mode == BESS_MODE_FAULT &&
            mode != BESS_MODE_STANDBY) {
            xSemaphoreGive(s_system_data_mutex);
            ESP_LOGE(TAG, "System is in fault state, resolve faults before changing mode");
            return ESP_ERR_INVALID_STATE;
        }
        
        /* Special handling for emergency shutdown mode */
        if (mode == BESS_MODE_EMERGENCY_SHUTDOWN) {
            xSemaphoreGive(s_system_data_mutex);
            return system_manager_emergency_shutdown("Mode change to emergency shutdown");
        }
        
        /* Update system mode */
        bess_system_mode_t old_mode = s_system_state.info.mode;
        s_system_state.info.mode = mode;
        xSemaphoreGive(s_system_data_mutex);
        
        /* Create event for mode change */
        bess_event_data_t event;
        event.event_id = 0;  /* Will be assigned in notify_event_callbacks */
        event.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
        event.event_type = BESS_EVENT_STATE_CHANGE;
        event.severity = 0;  /* Normal severity */
        event.module_id = 0;  /* Not module-specific */
        event.data = (old_mode << 16) | mode;  /* Pack old and new modes into data field */
        
        /* Notify callbacks about mode change */
        notify_event_callbacks(event);
        
        ESP_LOGI(TAG, "System mode changed from %d to %d", old_mode, mode);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for mode change");
    return ESP_FAIL;
}

/**
 * @brief Get the current system information
 */
esp_err_t system_manager_get_info(bess_system_info_t *info) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (info == NULL) {
        ESP_LOGE(TAG, "Invalid parameter (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        /* Update system information with latest data before returning */
        s_system_state.info.uptime = (uint32_t)(esp_timer_get_time() / 1000000);
        
        /* Copy system information to output */
        memcpy(info, &s_system_state.info, sizeof(bess_system_info_t));
        xSemaphoreGive(s_system_data_mutex);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for get_info");
    return ESP_FAIL;
}

/**
 * @brief Get communication statistics
 */
esp_err_t system_manager_get_comm_stats(bess_comm_stats_t *stats) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (stats == NULL) {
        ESP_LOGE(TAG, "Invalid parameter (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        /* Copy communication statistics to output */
        memcpy(stats, &s_system_state.comm_stats, sizeof(bess_comm_stats_t));
        xSemaphoreGive(s_system_data_mutex);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for get_comm_stats");
    return ESP_FAIL;
}

/**
 * @brief Reset communication statistics
 */
esp_err_t system_manager_reset_comm_stats(void) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        /* Reset all communication statistics */
        memset(&s_system_state.comm_stats, 0, sizeof(bess_comm_stats_t));
        xSemaphoreGive(s_system_data_mutex);
        ESP_LOGI(TAG, "Communication statistics reset");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for reset_comm_stats");
    return ESP_FAIL;
}

/**
 * @brief Configure the logging system
 */
esp_err_t system_manager_configure_logging(uint32_t level, uint32_t destinations) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) == pdTRUE) {
        /* Update logging configuration */
        s_system_state.config.log_level = level;
        s_system_state.config.log_destinations = destinations;
        xSemaphoreGive(s_config_mutex);
        
        ESP_LOGI(TAG, "Logging configured: level=%lu, destinations=0x%lx", level, destinations);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take config mutex for configure_logging");
    return ESP_FAIL;
}

/**
 * @brief Log a message to configured destinations
 */
esp_err_t system_manager_log(const char *tag, uint32_t level, const char *format, ...) {
    if (!s_system_state.initialized) {
        /* Allow logging even before full initialization for early debug */
        ESP_LOGD(TAG, "Logging before initialization");
    }
    
    if (tag == NULL || format == NULL) {
        ESP_LOGE(TAG, "Invalid logging parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Check if this log level should be logged based on configuration */
    uint32_t config_log_level = ESP_LOG_INFO;  /* Default to INFO if not initialized */
    if (s_system_state.initialized) {
        if (xSemaphoreTake(s_config_mutex, 0) == pdTRUE) {
            config_log_level = s_system_state.config.log_level;
            xSemaphoreGive(s_config_mutex);
        }
    }
    
    if (level > config_log_level) {
        return ESP_OK;  /* Skip logging for levels higher than configured */
    }
    
    /* Format the log message */
    log_queue_item_t log_item;
    va_list args;
    va_start(args, format);
    vsnprintf(log_item.message, SYSTEM_MANAGER_MAX_LOG_SIZE, format, args);
    va_end(args);
    
    /* Set metadata */
    log_item.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    log_item.level = level;
    strncpy(log_item.tag, tag, sizeof(log_item.tag) - 1);
    log_item.tag[sizeof(log_item.tag) - 1] = '\0';
    
    /* Always log to ESP_LOG for console output */
    switch (level) {
        case ESP_LOG_ERROR:
            ESP_LOGE(tag, "%s", log_item.message);
            break;
        case ESP_LOG_WARN:
            ESP_LOGW(tag, "%s", log_item.message);
            break;
        case ESP_LOG_INFO:
            ESP_LOGI(tag, "%s", log_item.message);
            break;
        case ESP_LOG_DEBUG:
            ESP_LOGD(tag, "%s", log_item.message);
            break;
        case ESP_LOG_VERBOSE:
            ESP_LOGV(tag, "%s", log_item.message);
            break;
        default:
            ESP_LOGI(tag, "%s", log_item.message);
            break;
    }
    
    /* If log queue is initialized, queue for processing by log task */
    if (s_log_queue != NULL) {
        if (xQueueSend(s_log_queue, &log_item, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Log queue full, message dropped");
            return ESP_ERR_NO_MEM;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Register a callback for system events
 */
esp_err_t system_manager_register_event_callback(
    uint32_t event_mask,
    bess_event_callback_t callback,
    void *user_data) {
    
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (callback == NULL) {
        ESP_LOGE(TAG, "Invalid callback (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Find an empty slot or check if callback already registered */
    int empty_slot = -1;
    for (int i = 0; i < SYSTEM_MANAGER_MAX_EVENT_CALLBACKS; i++) {
        if (s_event_callbacks[i].active && s_event_callbacks[i].callback == callback) {
            /* Update existing callback */
            s_event_callbacks[i].event_mask = event_mask;
            s_event_callbacks[i].user_data = user_data;
            ESP_LOGI(TAG, "Updated event callback at slot %d", i);
            return ESP_OK;
        }
        
        if (!s_event_callbacks[i].active && empty_slot == -1) {
            empty_slot = i;
        }
    }
    
    if (empty_slot != -1) {
        /* Register new callback in empty slot */
        s_event_callbacks[empty_slot].active = true;
        s_event_callbacks[empty_slot].event_mask = event_mask;
        s_event_callbacks[empty_slot].callback = callback;
        s_event_callbacks[empty_slot].user_data = user_data;
        ESP_LOGI(TAG, "Registered event callback at slot %d", empty_slot);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "No free slots for event callbacks");
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Unregister a previously registered event callback
 */
esp_err_t system_manager_unregister_event_callback(bess_event_callback_t callback) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (callback == NULL) {
        ESP_LOGE(TAG, "Invalid callback (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Find the callback and remove it */
    for (int i = 0; i < SYSTEM_MANAGER_MAX_EVENT_CALLBACKS; i++) {
        if (s_event_callbacks[i].active && s_event_callbacks[i].callback == callback) {
            s_event_callbacks[i].active = false;
            s_event_callbacks[i].callback = NULL;
            s_event_callbacks[i].event_mask = 0;
            s_event_callbacks[i].user_data = NULL;
            ESP_LOGI(TAG, "Unregistered event callback from slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Callback not found for unregistration");
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Clear a system alarm
 */
esp_err_t system_manager_clear_alarm(bess_alarm_type_t alarm_type) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (alarm_type <= BESS_ALARM_NONE || alarm_type >= BESS_ALARM_EMERGENCY_STOP_ACTIVATED + 1) {
        ESP_LOGE(TAG, "Invalid alarm type: %d", alarm_type);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Clearing alarm: type=%d", alarm_type);
    
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        /* Clear alarm bit in active alarms */
        s_system_state.info.active_alarms &= ~(1 << (alarm_type - 1));
        
        /* Check if all alarms are cleared */
        if (s_system_state.info.active_alarms == 0 && 
            s_system_state.info.mode == BESS_MODE_FAULT) {
            /* Automatically transition back to standby if all alarms cleared */
            s_system_state.info.mode = BESS_MODE_STANDBY;
            ESP_LOGI(TAG, "All alarms cleared, transitioning to STANDBY mode");
            
            /* Create event for mode change */
            bess_event_data_t event;
            event.event_id = 0;
            event.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
            event.event_type = BESS_EVENT_STATE_CHANGE;
            event.severity = 0;
            event.module_id = 0;
            event.data = (BESS_MODE_FAULT << 16) | BESS_MODE_STANDBY;
            
            xSemaphoreGive(s_system_data_mutex);
            
            /* Notify callbacks about mode change */
            notify_event_callbacks(event);
        } else {
            xSemaphoreGive(s_system_data_mutex);
        }
        
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for clear_alarm");
    return ESP_FAIL;
}

/**
 * @brief Get active alarms
 */
esp_err_t system_manager_get_active_alarms(uint32_t *alarm_bitmap) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (alarm_bitmap == NULL) {
        ESP_LOGE(TAG, "Invalid parameter (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        *alarm_bitmap = s_system_state.info.active_alarms;
        xSemaphoreGive(s_system_data_mutex);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for get_active_alarms");
    return ESP_FAIL;
}

/**
 * @brief Run system diagnostics
 */
esp_err_t system_manager_run_diagnostics(bool detailed) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Running system diagnostics (detailed=%s)", detailed ? "true" : "false");
    
    /* Check subsystem handlers */
    if (s_bms_handler == NULL) {
        ESP_LOGW(TAG, "Diagnostics: BMS handler not initialized");
    }
    
    if (s_modbus_handler == NULL) {
        ESP_LOGW(TAG, "Diagnostics: Modbus handler not initialized");
    }
    
    if (s_canbus_handler == NULL) {
        ESP_LOGW(TAG, "Diagnostics: CANBus handler not initialized");
    }
    
    if (s_aws_handler == NULL) {
        ESP_LOGW(TAG, "Diagnostics: AWS handler not initialized");
    }
    
    /* Run BMS diagnostics if available */
    if (s_bms_handler != NULL) {
        ESP_LOGI(TAG, "Running BMS diagnostics");
        battery_manager_run_diagnostics(detailed);
    }
    
    /* Run Modbus diagnostics if available */
    if (s_modbus_handler != NULL) {
        ESP_LOGI(TAG, "Running Modbus diagnostics");
        // Call Modbus diagnostics function
    }
    
    /* Run CANBus diagnostics if available */
    if (s_canbus_handler != NULL) {
        ESP_LOGI(TAG, "Running CANBus diagnostics");
        // Call CANBus diagnostics function
    }
    
    /* Check SD card status */
    if (s_system_state.sd_card_mounted) {
        ESP_LOGI(TAG, "Diagnostics: SD card is mounted");
    } else {
        ESP_LOGW(TAG, "Diagnostics: SD card is not mounted");
    }
    
    /* Additional detailed diagnostics */
    if (detailed) {
        /* Check system heap usage */
        ESP_LOGI(TAG, "Diagnostics: Free heap size: %lu bytes", esp_get_free_heap_size());
        
        /* Check task stack usage */
        if (s_system_manager_task_handle != NULL) {
            ESP_LOGI(TAG, "Diagnostics: System manager task high water mark: %lu bytes", 
                    uxTaskGetStackHighWaterMark(s_system_manager_task_handle));
        }
        
        if (s_log_task_handle != NULL) {
            ESP_LOGI(TAG, "Diagnostics: Log task high water mark: %lu bytes", 
                    uxTaskGetStackHighWaterMark(s_log_task_handle));
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Synchronize system time with an external time source
 */
esp_err_t system_manager_sync_time(void) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Synchronizing system time");
    
    /* Initialize SNTP if not already done */
    esp_err_t ret = init_time_sync();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize time synchronization");
        return ret;
    }
    
    /* Trigger SNTP update */
    sntp_restart();
    
    /* Update timestamp of last sync attempt */
    s_system_state.last_time_sync = esp_timer_get_time() / 1000000;
    
    ESP_LOGI(TAG, "Time synchronization requested");
    return ESP_OK;
}

/**
 * @brief Update system configuration
 */
esp_err_t system_manager_update_config(const bess_system_config_t *config) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Updating system configuration");
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) == pdTRUE) {
        /* Store old values of parameters that can't be changed at runtime */
        uint16_t old_module_count = s_system_state.config.module_count;
        
        /* Update configuration */
        memcpy(&s_system_state.config, config, sizeof(bess_system_config_t));
        
        /* Restore values that can't be changed at runtime */
        if (old_module_count != config->module_count) {
            ESP_LOGW(TAG, "Module count cannot be changed at runtime, keeping old value");
            s_system_state.config.module_count = old_module_count;
        }
        
        xSemaphoreGive(s_config_mutex);
        
        /* Apply configuration changes to subsystems */
        
        /* BMS thresholds */
        if (s_bms_handler != NULL) {
            /* Update BMS thresholds */
            // Call BMS configuration functions
        }
        
        /* Modbus settings */
        if (s_modbus_handler != NULL) {
            /* Update Modbus settings */
            // Call Modbus configuration functions
        }
        
        /* CANBus settings */
        if (s_canbus_handler != NULL) {
            /* Update CANBus settings */
            // Call CANBus configuration functions
        }
        
        ESP_LOGI(TAG, "System configuration updated successfully");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take config mutex for update_config");
    return ESP_FAIL;
}

/**
 * @brief Save current configuration to non-volatile storage
 */
esp_err_t system_manager_save_config(void) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Saving system configuration to NVS");
    
    /* Take a local copy of the configuration */
    bess_system_config_t config_copy;
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&config_copy, &s_system_state.config, sizeof(bess_system_config_t));
        xSemaphoreGive(s_config_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take config mutex for save_config");
        return ESP_FAIL;
    }
    
    /* Save to NVS */
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(SYSTEM_MANAGER_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_blob(nvs_handle, SYSTEM_MANAGER_NVS_CONFIG_KEY, 
                       &config_copy, sizeof(bess_system_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    nvs_close(nvs_handle);
    
    /* Also save to SD card if available */
    if (s_system_state.sd_card_mounted) {
        ESP_LOGI(TAG, "Saving config to SD card");
        
        /* Ensure config directory exists */
        create_directory(SYSTEM_MANAGER_SD_MOUNT_POINT "/config");
        
        /* Write config to file */
        FILE *config_file = fopen(SYSTEM_MANAGER_CONFIG_FILENAME, "wb");
        if (config_file != NULL) {
            size_t written = fwrite(&config_copy, 1, sizeof(bess_system_config_t), config_file);
            fclose(config_file);
            
            if (written != sizeof(bess_system_config_t)) {
                ESP_LOGW(TAG, "Failed to write complete config to SD card");
            } else {
                ESP_LOGI(TAG, "Config saved to SD card successfully");
            }
        } else {
            ESP_LOGW(TAG, "Failed to open config file on SD card");
        }
    }
    
    ESP_LOGI(TAG, "System configuration saved successfully");
    return ESP_OK;
}

/**
 * @brief Load configuration from non-volatile storage
 */
esp_err_t system_manager_load_config(bess_system_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid parameter (NULL)");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Loading system configuration from NVS");
    
    /* Load from NVS */
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(SYSTEM_MANAGER_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    size_t required_size = sizeof(bess_system_config_t);
    err = nvs_get_blob(nvs_handle, SYSTEM_MANAGER_NVS_CONFIG_KEY, config, &required_size);
    nvs_close(nvs_handle);
    
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "No saved configuration found in NVS");
        } else {
            ESP_LOGE(TAG, "Failed to read config from NVS: %s", esp_err_to_name(err));
        }
        
        /* Try to load from SD card if NVS fails */
        if (s_system_state.sd_card_mounted) {
            ESP_LOGI(TAG, "Attempting to load config from SD card");
            
            FILE *config_file = fopen(SYSTEM_MANAGER_CONFIG_FILENAME, "rb");
            if (config_file != NULL) {
                size_t read = fread(config, 1, sizeof(bess_system_config_t), config_file);
                fclose(config_file);
                
                if (read != sizeof(bess_system_config_t)) {
                    ESP_LOGW(TAG, "Failed to read complete config from SD card");
                    return ESP_FAIL;
                }
                
                ESP_LOGI(TAG, "Config loaded from SD card successfully");
                return ESP_OK;
            } else {
                ESP_LOGW(TAG, "Failed to open config file on SD card");
                return ESP_FAIL;
            }
        }
        
        return err;
    }
    
    ESP_LOGI(TAG, "System configuration loaded successfully");
    return ESP_OK;
}

/**
 * @brief Perform a controlled system shutdown
 */
esp_err_t system_manager_shutdown(const char *reason) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "System shutdown requested: %s", reason ? reason : "No reason provided");
    
    /* Set event flag to signal shutdown to system manager task */
    xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_SHUTDOWN);
    
    /* Log shutdown event */
    system_manager_log(TAG, ESP_LOG_WARN, "System shutdown initiated: %s", 
                     reason ? reason : "No reason provided");
    
    return ESP_OK;
}

/**
 * @brief Perform a controlled system reboot
 */
esp_err_t system_manager_reboot(const char *reason) {
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "System reboot requested: %s", reason ? reason : "No reason provided");
    
    /* Set event flag to signal reboot to system manager task */
    xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_REBOOT);
    
    /* Log reboot event */
    system_manager_log(TAG, ESP_LOG_WARN, "System reboot initiated: %s", 
                     reason ? reason : "No reason provided");
    
    return ESP_OK;
}

/**
 * @brief Execute emergency shutdown procedure
 */
esp_err_t system_manager_emergency_shutdown(const char *reason) {
    ESP_LOGE(TAG, "EMERGENCY SHUTDOWN: %s", reason ? reason : "No reason provided");
    
    /* Set emergency mode even if not fully initialized */
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        s_system_state.info.mode = BESS_MODE_EMERGENCY_SHUTDOWN;
        xSemaphoreGive(s_system_data_mutex);
    }
    
    /* Set event flag to signal emergency to system manager task */
    if (s_system_event_group != NULL) {
        xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_EMERGENCY);
    }
    
    /* Log emergency event with highest priority */
    system_manager_log(TAG, ESP_LOG_ERROR, "EMERGENCY SHUTDOWN INITIATED: %s", 
                     reason ? reason : "No reason provided");
    
    /* If system not properly initialized, execute emergency procedure directly */
    if (!s_system_state.initialized || s_system_event_group == NULL) {
        emergency_shutdown_handler();
    }
    
    return ESP_OK;
}

/**
 * @brief Trigger a system alarm
 */
esp_err_t system_manager_trigger_alarm(
    bess_alarm_type_t alarm_type,
    uint16_t module_id,
    uint32_t data) {
    
    if (!s_system_state.initialized) {
        ESP_LOGE(TAG, "System manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (alarm_type <= BESS_ALARM_NONE || alarm_type >= BESS_ALARM_EMERGENCY_STOP_ACTIVATED + 1) {
        ESP_LOGE(TAG, "Invalid alarm type: %d", alarm_type);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGW(TAG, "Alarm triggered: type=%d, module=%u, data=0x%lx", 
             alarm_type, module_id, data);
    
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        /* Set alarm bit in active alarms */
        s_system_state.info.active_alarms |= (1 << (alarm_type - 1));
        xSemaphoreGive(s_system_data_mutex);
        
        /* Create event for alarm */
        bess_event_data_t event;
        event.event_id = 0;  /* Will be assigned in notify_event_callbacks */
        event.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
        event.event_type = BESS_EVENT_ERROR;
        event.severity = alarm_type;  /* Use alarm type as severity */
        event.module_id = module_id;
        event.data = data;
        
        /* Notify callbacks about alarm */
        notify_event_callbacks(event);
        
        /* Handle critical alarms */
        if (alarm_type == BESS_ALARM_THERMAL_RUNAWAY ||
            alarm_type == BESS_ALARM_EMERGENCY_STOP_ACTIVATED) {
            return system_manager_emergency_shutdown("Critical alarm triggered");
        }
        
        /* Transition to fault mode for most alarms */
        if (alarm_type != BESS_ALARM_SOC_LOW &&
            alarm_type != BESS_ALARM_COMMUNICATION_ERROR &&
            s_system_state.info.mode != BESS_MODE_FAULT) {
            system_manager_set_mode(BESS_MODE_FAULT);
        }
        
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to take data mutex for trigger_alarm");
    return ESP_FAIL;
}

/**
 * @brief Main system manager task function
 */
static void system_manager_task(void *pvParameters) {
    ESP_LOGI(TAG, "System manager task started");
    
    /* Initialize task watchdog */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    /* Initialize subsystems */
    esp_err_t ret = init_subsystems();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize subsystems: %s", esp_err_to_name(ret));
        /* Continue with limited functionality */
    }
    
    /* Mark initialization complete */
    xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_INIT_COMPLETE);
    
    TickType_t last_update_time = xTaskGetTickCount();
    uint32_t loop_counter = 0;
    
    /* Main task loop */
    while (1) {
        /* Reset watchdog */
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        
        /* Check event flags */
        EventBits_t events = xEventGroupGetBits(s_system_event_group);
        
        if (events & SYSTEM_EVENT_EMERGENCY) {
            ESP_LOGE(TAG, "Emergency shutdown requested");
            emergency_shutdown_handler();
            /* This should never return, but just in case */
            continue;
        }
        
        if (events & SYSTEM_EVENT_SHUTDOWN) {
            ESP_LOGW(TAG, "Shutdown requested, stopping system");
            break;
        }
        
        if (events & SYSTEM_EVENT_REBOOT) {
            ESP_LOGW(TAG, "Reboot requested, stopping system");
            vTaskDelay(pdMS_TO_TICKS(1000));  /* Brief delay for logs to flush */
            esp_restart();
        }
        
        /* Process any pending events */
        process_events();
        
        /* Update system status */
        update_system_status();
        
        /* Check if time synchronization is needed */
        uint64_t current_time = esp_timer_get_time() / 1000000;
        if (current_time - s_system_state.last_time_sync > SYSTEM_MANAGER_TIME_SYNC_INTERVAL_SEC) {
            ESP_LOGI(TAG, "Periodic time sync");
            system_manager_sync_time();
        }
        
        /* Perform periodic actions */
        loop_counter++;
        
        /* Every 60 seconds, run basic diagnostics */
        if (loop_counter % 60 == 0) {
            system_manager_run_diagnostics(false);
        }
        
        /* Check if log file needs rotation (daily) */
        if (s_system_state.sd_card_mounted && s_system_state.log_file != NULL) {
            time_t now;
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            
            /* If it's midnight (or close to it), rotate log file */
            if (timeinfo.tm_hour == 0 && timeinfo.tm_min < 5) {
                rotate_log_file();
            }
        }
        
        /* Sleep until next cycle */
        vTaskDelayUntil(&last_update_time, pdMS_TO_TICKS(SYSTEM_MANAGER_MAIN_LOOP_INTERVAL_MS));
    }
    
    /* Clean up before exiting */
    ESP_LOGI(TAG, "System manager task stopping");
    
    /* Close log file if open */
    if (s_system_state.log_file != NULL) {
        fclose(s_system_state.log_file);
        s_system_state.log_file = NULL;
    }
    
    /* Unregister from task watchdog */
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    
    /* Set running flag to false */
    s_system_state.running = false;
    
    /* Delete the task */
    vTaskDelete(NULL);
}

/**
 * @brief Log processing task function
 */
static void log_processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "Log processing task started");
    
    log_queue_item_t log_item;
    while (1) {
        /* Wait for log items in the queue */
        if (xQueueReceive(s_log_queue, &log_item, portMAX_DELAY) == pdTRUE) {
            /* Check which destinations are configured */
            uint32_t destinations = BESS_LOG_CONSOLE;  /* Default to console only */
            
            if (xSemaphoreTake(s_config_mutex, 0) == pdTRUE) {
                destinations = s_system_state.config.log_destinations;
                xSemaphoreGive(s_config_mutex);
            }
            
            /* Write to SD card if enabled and available */
            if ((destinations & BESS_LOG_SD_CARD) && s_system_state.sd_card_mounted) {
                write_log_to_sd(&log_item);
            }
            
            /* Send to CloudWatch if enabled and connected */
            if ((destinations & BESS_LOG_CLOUDWATCH) && (s_system_state.info.status_flags & BESS_STATUS_AWS_CONNECTED)) {
                send_log_to_cloudwatch(&log_item);
            }
        }
        
        /* Yield to other tasks periodically */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Time synchronization callback function
 */
static void time_sync_callback(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronized with NTP server");
    
    /* Update system state to indicate time is synced */
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        s_system_state.info.status_flags |= BESS_STATUS_TIME_SYNCED;
        xSemaphoreGive(s_system_data_mutex);
    }
    
    /* Set event bit for time sync complete */
    xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_TIME_SYNCED);
    
    /* Update log file name based on current date if needed */
    if (s_system_state.sd_card_mounted) {
        rotate_log_file();
    }
}

/**
 * @brief Initialize all subsystems
 */
static esp_err_t init_subsystems(void) {
    esp_err_t ret;
    
    /* Initialize NVS if not already done */
    ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        /* Continue anyway */
    }
    
    /* Initialize SD card */
    ret = init_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        /* Continue without SD card */
    } else {
        xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_SD_READY);
        s_system_state.info.status_flags |= BESS_STATUS_SD_CARD_PRESENT;
    }
    
    /* Initialize logging system */
    ret = init_logging();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize logging: %s", esp_err_to_name(ret));
        /* Continue with limited logging */
    } else {
        s_system_state.info.status_flags |= BESS_STATUS_LOGGING_ACTIVE;
    }
    
    /* Initialize time synchronization */
    ret = init_time_sync();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize time sync: %s", esp_err_to_name(ret));
        /* Continue without time sync */
    }
    
    /* Initialize BMS subsystem */
    ret = init_bms_subsystem();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMS: %s", esp_err_to_name(ret));
        /* This is critical, but continue with limited functionality */
    } else {
        s_system_state.info.status_flags |= BESS_STATUS_BMS_ACTIVE;
        xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_BMS_READY);
    }
    
    /* Initialize communication interfaces */
    ret = init_communication_interfaces();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize communication interfaces: %s", esp_err_to_name(ret));
        /* Continue with limited connectivity */
    } else {
        xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_COMM_READY);
    }
    
    /* Initialize AWS interface */
    ret = init_aws_interface();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize AWS interface: %s", esp_err_to_name(ret));
        /* Continue without cloud connectivity */
    } else {
        xEventGroupSetBits(s_system_event_group, SYSTEM_EVENT_CLOUD_READY);
    }
    
    return ESP_OK;
}

/**
 * @brief Initialize SD card
 */
static esp_err_t init_sd_card(void) {
    ESP_LOGI(TAG, "Initializing SD card");
    
    /* Check if already mounted */
    if (s_system_state.sd_card_mounted) {
        ESP_LOGW(TAG, "SD card already mounted");
        return ESP_OK;
    }
    
    /* Configure SPI bus for SD card */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = SD_PIN_NUM_MISO;
    slot_config.gpio_mosi = SD_PIN_NUM_MOSI;
    slot_config.gpio_sck  = SD_PIN_NUM_CLK;
    slot_config.gpio_cs   = SD_PIN_NUM_CS;
    
    /* Mount the filesystem */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SYSTEM_MANAGER_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount the SD card filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the SD card: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    
    /* Card mounted successfully */
    s_system_state.sd_card_mounted = true;
    
    /* Create necessary directories */
    create_directory(SYSTEM_MANAGER_SD_MOUNT_POINT "/logs");
    create_directory(SYSTEM_MANAGER_SD_MOUNT_POINT "/config");
    create_directory(SYSTEM_MANAGER_SD_MOUNT_POINT "/data");
    
    /* Log card info */
    sdmmc_card_print_info(stdout, card);
    
    ESP_LOGI(TAG, "SD card initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize logging system
 */
static esp_err_t init_logging(void) {
    ESP_LOGI(TAG, "Initializing logging system");
    
    /* Get current time for log filename */
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    /* Create log filename with date */
    char log_path[64];
    strftime(log_path, sizeof(log_path), SYSTEM_MANAGER_LOG_FILENAME_FORMAT, &timeinfo);
    
    /* Try to open log file if SD card is mounted */
    if (s_system_state.sd_card_mounted) {
        /* Create logs directory if it doesn't exist */
        create_directory(SYSTEM_MANAGER_SD_MOUNT_POINT "/logs");
        
        /* Open log file */
        s_system_state.log_file = fopen(log_path, "a+");
        if (s_system_state.log_file == NULL) {
            ESP_LOGE(TAG, "Failed to open log file: %s", log_path);
        } else {
            ESP_LOGI(TAG, "Logging to file: %s", log_path);
            strncpy(s_system_state.log_filename, log_path, sizeof(s_system_state.log_filename) - 1);
            s_system_state.log_filename[sizeof(s_system_state.log_filename) - 1] = '\0';
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Initialize NVS (Non-Volatile Storage)
 */
static esp_err_t init_nvs(void) {
    ESP_LOGI(TAG, "Initializing NVS");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs to be erased");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "NVS initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize time synchronization
 */
static esp_err_t init_time_sync(void) {
    ESP_LOGI(TAG, "Initializing time synchronization");
    
    /* Initialize SNTP */
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_callback);
    sntp_init();
    
    ESP_LOGI(TAG, "Time synchronization initialized");
    return ESP_OK;
}

/**
 * @brief Process pending events from the event queue
 */
static esp_err_t process_events(void) {
    bess_event_data_t event;
    
    /* Process up to 10 events per call to avoid blocking too long */
    for (int i = 0; i < 10; i++) {
        if (xQueueReceive(s_event_queue, &event, 0) != pdTRUE) {
            /* No more events in queue */
            break;
        }
        
        /* Process the event */
        ESP_LOGD(TAG, "Processing event: type=%d, module=%u, data=0x%lx", 
                event.event_type, event.module_id, event.data);
        
        /* Check for critical events that need immediate action */
        if (event.event_type == BESS_EVENT_ERROR && 
            (event.severity == BESS_ALARM_THERMAL_RUNAWAY || 
             event.severity == BESS_ALARM_EMERGENCY_STOP_ACTIVATED)) {
            ESP_LOGE(TAG, "Critical event detected, initiating emergency shutdown");
            system_manager_emergency_shutdown("Critical event");
            return ESP_OK;  /* Stop processing events */
        }
        
        /* Notify registered callbacks */
        notify_event_callbacks(event);
    }
    
    return ESP_OK;
}

/**
 * @brief Write log to SD card
 */
static esp_err_t write_log_to_sd(const log_queue_item_t *log_item) {
    if (!s_system_state.sd_card_mounted || s_system_state.log_file == NULL) {
        return ESP_FAIL;
    }
    
    /* Get current time */
    time_t now = log_item->timestamp;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    /* Format time string */
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    /* Format log level string */
    const char *level_str;
    switch (log_item->level) {
        case ESP_LOG_ERROR:
            level_str = "ERROR";
            break;
        case ESP_LOG_WARN:
            level_str = "WARN";
            break;
        case ESP_LOG_INFO:
            level_str = "INFO";
            break;
        case ESP_LOG_DEBUG:
            level_str = "DEBUG";
            break;
        case ESP_LOG_VERBOSE:
            level_str = "VERBOSE";
            break;
        default:
            level_str = "UNKNOWN";
            break;
    }
    
    /* Write to log file */
    fprintf(s_system_state.log_file, "[%s] [%s] [%s] %s\n", 
            time_str, level_str, log_item->tag, log_item->message);
    
    /* Flush to ensure data is written to disk */
    fflush(s_system_state.log_file);
    
    return ESP_OK;
}

/**
 * @brief Send log to AWS CloudWatch
 */
static esp_err_t send_log_to_cloudwatch(const log_queue_item_t *log_item) {
    if (s_aws_handler == NULL) {
        return ESP_FAIL;
    }
    
    /* Check if this log level should be sent to CloudWatch */
    if (log_item->level > ESP_LOG_INFO) {
        /* Skip DEBUG and VERBOSE logs for CloudWatch to reduce bandwidth */
        return ESP_OK;
    }
    
    /* Get current time */
    time_t now = log_item->timestamp;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    /* Format time string */
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    /* Format log level string */
    const char *level_str;
    switch (log_item->level) {
        case ESP_LOG_ERROR:
            level_str = "ERROR";
            break;
        case ESP_LOG_WARN:
            level_str = "WARN";
            break;
        case ESP_LOG_INFO:
            level_str = "INFO";
            break;
        case ESP_LOG_DEBUG:
            level_str = "DEBUG";
            break;
        case ESP_LOG_VERBOSE:
            level_str = "VERBOSE";
            break;
        default:
            level_str = "UNKNOWN";
            break;
    }
    
    /* Prepare CloudWatch log message */
    char cloudwatch_message[SYSTEM_MANAGER_MAX_LOG_SIZE + 100];
    snprintf(cloudwatch_message, sizeof(cloudwatch_message), 
             "{ \"timestamp\": \"%s\", \"level\": \"%s\", \"tag\": \"%s\", \"message\": \"%s\" }", 
             time_str, level_str, log_item->tag, log_item->message);
    
    /* Send to CloudWatch via AWS IoT */
    char topic[64];
    snprintf(topic, sizeof(topic), "bess/%s/logs", s_system_state.info.system_id);
    
    esp_err_t result = aws_interface_publish(topic, cloudwatch_message, strlen(cloudwatch_message), 0);
    
    /* Update statistics */
    if (result == ESP_OK) {
        if (xSemaphoreTake(s_system_data_mutex, 0) == pdTRUE) {
            s_system_state.comm_stats.cloud_tx_count++;
            xSemaphoreGive(s_system_data_mutex);
        }
    } else {
        if (xSemaphoreTake(s_system_data_mutex, 0) == pdTRUE) {
            s_system_state.comm_stats.cloud_error_count++;
            xSemaphoreGive(s_system_data_mutex);
        }
    }
    
    return result;
}

/**
 * @brief Notify registered callbacks about an event
 */
static void notify_event_callbacks(bess_event_data_t event) {
    static uint32_t next_event_id = 1;
    
    /* Assign a unique event ID */
    event.event_id = next_event_id++;
    
    /* Set timestamp if not already set */
    if (event.timestamp == 0) {
        event.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    }
    
    /* Call all registered callbacks that match the event type */
    for (int i = 0; i < SYSTEM_MANAGER_MAX_EVENT_CALLBACKS; i++) {
        if (s_event_callbacks[i].active && 
            (s_event_callbacks[i].event_mask & (1 << event.event_type))) {
            /* Call the callback */
            s_event_callbacks[i].callback(event, s_event_callbacks[i].user_data);
        }
    }
}

/**
 * @brief Handle emergency shutdown procedure
 */
static void emergency_shutdown_handler(void) {
    ESP_LOGE(TAG, "EMERGENCY SHUTDOWN HANDLER ACTIVATED");
    
    /* Set emergency mode */
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        s_system_state.info.mode = BESS_MODE_EMERGENCY_SHUTDOWN;
        xSemaphoreGive(s_system_data_mutex);
    }
    
    /* Log the emergency */
    system_manager_log(TAG, ESP_LOG_ERROR, "EMERGENCY SHUTDOWN IN PROGRESS");
    
    /* If BMS is available, command emergency shutdown */
    if (s_bms_handler != NULL) {
        ESP_LOGI(TAG, "Commanding BMS emergency shutdown");
        battery_manager_emergency_shutdown();
    }
    
    /* If logging is active, flush logs */
    if (s_system_state.log_file != NULL) {
        fflush(s_system_state.log_file);
        fsync(fileno(s_system_state.log_file));
        fclose(s_system_state.log_file);
        s_system_state.log_file = NULL;
    }
    
    /* Wait briefly to allow any critical operations to complete */
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    /* Trigger system restart */
    ESP_LOGE(TAG, "Restarting system after emergency shutdown");
    esp_restart();
}

/**
 * @brief Update system status information
 */
static esp_err_t update_system_status(void) {
    /* Only update if BMS is initialized */
    if (s_bms_handler == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    /* Query BMS for battery system status */
    if (xSemaphoreTake(s_system_data_mutex, portMAX_DELAY) == pdTRUE) {
        /* Update State of Charge */
        float system_soc = 0.0f;
        battery_manager_get_soc(&system_soc);
        s_system_state.info.system_soc = system_soc;
        
        /* Update State of Health */
        float system_soh = 0.0f;
        battery_manager_get_soh(&system_soh);
        s_system_state.info.system_soh = system_soh;
        
        /* Update temperatures */
        float max_temp = 0.0f, min_temp = 100.0f, avg_temp = 0.0f;
        thermal_monitor_get_temperatures(&max_temp, &min_temp, &avg_temp);
        s_system_state.info.max_temperature = max_temp;
        s_system_state.info.min_temperature = min_temp;
        s_system_state.info.avg_temperature = avg_temp;
        
        /* Update power output and energy remaining */
        battery_manager_get_system_status(&s_system_state.info.power_output, 
                                        &s_system_state.info.energy_remaining);
        
        /* Update uptime */
        s_system_state.info.uptime = (uint32_t)(esp_timer_get_time() / 1000000);
        
        xSemaphoreGive(s_system_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take data mutex for update_system_status");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Rotate log file (typically at midnight)
 */
static esp_err_t rotate_log_file(void) {
    if (!s_system_state.sd_card_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Get current time */
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    /* Create new log filename with current date */
    char new_log_path[64];
    strftime(new_log_path, sizeof(new_log_path), SYSTEM_MANAGER_LOG_FILENAME_FORMAT, &timeinfo);
    
    /* Check if the log filename is different from current one */
    if (strcmp(new_log_path, s_system_state.log_filename) == 0) {
        /* No need to rotate, same day */
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Rotating log file to: %s", new_log_path);
    
    /* Close existing log file if open */
    if (s_system_state.log_file != NULL) {
        fclose(s_system_state.log_file);
        s_system_state.log_file = NULL;
    }
    
    /* Open new log file */
    s_system_state.log_file = fopen(new_log_path, "a+");
    if (s_system_state.log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open new log file: %s", new_log_path);
        return ESP_FAIL;
    }
    
    /* Update log filename */
    strncpy(s_system_state.log_filename, new_log_path, sizeof(s_system_state.log_filename) - 1);
    s_system_state.log_filename[sizeof(s_system_state.log_filename) - 1] = '\0';
    
    /* Log rotation event to new file */
    fprintf(s_system_state.log_file, "--- Log file created ---\n");
    fflush(s_system_state.log_file);
    
    return ESP_OK;
}

/**
 * @brief Create directory if it doesn't exist
 */
static esp_err_t create_directory(const char *dir) {
    struct stat st;
    if (stat(dir, &st) == 0) {
        /* Directory already exists */
        return ESP_OK;
    }
    
    /* Create the directory */
    if (mkdir(dir, 0755) != 0) {
        ESP_LOGE(TAG, "Failed to create directory: %s", dir);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Created directory: %s", dir);
    return ESP_OK;
}

/**
 * @brief Initialize BMS subsystem
 */
static esp_err_t init_bms_subsystem(void) {
    ESP_LOGI(TAG, "Initializing BMS subsystem");
    
    if (s_bms_handler != NULL) {
        ESP_LOGW(TAG, "BMS already initialized");
        return ESP_OK;
    }
    
    /* Initialize the battery manager */
    esp_err_t ret = battery_manager_init(s_system_state.config.module_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize battery manager: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize the SoC calculator */
    ret = soc_calculator_init(SOC_METHOD_HYBRID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SoC calculator: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize the cell balancer */
    cell_balancer_config_t balancer_config = {
        .voltage_threshold_mv = 50,    /* 50mV threshold */
        .balance_current_ma = 100,     /* 100mA balance current */
        .min_cell_voltage_mv = 2800,   /* 2.8V minimum cell voltage */
        .max_balance_time_ms = 3600000, /* 1 hour maximum */
        .rest_time_ms = 300000,        /* 5 minutes rest time */
        .cells_per_module = 16,        /* 16 cells per module */
        .mode = 1                      /* Passive balancing mode */
    };
    
    ret = cell_balancer_init(&balancer_config, s_system_state.config.module_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize cell balancer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize the thermal monitor */
    ret = thermal_monitor_init(COOLING_METHOD_FORCED_AIR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize thermal monitor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set thermal thresholds from configuration */
    thermal_monitor_set_thresholds(
        s_system_state.config.temperature_warning_threshold,
        s_system_state.config.temperature_critical_threshold,
        s_system_state.config.temperature_emergency_threshold
    );
    
    /* Configure cell balancing threshold */
    battery_manager_set_balance_threshold(s_system_state.config.balance_threshold_mv);
    
    /* Configure cell voltage limits */
    battery_manager_set_cell_voltage_limits(
        s_system_state.config.min_cell_voltage,
        s_system_state.config.max_cell_voltage
    );
    
    /* Start the BMS subsystem */
    ret = battery_manager_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start battery manager: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Store BMS handler for future use */
    s_bms_handler = (void*)1;  /* Just a non-NULL value to indicate initialization */
    
    ESP_LOGI(TAG, "BMS subsystem initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize communication interfaces (Modbus and CANBus)
 */
static esp_err_t init_communication_interfaces(void) {
    ESP_LOGI(TAG, "Initializing communication interfaces");
    
    /* Initialize Modbus interface */
    modbus_config_t modbus_config = {
        .slave_address = s_system_state.config.modbus_slave_address,
        .baud_rate = s_system_state.config.modbus_baud_rate,
        .parity = MODBUS_PARITY_NONE,
        .stop_bits = 1
    };
    
    esp_err_t ret = modbus_interface_init(&modbus_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Modbus interface: %s", esp_err_to_name(ret));
        /* Continue without Modbus */
    } else {
        s_system_state.info.status_flags |= BESS_STATUS_MODBUS_ACTIVE;
        s_modbus_handler = (void*)1;  /* Just a non-NULL value to indicate initialization */
        ESP_LOGI(TAG, "Modbus interface initialized successfully");
    }
    
    /* Initialize CANBus interface */
    canbus_config_t canbus_config = {
        .node_id = s_system_state.config.canbus_node_id,
        .bitrate = s_system_state.config.canbus_bitrate
    };
    
    ret = canbus_interface_init(&canbus_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CANBus interface: %s", esp_err_to_name(ret));
        /* Continue without CANBus */
    } else {
        s_system_state.info.status_flags |= BESS_STATUS_CANBUS_ACTIVE;
        s_canbus_handler = (void*)1;  /* Just a non-NULL value to indicate initialization */
        ESP_LOGI(TAG, "CANBus interface initialized successfully");
    }
    
    /* Register communication event callbacks */
    if (s_modbus_handler != NULL) {
        modbus_interface_register_event_callback(modbus_event_handler, NULL);
    }
    
    if (s_canbus_handler != NULL) {
        canbus_interface_register_event_callback(canbus_event_handler, NULL);
    }
    
    /* Start communication tasks */
    if (s_modbus_handler != NULL) {
        ret = modbus_interface_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start Modbus interface: %s", esp_err_to_name(ret));
            s_system_state.info.status_flags &= ~BESS_STATUS_MODBUS_ACTIVE;
        }
    }
    
    if (s_canbus_handler != NULL) {
        ret = canbus_interface_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start CANBus interface: %s", esp_err_to_name(ret));
            s_system_state.info.status_flags &= ~BESS_STATUS_CANBUS_ACTIVE;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Modbus event handler callback
 */
static void modbus_event_handler(modbus_event_t event, void *user_data) {
    ESP_LOGD(TAG, "Modbus event: %d", event.type);
    
    /* Update communication statistics */
    if (xSemaphoreTake(s_system_data_mutex, 0) == pdTRUE) {
        switch (event.type) {
            case MODBUS_EVENT_TX_COMPLETE:
                s_system_state.comm_stats.modbus_tx_count++;
                break;
                
            case MODBUS_EVENT_RX_COMPLETE:
                s_system_state.comm_stats.modbus_rx_count++;
                break;
                
            case MODBUS_EVENT_ERROR:
                s_system_state.comm_stats.modbus_error_count++;
                
                /* Check for persistent errors */
                if (s_system_state.comm_stats.modbus_error_count > 100) {
                    system_manager_trigger_alarm(
                        BESS_ALARM_COMMUNICATION_ERROR,
                        0,  /* Not module-specific */
                        MODBUS_ERR_BASE + event.data
                    );
                }
                break;
            
            default:
                /* No action for other events */
                break;
        }
        xSemaphoreGive(s_system_data_mutex);
    }
}

/**
 * @brief CANBus event handler callback
 */
static void canbus_event_handler(canbus_event_t event, void *user_data) {
    ESP_LOGD(TAG, "CANBus event: %d", event.type);
    
    /* Update communication statistics */
    if (xSemaphoreTake(s_system_data_mutex, 0) == pdTRUE) {
        switch (event.type) {
            case CANBUS_EVENT_TX_COMPLETE:
                s_system_state.comm_stats.canbus_tx_count++;
                break;
                
            case CANBUS_EVENT_RX_COMPLETE:
                s_system_state.comm_stats.canbus_rx_count++;
                break;
                
            case CANBUS_EVENT_ERROR:
                s_system_state.comm_stats.canbus_error_count++;
                
                /* Check for persistent errors */
                if (s_system_state.comm_stats.canbus_error_count > 100) {
                    system_manager_trigger_alarm(
                        BESS_ALARM_COMMUNICATION_ERROR,
                        0,  /* Not module-specific */
                        CANBUS_ERR_BASE + event.data
                    );
                }
                break;
            
            default:
                /* No action for other events */
                break;
        }
        xSemaphoreGive(s_system_data_mutex);
    }
}

/**
 * @brief Initialize AWS interface for cloud connectivity
 */
static esp_err_t init_aws_interface(void) {
    ESP_LOGI(TAG, "Initializing AWS interface");
    
    /* Initialize WiFi first if not already done */
    if ((s_system_state.info.status_flags & BESS_STATUS_WIFI_CONNECTED) == 0) {
        esp_err_t ret = init_wifi();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
            return ret;
        }
        
        /* Wait for WiFi connection */
        EventBits_t bits = xEventGroupWaitBits(
            s_system_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(10000)
        );
        
        if ((bits & WIFI_CONNECTED_BIT) == 0) {
            ESP_LOGE(TAG, "WiFi connection timeout");
            return ESP_ERR_TIMEOUT;
        }
        
        s_system_state.info.status_flags |= BESS_STATUS_WIFI_CONNECTED;
    }
    
    /* Initialize AWS IoT interface */
    aws_interface_config_t aws_config = {
        .endpoint = s_system_state.config.aws_endpoint,
        .client_id = s_system_state.config.aws_client_id,
        .thing_name = s_system_state.info.system_id
    };
    
    esp_err_t ret = aws_interface_init(&aws_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AWS interface: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Register event callback */
    ret = aws_interface_register_event_callback(aws_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register AWS event callback: %s", esp_err_to_name(ret));
        aws_interface_deinit();
        return ret;
    }
    
    /* Connect to AWS IoT */
    ret = aws_interface_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to AWS IoT: %s", esp_err_to_name(ret));
        aws_interface_deinit();
        return ret;
    }
    
    /* Subscribe to command topic */
    char command_topic[64];
    snprintf(command_topic, sizeof(command_topic), "bess/%s/commands", s_system_state.info.system_id);
    ret = aws_interface_subscribe(command_topic, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to command topic: %s", esp_err_to_name(ret));
        aws_interface_disconnect();
        aws_interface_deinit();
        return ret;
    }
    
    /* Store AWS handler for future use */
    s_aws_handler = (void*)1;  /* Just a non-NULL value to indicate initialization */
    
    /* Update status flag */
    s_system_state.info.status_flags |= BESS_STATUS_AWS_CONNECTED;
    
    ESP_LOGI(TAG, "AWS interface initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize WiFi
 */
static esp_err_t init_wifi(void) {
    ESP_LOGI(TAG, "Initializing WiFi");
    
    /* Create event group for WiFi events if not already created */
    static EventGroupHandle_t wifi_event_group = NULL;
    if (wifi_event_group == NULL) {
        wifi_event_group = xEventGroupCreate();
        if (wifi_event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create WiFi event group");
            return ESP_ERR_NO_MEM;
        }
    }
    
    /* Initialize WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set WiFi mode to station */
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Configure WiFi connection */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .scan_method = WIFI_FAST_SCAN,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Start WiFi */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Connect to WiFi */
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WiFi initialized successfully");
    return ESP_OK;
}

/**
 * @brief AWS event handler callback
 */
static void aws_event_handler(aws_event_t event, void *event_data, void *user_data) {
    ESP_LOGD(TAG, "AWS event: %d", event.type);
    
    switch (event.type) {
        case AWS_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to AWS IoT");
            s_system_state.info.status_flags |= BESS_STATUS_AWS_CONNECTED;
            break;
            
        case AWS_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from AWS IoT");
            s_system_state.info.status_flags &= ~BESS_STATUS_AWS_CONNECTED;
            break;
            
        case AWS_EVENT_ERROR:
            ESP_LOGE(TAG, "AWS IoT error: %d", event.data);
            if (xSemaphoreTake(s_system_data_mutex, 0) == pdTRUE) {
                s_system_state.comm_stats.cloud_error_count++;
                xSemaphoreGive(s_system_data_mutex);
            }
            
            /* Check for persistent errors */
            if (s_system_state.comm_stats.cloud_error_count > 50) {
                system_manager_trigger_alarm(
                    BESS_ALARM_CLOUD_CONNECTION_ERROR,
                    0,  /* Not module-specific */
                    event.data
                );
            }
            break;
            
        case AWS_EVENT_DATA:
            /* Handle command messages */
            if (event_data != NULL) {
                aws_data_event_t *data_event = (aws_data_event_t*)event_data;
                process_cloud_command(data_event->topic, data_event->data, data_event->data_len);
            }
            
            if (xSemaphoreTake(s_system_data_mutex, 0) == pdTRUE) {
                s_system_state.comm_stats.cloud_rx_count++;
                xSemaphoreGive(s_system_data_mutex);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief Process commands received from the cloud
 */
static void process_cloud_command(const char *topic, const void *data, size_t data_len) {
    ESP_LOGI(TAG, "Processing cloud command: topic=%s, data_len=%d", topic, data_len);
    
    /* Simple command processing - in a real implementation, use a JSON parser */
    char command[256];
    if (data_len >= sizeof(command)) {
        /* Command too long */
        ESP_LOGW(TAG, "Command data too long, truncating");
        memcpy(command, data, sizeof(command) - 1);
        command[sizeof(command) - 1] = '\0';
    } else {
        memcpy(command, data, data_len);
        command[data_len] = '\0';
    }
    
    /* Parse and execute command */
    if (strncmp(command, "mode:", 5) == 0) {
        /* Mode change command */
        const char *mode_str = command + 5;
        bess_system_mode_t target_mode = BESS_MODE_STANDBY;
        
        if (strcmp(mode_str, "standby") == 0) {
            target_mode = BESS_MODE_STANDBY;
        } else if (strcmp(mode_str, "charging") == 0) {
            target_mode = BESS_MODE_CHARGING;
        } else if (strcmp(mode_str, "discharging") == 0) {
            target_mode = BESS_MODE_DISCHARGING;
        } else if (strcmp(mode_str, "maintenance") == 0) {
            target_mode = BESS_MODE_MAINTENANCE;
        } else if (strcmp(mode_str, "calibration") == 0) {
            target_mode = BESS_MODE_CALIBRATION;
        } else {
            ESP_LOGW(TAG, "Unknown mode command: %s", mode_str);
            return;
        }
        
        ESP_LOGI(TAG, "Cloud command: Set mode to %s", mode_str);
        system_manager_set_mode(target_mode);
    } else if (strncmp(command, "reboot", 6) == 0) {
        /* Reboot command */
        ESP_LOGI(TAG, "Cloud command: Reboot system");
        system_manager_reboot("Remote command");
    } else if (strncmp(command, "diagnostics", 11) == 0) {
        /* Run diagnostics */
        ESP_LOGI(TAG, "Cloud command: Run diagnostics");
        system_manager_run_diagnostics(true);
    } else if (strncmp(command, "sync_time", 9) == 0) {
        /* Synchronize time */
        ESP_LOGI(TAG, "Cloud command: Synchronize time");
        system_manager_sync_time();
    } else {
        ESP_LOGW(TAG, "Unknown cloud command: %s", command);
    }
}

/**
 * @brief WiFi event handler 
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                esp_wifi_connect();
                xEventGroupClearBits(s_system_event_group, WIFI_CONNECTED_BIT);
                s_system_state.info.status_flags &= ~BESS_STATUS_WIFI_CONNECTED;
                ESP_LOGW(TAG, "WiFi disconnected, attempting to reconnect");
                break;
                
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "WiFi connected with IP: " IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(s_system_event_group, WIFI_CONNECTED_BIT);
            s_system_state.info.status_flags |= BESS_STATUS_WIFI_CONNECTED;
        }
    }
}