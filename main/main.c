/**
 * @file main.c
 * @brief Main entry point for the Battery Energy Storage System (BESS) firmware
 * 
 * This file contains the main application entry point and initialization routines
 * for the 100kW/200kWh BESS using ESP32-P4 and FreeRTOS.
 * The main execution flow is:
 * 1. Initialize hardware and basic services
 * 2. Initialize all system components
 * 3. Perform system self-tests
 * 4. Start the system watchdog
 * 5. Start all system components and tasks
 * 6. Enter the main loop for system monitoring
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
 
 /* Include application headers */
 #include "bess_config.h"
 #include "bess_types.h"
 
 /* These would be from our component directories */
 /* We'll define prototypes here since we don't have the actual files yet */
 /* In practice these would be included from their respective header files */
 
 /* Battery Management */
 extern esp_err_t battery_manager_init(void);
 extern esp_err_t battery_manager_start(void);
 
 /* Power Management */
 extern esp_err_t power_manager_init(void);
 extern esp_err_t power_manager_start(void);
 
 /* Safety Systems */
 extern esp_err_t safety_manager_init(void);
 extern esp_err_t safety_manager_start(void);
 
 /* Communication */
 extern esp_err_t comm_manager_init(void);
 extern esp_err_t modbus_interface_init(void);
 extern esp_err_t canbus_interface_init(void);
 extern esp_err_t mqtt_client_init(void);
 extern esp_err_t comm_manager_start(void);
 
 /* System Control */
 extern esp_err_t system_manager_init(void);
 extern esp_err_t system_manager_start(void);
 
 /* Data Logging */
 extern esp_err_t logger_init(void);
 extern esp_err_t logger_start(void);
 
 /* Firmware Management */
 extern esp_err_t ota_manager_init(void);
 extern esp_err_t config_manager_init(void);
 extern esp_err_t config_manager_load(void);
 
 /* User Interface */
 extern esp_err_t ui_manager_init(void);
 extern esp_err_t ui_manager_start(void);
 
 /* Hardware Interface */
 extern esp_err_t hw_interface_init(void);
 
 /* Task Manager */
 extern esp_err_t task_manager_init(void);
 extern esp_err_t task_manager_register_all_tasks(void);
 extern esp_err_t task_manager_start_all_tasks(void);
 
 /* Define tag for logging */
 static const char* TAG = "BESS-Main";
 
 /* Global system status variable */
 static bess_system_status_t g_system_status = {
     .mode = BESS_MODE_STANDBY,
     .flags = {0},
     .error_code = BESS_ERROR_NONE,
 };
 
 /* Event group to signal system ready state */
 static EventGroupHandle_t g_system_event_group;
 #define SYSTEM_READY_BIT BIT0
 
 /**
  * @brief Initialize the SD card
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t init_sd_card(void)
 {
     esp_err_t ret;
     
     ESP_LOGI(TAG, "Initializing SD card");
     
     sdmmc_host_t host = SDMMC_HOST_DEFAULT();
     sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
     
     // Set up SPI bus for SD card
     slot_config.width = 1; // 1-line SD mode
     slot_config.clk = BESS_SD_CARD_CLK_PIN;
     slot_config.cmd = BESS_SD_CARD_MOSI_PIN;
     slot_config.d0 = BESS_SD_CARD_MISO_PIN;
     slot_config.d1 = GPIO_NUM_NC;
     slot_config.d2 = GPIO_NUM_NC;
     slot_config.d3 = BESS_SD_CARD_CS_PIN;
     
     // Mount options for the FAT filesystem
     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
         .format_if_mount_failed = true,
         .max_files = 5,
         .allocation_unit_size = 16 * 1024
     };
     
     sdmmc_card_t* card;
     ret = esp_vfs_fat_sdmmc_mount(BESS_SD_CARD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
     
     if (ret != ESP_OK) {
         if (ret == ESP_FAIL) {
             ESP_LOGE(TAG, "Failed to mount filesystem on SD card");
         } else {
             ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
         }
         return ret;
     }
     
     // Card has been initialized, print its properties
     sdmmc_card_print_info(stdout, card);
     
     // Create log directory if it doesn't exist
     struct stat st;
     if (stat(BESS_LOG_FILE_PATH, &st) != 0) {
         ESP_LOGI(TAG, "Creating log directory: %s", BESS_LOG_FILE_PATH);
         if (mkdir(BESS_LOG_FILE_PATH, 0755) != 0) {
             ESP_LOGE(TAG, "Failed to create log directory");
             return ESP_FAIL;
         }
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Initialize non-volatile storage (NVS)
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t init_nvs(void)
 {
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         // NVS partition was truncated or is a new version, erase and retry
         ESP_LOGI(TAG, "Erasing NVS partition");
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     
     return ret;
 }
 
 /**
  * @brief Initialize hardware components
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t init_hardware(void)
 {
     esp_err_t ret;
     
     ESP_LOGI(TAG, "Initializing hardware components");
     
     // Initialize GPIO pins
     gpio_config_t io_conf = {};
     
     // Configure status LED pins as output
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.pin_bit_mask = (1ULL << BESS_LED_POWER_PIN) | 
                            (1ULL << BESS_LED_STATUS_PIN) | 
                            (1ULL << BESS_LED_ERROR_PIN) |
                            (1ULL << BESS_CONTACTOR_CONTROL_PIN);
     io_conf.pull_down_en = 0;
     io_conf.pull_up_en = 0;
     ret = gpio_config(&io_conf);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure output GPIO pins: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Set initial state of LEDs
     gpio_set_level(BESS_LED_POWER_PIN, 1);  // Power LED on
     gpio_set_level(BESS_LED_STATUS_PIN, 0); // Status LED off
     gpio_set_level(BESS_LED_ERROR_PIN, 0);  // Error LED off
     gpio_set_level(BESS_CONTACTOR_CONTROL_PIN, 0); // Contactor off
     
     // Configure emergency stop pin as input
     io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupt on falling edge (emergency stop pressed)
     io_conf.mode = GPIO_MODE_INPUT;
     io_conf.pin_bit_mask = (1ULL << BESS_EMERGENCY_STOP_PIN);
     io_conf.pull_up_en = 1; // Enable pull-up resistor
     ret = gpio_config(&io_conf);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure input GPIO pins: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Initialize GPIO interrupt for emergency stop
     ret = gpio_install_isr_service(0);
     if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // ESP_ERR_INVALID_STATE means ISR service already installed
         ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Initialize hardware interface component
     ret = hw_interface_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize hardware interface: %s", esp_err_to_name(ret));
         return ret;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Initialize all system components
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t init_system_components(void)
 {
     esp_err_t ret;
     
     ESP_LOGI(TAG, "Initializing system components");
     
     // Initialize configuration manager first
     ret = config_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize configuration manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Load system configuration
     ret = config_manager_load();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to load system configuration: %s", esp_err_to_name(ret));
         // Continue with default configuration
         ESP_LOGW(TAG, "Using default configuration");
     }
     
     // Initialize the SD card
     ret = init_sd_card();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to initialize SD card, logging to SD will be disabled");
         // Continue without SD card, but disable SD card logging
         g_system_status.flags.fault_condition = true;
         g_system_status.error_code = BESS_ERROR_SD_CARD;
     }
     
     // Initialize logging system
     ret = logger_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize logging system: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Initialize remaining components
     
     // Task manager (FreeRTOS task management)
     ret = task_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize task manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Safety systems (highest priority)
     ret = safety_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize safety manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Battery management
     ret = battery_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize battery manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Power management
     ret = power_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize power manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // System control
     ret = system_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize system manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Communication interfaces
     ret = comm_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize communication manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Initialize specific communication protocols
     ret = modbus_interface_init();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to initialize Modbus interface: %s", esp_err_to_name(ret));
         // Continue without Modbus
     }
     
     ret = canbus_interface_init();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to initialize CANbus interface: %s", esp_err_to_name(ret));
         // Continue without CANbus
     }
     
     ret = mqtt_client_init();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to initialize MQTT client: %s", esp_err_to_name(ret));
         // Continue without MQTT
     }
     
     // OTA manager
     ret = ota_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to initialize OTA manager: %s", esp_err_to_name(ret));
         // Continue without OTA
     }
     
     // User interface
     ret = ui_manager_init();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to initialize UI manager: %s", esp_err_to_name(ret));
         // Continue without UI
     }
     
     // Register all tasks with the task manager
     ret = task_manager_register_all_tasks();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to register tasks: %s", esp_err_to_name(ret));
         return ret;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Start all system components
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t start_system_components(void)
 {
     esp_err_t ret;
     
     ESP_LOGI(TAG, "Starting system components");
     
     // Start logger first to capture logs from other components
     ret = logger_start();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start logger: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Start safety manager (highest priority)
     ret = safety_manager_start();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start safety manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Start battery manager
     ret = battery_manager_start();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start battery manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Start power manager
     ret = power_manager_start();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start power manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Start system manager
     ret = system_manager_start();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start system manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Start communication manager
     ret = comm_manager_start();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start communication manager: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Start UI manager
     ret = ui_manager_start();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to start UI manager: %s", esp_err_to_name(ret));
         // Continue without UI
     }
     
     // Start all registered tasks
     ret = task_manager_start_all_tasks();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start tasks: %s", esp_err_to_name(ret));
         return ret;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Self-test routine to verify system integrity
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t perform_self_test(void)
 {
     ESP_LOGI(TAG, "Performing system self-test");
     
     // TODO: Implement comprehensive self-test routines
     
     // Check emergency stop circuit
     bool emergency_stop_pressed = (gpio_get_level(BESS_EMERGENCY_STOP_PIN) == 0);
     if (emergency_stop_pressed) {
         ESP_LOGW(TAG, "Emergency stop is pressed during startup, system will remain in standby mode");
         g_system_status.flags.emergency_stop = true;
         g_system_status.mode = BESS_MODE_EMERGENCY_STOP;
         return ESP_ERR_INVALID_STATE;
     }
     
     // Basic hardware check
     ESP_LOGI(TAG, "Testing status LEDs");
     gpio_set_level(BESS_LED_STATUS_PIN, 1);
     vTaskDelay(500 / portTICK_PERIOD_MS);
     gpio_set_level(BESS_LED_ERROR_PIN, 1);
     vTaskDelay(500 / portTICK_PERIOD_MS);
     gpio_set_level(BESS_LED_STATUS_PIN, 0);
     gpio_set_level(BESS_LED_ERROR_PIN, 0);
     
     // Check memory
     ESP_LOGI(TAG, "Checking memory");
     size_t free_heap = esp_get_free_heap_size();
     ESP_LOGI(TAG, "Free heap: %u bytes", free_heap);
     if (free_heap < 50000) {
         ESP_LOGW(TAG, "Low memory condition detected");
     }
     
     // TODO: Add more comprehensive tests
     
     return ESP_OK;
 }
 
 /**
  * @brief System watchdog thread function
  * 
  * @param pvParameters FreeRTOS task parameters (unused)
  */
 static void system_watchdog_task(void *pvParameters)
 {
     TickType_t last_wake_time = xTaskGetTickCount();
     
     ESP_LOGI(TAG, "System watchdog task started");
     
     // Wait for system to be fully initialized
     xEventGroupWaitBits(g_system_event_group, SYSTEM_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
     
     while (1) {
         // TODO: Implement comprehensive watchdog checks
         
         // Monitor system components and tasks
         ESP_LOGD(TAG, "Watchdog check");
         
         // If system is in error mode, blink error LED
         if (g_system_status.error_code != BESS_ERROR_NONE) {
             gpio_set_level(BESS_LED_ERROR_PIN, !gpio_get_level(BESS_LED_ERROR_PIN));
         } else {
             gpio_set_level(BESS_LED_ERROR_PIN, 0); // Ensure error LED is off
         }
         
         // Blink status LED to indicate system is running
         gpio_set_level(BESS_LED_STATUS_PIN, !gpio_get_level(BESS_LED_STATUS_PIN));
         
         // Update system uptime
         g_system_status.uptime_seconds += BESS_SYSTEM_WATCHDOG_TIMEOUT_MS / 1000;
         
         // Sleep until next check
         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BESS_SYSTEM_WATCHDOG_TIMEOUT_MS));
     }
 }
 
 /**
  * @brief Create the system watchdog task
  * 
  * @return esp_err_t ESP_OK if successful, otherwise an error code
  */
 static esp_err_t create_system_watchdog(void)
 {
     BaseType_t result = xTaskCreatePinnedToCore(
         system_watchdog_task,        /* Task function */
         "system_watchdog",           /* Task name */
         4096,                        /* Stack size */
         NULL,                        /* Task parameters */
         tskIDLE_PRIORITY + 5,        /* Priority (high) */
         NULL,                        /* Task handle */
         tskNO_AFFINITY               /* Core ID (any core) */
     );
     
     if (result != pdPASS) {
         ESP_LOGE(TAG, "Failed to create system watchdog task");
         return ESP_FAIL;
     }
     
     return ESP_OK;
 }
 
 /**
  * @brief Application main entry point
  */
 void app_main(void)
 {
     esp_err_t ret;
     
     // Print banner
     printf("\n\n");
     printf("************************************************\n");
     printf("*     Battery Energy Storage System (BESS)     *\n");
     printf("*              Firmware v%s               *\n", BESS_FIRMWARE_VERSION);
     printf("*         %s         *\n", BESS_SYSTEM_ID);
     printf("************************************************\n");
     printf("\n");
     
     // Create system event group
     g_system_event_group = xEventGroupCreate();
     if (g_system_event_group == NULL) {
         ESP_LOGE(TAG, "Failed to create system event group");
         return;
     }
     
     // Initialize non-volatile storage
     ret = init_nvs();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
         return;
     }
     
     // Initialize TCP/IP stack
     ESP_LOGI(TAG, "Initializing TCP/IP adapter");
     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     
     // Initialize hardware components
     ret = init_hardware();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Hardware initialization failed: %s", esp_err_to_name(ret));
         // Flash error LED rapidly to indicate fatal error
         while (1) {
             gpio_set_level(BESS_LED_ERROR_PIN, 1);
             vTaskDelay(100 / portTICK_PERIOD_MS);
             gpio_set_level(BESS_LED_ERROR_PIN, 0);
             vTaskDelay(100 / portTICK_PERIOD_MS);
         }
     }
     
     // Initialize system components
     ret = init_system_components();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "System component initialization failed: %s", esp_err_to_name(ret));
         // Flash error LED rapidly to indicate fatal error
         while (1) {
             gpio_set_level(BESS_LED_ERROR_PIN, 1);
             vTaskDelay(100 / portTICK_PERIOD_MS);
             gpio_set_level(BESS_LED_ERROR_PIN, 0);
             vTaskDelay(100 / portTICK_PERIOD_MS);
         }
     }
     
     // Perform system self-test
     ret = perform_self_test();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "System self-test failed: %s", esp_err_to_name(ret));
         // Continue in degraded mode
     }
     
     // Create system watchdog task
     ret = create_system_watchdog();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to create system watchdog: %s", esp_err_to_name(ret));
         // Continue without watchdog
     }
     
     // Start all system components
     ret = start_system_components();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start system components: %s", esp_err_to_name(ret));
         // Flash error LED rapidly to indicate fatal error
         while (1) {
             gpio_set_level(BESS_LED_ERROR_PIN, 1);
             vTaskDelay(100 / portTICK_PERIOD_MS);
             gpio_set_level(BESS_LED_ERROR_PIN, 0);
             vTaskDelay(100 / portTICK_PERIOD_MS);
         }
     }
     
     // System is now ready
     ESP_LOGI(TAG, "System initialized successfully, entering main loop");
     xEventGroupSetBits(g_system_event_group, SYSTEM_READY_BIT);
     
     // Set status LED to solid on
     gpio_set_level(BESS_LED_STATUS_PIN, 1);
     
     // Main loop
     while (1) {
         // The main application logic is handled by FreeRTOS tasks
         // This is just a background task that can be used for system-level monitoring
         
         // Update status LEDs based on system state
         switch (g_system_status.mode) {
             case BESS_MODE_STANDBY:
                 // Status LED already handled by watchdog task
                 break;
                 
             case BESS_MODE_CHARGING:
                 // Fast blink of status LED
                 gpio_set_level(BESS_LED_STATUS_PIN, (esp_timer_get_time() / 250000) % 2);
                 break;
                 
             case BESS_MODE_DISCHARGING:
                 // Slow blink of status LED
                 gpio_set_level(BESS_LED_STATUS_PIN, (esp_timer_get_time() / 1000000) % 2);
                 break;
                 
             case BESS_MODE_ERROR:
             case BESS_MODE_EMERGENCY_STOP:
                 // Solid error LED
                 gpio_set_level(BESS_LED_ERROR_PIN, 1);
                 gpio_set_level(BESS_LED_STATUS_PIN, 0);
                 break;
                 
             default:
                 // Default LED pattern
                 gpio_set_level(BESS_LED_STATUS_PIN, (esp_timer_get_time() / 500000) % 2);
                 break;
         }
         
         // Sleep to avoid consuming CPU
         vTaskDelay(pdMS_TO_TICKS(500));
     }
 }