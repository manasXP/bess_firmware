/**
 * @file bess_config.h
 * @brief Configuration parameters for the Battery Energy Storage System
 * 
 * This file contains all configurable parameters for the BESS firmware,
 * including system specifications, communication settings, and operation limits.
 */

 #ifndef BESS_CONFIG_H
 #define BESS_CONFIG_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "esp_system.h"
 
 /**
  * @brief System identification and version information
  */
 #define BESS_FIRMWARE_VERSION        "1.0.0"
 #define BESS_SYSTEM_ID               "BESS-100kW-01"
 #define BESS_MANUFACTURER            "Your Company Name"
 
 /**
  * @brief System specifications
  */
 #define BESS_NOMINAL_POWER_KW        100     /* System nominal power in kW */
 #define BESS_NOMINAL_CAPACITY_KWH    200     /* System nominal capacity in kWh */
 
 /**
  * @brief Battery module specifications
  */
 #define BESS_MODULE_NOMINAL_VOLTAGE  48.0f   /* Module nominal voltage in V */
 #define BESS_MODULE_CAPACITY_KWH     16.0f   /* Module capacity in kWh */
 #define BESS_MODULE_COUNT            13      /* Number of battery modules */
 #define BESS_CELLS_PER_MODULE        16      /* Number of cells per module */
 #define BESS_MODULE_MAX_TEMP_C       60      /* Maximum module temperature in °C */
 #define BESS_MODULE_MIN_TEMP_C      -10      /* Minimum module temperature in °C */
 
 /**
  * @brief Cell specifications
  */
 #define BESS_CELL_NOMINAL_VOLTAGE    3.2f    /* Cell nominal voltage in V */
 #define BESS_CELL_MAX_VOLTAGE        3.65f   /* Cell maximum voltage in V */
 #define BESS_CELL_MIN_VOLTAGE        2.5f    /* Cell minimum voltage in V */
 #define BESS_CELL_MAX_CURRENT        100.0f  /* Cell maximum current in A */
 
 /**
  * @brief Battery management system parameters
  */
 #define BESS_BMS_VOLTAGE_DELTA_MAX   0.05f   /* Max allowed voltage difference between cells in V */
 #define BESS_BMS_TEMP_DELTA_MAX      5.0f    /* Max allowed temperature difference between cells in °C */
 #define BESS_BMS_SOC_MIN             10.0f   /* Minimum State of Charge (%) */
 #define BESS_BMS_SOC_MAX             90.0f   /* Maximum State of Charge (%) */
 #define BESS_BMS_CELL_BALANCING_THRESHOLD 0.03f /* Voltage threshold for cell balancing in V */
 
 /**
  * @brief Safety limits
  */
 #define BESS_SAFETY_MAX_CURRENT      150.0f  /* Maximum system current in A */
 #define BESS_SAFETY_MAX_TEMP_C       55      /* Maximum operating temperature in °C */
 #define BESS_SAFETY_WARN_TEMP_C      45      /* Warning temperature threshold in °C */
 #define BESS_SAFETY_MIN_TEMP_C       0       /* Minimum operating temperature in °C */
 #define BESS_SAFETY_WARN_MIN_TEMP_C  5       /* Warning minimum temperature threshold in °C */
 #define BESS_SAFETY_MAX_VOLTAGE      800.0f  /* Maximum system voltage in V */
 #define BESS_SAFETY_MIN_VOLTAGE      500.0f  /* Minimum system voltage in V */
 
 /**
  * @brief Communication settings
  */
 /* Modbus settings */
 #define BESS_MODBUS_SLAVE_ID         1       /* Modbus slave ID */
 #define BESS_MODBUS_TCP_PORT         502     /* Modbus TCP port */
 #define BESS_MODBUS_UART_NUM         UART_NUM_1  /* UART port for Modbus RTU */
 #define BESS_MODBUS_UART_BAUD_RATE   9600    /* Baud rate for Modbus RTU */
 #define BESS_MODBUS_UART_TX_PIN      17      /* TX pin for Modbus RTU */
 #define BESS_MODBUS_UART_RX_PIN      16      /* RX pin for Modbus RTU */
 
 /* CAN Bus settings */
 #define BESS_CAN_TX_PIN              21      /* TX pin for CAN Bus */
 #define BESS_CAN_RX_PIN              22      /* RX pin for CAN Bus */
 #define BESS_CAN_BITRATE             500000  /* CAN Bus bitrate in bps */
 
 /* WiFi settings */
 #define BESS_WIFI_SSID               "BESS_SYSTEM"
 #define BESS_WIFI_PASSWORD           "BessSecurePass123"
 #define BESS_WIFI_MAX_RETRY          5
 
 /* AWS IoT settings */
 #define BESS_AWS_IOT_ENDPOINT        "your-iot-endpoint.amazonaws.com"
 #define BESS_AWS_IOT_PORT            8883
 #define BESS_AWS_REGION              "us-east-1"
 #define BESS_CLOUDWATCH_LOG_GROUP    "/bess/logs"
 
 /**
  * @brief Logging settings
  */
 #define BESS_LOG_LEVEL               ESP_LOG_INFO
 #define BESS_SD_CARD_MOUNT_POINT     "/sdcard"
 #define BESS_LOG_FILE_PATH           "/sdcard/logs"
 #define BESS_LOG_FILE_MAX_SIZE       (1024 * 1024)  /* 1MB */
 #define BESS_LOG_FILE_ROTATION_COUNT 10
 #define BESS_LOG_TO_CONSOLE          true
 #define BESS_LOG_TO_SD_CARD          true
 #define BESS_LOG_TO_AWS              true
 #define BESS_LOG_UPLOAD_INTERVAL_MS  (5 * 60 * 1000)  /* 5 minutes */
 
 /**
  * @brief Task configuration
  */
 #define BESS_SAFETY_TASK_PRIORITY        (tskIDLE_PRIORITY + 5)
 #define BESS_SAFETY_TASK_STACK_SIZE      4096
 #define BESS_SAFETY_TASK_CORE            0
 #define BESS_SAFETY_TASK_INTERVAL_MS     100
 
 #define BESS_BMS_TASK_PRIORITY           (tskIDLE_PRIORITY + 4)
 #define BESS_BMS_TASK_STACK_SIZE         4096
 #define BESS_BMS_TASK_CORE               0
 #define BESS_BMS_TASK_INTERVAL_MS        500
 
 #define BESS_POWER_TASK_PRIORITY         (tskIDLE_PRIORITY + 4)
 #define BESS_POWER_TASK_STACK_SIZE       4096
 #define BESS_POWER_TASK_CORE             0
 #define BESS_POWER_TASK_INTERVAL_MS      500
 
 #define BESS_COMM_TASK_PRIORITY          (tskIDLE_PRIORITY + 3)
 #define BESS_COMM_TASK_STACK_SIZE        8192
 #define BESS_COMM_TASK_CORE              1
 #define BESS_COMM_TASK_INTERVAL_MS       1000
 
 #define BESS_LOGGER_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)
 #define BESS_LOGGER_TASK_STACK_SIZE      8192
 #define BESS_LOGGER_TASK_CORE            1
 #define BESS_LOGGER_TASK_INTERVAL_MS     1000
 
 #define BESS_UI_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)
 #define BESS_UI_TASK_STACK_SIZE          8192
 #define BESS_UI_TASK_CORE                1
 #define BESS_UI_TASK_INTERVAL_MS         50
 
 /**
  * @brief Pin definitions
  */
 #define BESS_LED_POWER_PIN           12
 #define BESS_LED_STATUS_PIN          13
 #define BESS_LED_ERROR_PIN           14
 #define BESS_EMERGENCY_STOP_PIN      15
 #define BESS_CONTACTOR_CONTROL_PIN   18
 #define BESS_SD_CARD_CS_PIN          5
 #define BESS_SD_CARD_MOSI_PIN        23
 #define BESS_SD_CARD_MISO_PIN        19
 #define BESS_SD_CARD_CLK_PIN         18
 
 /**
  * @brief System operational parameters
  */
 #define BESS_CONTROL_LOOP_INTERVAL_MS    100  /* Control loop interval in ms */
 #define BESS_DATA_LOGGING_INTERVAL_MS    5000 /* Data logging interval in ms */
 #define BESS_SYSTEM_WATCHDOG_TIMEOUT_MS  5000 /* Watchdog timeout in ms */
 #define BESS_GRID_NOMINAL_VOLTAGE        230.0f /* Grid nominal voltage in V */
 #define BESS_GRID_NOMINAL_FREQUENCY      50.0f  /* Grid nominal frequency in Hz */
 #define BESS_GRID_FREQUENCY_TOLERANCE    0.5f   /* Grid frequency tolerance in Hz */
 
 /**
  * @brief Web server configuration
  */
 #define BESS_WEB_SERVER_PORT         80
 #define BESS_WEB_SERVER_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)
 #define BESS_WEB_SERVER_TASK_STACK_SIZE  8192
 #define BESS_WEB_SERVER_TASK_CORE        1
 
 /**
  * @brief OTA Update configuration
  */
 #define BESS_OTA_SERVER_URL          "https://your-ota-server.com/firmware"
 #define BESS_OTA_CHECK_INTERVAL_MS   (24 * 60 * 60 * 1000)  /* 24 hours */
 #define BESS_OTA_TASK_PRIORITY       (tskIDLE_PRIORITY + 1)
 #define BESS_OTA_TASK_STACK_SIZE     8192
 #define BESS_OTA_TASK_CORE           1
 
 #endif /* BESS_CONFIG_H */