/**
 * @file modbus_interface.c
 * @brief Modbus communication interface implementation for BESS controller
 * 
 * This file implements the Modbus RTU and Modbus TCP interfaces for the
 * 100KW/200KWH Battery Energy Storage System (BESS) controller.
 * It provides functionality for both Modbus master and slave modes to communicate
 * with other system components including inverters, BMS modules, and SCADA systems.
 * 
 * @copyright Copyright (c) 2025
 */

 #include "modbus_interface.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 #include "driver/uart.h"
 #include "driver/gpio.h"
 #include "lwip/err.h"
 #include "lwip/sockets.h"
 #include "lwip/sys.h"
 #include "lwip/netdb.h"
 #include "lwip/dns.h"
 #include "logger.h"
 
 /* Modbus protocol constants */
 #define MODBUS_RTU_MAX_ADU_LENGTH     256
 #define MODBUS_TCP_MAX_ADU_LENGTH     260
 #define MODBUS_TCP_HEADER_LENGTH      7
 #define MODBUS_CRC_LENGTH             2
 #define MODBUS_BROADCAST_ADDRESS      0
 
 /* Default Modbus RTU settings */
 #define MODBUS_DEFAULT_BAUDRATE       9600
 #define MODBUS_DEFAULT_PARITY         UART_PARITY_EVEN
 #define MODBUS_DEFAULT_STOP_BITS      UART_STOP_BITS_1
 #define MODBUS_DEFAULT_DATA_BITS      UART_DATA_8_BITS
 
 /* Function codes */
 #define MODBUS_FC_READ_COILS          0x01
 #define MODBUS_FC_READ_DISCRETE_INPUTS 0x02
 #define MODBUS_FC_READ_HOLDING_REGS   0x03
 #define MODBUS_FC_READ_INPUT_REGS     0x04
 #define MODBUS_FC_WRITE_SINGLE_COIL   0x05
 #define MODBUS_FC_WRITE_SINGLE_REG    0x06
 #define MODBUS_FC_WRITE_MULTIPLE_COILS 0x0F
 #define MODBUS_FC_WRITE_MULTIPLE_REGS 0x10
 
 /* Exception codes */
 #define MODBUS_EXCEPTION_ILLEGAL_FUNCTION     0x01
 #define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
 #define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE   0x03
 #define MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE 0x04
 
 /* Module TAG for logging */
 static const char *TAG = "MODBUS";
 
 /* Module state variables */
 typedef struct {
     modbus_mode_t mode;
     modbus_transport_t transport;
     uint8_t slave_address;
     
     /* UART settings for RTU mode */
     uart_port_t uart_port;
     gpio_num_t rts_pin;
     gpio_num_t de_pin;        /* Driver Enable pin for RS-485 */
     
     /* TCP settings */
     int server_socket;
     uint16_t tcp_port;
     bool tcp_server_running;
     
     /* Protocol data */
     uint8_t rx_buffer[MODBUS_TCP_MAX_ADU_LENGTH];
     uint8_t tx_buffer[MODBUS_TCP_MAX_ADU_LENGTH];
     
     /* Thread safety */
     SemaphoreHandle_t modbus_mutex;
     
     /* Callback functions */
     modbus_register_callback_t register_callbacks[4]; /* Array for 4 register types */
     void* callback_args[4];
     
     /* Task handle for TCP server */
     TaskHandle_t tcp_task_handle;
 } modbus_context_t;
 
 /* Static instance of Modbus context */
 static modbus_context_t s_modbus_ctx = {0};
 
 /* Memory map for Modbus registers */
 static uint16_t s_holding_registers[MODBUS_MAX_REGISTERS];
 static uint16_t s_input_registers[MODBUS_MAX_REGISTERS];
 static uint8_t s_coils[MODBUS_MAX_REGISTERS / 8];
 static uint8_t s_discrete_inputs[MODBUS_MAX_REGISTERS / 8];
 
 /* Forward declarations for internal functions */
 static esp_err_t modbus_rtu_init(const modbus_rtu_config_t *config);
 static esp_err_t modbus_tcp_init(const modbus_tcp_config_t *config);
 static void modbus_tcp_server_task(void *pvParameters);
 /**
  * @brief Process a Modbus request and generate a response
  * 
  * @param request Pointer to the Modbus PDU (without transport headers)
  * @param request_len Length of the request PDU
  * @param response Buffer to store the response PDU
  * @param response_len Pointer to store the length of the response PDU
  * 
  * @return ESP_OK if successful, or an error code
  */
 static esp_err_t modbus_process_request(uint8_t *request, uint16_t request_len, 
                                       uint8_t *response, uint16_t *response_len)
 {
     if (request == NULL || response == NULL || response_len == NULL || request_len < 2) {
         return ESP_ERR_INVALID_ARG;
     }
     
     uint8_t function_code = request[0];
     esp_err_t ret = ESP_OK;
     
     /* Check if it's an exception response */
     if (function_code & 0x80) {
         BESS_LOGE(TAG, "Received exception response for function 0x%02X, code: 0x%02X", 
                  function_code & 0x7F, request[1]);
         return ESP_ERR_INVALID_RESPONSE;
     }
     
     /* Initialize response with function code */
     response[0] = function_code;
     
     /* Process by function code */
     switch (function_code) {
         case MODBUS_FC_READ_COILS:
         case MODBUS_FC_READ_DISCRETE_INPUTS:
         case MODBUS_FC_READ_HOLDING_REGS:
         case MODBUS_FC_READ_INPUT_REGS: {
             if (request_len < 5) {
                 /* Request too short */
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             }
             
             uint16_t address = (request[1] << 8) | request[2];
             uint16_t count = (request[3] << 8) | request[4];
             
             /* Validate count based on function code */
             if ((function_code == MODBUS_FC_READ_COILS || 
                  function_code == MODBUS_FC_READ_DISCRETE_INPUTS) && 
                 (count < 1 || count > 2000)) {
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             } else if ((function_code == MODBUS_FC_READ_HOLDING_REGS || 
                         function_code == MODBUS_FC_READ_INPUT_REGS) && 
                        (count < 1 || count > 125)) {
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             }
             
             /* Execute read operation */
             ret = modbus_handle_read_registers(function_code, address, count, response, response_len);
             break;
         }
         
         case MODBUS_FC_WRITE_SINGLE_COIL:
         case MODBUS_FC_WRITE_SINGLE_REG: {
             if (request_len < 5) {
                 /* Request too short */
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             }
             
             uint16_t address = (request[1] << 8) | request[2];
             uint16_t value = (request[3] << 8) | request[4];
             
             /* Validate value for coil */
             if (function_code == MODBUS_FC_WRITE_SINGLE_COIL && value != 0 && value != 0xFF00) {
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             }
             
             /* Execute write operation */
             ret = modbus_handle_write_registers(function_code, address, 1, &request[3], response, response_len);
             break;
         }
         
         case MODBUS_FC_WRITE_MULTIPLE_COILS:
         case MODBUS_FC_WRITE_MULTIPLE_REGS: {
             if (request_len < 6) {
                 /* Request too short */
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             }
             
             uint16_t address = (request[1] << 8) | request[2];
             uint16_t count = (request[3] << 8) | request[4];
             uint8_t byte_count = request[5];
             
             /* Validate count and byte count */
             if (function_code == MODBUS_FC_WRITE_MULTIPLE_COILS) {
                 if (count < 1 || count > 1968 || byte_count != ((count + 7) / 8)) {
                     ret = ESP_ERR_INVALID_ARG;
                     break;
                 }
             } else { /* MODBUS_FC_WRITE_MULTIPLE_REGS */
                 if (count < 1 || count > 123 || byte_count != (count * 2)) {
                     ret = ESP_ERR_INVALID_ARG;
                     break;
                 }
             }
             
             /* Check if request has enough data */
             if (request_len < (6 + byte_count)) {
                 ret = ESP_ERR_INVALID_ARG;
                 break;
             }
             
             /* Execute write operation */
             ret = modbus_handle_write_registers(function_code, address, count, &request[6], response, response_len);
             break;
         }
         
         default:
             /* Unsupported function code - return exception */
             response[0] = function_code | 0x80;
             response[1] = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
             *response_len = 2;
             ret = ESP_OK;
             break;
     }
     
     /* Handle errors by sending exception response */
     if (ret != ESP_OK) {
         uint8_t exception_code;
         
         switch (ret) {
             case ESP_ERR_INVALID_ARG:
                 exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
                 break;
                 
             case ESP_ERR_INVALID_STATE:
                 exception_code = MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE;
                 break;
                 
             case ESP_ERR_INVALID_SIZE:
                 exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
                 break;
                 
             default:
                 exception_code = MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE;
                 break;
         }
         
         response[0] = function_code | 0x80;
         response[1] = exception_code;
         *response_len = 2;
         return ESP_OK; /* Return OK to send the exception response */
     }
     
     return ret;
 }
 /**
  * @brief Validate a Modbus request
  * 
  * @param request Request buffer
  * @param request_len Request length
  * 
  * @return true if valid, false otherwise
  */
 static bool modbus_validate_request(uint8_t *request, uint16_t request_len)
 {
     if (request == NULL || request_len < 4) {
         return false;
     }
     
     /* Check CRC for RTU mode */
     if (s_modbus_ctx.transport == MODBUS_TRANSPORT_RTU) {
         uint16_t crc = modbus_calc_crc(request, request_len - 2);
         uint16_t received_crc = (request[request_len - 1] << 8) | request[request_len - 2];
         
         if (crc != received_crc) {
             BESS_LOGW(TAG, "CRC error: calculated=0x%04X, received=0x%04X", crc, received_crc);
             return false;
         }
     }
     
     /* Validate function code */
     uint8_t function_code = request[1];
     switch (function_code) {
         case MODBUS_FC_READ_COILS:
         case MODBUS_FC_READ_DISCRETE_INPUTS:
         case MODBUS_FC_READ_HOLDING_REGS:
         case MODBUS_FC_READ_INPUT_REGS:
         case MODBUS_FC_WRITE_SINGLE_COIL:
         case MODBUS_FC_WRITE_SINGLE_REG:
         case MODBUS_FC_WRITE_MULTIPLE_COILS:
         case MODBUS_FC_WRITE_MULTIPLE_REGS:
             /* Supported function code */
             break;
         
         default:
             /* Unsupported function code */
             BESS_LOGW(TAG, "Unsupported function code: 0x%02X", function_code);
             return false;
     }
     
     return true;
 }
 esp_err_t modbus_master_read_coils(uint8_t slave_addr, uint16_t start_address, 
                               uint16_t quantity, uint8_t *coil_status)
 {
     if (coil_status == NULL || quantity == 0 || quantity > 2000) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Allocate buffer for holding register values as 16-bit values */
     uint16_t values_size = (quantity + 15) / 16;  /* Round up to nearest 16-bit word */
     uint16_t *values = (uint16_t *)malloc(values_size * sizeof(uint16_t));
     if (values == NULL) {
         return ESP_ERR_NO_MEM;
     }
     
     /* Read coils as holding registers */
     esp_err_t ret = modbus_master_read_registers(slave_addr, MODBUS_FC_READ_COILS, 
                                                start_address, quantity, values);
     
     /* Convert the bit-packed format to array of bytes */
     if (ret == ESP_OK) {
         for (uint16_t i = 0; i < quantity; i++) {
             uint16_t word_idx = i / 16;
             uint16_t bit_idx = i % 16;
             coil_status[i] = (values[word_idx] >> bit_idx) & 0x01;
         }
     }
     
     free(values);
     return ret;
 }
 
 esp_err_t modbus_master_read_discrete_inputs(uint8_t slave_addr, uint16_t start_address, 
                                            uint16_t quantity, uint8_t *input_status)
 {
     if (input_status == NULL || quantity == 0 || quantity > 2000) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Allocate buffer for holding register values as 16-bit values */
     uint16_t values_size = (quantity + 15) / 16;  /* Round up to nearest 16-bit word */
     uint16_t *values = (uint16_t *)malloc(values_size * sizeof(uint16_t));
     if (values == NULL) {
         return ESP_ERR_NO_MEM;
     }
     
     /* Read discrete inputs as holding registers */
     esp_err_t ret = modbus_master_read_registers(slave_addr, MODBUS_FC_READ_DISCRETE_INPUTS, 
                                                start_address, quantity, values);
     
     /* Convert the bit-packed format to array of bytes */
     if (ret == ESP_OK) {
         for (uint16_t i = 0; i < quantity; i++) {
             uint16_t word_idx = i / 16;
             uint16_t bit_idx = i % 16;
             input_status[i] = (values[word_idx] >> bit_idx) & 0x01;
         }
     }
     
     free(values);
     return ret;
 }
 
 esp_err_t modbus_master_read_holding_registers(uint8_t slave_addr, uint16_t start_address, 
                                              uint16_t quantity, uint16_t *holding_registers)
 {
     if (holding_registers == NULL || quantity == 0 || quantity > 125) {
         return ESP_ERR_INVALID_ARG;
     }
     
     return modbus_master_read_registers(slave_addr, MODBUS_FC_READ_HOLDING_REGS, 
                                       start_address, quantity, holding_registers);
 }
 
 esp_err_t modbus_master_read_input_registers(uint8_t slave_addr, uint16_t start_address, 
                                            uint16_t quantity, uint16_t *input_registers)
 {
     if (input_registers == NULL || quantity == 0 || quantity > 125) {
         return ESP_ERR_INVALID_ARG;
     }
     
     return modbus_master_read_registers(slave_addr, MODBUS_FC_READ_INPUT_REGS, 
                                       start_address, quantity, input_registers);
 }
 
 esp_err_t modbus_master_write_single_coil(uint8_t slave_addr, uint16_t coil_address, 
                                         uint8_t coil_value)
 {
     /* Coil value is 0xFF00 for ON and 0x0000 for OFF */
     uint16_t value = coil_value ? 0xFF00 : 0x0000;
     
     return modbus_master_write_registers(slave_addr, MODBUS_FC_WRITE_SINGLE_COIL, 
                                        coil_address, 1, &value);
 }
 
 esp_err_t modbus_master_write_single_register(uint8_t slave_addr, uint16_t register_address, 
                                             uint16_t register_value)
 {
     return modbus_master_write_registers(slave_addr, MODBUS_FC_WRITE_SINGLE_REG, 
                                        register_address, 1, &register_value);
 }
 
 esp_err_t modbus_master_write_multiple_coils(uint8_t slave_addr, uint16_t start_address, 
                                            uint16_t quantity, const uint8_t *coil_values)
 {
     if (coil_values == NULL || quantity == 0 || quantity > 1968) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Convert the coil values to the packed format required by Modbus */
     uint16_t num_registers = (quantity + 15) / 16;  /* Round up to nearest 16-bit word */
     uint16_t *packed_values = (uint16_t *)malloc(num_registers * sizeof(uint16_t));
     if (packed_values == NULL) {
         return ESP_ERR_NO_MEM;
     }
     
     memset(packed_values, 0, num_registers * sizeof(uint16_t));
     
     for (uint16_t i = 0; i < quantity; i++) {
         if (coil_values[i]) {
             uint16_t word_idx = i / 16;
             uint16_t bit_idx = i % 16;
             packed_values[word_idx] |= (1 << bit_idx);
         }
     }
     
     esp_err_t ret = modbus_master_write_registers(slave_addr, MODBUS_FC_WRITE_MULTIPLE_COILS, 
                                                 start_address, quantity, packed_values);
     
     free(packed_values);
     return ret;
 }
 
 esp_err_t modbus_master_write_multiple_registers(uint8_t slave_addr, uint16_t start_address, 
                                                uint16_t quantity, const uint16_t *register_values)
 {
     if (register_values == NULL || quantity == 0 || quantity > 123) {
         return ESP_ERR_INVALID_ARG;
     }
     
     return modbus_master_write_registers(slave_addr, MODBUS_FC_WRITE_MULTIPLE_REGS, 
                                        start_address, quantity, register_values);
 }
 
 esp_err_t modbus_slave_set_register_map(modbus_register_type_t reg_type, 
                                       uint16_t start_address, uint16_t count, 
                                       const uint16_t *values)
 {
     if (values == NULL || count == 0 || 
         start_address + count > MODBUS_MAX_REGISTERS) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     switch (reg_type) {
         case MODBUS_REG_HOLDING: {
             /* Copy values to holding registers */
             for (uint16_t i = 0; i < count; i++) {
                 s_holding_registers[start_address + i] = values[i];
             }
             break;
         }
         
         case MODBUS_REG_INPUT: {
             /* Copy values to input registers */
             for (uint16_t i = 0; i < count; i++) {
                 s_input_registers[start_address + i] = values[i];
             }
             break;
         }
         
         case MODBUS_REG_COIL: {
             /* Copy values to coils (bit by bit) */
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_addr = start_address + i;
                 if (values[i]) {
                     s_coils[reg_addr / 8] |= (1 << (reg_addr % 8));
                 } else {
                     s_coils[reg_addr / 8] &= ~(1 << (reg_addr % 8));
                 }
             }
             break;
         }
         
         case MODBUS_REG_DISCRETE_INPUTS: {
             /* Copy values to discrete inputs (bit by bit) */
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_addr = start_address + i;
                 if (values[i]) {
                     s_discrete_inputs[reg_addr / 8] |= (1 << (reg_addr % 8));
                 } else {
                     s_discrete_inputs[reg_addr / 8] &= ~(1 << (reg_addr % 8));
                 }
             }
             break;
         }
         
         default:
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ESP_OK;
 }
 
 esp_err_t modbus_slave_get_register_map(modbus_register_type_t reg_type, 
                                       uint16_t start_address, uint16_t count, 
                                       uint16_t *values)
 {
     if (values == NULL || count == 0 || 
         start_address + count > MODBUS_MAX_REGISTERS) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     switch (reg_type) {
         case MODBUS_REG_HOLDING: {
             /* Copy values from holding registers */
             for (uint16_t i = 0; i < count; i++) {
                 values[i] = s_holding_registers[start_address + i];
             }
             break;
         }
         
         case MODBUS_REG_INPUT: {
             /* Copy values from input registers */
             for (uint16_t i = 0; i < count; i++) {
                 values[i] = s_input_registers[start_address + i];
             }
             break;
         }
         
         case MODBUS_REG_COIL: {
             /* Copy values from coils (bit by bit) */
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_addr = start_address + i;
                 values[i] = (s_coils[reg_addr / 8] >> (reg_addr % 8)) & 0x01;
             }
             break;
         }
         
         case MODBUS_REG_DISCRETE_INPUTS: {
             /* Copy values from discrete inputs (bit by bit) */
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_addr = start_address + i;
                 values[i] = (s_discrete_inputs[reg_addr / 8] >> (reg_addr % 8)) & 0x01;
             }
             break;
         }
         
         default:
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ESP_OK;
 }
 /**
  * @brief Handle read register operations (coils, discrete inputs, holding, input)
  * 
  * @param function_code Modbus function code
  * @param address Starting register address
  * @param count Number of registers to read
  * @param response Buffer to store response
  * @param response_len Pointer to store response length
  * 
  * @return ESP_OK if successful, or an error code
  */
 static esp_err_t modbus_handle_read_registers(uint8_t function_code, uint16_t address, 
                                            uint16_t count, uint8_t *response, 
                                            uint16_t *response_len)
 {
     if (response == NULL || response_len == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Calculate response size and set initial byte count */
     uint8_t byte_count;
     
     if (function_code == MODBUS_FC_READ_COILS || function_code == MODBUS_FC_READ_DISCRETE_INPUTS) {
         byte_count = (count + 7) / 8; /* Round up to nearest byte */
     } else {
         byte_count = count * 2; /* 2 bytes per register */
     }
     
     /* Set function code and byte count in response */
     response[0] = function_code;
     response[1] = byte_count;
     
     /* Initialize response data area */
     memset(&response[2], 0, byte_count);
     
     /* Get register type based on function code */
     modbus_register_type_t reg_type;
     switch (function_code) {
         case MODBUS_FC_READ_COILS:
             reg_type = MODBUS_REG_COIL;
             break;
             
         case MODBUS_FC_READ_DISCRETE_INPUTS:
             reg_type = MODBUS_REG_DISCRETE_INPUTS;
             break;
             
         case MODBUS_FC_READ_HOLDING_REGS:
             reg_type = MODBUS_REG_HOLDING;
             break;
             
         case MODBUS_FC_READ_INPUT_REGS:
             reg_type = MODBUS_REG_INPUT;
             break;
             
         default:
             return ESP_ERR_INVALID_ARG;
     }
     
     /* Invoke callback if registered */
     if (s_modbus_ctx.register_callbacks[reg_type] != NULL) {
         esp_err_t ret = modbus_execute_callback(reg_type, function_code, address, count, &response[2]);
         if (ret != ESP_OK) {
             return ret;
         }
     } else {
         /* No callback, use internal register memory */
         if (address + count > MODBUS_MAX_REGISTERS) {
             return ESP_ERR_INVALID_SIZE;
         }
         
         if (function_code == MODBUS_FC_READ_COILS || function_code == MODBUS_FC_READ_DISCRETE_INPUTS) {
             /* Read bit values (coils or discrete inputs) */
             uint8_t *reg_array = (function_code == MODBUS_FC_READ_COILS) ? 
                                 s_coils : s_discrete_inputs;
             
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_addr = address + i;
                 uint8_t bit_value = (reg_array[reg_addr / 8] >> (reg_addr % 8)) & 0x01;
                 
                 if (bit_value) {
                     response[2 + (i / 8)] |= (1 << (i % 8));
                 }
             }
         } else {
             /* Read 16-bit register values (holding or input registers) */
             uint16_t *reg_array = (function_code == MODBUS_FC_READ_HOLDING_REGS) ? 
                                  s_holding_registers : s_input_registers;
             
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_value = reg_array[address + i];
                 response[2 + (i * 2)] = (reg_value >> 8) & 0xFF;
                 response[2 + (i * 2) + 1] = reg_value & 0xFF;
             }
         }
     }
     
     /* Set response length */
     *response_len = 2 + byte_count;
     
     return ESP_OK;
 }
 /**
  * @brief Handle write register operations (coils or registers)
  * 
  * @param function_code Modbus function code
  * @param address Starting register address
  * @param count Number of registers to write
  * @param data Data to write
  * @param response Buffer to store response
  * @param response_len Pointer to store response length
  * 
  * @return ESP_OK if successful, or an error code
  */
 static esp_err_t modbus_handle_write_registers(uint8_t function_code, uint16_t address, 
                                             uint16_t count, uint8_t *data, 
                                             uint8_t *response, uint16_t *response_len)
 {
     if (data == NULL || response == NULL || response_len == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Check address range */
     if (address + count > MODBUS_MAX_REGISTERS) {
         return ESP_ERR_INVALID_SIZE;
     }
     
     /* Get register type based on function code */
     modbus_register_type_t reg_type;
     switch (function_code) {
         case MODBUS_FC_WRITE_SINGLE_COIL:
         case MODBUS_FC_WRITE_MULTIPLE_COILS:
             reg_type = MODBUS_REG_COIL;
             break;
             
         case MODBUS_FC_WRITE_SINGLE_REG:
         case MODBUS_FC_WRITE_MULTIPLE_REGS:
             reg_type = MODBUS_REG_HOLDING;
             break;
             
         default:
             return ESP_ERR_INVALID_ARG;
     }
     
     /* Invoke callback if registered */
     if (s_modbus_ctx.register_callbacks[reg_type] != NULL) {
         esp_err_t ret = modbus_execute_callback(reg_type, function_code, address, count, data);
         if (ret != ESP_OK) {
             return ret;
         }
     } else {
         /* No callback, use internal register memory */
         if (function_code == MODBUS_FC_WRITE_SINGLE_COIL) {
             /* Single coil: data in format 0xFF00 (ON) or 0x0000 (OFF) */
             uint16_t value = (data[0] << 8) | data[1];
             uint8_t bit_value = (value == 0xFF00) ? 1 : 0;
             
             /* Set or clear the bit */
             if (bit_value) {
                 s_coils[address / 8] |= (1 << (address % 8));
             } else {
                 s_coils[address / 8] &= ~(1 << (address % 8));
             }
         } 
         else if (function_code == MODBUS_FC_WRITE_MULTIPLE_COILS) {
             /* Multiple coils */
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t reg_addr = address + i;
                 uint8_t byte_index = i / 8;
                 uint8_t bit_index = i % 8;
                 uint8_t bit_value = (data[byte_index] >> bit_index) & 0x01;
                 
                 /* Set or clear the bit */
                 if (bit_value) {
                     s_coils[reg_addr / 8] |= (1 << (reg_addr % 8));
                 } else {
                     s_coils[reg_addr / 8] &= ~(1 << (reg_addr % 8));
                 }
             }
         } 
         else if (function_code == MODBUS_FC_WRITE_SINGLE_REG) {
             /* Single register */
             uint16_t value = (data[0] << 8) | data[1];
             s_holding_registers[address] = value;
         } 
         else if (function_code == MODBUS_FC_WRITE_MULTIPLE_REGS) {
             /* Multiple registers */
             for (uint16_t i = 0; i < count; i++) {
                 uint16_t value = (data[i * 2] << 8) | data[i * 2 + 1];
                 s_holding_registers[address + i] = value;
             }
         }
     }
     
     /* Prepare response */
     response[0] = function_code;
     
     if (function_code == MODBUS_FC_WRITE_SINGLE_COIL || function_code == MODBUS_FC_WRITE_SINGLE_REG) {
         /* Single write response: echo address and value */
         response[1] = (address >> 8) & 0xFF;
         response[2] = address & 0xFF;
         response[3] = data[0];
         response[4] = data[1];
         *response_len = 5;
     } else {
         /* Multiple write response: address and count */
         response[1] = (address >> 8) & 0xFF;
         response[2] = address & 0xFF;
         response[3] = (count >> 8) & 0xFF;
         response[4] = count & 0xFF;
         *response_len = 5;
     }
     
     return ESP_OK;
 }
 /**
  * @brief Send data over Modbus RTU interface
  * 
  * @param data Data buffer to send
  * @param length Length of data
  */
 static void modbus_rtu_send(uint8_t *data, uint16_t length)
 {
     if (data == NULL || length == 0) {
         return;
     }
     
     /* Set DE/RE pin high for transmit mode if in RS-485 mode with separate DE pin */
     if (s_modbus_ctx.de_pin != GPIO_NUM_NC) {
         gpio_set_level(s_modbus_ctx.de_pin, 1);
     }
     
     /* Send data */
     uart_write_bytes(s_modbus_ctx.uart_port, (const char *)data, length);
     uart_wait_tx_done(s_modbus_ctx.uart_port, pdMS_TO_TICKS(100));
     
     /* Set DE/RE pin low for receive mode if in RS-485 mode with separate DE pin */
     if (s_modbus_ctx.de_pin != GPIO_NUM_NC) {
         gpio_set_level(s_modbus_ctx.de_pin, 0);
     }
 }
 /**
  * @brief Receive data from Modbus RTU interface
  * 
  * @param data Buffer to store received data
  * @param length Pointer to store the length of received data
  * @param timeout_ms Timeout in milliseconds
  * 
  * @return ESP_OK if successful, or an error code
  */
 static esp_err_t modbus_rtu_receive(uint8_t *data, uint16_t *length, uint32_t timeout_ms)
 {
     if (data == NULL || length == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     uint16_t received = 0;
     uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
     uint32_t current_time;
     uint32_t elapsed_time;
     
     /* Clear any existing data in the RX buffer */
     uart_flush_input(s_modbus_ctx.uart_port);
     
     /* Read data until timeout or buffer full */
     do {
         int bytes_available = 0;
         esp_err_t ret = uart_get_buffered_data_len(s_modbus_ctx.uart_port, (size_t*)&bytes_available);
         if (ret != ESP_OK) {
             return ret;
         }
         
         if (bytes_available > 0) {
             /* Read available data */
             int bytes_read = uart_read_bytes(s_modbus_ctx.uart_port, 
                                             &data[received], 
                                             bytes_available, 
                                             pdMS_TO_TICKS(50));
             
             if (bytes_read < 0) {
                 return ESP_FAIL;
             }
             
             received += bytes_read;
             
             /* For Modbus RTU, we need to wait for a silent interval to detect end of frame */
             /* Reset timer if we got data */
             start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
         }
         
         /* Check timeout */
         current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
         elapsed_time = current_time - start_time;
         
         /* 3.5 character time silence indicates end of frame in Modbus RTU */
         /* We'll use a 10ms silence period as a simplification */
         if (received > 0 && elapsed_time >= 10) {
             break;
         }
         
         /* Small delay to prevent CPU hogging */
         vTaskDelay(pdMS_TO_TICKS(5));
         
     } while (elapsed_time < timeout_ms && received < MODBUS_RTU_MAX_ADU_LENGTH);
     
     *length = received;
     
     if (received == 0) {
         BESS_LOGW(TAG, "No data received within timeout period");
         return ESP_ERR_TIMEOUT;
     }
     
     return ESP_OK;
 }
 /**
  * @brief Execute registered callback for register operations
  * 
  * @param reg_type Register type
  * @param function_code Modbus function code
  * @param address Starting register address
  * @param count Number of registers
  * @param data Data buffer (for write operations or to store read results)
  * 
  * @return ESP_OK if successful, or an error code
  */
 static esp_err_t modbus_execute_callback(modbus_register_type_t reg_type, uint8_t function_code, 
                                        uint16_t address, uint16_t count, uint8_t *data)
 {
     if (data == NULL || reg_type > MODBUS_REG_DISCRETE_INPUTS) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (s_modbus_ctx.register_callbacks[reg_type] == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     /* Determine if this is a read or write operation */
     bool is_read;
     switch (function_code) {
         case MODBUS_FC_READ_COILS:
         case MODBUS_FC_READ_DISCRETE_INPUTS:
         case MODBUS_FC_READ_HOLDING_REGS:
         case MODBUS_FC_READ_INPUT_REGS:
             is_read = true;
             break;
             
         case MODBUS_FC_WRITE_SINGLE_COIL:
         case MODBUS_FC_WRITE_SINGLE_REG:
         case MODBUS_FC_WRITE_MULTIPLE_COILS:
         case MODBUS_FC_WRITE_MULTIPLE_REGS:
             is_read = false;
             break;
             
         default:
             return ESP_ERR_INVALID_ARG;
     }
     
     /* Call the registered callback */
     return s_modbus_ctx.register_callbacks[reg_type](
         is_read,
         address,
         count,
         data,
         s_modbus_ctx.callback_args[reg_type]
     );
 }
 
 /* Implementation of public functions */
 
 esp_err_t modbus_init(const modbus_config_t *config)
 {
     BESS_LOGI(TAG, "Initializing Modbus interface");
     
     if (config == NULL) {
         BESS_LOGE(TAG, "Invalid configuration parameter");
         return ESP_ERR_INVALID_ARG;
     }
     
     /* Initialize context */
     memset(&s_modbus_ctx, 0, sizeof(modbus_context_t));
     s_modbus_ctx.mode = config->mode;
     s_modbus_ctx.transport = config->transport;
     s_modbus_ctx.slave_address = config->slave_address;
     
     /* Create mutex for thread safety */
     s_modbus_ctx.modbus_mutex = xSemaphoreCreateMutex();
     if (s_modbus_ctx.modbus_mutex == NULL) {
         BESS_LOGE(TAG, "Failed to create Modbus mutex");
         return ESP_ERR_NO_MEM;
     }
     
     /* Initialize register memory */
     memset(s_holding_registers, 0, sizeof(s_holding_registers));
     memset(s_input_registers, 0, sizeof(s_input_registers));
     memset(s_coils, 0, sizeof(s_coils));
     memset(s_discrete_inputs, 0, sizeof(s_discrete_inputs));
     
     esp_err_t ret = ESP_OK;
     
     /* Initialize transport */
     if (config->transport == MODBUS_TRANSPORT_RTU) {
         ret = modbus_rtu_init(&config->rtu_config);
     } else if (config->transport == MODBUS_TRANSPORT_TCP) {
         ret = modbus_tcp_init(&config->tcp_config);
     } else {
         BESS_LOGE(TAG, "Unsupported transport mode");
         ret = ESP_ERR_NOT_SUPPORTED;
     }
     
     if (ret != ESP_OK) {
         vSemaphoreDelete(s_modbus_ctx.modbus_mutex);
         return ret;
     }
     
     BESS_LOGI(TAG, "Modbus interface initialized successfully");
     return ESP_OK;
 }
 
 esp_err_t modbus_deinit(void)
 {
     BESS_LOGI(TAG, "Deinitializing Modbus interface");
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     /* Clean up based on transport type */
     if (s_modbus_ctx.transport == MODBUS_TRANSPORT_RTU) {
         uart_driver_delete(s_modbus_ctx.uart_port);
     } else if (s_modbus_ctx.transport == MODBUS_TRANSPORT_TCP) {
         if (s_modbus_ctx.tcp_server_running) {
             s_modbus_ctx.tcp_server_running = false;
             /* Wait for TCP server task to finish */
             vTaskDelay(pdMS_TO_TICKS(100));
             if (s_modbus_ctx.server_socket >= 0) {
                 close(s_modbus_ctx.server_socket);
                 s_modbus_ctx.server_socket = -1;
             }
         }
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     vSemaphoreDelete(s_modbus_ctx.modbus_mutex);
     
     BESS_LOGI(TAG, "Modbus interface deinitialized");
     return ESP_OK;
 }
 
 esp_err_t modbus_register_callback(modbus_register_type_t reg_type, modbus_register_callback_t callback, void *arg)
 {
     if (callback == NULL || reg_type > MODBUS_REG_DISCRETE_INPUTS) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     s_modbus_ctx.register_callbacks[reg_type] = callback;
     s_modbus_ctx.callback_args[reg_type] = arg;
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ESP_OK;
 }
 
 esp_err_t modbus_set_register(modbus_register_type_t reg_type, uint16_t address, uint16_t value)
 {
     if (address >= MODBUS_MAX_REGISTERS) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     switch (reg_type) {
         case MODBUS_REG_HOLDING:
             s_holding_registers[address] = value;
             break;
         
         case MODBUS_REG_INPUT:
             s_input_registers[address] = value;
             break;
             
         case MODBUS_REG_COIL:
             if (value == 0) {
                 s_coils[address / 8] &= ~(1 << (address % 8));
             } else {
                 s_coils[address / 8] |= (1 << (address % 8));
             }
             break;
             
         case MODBUS_REG_DISCRETE_INPUTS:
             if (value == 0) {
                 s_discrete_inputs[address / 8] &= ~(1 << (address % 8));
             } else {
                 s_discrete_inputs[address / 8] |= (1 << (address % 8));
             }
             break;
             
         default:
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ESP_OK;
 }
 
 esp_err_t modbus_get_register(modbus_register_type_t reg_type, uint16_t address, uint16_t *value)
 {
     if (address >= MODBUS_MAX_REGISTERS || value == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     switch (reg_type) {
         case MODBUS_REG_HOLDING:
             *value = s_holding_registers[address];
             break;
         
         case MODBUS_REG_INPUT:
             *value = s_input_registers[address];
             break;
             
         case MODBUS_REG_COIL:
             *value = (s_coils[address / 8] >> (address % 8)) & 0x01;
             break;
             
         case MODBUS_REG_DISCRETE_INPUTS:
             *value = (s_discrete_inputs[address / 8] >> (address % 8)) & 0x01;
             break;
             
         default:
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return ESP_ERR_INVALID_ARG;
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ESP_OK;
 }
 
 esp_err_t modbus_master_read_registers(uint8_t slave_addr, uint8_t function_code, 
                                       uint16_t start_address, uint16_t quantity,
                                       uint16_t *values)
 {
     if (s_modbus_ctx.mode != MODBUS_MODE_MASTER || values == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     esp_err_t ret = ESP_OK;
     uint8_t request[12];
     uint16_t request_len = 0;
     
     if (s_modbus_ctx.transport == MODBUS_TRANSPORT_RTU) {
         request[0] = slave_addr;
         request[1] = function_code;
         request[2] = (start_address >> 8) & 0xFF; /* High byte of address */
         request[3] = start_address & 0xFF;        /* Low byte of address */
         request[4] = (quantity >> 8) & 0xFF;      /* High byte of quantity */
         request[5] = quantity & 0xFF;             /* Low byte of quantity */
         
         uint16_t crc = modbus_calc_crc(request, 6);
         request[6] = crc & 0xFF;                  /* Low byte of CRC */
         request[7] = (crc >> 8) & 0xFF;           /* High byte of CRC */
         
         request_len = 8;
         modbus_rtu_send(request, request_len);
         
         uint16_t expected_response_len = 3 + (quantity * 2) + 2; /* 3 header bytes + data + 2 CRC bytes */
         uint16_t response_len = 0;
         
         ret = modbus_rtu_receive(s_modbus_ctx.rx_buffer, &response_len, 1000);
         if (ret != ESP_OK || response_len != expected_response_len) {
             BESS_LOGE(TAG, "Failed to receive Modbus response or invalid length");
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return (ret != ESP_OK) ? ret : ESP_ERR_INVALID_RESPONSE;
         }
         
         /* Validate response */
         if (s_modbus_ctx.rx_buffer[0] != slave_addr || 
             s_modbus_ctx.rx_buffer[1] != function_code ||
             s_modbus_ctx.rx_buffer[2] != (quantity * 2)) {
             BESS_LOGE(TAG, "Invalid Modbus response format");
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return ESP_ERR_INVALID_RESPONSE;
         }
         
         /* Check CRC */
         uint16_t received_crc = (s_modbus_ctx.rx_buffer[response_len - 1] << 8) | 
                                  s_modbus_ctx.rx_buffer[response_len - 2];
         uint16_t calculated_crc = modbus_calc_crc(s_modbus_ctx.rx_buffer, response_len - 2);
         
         if (received_crc != calculated_crc) {
             BESS_LOGE(TAG, "Modbus CRC error");
             xSemaphoreGive(s_modbus_ctx.modbus_mutex);
             return ESP_ERR_INVALID_CRC;
         }
         
         /* Extract values */
         for (int i = 0; i < quantity; i++) {
             values[i] = (s_modbus_ctx.rx_buffer[3 + (i * 2)] << 8) | 
                          s_modbus_ctx.rx_buffer[4 + (i * 2)];
         }
     }
     else if (s_modbus_ctx.transport == MODBUS_TRANSPORT_TCP) {
         /* Not implemented in this example */
         ret = ESP_ERR_NOT_SUPPORTED;
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ret;
 }
 
 esp_err_t modbus_master_write_registers(uint8_t slave_addr, uint8_t function_code,
                                       uint16_t start_address, uint16_t quantity,
                                       const uint16_t *values)
 {
     if (s_modbus_ctx.mode != MODBUS_MODE_MASTER || values == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
     
     if (xSemaphoreTake(s_modbus_ctx.modbus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
         BESS_LOGE(TAG, "Failed to take Modbus mutex");
         return ESP_ERR_TIMEOUT;
     }
     
     esp_err_t ret = ESP_OK;
     
     if (s_modbus_ctx.transport == MODBUS_TRANSPORT_RTU) {
         if (function_code == MODBUS_FC_WRITE_SINGLE_REG) {
             /* Write single register */
             uint8_t request[8];
             request[0] = slave_addr;
             request[1] = function_code;
             request[2] = (start_address >> 8) & 0xFF;
             request[3] = start_address & 0xFF;
             request[4] = (values[0] >> 8) & 0xFF;
             request[5] = values[0] & 0xFF;
             
             uint16_t crc = modbus_calc_crc(request, 6);
             request[6] = crc & 0xFF;
             request[7] = (crc >> 8) & 0xFF;
             
             modbus_rtu_send(request, 8);
             
             /* Process response */
             uint16_t response_len = 0;
             ret = modbus_rtu_receive(s_modbus_ctx.rx_buffer, &response_len, 1000);
             if (ret != ESP_OK || response_len != 8) {
                 BESS_LOGE(TAG, "Failed to receive Modbus response or invalid length");
                 xSemaphoreGive(s_modbus_ctx.modbus_mutex);
                 return (ret != ESP_OK) ? ret : ESP_ERR_INVALID_RESPONSE;
             }
             
             /* Validate response */
             if (s_modbus_ctx.rx_buffer[0] != slave_addr || 
                 s_modbus_ctx.rx_buffer[1] != function_code) {
                 BESS_LOGE(TAG, "Invalid Modbus response format");
                 xSemaphoreGive(s_modbus_ctx.modbus_mutex);
                 return ESP_ERR_INVALID_RESPONSE;
             }
         }
         else if (function_code == MODBUS_FC_WRITE_MULTIPLE_REGS) {
             /* Write multiple registers */
             uint8_t *request = s_modbus_ctx.tx_buffer;
             uint16_t request_len = 7 + (quantity * 2);
             
             request[0] = slave_addr;
             request[1] = function_code;
             request[2] = (start_address >> 8) & 0xFF;
             request[3] = start_address & 0xFF;
             request[4] = (quantity >> 8) & 0xFF;
             request[5] = quantity & 0xFF;
             request[6] = quantity * 2; /* Byte count */
             
             /* Copy register values */
             for (int i = 0; i < quantity; i++) {
                 request[7 + (i * 2)] = (values[i] >> 8) & 0xFF;
                 request[8 + (i * 2)] = values[i] & 0xFF;
             }
             
             uint16_t crc = modbus_calc_crc(request, request_len);
             request[request_len] = crc & 0xFF;
             request[request_len + 1] = (crc >> 8) & 0xFF;
             
             modbus_rtu_send(request, request_len + 2);
             
             /* Process response */
             uint16_t response_len = 0;
             ret = modbus_rtu_receive(s_modbus_ctx.rx_buffer, &response_len, 1000);
             if (ret != ESP_OK || response_len != 8) {
                 BESS_LOGE(TAG, "Failed to receive Modbus response or invalid length");
                 xSemaphoreGive(s_modbus_ctx.modbus_mutex);
                 return (ret != ESP_OK) ? ret : ESP_ERR_INVALID_RESPONSE;
             }
             
             /* Validate response */
             if (s_modbus_ctx.rx_buffer[0] != slave_addr || 
                 s_modbus_ctx.rx_buffer[1] != function_code) {
                 BESS_LOGE(TAG, "Invalid Modbus response format");
                 xSemaphoreGive(s_modbus_ctx.modbus_mutex);
                 return ESP_ERR_INVALID_RESPONSE;
             }
         }
         else {
             ret = ESP_ERR_INVALID_ARG;
         }
     }
     else if (s_modbus_ctx.transport == MODBUS_TRANSPORT_TCP) {
         /* Not implemented in this example */
         ret = ESP_ERR_NOT_SUPPORTED;
     }
     
     xSemaphoreGive(s_modbus_ctx.modbus_mutex);
     return ret;
 }
 
 /* Implementation of internal functions */
 
 static esp_err_t modbus_rtu_init(const modbus_rtu_config_t *config)
 {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     s_modbus_ctx.uart_port = config->uart_port;
     s_modbus_ctx.rts_pin = config->rts_pin;
     s_modbus_ctx.de_pin = config->de_pin;
     
     /* Configure UART parameters */
     uart_config_t uart_config = {
         .baud_rate = config->baudrate,
         .data_bits = config->data_bits,
         .parity = config->parity,
         .stop_bits = config->stop_bits,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .rx_flow_ctrl_thresh = 0,
         .source_clk = UART_SCLK_DEFAULT,
     };
     
     esp_err_t ret = uart_param_config(config->uart_port, &uart_config);
     if (ret != ESP_OK) {
         BESS_LOGE(TAG, "Failed to configure UART parameters: %d", ret);
         return ret;
     }
     
     /* Set pins */
     ret = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, 
                        config->rts_pin, UART_PIN_NO_CHANGE);
     if (ret != ESP_OK) {
         BESS_LOGE(TAG, "Failed to set UART pins: %d", ret);
         return ret;
     }
     
     /* Configure RS-485 half-duplex mode if needed */
     if (config->is_rs485) {
         ret = uart_set_mode(config->uart_port, UART_MODE_RS485_HALF_DUPLEX);
         if (ret != ESP_OK) {
             BESS_LOGE(TAG, "Failed to set UART RS-485 mode: %d", ret);
             return ret;
         }
     }
     
     /* Install UART driver */
     ret = uart_driver_install(config->uart_port, 
                              MODBUS_RTU_MAX_ADU_LENGTH * 2, 
                              MODBUS_RTU_MAX_ADU_LENGTH * 2, 
                              0, NULL, 0);
     if (ret != ESP_OK) {
         BESS_LOGE(TAG, "Failed to install UART driver: %d", ret);
         return ret;
     }
     
     /* If using separate DE pin for RS-485 rather than RTS, configure it */
     if (config->is_rs485 && config->de_pin != GPIO_NUM_NC) {
         gpio_config_t io_conf = {
             .intr_type = GPIO_INTR_DISABLE,
             .mode = GPIO_MODE_OUTPUT,
             .pin_bit_mask = (1ULL << config->de_pin),
             .pull_down_en = 0,
             .pull_up_en = 0,
         };
         
         ret = gpio_config(&io_conf);
         if (ret != ESP_OK) {
             BESS_LOGE(TAG, "Failed to configure DE pin: %d", ret);
             uart_driver_delete(config->uart_port);
             return ret;
         }
         
         gpio_set_level(config->de_pin, 0); /* Initially set to receive mode */
     }
     
     BESS_LOGI(TAG, "Modbus RTU initialized on UART%d, %d baud", 
              config->uart_port, config->baudrate);
     return ESP_OK;
 }
 
 static esp_err_t modbus_tcp_init(const modbus_tcp_config_t *config)
 {
     if (config == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     s_modbus_ctx.tcp_port = config->port;
     s_modbus_ctx.server_socket = -1;
     
     if (s_modbus_ctx.mode == MODBUS_MODE_SLAVE) {
         /* Start TCP server task for slave mode */
         s_modbus_ctx.tcp_server_running = true;
         
         if (xTaskCreatePinnedToCore(modbus_tcp_server_task, "modbus_tcp", 
                                     4096, NULL, config->task_priority, 
                                     &s_modbus_ctx.tcp_task_handle, config->core_id) != pdPASS) {
             BESS_LOGE(TAG, "Failed to create Modbus TCP server task");
             return ESP_ERR_NO_MEM;
         }
         
         BESS_LOGI(TAG, "Modbus TCP server initialized on port %d", config->port);
     } else {
         /* Master mode initialization not implemented in this example */
         BESS_LOGI(TAG, "Modbus TCP master initialized");
     }
     
     return ESP_OK;
 }
 
 static void modbus_tcp_server_task(void *pvParameters)
 {
     struct sockaddr_in server_addr = {
         .sin_family = AF_INET,
         .sin_addr.s_addr = htonl(INADDR_ANY),
         .sin_port = htons(s_modbus_ctx.tcp_port),
     };
     
     int server_socket = socket(AF_INET, SOCK_STREAM, 0);
     if (server_socket < 0) {
         BESS_LOGE(TAG, "Failed to create socket: %d", errno);
         vTaskDelete(NULL);
         return;
     }
     
     int opt = 1;
     setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
     
     if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
         BESS_LOGE(TAG, "Failed to bind socket: %d", errno);
         close(server_socket);
         vTaskDelete(NULL);
         return;
     }
     
     if (listen(server_socket, 5) != 0) {
         BESS_LOGE(TAG, "Failed to listen on socket: %d", errno);
         close(server_socket);
         vTaskDelete(NULL);
         return;
     }
     
     s_modbus_ctx.server_socket = server_socket;
     BESS_LOGI(TAG, "Modbus TCP server listening on port %d", s_modbus_ctx.tcp_port);
     
     struct sockaddr_in client_addr;
     socklen_t client_addr_len = sizeof(client_addr);
     
     while (s_modbus_ctx.tcp_server_running) {
         /* Accept client connection */
         int client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &client_addr_len);
         if (client_socket < 0) {
             if (errno != EINTR) {
                 BESS_LOGE(TAG, "Failed to accept client connection: %d", errno);
             }
             vTaskDelay(pdMS_TO_TICKS(100));
             continue;
         }
         
         char client_ip[16];
         inet_ntoa_r(client_addr.sin_addr, client_ip, sizeof(client_ip));
         BESS_LOGI(TAG, "Modbus TCP client connected: %s:%d", client_ip, ntohs(client_addr.sin_port));
         
         /* Process client requests */
         uint8_t request[MODBUS_TCP_MAX_ADU_LENGTH];
         uint8_t response[MODBUS_TCP_MAX_ADU_LENGTH];
         
         struct timeval timeout = {
             .tv_sec = 5,
             .tv_usec = 0,
         };
         setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
         
         while (s_modbus_ctx.tcp_server_running) {
             /* Read Modbus TCP request */
             ssize_t bytes_read = read(client_socket, request, MODBUS_TCP_MAX_ADU_LENGTH);
             if (bytes_read <= 0) {
                 if (bytes_read == 0 || errno == ECONNRESET) {
                     BESS_LOGI(TAG, "Modbus TCP client disconnected");
                 } else if (errno != EAGAIN) {
                     BESS_LOGE(TAG, "Error reading from socket: %d", errno);
                 }
                 break;
             }
             
             /* Process request */
             uint16_t response_len = 0;
             
             /* Check minimal Modbus TCP frame length */
             if (bytes_read < MODBUS_TCP_HEADER_LENGTH) {
                 BESS_LOGE(TAG, "Received Modbus TCP message too short: %d bytes", bytes_read);
                 break;
             }
             
             /* Extract Modbus PDU from TCP frame and process it */
             uint16_t transaction_id = (request[0] << 8) | request[1];
             uint16_t length = (request[4] << 8) | request[5];
             uint8_t unit_id = request[6];
             
             /* Validate length */
             if (length != bytes_read - 6) {
                 BESS_LOGE(TAG, "Modbus TCP length mismatch: %d != %d", length, bytes_read - 6);
                 break;
             }
             
             /* Process request if it's addressed to us or is a broadcast */
             if (unit_id == s_modbus_ctx.slave_address || unit_id == MODBUS_BROADCAST_ADDRESS) {
                 uint16_t pdu_length = bytes_read - MODBUS_TCP_HEADER_LENGTH;
                 
                 /* Copy TCP header to response */
                 memcpy(response, request, MODBUS_TCP_HEADER_LENGTH);
                 
                 /* Process Modbus PDU */
                 uint16_t resp_pdu_len = 0;
                 esp_err_t ret = modbus_process_request(&request[MODBUS_TCP_HEADER_LENGTH], pdu_length,
                                                      &response[MODBUS_TCP_HEADER_LENGTH], &resp_pdu_len);
                 
                 if (ret == ESP_OK) {
                     /* Update length field in Modbus TCP header */
                     response[4] = (resp_pdu_len + 1) >> 8;
                     response[5] = (resp_pdu_len + 1) & 0xFF;
                     
                     /* Send response */
                     response_len = resp_pdu_len + MODBUS_TCP_HEADER_LENGTH;
                     send(client_socket, response, response_len, 0);
                 } else {
                     BESS_LOGE(TAG, "Failed to process Modbus request: %d", ret);
                 }
             }
         }
         
         close(client_socket);
     }
     
     close(server_socket);
     s_modbus_ctx.server_socket = -1;
     BESS_LOGI(TAG, "Modbus TCP server stopped");
     
     vTaskDelete(NULL);
}