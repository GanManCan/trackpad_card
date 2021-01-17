/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <stdint.h>

#include "tensorflow/lite/c/common.h"

#define UART_NUMBER (UART_NUM_0)
#define TXD_PIN (GPIO_NUM_1) //(U0TXD)
#define RXD_PIN (GPIO_NUM_3) //(U0RXD)
#define RX_BUF_SIZE 1024

// Button Inputs
#define CALC_PIN  (GPIO_NUM_0)   // Calculate GPIO pin
#define RST_PIN   (GPIO_NUM_19)  // Rst GPIO Pin


//I2C Defines
#define IQS572_I2C_R_ADDR 0xe9
#define IQS572_I2C_W_ADDR 0xe8
#define IQS572_RDY (GPIO_NUM_25)     // gpio number for IQS527 RDY signal

#define SAMPLE_PERIOD_MS		200

#define I2C_SCL_IO				27	               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO				26	               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ				100000           /*!< I2C master clock frequency */
#define I2C_PORT_NUM			I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */

// I2C common protocol defines
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

void initUart(uart_port_t uart_num);
void readUartBytes(float *data, int imageSize);
int sendData(const char *data);
void normalizeImageData(float *data, int imageSize);
void sendBackPredictions(TfLiteTensor *output);
void zero_input(float *data, int imageSize);
void set_input(float *tf_data, uint8_t *pad_data, int imageSize);
void initI2C();
extern "C" {esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size);}
extern "C" {esp_err_t rdIQS572(uint8_t reg, uint8_t *pdata, uint8_t count);}
