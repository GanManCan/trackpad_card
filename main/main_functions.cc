#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <stdint.h>

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "main_functions.h"
#include "model_data.h"
#include "esp_log.h"


void initUart(uart_port_t uart_num)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB};

    // We will not use a buffer for sending data.
    uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Cnfigure the physical GPIO pins to which the UART device will be connected.
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void readUartBytes(float *data, int imageSize)
{
    uint8_t *rxBuffer = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    int rxIdx = 0;
    int rxBytes = 0;

    for (;;)
    {
        rxBytes = uart_read_bytes(UART_NUMBER, rxBuffer, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            for (int i = 0; i < rxBytes; rxIdx++, i++)
            {
                data[rxIdx] = static_cast<float>(rxBuffer[i]) / 255.0f;
            }
        }
        if (rxIdx >= imageSize - 1)
        {
            rxIdx = 0;
            break;
        }
    }
    return;
}

void zero_input(float *data, int imageSize)
{

  for(int rxIdx=0;rxIdx<imageSize; rxIdx++){
    data[rxIdx]=0;
  }
  return;
}

void set_input(float *tf_data, uint8_t *pad_data, int imageSize)
{
  int idx=0;
  uint8_t x_value,y_value;

  // Increase pointer index to X_value location
  pad_data++;
  x_value = *pad_data; // Cast uint8_t to float
  printf("Value_x:  %d\n", x_value);
  //Increase pointer to Y_value location
  pad_data++;
  pad_data++;
  y_value  = *pad_data;
  printf("Value_y:  %d\n", y_value);

  idx = x_value*y_value;
  printf("Index:   %d\n",idx);

  // Range check idx
  if(idx>=0 && idx < imageSize){
    tf_data[idx]=1;
    printf("Data stored\n");
  }
  printf("\n");
  return;
}

int sendData(const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUMBER, data, len);
    return txBytes;
}

void normalizeImageData(float *data, int imageSize)
{
    for (int i = 0; i < imageSize; i++)
    {
          //data[i] = data[i] / 255.0f;
      }
  }

void sendBackPredictions(TfLiteTensor *output)
{
    // Read the predicted y values from the model's output tensor
    char str[250] = {0};
    char buf[20] = {0};
    int numElements = output->dims->data[1];
    for (int i = 0; i < numElements; i++)
    {
        sprintf(buf, "%e,", static_cast<float>(output->data.f[i]));
        strcat(str, buf);
    }
    strcat(str, "\n");
    sendData(str);
}

void initI2C()
{
    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode,
    		I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
extern "C" {esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, (IQS572_I2C_W_ADDR), ACK_CHECK_EN);
    // send register we want
    //i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x16, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, (IQS572_I2C_R_ADDR), ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    // Write to reg 0xEEEE to end rst rdy signal
    /*
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IQS572_I2C_W_ADDR), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xEE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xEE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xEE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    */
    i2c_cmd_link_delete(cmd);

    return ret;
}
}
extern "C" {esp_err_t rdIQS572(uint8_t reg, uint8_t *pdata, uint8_t count)
{
	return( i2c_master_read_slave_reg( I2C_NUM_1, IQS572_I2C_R_ADDR,  reg, pdata, count ) );
}
}
