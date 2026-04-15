#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO    22    // Clock pin
#define I2C_MASTER_SDA_IO    21    // Data pin
#define I2C_MASTER_NUM       I2C_NUM_0 
#define DS1307_ADDR          0x68  // DS1307 I2C address

void init_ds1307_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100kHz standard mode
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

uint8_t bcd_to_decimal(uint8_t val) {
    return ((val / 16 * 10) + (val % 16));
}

void rtc_read_task(void *pvParameters) {
    while (1) {
        uint8_t reg_addr = 0x00; // Start reading at register 0x00 (Seconds)
        uint8_t data[7];         // Buffer for sec, min, hour, day, date, month, year

        // I2C combined transaction: Write register address then read 7 bytes
        esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, DS1307_ADDR, 
                                                    &reg_addr, 1, 
                                                    data, 7, 
                                                    pdMS_TO_TICKS(1000));

        if (ret == ESP_OK) {
            // DS1307 data format: data[0]=sec, data[1]=min, data[2]=hour
            printf("Time: %02d:%02d:%02d\n",
                   bcd_to_decimal(data[2] & 0x3F), // Hours (bit 6 is 12/24h toggle)
                   bcd_to_decimal(data[1]),        // Minutes
                   bcd_to_decimal(data[0] & 0x7F)); // Seconds (bit 7 is Clock Halt)
        } else {
            printf("I2C Read Failed: %s\n", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    init_ds1307_i2c();
    xTaskCreate(rtc_read_task, "rtc_read_task", 2048, NULL, 5, NULL);
}
