#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

spi_device_handle_t rtc_spi;

void init_ds1306_spi()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 1,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_POSITIVE_CS,
    };

    // Initialize the SPI bus and add the device
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI3_HOST, &devcfg, &rtc_spi);
}

uint8_t bcd_to_decimal(uint8_t val)
{
    return ((val / 16 * 10) + (val % 16));
}

void rtc_read_task(void *pvParameters)
{
    while (1)
    {
        uint8_t addr = 0x00; // Start address for Seconds (Read mode: bit 7 is 0)
        uint8_t data[7];     // To hold sec, min, hour, day, date, month, year

        spi_transaction_t t = {
            .length = 8 * 7, // 7 bytes to read
            .addr = addr,
            .rx_buffer = data,
        };

        spi_device_polling_transmit(rtc_spi, &t);

        printf("Time: %02d:%02d:%02d\n",
               bcd_to_decimal(data[2] & 0x3F), // Hours
               bcd_to_decimal(data[1]),        // Minutes
               bcd_to_decimal(data[0]));       // Seconds

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    init_ds1306_spi();
    xTaskCreate(rtc_read_task, "rtc_read_task", 2048, NULL, 5, NULL);
}
