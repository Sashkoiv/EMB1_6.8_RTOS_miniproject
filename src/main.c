#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "rom/ets_sys.h"

#define APP_TAG "POWER_LOGGER"

// =========================
// User configuration
// =========================

#define UART_PORT_NUM               UART_NUM_0
#define UART_BAUD_RATE              115200
#define UART_RX_BUFFER_SIZE         256
#define UART_TX_BUFFER_SIZE         0

#define WAKE_INTERVAL_MS            1000U
#define LOG_INTERVAL_MINUTES        1U      // change to 5 if needed
#define LOG_INTERVAL_SECONDS        (LOG_INTERVAL_MINUTES * 60U)

#define FLASH_SECTOR_SIZE_BYTES     4096U
#define LOG_PARTITION_LABEL         "logdata"

#define DISPLAY_QUEUE_LENGTH        4U
#define SENSOR_QUEUE_LENGTH         8U
#define COMMAND_QUEUE_LENGTH        8U

#define SENSOR_TASK_STACK_SIZE      4096U
#define UART_TASK_STACK_SIZE        4096U
#define DISPLAY_TASK_STACK_SIZE     4096U
#define MANAGER_TASK_STACK_SIZE     6144U
#define FLASH_TASK_STACK_SIZE       4096U

#define SENSOR_TASK_PRIORITY        5U
#define UART_TASK_PRIORITY          4U
#define DISPLAY_TASK_PRIORITY       3U
#define MANAGER_TASK_PRIORITY       6U
#define FLASH_TASK_PRIORITY         4U

// =========================
// Types
// =========================

typedef struct __attribute__((packed)) {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    int16_t  temperature_c_x100;
    uint16_t humidity_pct_x100;
    uint16_t pressure_mmhg;
    uint8_t  reserved[3]; // makes record 16 bytes total
} log_record_t;

_Static_assert(sizeof(log_record_t) == 16, "log_record_t must be 16 bytes");

#define LOG_RECORDS_PER_SECTOR (FLASH_SECTOR_SIZE_BYTES / sizeof(log_record_t))

typedef struct {
    log_record_t records[LOG_RECORDS_PER_SECTOR];
    uint16_t count; // RAM-only metadata; not written to flash
} ram_log_buffer_t;

typedef struct {
    log_record_t latest_record;
    bool logging_enabled;
    bool partition_full;
    uint16_t buffered_records;
    uint32_t sectors_written;
} display_snapshot_t;

typedef enum {
    UART_CMD_NONE = 0,
    UART_CMD_HELP,
    UART_CMD_STATUS,
    UART_CMD_FLUSH,
    UART_CMD_ERASE,
    UART_CMD_LOG_ON,
    UART_CMD_LOG_OFF
} uart_command_type_t;

typedef struct {
    uart_command_type_t type;
} uart_command_t;

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} rtc_datetime_t;

typedef struct {
    int32_t temperature_c_x100;
    uint32_t humidity_pct_x100;
    uint32_t pressure_mmhg;
} bme280_sample_t;

typedef struct {
    bool logging_enabled;
    bool partition_full;
    bool flash_flush_in_progress;
    uint32_t elapsed_seconds_since_last_log;
    uint32_t flash_write_offset_bytes;
    uint32_t flash_sectors_written;
    log_record_t latest_record;
} app_state_t;

// =========================
// Globals
// =========================

static QueueHandle_t g_sensor_queue = NULL;
static QueueHandle_t g_command_queue = NULL;
static QueueHandle_t g_display_queue = NULL;

static SemaphoreHandle_t g_i2c_mutex = NULL;
static SemaphoreHandle_t g_flash_done_semaphore = NULL;
static SemaphoreHandle_t g_log_buffer_mutex = NULL;

static TaskHandle_t g_sensor_task_handle = NULL;
static TaskHandle_t g_flash_task_handle = NULL;

static const esp_partition_t *g_log_partition = NULL;

static ram_log_buffer_t g_log_buffer;
static app_state_t g_app_state;

static uint8_t g_uart_rx_line[UART_RX_BUFFER_SIZE];

// =========================
// Hardware abstraction layer
// Replace these with your real drivers
// =========================

static esp_err_t hal_i2c_init(void)
{
    // TODO: initialize I2C master here
    // You can reuse your existing ESP-IDF v5.5.2 I2C code.
    return ESP_OK;
}

static esp_err_t hal_bme280_init(void)
{
    // TODO: initialize BME280 here
    return ESP_OK;
}

static esp_err_t hal_ds1306_init(void)
{
    // TODO: initialize DS1306 here
    // DS1306 usually uses SPI.
    return ESP_OK;
}

static esp_err_t hal_ssd1306_init(void)
{
    // TODO: initialize SSD1306 here
    return ESP_OK;
}

static esp_err_t hal_bme280_read_sample(bme280_sample_t *sample_out)
{
    if (sample_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // TODO: replace this dummy data with real BME280 reading
    static int32_t fake_temp = 2300;
    static uint32_t fake_humidity = 5000;
    static uint32_t fake_pressure = 760;

    fake_temp += 1;
    if (fake_temp > 2500) {
        fake_temp = 2300;
    }

    fake_humidity += 3;
    if (fake_humidity > 6500) {
        fake_humidity = 5000;
    }

    fake_pressure += 1;
    if (fake_pressure > 765) {
        fake_pressure = 760;
    }

    sample_out->temperature_c_x100 = fake_temp;
    sample_out->humidity_pct_x100 = fake_humidity;
    sample_out->pressure_mmhg = fake_pressure;

    return ESP_OK;
}

static esp_err_t hal_ds1306_read_datetime(rtc_datetime_t *datetime_out)
{
    if (datetime_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // TODO: replace with real DS1306 reading
    // Dummy clock for demonstration
    static uint32_t fake_seconds = 0;
    fake_seconds++;

    datetime_out->year = 2026;
    datetime_out->month = 4;
    datetime_out->day = 15;
    datetime_out->hour = (fake_seconds / 3600U) % 24U;
    datetime_out->minute = (fake_seconds / 60U) % 60U;
    datetime_out->second = fake_seconds % 60U;

    return ESP_OK;
}

static esp_err_t hal_ssd1306_show_snapshot(const display_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // TODO: replace with real SSD1306 rendering
    // This stub logs to UART so the architecture can still be tested.
    ESP_LOGI(APP_TAG,
             "DISPLAY | %04u-%02u-%02u %02u:%02u:%02u | T=%d.%02dC H=%u.%02u%% P=%u mmHg | log=%s | full=%s | buf=%u | sectors=%" PRIu32,
             snapshot->latest_record.year,
             snapshot->latest_record.month,
             snapshot->latest_record.day,
             snapshot->latest_record.hour,
             snapshot->latest_record.minute,
             snapshot->latest_record.second,
             snapshot->latest_record.temperature_c_x100 / 100,
             abs(snapshot->latest_record.temperature_c_x100 % 100),
             snapshot->latest_record.humidity_pct_x100 / 100,
             snapshot->latest_record.humidity_pct_x100 % 100,
             snapshot->latest_record.pressure_mmhg,
             snapshot->logging_enabled ? "ON" : "OFF",
             snapshot->partition_full ? "YES" : "NO",
             snapshot->buffered_records,
             snapshot->sectors_written);

    return ESP_OK;
}

// =========================
// Utility functions
// =========================

static void uart_print_line(const char *text)
{
    if (text == NULL) {
        return;
    }

    printf("%s\r\n", text);
}

static void print_help(void)
{
    uart_print_line("Commands:");
    uart_print_line("  help      - show commands");
    uart_print_line("  status    - show current status");
    uart_print_line("  flush     - force write RAM buffer to flash");
    uart_print_line("  erase     - erase log partition and clear buffer");
    uart_print_line("  log on    - enable logging");
    uart_print_line("  log off   - disable logging");
}

static void print_status(void)
{
    printf("STATUS: logging=%s, partition_full=%s, flash_busy=%s, buffer_count=%u/%u, sectors_written=%" PRIu32 ", write_offset=%" PRIu32 "\r\n",
           g_app_state.logging_enabled ? "ON" : "OFF",
           g_app_state.partition_full ? "YES" : "NO",
           g_app_state.flash_flush_in_progress ? "YES" : "NO",
           g_log_buffer.count,
           (unsigned)LOG_RECORDS_PER_SECTOR,
           g_app_state.flash_sectors_written,
           g_app_state.flash_write_offset_bytes);

    printf("LATEST: %04u-%02u-%02u %02u:%02u:%02u | T=%d.%02dC | H=%u.%02u%% | P=%u mmHg\r\n",
           g_app_state.latest_record.year,
           g_app_state.latest_record.month,
           g_app_state.latest_record.day,
           g_app_state.latest_record.hour,
           g_app_state.latest_record.minute,
           g_app_state.latest_record.second,
           g_app_state.latest_record.temperature_c_x100 / 100,
           abs(g_app_state.latest_record.temperature_c_x100 % 100),
           g_app_state.latest_record.humidity_pct_x100 / 100,
           g_app_state.latest_record.humidity_pct_x100 % 100,
           g_app_state.latest_record.pressure_mmhg);
}

static display_snapshot_t make_display_snapshot(void)
{
    display_snapshot_t snapshot_local;
    memset(&snapshot_local, 0, sizeof(snapshot_local));

    snapshot_local.latest_record = g_app_state.latest_record;
    snapshot_local.logging_enabled = g_app_state.logging_enabled;
    snapshot_local.partition_full = g_app_state.partition_full;
    snapshot_local.buffered_records = g_log_buffer.count;
    snapshot_local.sectors_written = g_app_state.flash_sectors_written;

    return snapshot_local;
}

static void send_display_snapshot(void)
{
    display_snapshot_t snapshot_local = make_display_snapshot();

    if (xQueueOverwrite(g_display_queue, &snapshot_local) != pdPASS) {
        ESP_LOGW(APP_TAG, "Failed to update display queue");
    }
}

static bool flash_partition_has_room_for_next_sector(void)
{
    if (g_log_partition == NULL) {
        return false;
    }

    return (g_app_state.flash_write_offset_bytes + FLASH_SECTOR_SIZE_BYTES) <= g_log_partition->size;
}

static esp_err_t erase_entire_log_partition(void)
{
    if (g_log_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(APP_TAG, "Erasing whole log partition...");
    return esp_partition_erase_range(g_log_partition, 0, g_log_partition->size);
}

static void clear_ram_log_buffer(void)
{
    memset(&g_log_buffer, 0xFF, sizeof(g_log_buffer.records));
    g_log_buffer.count = 0;
}

static void stop_logging_and_notify_full(void)
{
    g_app_state.logging_enabled = false;
    g_app_state.partition_full = true;

    uart_print_line("WARNING: flash log partition is full. Logging stopped.");
    send_display_snapshot();
}

static esp_err_t append_record_to_ram_buffer(const log_record_t *record_in)
{
    if (record_in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_log_buffer.count >= LOG_RECORDS_PER_SECTOR) {
        return ESP_ERR_NO_MEM;
    }

    g_log_buffer.records[g_log_buffer.count] = *record_in;
    g_log_buffer.count++;

    return ESP_OK;
}

static bool ram_log_buffer_is_full(void)
{
    return g_log_buffer.count >= LOG_RECORDS_PER_SECTOR;
}

static esp_err_t parse_uart_command(const char *line_in, uart_command_t *command_out)
{
    if ((line_in == NULL) || (command_out == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    command_out->type = UART_CMD_NONE;

    if (strcmp(line_in, "help") == 0) {
        command_out->type = UART_CMD_HELP;
    } else if (strcmp(line_in, "status") == 0) {
        command_out->type = UART_CMD_STATUS;
    } else if (strcmp(line_in, "flush") == 0) {
        command_out->type = UART_CMD_FLUSH;
    } else if (strcmp(line_in, "erase") == 0) {
        command_out->type = UART_CMD_ERASE;
    } else if (strcmp(line_in, "log on") == 0) {
        command_out->type = UART_CMD_LOG_ON;
    } else if (strcmp(line_in, "log off") == 0) {
        command_out->type = UART_CMD_LOG_OFF;
    } else {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

static esp_err_t build_log_record(log_record_t *record_out,
                                  const rtc_datetime_t *datetime_in,
                                  const bme280_sample_t *sample_in)
{
    if ((record_out == NULL) || (datetime_in == NULL) || (sample_in == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(record_out, 0, sizeof(*record_out));

    record_out->year = datetime_in->year;
    record_out->month = datetime_in->month;
    record_out->day = datetime_in->day;
    record_out->hour = datetime_in->hour;
    record_out->minute = datetime_in->minute;
    record_out->second = datetime_in->second;

    record_out->temperature_c_x100 = (int16_t)sample_in->temperature_c_x100;
    record_out->humidity_pct_x100 = (uint16_t)sample_in->humidity_pct_x100;
    record_out->pressure_mmhg = (uint16_t)sample_in->pressure_mmhg;

    return ESP_OK;
}

// =========================
// Tasks
// =========================

static void sensor_task(void *pvParameters)
{
    (void)pvParameters;

    rtc_datetime_t current_datetime;
    bme280_sample_t current_sample;
    log_record_t current_record;

    for (;;) {
        uint32_t notify_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notify_value == 0U) {
            continue;
        }

        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            ESP_LOGW(APP_TAG, "Sensor Task: failed to lock I2C mutex");
            continue;
        }

        esp_err_t read_result = hal_bme280_read_sample(&current_sample);
        if (read_result == ESP_OK) {
            read_result = hal_ds1306_read_datetime(&current_datetime);
        }

        xSemaphoreGive(g_i2c_mutex);

        if (read_result != ESP_OK) {
            ESP_LOGE(APP_TAG, "Sensor Task: sensor/RTC read failed: %s", esp_err_to_name(read_result));
            continue;
        }

        read_result = build_log_record(&current_record, &current_datetime, &current_sample);
        if (read_result != ESP_OK) {
            ESP_LOGE(APP_TAG, "Sensor Task: failed to build record");
            continue;
        }

        if (xQueueSend(g_sensor_queue, &current_record, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(APP_TAG, "Sensor Task: sensor queue full");
        }
    }
}

static void uart_task(void *pvParameters)
{
    (void)pvParameters;

    int line_length = 0;
    memset(g_uart_rx_line, 0, sizeof(g_uart_rx_line));

    print_help();

    for (;;) {
        uint8_t received_char = 0;
        int bytes_read = uart_read_bytes(UART_PORT_NUM, &received_char, 1, pdMS_TO_TICKS(100));

        if (bytes_read <= 0) {
            continue;
        }

        if ((received_char == '\r') || (received_char == '\n')) {
            if (line_length == 0) {
                continue;
            }

            g_uart_rx_line[line_length] = '\0';

            uart_command_t parsed_command;
            esp_err_t parse_result = parse_uart_command((const char *)g_uart_rx_line, &parsed_command);
            if (parse_result == ESP_OK) {
                if (xQueueSend(g_command_queue, &parsed_command, pdMS_TO_TICKS(100)) != pdPASS) {
                    ESP_LOGW(APP_TAG, "UART Task: command queue full");
                }
            } else {
                uart_print_line("Unknown command. Type: help");
            }

            line_length = 0;
            memset(g_uart_rx_line, 0, sizeof(g_uart_rx_line));
            continue;
        }

        if ((line_length < (UART_RX_BUFFER_SIZE - 1)) && (received_char >= 32U) && (received_char <= 126U)) {
            g_uart_rx_line[line_length] = received_char;
            line_length++;
        }
    }
}

static void display_task(void *pvParameters)
{
    (void)pvParameters;

    display_snapshot_t snapshot_local;

    for (;;) {
        if (xQueueReceive(g_display_queue, &snapshot_local, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            ESP_LOGW(APP_TAG, "Display Task: failed to lock I2C mutex");
            continue;
        }

        esp_err_t display_result = hal_ssd1306_show_snapshot(&snapshot_local);

        xSemaphoreGive(g_i2c_mutex);

        if (display_result != ESP_OK) {
            ESP_LOGE(APP_TAG, "Display Task: display update failed: %s", esp_err_to_name(display_result));
        }
    }
}

static void flash_writer_task(void *pvParameters)
{
    (void)pvParameters;

    static log_record_t local_sector_copy[LOG_RECORDS_PER_SECTOR];

    for (;;) {
        uint32_t notify_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notify_value == 0U) {
            continue;
        }

        if (xSemaphoreTake(g_log_buffer_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(APP_TAG, "Flash Task: failed to lock log buffer mutex");
            xSemaphoreGive(g_flash_done_semaphore);
            continue;
        }

        const uint16_t record_count_to_write = g_log_buffer.count;
        memcpy(local_sector_copy, g_log_buffer.records, sizeof(local_sector_copy));

        xSemaphoreGive(g_log_buffer_mutex);

        if (record_count_to_write == 0U) {
            ESP_LOGW(APP_TAG, "Flash Task: flush requested with empty buffer");
            xSemaphoreGive(g_flash_done_semaphore);
            continue;
        }

        if (!flash_partition_has_room_for_next_sector()) {
            g_app_state.partition_full = true;
            xSemaphoreGive(g_flash_done_semaphore);
            continue;
        }

        esp_err_t erase_result = esp_partition_erase_range(
            g_log_partition,
            g_app_state.flash_write_offset_bytes,
            FLASH_SECTOR_SIZE_BYTES);

        if (erase_result != ESP_OK) {
            ESP_LOGE(APP_TAG, "Flash Task: erase failed: %s", esp_err_to_name(erase_result));
            xSemaphoreGive(g_flash_done_semaphore);
            continue;
        }

        size_t bytes_to_write = ((size_t)record_count_to_write) * sizeof(log_record_t);

        esp_err_t write_result = esp_partition_write(
            g_log_partition,
            g_app_state.flash_write_offset_bytes,
            local_sector_copy,
            bytes_to_write);

        if (write_result != ESP_OK) {
            ESP_LOGE(APP_TAG, "Flash Task: write failed: %s", esp_err_to_name(write_result));
            xSemaphoreGive(g_flash_done_semaphore);
            continue;
        }

        g_app_state.flash_write_offset_bytes += FLASH_SECTOR_SIZE_BYTES;
        g_app_state.flash_sectors_written++;

        if (xSemaphoreTake(g_log_buffer_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            clear_ram_log_buffer();
            xSemaphoreGive(g_log_buffer_mutex);
        } else {
            ESP_LOGE(APP_TAG, "Flash Task: failed to re-lock log buffer after write");
        }

        ESP_LOGI(APP_TAG,
                 "Flash Task: wrote %u records to flash, sectors_written=%" PRIu32 ", next_offset=%" PRIu32,
                 record_count_to_write,
                 g_app_state.flash_sectors_written,
                 g_app_state.flash_write_offset_bytes);

        xSemaphoreGive(g_flash_done_semaphore);
    }
}

static esp_err_t force_flush_log_buffer(void)
{
    if (g_app_state.partition_full) {
        stop_logging_and_notify_full();
        return ESP_ERR_NO_MEM;
    }

    if (g_log_buffer.count == 0U) {
        ESP_LOGI(APP_TAG, "Manager: flush requested, but buffer is empty");
        return ESP_OK;
    }

    g_app_state.flash_flush_in_progress = true;

    xTaskNotifyGive(g_flash_task_handle);

    if (xSemaphoreTake(g_flash_done_semaphore, pdMS_TO_TICKS(3000)) != pdTRUE) {
        g_app_state.flash_flush_in_progress = false;
        ESP_LOGE(APP_TAG, "Manager: flash flush timed out");
        return ESP_ERR_TIMEOUT;
    }

    g_app_state.flash_flush_in_progress = false;

    if (g_app_state.partition_full) {
        stop_logging_and_notify_full();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void handle_uart_command(const uart_command_t *command_in)
{
    if (command_in == NULL) {
        return;
    }

    switch (command_in->type) {
        case UART_CMD_HELP:
            print_help();
            break;

        case UART_CMD_STATUS:
            print_status();
            break;

        case UART_CMD_FLUSH: {
            esp_err_t flush_result = force_flush_log_buffer();
            if (flush_result == ESP_OK) {
                uart_print_line("Flush done.");
            } else {
                uart_print_line("Flush failed.");
            }
            break;
        }

        case UART_CMD_ERASE: {
            esp_err_t flush_result = ESP_OK;

            if (g_log_buffer.count > 0U) {
                flush_result = force_flush_log_buffer();
            }

            if ((flush_result == ESP_OK) || (flush_result == ESP_ERR_NO_MEM)) {
                esp_err_t erase_result = erase_entire_log_partition();
                if (erase_result == ESP_OK) {
                    if (xSemaphoreTake(g_log_buffer_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                        clear_ram_log_buffer();
                        xSemaphoreGive(g_log_buffer_mutex);
                    }

                    g_app_state.flash_write_offset_bytes = 0U;
                    g_app_state.flash_sectors_written = 0U;
                    g_app_state.partition_full = false;

                    uart_print_line("Log partition erased.");
                } else {
                    uart_print_line("Failed to erase log partition.");
                }
            } else {
                uart_print_line("Erase aborted because flush failed.");
            }
            break;
        }

        case UART_CMD_LOG_ON:
            if (!g_app_state.partition_full) {
                g_app_state.logging_enabled = true;
                uart_print_line("Logging enabled.");
            } else {
                uart_print_line("Cannot enable logging: partition is full.");
            }
            break;

        case UART_CMD_LOG_OFF:
            g_app_state.logging_enabled = false;
            uart_print_line("Logging disabled.");
            break;

        case UART_CMD_NONE:
        default:
            break;
    }

    send_display_snapshot();
}

static void manager_task(void *pvParameters)
{
    (void)pvParameters;

    log_record_t incoming_record;
    uart_command_t incoming_command;

    for (;;) {
        // 1) Wake cycle begins: trigger sensor reading
        xTaskNotifyGive(g_sensor_task_handle);

        // 2) Wait for sensor sample
        if (xQueueReceive(g_sensor_queue, &incoming_record, pdMS_TO_TICKS(500)) == pdPASS) {
            g_app_state.latest_record = incoming_record;
            send_display_snapshot();

            g_app_state.elapsed_seconds_since_last_log += (WAKE_INTERVAL_MS / 1000U);

            if (g_app_state.logging_enabled && !g_app_state.partition_full) {
                if (g_app_state.elapsed_seconds_since_last_log >= LOG_INTERVAL_SECONDS) {
                    g_app_state.elapsed_seconds_since_last_log = 0U;

                    if (xSemaphoreTake(g_log_buffer_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                        esp_err_t append_result = append_record_to_ram_buffer(&incoming_record);

                        if (append_result != ESP_OK) {
                            ESP_LOGW(APP_TAG, "Manager: RAM buffer is full, forcing flash flush");
                        }

                        bool should_flush = ram_log_buffer_is_full();
                        xSemaphoreGive(g_log_buffer_mutex);

                        if (should_flush) {
                            esp_err_t flush_result = force_flush_log_buffer();
                            if (flush_result != ESP_OK) {
                                ESP_LOGW(APP_TAG, "Manager: automatic flush failed");
                            }
                        }
                    } else {
                        ESP_LOGW(APP_TAG, "Manager: failed to lock RAM log buffer");
                    }
                }
            }
        } else {
            ESP_LOGW(APP_TAG, "Manager: no sensor data this cycle");
        }

        // 3) Drain command queue
        while (xQueueReceive(g_command_queue, &incoming_command, 0) == pdPASS) {
            handle_uart_command(&incoming_command);
        }

        // 4) Show latest screen state once more after command handling
        send_display_snapshot();

        // 5) Light sleep for 1 second
        esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_MS * 1000ULL);
        esp_light_sleep_start();
    }
}

// =========================
// Init
// =========================

static esp_err_t init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if SOC_UART_SUPPORT_REF_TICK
        .source_clk = UART_SCLK_REF_TICK,
#endif
    };

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT_NUM,
        UART_RX_BUFFER_SIZE,
        UART_TX_BUFFER_SIZE,
        0,
        NULL,
        0));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    return ESP_OK;
}

static esp_err_t init_flash_partition(void)
{
    g_log_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        LOG_PARTITION_LABEL);

    if (g_log_partition == NULL) {
        ESP_LOGE(APP_TAG, "Partition '%s' not found", LOG_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }

    if ((g_log_partition->size % FLASH_SECTOR_SIZE_BYTES) != 0U) {
        ESP_LOGE(APP_TAG, "Partition size must be multiple of sector size");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(APP_TAG,
             "Log partition found: label=%s, address=0x%08" PRIx32 ", size=%" PRIu32 " bytes",
             g_log_partition->label,
             g_log_partition->address,
             g_log_partition->size);

    // Simple teaching choice:
    // erase whole log partition on every boot so logging starts clean.
    return erase_entire_log_partition();
}

static void init_app_state(void)
{
    memset(&g_app_state, 0, sizeof(g_app_state));

    g_app_state.logging_enabled = true;
    g_app_state.partition_full = false;
    g_app_state.flash_flush_in_progress = false;
    g_app_state.elapsed_seconds_since_last_log = 0U;
    g_app_state.flash_write_offset_bytes = 0U;
    g_app_state.flash_sectors_written = 0U;

    clear_ram_log_buffer();
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_uart());

    uart_print_line("Power-aware logger starting...");

    g_sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(log_record_t));
    g_command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(uart_command_t));
    g_display_queue = xQueueCreate(1, sizeof(display_snapshot_t)); // overwrite queue

    g_i2c_mutex = xSemaphoreCreateMutex();
    g_log_buffer_mutex = xSemaphoreCreateMutex();
    g_flash_done_semaphore = xSemaphoreCreateBinary();

    if ((g_sensor_queue == NULL) ||
        (g_command_queue == NULL) ||
        (g_display_queue == NULL) ||
        (g_i2c_mutex == NULL) ||
        (g_log_buffer_mutex == NULL) ||
        (g_flash_done_semaphore == NULL)) {
        ESP_LOGE(APP_TAG, "Failed to create RTOS objects");
        return;
    }

    ESP_ERROR_CHECK(hal_i2c_init());
    ESP_ERROR_CHECK(hal_bme280_init());
    ESP_ERROR_CHECK(hal_ds1306_init());
    ESP_ERROR_CHECK(hal_ssd1306_init());
    ESP_ERROR_CHECK(init_flash_partition());

    init_app_state();

    BaseType_t create_result = pdPASS;

    create_result = xTaskCreate(
        sensor_task,
        "sensor_task",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        &g_sensor_task_handle);
    if (create_result != pdPASS) {
        ESP_LOGE(APP_TAG, "Failed to create sensor_task");
        return;
    }

    create_result = xTaskCreate(
        uart_task,
        "uart_task",
        UART_TASK_STACK_SIZE,
        NULL,
        UART_TASK_PRIORITY,
        NULL);
    if (create_result != pdPASS) {
        ESP_LOGE(APP_TAG, "Failed to create uart_task");
        return;
    }

    create_result = xTaskCreate(
        display_task,
        "display_task",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        NULL);
    if (create_result != pdPASS) {
        ESP_LOGE(APP_TAG, "Failed to create display_task");
        return;
    }

    create_result = xTaskCreate(
        flash_writer_task,
        "flash_writer_task",
        FLASH_TASK_STACK_SIZE,
        NULL,
        FLASH_TASK_PRIORITY,
        &g_flash_task_handle);
    if (create_result != pdPASS) {
        ESP_LOGE(APP_TAG, "Failed to create flash_writer_task");
        return;
    }

    create_result = xTaskCreate(
        manager_task,
        "manager_task",
        MANAGER_TASK_STACK_SIZE,
        NULL,
        MANAGER_TASK_PRIORITY,
        NULL);
    if (create_result != pdPASS) {
        ESP_LOGE(APP_TAG, "Failed to create manager_task");
        return;
    }

    uart_print_line("System ready.");
}
