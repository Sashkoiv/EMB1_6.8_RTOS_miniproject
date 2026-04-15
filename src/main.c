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

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_partition.h"

#define LOG_TAG                 "DATALOGGER"

// =========================
// Configuration
// =========================

#define PIN_I2C_SCL             4
#define PIN_I2C_SDA             5
#define I2C_PORT                I2C_NUM_0
#define I2C_CLOCK_HZ            100000U

#define DS1307_I2C_ADDR         0x68U

#define UART_PORT               UART_NUM_0
#define UART_BAUD               115200U
#define UART_RX_BUF_SIZE        256U

#define WAKE_PERIOD_MS          1000U
#define LOG_INTERVAL_MINUTES    1U
#define LOG_INTERVAL_SECONDS    (LOG_INTERVAL_MINUTES * 60U)

#define SECTOR_BYTES            4096U
#define LOG_PARTITION_LABEL     "logdata"

#define SENSOR_Q_LEN            8U
#define CMD_Q_LEN               8U
#define DISPLAY_Q_LEN           1U

#define STACK_SENSOR            4096U
#define STACK_UART              4096U
#define STACK_DISPLAY           4096U
#define STACK_MANAGER           6144U
#define STACK_FLASH             4096U

#define PRIO_SENSOR             5U
#define PRIO_UART               4U
#define PRIO_DISPLAY            3U
#define PRIO_MANAGER            6U
#define PRIO_FLASH              4U

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
    uint8_t  pad[3];
} log_record_t;

_Static_assert(sizeof(log_record_t) == 16, "log_record_t must be 16 bytes");

#define RECORDS_PER_SECTOR      (SECTOR_BYTES / sizeof(log_record_t))

typedef struct {
    log_record_t records[RECORDS_PER_SECTOR];
    uint16_t     count;
} sector_ram_buf_t;

typedef struct {
    log_record_t last_record;
    bool         logging_on;
    bool         flash_full;
    uint16_t     buffered_count;
    uint32_t     sectors_written;
} display_snapshot_t;

typedef enum {
    CMD_NONE = 0,
    CMD_HELP,
    CMD_STATUS,
    CMD_FLUSH,
    CMD_ERASE,
    CMD_LOG_ON,
    CMD_LOG_OFF
} cmd_type_t;

typedef struct {
    cmd_type_t type;
} parsed_cmd_t;

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} rtc_time_t;

typedef struct {
    int32_t  temp_c_x100;
    uint32_t humidity_x100;
    uint32_t pressure_mmhg;
} env_sample_t;

typedef struct {
    bool     logging_on;
    bool     flash_full;
    bool     flush_busy;
    uint32_t secs_since_last_log;
    uint32_t flash_offset_bytes;
    uint32_t sectors_written;
    log_record_t last_record;
} logger_state_t;

// =========================
// Globals
// =========================

static QueueHandle_t    g_sensor_q      = NULL;
static QueueHandle_t    g_cmd_q         = NULL;
static QueueHandle_t    g_display_q     = NULL;

static SemaphoreHandle_t g_i2c_lock     = NULL;
static SemaphoreHandle_t g_buf_lock     = NULL;
static SemaphoreHandle_t g_flash_done   = NULL;

static TaskHandle_t     g_sensor_task   = NULL;
static TaskHandle_t     g_flash_task    = NULL;

static const esp_partition_t *g_partition = NULL;

static sector_ram_buf_t  g_ram_buf;
static logger_state_t    g_state;
static uint8_t           g_uart_line[UART_RX_BUF_SIZE];

// =========================
// HAL — I2C / RTC / BME280 / OLED
// Replace TODO stubs with real drivers when hardware is ready
// =========================

static esp_err_t hal_i2c_init(void)
{
    const i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLOCK_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err == ESP_OK) {
        err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    }

    return err;
}

static esp_err_t hal_bme280_init(void)
{
    // TODO: initialize BME280 over I2C
    return ESP_OK;
}

static esp_err_t hal_ssd1306_init(void)
{
    // TODO: initialize SSD1306 over I2C
    return ESP_OK;
}

static uint8_t bcd_to_decimal(uint8_t val)
{
    return (uint8_t)(((val >> 4U) * 10U) + (val & 0x0FU));
}

static esp_err_t hal_ds1307_read(rtc_time_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t reg = 0x00U;
    uint8_t data[7]   = {0};

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, DS1307_I2C_ADDR,
        &reg, 1,
        data, sizeof(data),
        pdMS_TO_TICKS(1000));

    if (err != ESP_OK) {
        return err;
    }

    // data[0]=sec, data[1]=min, data[2]=hour, data[3]=dow, data[4]=day, data[5]=month, data[6]=year
    out->second = bcd_to_decimal(data[0] & 0x7FU);
    out->minute = bcd_to_decimal(data[1] & 0x7FU);
    out->hour   = bcd_to_decimal(data[2] & 0x3FU);
    out->day    = bcd_to_decimal(data[4] & 0x3FU);
    out->month  = bcd_to_decimal(data[5] & 0x1FU);
    out->year   = (uint16_t)(2000U + bcd_to_decimal(data[6]));

    return ESP_OK;
}

static esp_err_t hal_bme280_read(env_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // TODO: replace with real BME280 reading over I2C
    static int32_t  fake_temp     = 2300;
    static uint32_t fake_humidity = 5000;
    static uint32_t fake_pressure = 760;

    fake_temp++;
    if (fake_temp > 2500) { fake_temp = 2300; }
    fake_humidity += 3U;
    if (fake_humidity > 6500U) { fake_humidity = 5000U; }
    fake_pressure++;
    if (fake_pressure > 765U) { fake_pressure = 760U; }

    out->temp_c_x100   = fake_temp;
    out->humidity_x100 = fake_humidity;
    out->pressure_mmhg = fake_pressure;

    return ESP_OK;
}

static esp_err_t hal_ssd1306_render(const display_snapshot_t *snap)
{
    if (snap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // TODO: replace with real SSD1306 rendering
    // Logs to UART so the system is fully testable without the display wired up.
    printf("OLED | %04u-%02u-%02u %02u:%02u:%02u"
           " | T=%d.%02dC H=%u.%02u%% P=%u mmHg"
           " | log=%s full=%s buf=%u sec=%" PRIu32 "\r\n",
           snap->last_record.year,
           snap->last_record.month,
           snap->last_record.day,
           snap->last_record.hour,
           snap->last_record.minute,
           snap->last_record.second,
           snap->last_record.temperature_c_x100 / 100,
           (unsigned int)abs(snap->last_record.temperature_c_x100 % 100),
           snap->last_record.humidity_pct_x100 / 100U,
           snap->last_record.humidity_pct_x100 % 100U,
           snap->last_record.pressure_mmhg,
           snap->logging_on ? "ON" : "OFF",
           snap->flash_full ? "YES" : "NO",
           snap->buffered_count,
           snap->sectors_written);

    return ESP_OK;
}

// =========================
// Utilities
// =========================

static void print_line(const char *text)
{
    if (text != NULL) {
        printf("%s\r\n", text);
    }
}

static void print_help(void)
{
    print_line("Commands:");
    print_line("  help     - show this list");
    print_line("  status   - show system state");
    print_line("  flush    - write RAM buffer to flash now");
    print_line("  erase    - erase flash log partition");
    print_line("  log on   - enable logging");
    print_line("  log off  - disable logging");
}

static void print_status(void)
{
    printf("STATUS: log=%s full=%s busy=%s buf=%u/%u sec=%" PRIu32 " offset=%" PRIu32 "\r\n",
           g_state.logging_on ? "ON" : "OFF",
           g_state.flash_full ? "YES" : "NO",
           g_state.flush_busy ? "YES" : "NO",
           g_ram_buf.count,
           (unsigned int)RECORDS_PER_SECTOR,
           g_state.sectors_written,
           g_state.flash_offset_bytes);

    printf("LATEST: %04u-%02u-%02u %02u:%02u:%02u"
           " T=%d.%02dC H=%u.%02u%% P=%u mmHg\r\n",
           g_state.last_record.year,
           g_state.last_record.month,
           g_state.last_record.day,
           g_state.last_record.hour,
           g_state.last_record.minute,
           g_state.last_record.second,
           g_state.last_record.temperature_c_x100 / 100,
           (unsigned int)abs(g_state.last_record.temperature_c_x100 % 100),
           g_state.last_record.humidity_pct_x100 / 100U,
           g_state.last_record.humidity_pct_x100 % 100U,
           g_state.last_record.pressure_mmhg);
}

static display_snapshot_t make_snapshot(void)
{
    display_snapshot_t snap;
    memset(&snap, 0, sizeof(snap));

    snap.last_record      = g_state.last_record;
    snap.logging_on       = g_state.logging_on;
    snap.flash_full       = g_state.flash_full;
    snap.buffered_count   = g_ram_buf.count;
    snap.sectors_written  = g_state.sectors_written;

    return snap;
}

static void push_display_snapshot(void)
{
    display_snapshot_t snap = make_snapshot();
    xQueueOverwrite(g_display_q, &snap);
}

static void clear_ram_buf(void)
{
    memset(g_ram_buf.records, 0xFF, sizeof(g_ram_buf.records));
    g_ram_buf.count = 0U;
}

static bool partition_has_room(void)
{
    if (g_partition == NULL) {
        return false;
    }

    return (g_state.flash_offset_bytes + SECTOR_BYTES) <= g_partition->size;
}

static void handle_flash_full(void)
{
    g_state.logging_on = false;
    g_state.flash_full = true;
    print_line("WARNING: flash partition full. Logging stopped.");
    push_display_snapshot();
}

static esp_err_t erase_partition(void)
{
    if (g_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(LOG_TAG, "Erasing log partition...");
    return esp_partition_erase_range(g_partition, 0, g_partition->size);
}

static esp_err_t append_to_ram_buf(const log_record_t *rec)
{
    if (rec == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ram_buf.count >= RECORDS_PER_SECTOR) {
        return ESP_ERR_NO_MEM;
    }

    g_ram_buf.records[g_ram_buf.count] = *rec;
    g_ram_buf.count++;

    return ESP_OK;
}

static bool ram_buf_full(void)
{
    return (g_ram_buf.count >= RECORDS_PER_SECTOR);
}

static esp_err_t parse_command(const char *line, parsed_cmd_t *out)
{
    if ((line == NULL) || (out == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    out->type = CMD_NONE;

    if      (strcmp(line, "help")    == 0) { out->type = CMD_HELP;   }
    else if (strcmp(line, "status")  == 0) { out->type = CMD_STATUS; }
    else if (strcmp(line, "flush")   == 0) { out->type = CMD_FLUSH;  }
    else if (strcmp(line, "erase")   == 0) { out->type = CMD_ERASE;  }
    else if (strcmp(line, "log on")  == 0) { out->type = CMD_LOG_ON; }
    else if (strcmp(line, "log off") == 0) { out->type = CMD_LOG_OFF;}
    else {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

static esp_err_t build_record(log_record_t *out,
                               const rtc_time_t *t,
                               const env_sample_t *e)
{
    if ((out == NULL) || (t == NULL) || (e == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));

    out->year                = t->year;
    out->month               = t->month;
    out->day                 = t->day;
    out->hour                = t->hour;
    out->minute              = t->minute;
    out->second              = t->second;
    out->temperature_c_x100  = (int16_t)e->temp_c_x100;
    out->humidity_pct_x100   = (uint16_t)e->humidity_x100;
    out->pressure_mmhg       = (uint16_t)e->pressure_mmhg;

    return ESP_OK;
}

// =========================
// Tasks
// =========================

static void sensor_task(void *pvParameters)
{
    (void)pvParameters;

    rtc_time_t   rtc_val;
    env_sample_t env_val;
    log_record_t rec;

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0U) {
            continue;
        }

        if (xSemaphoreTake(g_i2c_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
            ESP_LOGW(LOG_TAG, "sensor_task: I2C lock timeout");
            continue;
        }

        esp_err_t err = hal_ds1307_read(&rtc_val);
        if (err == ESP_OK) {
            err = hal_bme280_read(&env_val);
        }

        xSemaphoreGive(g_i2c_lock);

        if (err != ESP_OK) {
            ESP_LOGE(LOG_TAG, "sensor_task: read error %s", esp_err_to_name(err));
            continue;
        }

        if (build_record(&rec, &rtc_val, &env_val) == ESP_OK) {
            if (xQueueSend(g_sensor_q, &rec, pdMS_TO_TICKS(100)) != pdPASS) {
                ESP_LOGW(LOG_TAG, "sensor_task: sensor queue full");
            }
        }
    }
}

static void uart_task(void *pvParameters)
{
    (void)pvParameters;

    int  line_pos = 0;
    memset(g_uart_line, 0, sizeof(g_uart_line));

    print_help();

    for (;;) {
        uint8_t ch       = 0;
        int     n_read   = uart_read_bytes(UART_PORT, &ch, 1, pdMS_TO_TICKS(100));

        if (n_read <= 0) {
            continue;
        }

        if ((ch == '\r') || (ch == '\n')) {
            if (line_pos == 0) {
                continue;
            }

            g_uart_line[line_pos] = '\0';

            parsed_cmd_t cmd;
            if (parse_command((const char *)g_uart_line, &cmd) == ESP_OK) {
                if (xQueueSend(g_cmd_q, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
                    ESP_LOGW(LOG_TAG, "uart_task: command queue full");
                }
            } else {
                print_line("Unknown command. Type: help");
            }

            line_pos = 0;
            memset(g_uart_line, 0, sizeof(g_uart_line));
            continue;
        }

        if ((line_pos < (int)(UART_RX_BUF_SIZE - 1U)) &&
            (ch >= 32U) && (ch <= 126U)) {
            g_uart_line[line_pos] = ch;
            line_pos++;
        }
    }
}

static void display_task(void *pvParameters)
{
    (void)pvParameters;

    display_snapshot_t snap;

    for (;;) {
        if (xQueueReceive(g_display_q, &snap, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (xSemaphoreTake(g_i2c_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
            ESP_LOGW(LOG_TAG, "display_task: I2C lock timeout");
            continue;
        }

        esp_err_t err = hal_ssd1306_render(&snap);

        xSemaphoreGive(g_i2c_lock);

        if (err != ESP_OK) {
            ESP_LOGE(LOG_TAG, "display_task: render error %s", esp_err_to_name(err));
        }
    }
}

static void flash_writer_task(void *pvParameters)
{
    (void)pvParameters;

    static log_record_t local_copy[RECORDS_PER_SECTOR];

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0U) {
            continue;
        }

        if (xSemaphoreTake(g_buf_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(LOG_TAG, "flash_task: buffer lock timeout");
            xSemaphoreGive(g_flash_done);
            continue;
        }

        const uint16_t n_records = g_ram_buf.count;
        memcpy(local_copy, g_ram_buf.records, sizeof(local_copy));

        xSemaphoreGive(g_buf_lock);

        if (n_records == 0U) {
            ESP_LOGW(LOG_TAG, "flash_task: flush called with empty buffer");
            xSemaphoreGive(g_flash_done);
            continue;
        }

        if (!partition_has_room()) {
            g_state.flash_full = true;
            xSemaphoreGive(g_flash_done);
            continue;
        }

        esp_err_t err = esp_partition_erase_range(
            g_partition, g_state.flash_offset_bytes, SECTOR_BYTES);

        if (err != ESP_OK) {
            ESP_LOGE(LOG_TAG, "flash_task: erase failed %s", esp_err_to_name(err));
            xSemaphoreGive(g_flash_done);
            continue;
        }

        const size_t write_bytes = (size_t)n_records * sizeof(log_record_t);
        err = esp_partition_write(
            g_partition, g_state.flash_offset_bytes, local_copy, write_bytes);

        if (err != ESP_OK) {
            ESP_LOGE(LOG_TAG, "flash_task: write failed %s", esp_err_to_name(err));
            xSemaphoreGive(g_flash_done);
            continue;
        }

        g_state.flash_offset_bytes += SECTOR_BYTES;
        g_state.sectors_written++;

        if (xSemaphoreTake(g_buf_lock, pdMS_TO_TICKS(1000)) == pdTRUE) {
            clear_ram_buf();
            xSemaphoreGive(g_buf_lock);
        } else {
            ESP_LOGE(LOG_TAG, "flash_task: failed to re-lock buffer after write");
        }

        ESP_LOGI(LOG_TAG,
                 "flash_task: wrote %u records, sectors=%" PRIu32 " next_offset=%" PRIu32,
                 n_records, g_state.sectors_written, g_state.flash_offset_bytes);

        xSemaphoreGive(g_flash_done);
    }
}

// =========================
// Manager helpers
// =========================

static esp_err_t do_flush(void)
{
    if (g_state.flash_full) {
        handle_flash_full();
        return ESP_ERR_NO_MEM;
    }

    if (g_ram_buf.count == 0U) {
        ESP_LOGI(LOG_TAG, "manager: flush requested, buffer empty");
        return ESP_OK;
    }

    g_state.flush_busy = true;
    xTaskNotifyGive(g_flash_task);

    if (xSemaphoreTake(g_flash_done, pdMS_TO_TICKS(3000)) != pdTRUE) {
        g_state.flush_busy = false;
        ESP_LOGE(LOG_TAG, "manager: flash flush timed out");
        return ESP_ERR_TIMEOUT;
    }

    g_state.flush_busy = false;

    if (g_state.flash_full) {
        handle_flash_full();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void handle_command(const parsed_cmd_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case CMD_HELP:
            print_help();
            break;

        case CMD_STATUS:
            print_status();
            break;

        case CMD_FLUSH: {
            esp_err_t r = do_flush();
            print_line(r == ESP_OK ? "Flush done." : "Flush failed.");
            break;
        }

        case CMD_ERASE: {
            if (g_ram_buf.count > 0U) {
                (void)do_flush();
            }

            esp_err_t r = erase_partition();
            if (r == ESP_OK) {
                if (xSemaphoreTake(g_buf_lock, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    clear_ram_buf();
                    xSemaphoreGive(g_buf_lock);
                }

                g_state.flash_offset_bytes = 0U;
                g_state.sectors_written    = 0U;
                g_state.flash_full         = false;
                print_line("Log partition erased.");
            } else {
                print_line("Erase failed.");
            }
            break;
        }

        case CMD_LOG_ON:
            if (!g_state.flash_full) {
                g_state.logging_on = true;
                print_line("Logging enabled.");
            } else {
                print_line("Cannot enable: partition full.");
            }
            break;

        case CMD_LOG_OFF:
            g_state.logging_on = false;
            g_state.secs_since_last_log = 0U;
            print_line("Logging disabled.");
            break;

        case CMD_NONE:
        default:
            break;
    }

    push_display_snapshot();
}

static void manager_task(void *pvParameters)
{
    (void)pvParameters;

    log_record_t  incoming;
    parsed_cmd_t  cmd;

    for (;;) {
        // 1. Trigger sensor reading
        xTaskNotifyGive(g_sensor_task);

        // 2. Wait for sensor data
        if (xQueueReceive(g_sensor_q, &incoming, pdMS_TO_TICKS(500)) == pdPASS) {
            g_state.last_record = incoming;
            push_display_snapshot();

            g_state.secs_since_last_log += (WAKE_PERIOD_MS / 1000U);

            if (g_state.logging_on && !g_state.flash_full) {
                if (g_state.secs_since_last_log >= LOG_INTERVAL_SECONDS) {
                    g_state.secs_since_last_log = 0U;

                    if (xSemaphoreTake(g_buf_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
                        esp_err_t append_err = append_to_ram_buf(&incoming);

                        if (append_err != ESP_OK) {
                            ESP_LOGW(LOG_TAG, "manager: buffer full before scheduled flush");
                        }

                        bool need_flush = ram_buf_full();
                        xSemaphoreGive(g_buf_lock);

                        if (need_flush) {
                            (void)do_flush();
                        }
                    } else {
                        ESP_LOGW(LOG_TAG, "manager: buffer lock timeout");
                    }
                }
            }
        } else {
            ESP_LOGW(LOG_TAG, "manager: no sensor data this cycle");
        }

        // 3. Drain command queue
        while (xQueueReceive(g_cmd_q, &cmd, 0) == pdPASS) {
            handle_command(&cmd);
        }

        // 4. Refresh display with latest state
        push_display_snapshot();

        // 5. Light sleep for the rest of the wake period
        esp_sleep_enable_timer_wakeup((uint64_t)WAKE_PERIOD_MS * 1000ULL);
        esp_light_sleep_start();
    }
}

// =========================
// Init
// =========================

static esp_err_t init_uart_driver(void)
{
    const uart_config_t cfg = {
        .baud_rate  = (int)UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));

    return ESP_OK;
}

static esp_err_t init_log_partition(void)
{
    g_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        LOG_PARTITION_LABEL);

    if (g_partition == NULL) {
        ESP_LOGE(LOG_TAG, "Partition '%s' not found; flash logging disabled", LOG_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }

    if ((g_partition->size % SECTOR_BYTES) != 0U) {
        ESP_LOGE(LOG_TAG, "Partition size not a multiple of sector size");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(LOG_TAG,
             "Log partition: label=%s addr=0x%08" PRIx32 " size=%" PRIu32 " bytes",
             g_partition->label,
             g_partition->address,
             g_partition->size);

    return erase_partition();
}

static void init_state(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.logging_on       = true;
    g_state.flash_full       = false;
    g_state.flush_busy       = false;
    g_state.secs_since_last_log = 0U;
    g_state.flash_offset_bytes  = 0U;
    g_state.sectors_written     = 0U;
    clear_ram_buf();
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_uart_driver());
    print_line("Power-aware logger starting...");

    g_sensor_q  = xQueueCreate(SENSOR_Q_LEN, sizeof(log_record_t));
    g_cmd_q     = xQueueCreate(CMD_Q_LEN,    sizeof(parsed_cmd_t));
    g_display_q = xQueueCreate(DISPLAY_Q_LEN, sizeof(display_snapshot_t));

    g_i2c_lock   = xSemaphoreCreateMutex();
    g_buf_lock   = xSemaphoreCreateMutex();
    g_flash_done = xSemaphoreCreateBinary();

    if ((g_sensor_q  == NULL) || (g_cmd_q    == NULL) ||
        (g_display_q == NULL) || (g_i2c_lock == NULL) ||
        (g_buf_lock  == NULL) || (g_flash_done == NULL)) {
        ESP_LOGE(LOG_TAG, "app_main: failed to create RTOS objects");
        return;
    }

    ESP_ERROR_CHECK(hal_i2c_init());
    ESP_ERROR_CHECK(hal_bme280_init());
    ESP_ERROR_CHECK(hal_ssd1306_init());

    // Flash partition is optional: if not found, logging auto-disables later
    (void)init_log_partition();

    init_state();

    BaseType_t rc;

    rc = xTaskCreate(sensor_task, "sensor_task",
                     STACK_SENSOR, NULL, PRIO_SENSOR, &g_sensor_task);
    if (rc != pdPASS) { ESP_LOGE(LOG_TAG, "Failed to create sensor_task");  return; }

    rc = xTaskCreate(uart_task, "uart_task",
                     STACK_UART, NULL, PRIO_UART, NULL);
    if (rc != pdPASS) { ESP_LOGE(LOG_TAG, "Failed to create uart_task");    return; }

    rc = xTaskCreate(display_task, "display_task",
                     STACK_DISPLAY, NULL, PRIO_DISPLAY, NULL);
    if (rc != pdPASS) { ESP_LOGE(LOG_TAG, "Failed to create display_task"); return; }

    rc = xTaskCreate(flash_writer_task, "flash_task",
                     STACK_FLASH, NULL, PRIO_FLASH, &g_flash_task);
    if (rc != pdPASS) { ESP_LOGE(LOG_TAG, "Failed to create flash_task");   return; }

    rc = xTaskCreate(manager_task, "manager_task",
                     STACK_MANAGER, NULL, PRIO_MANAGER, NULL);
    if (rc != pdPASS) { ESP_LOGE(LOG_TAG, "Failed to create manager_task"); return; }

    print_line("System ready.");
}