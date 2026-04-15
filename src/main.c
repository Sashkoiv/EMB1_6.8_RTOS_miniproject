#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define APP_TAG "DATALOGGER"

#define WAKE_INTERVAL_US (1000000ULL)
#define LOG_INTERVAL_SECONDS (60U)

#define SENSOR_QUEUE_LEN (4U)
#define COMMAND_QUEUE_LEN (8U)
#define DISPLAY_QUEUE_LEN (1U)
#define FLASH_QUEUE_LEN (2U)
#define FLASH_RESULT_QUEUE_LEN (4U)

#define FLASH_SECTOR_SIZE_BYTES (4096U)

typedef struct __attribute__((packed)) {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	int16_t temperature_c_x100;
	uint16_t humidity_pct_x100;
	uint16_t pressure_mmhg;
} log_record_t;

#define RECORDS_PER_SECTOR (FLASH_SECTOR_SIZE_BYTES / sizeof(log_record_t))

typedef struct {
	log_record_t records[RECORDS_PER_SECTOR];
	uint16_t count;
} flash_block_t;

typedef struct {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	int16_t temperature_c_x100;
	uint16_t humidity_pct_x100;
	uint16_t pressure_mmhg;
	bool logging_enabled;
	bool partition_full;
} display_snapshot_t;

typedef enum {
	COMMAND_HELP = 0,
	COMMAND_STATUS,
	COMMAND_FLUSH,
	COMMAND_ERASE,
	COMMAND_LOG_ON,
	COMMAND_LOG_OFF
} command_type_t;

typedef struct {
	command_type_t type;
	int32_t argument;
} uart_command_t;

typedef struct {
	bool logging_enabled;
	bool partition_full;
	uint32_t seconds_since_last_log;
	uint32_t write_offset;
} system_status_t;

typedef enum {
	FLASH_WRITE_OK = 0,
	FLASH_PARTITION_FULL,
	FLASH_WRITE_ERROR
} flash_result_type_t;

typedef struct {
	flash_result_type_t type;
	uint32_t bytes_advanced;
} flash_result_t;

static QueueHandle_t g_sensor_queue;
static QueueHandle_t g_command_queue;
static QueueHandle_t g_display_queue;
static QueueHandle_t g_flash_queue;
static QueueHandle_t g_flash_result_queue;

static SemaphoreHandle_t g_i2c_mutex;
static SemaphoreHandle_t g_status_mutex;
static SemaphoreHandle_t g_flash_mutex;

static TaskHandle_t g_sensor_task_handle;

static flash_block_t g_ram_block;
static const esp_partition_t *g_log_partition;
static system_status_t g_status = {
	.logging_enabled = true,
	.partition_full = false,
	.seconds_since_last_log = 0,
	.write_offset = 0,
};

static system_status_t get_status_snapshot(void)
{
	system_status_t snapshot = {0};
	if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
		snapshot = g_status;
		xSemaphoreGive(g_status_mutex);
	}
	return snapshot;
}

static void set_logging_enabled(system_status_t *status, bool enabled)
{
	status->logging_enabled = enabled;
	if (!enabled) {
		status->seconds_since_last_log = 0;
	}
}

static bool append_record_to_ram(const log_record_t *record)
{
	if (g_ram_block.count >= RECORDS_PER_SECTOR) {
		return true;
	}

	g_ram_block.records[g_ram_block.count] = *record;
	g_ram_block.count++;
	return (g_ram_block.count >= RECORDS_PER_SECTOR);
}

static void clear_ram_block(void)
{
	memset(&g_ram_block, 0, sizeof(g_ram_block));
}

static bool queue_block_for_flash(void)
{
	if (g_ram_block.count == 0) {
		return false;
	}

	flash_block_t block = g_ram_block;
	if (xQueueSend(g_flash_queue, &block, pdMS_TO_TICKS(100)) != pdTRUE) {
		ESP_LOGW(APP_TAG, "flash_queue full, dropping flush request");
		return false;
	}

	clear_ram_block();
	return true;
}

static void copy_record_to_display(const log_record_t *record, display_snapshot_t *snapshot)
{
	system_status_t status = get_status_snapshot();

	snapshot->year = record->year;
	snapshot->month = record->month;
	snapshot->day = record->day;
	snapshot->hour = record->hour;
	snapshot->minute = record->minute;
	snapshot->second = record->second;
	snapshot->temperature_c_x100 = record->temperature_c_x100;
	snapshot->humidity_pct_x100 = record->humidity_pct_x100;
	snapshot->pressure_mmhg = record->pressure_mmhg;
	snapshot->logging_enabled = status.logging_enabled;
	snapshot->partition_full = status.partition_full;
}

static void print_status_to_uart(void)
{
	system_status_t status = get_status_snapshot();
	ESP_LOGI(APP_TAG,
			 "status: logging=%s, partition_full=%s, seconds_since_last_log=%lu, write_offset=%lu",
			 status.logging_enabled ? "on" : "off",
			 status.partition_full ? "yes" : "no",
			 (unsigned long)status.seconds_since_last_log,
			 (unsigned long)status.write_offset);
}

static void print_help(void)
{
	ESP_LOGI(APP_TAG, "commands: help | status | flush | erase | log on | log off");
}

static bool parse_uart_command(const char *line, uart_command_t *cmd)
{
	if (strcmp(line, "help") == 0) {
		cmd->type = COMMAND_HELP;
		return true;
	}
	if (strcmp(line, "status") == 0) {
		cmd->type = COMMAND_STATUS;
		return true;
	}
	if (strcmp(line, "flush") == 0) {
		cmd->type = COMMAND_FLUSH;
		return true;
	}
	if (strcmp(line, "erase") == 0) {
		cmd->type = COMMAND_ERASE;
		return true;
	}
	if (strcmp(line, "log on") == 0) {
		cmd->type = COMMAND_LOG_ON;
		return true;
	}
	if (strcmp(line, "log off") == 0) {
		cmd->type = COMMAND_LOG_OFF;
		return true;
	}

	return false;
}

static log_record_t read_sensor_snapshot(void)
{
	log_record_t record = {0};
	const int64_t now_sec = esp_timer_get_time() / 1000000LL;

	record.year = 2026;
	record.month = 1 + (uint8_t)((now_sec / (30LL * 24LL * 3600LL)) % 12LL);
	record.day = 1 + (uint8_t)((now_sec / (24LL * 3600LL)) % 28LL);
	record.hour = (uint8_t)((now_sec / 3600LL) % 24LL);
	record.minute = (uint8_t)((now_sec / 60LL) % 60LL);
	record.second = (uint8_t)(now_sec % 60LL);

	// Placeholder synthetic values until real BME280/RTC drivers are connected.
	record.temperature_c_x100 = (int16_t)(2300 + (now_sec % 50));
	record.humidity_pct_x100 = (uint16_t)(5000 + (now_sec % 120));
	record.pressure_mmhg = (uint16_t)(755 + (now_sec % 5));

	return record;
}

static void sensor_task(void *arg)
{
	(void)arg;

	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
			log_record_t record = read_sensor_snapshot();
			xSemaphoreGive(g_i2c_mutex);

			if (xQueueSend(g_sensor_queue, &record, pdMS_TO_TICKS(100)) != pdTRUE) {
				ESP_LOGW(APP_TAG, "sensor_queue full");
			}
		} else {
			ESP_LOGW(APP_TAG, "i2c mutex timeout in sensor task");
		}
	}
}

static void display_task(void *arg)
{
	(void)arg;

	display_snapshot_t snapshot = {0};
	while (1) {
		if (xQueueReceive(g_display_queue, &snapshot, portMAX_DELAY) == pdTRUE) {
			if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
				// Placeholder OLED output via log until SSD1306 driver is integrated.
				ESP_LOGI(APP_TAG,
						 "OLED %04u-%02u-%02u %02u:%02u:%02u T=%d.%02dC H=%u.%02u%% P=%u mmHg log=%s full=%s",
						 snapshot.year,
						 snapshot.month,
						 snapshot.day,
						 snapshot.hour,
						 snapshot.minute,
						 snapshot.second,
						 snapshot.temperature_c_x100 / 100,
						 abs(snapshot.temperature_c_x100 % 100),
						 snapshot.humidity_pct_x100 / 100,
						 snapshot.humidity_pct_x100 % 100,
						 snapshot.pressure_mmhg,
						 snapshot.logging_enabled ? "on" : "off",
						 snapshot.partition_full ? "yes" : "no");
				xSemaphoreGive(g_i2c_mutex);
			}
		}
	}
}

static void uart_task(void *arg)
{
	(void)arg;

	const uart_config_t uart_cfg = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	(void)uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
	(void)uart_param_config(UART_NUM_0, &uart_cfg);

	char line[64] = {0};
	size_t line_len = 0;

	while (1) {
		uint8_t ch = 0;
		const int read = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(200));
		if (read <= 0) {
			continue;
		}

		if (ch == '\r' || ch == '\n') {
			if (line_len == 0) {
				continue;
			}

			line[line_len] = '\0';
			uart_command_t cmd = {0};
			if (parse_uart_command(line, &cmd)) {
				if (xQueueSend(g_command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
					ESP_LOGW(APP_TAG, "command_queue full");
				}
			} else {
				ESP_LOGW(APP_TAG, "unknown command: %s", line);
			}

			line_len = 0;
			continue;
		}

		if (line_len + 1 < sizeof(line)) {
			line[line_len++] = (char)ch;
		}
	}
}

static void flash_writer_task(void *arg)
{
	(void)arg;

	flash_block_t block = {0};
	while (1) {
		if (xQueueReceive(g_flash_queue, &block, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		flash_result_t result = {
			.type = FLASH_WRITE_ERROR,
			.bytes_advanced = 0,
		};

		if (g_log_partition == NULL) {
			result.type = FLASH_PARTITION_FULL;
			(void)xQueueSend(g_flash_result_queue, &result, 0);
			continue;
		}

		if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
			ESP_LOGW(APP_TAG, "flash mutex timeout");
			(void)xQueueSend(g_flash_result_queue, &result, 0);
			continue;
		}

		uint32_t write_offset = 0;
		if (xSemaphoreTake(g_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
			write_offset = g_status.write_offset;
			xSemaphoreGive(g_status_mutex);
		}

		if (write_offset + FLASH_SECTOR_SIZE_BYTES > g_log_partition->size) {
			result.type = FLASH_PARTITION_FULL;
			xSemaphoreGive(g_flash_mutex);
			(void)xQueueSend(g_flash_result_queue, &result, 0);
			continue;
		}

		uint8_t sector_buf[FLASH_SECTOR_SIZE_BYTES];
		memset(sector_buf, 0xFF, sizeof(sector_buf));

		const size_t data_bytes = (size_t)block.count * sizeof(log_record_t);
		if (data_bytes > sizeof(sector_buf)) {
			result.type = FLASH_WRITE_ERROR;
			xSemaphoreGive(g_flash_mutex);
			(void)xQueueSend(g_flash_result_queue, &result, 0);
			continue;
		}
		memcpy(sector_buf, block.records, data_bytes);

		esp_err_t err = esp_partition_erase_range(g_log_partition, write_offset, FLASH_SECTOR_SIZE_BYTES);
		if (err == ESP_OK) {
			err = esp_partition_write(g_log_partition, write_offset, sector_buf, sizeof(sector_buf));
		}

		if (err == ESP_OK) {
			result.type = FLASH_WRITE_OK;
			result.bytes_advanced = FLASH_SECTOR_SIZE_BYTES;
			if (xSemaphoreTake(g_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
				g_status.write_offset += FLASH_SECTOR_SIZE_BYTES;
				xSemaphoreGive(g_status_mutex);
			}
		} else {
			ESP_LOGE(APP_TAG, "flash write failed at offset=%lu err=0x%x", (unsigned long)write_offset, err);
			result.type = FLASH_WRITE_ERROR;
		}

		xSemaphoreGive(g_flash_mutex);
		(void)xQueueSend(g_flash_result_queue, &result, 0);
	}
}

static void handle_uart_command(const uart_command_t *cmd)
{
	switch (cmd->type) {
	case COMMAND_HELP:
		print_help();
		break;

	case COMMAND_STATUS:
		print_status_to_uart();
		break;

	case COMMAND_FLUSH:
		if (queue_block_for_flash()) {
			ESP_LOGI(APP_TAG, "flush queued");
		}
		break;

	case COMMAND_ERASE:
		if (g_log_partition == NULL) {
			ESP_LOGW(APP_TAG, "cannot erase: log partition not found");
			break;
		}
		if (uxQueueMessagesWaiting(g_flash_queue) > 0) {
			ESP_LOGW(APP_TAG, "cannot erase while flash queue has pending writes");
			break;
		}
		if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
			esp_err_t err = esp_partition_erase_range(g_log_partition, 0, g_log_partition->size);
			xSemaphoreGive(g_flash_mutex);

			if (err == ESP_OK) {
				clear_ram_block();
				if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
					g_status.write_offset = 0;
					g_status.partition_full = false;
					xSemaphoreGive(g_status_mutex);
				}
				ESP_LOGI(APP_TAG, "log partition erased");
			} else {
				ESP_LOGE(APP_TAG, "erase failed err=0x%x", err);
			}
		}
		break;

	case COMMAND_LOG_ON:
		if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
			set_logging_enabled(&g_status, true);
			xSemaphoreGive(g_status_mutex);
		}
		ESP_LOGI(APP_TAG, "logging enabled");
		break;

	case COMMAND_LOG_OFF:
		if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
			set_logging_enabled(&g_status, false);
			xSemaphoreGive(g_status_mutex);
		}
		ESP_LOGI(APP_TAG, "logging disabled");
		break;

	default:
		break;
	}
}

static void manager_task(void *arg)
{
	(void)arg;

	while (1) {
		xTaskNotifyGive(g_sensor_task_handle);

		log_record_t record = {0};
		const BaseType_t got_record = xQueueReceive(g_sensor_queue, &record, pdMS_TO_TICKS(1200));
		if (got_record == pdTRUE) {
			display_snapshot_t snapshot = {0};
			copy_record_to_display(&record, &snapshot);
			(void)xQueueOverwrite(g_display_queue, &snapshot);
		}

		uart_command_t cmd;
		while (xQueueReceive(g_command_queue, &cmd, 0) == pdTRUE) {
			handle_uart_command(&cmd);
		}

		flash_result_t flash_result;
		while (xQueueReceive(g_flash_result_queue, &flash_result, 0) == pdTRUE) {
			if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
				if (flash_result.type == FLASH_PARTITION_FULL) {
					g_status.partition_full = true;
					g_status.logging_enabled = false;
					ESP_LOGW(APP_TAG, "logging stopped: flash full");
				}
				xSemaphoreGive(g_status_mutex);
			}
		}

		if (got_record == pdTRUE) {
			if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
				if (g_status.logging_enabled && !g_status.partition_full) {
					g_status.seconds_since_last_log++;

					if (g_status.seconds_since_last_log >= LOG_INTERVAL_SECONDS) {
						g_status.seconds_since_last_log = 0;
						const bool buffer_full = append_record_to_ram(&record);
						if (buffer_full) {
							(void)queue_block_for_flash();
						}
					}
				}
				xSemaphoreGive(g_status_mutex);
			}
		}

		esp_sleep_enable_timer_wakeup(WAKE_INTERVAL_US);
		esp_light_sleep_start();
	}
}

void app_main(void)
{
	g_sensor_queue = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(log_record_t));
	g_command_queue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(uart_command_t));
	g_display_queue = xQueueCreate(DISPLAY_QUEUE_LEN, sizeof(display_snapshot_t));
	g_flash_queue = xQueueCreate(FLASH_QUEUE_LEN, sizeof(flash_block_t));
	g_flash_result_queue = xQueueCreate(FLASH_RESULT_QUEUE_LEN, sizeof(flash_result_t));

	g_i2c_mutex = xSemaphoreCreateMutex();
	g_status_mutex = xSemaphoreCreateMutex();
	g_flash_mutex = xSemaphoreCreateMutex();

	clear_ram_block();

	g_log_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "logdata");
	if (g_log_partition == NULL) {
		ESP_LOGW(APP_TAG, "partition 'logdata' not found, logging to flash disabled");
		if (xSemaphoreTake(g_status_mutex, portMAX_DELAY) == pdTRUE) {
			g_status.partition_full = true;
			g_status.logging_enabled = false;
			xSemaphoreGive(g_status_mutex);
		}
	}

	print_help();

	xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &g_sensor_task_handle);
	xTaskCreate(manager_task, "manager_task", 6144, NULL, 6, NULL);
	xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);
	xTaskCreate(uart_task, "uart_task", 4096, NULL, 4, NULL);
	xTaskCreate(flash_writer_task, "flash_writer_task", 6144, NULL, 5, NULL);
}