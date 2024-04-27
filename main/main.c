/*
 * Boring Wall Clock Firmware
 *
 * Copyright (C) 2023 Calvin Owens <jcalvinowens@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/rtc_io.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_http_server.h"

extern const char index_html_start[] asm ("_binary_index_html_start");
extern const char index_html_end[] asm ("_binary_index_html_end");
static const int led_pwm_duty_max = 1024;

enum clock_modes {
	CLOCK_MODE_TIME,
	CLOCK_MODE_SECONDS,
	CLOCK_MODE_MILLIS,
	CLOCK_MODE_MONTHDAY,
	CLOCK_MODE_DAYMONTH,
	CLOCK_MODE_DAYOFYEAR,
	CLOCK_MODE_DAYOFWEEK,
	CLOCK_MODE_YEAR,
	CLOCK_MODE_STATIC,
};

static int clock_mode = CLOCK_MODE_TIME;

static inline void delay_ms(int64_t ms)
{
	if (ms <= 0)
		return;

	vTaskDelay(ms / portTICK_PERIOD_MS);
}

/*
 * Wifi connect code, largely derived from the CC0 ESP-IDF example code.
 */

static EventGroupHandle_t wifi_eg;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT	   BIT1

static void event_handler(void *arg, esp_event_base_t event_base,
			  int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT) {
		switch (event_id) {
		case WIFI_EVENT_STA_START:
			esp_wifi_connect();
			return;

		case WIFI_EVENT_STA_DISCONNECTED:
			xEventGroupSetBits(wifi_eg, WIFI_FAIL_BIT);
			return;

		default:
			return;
		}
	}

	if (event_base == IP_EVENT) {
		switch (event_id) {
		case IP_EVENT_STA_GOT_IP:
			xEventGroupSetBits(wifi_eg, WIFI_CONNECTED_BIT);
			return;

		case IP_EVENT_STA_LOST_IP:
			xEventGroupSetBits(wifi_eg, WIFI_FAIL_BIT);
			return;

		default:
			return;
		}
	}
}

static esp_err_t wifi_connect(void)
{
	EventBits_t bits = xEventGroupGetBits(wifi_eg);

	if (bits & WIFI_CONNECTED_BIT)
		return ESP_OK;

	if (bits & WIFI_FAIL_BIT) {
		xEventGroupClearBits(wifi_eg, WIFI_FAIL_BIT);
		esp_wifi_connect();
	}

	bits = xEventGroupWaitBits(wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
				   pdFALSE, pdFALSE, portMAX_DELAY);

	if (bits & WIFI_CONNECTED_BIT)
		return ESP_OK;

	if (bits & WIFI_FAIL_BIT)
		return ESP_FAIL;

	return ESP_ERR_INVALID_STATE;
}

static void wifi_init(void)
{
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASSWORD,
			.threshold.authmode = WIFI_AUTH_WPA3_PSK,
			.sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
		},
	};
	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	wifi_eg = xEventGroupCreate();
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
							    ESP_EVENT_ANY_ID,
							    &event_handler, NULL,
							    &instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
							    IP_EVENT_STA_GOT_IP,
							    &event_handler, NULL,
							    &instance_got_ip));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

static void start_ntp(void)
{
	esp_sntp_setservername(0, CONFIG_NTP_SERVER_HOSTNAME);
	sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
	esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_set_sync_interval(CONFIG_NTP_SYNC_INTERVAL_MS);
	esp_sntp_init();
}

static esp_err_t start_wifi(void)
{
	int attempts = 0;
	wifi_init();

	while (1) {
		if (wifi_connect() == ESP_OK)
			break;

		delay_ms(1000 + esp_random() % 1000);
		if (++attempts == 5)
			return ESP_FAIL;
	}

	start_ntp();
	return ESP_OK;
}

static void stop_wifi(void)
{
	ESP_ERROR_CHECK(esp_wifi_disconnect());
	ESP_ERROR_CHECK(esp_wifi_stop());
	ESP_ERROR_CHECK(esp_wifi_deinit());
}

/*
 * The led_digit structure abstracts the physical layout of the shift registers
 * from the font definition. If the order of LED segments is changed on the
 * shift register outputs, change the order of the fields here to match.
 */

struct led_digit {
	uint8_t a : 1;
	uint8_t g : 1;
	uint8_t f : 1;
	uint8_t h : 1;
	uint8_t e : 1;
	uint8_t d : 1;
	uint8_t b : 1;
	uint8_t c : 1;
};

/*
 * LED Segment Layout (as viewed looking at the clock face):
 *
 *  (0)       (1)       (2)       (3)      <=== Index into ledstate.digits
 *
 * |-E-|     |-E-|     |-E-|     |-E-|     <===\
 * F   D     F   D     F   D     F   D     <===\\
 * |-G-|     |-G-|     |-G-|     |-G-|     <=== Segment ID
 * A   C     A   C     A   C     A   C     <===//
 * |-B-| (H) |-B-| (H) |-B-| (H) |-B-| (H) <===/
 */

static const struct led_digit led_font[21] = {
// Numbers: 0123456789
(struct led_digit){ .a = 1, .b = 1, .c = 1, .d = 1, .e = 1, .f = 1, },
(struct led_digit){ .c = 1, .d = 1, },
(struct led_digit){ .a = 1, .b = 1, .d = 1, .e = 1, .g = 1, },
(struct led_digit){ .b = 1, .c = 1, .d = 1, .e = 1, .g = 1, },
(struct led_digit){ .c = 1, .d = 1, .f = 1, .g = 1, },
(struct led_digit){ .b = 1, .c = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .a = 1, .b = 1, .c = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .c = 1, .d = 1, .e = 1, },
(struct led_digit){ .a = 1, .b = 1, .c = 1, .d = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .c = 1, .d = 1, .e = 1, .f = 1, .g = 1, },
// Blank
(struct led_digit){ 0 },
// Letters: ACEFHLPU
(struct led_digit){ .a = 1, .c = 1, .d = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .a = 1, .b = 1, .e = 1, .f = 1, },
(struct led_digit){ .a = 1, .b = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .a = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .a = 1, .c = 1, .d = 1, .f = 1, .g = 1, },
(struct led_digit){ .a = 1, .b = 1, .f = 1, },
(struct led_digit){ .a = 1, .d = 1, .e = 1, .f = 1, .g = 1, },
(struct led_digit){ .a = 1, .b = 1, .c = 1, .d = 1, .f = 1, },
// Hyphen
(struct led_digit){ .g = 1, },
// Degree symbol
(struct led_digit){ .d = 1, .e = 1, .f = 1, .g = 1, },
};

/*
 * The individual LED segments are controlled by tiny MOSFETs driven by four
 * chained 8-bit shift registers. Each segment is individually switched, to
 * avoid rolling shutter artifacts in photos and videos.
 *
 * The spi_task programs the shift registers with the current contents of the
 * ledstate structure when kicked. The update_timer_work keeps the ledstate
 * synced with the current time, and kicks the spi_task when it changes.
 */

static const int gpio_rear_led = 19;
static const int gpio_regs_clk = 18;
static const int gpio_regs_lat = 13;
static const int gpio_regs_pwm = 12;
static const int gpio_regs_ser = 10;

struct ledstate {
	struct led_digit digits[4];
};

static struct ledstate ledbuffer;
static struct ledstate user_static;

static StackType_t spi_task_stack[1024];
static StaticTask_t spi_task_buffer;

static void spi_tx_done(spi_transaction_t *t)
{
	return;
}

static void spi_write_new_ledstate(spi_device_handle_t spi)
{
	spi_transaction_t tx = {
		.tx_buffer = &ledbuffer,
		.length = sizeof(ledbuffer) * 8,
		.rxlength = 0,
	};

	spi_device_acquire_bus(spi, portMAX_DELAY);
	ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &tx));
	spi_device_release_bus(spi);
}

/*
 * FIXME: This spinlock is pointless, but it seems to be required to mask irqs?
 */
static portMUX_TYPE latch_spinlock = portMUX_INITIALIZER_UNLOCKED;

static void latch_new_ledstate(void)
{
	taskENTER_CRITICAL(&latch_spinlock);
	gpio_set_level(gpio_regs_lat, 0);
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	asm volatile ("nop" ::: "memory");
	gpio_set_level(gpio_regs_lat, 1);
	taskEXIT_CRITICAL(&latch_spinlock);
}

static void spi_task(void *arg)
{
	spi_bus_config_t busconfig = {
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.mosi_io_num = gpio_regs_ser,
		.sclk_io_num = gpio_regs_clk,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS |
			 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_SCLK,
		.max_transfer_sz = sizeof(ledbuffer),
	};
	spi_device_interface_config_t devconfig = {
		.clock_speed_hz = CONFIG_SPI_HZ,
		.mode = 0,
		.spics_io_num = -1,
		.queue_size = 1,
		.command_bits = 0,
		.address_bits = 0,
		.post_cb = spi_tx_done,
		.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_RETURN_RESULT,
	};
	spi_device_handle_t spihandle;
	esp_err_t ret;
	uint32_t pv;

	ret = spi_bus_initialize(SPI2_HOST, &busconfig, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	ret = spi_bus_add_device(SPI2_HOST, &devconfig, &spihandle);
	ESP_ERROR_CHECK(ret);

	do {
		spi_write_new_ledstate(spihandle);
		latch_new_ledstate();

	} while (xTaskGenericNotifyWait(0, 0, 0, &pv, portMAX_DELAY) == pdPASS);
}

static void update_timer_work(void *arg)
{
	TaskHandle_t spi_task_handle = (TaskHandle_t)arg;
	struct ledstate newbuffer = {0};
	BaseType_t no;
	uint32_t prev;

	if (clock_mode == CLOCK_MODE_SECONDS) {
		struct timeval tv_now;
		char buf[5];
		int i;

		gettimeofday(&tv_now, NULL);
		snprintf(buf, sizeof(buf), "%02lld%02ld",
			 tv_now.tv_sec % 60LL,
			 tv_now.tv_usec / 10000L);

		for (i = 0; i < 4; i++)
			newbuffer.digits[i] = led_font[buf[i] - '0'];

		newbuffer.digits[1].h = 1;

	} else if (clock_mode == CLOCK_MODE_MILLIS) {
		struct timeval tv_now;
		char buf[4];
		int i;

		gettimeofday(&tv_now, NULL);
		snprintf(buf, sizeof(buf), "%03ld", tv_now.tv_usec / 1000L);

		newbuffer.digits[0].h = 1;
		for (i = 1; i < 4; i++)
			newbuffer.digits[i] = led_font[buf[i - 1] - '0'];

	} else if (clock_mode == CLOCK_MODE_STATIC) {
		memcpy(&newbuffer, &user_static, sizeof(newbuffer));
	} else {
		const char *strftimefmt;
		struct tm tmi;
		char buf[5];
		time_t now;
		int i;

		switch (clock_mode) {
		case CLOCK_MODE_MONTHDAY:
			strftimefmt = "%m%d";
			break;

		case CLOCK_MODE_DAYMONTH:
			strftimefmt = "%d%m";
			break;

		case CLOCK_MODE_DAYOFYEAR:
			strftimefmt = "%j";
			break;

		case CLOCK_MODE_DAYOFWEEK:
			strftimefmt = "%U %w";
			break;

		case CLOCK_MODE_YEAR:
			strftimefmt = "%Y";
			break;

		default:
		case CLOCK_MODE_TIME:
			strftimefmt = "%H%M";
			break;
		}

		time(&now);
		localtime_r(&now, &tmi);
		strftime(buf, sizeof(buf), strftimefmt, &tmi);

		for (i = 0; i < 4; i++)
			if (buf[i] >= '0' && buf[i] <= '9')
				newbuffer.digits[i] = led_font[buf[i] - '0'];
	}

	/*
	 * Simple optimization: don't kick SPI if nothing has changed.
	 */
	if (!memcmp(&ledbuffer, &newbuffer, sizeof(ledbuffer)))
		return;

	memcpy(&ledbuffer, &newbuffer, sizeof(ledbuffer));
	xTaskGenericNotifyFromISR(spi_task_handle, 0, 0, eNoAction,
				  &prev, &no);
}

static esp_err_t web_ui_get(httpd_req_t *req)
{
	const unsigned index_html_len = index_html_end - index_html_start;

	httpd_resp_set_status(req, "200 OK");
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, index_html_start, index_html_len);
	return ESP_OK;
}

static httpd_uri_t web_root_get = {
	.uri      = "/",
	.method   = HTTP_GET,
	.handler  = web_ui_get,
};

/*
 * Extract key-value pairs from a uri-encoded POST body in sequence.
 */
static int getkv(char **data, const char **key, const char **val)
{
	char *p1, *p2;

	if (!**data)
		return 1;

	p1 = strchrnul(*data, '=');
	if (!*p1)
		return 1;

	*val = NULL;
	*key = *data;
	*p1++ = '\0';

	p2 = strchrnul(p1, '&');
	if (*p2)
		*p2++ = '\0';

	if (*p1)
		*val = p1;

	*data = p2;
	return 0;
}

static esp_err_t web_ui_post(httpd_req_t *req)
{
	char data[256], *next;
	const char *key, *val;
	size_t len;
	int ret;

	len = MIN(req->content_len, sizeof(data) - 1);
	ret = httpd_req_recv(req, data, len);
	if (ret <= 0) {
		if (ret == HTTPD_SOCK_ERR_TIMEOUT)
			httpd_resp_send_408(req);

		return ESP_FAIL;
	}

	data[ret] = '\0';
	next = data;

	while (!getkv(&next, &key, &val)) {
		nvs_handle_t nvshandle;

		if (!strcmp(key, "duty") && val) {
			uint32_t newduty = strtoul(val, NULL, 10);
			newduty = MIN(newduty, led_pwm_duty_max);

			if (nvs_open("cfg", NVS_READWRITE, &nvshandle) == ESP_OK) {
				nvs_set_u32(nvshandle, "duty", newduty);
				nvs_close(nvshandle);
			}

			ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,
						 LEDC_CHANNEL_0,
						 led_pwm_duty_max - newduty, 0);

		} else if (!strcmp(key, "mode") && val) {
			if (!strcmp(val, "time")) {
				clock_mode = CLOCK_MODE_TIME;
			} else if (!strcmp(val, "seconds")) {
				clock_mode = CLOCK_MODE_SECONDS;
			} else if (!strcmp(val, "millis")) {
				clock_mode = CLOCK_MODE_MILLIS;
			} else if (!strcmp(val, "monthday")) {
				clock_mode = CLOCK_MODE_MONTHDAY;
			} else if (!strcmp(val, "daymonth")) {
				clock_mode = CLOCK_MODE_DAYMONTH;
			} else if (!strcmp(val, "dayofyear")) {
				clock_mode = CLOCK_MODE_DAYOFYEAR;
			} else if (!strcmp(val, "dayofweek")) {
				clock_mode = CLOCK_MODE_DAYOFWEEK;
			} else if (!strcmp(val, "year")) {
				clock_mode = CLOCK_MODE_YEAR;
			} else if (!strcmp(val, "static")) {
				clock_mode = CLOCK_MODE_STATIC;
			}

		} else if (!strcmp(key, "display") && val) {
			int i = 0, o = 0;

			while (val[i] && o < 4) {
				switch (val[i]) {
				case '0': case '1': case '2': case '3':
				case '4': case '5': case '6': case '7':
				case '8': case '9':
					user_static.digits[o++] =
						led_font[val[i] - '0'];
					break;
				case '_':
					user_static.digits[o++] = led_font[10];
					break;
				case 'A':
					user_static.digits[o++] = led_font[11];
					break;
				case 'C':
					user_static.digits[o++] = led_font[12];
					break;
				case 'E':
					user_static.digits[o++] = led_font[13];
					break;
				case 'F':
					user_static.digits[o++] = led_font[14];
					break;
				case 'H':
					user_static.digits[o++] = led_font[15];
					break;
				case 'L':
					user_static.digits[o++] = led_font[16];
					break;
				case 'P':
					user_static.digits[o++] = led_font[17];
					break;
				case 'U':
					user_static.digits[o++] = led_font[18];
					break;
				case '-':
					user_static.digits[o++] = led_font[19];
					break;
				case '*':
					user_static.digits[o++] = led_font[20];
					break;
				case '.':
					if (o > 0)
						user_static.digits[o - 1].h = 1;
					break;
				}

				i++;
			}

			if (val[i] == '.')
				user_static.digits[MIN(o, 3)].h = 1;
		}
	}

	return web_ui_get(req);
}

static httpd_uri_t web_root_post = {
	.uri      = "/",
	.method   = HTTP_POST,
	.handler  = web_ui_post,
};

void app_main(void)
{
	esp_timer_create_args_t tickconfig = {
		.callback = update_timer_work,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "update_timer_work",
	};
	ledc_timer_config_t pwm_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.freq_hz = CONFIG_PWM_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ledc_channel_config_t pwm_channel = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = gpio_regs_pwm,
		.duty = led_pwm_duty_max,
		.hpoint = 0,
	};
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	esp_timer_handle_t tick_handle;
	TaskHandle_t spi_task_handle;
	httpd_handle_t server = NULL;
	nvs_handle_t nvshandle;
	esp_err_t ret;

	gpio_reset_pin(gpio_rear_led);
	gpio_reset_pin(gpio_regs_lat);
	gpio_reset_pin(gpio_regs_pwm);
	gpio_set_direction(gpio_rear_led, GPIO_MODE_OUTPUT_OD);
	gpio_set_direction(gpio_regs_lat, GPIO_MODE_OUTPUT);

	gpio_set_direction(gpio_regs_pwm, GPIO_MODE_OUTPUT);
	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
	ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
	ledc_fade_func_install(0);

	gpio_set_level(gpio_rear_led, 0);
	gpio_set_level(gpio_regs_pwm, 1);
	gpio_set_level(gpio_regs_lat, 1);

	spi_task_handle = xTaskCreateStatic(spi_task, "spi_task", 1024, NULL,
					    configMAX_PRIORITIES - 1,
					    spi_task_stack, &spi_task_buffer);

	tickconfig.arg = (void *)spi_task_handle;
	ret = esp_timer_create(&tickconfig, &tick_handle);
	ESP_ERROR_CHECK(ret);

	ret = esp_timer_start_periodic(tick_handle, CONFIG_TIMER_WORK_HZ);
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(start_wifi());

	if (nvs_open("cfg", NVS_READWRITE, &nvshandle) == ESP_OK) {
		uint32_t led_duty = led_pwm_duty_max;

		nvs_get_u32(nvshandle, "duty", &led_duty);
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,
					 LEDC_CHANNEL_0,
					 led_pwm_duty_max - led_duty, 0);

		nvs_close(nvshandle);
	}

	gpio_set_level(gpio_rear_led, 1);

	setenv("TZ", CONFIG_DISPLAY_TZ, 1);
	tzset();

	ESP_ERROR_CHECK(httpd_start(&server, &config));
	httpd_register_uri_handler(server, &web_root_get);
	httpd_register_uri_handler(server, &web_root_post);

	while (1) {
		EventBits_t bits = xEventGroupGetBits(wifi_eg);
		if (bits & WIFI_FAIL_BIT) {
			stop_wifi();
			start_wifi();
		}

		delay_ms(CONFIG_NTP_SYNC_INTERVAL_MS / 2);
	}
}
