#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "co2_sensor.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sensor_message.h"

static const char *TAG = "CO2_SENSOR";

static bool parse_ppm_from_ascii(const char *buf, int len, int *ppm_out)
{
    int i = 0;
    int value = 0;
    bool found_digit = false;

    while (i < len && !isdigit((unsigned char)buf[i])) {
        i++;
    }

    while (i < len && isdigit((unsigned char)buf[i])) {
        found_digit = true;
        value = (value * 10) + (buf[i] - '0');
        i++;
    }

    if (!found_digit) {
        return false;
    }

    if (value < 0) {
        value = 0;
    }

    *ppm_out = value;
    return true;
}

static bool co2_sensor_read_ppm(int *ppm)
{
    uint8_t rx_buf[64];
    int read_len = uart_read_bytes(
        CO2_SENSOR_UART_PORT,
        rx_buf,
        sizeof(rx_buf) - 1,
        pdMS_TO_TICKS(CO2_SENSOR_READ_TIMEOUT_MS));

    if (read_len <= 0) {
        return false;
    }

    rx_buf[read_len] = '\0';

    return parse_ppm_from_ascii((const char *)rx_buf, read_len, ppm);
}

static bool wait_level_with_timeout(gpio_num_t pin, int expected_level, int64_t timeout_us)
{
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(pin) != expected_level) {
        if ((esp_timer_get_time() - t0) > timeout_us) {
            return false;
        }
    }
    return true;
}

static bool co2_sensor_read_pwm_ppm(int *ppm_out)
{
    const int64_t timeout_us = CO2_SENSOR_PWM_TIMEOUT_US;
    int64_t high_start;
    int64_t high_end;
    int64_t next_rising;
    int64_t high_us;
    int64_t low_us;
    float numerator;
    float denominator;
    int ppm;

    if (!wait_level_with_timeout(CO2_SENSOR_PWM_PIN, 0, timeout_us)) {
        return false;
    }
    if (!wait_level_with_timeout(CO2_SENSOR_PWM_PIN, 1, timeout_us)) {
        return false;
    }
    high_start = esp_timer_get_time();

    if (!wait_level_with_timeout(CO2_SENSOR_PWM_PIN, 0, timeout_us)) {
        return false;
    }
    high_end = esp_timer_get_time();

    if (!wait_level_with_timeout(CO2_SENSOR_PWM_PIN, 1, timeout_us)) {
        return false;
    }
    next_rising = esp_timer_get_time();

    high_us = high_end - high_start;
    low_us = next_rising - high_end;

    if (high_us <= 0 || low_us <= 0) {
        return false;
    }

    /* Formule PWM courante des capteurs NDIR: ppm = range * (Th - 2ms) / (Th + Tl - 4ms). */
    numerator = (float)high_us - 2000.0f;
    denominator = ((float)high_us + (float)low_us) - 4000.0f;
    if (denominator <= 0.0f) {
        return false;
    }

    ppm = (int)((CO2_SENSOR_PWM_MAX_PPM * numerator) / denominator);
    if (ppm < 0) {
        ppm = 0;
    }
    if (ppm > CO2_SENSOR_PWM_MAX_PPM) {
        ppm = CO2_SENSOR_PWM_MAX_PPM;
    }

    *ppm_out = ppm;
    return true;
}

esp_err_t co2_sensor_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate = CO2_SENSOR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    const gpio_config_t pwm_cfg = {
        .pin_bit_mask = (1ULL << CO2_SENSOR_PWM_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const gpio_config_t sync_cfg = {
        .pin_bit_mask = (1ULL << CO2_SENSOR_SYNC_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(CO2_SENSOR_UART_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CO2_SENSOR_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        CO2_SENSOR_UART_PORT,
        CO2_SENSOR_UART_TX_PIN,
        CO2_SENSOR_UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_config(&pwm_cfg));
    ESP_ERROR_CHECK(gpio_config(&sync_cfg));

    ESP_LOGI(TAG, "CO2 init OK: UART%d @ %d, TX=%d RX=%d, PWM=%d, SYNC=%d",
             CO2_SENSOR_UART_PORT,
             CO2_SENSOR_UART_BAUD,
             CO2_SENSOR_UART_TX_PIN,
             CO2_SENSOR_UART_RX_PIN,
             CO2_SENSOR_PWM_PIN,
             CO2_SENSOR_SYNC_PIN);
    return ESP_OK;
}

void co2_producer_task(void *arg)
{
    QueueHandle_t sensor_queue = (QueueHandle_t)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        sensor_msg_t msg_uart;
        sensor_msg_t msg_pwm;
        int ppm_uart = 0;
        int ppm_pwm = 0;
        bool ok_uart = co2_sensor_read_ppm(&ppm_uart);
        bool ok_pwm = co2_sensor_read_pwm_ppm(&ppm_pwm);

        msg_uart.type = SENSOR_TYPE_CO2_UART;
        if (ok_uart) {
            msg_uart.data.co2_uart_ppm = ppm_uart;
            ESP_LOGI(TAG, "CO2 UART: %d ppm", ppm_uart);
        } else {
            ESP_LOGW(TAG, "Lecture CO2 UART en echec");
            msg_uart.data.co2_uart_ppm = 0;
        }

        msg_pwm.type = SENSOR_TYPE_CO2_PWM;
        if (ok_pwm) {
            msg_pwm.data.co2_pwm_ppm = ppm_pwm;
            ESP_LOGI(TAG, "CO2 PWM: %d ppm", ppm_pwm);
        } else {
            ESP_LOGW(TAG, "Lecture CO2 PWM en echec");
            msg_pwm.data.co2_pwm_ppm = 0;
        }

        if (xQueueSend(sensor_queue, &msg_uart, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue pleine: mesure CO2 UART ignoree");
        }

        if (xQueueSend(sensor_queue, &msg_pwm, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue pleine: mesure CO2 PWM ignoree");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CO2_TASK_PERIOD_MS));
    }
}
