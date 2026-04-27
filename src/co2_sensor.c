#include <stdbool.h>
#include <stdint.h>

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
static int64_t s_init_time_us;

static uint8_t mhz19_checksum(const uint8_t frame[9])
{
    uint16_t sum = 0;
    for (int i = 1; i < 8; i++) {
        sum += frame[i];
    }
    return (uint8_t)(0xFF - (sum & 0xFF) + 1);
}

static bool mhz19_send_command(const uint8_t command[9])
{
    uart_flush_input(CO2_SENSOR_UART_PORT);

    if (uart_write_bytes(CO2_SENSOR_UART_PORT, command, 9) != 9) {
        ESP_LOGW(TAG, "Commande MH-Z19B non envoyee");
        return false;
    }

    if (uart_wait_tx_done(CO2_SENSOR_UART_PORT, pdMS_TO_TICKS(100)) != ESP_OK) {
        ESP_LOGW(TAG, "Timeout TX commande MH-Z19B");
        return false;
    }

    return true;
}

static bool co2_sensor_read_ppm(int *ppm)
{
    const uint8_t cmd_read_ppm[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    uint8_t response[9];

    if (!mhz19_send_command(cmd_read_ppm)) {
        return false;
    }

    int read_len = uart_read_bytes(CO2_SENSOR_UART_PORT,
                                   response,
                                   sizeof(response),
                                   pdMS_TO_TICKS(CO2_SENSOR_READ_TIMEOUT_MS));

    if (read_len != sizeof(response)) {
        ESP_LOGW(TAG, "Reponse UART MH-Z19B incomplete (len=%d)", read_len);
        return false;
    }

    if (response[0] != 0xFF || response[1] != 0x86) {
        ESP_LOGW(TAG, "Entete reponse MH-Z19B invalide: %02X %02X", response[0], response[1]);
        return false;
    }

    if (mhz19_checksum(response) != response[8]) {
        ESP_LOGW(TAG, "Checksum MH-Z19B invalide: calc=%02X recv=%02X", mhz19_checksum(response), response[8]);
        return false;
    }

    *ppm = ((int)response[2] << 8) | response[3];
    if (*ppm < 0) {
        *ppm = 0;
    }

    return true;
}

static bool co2_sensor_zero_calibrate(void)
{
    const uint8_t cmd_zero[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};

    ESP_LOGW(TAG, "Calibration zero MH-Z19B demandee: verifier air frais stable avant boot");
    return mhz19_send_command(cmd_zero);
}

static bool co2_sensor_set_abc(bool enabled)
{
    uint8_t cmd_abc[9] = {0xFF, 0x01, 0x79, enabled ? 0xA0 : 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    cmd_abc[8] = mhz19_checksum(cmd_abc);
    ESP_LOGI(TAG, "ABC MH-Z19B: %s", enabled ? "active" : "desactive");
    return mhz19_send_command(cmd_abc);
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

    ppm = (int)((CO2_SENSOR_DETECTION_RANGE_PPM * numerator) / denominator);
    if (ppm < 0) {
        ppm = 0;
    }
    if (ppm > CO2_SENSOR_DETECTION_RANGE_PPM) {
        ppm = CO2_SENSOR_DETECTION_RANGE_PPM;
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

    s_init_time_us = esp_timer_get_time();

#if CO2_SENSOR_CONFIGURE_ABC_ON_BOOT
    if (!co2_sensor_set_abc(CO2_SENSOR_ABC_ENABLED != 0)) {
        ESP_LOGW(TAG, "Configuration ABC MH-Z19B en echec");
    }
#endif

#if CO2_SENSOR_ZERO_CALIBRATE_ON_BOOT
    if (!co2_sensor_zero_calibrate()) {
        ESP_LOGW(TAG, "Calibration zero MH-Z19B en echec");
    }
#else
    ESP_LOGI(TAG, "Calibration zero MH-Z19B au boot desactivee");
#endif

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
        int64_t age_ms = (esp_timer_get_time() - s_init_time_us) / 1000;
        bool ok_uart = co2_sensor_read_ppm(&ppm_uart);
        bool ok_pwm = co2_sensor_read_pwm_ppm(&ppm_pwm);

        if (age_ms < CO2_SENSOR_WARMUP_MS) {
            ESP_LOGW(TAG, "Capteur CO2 en chauffe (%lld/%d ms): mesures possiblement instables",
                     (long long)age_ms, CO2_SENSOR_WARMUP_MS);
        }

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
