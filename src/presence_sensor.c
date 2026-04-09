#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "presence_sensor.h"
#include "sensor_message.h"

static const char *TAG = "PRES_SENSOR";

static void busy_wait_us(uint32_t delay_us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (int64_t)delay_us) {
        /* Busy wait court pour l'impulsion TRIG (10 us). */
    }
}

static bool ultrasonic_read_distance_cm(float *distance_cm)
{
    int64_t t0;
    int64_t pulse_start;
    int64_t pulse_end;

    gpio_set_level(ULTRASON_TRIG_PIN, 0);
    busy_wait_us(2);
    gpio_set_level(ULTRASON_TRIG_PIN, 1);
    busy_wait_us(10);
    gpio_set_level(ULTRASON_TRIG_PIN, 0);

    t0 = esp_timer_get_time();
    while (gpio_get_level(ULTRASON_ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - t0) > ECHO_TIMEOUT_US) {
            return false;
        }
    }

    pulse_start = esp_timer_get_time();
    while (gpio_get_level(ULTRASON_ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - pulse_start) > ECHO_TIMEOUT_US) {
            return false;
        }
    }
    pulse_end = esp_timer_get_time();

    *distance_cm = (float)(pulse_end - pulse_start) / 58.0f;
    return true;
}

esp_err_t presence_sensor_init(void)
{
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << ULTRASON_TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << ULTRASON_ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&trig_cfg));
    ESP_ERROR_CHECK(gpio_config(&echo_cfg));
    ESP_ERROR_CHECK(gpio_set_level(ULTRASON_TRIG_PIN, 0));

    ESP_LOGI(TAG, "Ultrason init OK: TRIG=%d ECHO=%d", ULTRASON_TRIG_PIN, ULTRASON_ECHO_PIN);
    return ESP_OK;
}

void presence_producer_task(void *arg)
{
    QueueHandle_t sensor_queue = (QueueHandle_t)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        sensor_msg_t msg;
        float distance_cm = 0.0f;
        bool ok = ultrasonic_read_distance_cm(&distance_cm);
        int presence = 0;

        if (!ok) {
            ESP_LOGW(TAG, "Timeout ultrason");
            presence = 0;
        } else {
            presence = (distance_cm > 0.0f && distance_cm <= PRESENCE_THRESHOLD_CM) ? 1 : 0;
            ESP_LOGI(TAG, "Presence mesure: distance=%.1f cm -> PRES=%d", distance_cm, presence);
        }

        msg.type = SENSOR_TYPE_PRESENCE;
        msg.data.presence = presence;

        if (xQueueSend(sensor_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue pleine: mesure presence ignoree");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PRES_TASK_PERIOD_MS));
    }
}
