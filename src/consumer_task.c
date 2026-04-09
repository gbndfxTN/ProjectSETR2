#include "config.h"
#include "consumer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor_message.h"
#include "uart_link.h"

static const char *TAG = "CONSUMER";

void consumer_uart_task(void *arg)
{
    QueueHandle_t sensor_queue = (QueueHandle_t)arg;
    sensor_msg_t msg;
    int last_co2_uart_ppm = 400;
    int last_co2_pwm_ppm = 400;
    int last_presence = 0;

    TickType_t next_tx_tick = xTaskGetTickCount() + pdMS_TO_TICKS(UART_TX_PERIOD_MS);

    while (1) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait_ticks = (next_tx_tick > now) ? (next_tx_tick - now) : 0;

        if (xQueueReceive(sensor_queue, &msg, wait_ticks) == pdTRUE) {
            if (msg.type == SENSOR_TYPE_CO2_UART) {
                last_co2_uart_ppm = msg.data.co2_uart_ppm;
            } else if (msg.type == SENSOR_TYPE_CO2_PWM) {
                last_co2_pwm_ppm = msg.data.co2_pwm_ppm;
            } else if (msg.type == SENSOR_TYPE_PRESENCE) {
                last_presence = msg.data.presence;
            }
            continue;
        }

        if (uart_link_send_frame(last_co2_uart_ppm, last_co2_pwm_ppm, last_presence) < 0) {
            ESP_LOGE(TAG, "Echec envoi trame UART");
        }

        next_tx_tick += pdMS_TO_TICKS(UART_TX_PERIOD_MS);

        if (next_tx_tick < xTaskGetTickCount()) {
            next_tx_tick = xTaskGetTickCount() + pdMS_TO_TICKS(UART_TX_PERIOD_MS);
        }
    }
}
