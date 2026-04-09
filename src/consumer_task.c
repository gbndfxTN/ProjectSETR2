#include <stdio.h>

#include "config.h"
#include "consumer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor_message.h"
#include "uart_link.h"

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
                printf("[CONSUMER] Maj queue CO2_UART=%d ppm\n", last_co2_uart_ppm);
            } else if (msg.type == SENSOR_TYPE_CO2_PWM) {
                last_co2_pwm_ppm = msg.data.co2_pwm_ppm;
                printf("[CONSUMER] Maj queue CO2_PWM=%d ppm\n", last_co2_pwm_ppm);
            } else if (msg.type == SENSOR_TYPE_PRESENCE) {
                last_presence = msg.data.presence;
                printf("[CONSUMER] Maj queue PRES=%d\n", last_presence);
            }
        }

        now = xTaskGetTickCount();
        if (now >= next_tx_tick) {
            printf("[CONSUMER] TX tick: co2_uart=%d co2_pwm=%d pres=%d\n",
                   last_co2_uart_ppm, last_co2_pwm_ppm, last_presence);
            if (uart_link_send_frame(last_co2_uart_ppm, last_co2_pwm_ppm, last_presence) < 0) {
                printf("[CONSUMER] Echec envoi trame UART\n");
            }

            next_tx_tick += pdMS_TO_TICKS(UART_TX_PERIOD_MS);
            if (next_tx_tick < now) {
                next_tx_tick = now + pdMS_TO_TICKS(UART_TX_PERIOD_MS);
            }
        }
    }
}
