#include "co2_sensor.h"
#include "config.h"
#include "consumer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "presence_sensor.h"
#include "sensor_message.h"
#include "uart_link.h"

static const char *TAG = "REMOTE_SENSORS";

void app_main(void)
{
    QueueHandle_t sensor_queue;

    ESP_LOGI(TAG, "Demarrage carte capteurs distants");

    ESP_ERROR_CHECK(uart_link_init());
    ESP_ERROR_CHECK(co2_sensor_init());
    ESP_ERROR_CHECK(presence_sensor_init());

    sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(sensor_msg_t));
    if (sensor_queue == NULL) {
        ESP_LOGE(TAG, "Impossible de creer la queue capteurs");
        return;
    }

    ESP_LOGI(TAG, "Queue capteurs creee (taille=%d)", SENSOR_QUEUE_LENGTH);

    xTaskCreate(co2_producer_task, "co2_producer", 3072, sensor_queue, 5, NULL);
    xTaskCreate(presence_producer_task, "presence_producer", 4096, sensor_queue, 5, NULL);
    xTaskCreate(consumer_uart_task, "uart_consumer", 4096, sensor_queue, 6, NULL);
}