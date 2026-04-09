#include <stdio.h>

#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_link.h"

static const char *TAG = "UART_LINK";

esp_err_t uart_link_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2 init OK: %d bauds, TX=%d RX=%d",
             UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    return ESP_OK;
}

int uart_link_send_frame(int co2_uart_ppm, int co2_pwm_ppm, int presence)
{
    char frame[64];
    int len = snprintf(frame, sizeof(frame), "CO2_UART:%d;CO2_PWM:%d;PRES:%d\n",
                       co2_uart_ppm, co2_pwm_ppm, presence);
    int written = uart_write_bytes(UART_PORT_NUM, frame, len);

    if (written < 0) {
        ESP_LOGE(TAG, "Erreur UART write");
    } else {
        ESP_LOGI(TAG, "Trame envoyee: %s", frame);
    }

    return written;
}
