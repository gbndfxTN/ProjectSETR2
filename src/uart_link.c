#include <stdio.h>

#include "config.h"
#include "driver/uart.h"
#include "uart_link.h"

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

    printf("[UART_LINK] UART2 init OK: %d bauds, TX=%d RX=%d\n",
           UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    return ESP_OK;
}

int uart_link_send_frame(int co2_uart_ppm, int co2_pwm_ppm, int presence)
{
    char frame[64];
    int len = snprintf(frame, sizeof(frame), "CO2_UART:%d;CO2_PWM:%d;PRES:%d\n",
                       co2_uart_ppm, co2_pwm_ppm, presence);
    if (len <= 0 || len >= (int)sizeof(frame)) {
        printf("[UART_LINK] Construction trame invalide (len=%d)\n", len);
        return -1;
    }

    printf("[UART_LINK] TX preparee (len=%d): %s", len, frame);
    int written = uart_write_bytes(UART_PORT_NUM, frame, len);

    if (written < 0) {
        printf("[UART_LINK] Erreur UART write\n");
    } else {
        esp_err_t tx_done = uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));
        printf("[UART_LINK] UART write=%d/%d, tx_done=%s\n", written, len,
               (tx_done == ESP_OK) ? "OK" : "TIMEOUT");
        printf("[UART_LINK] Trame envoyee: %s", frame);
    }

    return written;
}
