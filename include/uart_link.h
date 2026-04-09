#ifndef UART_LINK_H
#define UART_LINK_H

#include "esp_err.h"

esp_err_t uart_link_init(void);
int uart_link_send_frame(int co2_uart_ppm, int co2_pwm_ppm, int presence);

#endif
