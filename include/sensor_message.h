#ifndef SENSOR_MESSAGE_H
#define SENSOR_MESSAGE_H

typedef enum {
    SENSOR_TYPE_CO2_UART = 0,
    SENSOR_TYPE_CO2_PWM,
    SENSOR_TYPE_PRESENCE
} sensor_type_t;

typedef struct {
    sensor_type_t type;
    union {
        int co2_uart_ppm;
        int co2_pwm_ppm;
        int presence;
    } data;
} sensor_msg_t;

#endif
