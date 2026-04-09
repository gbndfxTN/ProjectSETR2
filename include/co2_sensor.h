#ifndef CO2_SENSOR_H
#define CO2_SENSOR_H

#include "esp_err.h"

esp_err_t co2_sensor_init(void);
void co2_producer_task(void *arg);

#endif
