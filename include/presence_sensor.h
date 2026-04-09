#ifndef PRESENCE_SENSOR_H
#define PRESENCE_SENSOR_H

#include "esp_err.h"

esp_err_t presence_sensor_init(void);
void presence_producer_task(void *arg);

#endif
