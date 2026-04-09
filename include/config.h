#ifndef CONFIG_H
#define CONFIG_H

/* UART2 -> liaison RS232 vers l'ESP32 OLED */
#define UART_PORT_NUM            2
#define UART_TX_PIN              27
#define UART_RX_PIN              26
#define UART_BAUD_RATE           9600

/* Capteur CO2 serie (UART dedie) */
#define CO2_SENSOR_UART_PORT     1
#define CO2_SENSOR_UART_TX_PIN   16
#define CO2_SENSOR_UART_RX_PIN   17
#define CO2_SENSOR_UART_BAUD     9600
#define CO2_SENSOR_READ_TIMEOUT_MS 300

/* Lignes annexes capteur CO2 */
#define CO2_SENSOR_SYNC_PIN      15
#define CO2_SENSOR_PWM_PIN       23
#define CO2_SENSOR_PWM_TIMEOUT_US 1500000
#define CO2_SENSOR_PWM_MAX_PPM   5000

/* Capteur ultrason */
#define ULTRASON_TRIG_PIN        33
#define ULTRASON_ECHO_PIN        32

/* Seuil présence */
#define PRESENCE_THRESHOLD_CM    90.0f

/* Timing des tâches */
#define CO2_TASK_PERIOD_MS       4000
#define PRES_TASK_PERIOD_MS      500
#define UART_TX_PERIOD_MS        1000

/* Timeout écho ultrason */
#define ECHO_TIMEOUT_US          30000

/* Taille queue producteur/consommateur */
#define SENSOR_QUEUE_LENGTH      16

#endif
