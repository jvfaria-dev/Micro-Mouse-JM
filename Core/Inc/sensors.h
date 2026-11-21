/**
 ******************************************************************************
 * @file    sensors.h
 * @brief   Leitura e calibracao dos 5 sensores Sharp IR pelo ADC1 (Blue Pill).
 *          Mapeamento dos 5 canais e compatibilidade com fw_micromouse_f411.
 *
 * Mapeamento:
 *   SL1 (PA3) -> Lateral Direito  (SENSOR_RIGHT)
 *   SL2 (PB0) -> Lateral Esquerdo (SENSOR_LEFT)
 *   SL3 (PA4) -> Diagonal Direita (SENSOR_FRONT_R)
 *   SL4 (PA6) -> Diagonal Esquerda (SENSOR_FRONT_L)
 *   SL5 (PA5) -> Central Frontal  (SENSOR_FRONT)
 ******************************************************************************
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SENSOR_LEFT = 0,    /* PB0 - SL2: Lateral Esquerda */
    SENSOR_FRONT_L,     /* PA6 - SL4: Frontal / Diagonal Esquerda */
    SENSOR_FRONT,       /* PA5 - SL5: Central Frontal */
    SENSOR_FRONT_R,     /* PA4 - SL3: Frontal / Diagonal Direita */
    SENSOR_RIGHT,       /* PA3 - SL1: Lateral Direita */
    SENSOR_COUNT
} sensor_id_t;

/* Mapeamento nominal dos 5 sensores Sharp */
#define SENSOR_SL1      SENSOR_RIGHT
#define SENSOR_SL2      SENSOR_LEFT
#define SENSOR_SL3      SENSOR_FRONT_R
#define SENSOR_SL4      SENSOR_FRONT_L
#define SENSOR_SL5      SENSOR_FRONT

/* --- Funcoes Principais (compatibilidade com ambos port_sensors_ e sensors_) --- */
void     sensors_init(void);
void     sensors_update(void);
uint16_t sensors_get_raw(sensor_id_t s);
uint16_t sensors_get_raw_filtered(sensor_id_t s);
uint32_t sensors_get_mv(sensor_id_t s);
int16_t  sensors_get_prox(sensor_id_t s);
bool     sensors_wall(sensor_id_t s);
bool     sensors_wall_ahead(void);
int16_t  sensors_front(void);
int16_t  sensors_center_error(void);
void     sensors_set_cal(sensor_id_t s, int32_t raw_far, int32_t raw_near);
int32_t  sensors_get_cal_far(sensor_id_t s);
int32_t  sensors_get_cal_near(sensor_id_t s);
const char *sensors_name(sensor_id_t s);

/* Funcoes legadas para testes */
uint16_t sensors_read_raw(uint32_t channel);
void     sensors_read_all(uint16_t raw[5]);

/* Aliases port_sensors_... do fw_micromouse_f411 */
#define port_sensors_init()                 sensors_init()
#define port_sensors_update()               sensors_update()
#define port_sensors_get_raw(s)             sensors_get_raw(s)
#define port_sensors_get_raw_filtered(s)    sensors_get_raw_filtered(s)
#define port_sensors_get_prox(s)            sensors_get_prox(s)
#define port_sensors_wall(s)                sensors_wall(s)
#define port_sensors_wall_ahead()           sensors_wall_ahead()
#define port_sensors_front()                sensors_front()
#define port_sensors_center_error()         sensors_center_error()
#define port_sensors_set_cal(s, f, n)       sensors_set_cal(s, f, n)
#define port_sensors_get_cal_far(s)         sensors_get_cal_far(s)
#define port_sensors_get_cal_near(s)        sensors_get_cal_near(s)
#define port_sensors_name(s)                sensors_name(s)

#endif /* SENSORS_H */
