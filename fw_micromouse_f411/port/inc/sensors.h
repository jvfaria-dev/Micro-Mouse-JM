/**
 ******************************************************************************
 * @file    sensors.h
 * @brief   Quatro sensores IR de refletancia lidos pelo ADC1.
 *
 * Arranjo no carrinho (visto de cima, frente para o topo):
 *
 *        PA2              PA1          <- frontais (esq / dir)
 *          \              /
 *   PA3 -[  CARRINHO  ]- PA0           <- laterais, olhando a 90 graus
 *
 * O driver entrega duas coisas:
 *   - o valor CRU do ADC, para calibrar;
 *   - a PROXIMIDADE de 0 a 1000, que ja' vem calibrada e filtrada.
 *
 * A proximidade e' o que o resto do firmware usa, e ela e' independente da
 * polaridade do modulo: se o seu sensor da' tensao ALTA vendo branco ou
 * BAIXA vendo branco, basta que RAW_FAR e RAW_NEAR do board.h estejam
 * corretos -- a interpolacao cuida do sinal sozinha.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SENSOR_LEFT = 0,    /* PA3 (A3) - lateral esquerda, 90 graus */
    SENSOR_FRONT_L,     /* PA2 (A2) - frontal esquerda           */
    SENSOR_FRONT_R,     /* PA1 (A1) - frontal direita            */
    SENSOR_RIGHT,       /* PA0 (A0) - lateral direita, 90 graus  */
    SENSOR_COUNT
} sensor_id_t;

/** @brief Prepara o ADC. Chamar depois de MX_ADC1_Init(). */
void port_sensors_init(void);

/**
 * @brief Le os quatro canais e atualiza os filtros.
 *        Bloqueia por ~0,5 ms. Chamar do laco principal, nao de interrupcao.
 */
void port_sensors_update(void);

/** @brief Valor cru do ADC (0..4095), sem filtro. Use para calibrar. */
uint16_t port_sensors_get_raw(sensor_id_t s);

/** @brief Valor cru ja' filtrado, util para calibrar sem tremer. */
uint16_t port_sensors_get_raw_filtered(sensor_id_t s);

/**
 * @brief Proximidade calibrada, 0..2000.
 *        0    = sem parede / corredor aberto
 *        1000 = parede exatamente na distancia de REFERENCIA da calibracao
 *        >1000 = mais perto que a referencia (satura em 2000)
 */
int16_t port_sensors_get_prox(sensor_id_t s);

/** @brief true se ha' parede daquele lado (proximidade > limiar). */
bool port_sensors_wall(sensor_id_t s);

/** @brief true se ha' parede logo a' frente (media dos dois frontais). */
bool port_sensors_wall_ahead(void);

/** @brief Media dos dois sensores frontais, em proximidade. */
int16_t port_sensors_front(void);

/**
 * @brief Erro de centralizacao no corredor, -1000..+1000.
 *        Positivo = perto demais da parede DIREITA (precisa virar p/ esquerda).
 *        Com so' uma parede visivel, usa essa parede como referencia.
 *        Com nenhuma, devolve 0.
 */
int16_t port_sensors_center_error(void);

/**
 * @brief Troca a calibracao de um sensor em tempo de execucao (cru "sem parede"
 *        e cru "parede na distancia de referencia"). Usada na largada, onde as
 *        duas laterais sao parede com o robo no centro e a frente e' aberta.
 */
void    port_sensors_set_cal(sensor_id_t s, int32_t raw_far, int32_t raw_near);
int32_t port_sensors_get_cal_far(sensor_id_t s);
int32_t port_sensors_get_cal_near(sensor_id_t s);

/** @brief Nome curto do sensor, para a telemetria. */
const char *port_sensors_name(sensor_id_t s);
