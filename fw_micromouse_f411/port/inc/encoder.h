/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   Leitura dos encoders em quadratura (TIM2 e TIM3 em modo encoder).
 *
 * O contador do timer e' lido periodicamente e a diferenca e' acumulada em um
 * inteiro de 32 bits com sinal, tratando corretamente o wrap-around do
 * contador (inclusive no TIM3, que e' de 16 bits).
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

typedef enum
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_COUNT
} encoder_id_t;

/**
 * @brief Inicia os timers em modo encoder e zera os acumuladores.
 *        Chamar depois de MX_TIM2_Init() e MX_TIM3_Init().
 */
void port_encoder_init(void);

/**
 * @brief Amostra os contadores e atualiza acumulados e velocidade.
 *        Deve ser chamada a cada CONTROL_PERIOD_MS (1 kHz).
 */
void port_encoder_update(void);

/** @brief Zera os acumuladores e a velocidade (nao mexe no contador do timer). */
void port_encoder_reset(void);

/** @brief Contagem acumulada desde o ultimo reset, em ticks de quadratura. */
int32_t port_encoder_get_ticks(encoder_id_t enc);

/** @brief Ticks contados no ultimo port_encoder_update(). */
int32_t port_encoder_get_delta(encoder_id_t enc);

/**
 * @brief Velocidade da roda em milesimos de RPM (ex.: 12345 = 12,345 RPM).
 *        Inteiro de proposito: evita depender de printf com float.
 */
int32_t port_encoder_get_rpm_milli(encoder_id_t enc);

/** @brief Mesma velocidade, em float, para quem for fazer o controle. */
float port_encoder_get_rpm(encoder_id_t enc);

/** @brief Distancia percorrida pela roda desde o reset, em milimetros. */
float port_encoder_get_distance_mm(encoder_id_t enc);

/** @brief Valor cru do contador do timer (util para depurar a fiacao). */
uint32_t port_encoder_get_raw(encoder_id_t enc);
