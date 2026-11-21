/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   Leitura dos encoders em quadratura (TIM2 e TIM3).
 *          Baseado na logica do fw_micromouse_f411 adaptado para Blue Pill.
 ******************************************************************************
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    ENCODER_LEFT = 0,   /* TIM2 (PA15/PB3)  */
    ENCODER_RIGHT,      /* TIM3 (PB4/PB5)   */
    ENCODER_COUNT
} encoder_id_t;

/** @brief Inicia os timers em modo encoder e zera os acumuladores. */
void     encoder_init(void);

/** @brief Amostra os contadores e atualiza acumulados, delta e velocidade. */
void     encoder_update(void);

/** @brief Zera contadores acumulados e velocidade. */
void     encoder_reset(void);

/** @brief Ticks acumulados desde o reset. */
int32_t  encoder_get_ticks(encoder_id_t enc);

/** @brief Ticks contados no ultimo update. */
int32_t  encoder_get_delta(encoder_id_t enc);

/** @brief Velocidade em milesimos de RPM (ex: 12500 = 12.5 RPM). */
int32_t  encoder_get_rpm_milli(encoder_id_t enc);

/** @brief Velocidade em RPM (float). */
float    encoder_get_rpm(encoder_id_t enc);

/** @brief Distancia percorrida pela roda em milimetros (mm). */
float    encoder_get_distance_mm(encoder_id_t enc);

/** @brief Valor cru do contador do timer (0..65535). */
uint16_t encoder_get_raw(encoder_id_t enc);

/* --- Aliases da API do fw_micromouse_f411 (port_encoder_...) --- */
#define port_encoder_init()             encoder_init()
#define port_encoder_update()           encoder_update()
#define port_encoder_reset()            encoder_reset()
#define port_encoder_get_ticks(e)       encoder_get_ticks(e)
#define port_encoder_get_delta(e)       encoder_get_delta(e)
#define port_encoder_get_rpm_milli(e)   encoder_get_rpm_milli(e)
#define port_encoder_get_rpm(e)         encoder_get_rpm(e)
#define port_encoder_get_distance_mm(e) encoder_get_distance_mm(e)
#define port_encoder_get_raw(e)         ((uint32_t)encoder_get_raw(e))

#endif /* ENCODER_H */
