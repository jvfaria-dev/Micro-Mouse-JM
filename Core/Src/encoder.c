/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   Leitura dos encoders em quadratura (TIM2 e TIM3).
 *          Implementacao com tratamento de wrap-around de 16 bits.
 ******************************************************************************
 */
#include "encoder.h"
#include "board.h"
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ENCODER_SPEED_WINDOW_MS 50u

extern TIM_HandleTypeDef BOARD_ENC_L_TIM;
extern TIM_HandleTypeDef BOARD_ENC_R_TIM;

typedef struct
{
    TIM_HandleTypeDef *htim;
    bool               invert;
} encoder_cfg_t;

typedef struct
{
    uint16_t last_raw;
    int32_t  ticks;
    int32_t  delta;
    int32_t  window_ticks;
    uint32_t last_time_ms;
    int32_t  rpm_milli;
} encoder_state_t;

static encoder_cfg_t encoder_cfg[ENCODER_COUNT];
static encoder_state_t encoder_state[ENCODER_COUNT];

void encoder_init(void)
{
    encoder_cfg[ENCODER_LEFT].htim   = &BOARD_ENC_L_TIM;
    encoder_cfg[ENCODER_LEFT].invert = (bool)ENCODER_L_INVERT;

    encoder_cfg[ENCODER_RIGHT].htim   = &BOARD_ENC_R_TIM;
    encoder_cfg[ENCODER_RIGHT].invert = (bool)ENCODER_R_INVERT;

#if BOARD_USE_ENCODERS
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
        HAL_TIM_Encoder_Start(encoder_cfg[i].htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(encoder_cfg[i].htim, 0);
    }
#endif

    encoder_reset();
}

void encoder_reset(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
        encoder_state[i].last_raw     = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_cfg[i].htim);
        encoder_state[i].ticks        = 0;
        encoder_state[i].delta        = 0;
        encoder_state[i].window_ticks = 0;
        encoder_state[i].last_time_ms = now;
        encoder_state[i].rpm_milli    = 0;
    }
}

void encoder_update(void)
{
#if !BOARD_USE_ENCODERS
    /* Encoders desligados no board.h: nada a ler, tudo fica em zero. */
    return;
#endif
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < ENCODER_COUNT; i++)
    {
        encoder_cfg_t   *cfg = &encoder_cfg[i];
        encoder_state_t *st  = &encoder_state[i];

        uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(cfg->htim);

        /* Diferenca com tratamento correto de wrap-around de 16 bits */
        int32_t delta = (int32_t)(int16_t)(raw - st->last_raw);
        st->last_raw = raw;

        if (cfg->invert)
        {
            delta = -delta;
        }

        st->delta         = delta;
        st->ticks        += delta;
        st->window_ticks += delta;

        uint32_t elapsed_ms = now - st->last_time_ms;
        if (elapsed_ms >= ENCODER_SPEED_WINDOW_MS)
        {
            /* RPM = (window_ticks / ENCODER_TICKS_PER_REV) * (60000 / elapsed_ms) */
            int64_t num = (int64_t)st->window_ticks * 60000LL * 1000LL;
            int64_t den = (int64_t)ENCODER_TICKS_PER_REV * (int64_t)elapsed_ms;

            st->rpm_milli = (den != 0) ? (int32_t)(num / den) : 0;

            st->window_ticks = 0;
            st->last_time_ms = now;
        }
    }
}

int32_t encoder_get_ticks(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? encoder_state[enc].ticks : 0;
}

int32_t encoder_get_delta(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? encoder_state[enc].delta : 0;
}

int32_t encoder_get_rpm_milli(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? encoder_state[enc].rpm_milli : 0;
}

float encoder_get_rpm(encoder_id_t enc)
{
    return (float)encoder_get_rpm_milli(enc) / 1000.0f;
}

float encoder_get_distance_mm(encoder_id_t enc)
{
    return (float)encoder_get_ticks(enc) * MM_PER_TICK;
}

uint16_t encoder_get_raw(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? (uint16_t)__HAL_TIM_GET_COUNTER(encoder_cfg[enc].htim) : 0u;
}

