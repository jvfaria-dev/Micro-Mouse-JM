/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   Leitura dos encoders em quadratura.
 ******************************************************************************
 */

#include "encoder.h"
#include "board.h"

#include <stdbool.h>

extern TIM_HandleTypeDef BOARD_ENC_L_TIM;
extern TIM_HandleTypeDef BOARD_ENC_R_TIM;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct
{
    TIM_HandleTypeDef *htim;
    bool               is_32bit;
    bool               invert;
} encoder_cfg_t;

typedef struct
{
    uint32_t last_raw;      /* contador do timer na ultima amostra          */
    int32_t  ticks;         /* acumulado desde o reset                      */
    int32_t  delta;         /* ticks do ultimo update                       */
    int32_t  window_ticks;  /* ticks acumulados na janela de velocidade     */
    uint32_t window_ms;     /* ms decorridos na janela                      */
    int32_t  rpm_milli;     /* velocidade calculada (milesimos de RPM)      */
} encoder_state_t;

static encoder_cfg_t encoder_cfg[ENCODER_COUNT];
static encoder_state_t encoder_state[ENCODER_COUNT];

/* -------------------------------------------------------------------------- */

/**
 * O .ioc gerou os dois timers em "Encoder Mode TI1", que conta so' as bordas
 * de UM canal (x2). Todas as constantes do board.h pressupoem x4 (28 ticks
 * por volta do motor). Em vez de depender do CubeMX, o modo e o filtro sao
 * forcados aqui -- se alguem regerar o projeto, continua certo.
 *
 * SMCR.SMS = 011 -> encoder mode 3 (TI1 e TI2, as 4 bordas)
 * CCMR1.IC1F / IC2F -> filtro digital de entrada
 */
static void encoder_force_x4(TIM_HandleTypeDef *htim)
{
    TIM_TypeDef *t = htim->Instance;
    uint32_t     f = (uint32_t)BOARD_ENC_INPUT_FILTER & 0xFu;

    t->CR1  &= ~TIM_CR1_CEN;
    t->SMCR  = (t->SMCR & ~TIM_SMCR_SMS) | TIM_ENCODERMODE_TI12;
    t->CCMR1 = (t->CCMR1 & ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F))
             | (f << TIM_CCMR1_IC1F_Pos) | (f << TIM_CCMR1_IC2F_Pos);
}

/**
 * Pull-up interno nas quatro entradas dos encoders.
 *
 * O CubeMX gerou os pinos sem pull. Se a saida do encoder for coletor aberto
 * (comum em encoder Hall de N20) e o resistor da plaquinha faltar ou soltar,
 * o pino fica preso em 0 e aquele canal some -- em x4, com um canal parado,
 * as bordas do outro sobem e descem alternadas e a contagem fica em ZERO,
 * mesmo com a roda girando. Com o pull-up interno (~40k) esse modo de falha
 * deixa de existir. Para encoder com saida push-pull ele nao atrapalha.
 * Mexe so' no registrador de pull, sem tocar na funcao alternada do pino.
 */
static void encoder_pin_pullup(GPIO_TypeDef *port, uint16_t pin)
{
    for (uint32_t n = 0u; n < 16u; n++)
    {
        if ((pin & (1u << n)) != 0u)
        {
            port->PUPDR = (port->PUPDR & ~(3u << (2u * n))) | (1u << (2u * n));
        }
    }
}

void port_encoder_init(void)
{
    encoder_pin_pullup(BOARD_ENC_L_A_PORT, BOARD_ENC_L_A_PIN);
    encoder_pin_pullup(BOARD_ENC_L_B_PORT, BOARD_ENC_L_B_PIN);
    encoder_pin_pullup(BOARD_ENC_R_A_PORT, BOARD_ENC_R_A_PIN);
    encoder_pin_pullup(BOARD_ENC_R_B_PORT, BOARD_ENC_R_B_PIN);

    encoder_cfg[ENCODER_LEFT].htim     = &BOARD_ENC_L_TIM;
    encoder_cfg[ENCODER_LEFT].is_32bit = (bool)BOARD_ENC_L_IS_32BIT;
    encoder_cfg[ENCODER_LEFT].invert   = (bool)ENCODER_L_INVERT;

    encoder_cfg[ENCODER_RIGHT].htim     = &BOARD_ENC_R_TIM;
    encoder_cfg[ENCODER_RIGHT].is_32bit = (bool)BOARD_ENC_R_IS_32BIT;
    encoder_cfg[ENCODER_RIGHT].invert   = (bool)ENCODER_R_INVERT;

    for (int i = 0; i < ENCODER_COUNT; i++)
    {
        encoder_force_x4(encoder_cfg[i].htim);
        HAL_TIM_Encoder_Start(encoder_cfg[i].htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(encoder_cfg[i].htim, 0);
    }

    port_encoder_reset();
}

void port_encoder_reset(void)
{
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
        encoder_state[i].last_raw     = __HAL_TIM_GET_COUNTER(encoder_cfg[i].htim);
        encoder_state[i].ticks        = 0;
        encoder_state[i].delta        = 0;
        encoder_state[i].window_ticks = 0;
        encoder_state[i].window_ms    = 0;
        encoder_state[i].rpm_milli    = 0;
    }
}

void port_encoder_update(void)
{
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
        encoder_cfg_t   *cfg = &encoder_cfg[i];
        encoder_state_t *st  = &encoder_state[i];

        uint32_t raw = __HAL_TIM_GET_COUNTER(cfg->htim);

        /* Diferenca com tratamento de wrap-around.
         * A subtracao sem sinal ja' "da a volta" corretamente; basta
         * reinterpretar o resultado como inteiro com sinal da largura certa. */
        int32_t delta;

        if (cfg->is_32bit)
        {
            delta = (int32_t)(raw - st->last_raw);
        }
        else
        {
            delta = (int32_t)(int16_t)((uint16_t)raw - (uint16_t)st->last_raw);
        }

        st->last_raw = raw;

        if (cfg->invert)
        {
            delta = -delta;
        }

        st->delta         = delta;
        st->ticks        += delta;
        st->window_ticks += delta;
        st->window_ms    += CONTROL_PERIOD_MS;

        if (st->window_ms >= ENCODER_SPEED_WINDOW_MS)
        {
            /* rpm = ticks / ticks_por_volta * (60000 / janela_ms)
             * em milesimos de RPM, com int64 para nao estourar. */
            int64_t num = (int64_t)st->window_ticks * 60000LL * 1000LL;
            int64_t den = (int64_t)ENCODER_TICKS_PER_REV * (int64_t)st->window_ms;

            st->rpm_milli = (den != 0) ? (int32_t)(num / den) : 0;

            st->window_ticks = 0;
            st->window_ms    = 0;
        }
    }
}

int32_t port_encoder_get_ticks(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? encoder_state[enc].ticks : 0;
}

int32_t port_encoder_get_delta(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? encoder_state[enc].delta : 0;
}

int32_t port_encoder_get_rpm_milli(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? encoder_state[enc].rpm_milli : 0;
}

float port_encoder_get_rpm(encoder_id_t enc)
{
    return (float)port_encoder_get_rpm_milli(enc) / 1000.0f;
}

float port_encoder_get_distance_mm(encoder_id_t enc)
{
    float revs = (float)port_encoder_get_ticks(enc) / (float)ENCODER_TICKS_PER_REV;
    return revs * (float)M_PI * WHEEL_DIAMETER_MM;
}

uint32_t port_encoder_get_raw(encoder_id_t enc)
{
    return (enc < ENCODER_COUNT) ? __HAL_TIM_GET_COUNTER(encoder_cfg[enc].htim) : 0u;
}
