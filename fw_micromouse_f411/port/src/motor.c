/**
 ******************************************************************************
 * @file    motor.c
 * @brief   Driver da ponte H usando 4 canais de PWM do TIM4.
 *
 * A ponte (DRV8833 / TB6612 em modo "PWM nas entradas") recebe PWM direto nos
 * pinos INx. A tabela de acionamento e':
 *
 *   IN1   IN2   resultado
 *   ---------------------------------
 *    0     0    roda livre (coast)
 *   PWM    0    gira em um sentido, proporcional ao duty
 *    0    PWM   gira no sentido oposto
 *    1     1    freio (curto nos terminais)
 ******************************************************************************
 */

#include "motor.h"
#include "board.h"

extern TIM_HandleTypeDef BOARD_MOTOR_TIM;

/* Descritor estatico de cada motor: quais canais e se o sentido e' invertido */
typedef struct
{
    uint32_t ch_in1;
    uint32_t ch_in2;
    bool     invert;
    int32_t  deadband;      /* offset do atuador, em por-mil */
} motor_cfg_t;

static const motor_cfg_t motor_cfg[MOTOR_COUNT] =
{
    [MOTOR_LEFT]  = { BOARD_MOTOR_L_IN1_CH, BOARD_MOTOR_L_IN2_CH,
                      (bool)MOTOR_L_INVERT, MOTOR_L_DEADBAND },
    [MOTOR_RIGHT] = { BOARD_MOTOR_R_IN1_CH, BOARD_MOTOR_R_IN2_CH,
                      (bool)MOTOR_R_INVERT, MOTOR_R_DEADBAND },
};

static int32_t motor_duty[MOTOR_COUNT] = { 0, 0 };

/* ARR do TIM4, calculado no init a partir do clock real (ver board.h). */
static uint32_t pwm_arr = 799u;

/* Protege contra chamadas antes do init (ex.: Error_Handler disparado
 * durante a inicializacao dos perifericos). */
static bool motor_ready = false;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

/** Escreve o comparador de um canal a partir de um duty em por-mil (0..1000). */
static void motor_write_channel(uint32_t channel, int32_t permille)
{
    if (!motor_ready)
    {
        return;
    }

    if (permille < 0)
    {
        permille = 0;
    }
    else if (permille > 1000)
    {
        permille = 1000;
    }

    /* +1 no ARR porque o periodo real e' ARR+1 contagens */
    uint32_t ccr = (uint32_t)((permille * (int32_t)(pwm_arr + 1u)) / 1000);

    __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, channel, ccr);
}

/**
 * Satura o duty e aplica a compensacao de zona morta.
 *
 * A compensacao remapeia a faixa util: um comando de 1..1000 vira
 * deadband..1000, de modo que o menor comando ja' produza torque. Comando
 * zero continua sendo zero (o motor precisa poder ficar parado de fato).
 */
static int32_t motor_shape_duty(int32_t duty, int32_t deadband)
{
    int32_t limit = MOTOR_DUTY_MAX;

    if (duty > 1000)
    {
        duty = 1000;
    }
    else if (duty < -1000)
    {
        duty = -1000;
    }

    if (deadband > 0)
    {
        if (duty > 0)
        {
            duty = deadband + ((duty * (1000 - deadband)) / 1000);
        }
        else if (duty < 0)
        {
            duty = -deadband + ((duty * (1000 - deadband)) / 1000);
        }
    }

    /* Teto DEPOIS da zona morta: e' este numero que vira tensao no motor.
     * Saturar antes deixaria a compensacao empurrar uns 1% acima. */
    if (duty > limit)
    {
        duty = limit;
    }
    else if (duty < -limit)
    {
        duty = -limit;
    }

    return duty;
}

/* -------------------------------------------------------------------------- */
/* API publica                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * Clock que chega no TIM4. Na F4 os timers do APB1 recebem PCLK1 se o
 * divisor do APB1 for 1, e 2 x PCLK1 se for qualquer outro.
 */
static uint32_t motor_tim_clock(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1)
    {
        return pclk1;
    }
    return 2u * pclk1;
}

/**
 * O .ioc deixou o TIM4 com PSC = 0 e ARR = 65535, ou seja PWM de 244 Hz a
 * 16 MHz -- audivel e com ripple de corrente enorme. Aqui o periodo e'
 * refeito para BOARD_PWM_FREQ_HZ, seja qual for o clock.
 */
static void motor_config_pwm(void)
{
    uint32_t clk = motor_tim_clock();
    uint32_t psc = 0u;
    uint32_t arr = (clk / BOARD_PWM_FREQ_HZ) - 1u;

    /* TIM4 e' de 16 bits: se o clock for alto demais, divide. */
    while (arr > 0xFFFFu)
    {
        psc++;
        arr = (clk / ((psc + 1u) * BOARD_PWM_FREQ_HZ)) - 1u;
    }

    pwm_arr = arr;

    __HAL_TIM_SET_PRESCALER(&BOARD_MOTOR_TIM, psc);
    __HAL_TIM_SET_AUTORELOAD(&BOARD_MOTOR_TIM, arr);
    BOARD_MOTOR_TIM.Instance->EGR = TIM_EGR_UG;     /* aplica ja' */
}

uint32_t port_motor_get_pwm_arr(void)
{
    return pwm_arr;
}

void port_motor_init(void)
{
    motor_config_pwm();
    motor_ready = true;

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motor_write_channel(motor_cfg[i].ch_in1, 0);
        motor_write_channel(motor_cfg[i].ch_in2, 0);

        HAL_TIM_PWM_Start(&BOARD_MOTOR_TIM, motor_cfg[i].ch_in1);
        HAL_TIM_PWM_Start(&BOARD_MOTOR_TIM, motor_cfg[i].ch_in2);

        motor_duty[i] = 0;
    }
}

void port_motor_set(motor_id_t motor, int32_t duty)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    const motor_cfg_t *cfg = &motor_cfg[motor];

    duty = motor_shape_duty(duty, cfg->deadband);
    motor_duty[motor] = duty;

    int32_t applied = cfg->invert ? -duty : duty;
    int32_t magnitude = (applied < 0) ? -applied : applied;

#if (MOTOR_SLOW_DECAY != 0)
    /* Slow decay: o pino "parado" fica em 100% e o outro em 100% - duty.
     * A diferenca entre os dois e' o que vira torque. */
    if (applied >= 0)
    {
        motor_write_channel(cfg->ch_in1, 1000);
        motor_write_channel(cfg->ch_in2, 1000 - magnitude);
    }
    else
    {
        motor_write_channel(cfg->ch_in1, 1000 - magnitude);
        motor_write_channel(cfg->ch_in2, 1000);
    }
#else
    /* Fast decay (sinal-magnitude): PWM em um pino, 0 no outro. */
    if (applied >= 0)
    {
        motor_write_channel(cfg->ch_in1, magnitude);
        motor_write_channel(cfg->ch_in2, 0);
    }
    else
    {
        motor_write_channel(cfg->ch_in1, 0);
        motor_write_channel(cfg->ch_in2, magnitude);
    }
#endif
}

void port_motor_set_both(int32_t duty_left, int32_t duty_right)
{
    port_motor_set(MOTOR_LEFT, duty_left);
    port_motor_set(MOTOR_RIGHT, duty_right);
}

void port_motor_brake(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    motor_write_channel(motor_cfg[motor].ch_in1, 1000);
    motor_write_channel(motor_cfg[motor].ch_in2, 1000);
    motor_duty[motor] = 0;
}

void port_motor_coast(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    motor_write_channel(motor_cfg[motor].ch_in1, 0);
    motor_write_channel(motor_cfg[motor].ch_in2, 0);
    motor_duty[motor] = 0;
}

void port_motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        port_motor_coast((motor_id_t)i);
    }
}

int32_t port_motor_get_duty(motor_id_t motor)
{
    return (motor < MOTOR_COUNT) ? motor_duty[motor] : 0;
}
