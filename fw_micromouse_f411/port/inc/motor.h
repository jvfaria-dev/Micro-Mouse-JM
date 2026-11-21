/**
 ******************************************************************************
 * @file    motor.h
 * @brief   Driver da ponte H (2 motores DC, PWM nos 4 pinos de entrada).
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
} motor_id_t;

/**
 * @brief Liga os 4 canais de PWM e deixa os dois motores parados (coast).
 *        Chamar depois de MX_TIM4_Init().
 */
void port_motor_init(void);

/**
 * @brief Define o esforco de um motor.
 * @param motor MOTOR_LEFT ou MOTOR_RIGHT
 * @param duty  -1000 (re' total) .. 0 (parado) .. +1000 (frente total).
 *              Valores fora da faixa sao saturados em MOTOR_DUTY_MAX.
 */
void port_motor_set(motor_id_t motor, int32_t duty);

/** @brief Aplica o esforco nos dois motores de uma vez. */
void port_motor_set_both(int32_t duty_left, int32_t duty_right);

/**
 * @brief Frenagem ativa (curto nos terminais: IN1 = IN2 = 100%).
 *        Para rapido, mas gera pico de corrente.
 */
void port_motor_brake(motor_id_t motor);

/**
 * @brief Roda livre (IN1 = IN2 = 0). O motor desacelera sozinho.
 */
void port_motor_coast(motor_id_t motor);

/** @brief Coloca os dois motores em roda livre. Use em erro/emergencia. */
void port_motor_stop_all(void);

/** @brief ARR do PWM calculado no init (para o cabecalho do boot). */
uint32_t port_motor_get_pwm_arr(void);

/** @brief Ultimo duty comandado (util para telemetria). */
int32_t port_motor_get_duty(motor_id_t motor);
