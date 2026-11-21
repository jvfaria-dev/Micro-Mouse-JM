/**
 ******************************************************************************
 * @file    motor.h
 * @brief   Driver dos motores TB6612 para o MicroJM (Blue Pill).
 *          Implementa a API port_motor_... usada por app do fw_micromouse_f411.
 ******************************************************************************
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "board.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
} motor_id_t;

/** @brief Liga os canais de PWM e deixa os motores parados (coast). */
void port_motor_init(void);

/** @brief Define o esforco de um motor (-1000..+1000). */
void port_motor_set(motor_id_t motor, int32_t duty);

/** @brief Aplica o esforco nos dois motores simultaneamente. */
void port_motor_set_both(int32_t duty_left, int32_t duty_right);

/** @brief Frenagem ativa. */
void port_motor_brake(motor_id_t motor);

/** @brief Roda livre (coast). */
void port_motor_coast(motor_id_t motor);

/** @brief Coloca ambos os motores em roda livre. */
void port_motor_stop_all(void);

/** @brief ARR do PWM do timer de motores. */
uint32_t port_motor_get_pwm_arr(void);

/** @brief Ultimo duty comandado. */
int32_t port_motor_get_duty(motor_id_t motor);

/* Aliases para facilitar compatibilidade */
#define motor_init()                    port_motor_init()
#define motor_set(m, d)                 port_motor_set(m, d)
#define motor_set_both(l, r)            port_motor_set_both(l, r)
#define motor_brake(m)                  port_motor_brake(m)
#define motor_coast(m)                  port_motor_coast(m)
#define motor_stop_all()                port_motor_stop_all()
#define motor_get_duty(m)               port_motor_get_duty(m)

#endif /* MOTOR_H */

