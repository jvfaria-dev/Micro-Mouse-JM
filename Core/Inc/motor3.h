/**
 ******************************************************************************
 * @file    motor3.h
 * @brief   Controle dos motores TB6612 via PWM (TIM1) e pinos de direcao.
 *          Inclui tanto a API classica do MicroJM quanto a logica do fw_micromouse_f411.
 ******************************************************************************
 */
#ifndef MOTOR3_H_
#define MOTOR3_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* motor_id_t e a API port_motor_* vem do motor.h (motor_init(), motor_set()...
 * sao macros para port_motor_*). Antes os dois headers declaravam a mesma
 * coisa e o main.c nao compilava. */
#include "motor.h"

/* --- API classica MicroJM (compatibilidade total com app3.c) --- */
void motor3_init(void);
void motor_parada(void);
void setLeftSpeed(int speed);
void setRightSpeed(int speed);
void setSpeeds(int leftSpeed, int rightSpeed);

void setMotorPercent(int leftPercent, int rightPercent);
void motor_forward(int percent);
void motor_backward(int percent);
void motor_spin_left(int percent);
void motor_spin_right(int percent);

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define encoder_esq_read()  ((int16_t)__HAL_TIM_GET_COUNTER(&htim2))
#define encoder_dir_read()  ((int16_t)__HAL_TIM_GET_COUNTER(&htim3))
#define encoder_esq_reset() (__HAL_TIM_SET_COUNTER(&htim2, 0))
#define encoder_dir_reset() (__HAL_TIM_SET_COUNTER(&htim3, 0))

static inline void init(void)
{
    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        motor3_init();
    }
}

#endif /* MOTOR3_H_ */
