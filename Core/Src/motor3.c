/**
 ******************************************************************************
 * @file    motor3.c
 * @brief   Controle dos motores TB6612 via PWM (TIM1) e pinos de direcao.
 *          Implementa a logica do fw_micromouse_f411 (deadband, permille duty)
 *          mantendo compatibilidade com as funcoes legadas do app3.c.
 ******************************************************************************
 */
#include "motor3.h"
#include "board.h"
#include "main.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static int32_t motor_duty[MOTOR_COUNT] = { 0, 0 };
static bool    motor_ready = false;

/* -------------------------------------------------------------------------- */
/* Helpers para acionamento TB6612                                            */
/* -------------------------------------------------------------------------- */

static void motor_set_hardware(motor_id_t motor, int32_t permille)
{
    if (permille > 1000)  permille = 1000;
    if (permille < -1000) permille = -1000;

    motor_duty[motor] = permille;

    /* Inversao configuravel por canal */
    int32_t applied = permille;
    if ((motor == MOTOR_LEFT && MOTOR_L_INVERT) || (motor == MOTOR_RIGHT && MOTOR_R_INVERT))
    {
        applied = -applied;
    }

    uint32_t magnitude = (applied < 0) ? (uint32_t)(-applied) : (uint32_t)applied;
    uint32_t ccr = (magnitude * MOTOR_PWM_PERIOD) / 1000u;

    if (motor == MOTOR_LEFT)
    {
        __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_L_PWM_CH, ccr);
        if (applied > 0)
        {
            HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_SET);
        }
        else if (applied < 0)
        {
            HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_RESET);
        }
    }
    else if (motor == MOTOR_RIGHT)
    {
        __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_R_PWM_CH, ccr);
        if (applied > 0)
        {
            HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_SET);
        }
        else if (applied < 0)
        {
            HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_RESET);
        }
    }
}

/**
 * Compensacao de zona morta (deadband) do fw_micromouse_f411:
 * Converte um duty de 1..1000 em deadband..1000 para que o motor
 * responda imediatamente sem ficar travado por atrito estatico.
 */
static int32_t motor_shape_duty(int32_t duty, int32_t deadband)
{
    if (duty > 1000)  duty = 1000;
    if (duty < -1000) duty = -1000;

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
    return duty;
}

/* -------------------------------------------------------------------------- */
/* API port_motor_* (declarada no motor.h; motor_* sao macros para estas)     */
/* -------------------------------------------------------------------------- */

void port_motor_init(void)
{
    motor3_init();
}

void port_motor_set(motor_id_t motor, int32_t duty)
{
    if (motor >= MOTOR_COUNT) return;

    int32_t deadband = (motor == MOTOR_LEFT) ? MOTOR_L_DEADBAND : MOTOR_R_DEADBAND;
    int32_t shaped   = motor_shape_duty(duty, deadband);

    motor_set_hardware(motor, shaped);
}

void port_motor_set_both(int32_t duty_left, int32_t duty_right)
{
    motor_set(MOTOR_LEFT, duty_left);
    motor_set(MOTOR_RIGHT, duty_right);
}

void port_motor_brake(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) return;

    if (motor == MOTOR_LEFT)
    {
        /* TB6612: IN1=IN2=HIGH = freio curto (short brake). Com os dois em
         * LOW o motor fica solto (stop/coast), que era o que acontecia antes. */
        HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_L_PWM_CH, MOTOR_PWM_PERIOD);
    }
    else
    {
        HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_R_PWM_CH, MOTOR_PWM_PERIOD);
    }
    motor_duty[motor] = 0;
}

void port_motor_coast(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) return;

    if (motor == MOTOR_LEFT)
    {
        HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_L_PWM_CH, 0);
    }
    else
    {
        HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_R_PWM_CH, 0);
    }
    motor_duty[motor] = 0;
}

void port_motor_stop_all(void)
{
    motor_parada();
}

int32_t port_motor_get_duty(motor_id_t motor)
{
    return (motor < MOTOR_COUNT) ? motor_duty[motor] : 0;
}

uint32_t port_motor_get_pwm_arr(void)
{
    return MOTOR_PWM_PERIOD;
}


/* -------------------------------------------------------------------------- */
/* Funcoes legadas MicroJM                                                    */
/* -------------------------------------------------------------------------- */

void motor3_init(void)
{
    /* PWM de 20 kHz. O MX_TIM1_Init() do main.c deixa PSC=63 / ARR=19999,
     * que com o clock de 64 MHz da' so' 50 Hz. Reprograma aqui (codigo do
     * usuario, nao e' apagado se o CubeMX gerar o main.c de novo):
     *   64 MHz / (PSC+1) / (ARR+1) = 64e6 / 1 / 3200 = 20 kHz */
    __HAL_TIM_SET_PRESCALER(&BOARD_MOTOR_TIM, 0u);
    __HAL_TIM_SET_AUTORELOAD(&BOARD_MOTOR_TIM, MOTOR_PWM_PERIOD);
    BOARD_MOTOR_TIM.Init.Prescaler = 0u;
    BOARD_MOTOR_TIM.Init.Period    = MOTOR_PWM_PERIOD;
    BOARD_MOTOR_TIM.Instance->EGR  = TIM_EGR_UG;   /* carrega PSC/ARR ja' */

    HAL_TIM_Base_Start(&BOARD_MOTOR_TIM);
    HAL_TIM_PWM_Start(&BOARD_MOTOR_TIM, BOARD_MOTOR_L_PWM_CH);
    HAL_TIM_PWM_Start(&BOARD_MOTOR_TIM, BOARD_MOTOR_R_PWM_CH);

    HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_L_PWM_CH, 0);
    __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_R_PWM_CH, 0);

    /* Encoders */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    motor_ready = true;
}

void motor_parada(void)
{
    HAL_GPIO_WritePin(BOARD_MOTA_IN1_PORT, BOARD_MOTA_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_MOTA_IN2_PORT, BOARD_MOTA_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_MOTB_IN1_PORT, BOARD_MOTB_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_MOTB_IN2_PORT, BOARD_MOTB_IN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_L_PWM_CH, MOTOR_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&BOARD_MOTOR_TIM, BOARD_MOTOR_R_PWM_CH, MOTOR_PWM_PERIOD);

    motor_duty[MOTOR_LEFT]  = 0;
    motor_duty[MOTOR_RIGHT] = 0;
}

void setLeftSpeed(int speed)
{
    int32_t permille = (speed * 1000) / (int32_t)MOTOR_PWM_PERIOD;
    motor_set(MOTOR_LEFT, permille);
}

void setRightSpeed(int speed)
{
    int32_t permille = (speed * 1000) / (int32_t)MOTOR_PWM_PERIOD;
    motor_set(MOTOR_RIGHT, permille);
}

void setSpeeds(int leftSpeed, int rightSpeed)
{
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void setMotorPercent(int leftPercent, int rightPercent)
{
    if (leftPercent > 100)  leftPercent = 100;
    if (leftPercent < -100) leftPercent = -100;
    if (rightPercent > 100)  rightPercent = 100;
    if (rightPercent < -100) rightPercent = -100;

    motor_set(MOTOR_LEFT,  leftPercent * 10);
    motor_set(MOTOR_RIGHT, rightPercent * 10);
}

void motor_forward(int percent)
{
    setMotorPercent(percent, percent);
}

void motor_backward(int percent)
{
    setMotorPercent(-percent, -percent);
}

void motor_spin_left(int percent)
{
    setMotorPercent(-percent, percent);
}

void motor_spin_right(int percent)
{
    setMotorPercent(percent, -percent);
}
