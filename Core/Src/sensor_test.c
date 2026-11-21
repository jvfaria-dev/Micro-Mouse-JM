/**
 ******************************************************************************
 * @file    sensor_test.c
 * @brief   Modo de teste unificado para validacao de todos os perifericos:
 *          - 5 Sensores Sharp IR (ADC1)
 *          - IMU MPU-6050 (I2C1)
 *          - 2 Encoders em Quadratura (TIM2 e TIM3)
 *          - Controle de Motores TB6612 (TIM1 PWM)
 *
 * Baseado na arquitetura e logica do fw_micromouse_f411.
 ******************************************************************************
 */

#include "sensor_test.h"
#include "board.h"
#include "sensors.h"
#include "encoder.h"
#include "motor3.h"
#include "imu.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Variaveis Globais para Live Expressions no STM32CubeIDE
 * ========================================================================== */

/* 1. Sensores Sharp IR (5 canais: SL1_DIR, SL2_ESQ, SL3_BORDA_D, SL4_BORDA_E, SL5_CENTRO) */
volatile uint16_t adc_sharp_raw[SENSOR_COUNT];     /* Leitura crua (0..4095) */
volatile uint32_t adc_sharp_mv[SENSOR_COUNT];      /* Tensao em mV (0..3300) */
volatile int16_t  adc_sharp_prox[SENSOR_COUNT];    /* Proximidade calibrada (0..1000+) */

/* 2. Encoders (TIM2 esquerdo, TIM3 direito) */
volatile int32_t  enc_ticks_l   = 0;                /* Ticks acumulados */
volatile int32_t  enc_ticks_r   = 0;
volatile int32_t  enc_delta_l   = 0;                /* Ticks na ultima janela */
volatile int32_t  enc_delta_r   = 0;
volatile float    enc_rpm_l     = 0.0f;             /* Velocidade em RPM */
volatile float    enc_rpm_r     = 0.0f;
volatile float    enc_dist_l_mm = 0.0f;             /* Distancia percorrida em mm */
volatile float    enc_dist_r_mm = 0.0f;

/* 3. IMU MPU-6050 */
volatile bool     imu_ok        = false;            /* true se comunicou no I2C1 */
volatile float    imu_rate_z    = 0.0f;             /* Velocidade angular Z em °/s */
volatile float    imu_heading   = 0.0f;             /* Angulo de rumo (heading) em graus */
volatile uint8_t  imu_who_val   = 0;                /* WHO_AM_I (0x68) */

/* 4. Contador de ciclos de teste */
volatile uint32_t test_cycles   = 0;

/* Limiares */
#define DETECT_THRESHOLD_MV     2500u               /* 2500 mV para deteccao de dedo perto */
#define DETECT_THRESHOLD_DEG    20.0f               /* 20 graus de desvio na IMU */

/* -------------------------------------------------------------------------- */
void sensor_test_init(void)
{
    /* 1. Inicializa Motores (parados em frenagem ativa) */
    motor_init();
    motor_parada();

    /* 2. Inicializa Encoders (TIM2 e TIM3 em quadratura x4) */
    encoder_init();

    /* 3. Inicializa Sensores IR (5 canais ADC1) */
    sensors_init();

    /* 4. Inicializa IMU MPU-6050 (I2C1) */
    imu_ok = imu_init();
    if (imu_ok)
    {
        imu_who_val = imu_whoami();
        imu_reset_heading();
    }

    /* 5. Sinal visual de boot: pisca LED_1 e LED_2 3 vezes */
    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BOARD_LED2_PORT, BOARD_LED2_PIN, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BOARD_LED2_PORT, BOARD_LED2_PIN, GPIO_PIN_RESET);
        HAL_Delay(150);
    }
}

/* -------------------------------------------------------------------------- */
void sensor_test_loop(void)
{
    /* 1. Atualiza Sensores Sharp IR */
    sensors_update();
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_sharp_raw[i]  = sensors_get_raw((sensor_id_t)i);
        adc_sharp_mv[i]   = sensors_get_mv((sensor_id_t)i);
        adc_sharp_prox[i] = sensors_get_prox((sensor_id_t)i);
    }

    /* 2. Atualiza Encoders */
    encoder_update();
    enc_ticks_l   = encoder_get_ticks(ENCODER_LEFT);
    enc_ticks_r   = encoder_get_ticks(ENCODER_RIGHT);
    enc_delta_l   = encoder_get_delta(ENCODER_LEFT);
    enc_delta_r   = encoder_get_delta(ENCODER_RIGHT);
    enc_rpm_l     = encoder_get_rpm(ENCODER_LEFT);
    enc_rpm_r     = encoder_get_rpm(ENCODER_RIGHT);
    enc_dist_l_mm = encoder_get_distance_mm(ENCODER_LEFT);
    enc_dist_r_mm = encoder_get_distance_mm(ENCODER_RIGHT);

    /* 3. Atualiza IMU MPU-6050 (dt = 50 ms) */
    if (imu_ok)
    {
        imu_update(50u);
        imu_rate_z  = imu_get_rate_dps();
        imu_heading = imu_get_heading_deg();
    }

    /* 4. Feedback nos LEDs (Sharp IR + Angulo IMU > 20°) */

    /* Desvio angular:
     * - Girou > +20° (esquerda): angulo_esq = true
     * - Girou < -20° (direita):  angulo_dir = true
     */
    bool angulo_esq = (imu_ok && (imu_heading > DETECT_THRESHOLD_DEG));
    bool angulo_dir = (imu_ok && (imu_heading < -DETECT_THRESHOLD_DEG));

    /* LED_1 (PA1): Acende se Sharp Esquerdo (SL2) detectar objeto (> 2500 mV)
     *              OU se o angulo da IMU for > 20° para a esquerda */
    if ((adc_sharp_mv[SENSOR_SL2] > DETECT_THRESHOLD_MV) || angulo_esq)
    {
        HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);
    }

    /* LED_2 (PA0): Acende se Sharp Direito (SL1) detectar objeto (> 2500 mV)
     *              OU se o angulo da IMU for > 20° para a direita */
    if ((adc_sharp_mv[SENSOR_SL1] > DETECT_THRESHOLD_MV) || angulo_dir)
    {
        HAL_GPIO_WritePin(BOARD_LED2_PORT, BOARD_LED2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(BOARD_LED2_PORT, BOARD_LED2_PIN, GPIO_PIN_RESET);
    }

    test_cycles++;
    HAL_Delay(50);
}
