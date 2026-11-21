/**
 ******************************************************************************
 * @file    imu_test.h
 * @brief   APP_MODE_IMU_TEST -- confere a ligacao e o sinal do MPU-6050.
 *
 * O que faz:
 *   1. Diz se o chip respondeu no I2C e qual WHO_AM_I devolveu.
 *   2. Mede o offset do giroscopio por 2 s (deixe o robo PARADO).
 *   3. Imprime a 10 Hz a velocidade angular e o angulo acumulado.
 *
 * O teste em si: gire o robo na mao 90 graus para a ESQUERDA e veja o
 * angulo chegar perto de +90. Para a direita, -90. Se der o sinal trocado,
 * ponha IMU_GZ_INVERT 1 no imu.h. Parado por um minuto, o angulo nao deve
 * andar mais que 1 ou 2 graus.
 *
 * Os motores ficam DESLIGADOS neste modo.
 ******************************************************************************
 */
#pragma once

void imu_test_init(void);

/** @brief Chamar a cada SENSOR_PERIOD_MS. */
void imu_test_update(void);

/** @brief Chamar a cada TELEMETRY_PERIOD_MS. */
void imu_test_print(void);
