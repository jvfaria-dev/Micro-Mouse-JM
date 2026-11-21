/**
 ******************************************************************************
 * @file    imu_test.c
 * @brief   Teste do MPU-6050.
 ******************************************************************************
 */

#include "imu_test.h"
#include "board.h"
#include "imu.h"
#include "motor.h"
#include "debug.h"
#include "ihm.h"

#define IMU_TEST_CAL_SAMPLES    2000u

void imu_test_init(void)
{
    port_motor_stop_all();

    port_debug_printf("\r\n=== TESTE DA IMU (MPU-6050 no I2C3: SCL=PA8, SDA=PB4) ===\r\n");

    if (!port_imu_ok())
    {
        /* tenta de novo: o init do boot pode ter pego o modulo ainda
         * ligando, se a alimentacao dele subiu depois da Black Pill */
        (void)port_imu_init();
    }

    if (!port_imu_ok())
    {
        port_debug_printf("[!!] a IMU NAO respondeu no endereco 0x%02X.\r\n",
                          (unsigned)BOARD_IMU_ADDR);
        port_debug_printf("     Confira, nesta ordem:\r\n");
        port_debug_printf("       - VCC do modulo no 3V3 e GND no GND\r\n");
        port_debug_printf("       - SCL no PA8 e SDA no PB4 (nao invertidos)\r\n");
        port_debug_printf("       - AD0 no GND (com AD0 no 3V3 o endereco vira 0x69)\r\n");
        port_ihm_led_blink(150u);   /* pisca rapido = erro */
        return;
    }

    port_debug_printf("WHO_AM_I = 0x%02X %s\r\n", (unsigned)port_imu_whoami(),
                      (port_imu_whoami() == 0x68u) ? "(MPU-6050 original)"
                                                   : "(clone -- funciona igual)");
    port_debug_printf("Deixe o robo PARADO: medindo o offset por %lu s...\r\n",
                      (unsigned long)(IMU_TEST_CAL_SAMPLES / 1000u));
    port_debug_flush();

    port_imu_calibrate(IMU_TEST_CAL_SAMPLES);

    port_debug_printf("offset = %d LSB (%ld mgraus/s)\r\n",
                      (int)port_imu_get_bias_raw(),
                      (long)((float)port_imu_get_bias_raw() * 1000.0f
                             / IMU_GYRO_LSB_PER_DPS));
    port_debug_printf("Pronto. Gire 90 graus para a ESQUERDA: o angulo deve ir a +90.\r\n");
    port_debug_printf("Para a DIREITA: -90. Mande qualquer tecla para zerar o angulo.\r\n\r\n");

    port_ihm_led_blink(1000u);
}

void imu_test_update(void)
{
    port_imu_update(SENSOR_PERIOD_MS);
}

void imu_test_print(void)
{
    if (!port_imu_ok())
    {
        return;
    }

    if (port_debug_key())
    {
        port_imu_reset_heading();
        port_debug_printf("--- angulo zerado ---\r\n");
    }

    /* decimos de grau, sem %f. O sinal vai a' parte: -0,5 dividido por
     * 10 da' 0 e perderia o menos. */
    long r = (long)(port_imu_get_rate_dps() * 10.0f);
    long h = (long)(port_imu_get_heading_deg() * 10.0f);
    char rs = (r < 0) ? '-' : '+';
    char hs = (h < 0) ? '-' : '+';
    if (r < 0) { r = -r; }
    if (h < 0) { h = -h; }

    port_debug_printf("gz=%c%3ld.%01ld graus/s   angulo=%c%4ld.%01ld graus   erros I2C=%lu\r\n",
                      rs, r / 10, r % 10, hs, h / 10, h % 10,
                      (unsigned long)port_imu_get_errors());
}
