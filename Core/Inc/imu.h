/**
 ******************************************************************************
 * @file    imu.h
 * @brief   Driver do MPU-6050 via I2C1 (PB6=SCL, PB7=SDA).
 *          Compativel com fw_micromouse_f411 (port_imu_...) e Blue Pill.
 ******************************************************************************
 */
#ifndef IMU_H
#define IMU_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float ax;   /* g */
    float ay;   /* g */
    float az;   /* g */
    float gx;   /* °/s */
    float gy;   /* °/s */
    float gz;   /* °/s */
    float yaw;  /* angulo integrado em graus */
} imu_data_t;

/** @brief Inicializa a IMU com recuperacao automatica do barramento I2C. */
bool     imu_init(void);

/** @brief true se a IMU inicializou com sucesso. */
bool     imu_ok_status(void);

/** @brief Retorna o valor do registrador WHO_AM_I (0x68). */
uint8_t  imu_whoami(void);

/** @brief Calibra o offset do giroscopio com o robo parado na bancada. */
bool     imu_calibrate(uint32_t samples);

/** @brief Informa se o robo esta parado para correcao de offset (ZUPT). */
void     imu_set_still(bool s);

/** @brief Leitura e integracao do rumo (chamar periodicamente). */
void     imu_update(uint32_t dt_ms);

/** @brief Velocidade angular no eixo Z em °/s. */
float    imu_get_rate_dps(void);

/** @brief Angulo integrado (heading/yaw) em graus. */
float    imu_get_heading_deg(void);

/** @brief Zera o angulo integrado de rumo. */
void     imu_reset_heading(void);
void     imu_reset_yaw(void);

/** @brief Quantidade de tentativas de recuperacao no boot. */
uint8_t  imu_init_tries(void);

/** @brief Quantidade de erros I2C detectados. */
uint32_t imu_get_errors(void);

/** @brief Offset medido em LSBs crus. */
int16_t  imu_get_bias_raw(void);

/** @brief Variacao (max-min) do giroscopio na ultima calibracao. */
int16_t  imu_get_cal_spread(void);

/** @brief Leitura de todos os 6 eixos crus e calculados (compatibilidade). */
void     imu_get_raw(int16_t raw[6]);
void     imu_get_data(imu_data_t *out);

/* --- Aliases da API port_imu_... do fw_micromouse_f411 --- */
#define port_imu_init()             imu_init()
#define port_imu_ok()               imu_ok_status()
#define port_imu_whoami()           imu_whoami()
#define port_imu_calibrate(s)       imu_calibrate(s)
#define port_imu_set_still(s)       imu_set_still(s)
#define port_imu_update(dt)         imu_update(dt)
#define port_imu_get_rate_dps()     imu_get_rate_dps()
#define port_imu_get_heading_deg()  imu_get_heading_deg()
#define port_imu_reset_heading()    imu_reset_heading()
#define port_imu_init_tries()       imu_init_tries()
#define port_imu_get_errors()       imu_get_errors()
#define port_imu_get_bias_raw()     imu_get_bias_raw()
#define port_imu_get_cal_spread()   imu_get_cal_spread()

#endif /* IMU_H */
