/**
 ******************************************************************************
 * @file    imu.h
 * @brief   MPU-6050 (modulo GY-521) no I2C3: so' o giroscopio do eixo Z.
 *
 * Para um micromouse o que interessa da IMU e' UMA coisa: quanto o robo
 * girou em torno do eixo vertical. E' isso que o encoder mede mal quando a
 * roda escorrega no giro. Acelerometro e os outros dois eixos ficam de fora.
 *
 * Ligacao (GY-521):
 *      VCC -> 3V3      GND -> GND
 *      SCL -> PA8      SDA -> PB4
 *      AD0 -> GND (endereco 0x68)      INT, XDA, XCL -> soltos
 *
 * Sinal: positivo = girando para a ESQUERDA (anti-horario visto de cima),
 * o mesmo sentido do heading do speed_ctl. Se a placa estiver montada de
 * cabeca para baixo, troque IMU_GZ_INVERT.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* 1 se a IMU estiver montada de cabeca para baixo (chip virado para o chao). */
#ifndef IMU_GZ_INVERT
#define IMU_GZ_INVERT       0
#endif

/** Faixa do giroscopio: +-500 graus/s. Um micromouse lento nao passa de
 *  ~200 graus/s girando no lugar, entao sobra margem sem perder resolucao. */
/* Nominal 65.5 (+-3% de tolerancia do MPU-6050). 24o teste: a correcao de
 * rumo pelas paredes (h no log) seguiu o rumo total: h16 com -360 graus,
 * h-5 com +270 -> o giroscopio le ~3,4% A MENOS (cada 90 virava ~93 e o
 * robo saia torto das curvas). 65.5 * 0.967 = 63.3. */
#define IMU_GYRO_LSB_PER_DPS    63.3f

/** Calibracao: max - min aceito com o robo parado (cru). Ruido parado ~20-40. */
#define IMU_CAL_MAX_SPREAD      120

/** Re-medicao do offset parado: por leitura a 200 Hz (~0,17 s de constante;
 *  cada parada para ler paredes dura ~0,15 s). */
#define IMU_ZUPT_ALPHA          0.03f

/** Porteira do ZUPT: leitura a mais de isto do offset (cru, ~0,6 grau/s) nao
 *  e' robo parado -- ignora. E o offset nao se afasta mais que MAX_DRIFT da
 *  calibracao (parado de verdade ele quase nao muda: -27 -> -23). */
#define IMU_ZUPT_GATE           40.0f
#define IMU_ZUPT_MAX_DRIFT      15.0f

/**
 * @brief Acorda o MPU-6050 e configura giroscopio e filtro.
 * @return true se o chip respondeu. Se der false, a IMU fica desligada e
 *         as outras funcoes devolvem zero -- o robo continua andando sem ela.
 */
bool port_imu_init(void);

/** @brief Quantas tentativas o ultimo init precisou (1 = de primeira;
 *         mais que 1 = o barramento estava travado e foi destravado). */
uint8_t port_imu_init_tries(void);

/** @brief true se o init deu certo. */
bool port_imu_ok(void);

/** @brief Valor do registrador WHO_AM_I (0x68 no MPU-6050 original;
 *         clones costumam dar 0x70, 0x72 ou 0x98 -- funcionam igual). */
uint8_t port_imu_whoami(void);

/**
 * @brief Mede o offset do giroscopio. O robo TEM que estar parado.
 * @param samples quantas leituras (a 1 ms cada); 1000 = 1 segundo.
 */
bool port_imu_calibrate(uint32_t samples);

/** Variacao (max - min, cru) da ultima calibracao: robo mexido = grande. */
int16_t port_imu_get_cal_spread(void);

/**
 * @brief Avisa que o robo esta' PARADO de verdade (rodas sem girar). Enquanto
 *        true, o offset e' re-medido devagar e o rumo nao integra (ZUPT).
 */
void port_imu_set_still(bool s);

/**
 * @brief Le' o giroscopio e integra o angulo.
 * @param dt_ms tempo desde a ultima chamada.
 * Leva ~0,3 ms (transacao I2C a 100 kHz). Chamar de 5 em 5 ms basta.
 */
void port_imu_update(uint32_t dt_ms);

/** @brief Velocidade angular em Z, graus/s, ja' sem o offset. */
float port_imu_get_rate_dps(void);

/** @brief Angulo integrado desde o ultimo reset, em graus. */
float port_imu_get_heading_deg(void);

/** @brief Zera o angulo integrado. */
void port_imu_reset_heading(void);

/** @brief Offset medido na calibracao, em LSB crus (diagnostico). */
int16_t port_imu_get_bias_raw(void);

/** @brief Contador de falhas de leitura I2C (diagnostico). */
uint32_t port_imu_get_errors(void);
