/**
 ******************************************************************************
 * @file    imu.c
 * @brief   MPU-6050 pelo I2C3 -- giroscopio Z.
 ******************************************************************************
 */

#include "imu.h"
#include "board.h"

extern I2C_HandleTypeDef BOARD_IMU_I2C;

/* Registradores do MPU-6050 */
#define REG_SMPLRT_DIV      0x19u
#define REG_CONFIG          0x1Au
#define REG_GYRO_CONFIG     0x1Bu
#define REG_GYRO_ZOUT_H     0x47u
#define REG_PWR_MGMT_1      0x6Bu
#define REG_WHO_AM_I        0x75u

#define IMU_I2C_ADDR        ((uint16_t)(BOARD_IMU_ADDR << 1))
#define IMU_TIMEOUT_MS      5u

static bool     imu_ok      = false;
static uint8_t  imu_who     = 0u;
static int16_t  bias_raw    = 0;
static float    bias_f      = 0.0f;     /* offset em uso (fracionario)   */
static float    bias_cal    = 0.0f;     /* offset medido na calibracao   */
static int16_t  cal_spread  = 0;        /* max - min na ultima calibracao */
static volatile bool still  = false;    /* robo parado: re-mede o offset  */
static float    rate_dps    = 0.0f;
static float    heading_deg = 0.0f;
static uint32_t errors      = 0u;

/* -------------------------------------------------------------------------- */

static bool reg_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&BOARD_IMU_I2C, IMU_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1u,
                             IMU_TIMEOUT_MS) == HAL_OK;
}

static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&BOARD_IMU_I2C, IMU_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            IMU_TIMEOUT_MS) == HAL_OK;
}

static bool read_gz_raw(int16_t *out)
{
    uint8_t b[2];

    if (!reg_read(REG_GYRO_ZOUT_H, b, 2u))
    {
        errors++;
        return false;
    }

    int16_t v = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
#if (IMU_GZ_INVERT != 0)
    v = (int16_t)-v;
#endif
    *out = v;
    return true;
}

/* --------------------------------------------------------------------------
 * Destrava o barramento I2C.
 *
 * Se a placa for resetada no meio de uma leitura, o MPU-6050 (que nao foi
 * resetado) pode ficar segurando o SDA em zero, esperando clocks que nunca
 * chegam. O STM32 ve' o barramento "ocupado" e o chip "nao responde" ate'
 * desligar a bateria. A receita padrao: ate' 9 pulsos de clock na mao ate' o
 * SDA soltar, uma condicao de STOP, e o periferico reiniciado do zero.
 * ------------------------------------------------------------------------ */
#ifndef BOARD_IMU_SCL_PORT
#define BOARD_IMU_SCL_PORT      GPIOA
#define BOARD_IMU_SCL_PIN       GPIO_PIN_8
#define BOARD_IMU_SDA_PORT      GPIOB
#define BOARD_IMU_SDA_PIN       GPIO_PIN_4
#endif

static void bus_delay(void)
{
    for (volatile uint32_t i = 0; i < 400u; i++) { }    /* ~10 us */
}

static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};

    (void)HAL_I2C_DeInit(&BOARD_IMU_I2C);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = BOARD_IMU_SCL_PIN;
    HAL_GPIO_WritePin(BOARD_IMU_SCL_PORT, BOARD_IMU_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(BOARD_IMU_SCL_PORT, &g);
    g.Pin   = BOARD_IMU_SDA_PIN;
    HAL_GPIO_WritePin(BOARD_IMU_SDA_PORT, BOARD_IMU_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(BOARD_IMU_SDA_PORT, &g);
    bus_delay();

    /* clocks ate' o escravo soltar o SDA */
    for (int i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(BOARD_IMU_SDA_PORT, BOARD_IMU_SDA_PIN) == GPIO_PIN_SET)
        {
            break;
        }
        HAL_GPIO_WritePin(BOARD_IMU_SCL_PORT, BOARD_IMU_SCL_PIN, GPIO_PIN_RESET);
        bus_delay();
        HAL_GPIO_WritePin(BOARD_IMU_SCL_PORT, BOARD_IMU_SCL_PIN, GPIO_PIN_SET);
        bus_delay();
    }

    /* STOP: SDA sobe com SCL alto */
    HAL_GPIO_WritePin(BOARD_IMU_SDA_PORT, BOARD_IMU_SDA_PIN, GPIO_PIN_RESET);
    bus_delay();
    HAL_GPIO_WritePin(BOARD_IMU_SCL_PORT, BOARD_IMU_SCL_PIN, GPIO_PIN_SET);
    bus_delay();
    HAL_GPIO_WritePin(BOARD_IMU_SDA_PORT, BOARD_IMU_SDA_PIN, GPIO_PIN_SET);
    bus_delay();

    /* reset por software do periferico (BUSY travado, errata do F4) */
    BOARD_IMU_I2C.Instance->CR1 |=  I2C_CR1_SWRST;
    BOARD_IMU_I2C.Instance->CR1 &= ~I2C_CR1_SWRST;

    /* HAL_I2C_Init chama o MspInit, que devolve os pinos ao I2C */
    (void)HAL_I2C_Init(&BOARD_IMU_I2C);
}

static uint8_t init_tries = 0u;

uint8_t port_imu_init_tries(void)
{
    return init_tries;
}

/* -------------------------------------------------------------------------- */

bool port_imu_init(void)
{
    imu_ok = false;

    /* O chip responde no endereco? Se nao, destrava o barramento e tenta de
     * novo -- ate' 3 vezes. */
    init_tries = 0u;
    bool ready = false;
    for (uint8_t t = 0u; t < 3u && !ready; t++)
    {
        init_tries++;
        if (t > 0u)
        {
            i2c_bus_recover();
            HAL_Delay(20u);
        }
        ready = (HAL_I2C_IsDeviceReady(&BOARD_IMU_I2C, IMU_I2C_ADDR, 3u, 10u) == HAL_OK);
    }
    if (!ready)
    {
        return false;
    }

    if (!reg_read(REG_WHO_AM_I, &imu_who, 1u))
    {
        return false;
    }

    /* Reset, espera, e acorda com o PLL do giroscopio X como relogio --
     * mais estavel que o oscilador interno, recomendacao do datasheet. */
    (void)reg_write(REG_PWR_MGMT_1, 0x80u);
    HAL_Delay(100u);

    bool ok = true;
    ok &= reg_write(REG_PWR_MGMT_1,  0x01u);    /* acorda, clock = PLL gyro X */
    HAL_Delay(10u);
    ok &= reg_write(REG_CONFIG,      0x03u);    /* filtro passa-baixa ~44 Hz  */
    ok &= reg_write(REG_SMPLRT_DIV,  0x04u);    /* 1 kHz / (1+4) = 200 Hz     */
    ok &= reg_write(REG_GYRO_CONFIG, 0x08u);    /* +-500 graus/s              */

    imu_ok      = ok;
    bias_raw    = 0;
    heading_deg = 0.0f;
    rate_dps    = 0.0f;
    return ok;
}

bool port_imu_ok(void)
{
    return imu_ok;
}

uint8_t port_imu_whoami(void)
{
    return imu_who;
}

bool port_imu_calibrate(uint32_t samples)
{
    if (!imu_ok || samples == 0u)
    {
        return false;
    }

    int32_t  sum = 0;
    uint32_t n   = 0u;
    int16_t  vmin = 32767, vmax = -32768;

    for (uint32_t i = 0; i < samples; i++)
    {
        int16_t v;
        if (read_gz_raw(&v))
        {
            sum += v;
            n++;
            if (v < vmin) { vmin = v; }
            if (v > vmax) { vmax = v; }
        }
        HAL_Delay(1u);
    }

    bias_raw    = (n > 0u) ? (int16_t)(sum / (int32_t)n) : 0;
    bias_f      = (float)bias_raw;
    bias_cal    = bias_f;
    cal_spread  = (n > 0u) ? (int16_t)(vmax - vmin) : 32767;
    heading_deg = 0.0f;
    /* parado de verdade o ruido fica em poucas dezenas; mao encostada no
     * robo passa de centenas (13o teste: offset -212 em vez de -27) */
    return (n > samples / 2u) && (cal_spread <= IMU_CAL_MAX_SPREAD);
}

int16_t port_imu_get_cal_spread(void)
{
    return cal_spread;
}

void port_imu_set_still(bool s)
{
    still = s;
}

void port_imu_update(uint32_t dt_ms)
{
    if (!imu_ok)
    {
        return;
    }

    int16_t v;
    if (!read_gz_raw(&v))
    {
        return;     /* mantem a ultima velocidade; o erro fica contado */
    }

    if (still)
    {
        /* Parado: a velocidade verdadeira e' zero. O que o giroscopio mede
         * e' o offset -- re-mede devagar e nao integra o rumo. */
        /* so' re-mede se a leitura parece de robo parado (17o teste: o
         * offset foi de -34 para -65 -- o robo balancava/encostava) e sem
         * se afastar demais da calibracao */
        float d = (float)v - bias_f;
        if (d > -IMU_ZUPT_GATE && d < IMU_ZUPT_GATE)
        {
            bias_f += d * IMU_ZUPT_ALPHA;
            if (bias_f > bias_cal + IMU_ZUPT_MAX_DRIFT) { bias_f = bias_cal + IMU_ZUPT_MAX_DRIFT; }
            if (bias_f < bias_cal - IMU_ZUPT_MAX_DRIFT) { bias_f = bias_cal - IMU_ZUPT_MAX_DRIFT; }
            bias_raw = (int16_t)bias_f;
        }
        rate_dps = 0.0f;
        return;
    }

    rate_dps = ((float)v - bias_f) / IMU_GYRO_LSB_PER_DPS;
    heading_deg += rate_dps * ((float)dt_ms / 1000.0f);
}

float port_imu_get_rate_dps(void)
{
    return rate_dps;
}

float port_imu_get_heading_deg(void)
{
    return heading_deg;
}

void port_imu_reset_heading(void)
{
    heading_deg = 0.0f;
}

int16_t port_imu_get_bias_raw(void)
{
    return bias_raw;
}

uint32_t port_imu_get_errors(void)
{
    return errors;
}
