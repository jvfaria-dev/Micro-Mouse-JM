/**
 ******************************************************************************
 * @file    imu.c
 * @brief   Driver do MPU-6050 via I2C1 (PB6=SCL, PB7=SDA).
 *          Implementa recuperacao do barramento I2C, calibracao de bias,
 *          filtro passa-baixa e integracao ZUPT do fw_micromouse_f411.
 ******************************************************************************
 */
#include "imu.h"
#include "board.h"
#include "main.h"
#include <math.h>
#include <string.h>

extern I2C_HandleTypeDef BOARD_IMU_I2C;

#define REG_SMPLRT_DIV      0x19u
#define REG_CONFIG          0x1Au
#define REG_GYRO_CONFIG     0x1Bu
#define REG_ACCEL_CONFIG    0x1Cu
#define REG_INT_ENABLE      0x38u
#define REG_ACCEL_XOUT_H    0x3Bu
#define REG_GYRO_XOUT_H     0x43u
#define REG_GYRO_ZOUT_H     0x47u
#define REG_PWR_MGMT_1      0x6Bu
#define REG_WHO_AM_I        0x75u

#define IMU_I2C_ADDR_HAL    ((uint16_t)(BOARD_IMU_ADDR << 1))
#define IMU_TIMEOUT_MS      5u

static bool     imu_is_ok   = false;
static uint8_t  imu_who     = 0u;
static int16_t  bias_raw    = 0;
static float    bias_f      = 0.0f;
static float    bias_cal    = 0.0f;
static int16_t  cal_spread  = 0;
static volatile bool still  = false;
static float    rate_dps    = 0.0f;
static float    heading_deg = 0.0f;
static uint32_t errors      = 0u;
static uint8_t  init_tries  = 0u;

/* -------------------------------------------------------------------------- */
/* Comunicacao I2C Basica                                                     */
/* -------------------------------------------------------------------------- */

static bool reg_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&BOARD_IMU_I2C, IMU_I2C_ADDR_HAL, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1u,
                             IMU_TIMEOUT_MS) == HAL_OK;
}

static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&BOARD_IMU_I2C, IMU_I2C_ADDR_HAL, reg,
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

/* -------------------------------------------------------------------------- */
/* Recuperacao de travamento do barramento I2C                                */
/* -------------------------------------------------------------------------- */

static void bus_delay(void)
{
    for (volatile uint32_t i = 0; i < 400u; i++) { }
}

static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};

    (void)HAL_I2C_DeInit(&BOARD_IMU_I2C);

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

    /* Pulsa SCL ate' o MPU-6050 soltar o SDA (maximo 9 clocks) */
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

    /* Envia condicao de STOP */
    HAL_GPIO_WritePin(BOARD_IMU_SDA_PORT, BOARD_IMU_SDA_PIN, GPIO_PIN_RESET);
    bus_delay();
    HAL_GPIO_WritePin(BOARD_IMU_SCL_PORT, BOARD_IMU_SCL_PIN, GPIO_PIN_SET);
    bus_delay();
    HAL_GPIO_WritePin(BOARD_IMU_SDA_PORT, BOARD_IMU_SDA_PIN, GPIO_PIN_SET);
    bus_delay();

    /* Reinicializa o periferico I2C */
    BOARD_IMU_I2C.Instance->CR1 |=  I2C_CR1_SWRST;
    BOARD_IMU_I2C.Instance->CR1 &= ~I2C_CR1_SWRST;

    (void)HAL_I2C_Init(&BOARD_IMU_I2C);
}

/* -------------------------------------------------------------------------- */
/* API Publica                                                                */
/* -------------------------------------------------------------------------- */

bool imu_init(void)
{
    imu_is_ok = false;

    /* Tenta conectar (ate 3 tentativas com bus recovery) */
    bool ready = false;
    for (uint8_t t = 0u; t < 3u && !ready; t++)
    {
        init_tries = t + 1u;
        if (t > 0u)
        {
            i2c_bus_recover();
            HAL_Delay(20u);
        }
        ready = (HAL_I2C_IsDeviceReady(&BOARD_IMU_I2C, IMU_I2C_ADDR_HAL, 3u, 10u) == HAL_OK);
    }

    if (!ready) return false;

    if (!reg_read(REG_WHO_AM_I, &imu_who, 1u)) return false;
    if (imu_who != 0x68u && imu_who != 0x70u && imu_who != 0x72u && imu_who != 0x98u)
    {
        return false;
    }

    /* Reset e acorda com o PLL do Gyro X */
    (void)reg_write(REG_PWR_MGMT_1, 0x80u);
    HAL_Delay(100u);

    bool ok = true;
    ok &= reg_write(REG_PWR_MGMT_1,  0x01u);    /* PLL com Gyro X */
    HAL_Delay(10u);
    ok &= reg_write(REG_CONFIG,      0x03u);    /* Filtro passa-baixa ~44 Hz */
    ok &= reg_write(REG_SMPLRT_DIV,  0x04u);    /* 200 Hz */
    ok &= reg_write(REG_GYRO_CONFIG, 0x08u);    /* +-500 graus/s */

    imu_is_ok   = ok;
    bias_raw    = 0;
    heading_deg = 0.0f;
    rate_dps    = 0.0f;

    return ok;
}

bool imu_ok_status(void)
{
    return imu_is_ok;
}

uint8_t imu_whoami(void)
{
    return imu_who;
}

bool imu_calibrate(uint32_t samples)
{
    if (!imu_is_ok || samples == 0u) return false;

    int32_t sum = 0;
    uint32_t n = 0u;
    int16_t vmin = 32767, vmax = -32768;

    for (uint32_t i = 0; i < samples; i++)
    {
        int16_t v;
        if (read_gz_raw(&v))
        {
            sum += v;
            n++;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
        HAL_Delay(1u);
    }

    bias_raw    = (n > 0u) ? (int16_t)(sum / (int32_t)n) : 0;
    bias_f      = (float)bias_raw;
    bias_cal    = bias_f;
    cal_spread  = (n > 0u) ? (int16_t)(vmax - vmin) : 32767;
    heading_deg = 0.0f;

    return (n > samples / 2u) && (cal_spread <= IMU_CAL_MAX_SPREAD);
}

void imu_set_still(bool s)
{
    still = s;
}

void imu_update(uint32_t dt_ms)
{
    if (!imu_is_ok) return;

    int16_t v;
    if (!read_gz_raw(&v)) return;

    if (still)
    {
        float d = (float)v - bias_f;
        if (d > -IMU_ZUPT_GATE && d < IMU_ZUPT_GATE)
        {
            bias_f += d * IMU_ZUPT_ALPHA;
            if (bias_f > bias_cal + IMU_ZUPT_MAX_DRIFT) bias_f = bias_cal + IMU_ZUPT_MAX_DRIFT;
            if (bias_f < bias_cal - IMU_ZUPT_MAX_DRIFT) bias_f = bias_cal - IMU_ZUPT_MAX_DRIFT;
            bias_raw = (int16_t)bias_f;
        }
        rate_dps = 0.0f;
        return;
    }

    rate_dps = ((float)v - bias_f) / IMU_GYRO_LSB_PER_DPS;
    heading_deg += rate_dps * ((float)dt_ms / 1000.0f);
}

float imu_get_rate_dps(void)
{
    return rate_dps;
}

float imu_get_heading_deg(void)
{
    return heading_deg;
}

void imu_reset_heading(void)
{
    heading_deg = 0.0f;
}

void imu_reset_yaw(void)
{
    heading_deg = 0.0f;
}

/* Compatibilidade com leitura de todos os eixos */
void imu_get_raw(int16_t raw[6])
{
    uint8_t buf[14];
    if (!imu_is_ok || !reg_read(REG_ACCEL_XOUT_H, buf, 14u))
    {
        memset(raw, 0, sizeof(int16_t) * 6);
        return;
    }

    raw[0] = (int16_t)((buf[0] << 8) | buf[1]);   /* ax */
    raw[1] = (int16_t)((buf[2] << 8) | buf[3]);   /* ay */
    raw[2] = (int16_t)((buf[4] << 8) | buf[5]);   /* az */
    raw[3] = (int16_t)((buf[8] << 8) | buf[9]);   /* gx */
    raw[4] = (int16_t)((buf[10] << 8) | buf[11]); /* gy */
    raw[5] = (int16_t)((buf[12] << 8) | buf[13]); /* gz */
}

void imu_get_data(imu_data_t *out)
{
    if (!imu_is_ok) { memset(out, 0, sizeof(*out)); return; }

    int16_t raw[6];
    imu_get_raw(raw);

    out->ax  = (float)raw[0] / 16384.0f;
    out->ay  = (float)raw[1] / 16384.0f;
    out->az  = (float)raw[2] / 16384.0f;
    out->gx  = (float)raw[3] / IMU_GYRO_LSB_PER_DPS;
    out->gy  = (float)raw[4] / IMU_GYRO_LSB_PER_DPS;
    out->gz  = rate_dps;
    out->yaw = heading_deg;
}

uint8_t imu_init_tries(void)
{
    return init_tries;
}

uint32_t imu_get_errors(void)
{
    return errors;
}

int16_t imu_get_bias_raw(void)
{
    return bias_raw;
}

int16_t imu_get_cal_spread(void)
{
    return cal_spread;
}
