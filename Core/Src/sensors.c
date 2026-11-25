/**
 ******************************************************************************
 * @file    sensors.c
 * @brief   Leitura e calibracao dos 5 sensores Sharp IR pelo ADC1 (Blue Pill).
 *          Implementa filtro IIR exponencial e interpolacao de proximidade.
 ******************************************************************************
 */
#include "sensors.h"
#include "board.h"
#include "main.h"

extern ADC_HandleTypeDef BOARD_SENSOR_ADC;

typedef struct
{
    uint32_t    channel;
    int32_t     raw_far;
    int32_t     raw_near;
    const char *name;
} sensor_cfg_t;

static sensor_cfg_t sensor_cfg[SENSOR_COUNT] =
{
    [SENSOR_LEFT]    = { BOARD_SENSOR_CH_SL2, SENSOR_L_RAW_FAR,      SENSOR_L_RAW_NEAR,      "ESQ"   },
    [SENSOR_FRONT_L] = { BOARD_SENSOR_CH_SL4, SENSOR_FL_RAW_FAR,     SENSOR_FL_RAW_NEAR,     "FR_E"  },
    [SENSOR_FRONT]   = { BOARD_SENSOR_CH_SL5, SENSOR_CENTER_RAW_FAR, SENSOR_CENTER_RAW_NEAR, "CENTRO"},
    [SENSOR_FRONT_R] = { BOARD_SENSOR_CH_SL3, SENSOR_FR_RAW_FAR,     SENSOR_FR_RAW_NEAR,     "FR_D"  },
    [SENSOR_RIGHT]   = { BOARD_SENSOR_CH_SL1, SENSOR_R_RAW_FAR,      SENSOR_R_RAW_NEAR,      "DIR"   },
};

static uint16_t raw[SENSOR_COUNT];
static int32_t  raw_filt_x1000[SENSOR_COUNT];
static int16_t  prox[SENSOR_COUNT];
static bool     ready = false;

/* -------------------------------------------------------------------------- */
/* Helper para leitura sequencial do ADC                                      */
/* -------------------------------------------------------------------------- */

static uint16_t adc_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};

    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = BOARD_SENSOR_SAMPLETIME;

    if (HAL_ADC_ConfigChannel(&BOARD_SENSOR_ADC, &cfg) != HAL_OK)
    {
        return 0u;
    }

    if (HAL_ADC_Start(&BOARD_SENSOR_ADC) != HAL_OK)
    {
        return 0u;
    }

    uint16_t value = 0u;
    if (HAL_ADC_PollForConversion(&BOARD_SENSOR_ADC, 5u) == HAL_OK)
    {
        value = (uint16_t)HAL_ADC_GetValue(&BOARD_SENSOR_ADC);
    }

    HAL_ADC_Stop(&BOARD_SENSOR_ADC);
    return value;
}

static int16_t raw_to_prox(const sensor_cfg_t *cfg, int32_t value)
{
    int32_t span = cfg->raw_near - cfg->raw_far;
    if (span == 0) return 0;

    int32_t p = ((value - cfg->raw_far) * 1000) / span;

    if (p < 0)    p = 0;
    if (p > 2000) p = 2000;

    return (int16_t)p;
}

/* -------------------------------------------------------------------------- */
/* API Publica                                                                */
/* -------------------------------------------------------------------------- */

void sensors_init(void)
{
    /* O MX_ADC1_Init() deixa o ADC em varredura continua de 6 canais (era o
     * esquema DMA do adc.c). Assim o adc_read_channel() lia sempre a ULTIMA
     * conversao da sequencia (PB1, sensor central) para os 5 sensores.
     * Aqui o ADC passa a converter 1 canal por vez, sob comando. */
    /* PB0 (ADC1_IN8, lateral esquerdo) nao esta' no .ioc, entao o CubeMX nao
     * o configura como analogico. Feito aqui para sobreviver a uma
     * regeracao do codigo. */
    {
        GPIO_InitTypeDef g = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        g.Pin  = GPIO_PIN_0;
        g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &g);
    }

    HAL_ADC_Stop(&BOARD_SENSOR_ADC);
    BOARD_SENSOR_ADC.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    BOARD_SENSOR_ADC.Init.ContinuousConvMode    = DISABLE;
    BOARD_SENSOR_ADC.Init.DiscontinuousConvMode = DISABLE;
    BOARD_SENSOR_ADC.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    BOARD_SENSOR_ADC.Init.NbrOfConversion       = 1;
    (void)HAL_ADC_Init(&BOARD_SENSOR_ADC);
    (void)HAL_ADCEx_Calibration_Start(&BOARD_SENSOR_ADC);  /* recomendado no F1 */

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        raw[i]            = 0u;
        raw_filt_x1000[i] = 0;
        prox[i]           = 0;
    }

    ready = true;

    /* Le algumas amostras para o filtro estabilizar no valor real */
    for (int n = 0; n < 8; n++)
    {
        sensors_update();
    }
}

void sensors_update(void)
{
    if (!ready) return;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        const sensor_cfg_t *cfg = &sensor_cfg[i];

        uint16_t v = adc_read_channel(cfg->channel);
        raw[i] = v;

        int32_t target = (int32_t)v * 1000;

        if (raw_filt_x1000[i] == 0)
        {
            raw_filt_x1000[i] = target;
        }
        else
        {
            raw_filt_x1000[i] += ((target - raw_filt_x1000[i]) * SENSOR_FILTER_ALPHA) / 1000;
        }

        prox[i] = raw_to_prox(cfg, raw_filt_x1000[i] / 1000);
    }
}

uint16_t sensors_get_raw(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? raw[s] : 0u;
}

uint16_t sensors_get_raw_filtered(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? (uint16_t)(raw_filt_x1000[s] / 1000) : 0u;
}

uint32_t sensors_get_mv(sensor_id_t s)
{
    uint32_t v = (uint32_t)sensors_get_raw_filtered(s);
    return (v * 3300UL) / (uint32_t)SENSOR_RAW_MAX;
}

int16_t sensors_get_prox(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? prox[s] : 0;
}

bool sensors_wall(sensor_id_t s)
{
    return (sensors_get_prox(s) > SENSOR_WALL_THRESHOLD);
}

bool sensors_wall_ahead(void)
{
    return (sensors_front() > SENSOR_FRONT_STOP);
}

int16_t sensors_front(void)
{
#if !SENSOR_FRONT_USE_DIAGONALS
    /* So' o central: os diagonais veem as paredes laterais (board.h). */
    return prox[SENSOR_FRONT];
#else
    /* Prioriza o sensor central frontal SL5; se os frontais diagonais estiverem proximos, inclui na media */
    int32_t sum = prox[SENSOR_FRONT];
    int32_t count = 1;

    if (prox[SENSOR_FRONT_L] > SENSOR_WALL_THRESHOLD)
    {
        sum += prox[SENSOR_FRONT_L];
        count++;
    }
    if (prox[SENSOR_FRONT_R] > SENSOR_WALL_THRESHOLD)
    {
        sum += prox[SENSOR_FRONT_R];
        count++;
    }

    return (int16_t)(sum / count);
#endif
}

/* --- Zona morta (24/11) ---------------------------------------------- */
static int16_t  dz_max[SENSOR_COUNT];
static uint32_t dz_ms[SENSOR_COUNT];

void sensors_dead_reset(sensor_id_t s)
{
    if (s < SENSOR_COUNT) { dz_max[s] = 0; dz_ms[s] = 0u; }
}

sensor_zone_t sensors_dead_update(sensor_id_t s, uint32_t dt_ms)
{
    if (s >= SENSOR_COUNT) { return SENSOR_ZONE_OK; }

    int16_t p = prox[s];
    if (p > dz_max[s]) { dz_max[s] = p; }

    /* so' da' para entrar na zona morta vindo do topo da curva */
    if (dz_max[s] >= SENSOR_DEAD_MIN_PEAK && p < dz_max[s] - SENSOR_DEAD_DROP)
    {
        dz_ms[s] += dt_ms;
        if (dz_ms[s] >= SENSOR_DEAD_CONFIRM_MS) { return SENSOR_ZONE_DEAD; }
    }
    else
    {
        dz_ms[s] = 0u;
    }
    return (p >= SENSOR_DEAD_MIN_PEAK) ? SENSOR_ZONE_PEAK : SENSOR_ZONE_OK;
}

int16_t sensors_dead_max(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? dz_max[s] : 0;
}

const char *sensors_zone_name(sensor_zone_t z)
{
    return (z == SENSOR_ZONE_DEAD) ? "MORTA" : (z == SENSOR_ZONE_PEAK) ? "pico" : "ok";
}

int16_t sensors_center_error(void)
{
    bool wl = sensors_wall(SENSOR_LEFT);
    bool wr = sensors_wall(SENSOR_RIGHT);

    int16_t pl = prox[SENSOR_LEFT];
    int16_t pr = prox[SENSOR_RIGHT];

    if (wl && wr)
    {
        return (int16_t)((pr - pl) - SENSOR_CENTER_TRIM);
    }
    if (wr)
    {
        return (int16_t)((pr - 1000) - SENSOR_CENTER_TRIM);
    }
    if (wl)
    {
        return (int16_t)((1000 - pl) - SENSOR_CENTER_TRIM);
    }

    return 0;
}

void sensors_set_cal(sensor_id_t s, int32_t raw_far, int32_t raw_near)
{
    if (s < SENSOR_COUNT && raw_near != raw_far)
    {
        sensor_cfg[s].raw_far  = raw_far;
        sensor_cfg[s].raw_near = raw_near;
    }
}

int32_t sensors_get_cal_far(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? sensor_cfg[s].raw_far : 0;
}

int32_t sensors_get_cal_near(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? sensor_cfg[s].raw_near : 0;
}

const char *sensors_name(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? sensor_cfg[s].name : "?";
}

/* Compatibilidade legada */
uint16_t sensors_read_raw(uint32_t channel)
{
    return adc_read_channel(channel);
}

void sensors_read_all(uint16_t r[5])
{
    sensors_update();
    for (int i = 0; i < 5; i++)
    {
        r[i] = raw[i];
    }
}
