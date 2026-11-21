/**
 ******************************************************************************
 * @file    sensors.c
 * @brief   Leitura dos quatro sensores IR pelo ADC1.
 *
 * Os quatro canais sao lidos em sequencia, reconfigurando o rank 1 do ADC a
 * cada conversao. E' mais simples que scan+DMA e, a 200 Hz por sensor, o
 * custo e' irrelevante (800 conversoes/s num ADC que faz centenas de
 * milhares). Em compensacao nao depende de nenhum ajuste de DMA no CubeMX,
 * que e' onde configuracoes costumam se perder entre uma regeracao e outra.
 ******************************************************************************
 */

#include "sensors.h"
#include "board.h"

extern ADC_HandleTypeDef BOARD_SENSOR_ADC;

/* ---------------------------------------------------------------------------
 * Configuracao estatica de cada sensor
 * ------------------------------------------------------------------------- */

typedef struct
{
    uint32_t    channel;
    int32_t     raw_far;    /* leitura sem parede            */
    int32_t     raw_near;   /* leitura com parede a' distancia de referencia */
    const char *name;
} sensor_cfg_t;

/* nao e' const: a largada recalibra (port_sensors_set_cal) */
static sensor_cfg_t sensor_cfg[SENSOR_COUNT] =
{
    [SENSOR_LEFT]    = { BOARD_SENSOR_CH_LEFT,    SENSOR_L_RAW_FAR,  SENSOR_L_RAW_NEAR,  "ESQ"  },
    [SENSOR_FRONT_L] = { BOARD_SENSOR_CH_FRONT_L, SENSOR_FL_RAW_FAR, SENSOR_FL_RAW_NEAR, "FR-E" },
    [SENSOR_FRONT_R] = { BOARD_SENSOR_CH_FRONT_R, SENSOR_FR_RAW_FAR, SENSOR_FR_RAW_NEAR, "FR-D" },
    [SENSOR_RIGHT]   = { BOARD_SENSOR_CH_RIGHT,   SENSOR_R_RAW_FAR,  SENSOR_R_RAW_NEAR,  "DIR"  },
};

/* ---------------------------------------------------------------------------
 * Estado
 * ------------------------------------------------------------------------- */

static uint16_t raw[SENSOR_COUNT];          /* ultima leitura crua        */
static int32_t  raw_filt_x1000[SENSOR_COUNT]; /* filtro em ponto fixo     */
static int16_t  prox[SENSOR_COUNT];         /* 0..1000 calibrado          */
static bool     ready = false;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static uint16_t adc_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};

    cfg.Channel      = channel;
    cfg.Rank         = 1u;
    cfg.SamplingTime = BOARD_SENSOR_SAMPLETIME;
    cfg.Offset       = 0u;

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

/**
 * Converte a leitura crua em proximidade 0..1000 interpolando entre os dois
 * pontos de calibracao. Funciona com near > far e com near < far, entao nao
 * importa se o modulo da' tensao alta ou baixa vendo o branco.
 */
static int16_t raw_to_prox(const sensor_cfg_t *cfg, int32_t value)
{
    int32_t span = cfg->raw_near - cfg->raw_far;

    if (span == 0)
    {
        return 0;
    }

    int32_t p = ((value - cfg->raw_far) * 1000) / span;

    /* Deixa passar de 1000 de proposito: 1000 e' a distancia de REFERENCIA
     * da calibracao, nao o maximo fisico. Se saturasse em 1000, o sensor do
     * lado em que o carrinho encostou pararia de dar informacao justamente
     * quando ela mais importa, e a centralizacao ficaria cega de um lado. */
    if (p < 0)      { p = 0; }
    if (p > 2000)   { p = 2000; }

    return (int16_t)p;
}

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

/**
 * O .ioc gerou o ADC com PCLK2/2. A 16 MHz isso da' 8 MHz e esta' ok, mas se
 * o clock subir para 100 MHz vira 50 MHz -- acima dos 36 MHz que o ADC da
 * F411 aceita, e a leitura sai lixo sem nenhum erro. PCLK2/4 serve nos dois
 * casos (4 MHz ou 25 MHz).
 */
static void sensors_adc_prescaler(void)
{
    if (BOARD_SENSOR_ADC.Init.ClockPrescaler != ADC_CLOCK_SYNC_PCLK_DIV4)
    {
        BOARD_SENSOR_ADC.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
        (void)HAL_ADC_Init(&BOARD_SENSOR_ADC);
    }
}

void port_sensors_init(void)
{
    sensors_adc_prescaler();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        raw[i]            = 0u;
        raw_filt_x1000[i] = 0;
        prox[i]           = 0;
    }

    ready = true;

    /* Algumas leituras para o filtro nascer no valor certo em vez de subir
     * a partir do zero (o que faria o robo "ver parede" no primeiro ciclo). */
    for (int n = 0; n < 8; n++)
    {
        port_sensors_update();
    }
}

void port_sensors_update(void)
{
    if (!ready)
    {
        return;
    }

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        const sensor_cfg_t *cfg = &sensor_cfg[i];

        uint16_t v = adc_read_channel(cfg->channel);
        raw[i] = v;

        /* filtro exponencial em ponto fixo (x1000 para nao perder resolucao) */
        int32_t target = (int32_t)v * 1000;

        if (raw_filt_x1000[i] == 0)
        {
            raw_filt_x1000[i] = target;     /* primeira amostra */
        }
        else
        {
            raw_filt_x1000[i] += ((target - raw_filt_x1000[i])
                                  * SENSOR_FILTER_ALPHA) / 1000;
        }

        prox[i] = raw_to_prox(cfg, raw_filt_x1000[i] / 1000);
    }
}

uint16_t port_sensors_get_raw(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? raw[s] : 0u;
}

uint16_t port_sensors_get_raw_filtered(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? (uint16_t)(raw_filt_x1000[s] / 1000) : 0u;
}

int16_t port_sensors_get_prox(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? prox[s] : 0;
}

bool port_sensors_wall(sensor_id_t s)
{
    return (port_sensors_get_prox(s) > SENSOR_WALL_THRESHOLD);
}

int16_t port_sensors_front(void)
{
    return (int16_t)((prox[SENSOR_FRONT_L] + prox[SENSOR_FRONT_R]) / 2);
}

bool port_sensors_wall_ahead(void)
{
    return (port_sensors_front() > SENSOR_FRONT_STOP);
}

void port_sensors_set_cal(sensor_id_t s, int32_t raw_far, int32_t raw_near)
{
    if (s < SENSOR_COUNT && raw_far > raw_near + 100)
    {
        sensor_cfg[s].raw_far  = raw_far;
        sensor_cfg[s].raw_near = raw_near;
    }
}

int32_t port_sensors_get_cal_far(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? sensor_cfg[s].raw_far : 0;
}

int32_t port_sensors_get_cal_near(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? sensor_cfg[s].raw_near : 0;
}

int16_t port_sensors_center_error(void)
{
    bool wl = port_sensors_wall(SENSOR_LEFT);
    bool wr = port_sensors_wall(SENSOR_RIGHT);

    int16_t pl = prox[SENSOR_LEFT];
    int16_t pr = prox[SENSOR_RIGHT];

    if (wl && wr)
    {
        /* Duas paredes: a diferenca ja' diz de que lado esta' apertado.
         * Positivo = mais perto da direita.
         *
         * O TRIM entra so' aqui, e o motivo importa: este ramo compara DOIS
         * sensores diferentes entre si, e dois modulos IR nunca respondem
         * igual na mesma distancia. Essa diferenca de fabricacao vira um erro
         * constante, e um erro constante no centralizador nao e' desvio: e'
         * comando. O carrinho persegue obedientemente um centro que nao e' o
         * centro. Nos outros dois ramos cada sensor e' comparado com a
         * PROPRIA referencia de calibracao, entao o descasamento nao aparece
         * e o trim nao tem o que corrigir. */
        return (int16_t)((pr - pl) - SENSOR_CENTER_TRIM);
    }

    /* Com uma parede so', a referencia e' 1000: por construcao da calibracao,
     * 1000 e' a leitura com o carrinho centralizado no corredor. */
    if (wr)
    {
        return (int16_t)(pr - 1000);
    }

    if (wl)
    {
        return (int16_t)(1000 - pl);
    }

    return 0;   /* corredor aberto dos dois lados: segue reto */
}

const char *port_sensors_name(sensor_id_t s)
{
    return (s < SENSOR_COUNT) ? sensor_cfg[s].name : "?";
}
