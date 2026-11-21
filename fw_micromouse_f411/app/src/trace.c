/**
 ******************************************************************************
 * @file    trace.c
 * @brief   Registro do percurso em RAM.
 ******************************************************************************
 */

#include "trace.h"
#include "board.h"
#include "sensors.h"
#include "speed_ctl.h"
#include "nav.h"
#include "debug.h"

typedef struct
{
    uint16_t t_ds;          /* tempo desde o inicio, em decimos de segundo */
    int16_t  dist_mm;
    int16_t  head_deg;
    int16_t  prox[SENSOR_COUNT];
    int16_t  duty_l;
    int16_t  duty_r;
    uint8_t  state;         /* indice do estado da navegacao */
} trace_rec_t;

static trace_rec_t rec[TRACE_CAPACITY];
static uint32_t    head  = 0u;   /* proxima posicao a escrever */
static uint32_t    total = 0u;   /* quantas ja' foram gravadas no total  */
static uint32_t    t0_ms = 0u;

/* Guardar o NOME do estado custaria um ponteiro por amostra e obrigaria a
 * confiar que a string continua viva. Guardar o indice e resolver o nome so'
 * na hora de imprimir custa um byte e nao depende de nada. */
static const char *const state_short[] =
{
    "ESPERA", "CORRED", "APROX ", "RECUA ", "GIRO  ", "ASSENT", "FIM   ",
    "ABORT "
};

#define STATE_COUNT   (sizeof(state_short) / sizeof(state_short[0]))

void trace_init(void)
{
    head  = 0u;
    total = 0u;
    t0_ms = HAL_GetTick();
}

void trace_sample(void)
{
    trace_rec_t *r = &rec[head];

    r->t_ds     = (uint16_t)((HAL_GetTick() - t0_ms) / 100u);
    r->dist_mm  = (int16_t)speed_ctl_get_distance_mm();
    r->head_deg = (int16_t)speed_ctl_get_heading_deg();
    r->duty_l   = (int16_t)speed_ctl_get_duty(0);
    r->duty_r   = (int16_t)speed_ctl_get_duty(1);
    r->state    = (uint8_t)nav_state_index();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        r->prox[i] = port_sensors_get_prox((sensor_id_t)i);
    }

    head = (head + 1u) % TRACE_CAPACITY;
    total++;
}

uint32_t trace_count(void)
{
    return (total < TRACE_CAPACITY) ? total : TRACE_CAPACITY;
}

void trace_dump(void)
{
    uint32_t n = trace_count();

    /* Com o buffer cheio a amostra mais antiga esta' em head, nao em 0. */
    uint32_t first = (total < TRACE_CAPACITY) ? 0u : head;

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" REGISTRO DO PERCURSO -- %lu amostras\r\n",
                      (unsigned long)n);

    if (total > TRACE_CAPACITY)
    {
        port_debug_printf(" (o buffer encheu: as %lu primeiras foram perdidas)\r\n",
                          (unsigned long)(total - TRACE_CAPACITY));
    }

    port_debug_printf("==================================================\r\n");
    port_debug_printf("  t[s] estado   dist   ang |  ESQ   FE   FD  DIR |"
                      "  dutyE  dutyD\r\n");

    for (uint32_t i = 0u; i < n; i++)
    {
        const trace_rec_t *r = &rec[(first + i) % TRACE_CAPACITY];

        port_debug_printf("%3lu.%lu %s %6ld %5ld | %4d %4d %4d %4d |"
                          " %+6ld %+6ld\r\n",
                          (unsigned long)(r->t_ds / 10u),
                          (unsigned long)(r->t_ds % 10u),
                          (r->state < STATE_COUNT) ? state_short[r->state] : "??????",
                          (long)r->dist_mm,
                          (long)r->head_deg,
                          (int)r->prox[SENSOR_LEFT],
                          (int)r->prox[SENSOR_FRONT_L],
                          (int)r->prox[SENSOR_FRONT_R],
                          (int)r->prox[SENSOR_RIGHT],
                          (long)r->duty_l,
                          (long)r->duty_r);
    }

    port_debug_printf("==================================================\r\n");
    port_debug_printf(" Aperte uma tecla para repetir o despejo.\r\n\r\n");
}
