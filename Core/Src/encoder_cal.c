/**
 ******************************************************************************
 * @file    encoder_cal.c
 * @brief   Calibracao manual do encoder (ticks por volta da roda).
 ******************************************************************************
 */

#include "encoder_cal.h"
#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "debug.h"
#include "ihm.h"

static uint32_t print_acc = 0u;
static int32_t  last_l    = 0;
static int32_t  last_r    = 0;

/* --------------------------------------------------------------------------
 * Vigia dos quatro canais de encoder
 * --------------------------------------------------------------------------
 * Um canal preso num nivel produz contagem ZERO, nao contagem errada: em
 * quadratura, a borda de subida conta para um lado e a de descida para o
 * outro, e as duas se cancelam quando o outro canal nao acompanha. Por isso
 * "encoder lendo zero" e "encoder sem sinal" NAO sao a mesma coisa, e so'
 * olhando canal por canal da' pra saber qual dos dois falhou.
 *
 * Aqui cada canal e' lido como GPIO a 1 kHz e as BORDAS sao contadas. Um
 * contador e' melhor que um indicador de sim/nao porque distingue os dois
 * casos que se parecem: "o canal esta' morto" e "voce nao girou o bastante".
 * Girando, os numeros dos quatro canais tem que subir mais ou menos juntos.
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
    GPIO_PinState last;
    uint32_t      edges;
} chan_t;

static chan_t chan[4];

static void chan_init(void)
{
    chan[0] = (chan_t){ BOARD_ENC_L_A_PORT, BOARD_ENC_L_A_PIN, "ESQ-A", GPIO_PIN_RESET, 0u };
    chan[1] = (chan_t){ BOARD_ENC_L_B_PORT, BOARD_ENC_L_B_PIN, "ESQ-B", GPIO_PIN_RESET, 0u };
    chan[2] = (chan_t){ BOARD_ENC_R_A_PORT, BOARD_ENC_R_A_PIN, "DIR-A", GPIO_PIN_RESET, 0u };
    chan[3] = (chan_t){ BOARD_ENC_R_B_PORT, BOARD_ENC_R_B_PIN, "DIR-B", GPIO_PIN_RESET, 0u };

    for (int i = 0; i < 4; i++)
    {
        chan[i].last = HAL_GPIO_ReadPin(chan[i].port, chan[i].pin);
    }
}

/* Chamada a 1 kHz pela tarefa de controle. */
static void chan_sample(void)
{
    for (int i = 0; i < 4; i++)
    {
        GPIO_PinState v = HAL_GPIO_ReadPin(chan[i].port, chan[i].pin);

        if (v != chan[i].last)
        {
            chan[i].last = v;
            chan[i].edges++;
        }
    }
}

static void chan_report(void)
{
    port_debug_printf("  bordas: ");

    for (int i = 0; i < 4; i++)
    {
        port_debug_printf("%s=%-6lu(nivel %d)  ",
                          chan[i].name,
                          (unsigned long)chan[i].edges,
                          (int)HAL_GPIO_ReadPin(chan[i].port, chan[i].pin));
    }

    port_debug_printf("\r\n");
}

#if (ENCODER_CAL_RUN_DUTY > 0)
typedef enum { RUN_WAIT = 0, RUN_GO, RUN_DONE } run_state_t;
static run_state_t run_state = RUN_WAIT;
static uint32_t    run_ms    = 0u;
#endif

/* Imprime "n / voltas" como inteiro, sem float. */
static void print_ratio(const char *label, int32_t ticks, int32_t voltas)
{
    int32_t a = (ticks < 0) ? -ticks : ticks;
    port_debug_printf("  %s %2ld volta(s): %6ld ticks/volta\r\n",
                      label, (long)voltas, (long)(a / voltas));
}

void encoder_cal_init(void)
{
    port_motor_stop_all();      /* roda livre: da' pra girar com o dedo */
    port_encoder_reset();

    print_acc = 0u;
    last_l    = 0;
    last_r    = 0;

    chan_init();
    port_ihm_led_blink(200u);

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" CALIBRACAO DO ENCODER\r\n");
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" Os motores estao em roda livre.\r\n\r\n");
    port_debug_printf(" 1. Marque um ponto da roda com fita.\r\n");
    port_debug_printf(" 2. Gire DEVAGAR, para frente, contando as voltas.\r\n");
    port_debug_printf(" 3. Compare com o valor atual do board.h:\r\n");
    port_debug_printf("      ENCODER_TICKS_PER_REV = %ld\r\n",
                      (long)ENCODER_TICKS_PER_REV);
    port_debug_printf("\r\n Se o valor medido nao bater, corrija ENCODER_PPR\r\n");
    port_debug_printf(" ou ENCODER_GEAR_RATIO no board.h.\r\n");
    port_debug_printf("--------------------------------------------------\r\n");
}

#if (ENCODER_CAL_RUN_DUTY > 0)
/* Imprime "ticks / voltas" para varias contagens possiveis de volta. */
static void run_report(int32_t tl, int32_t tr)
{
    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" TESTE MOTORIZADO - duty %d por %lu ms\r\n",
                      (int)ENCODER_CAL_RUN_DUTY, (unsigned long)ENCODER_CAL_RUN_MS);
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" ticks contados: ESQ=%+ld  DIR=%+ld\r\n", (long)tl, (long)tr);

    int32_t al = (tl < 0) ? -tl : tl;
    int32_t ar = (tr < 0) ? -tr : tr;

    port_debug_printf(" ticks/segundo:  ESQ=%ld  DIR=%ld\r\n",
                      (long)(al * 1000 / (int32_t)ENCODER_CAL_RUN_MS),
                      (long)(ar * 1000 / (int32_t)ENCODER_CAL_RUN_MS));

    port_debug_printf("\r\n Conte as voltas da roda no video e ache a linha:\r\n");
    port_debug_printf("   voltas | ticks/volta ESQ | ticks/volta DIR\r\n");

    static const int32_t guesses[] = { 5, 10, 20, 50, 100, 200, 500 };

    for (unsigned i = 0; i < (sizeof(guesses) / sizeof(guesses[0])); i++)
    {
        int32_t v = guesses[i];
        port_debug_printf("   %6ld | %15ld | %15ld\r\n",
                          (long)v, (long)(al / v), (long)(ar / v));
    }

    port_debug_printf("\r\n Compare com o valor do teste manual. Se bater, a\r\n");
    port_debug_printf(" contagem e' confiavel. Se o numero girando for MAIOR,\r\n");
    port_debug_printf(" o motor esta' injetando pulsos falsos no encoder.\r\n");
    port_debug_printf("==================================================\r\n");
}
#endif

void encoder_cal_update(void)
{
    chan_sample();

    print_acc += CONTROL_PERIOD_MS;

    if (print_acc < ENCODER_CAL_PRINT_MS)
    {
        return;
    }

    print_acc = 0u;

    int32_t tl = port_encoder_get_ticks(ENCODER_LEFT);
    int32_t tr = port_encoder_get_ticks(ENCODER_RIGHT);

    /* O relatorio de bordas sai SEMPRE: e' o unico jeito de ver a diferenca
     * entre "canal morto" e "nao girei o bastante" enquanto voce gira. */
    port_debug_printf("\r\nticks ESQ=%+ld  DIR=%+ld\r\n", (long)tl, (long)tr);
    chan_report();

    if (tl != last_l || tr != last_r)
    {
        last_l = tl;
        last_r = tr;

        if (tl != 0)
        {
            port_debug_printf(" se a roda ESQ girou...\r\n");
            print_ratio("ESQ", tl, 1);
            print_ratio("ESQ", tl, 5);
            print_ratio("ESQ", tl, 10);
        }

        if (tr != 0)
        {
            port_debug_printf(" se a roda DIR girou...\r\n");
            print_ratio("DIR", tr, 1);
            print_ratio("DIR", tr, 5);
            print_ratio("DIR", tr, 10);
        }
    }
}
