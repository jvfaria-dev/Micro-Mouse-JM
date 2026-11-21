/**
 ******************************************************************************
 * @file    motor_test.c
 * @brief   Sequencia automatica de teste dos motores e encoders.
 ******************************************************************************
 */

#include "motor_test.h"
#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "debug.h"
#include "ihm.h"

#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Definicao das etapas                                                        */
/* -------------------------------------------------------------------------- */

typedef enum
{
    STEP_BOOT = 0,      /* espera inicial, motores parados                */
    STEP_L_FWD,         /* so' o motor esquerdo, para frente              */
    STEP_L_REV,         /* so' o motor esquerdo, para tras                */
    STEP_R_FWD,         /* so' o motor direito, para frente               */
    STEP_R_REV,         /* so' o motor direito, para tras                 */
    STEP_BOTH_FWD,      /* os dois para frente (anda reto)                */
    STEP_BOTH_REV,      /* os dois para tras                              */
    STEP_SPIN,          /* gira no proprio eixo (sentidos opostos)        */
    STEP_SWEEP,         /* varredura de duty p/ achar a zona morta        */
    STEP_BRAKE,         /* acelera e testa a frenagem ativa               */
    STEP_COUNT
} test_step_t;

typedef struct
{
    const char *name;
    const char *hint;       /* o que voce deve observar */
    uint32_t    duration_ms;
    int32_t     duty_l;
    int32_t     duty_r;
} step_cfg_t;

#define D   MOTOR_TEST_DUTY

#if (MOTOR_TEST_STATIC == 0)
static const step_cfg_t step_cfg[STEP_COUNT] =
{
    [STEP_BOOT]     = { "BOOT",      "motores parados; confira a alimentacao",              2000u,  0,  0 },
    [STEP_L_FWD]    = { "ESQ FRENTE","OLHE: qual roda gira, e para que lado?",              2500u,  D,  0 },
    [STEP_L_REV]    = { "ESQ RE",    "so' a roda ESQUERDA gira p/ TRAS, ticks L descem",    2500u, -D,  0 },
    [STEP_R_FWD]    = { "DIR FRENTE","so' a roda DIREITA gira p/ FRENTE, ticks R sobem",    2500u,  0,  D },
    [STEP_R_REV]    = { "DIR RE",    "so' a roda DIREITA gira p/ TRAS, ticks R descem",     2500u,  0, -D },
    [STEP_BOTH_FWD] = { "AMBOS FRE", "as duas p/ frente; RPM L e R devem bater",            3000u,  D,  D },
    [STEP_BOTH_REV] = { "AMBOS RE",  "as duas p/ tras; RPM L e R devem bater",              3000u, -D, -D },
    [STEP_SPIN]     = { "GIRO",      "sentidos opostos; o carrinho gira no lugar",          3000u,  D, -D },
    [STEP_SWEEP]    = { "VARREDURA", "anote o duty em que as rodas COMECAM a girar",        8000u,  0,  0 },
    [STEP_BRAKE]    = { "FREIO",     "acelera e freia; a parada deve ser bem mais seca",    3000u,  D,  D },
};

#undef D

/* -------------------------------------------------------------------------- */
/* Estado                                                                      */
/* -------------------------------------------------------------------------- */

static test_step_t step        = STEP_BOOT;
static uint32_t    elapsed_ms  = 0u;
#endif /* MOTOR_TEST_STATIC == 0 */

static uint32_t    cycle       = 0u;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

#if (MOTOR_TEST_STATIC == 0)
/**
 * Perfil trapezoidal: sobe de 0 ate target em MOTOR_TEST_RAMP_MS, mantem, e
 * volta a 0 nos ultimos MOTOR_TEST_RAMP_MS. Evita pico de corrente e deixa
 * a leitura de velocidade estabilizar no meio da etapa.
 */
static int32_t ramp_profile(int32_t target, uint32_t t_ms, uint32_t total_ms)
{
    uint32_t ramp = MOTOR_TEST_RAMP_MS;

    if (target == 0)
    {
        return 0;
    }

    /* etapa curta demais para o trapezio: reduz a rampa */
    if ((2u * ramp) > total_ms)
    {
        ramp = total_ms / 2u;
    }

    if (ramp == 0u)
    {
        return target;
    }

    if (t_ms < ramp)
    {
        return (int32_t)((int64_t)target * (int64_t)t_ms / (int64_t)ramp);
    }

    if (t_ms >= (total_ms - ramp))
    {
        uint32_t left = total_ms - t_ms;
        return (int32_t)((int64_t)target * (int64_t)left / (int64_t)ramp);
    }

    return target;
}

/** Varredura triangular: 0 -> +max -> 0 -> -max -> 0 ao longo da etapa. */
static int32_t sweep_profile(uint32_t t_ms, uint32_t total_ms)
{
    if (total_ms == 0u)
    {
        return 0;
    }

    uint32_t quarter = total_ms / 4u;

    if (quarter == 0u)
    {
        return 0;
    }

    int32_t max = MOTOR_TEST_SWEEP_DUTY;
    int32_t v;

    if (t_ms < quarter)
    {
        v = (int32_t)((int64_t)max * (int64_t)t_ms / (int64_t)quarter);
    }
    else if (t_ms < (2u * quarter))
    {
        v = (int32_t)((int64_t)max * (int64_t)((2u * quarter) - t_ms) / (int64_t)quarter);
    }
    else if (t_ms < (3u * quarter))
    {
        v = -(int32_t)((int64_t)max * (int64_t)(t_ms - (2u * quarter)) / (int64_t)quarter);
    }
    else
    {
        uint32_t left = ((4u * quarter) > t_ms) ? ((4u * quarter) - t_ms) : 0u;
        v = -(int32_t)((int64_t)max * (int64_t)left / (int64_t)quarter);
    }

    return v;
}

static void step_enter(test_step_t s)
{
    step       = s;
    elapsed_ms = 0u;

    port_motor_stop_all();
    port_encoder_reset();

    port_debug_printf("\r\n--- etapa %d/%d: %s ---\r\n",
                      (int)s + 1, (int)STEP_COUNT, step_cfg[s].name);
    port_debug_printf("    esperado: %s\r\n", step_cfg[s].hint);
}

/* Quantos ticks ja' contam como "essa roda andou de verdade". Abaixo disso
 * pode ser vibracao da outra roda passando pela estrutura. */
#define MT_MOVED    200

/* Placar do ciclo, para o veredito do final. Uma linha isolada no meio de
 * dez etapas passa despercebida; um veredito no fim, nao. */
static int mt_cross = 0;    /* etapas em que o lado errado contou      */
static int mt_ok    = 0;    /* etapas em que o lado certo contou       */

static void step_exit(void)
{
    int32_t tl = port_encoder_get_ticks(ENCODER_LEFT);
    int32_t tr = port_encoder_get_ticks(ENCODER_RIGHT);

    int32_t al = (tl < 0) ? -tl : tl;
    int32_t ar = (tr < 0) ? -tr : tr;

    port_debug_printf("    resultado: ticks L=%+ld  R=%+ld", (long)tl, (long)tr);

    /* --- lados trocados ---------------------------------------------------
     * Numa etapa que comanda UM lado so', se o contador daquele lado ficou
     * mudo e o do OUTRO lado disparou, alguma coisa esta' cruzada. O teste
     * nao consegue dizer sozinho se foram os fios do MOTOR ou os do ENCODER
     * que trocaram de lugar -- os dois casos produzem exatamente este log.
     * Quem desempata e' voce olhando QUAL RODA girou. */
    if (step_cfg[step].duty_l != 0 && step_cfg[step].duty_r == 0
        && al < MT_MOVED && ar >= MT_MOVED)
    {
        port_debug_printf("   [!!] CRUZADO: comandei ESQ e quem contou foi DIR");
        mt_cross++;
    }
    else if (step_cfg[step].duty_l != 0 && step_cfg[step].duty_r == 0
             && al >= MT_MOVED)
    {
        mt_ok++;
    }
    if (step_cfg[step].duty_r != 0 && step_cfg[step].duty_l == 0
        && ar < MT_MOVED && al >= MT_MOVED)
    {
        port_debug_printf("   [!!] CRUZADO: comandei DIR e quem contou foi ESQ");
        mt_cross++;
    }
    else if (step_cfg[step].duty_r != 0 && step_cfg[step].duty_l == 0
             && ar >= MT_MOVED)
    {
        mt_ok++;
    }

    /* Diagnostico automatico simples */
    if (step_cfg[step].duty_l != 0 && tl == 0)
    {
        port_debug_printf("   [!] encoder ESQ nao contou");
    }
    if (step_cfg[step].duty_r != 0 && tr == 0)
    {
        port_debug_printf("   [!] encoder DIR nao contou");
    }
    if (step_cfg[step].duty_l > 0 && tl < 0)
    {
        port_debug_printf("   [!] sinal ESQ invertido (ver ENCODER_L_INVERT)");
    }
    if (step_cfg[step].duty_r > 0 && tr < 0)
    {
        port_debug_printf("   [!] sinal DIR invertido (ver ENCODER_R_INVERT)");
    }

    port_debug_printf("\r\n");
}
#endif /* MOTOR_TEST_STATIC == 0 */

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

void motor_test_init(void)
{
    cycle = 0u;
    port_ihm_led_blink(1000u);

#if (MOTOR_TEST_STATIC != 0)
    port_debug_printf("\r\n*** MODO BANCADA: duty fixo de %d nos dois motores ***\r\n",
                      (int)MOTOR_TEST_STATIC_DUTY);
    port_motor_set_both(MOTOR_TEST_STATIC_DUTY, MOTOR_TEST_STATIC_DUTY);
#else
    port_debug_printf("\r\nTeste automatico iniciando.\r\n");
    port_debug_printf("Deixe o carrinho SUSPENSO (rodas no ar) na primeira vez.\r\n");

    step_enter(STEP_BOOT);
#endif
}

void motor_test_update(void)
{
#if (MOTOR_TEST_STATIC != 0)
    /* Modo bancada: saida fixa, sem maquina de estados. */
    port_motor_set_both(MOTOR_TEST_STATIC_DUTY, MOTOR_TEST_STATIC_DUTY);
    return;
#else
    const step_cfg_t *cfg = &step_cfg[step];

    /* --- comando dos motores nesta etapa --- */
    if (step == STEP_SWEEP)
    {
        int32_t v = sweep_profile(elapsed_ms, cfg->duration_ms);
        port_motor_set_both(v, v);
    }
    else if (step == STEP_BRAKE)
    {
        /* 2/3 do tempo acelerando, 1/3 com freio ativo */
        uint32_t brake_at = (cfg->duration_ms * 2u) / 3u;

        if (elapsed_ms < brake_at)
        {
            int32_t v = ramp_profile(cfg->duty_l, elapsed_ms, brake_at);
            port_motor_set_both(v, v);
        }
        else
        {
            port_motor_brake(MOTOR_LEFT);
            port_motor_brake(MOTOR_RIGHT);
        }
    }
    else
    {
        port_motor_set(MOTOR_LEFT,
                       ramp_profile(cfg->duty_l, elapsed_ms, cfg->duration_ms));
        port_motor_set(MOTOR_RIGHT,
                       ramp_profile(cfg->duty_r, elapsed_ms, cfg->duration_ms));
    }

    /* --- avanco do tempo / transicao --- */
    elapsed_ms += CONTROL_PERIOD_MS;

    bool skip = port_ihm_button_clicked();

    if ((elapsed_ms >= cfg->duration_ms) || skip)
    {
        if (skip)
        {
            port_debug_printf("    (pulada pelo botao)\r\n");
        }

        step_exit();

        test_step_t next = (test_step_t)((step + 1) % STEP_COUNT);

        if (next == STEP_BOOT)
        {
            cycle++;
            port_debug_printf("\r\n===== ciclo %lu concluido =====\r\n",
                              (unsigned long)cycle);

            /* VEREDITO -- uma linha so', impossivel de nao ver.
             *
             * Existe porque "olhei e estava tudo certo" nao consegue detectar
             * cruzamento: as rodas giram do jeito certo nos dois casos, e o
             * que muda e' apenas QUAL CONTADOR se mexe. So' o numero acusa. */
            port_debug_printf("\r\n>>> VEREDITO: ");

            if (mt_cross > 0)
            {
                port_debug_printf("CRUZADO em %d de %d etapas de uma roda so'\r\n",
                                  mt_cross, mt_cross + mt_ok);
                port_debug_printf(">>> Motor e encoder de lados diferentes.\r\n");
                port_debug_printf(">>> Em malha aberta isso nao atrapalha, mas\r\n");
                port_debug_printf(">>> com o PID vira divergencia: uma roda\r\n");
                port_debug_printf(">>> acelera e a outra tenta dar re'.\r\n");
            }
            else if (mt_ok >= 4)
            {
                port_debug_printf("cada motor com o SEU encoder (%d/4 ok)\r\n",
                                  mt_ok);
            }
            else
            {
                port_debug_printf("inconclusivo -- so' %d etapas contaram.\r\n",
                                  mt_ok);
                port_debug_printf(">>> Alguma roda nao girou o bastante.\r\n");
            }

            mt_cross = 0;
            mt_ok    = 0;

            /* Guia de decisao. Sao QUATRO espelhamentos independentes e cada
             * um tem seu proprio ajuste; a tentacao e' trocar dois de uma vez
             * e acabar com o carrinho andando igual e o log mentindo. Mude um
             * de cada vez e rode o teste de novo. */
            if (cycle == 1u)
            {
                port_debug_printf("\r\n O QUE FAZER COM O QUE VOCE VIU\r\n");
                port_debug_printf(" ------------------------------------------\r\n");
                port_debug_printf(" Na etapa 'ESQ FRENTE' girou a roda DIREITA?\r\n");
                port_debug_printf("   -> fios dos MOTORES trocados. No board.h\r\n");
                port_debug_printf("      troque BOARD_MOTOR_L_IN1_CH/IN2_CH\r\n");
                port_debug_printf("      com   BOARD_MOTOR_R_IN1_CH/IN2_CH\r\n\r\n");
                port_debug_printf(" Apareceu [!!] CRUZADO?\r\n");
                port_debug_printf("   -> se a roda CERTA girou, quem trocou foi\r\n");
                port_debug_printf("      o ENCODER: troque BOARD_ENC_L_TIM com\r\n");
                port_debug_printf("      BOARD_ENC_R_TIM, e leve junto os\r\n");
                port_debug_printf("      _IS_32BIT e os pinos _A_/_B_ de cada um\r\n");
                port_debug_printf("      (TIM2 e' de 32 bits, TIM3 de 16)\r\n");
                port_debug_printf("   -> se a roda ERRADA girou, foi so' motor:\r\n");
                port_debug_printf("      trocando os motores o [!!] some sozinho\r\n\r\n");
                port_debug_printf(" A roda girou para o lado errado?\r\n");
                port_debug_printf("   -> MOTOR_?_INVERT daquele lado\r\n\r\n");
                port_debug_printf(" Girou certo mas os ticks vieram negativos?\r\n");
                port_debug_printf("   -> ENCODER_?_INVERT daquele lado\r\n\r\n");
                port_debug_printf(" MUDE UMA COISA DE CADA VEZ. Dois ajustes\r\n");
                port_debug_printf(" errados se cancelam no log e escondem o\r\n");
                port_debug_printf(" defeito -- foi o que aconteceu antes, com o\r\n");
                port_debug_printf(" carrinho andando de re' e o teste limpo.\r\n");
                port_debug_printf(" ------------------------------------------\r\n");
                port_debug_printf(" E confira NO CHAO se 'frente' e' a frente\r\n");
                port_debug_printf(" do carrinho. Suspenso isso nao aparece.\r\n");
            }

            port_debug_printf("\r\n===== reiniciando =====\r\n");
        }

        step_enter(next);
    }
#endif
}

const char *motor_test_step_name(void)
{
#if (MOTOR_TEST_STATIC != 0)
    return "BANCADA";
#else
    return step_cfg[step].name;
#endif
}
