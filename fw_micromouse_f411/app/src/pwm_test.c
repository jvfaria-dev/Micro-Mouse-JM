/**
 ******************************************************************************
 * @file    pwm_test.c
 * @brief   Ensaio de caracterizacao do PWM (escada de duty x velocidade).
 ******************************************************************************
 */

#include "pwm_test.h"
#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "debug.h"
#include "ihm.h"

/* -------------------------------------------------------------------------- */
/* Fases do ensaio                                                             */
/* -------------------------------------------------------------------------- */

typedef enum
{
    PH_INTRO = 0,
    PH_FWD_UP,      /* frente, duty subindo   -> zona morta de partida */
    PH_FWD_DOWN,    /* frente, duty descendo  -> duty de parada        */
    PH_REV_UP,      /* re',    duty subindo                            */
    PH_REV_DOWN,    /* re',    duty descendo                           */
    PH_REPORT,
    PH_DONE
} pwm_phase_t;

static const char *const phase_name[] =
{
    "INTRO", "FRENTE+", "FRENTE-", "RE+", "RE-", "RELATORIO", "FIM"
};

/* Sub-estado dentro de um patamar */
typedef enum
{
    SUB_SETTLE = 0,
    SUB_MEASURE
} sub_state_t;

/* Resultados de um sentido de rotacao */
/* Acumuladores de minimos quadrados de um motor: rpm = a*duty + b.
 * Alimentados so' com os patamares em que a roda estava de fato girando. */
typedef struct
{
    int64_t n;
    int64_t sd;     /* soma de duty            */
    int64_t sr;     /* soma de rpm_milli       */
    int64_t sdr;    /* soma de duty*rpm_milli  */
    int64_t sdd;    /* soma de duty*duty       */
} fit_acc_t;

typedef struct
{
    int32_t dead_up_l;      /* duty em que a roda ESQ comecou a girar   */
    int32_t dead_up_r;
    int32_t dead_dn_l;      /* duty em que a roda ESQ parou (descendo)  */
    int32_t dead_dn_r;
    int32_t rpm_max_l;      /* |RPM| no duty maximo                     */
    int32_t rpm_max_r;
    fit_acc_t fit_l;
    fit_acc_t fit_r;
} dir_result_t;

/* -------------------------------------------------------------------------- */
/* Estado                                                                      */
/* -------------------------------------------------------------------------- */

static pwm_phase_t  phase       = PH_INTRO;
static sub_state_t  sub         = SUB_SETTLE;
static uint32_t     sub_ms      = 0u;
static int32_t      duty        = 0;
static int32_t      snapshot_l  = 0;
static int32_t      snapshot_r  = 0;
static uint32_t     run         = 0u;

static dir_result_t res_fwd;
static dir_result_t res_rev;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static void result_clear(dir_result_t *r)
{
    r->dead_up_l = -1;
    r->dead_up_r = -1;
    r->dead_dn_l = -1;
    r->dead_dn_r = -1;
    r->rpm_max_l = 0;
    r->rpm_max_r = 0;

    const fit_acc_t zero = { 0, 0, 0, 0, 0 };
    r->fit_l = zero;
    r->fit_r = zero;
}

static void fit_add(fit_acc_t *f, int32_t duty, int32_t rpm_milli)
{
    f->n++;
    f->sd  += duty;
    f->sr  += rpm_milli;
    f->sdr += (int64_t)duty * (int64_t)rpm_milli;
    f->sdd += (int64_t)duty * (int64_t)duty;
}

/**
 * Resolve a reta por minimos quadrados e devolve o duty em que ela cruza
 * RPM = 0 (o offset real do atuador) e a inclinacao em milesimos de RPM
 * por 1000 de duty. Devolve false se nao houver pontos suficientes.
 */
static bool fit_solve(const fit_acc_t *f, int32_t *duty_zero, int32_t *slope_k)
{
    if (f->n < 3)
    {
        return false;
    }

    int64_t num = f->n * f->sdr - f->sd * f->sr;   /* numerador da inclinacao */
    int64_t den = f->n * f->sdd - f->sd * f->sd;   /* denominador             */

    if (num == 0 || den == 0)
    {
        return false;
    }

    /* cruzamento com zero: duty0 = (sd*num - sr*den) / (n*num) */
    *duty_zero = (int32_t)((f->sd * num - f->sr * den) / (f->n * num));
    *slope_k   = (int32_t)((num * 1000) / den);

    return true;
}

/** RPM medio na janela, em milesimos, a partir do numero de ticks contados. */
static int32_t ticks_to_rpm_milli(int32_t ticks, uint32_t window_ms)
{
    if (window_ms == 0u)
    {
        return 0;
    }

    int64_t num = (int64_t)ticks * 60000LL * 1000LL;
    int64_t den = (int64_t)ENCODER_TICKS_PER_REV * (int64_t)window_ms;

    return (den != 0) ? (int32_t)(num / den) : 0;
}

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static bool phase_is_reverse(void)
{
    return (phase == PH_REV_UP) || (phase == PH_REV_DOWN);
}

static bool phase_is_ascending(void)
{
    return (phase == PH_FWD_UP) || (phase == PH_REV_UP);
}

static dir_result_t *current_result(void)
{
    return phase_is_reverse() ? &res_rev : &res_fwd;
}

/** Aplica o duty do patamar atual, com sinal conforme o sentido da fase. */
static void apply_duty(void)
{
    int32_t signed_duty = phase_is_reverse() ? -duty : duty;
    port_motor_set_both(signed_duty, signed_duty);
}

static void phase_header(void)
{
    port_debug_printf("\r\n=== %s ===\r\n", phase_name[phase]);
    port_debug_printf("duty;rpm_esq;rpm_dir\r\n");
}

/* -------------------------------------------------------------------------- */
/* Fim de um patamar: mede, imprime e registra                                 */
/* -------------------------------------------------------------------------- */

static void level_finish(void)
{
    int32_t dl = port_encoder_get_ticks(ENCODER_LEFT)  - snapshot_l;
    int32_t dr = port_encoder_get_ticks(ENCODER_RIGHT) - snapshot_r;

    int32_t rpm_l = ticks_to_rpm_milli(dl, PWM_TEST_MEASURE_MS);
    int32_t rpm_r = ticks_to_rpm_milli(dr, PWM_TEST_MEASURE_MS);

    port_debug_printf("%ld;" DEBUG_MILLI_FMT ";" DEBUG_MILLI_FMT "\r\n",
                      (long)duty,
                      DEBUG_MILLI_ARGS(rpm_l),
                      DEBUG_MILLI_ARGS(rpm_r));

    dir_result_t *r = current_result();

    bool moving_l = (abs32(rpm_l) >= PWM_TEST_MOVE_RPM_MILLI);
    bool moving_r = (abs32(rpm_r) >= PWM_TEST_MOVE_RPM_MILLI);

    if (phase_is_ascending())
    {
        /* primeira vez que se mexeu subindo = zona morta de partida */
        if (moving_l && r->dead_up_l < 0) { r->dead_up_l = duty; }
        if (moving_r && r->dead_up_r < 0) { r->dead_up_r = duty; }

        if (abs32(rpm_l) > abs32(r->rpm_max_l)) { r->rpm_max_l = rpm_l; }
        if (abs32(rpm_r) > abs32(r->rpm_max_r)) { r->rpm_max_r = rpm_r; }

        /* so' pontos com a roda girando entram na reta */
        if (moving_l) { fit_add(&r->fit_l, duty, abs32(rpm_l)); }
        if (moving_r) { fit_add(&r->fit_r, duty, abs32(rpm_r)); }
    }
    else
    {
        /* descendo: guarda o ultimo duty em que ainda estava girando */
        if (moving_l) { r->dead_dn_l = duty; }
        if (moving_r) { r->dead_dn_r = duty; }
    }
}

/* -------------------------------------------------------------------------- */
/* Relatorio final                                                             */
/* -------------------------------------------------------------------------- */

static void print_deadband(const char *label, int32_t up, int32_t dn)
{
    if (up < 0)
    {
        port_debug_printf("  %s: NAO GIROU em nenhum duty ate %d\r\n",
                          label, (int)PWM_TEST_MAX_DUTY);
        return;
    }

    port_debug_printf("  %s: parte em %ld", label, (long)up);

    if (dn >= 0)
    {
        port_debug_printf(", para em %ld (histerese %ld)",
                          (long)dn, (long)(up - dn));
    }

    port_debug_printf("\r\n");
}

static int32_t suggest_deadband(void)
{
    /* Sugere o menor duty de partida entre os quatro casos: e' o ponto a
     * partir do qual QUALQUER roda responde, entao e' o piso seguro. */
    int32_t best = -1;
    const int32_t cands[4] =
    {
        res_fwd.dead_up_l, res_fwd.dead_up_r,
        res_rev.dead_up_l, res_rev.dead_up_r
    };

    for (int i = 0; i < 4; i++)
    {
        if (cands[i] >= 0 && (best < 0 || cands[i] > best))
        {
            best = cands[i];   /* o MAIOR: so' acima dele as quatro respondem */
        }
    }

    return best;
}

static void print_report(void)
{
    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" RELATORIO DO ENSAIO DE PWM (execucao %lu)\r\n",
                      (unsigned long)run);
    port_debug_printf("==================================================\r\n");

    port_debug_printf("\r\nZona morta (duty por-mil):\r\n");
    print_deadband("ESQ frente", res_fwd.dead_up_l, res_fwd.dead_dn_l);
    print_deadband("DIR frente", res_fwd.dead_up_r, res_fwd.dead_dn_r);
    print_deadband("ESQ re'   ", res_rev.dead_up_l, res_rev.dead_dn_l);
    print_deadband("DIR re'   ", res_rev.dead_up_r, res_rev.dead_dn_r);

    port_debug_printf("\r\nRPM no duty maximo (%d):\r\n", (int)PWM_TEST_MAX_DUTY);
    port_debug_printf("  frente: ESQ=" DEBUG_MILLI_FMT "  DIR=" DEBUG_MILLI_FMT "\r\n",
                      DEBUG_MILLI_ARGS(res_fwd.rpm_max_l),
                      DEBUG_MILLI_ARGS(res_fwd.rpm_max_r));
    port_debug_printf("  re'   : ESQ=" DEBUG_MILLI_FMT "  DIR=" DEBUG_MILLI_FMT "\r\n",
                      DEBUG_MILLI_ARGS(res_rev.rpm_max_l),
                      DEBUG_MILLI_ARGS(res_rev.rpm_max_r));

    /* Casamento entre os lados, em % de diferenca no duty maximo */
    int32_t a = abs32(res_fwd.rpm_max_l);
    int32_t b = abs32(res_fwd.rpm_max_r);
    int32_t maior = (a > b) ? a : b;

    if (maior > 0)
    {
        int32_t diff_pct = (int32_t)(((int64_t)abs32(a - b) * 100LL) / (int64_t)maior);
        port_debug_printf("\r\nCasamento ESQ/DIR: %ld%% de diferenca", (long)diff_pct);

        if (diff_pct > 15)
        {
            port_debug_printf("  [!] alto - o carrinho vai puxar para um lado");
        }

        port_debug_printf("\r\n");
    }

    /* Reta ajustada ao trecho em que a roda gira. O cruzamento com RPM = 0
     * e' o offset real do atuador -- e' esse valor que a compensacao de zona
     * morta tem que cancelar, NAO o duty de partida (que e' atrito estatico
     * e vale so' no instante em que a roda sai da inercia). */
    int32_t z_l, k_l, z_r, k_r;
    bool ok_l = fit_solve(&res_fwd.fit_l, &z_l, &k_l);
    bool ok_r = fit_solve(&res_fwd.fit_r, &z_r, &k_r);

    if (ok_l || ok_r)
    {
        port_debug_printf("\r\nReta ajustada (rpm = a*duty + b):\r\n");

        if (ok_l)
        {
            port_debug_printf("  ESQ: %ld.%03ld RPM por 100 de duty, cruza zero em duty %ld\r\n",
                              (long)(k_l / 10000), (long)((k_l / 10) % 1000), (long)z_l);
        }
        if (ok_r)
        {
            port_debug_printf("  DIR: %ld.%03ld RPM por 100 de duty, cruza zero em duty %ld\r\n",
                              (long)(k_r / 10000), (long)((k_r / 10) % 1000), (long)z_r);
        }

        port_debug_printf("\r\nSugestao para board.h:\r\n");

        if (ok_l) { port_debug_printf("  #define MOTOR_L_DEADBAND   %ld\r\n", (long)z_l); }
        if (ok_r) { port_debug_printf("  #define MOTOR_R_DEADBAND   %ld\r\n", (long)z_r); }
    }

    int32_t db = suggest_deadband();

    if (db > 0)
    {
        port_debug_printf("\r\n(duty de PARTIDA parado: %ld -- nao use como offset)\r\n",
                          (long)db);
    }

    port_debug_printf("==================================================\r\n");
}

/* -------------------------------------------------------------------------- */
/* Transicao de fase                                                           */
/* -------------------------------------------------------------------------- */

static void phase_enter(pwm_phase_t p)
{
    phase  = p;
    sub    = SUB_SETTLE;
    sub_ms = 0u;

    switch (p)
    {
    case PH_FWD_UP:
        result_clear(&res_fwd);
        duty = 0;
        phase_header();
        break;

    case PH_FWD_DOWN:
        duty = PWM_TEST_MAX_DUTY;
        phase_header();
        break;

    case PH_REV_UP:
        result_clear(&res_rev);
        duty = 0;
        phase_header();
        break;

    case PH_REV_DOWN:
        duty = PWM_TEST_MAX_DUTY;
        phase_header();
        break;

    case PH_REPORT:
        port_motor_stop_all();
        print_report();
        break;

    case PH_DONE:
        port_motor_stop_all();
        port_ihm_led_blink(0u);
        port_ihm_led_set(true);
        port_debug_printf("\r\nEnsaio concluido. Motores parados.\r\n");
        break;

    case PH_INTRO:
    default:
        duty = 0;
        port_motor_stop_all();
        break;
    }

    port_encoder_reset();
    apply_duty();
}

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

void pwm_test_init(void)
{
    run = 0u;
    result_clear(&res_fwd);
    result_clear(&res_rev);

    port_ihm_led_blink(500u);

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" ENSAIO DE PWM - duty x velocidade\r\n");
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" degrau ......... %d por-mil\r\n",  (int)PWM_TEST_STEP);
    port_debug_printf(" duty maximo .... %d\r\n",          (int)PWM_TEST_MAX_DUTY);
    port_debug_printf(" por patamar .... %lu ms espera + %lu ms medindo\r\n",
                      (unsigned long)PWM_TEST_SETTLE_MS,
                      (unsigned long)PWM_TEST_MEASURE_MS);
    port_debug_printf(" ticks/volta .... %ld\r\n", (long)ENCODER_TICKS_PER_REV);
    port_debug_printf("\r\n Deixe o carrinho SUSPENSO. As tabelas usam ';'\r\n");
    port_debug_printf(" como separador: da' pra colar direto no Excel.\r\n");

    phase_enter(PH_FWD_UP);
}

void pwm_test_update(void)
{
    if (phase == PH_DONE)
    {
        return;
    }

    if (phase == PH_REPORT)
    {
#if (PWM_TEST_LOOP != 0)
        run++;
        port_debug_printf("\r\n--- reiniciando o ensaio ---\r\n");
        phase_enter(PH_FWD_UP);
#else
        phase_enter(PH_DONE);
#endif
        return;
    }

    sub_ms += CONTROL_PERIOD_MS;

    if (sub == SUB_SETTLE)
    {
        if (sub_ms >= PWM_TEST_SETTLE_MS)
        {
            /* comeca a janela de medicao */
            sub        = SUB_MEASURE;
            sub_ms     = 0u;
            snapshot_l = port_encoder_get_ticks(ENCODER_LEFT);
            snapshot_r = port_encoder_get_ticks(ENCODER_RIGHT);
        }

        return;
    }

    /* SUB_MEASURE */
    if (sub_ms < PWM_TEST_MEASURE_MS)
    {
        return;
    }

    level_finish();

    /* proximo patamar */
    sub    = SUB_SETTLE;
    sub_ms = 0u;

    if (phase_is_ascending())
    {
        if (duty >= PWM_TEST_MAX_DUTY)
        {
            phase_enter((phase == PH_FWD_UP) ? PH_FWD_DOWN : PH_REV_DOWN);
            return;
        }

        duty += PWM_TEST_STEP;

        if (duty > PWM_TEST_MAX_DUTY)
        {
            duty = PWM_TEST_MAX_DUTY;
        }
    }
    else
    {
        if (duty <= 0)
        {
            phase_enter((phase == PH_FWD_DOWN) ? PH_REV_UP : PH_REPORT);
            return;
        }

        duty -= PWM_TEST_STEP;

        if (duty < 0)
        {
            duty = 0;
        }
    }

    apply_duty();
}

const char *pwm_test_phase_name(void)
{
    return phase_name[phase];
}

bool pwm_test_done(void)
{
    return (phase == PH_DONE);
}
