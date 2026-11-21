/**
 ******************************************************************************
 * @file    cal_modes.c
 * @brief   Calibracao dos sensores IR e da geometria do carrinho.
 ******************************************************************************
 */

#include "cal_modes.h"
#include "board.h"
#include "sensors.h"
#include "speed_ctl.h"
#include "motor.h"
#include "debug.h"
#include "ihm.h"

#include <stdbool.h>
#include <math.h>

/* ===========================================================================
 * 1) CALIBRACAO DOS SENSORES
 * ========================================================================= */

static uint32_t sc_acc = 0u;
static uint16_t sc_min[SENSOR_COUNT];
static uint16_t sc_max[SENSOR_COUNT];

void sensor_cal_init(void)
{
    port_motor_stop_all();
    speed_ctl_enable(false);
    port_ihm_led_blink(400u);

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sc_min[i] = 0xFFFFu;
        sc_max[i] = 0u;
    }

    sc_acc = 0u;

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" CALIBRACAO DOS SENSORES IR\r\n");
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" Motores parados. Colunas: cru (filtrado) [min..max]\r\n\r\n");
    port_debug_printf(" 1. Corredor livre, nada perto  -> anote como RAW_FAR\r\n");
    port_debug_printf(" 2. Parede BRANCA na distancia\r\n");
    port_debug_printf("    de referencia               -> anote como RAW_NEAR\r\n");
    port_debug_printf(" 3. Ponha os 8 numeros no board.h\r\n");
    port_debug_printf("--------------------------------------------------\r\n");
}

void sensor_cal_update(void)
{
    sc_acc += NAV_PERIOD_MS;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t v = port_sensors_get_raw((sensor_id_t)i);

        if (v < sc_min[i]) { sc_min[i] = v; }
        if (v > sc_max[i]) { sc_max[i] = v; }
    }

    if (sc_acc < SENSOR_CAL_PRINT_MS)
    {
        return;
    }

    sc_acc = 0u;

    port_debug_printf("\r\n");

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        port_debug_printf("  %-5s cru=%5u  filt=%5u  [%5u..%5u]  prox=%4d\r\n",
                          port_sensors_name((sensor_id_t)i),
                          (unsigned)port_sensors_get_raw((sensor_id_t)i),
                          (unsigned)port_sensors_get_raw_filtered((sensor_id_t)i),
                          (unsigned)sc_min[i],
                          (unsigned)sc_max[i],
                          (int)port_sensors_get_prox((sensor_id_t)i));
    }

    port_debug_printf("  parede: ESQ=%d FRENTE=%d DIR=%d | erro centro=%d\r\n",
                      port_sensors_wall(SENSOR_LEFT)  ? 1 : 0,
                      port_sensors_wall_ahead()       ? 1 : 0,
                      port_sensors_wall(SENSOR_RIGHT) ? 1 : 0,
                      (int)port_sensors_center_error());
}

/* ===========================================================================
 * 1b) CALIBRACAO DO GIRO
 * =========================================================================
 * Quatro giros de 90 graus = uma volta completa. Fecha por DIFERENCA DE TICKS
 * entre as rodas, igual a navegacao faz -- de proposito: calibrar com um
 * mecanismo e navegar com outro seria calibrar a coisa errada.
 */

typedef enum
{
    TC_INTRO = 0,
    TC_STRAIGHT,
    TC_STRAIGHT_PAUSE,
    TC_TURN,
    TC_GAP,
    TC_REPORT
} tc_state_t;

/* Guardados da RETA para serem repetidos no relatorio final.
 *
 * O relatorio da reta e' impresso a 3 s de percurso, com o carrinho andando
 * pelo chao -- exatamente quando nao da' para estar com o fio da serial
 * ligado. Sem guardar, o numero mais util do modo se perde toda vez. No fim
 * da volta o carrinho esta' parado e voce conecta com calma. */
static int32_t tc_st_ticks_l = 0, tc_st_ticks_r = 0;
static int32_t tc_st_duty_l  = 0, tc_st_duty_r  = 0;
static int32_t tc_st_mms_l   = 0, tc_st_mms_r   = 0;
static bool    tc_st_valid   = false;

static tc_state_t tc_state   = TC_INTRO;
static uint32_t   tc_ms      = 0u;
static int32_t    tc_target  = 0;
static int        tc_done    = 0;

static const char *const tc_name[] =
    { "ESPERA", "RETA", "PAUSA-RETA", "GIRO", "PAUSA", "RESULTADO" };

const char *turn_cal_state_name(void)
{
    return tc_name[tc_state];
}

static void tc_enter(tc_state_t s)
{
    tc_state = s;
    tc_ms    = 0u;

    if (s != TC_TURN)
    {
        speed_ctl_stop();
    }
}

void turn_cal_init(void)
{
    speed_ctl_enable(true);
    speed_ctl_stop();
    speed_ctl_reset_odometry();
    port_ihm_led_blink(600u);

    tc_target = 0;
    tc_done   = 0;

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" CALIBRACAO DO GIRO\r\n");
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" TURN_TICKS_PER_90 = %ld  (valor atual do board.h)\r\n",
                      (long)TURN_TICKS_PER_90);
    port_debug_printf("\r\n Ponha o carrinho no chao, com espaco livre em\r\n");
    port_debug_printf(" volta, e MARQUE A POSICAO INICIAL com fita.\r\n");
    port_debug_printf(" Ele vai dar %d giros de 90 graus e parar.\r\n",
                      (int)TURN_CAL_QUARTERS);
    port_debug_printf("--------------------------------------------------\r\n");

    tc_enter(TC_INTRO);
}

void turn_cal_update(void)
{
    tc_ms += NAV_PERIOD_MS;

    switch (tc_state)
    {
    case TC_INTRO:
        speed_ctl_stop();

        if (tc_ms >= 3000u)
        {
            speed_ctl_reset_odometry();
            port_debug_printf("\r\n--- RETA: %lu ms a %ld mm/s ---\r\n",
                              (unsigned long)TURN_CAL_STRAIGHT_MS,
                              (long)TURN_CAL_STRAIGHT_MMS);
            port_debug_printf("    olhe SO' para que lado ele puxa.\r\n");
            tc_enter(TC_STRAIGHT);
        }
        break;

    case TC_STRAIGHT:
        if (tc_ms >= TURN_CAL_STRAIGHT_MS)
        {
            /* Captura ANTES de parar: speed_ctl_stop() zera duty e alvos, e
             * era justamente o esforco aplicado que a gente queria ver. */
            int32_t duty_l = speed_ctl_get_duty(0);
            int32_t duty_r = speed_ctl_get_duty(1);
            int32_t mms_l  = (int32_t)speed_ctl_get_mms(0);
            int32_t mms_r  = (int32_t)speed_ctl_get_mms(1);

            tc_st_ticks_l = speed_ctl_get_fwd_ticks()
                          - speed_ctl_get_turn_ticks() / 2;
            tc_st_ticks_r = speed_ctl_get_fwd_ticks()
                          + speed_ctl_get_turn_ticks() / 2;
            tc_st_duty_l  = duty_l;
            tc_st_duty_r  = duty_r;
            tc_st_mms_l   = mms_l;
            tc_st_mms_r   = mms_r;
            tc_st_valid   = true;

            speed_ctl_stop();

            port_debug_printf("\r\n    ticks: ESQ=%ld  DIR=%ld  (diferenca %ld)\r\n",
                              (long)(speed_ctl_get_fwd_ticks()
                                     - speed_ctl_get_turn_ticks() / 2),
                              (long)(speed_ctl_get_fwd_ticks()
                                     + speed_ctl_get_turn_ticks() / 2),
                              (long)speed_ctl_get_turn_ticks());

            /* O ESFORCO e' o que separa as duas causas de uma roda que conta
             * pouco, e elas pedem consertos opostos:
             *
             *   duty SATURADO (perto de 1000) e ticks baixos -> o PID esta'
             *   empurrando com tudo e a medida nao sobe. A roda provavelmente
             *   GIRA de verdade e quem falha e' a LEITURA sob ruido do motor.
             *
             *   duty NORMAL (uns 300) e ticks baixos -> o controlador esta'
             *   satisfeito, logo a medida bate com a realidade: a roda esta'
             *   travada, patinando, ou o motor nao recebe acionamento.
             *
             * Sem este numero as duas hipoteses produzem o mesmo relatorio. */
            port_debug_printf("    duty:  ESQ=%+ld  DIR=%+ld   "
                              "(esperado ~%ld para %ld mm/s)\r\n",
                              (long)duty_l,
                              (long)duty_r,
                              (long)((TURN_CAL_STRAIGHT_MMS * 60.0f
                                      / WHEEL_CIRC_MM) / SPEED_GAIN_RPM_PER_DUTY),
                              (long)TURN_CAL_STRAIGHT_MMS);
            port_debug_printf("    medido: ESQ=%+ld  DIR=%+ld mm/s\r\n",
                              (long)mms_l,
                              (long)mms_r);
            port_debug_printf("\r\n    andou RETO      -> era o motor, nao mexa\r\n");
            port_debug_printf("    puxou p/ ESQUERDA -> suba WHEEL_R_SCALE\r\n");
            port_debug_printf("    puxou p/ DIREITA  -> suba WHEEL_L_SCALE\r\n");
            port_debug_printf("    atual: L=1.%03ld  R=1.%03ld  (passo 0,02)\r\n",
                              (long)((WHEEL_L_SCALE - 1.0f) * 1000.0f),
                              (long)((WHEEL_R_SCALE - 1.0f) * 1000.0f));
            port_debug_printf("\r\n    Agora reposicione o carrinho: vem a volta.\r\n");
            tc_enter(TC_STRAIGHT_PAUSE);
            break;
        }

        speed_ctl_set_vw(TURN_CAL_STRAIGHT_MMS, 0.0f);
        break;

    case TC_STRAIGHT_PAUSE:
        speed_ctl_stop();

        /* Contagem regressiva na serial. Sem ela esta pausa e' um carrinho
         * parado e um terminal mudo -- indistinguivel de firmware travado, e
         * ja' fez a gente concluir duas vezes que "ele nao faz os giros"
         * quando na verdade era so' esperar. */
        if ((tc_ms % 1000u) == 0u && tc_ms < TURN_CAL_STRAIGHT_PAUSE)
        {
            port_debug_printf("    ... giros em %lu s\r\n",
                              (unsigned long)((TURN_CAL_STRAIGHT_PAUSE - tc_ms)
                                              / 1000u));
        }

        if (tc_ms >= TURN_CAL_STRAIGHT_PAUSE)
        {
            speed_ctl_reset_odometry();
            tc_target = TURN_TICKS_PER_90;
            tc_done   = 0;
            port_debug_printf("\r\n--- giro 1/%d ---\r\n", (int)TURN_CAL_QUARTERS);
            tc_enter(TC_TURN);
        }
        break;

    case TC_TURN:
    {
        int32_t left = tc_target - speed_ctl_get_turn_ticks();
        int32_t mag  = (left < 0) ? -left : left;

        if (mag <= (TURN_TICKS_PER_90 / 30))
        {
            tc_done++;
            port_debug_printf("    parou com %ld ticks (alvo %ld)\r\n",
                              (long)speed_ctl_get_turn_ticks(),
                              (long)tc_target);
            tc_enter((tc_done >= TURN_CAL_QUARTERS) ? TC_REPORT : TC_GAP);
            break;
        }

        float speed = TURN_CAL_SPEED_MMS;
        const int32_t slow_zone = (TURN_TICKS_PER_90 / 4);

        if (mag < slow_zone)
        {
            speed *= ((float)mag / (float)slow_zone);
            if (speed < 40.0f) { speed = 40.0f; }
        }

        speed_ctl_set_vw(0.0f, (left > 0) ? speed : -speed);
        break;
    }

    case TC_GAP:
        speed_ctl_stop();

        if (tc_ms >= TURN_CAL_GAP_MS)
        {
            tc_target += TURN_TICKS_PER_90;
            port_debug_printf("\r\n--- giro %d/%d ---\r\n",
                              tc_done + 1, (int)TURN_CAL_QUARTERS);
            tc_enter(TC_TURN);
        }
        break;

    case TC_REPORT:
    default:
        speed_ctl_stop();

        if (tc_ms == NAV_PERIOD_MS)   /* imprime uma vez so' ao entrar */
        {
            port_debug_printf("\r\n==================================================\r\n");
            port_debug_printf(" VOLTA COMPLETA -- compare com a fita\r\n");
            port_debug_printf("==================================================\r\n");
            port_debug_printf(" TURN_TICKS_PER_90 usado: %ld\r\n",
                              (long)TURN_TICKS_PER_90);

            if (tc_st_valid)
            {
                port_debug_printf("\r\n A RETA do inicio, repetida aqui porque\r\n");
                port_debug_printf(" com o carrinho andando nao da' para ler:\r\n");
                port_debug_printf("   ticks:  ESQ=%ld  DIR=%ld\r\n",
                                  (long)tc_st_ticks_l, (long)tc_st_ticks_r);
                port_debug_printf("   duty:   ESQ=%+ld  DIR=%+ld  (esperado ~%ld)\r\n",
                                  (long)tc_st_duty_l, (long)tc_st_duty_r,
                                  (long)((TURN_CAL_STRAIGHT_MMS * 60.0f
                                          / WHEEL_CIRC_MM)
                                         / SPEED_GAIN_RPM_PER_DUTY));
                port_debug_printf("   medido: ESQ=%+ld  DIR=%+ld mm/s\r\n",
                                  (long)tc_st_mms_l, (long)tc_st_mms_r);
            }
            port_debug_printf("\r\n Quanto ele girou DE VERDADE?\r\n");
            port_debug_printf("   voltou para a fita  -> esta' calibrado\r\n");
            port_debug_printf("   faltou para chegar  -> AUMENTE o numero\r\n");
            port_debug_printf("   passou da fita      -> DIMINUA\r\n");
            port_debug_printf("\r\n Tabela pronta (novo = %ld * 360/girado):\r\n",
                              (long)TURN_TICKS_PER_90);

            {
                static const int32_t medidos[] =
                    { 240, 270, 300, 330, 360, 390, 420, 450, 540 };

                for (unsigned i = 0; i < sizeof(medidos)/sizeof(medidos[0]); i++)
                {
                    port_debug_printf("   girou %3ld graus -> TURN_TICKS_PER_90 = %ld\r\n",
                                      (long)medidos[i],
                                      (long)((int32_t)TURN_TICKS_PER_90 * 360
                                             / medidos[i]));
                }
            }

            port_debug_printf("\r\n Repete em %lu ms.\r\n",
                              (unsigned long)TURN_CAL_REPORT_MS);
        }

        if (tc_ms >= TURN_CAL_REPORT_MS)
        {
            speed_ctl_reset_odometry();
            tc_target = TURN_TICKS_PER_90;
            tc_done   = 0;
            port_debug_printf("\r\n--- giro 1/%d ---\r\n", (int)TURN_CAL_QUARTERS);
            tc_enter(TC_TURN);
        }
        break;
    }
}

/* ===========================================================================
 * 2) CALIBRACAO DA ODOMETRIA
 * ========================================================================= */

typedef enum
{
    OC_INTRO = 0,
    OC_STRAIGHT,
    OC_PAUSE_STRAIGHT,
    OC_SPIN,
    OC_PAUSE_SPIN
} oc_state_t;

static oc_state_t oc_state = OC_INTRO;
static uint32_t   oc_ms    = 0u;

static const char *const oc_name[] =
{
    "ESPERA", "RETA", "PAUSA-RETA", "GIRO", "PAUSA-GIRO"
};

const char *odom_cal_state_name(void)
{
    return oc_name[oc_state];
}

static void oc_enter(oc_state_t s)
{
    oc_state = s;
    oc_ms    = 0u;

    switch (s)
    {
    case OC_STRAIGHT:
        speed_ctl_reset_odometry();
        port_debug_printf("\r\n--- RETA: %ld mm comandados ---\r\n",
                          (long)ODOM_CAL_DISTANCE_MM);
        break;

    case OC_SPIN:
        speed_ctl_reset_odometry();
        port_debug_printf("\r\n--- GIRO: %ld voltas comandadas ---\r\n",
                          (long)ODOM_CAL_TURNS);
        port_debug_printf("    marque o chao com fita antes de comecar\r\n");
        break;

    default:
        speed_ctl_stop();
        break;
    }
}

void odom_cal_init(void)
{
    speed_ctl_enable(true);
    speed_ctl_stop();
    port_ihm_led_blink(600u);

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" CALIBRACAO DA GEOMETRIA\r\n");
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" valores atuais do board.h:\r\n");
    port_debug_printf("   WHEEL_DIAMETER_MM = %ld\r\n", (long)WHEEL_DIAMETER_MM);
    port_debug_printf("   WHEEL_BASE_MM     = %ld\r\n", (long)WHEEL_BASE_MM);
    port_debug_printf("   mm por tick       = 0.%03ld\r\n",
                      (long)(MM_PER_TICK * 1000.0f));
    port_debug_printf("\r\n Ponha o carrinho no chao com espaco livre.\r\n");
    port_debug_printf(" Alterna RETA e GIRO, com %lu ms de pausa para medir.\r\n",
                      (unsigned long)ODOM_CAL_PAUSE_MS);

    oc_enter(OC_INTRO);
}

void odom_cal_update(void)
{
    oc_ms += NAV_PERIOD_MS;

    switch (oc_state)
    {
    case OC_INTRO:
        speed_ctl_stop();

        if (oc_ms >= 3000u)
        {
#if (ODOM_CAL_SKIP_STRAIGHT != 0)
            oc_enter(OC_SPIN);
#else
            oc_enter(OC_STRAIGHT);
#endif
        }
        break;

    case OC_STRAIGHT:
    {
        float d = speed_ctl_get_distance_mm();

        if (d >= ODOM_CAL_DISTANCE_MM)
        {
            speed_ctl_stop();
            port_debug_printf("    parou com %ld mm no odometro\r\n", (long)d);
            port_debug_printf("    MECA a distancia real com regua.\r\n\r\n");
            port_debug_printf("    Duas constantes entram nessa conta, e as\r\n");
            port_debug_printf("    duas corrigem o erro igualmente na formula.\r\n");
            port_debug_printf("    Corrija a que estiver ERRADA DE FATO:\r\n\r\n");
            port_debug_printf("    1) Meca a roda com paquimetro. Se nao for\r\n");
            port_debug_printf("       %ld mm, corrija WHEEL_DIAMETER_MM e\r\n",
                              (long)WHEEL_DIAMETER_MM);
            port_debug_printf("       rode de novo antes de mexer no resto.\r\n\r\n");
            port_debug_printf("    2) Com o diametro certo e a distancia ainda\r\n");
            port_debug_printf("       errada, o erro esta' na reducao da caixa:\r\n");
            port_debug_printf("         ENCODER_GEAR_RATIO = %ld * (%ld / medido)\r\n",
                              (long)ENCODER_GEAR_RATIO, (long)ODOM_CAL_DISTANCE_MM);
            port_debug_printf("       (andou MENOS que o comandado = reducao\r\n");
            port_debug_printf("        maior do que o board.h supoe)\r\n");
            oc_enter(OC_PAUSE_STRAIGHT);
            break;
        }

        /* desacelera nos ultimos 80 mm para nao passar do ponto */
        float left = ODOM_CAL_DISTANCE_MM - d;
        float v = ODOM_CAL_SPEED_MMS;

        if (left < 80.0f)
        {
            v *= (left / 80.0f);
            if (v < 60.0f) { v = 60.0f; }
        }

        speed_ctl_set_vw(v, 0.0f);
        break;
    }

    case OC_PAUSE_STRAIGHT:
        speed_ctl_stop();

        if (oc_ms >= ODOM_CAL_PAUSE_MS)
        {
            oc_enter(OC_SPIN);
        }
        break;

    case OC_SPIN:
    {
        float target = ODOM_CAL_TURNS * 360.0f;
        float a = speed_ctl_get_heading_deg();

        if (a >= target)
        {
            speed_ctl_stop();
            port_debug_printf("    o odometro marcou %ld graus\r\n", (long)a);
            port_debug_printf("    MECA quantos graus ele girou DE VERDADE.\r\n\r\n");
            port_debug_printf("    theta = (mm_dir - mm_esq) / base. Girar MENOS\r\n");
            port_debug_printf("    que o comandado quer dizer que o firmware\r\n");
            port_debug_printf("    superestima a rotacao, ou seja, esta' dividindo\r\n");
            port_debug_printf("    por uma base PEQUENA demais:\r\n\r\n");
            port_debug_printf("      WHEEL_BASE_MM = %ld * (%ld / girado_real)\r\n",
                              (long)WHEEL_BASE_MM, (long)target);
            port_debug_printf("\r\n    Tabela pronta com a base atual de %ld mm:\r\n",
                              (long)WHEEL_BASE_MM);

            {
                static const int32_t medidos[] = { 180, 240, 270, 300, 330, 360, 400 };

                for (unsigned i = 0; i < sizeof(medidos)/sizeof(medidos[0]); i++)
                {
                    if ((float)medidos[i] > target * 1.3f) { continue; }

                    port_debug_printf("      girou %3ld graus -> base = %ld\r\n",
                                      (long)medidos[i],
                                      (long)(WHEEL_BASE_MM * target / (float)medidos[i]));
                }
            }

            port_debug_printf("\r\n    Repita depois de corrigir: deve convergir\r\n");
            port_debug_printf("    para fechar a volta em 2 ou 3 iteracoes.\r\n");
            oc_enter(OC_PAUSE_SPIN);
            break;
        }

        float left = target - a;
        float v = ODOM_CAL_TURN_MMS;

        if (left < 40.0f)
        {
            v *= (left / 40.0f);
            if (v < 50.0f) { v = 50.0f; }
        }

        speed_ctl_set_vw(0.0f, v);
        break;
    }

    case OC_PAUSE_SPIN:
    default:
        speed_ctl_stop();

        if (oc_ms >= ODOM_CAL_PAUSE_MS)
        {
#if (ODOM_CAL_SKIP_STRAIGHT != 0)
            oc_enter(OC_SPIN);       /* so' giro, repetindo */
#else
            oc_enter(OC_STRAIGHT);   /* repete o ciclo completo */
#endif
        }
        break;
    }
}
