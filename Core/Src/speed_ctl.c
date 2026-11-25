/**
 ******************************************************************************
 * @file    speed_ctl.c
 * @brief   PID de velocidade por roda + odometria.
 ******************************************************************************
 */

#include "speed_ctl.h"
#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "debug.h"

#define WHEELS  2

/* ---------------------------------------------------------------------------
 * Estado
 * ------------------------------------------------------------------------- */

typedef struct
{
    float   target_mms;
    float   meas_mms;
    float   integral;       /* em unidades de duty */
    float   prev_error;
    int32_t duty;
    uint32_t silent_ms;     /* ha' quanto tempo esta' comandada e muda    */
    uint32_t wrong_ms;      /* ha' quanto tempo anda para o lado errado   */
} wheel_ctl_t;

static wheel_ctl_t wheel[WHEELS];
static bool        enabled    = false;
static int         fault      = 0;
static int         fault_sign = 0;   /* falha foi de SENTIDO, nao de silencio */

/* odometria */
static float odo_distance_mm = 0.0f;
static float odo_heading_deg = 0.0f;

/* Contagem CRUA acumulada, sem passar por milimetro nenhum.
 * A navegacao fecha o giro por aqui de proposito: assim o angulo nao depende
 * do diametro da roda, da distancia entre elas nem da assimetria dos
 * encoders -- tres numeros que aqui nao estao medidos com confianca. */
static int32_t odo_ticks_l = 0;
static int32_t odo_ticks_r = 0;

/* Conversao de mm/s para RPM da roda:
 *   rpm = (mm/s) / circunferencia * 60 */
#define MMS_TO_RPM(v)   ((v) * 60.0f / WHEEL_CIRC_MM)

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/**
 * Realimentacao direta: duty necessario para uma velocidade alvo, tirado da
 * reta medida no ensaio de PWM. A zona morta ja' e' cancelada dentro do
 * driver do motor, entao aqui basta a inclinacao.
 */
static float feedforward_duty(float target_mms)
{
    float rpm = MMS_TO_RPM(target_mms);
    float ff  = rpm / SPEED_GAIN_RPM_PER_DUTY;

#ifdef SPEED_FF_STATIC
    /* atrito parado: um empurrao fixo no sentido do movimento */
    if (target_mms > 0.0f)      { ff += SPEED_FF_STATIC; }
    else if (target_mms < 0.0f) { ff -= SPEED_FF_STATIC; }
#endif
    return ff;
}

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

void speed_ctl_init(void)
{
    for (int i = 0; i < WHEELS; i++)
    {
        wheel[i].target_mms = 0.0f;
        wheel[i].meas_mms   = 0.0f;
        wheel[i].integral   = 0.0f;
        wheel[i].prev_error = 0.0f;
        wheel[i].duty       = 0;
        wheel[i].silent_ms  = 0u;
        wheel[i].wrong_ms   = 0u;
    }

    enabled    = false;
    fault      = 0;
    fault_sign = 0;
    speed_ctl_reset_odometry();
    port_motor_stop_all();
}

void speed_ctl_set(float left_mms, float right_mms)
{
    wheel[0].target_mms = left_mms;
    wheel[1].target_mms = right_mms;
}

void speed_ctl_set_vw(float linear_mms, float turn_mms)
{
    speed_ctl_set(linear_mms - turn_mms, linear_mms + turn_mms);
}

void speed_ctl_stop(void)
{
    speed_ctl_set(0.0f, 0.0f);

    for (int i = 0; i < WHEELS; i++)
    {
        wheel[i].integral   = 0.0f;
        wheel[i].prev_error = 0.0f;
        wheel[i].duty       = 0;
        wheel[i].silent_ms  = 0u;
        wheel[i].wrong_ms   = 0u;
    }

    port_motor_stop_all();
}

void speed_ctl_enable(bool on)
{
    if (on != enabled)
    {
        speed_ctl_stop();
    }

    enabled = on;
}

void speed_ctl_update(void)
{
    const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;

#if !BOARD_USE_ENCODERS
    /* --- SEM ENCODER: medida "virtual" --------------------------------------
     * Supoe que cada roda anda exatamente o que foi comandado. Assim o erro
     * do PID e' zero (sai so' o feedforward = malha aberta) e a odometria e'
     * estimada integrando a velocidade alvo. Os ticks virtuais acumulam a
     * fracao para nao perder distancia a 1 kHz. */
    static float virt_l = 0.0f;
    static float virt_r = 0.0f;
    wheel[0].meas_mms = wheel[0].target_mms;
    wheel[1].meas_mms = wheel[1].target_mms;
    virt_l += (wheel[0].target_mms * dt) / MM_PER_TICK;
    virt_r += (wheel[1].target_mms * dt) / MM_PER_TICK;
    int32_t dl = (int32_t)virt_l;
    int32_t dr = (int32_t)virt_r;
    virt_l -= (float)dl;
    virt_r -= (float)dr;
#else
    /* --- medida: delta de ticks do ultimo milissegundo vira mm/s --- */
    int32_t dl = port_encoder_get_delta(ENCODER_LEFT);
    int32_t dr = port_encoder_get_delta(ENCODER_RIGHT);
#endif

    odo_ticks_l += dl;
    odo_ticks_r += dr;

    /* O trim por roda entra AQUI, na conversao de ticks para milimetros:
     * a partir deste ponto todo o resto do firmware (PID, odometria,
     * navegacao) ja' trabalha com distancia real de chao. */
    float mm_l = (float)dl * MM_PER_TICK * WHEEL_L_SCALE;
    float mm_r = (float)dr * MM_PER_TICK * WHEEL_R_SCALE;

    /* Filtro leve: a 1 kHz um delta de encoder e' um numero pequeno e
     * granular (0, 1, 2 ticks), entao a velocidade instantanea e' ruidosa.
     * Uma media exponencial de ~20 ms limpa isso sem atrasar o controle. */
#if BOARD_USE_ENCODERS
    const float alpha = 0.05f;
    wheel[0].meas_mms += ((mm_l / dt) - wheel[0].meas_mms) * alpha;
    wheel[1].meas_mms += ((mm_r / dt) - wheel[1].meas_mms) * alpha;
#endif

    /* --- odometria --- */
    float mm_center = (mm_l + mm_r) * 0.5f;
    odo_distance_mm += mm_center;

    /* rotacao: diferenca entre as rodas dividida pela base */
    float d_theta_rad = (mm_r - mm_l) / WHEEL_BASE_MM;
    odo_heading_deg  += d_theta_rad * (180.0f / 3.14159265f);

    if (!enabled || fault != 0)
    {
        return;
    }

    /* --- vigilancia de roda muda --- */
#if BOARD_USE_ENCODERS      /* sem encoder nao ha' o que vigiar */
    for (int i = 0; i < WHEELS; i++)
    {
        wheel_ctl_t *w = &wheel[i];

        float at = (w->target_mms < 0.0f) ? -w->target_mms : w->target_mms;
        float am = (w->meas_mms   < 0.0f) ? -w->meas_mms   : w->meas_mms;
        int32_t ad = (w->duty < 0) ? -w->duty : w->duty;

        if (at >= SPEED_FAULT_MIN_TARGET
            && ad >= SPEED_FAULT_MIN_DUTY
            && am <  SPEED_FAULT_MOVING_MMS)
        {
            w->silent_ms += CONTROL_PERIOD_MS;

            if (w->silent_ms >= SPEED_FAULT_MS)
            {
                fault |= (1 << i);
            }
        }
        else
        {
            w->silent_ms = 0u;
        }

        /* --- vigilancia de SENTIDO INVERTIDO -------------------------------
         * A roda esta' andando com convicção para o lado OPOSTO ao comandado.
         * Isso nao e' erro de sintonia -- nenhum ganho de PID faz uma roda
         * andar ao contrario. E' sinal de MOTOR_*_INVERT e ENCODER_*_INVERT
         * discordando: a medida chega com o sinal trocado, a realimentacao
         * vira POSITIVA e o duty foge para a saturacao em vez de estabilizar.
         *
         * Deixado solto, o carrinho sai em disparada com esforco maximo. Meio
         * segundo e' tempo de sobra para distinguir isso de um transitorio de
         * partida ou de uma inversao de sentido comandada. */
        if (at >= SPEED_FAULT_MIN_TARGET
            && am >= SPEED_FAULT_MOVING_MMS
            && ((w->target_mms > 0.0f) != (w->meas_mms > 0.0f)))
        {
            w->wrong_ms += CONTROL_PERIOD_MS;

            if (w->wrong_ms >= SPEED_FAULT_WRONG_MS)
            {
                fault      |= (1 << i);
                fault_sign |= (1 << i);
            }
        }
        else
        {
            w->wrong_ms = 0u;
        }
    }

    if (fault_sign != 0)
    {
        port_motor_stop_all();

        port_debug_printf("\r\n*** FALHA: roda %s girando ao CONTRARIO ***\r\n",
                          (fault_sign == 1) ? "ESQUERDA"
                        : (fault_sign == 2) ? "DIREITA" : "ESQ e DIR");
        port_debug_printf("  Comandada para um lado, o encoder acusou o outro\r\n");
        port_debug_printf("  por %lu ms. Motores parados antes de disparar.\r\n",
                          (unsigned long)SPEED_FAULT_WRONG_MS);
        port_debug_printf("  Isso e' realimentacao POSITIVA, nao ma' sintonia.\r\n");
        port_debug_printf("  No board.h, para a(s) roda(s) acusada(s), troque\r\n");
        port_debug_printf("  0 <-> 1 em UM dos dois, nao nos dois:\r\n");
        port_debug_printf("   - ENCODER_?_INVERT  (se a roda gira p/ o lado certo)\r\n");
        port_debug_printf("   - MOTOR_?_INVERT    (se ela gira p/ o lado errado)\r\n\r\n");
        return;
    }

    if (fault != 0)
    {
        port_motor_stop_all();

        port_debug_printf("\r\n*** FALHA: roda %s sem leitura de encoder ***\r\n",
                          (fault == 1) ? "ESQUERDA"
                        : (fault == 2) ? "DIREITA" : "ESQ e DIR");
        port_debug_printf("  A roda foi comandada com esforco e o encoder\r\n");
        port_debug_printf("  nao acusou movimento por %lu ms. Motores parados.\r\n",
                          (unsigned long)SPEED_FAULT_MS);
        port_debug_printf("  Verifique, nesta ordem:\r\n");
        port_debug_printf("   1. fios do encoder (canais A e B) na placa\r\n");
        port_debug_printf("   2. alimentacao do modulo do encoder\r\n");
        port_debug_printf("   3. a roda gira livre? o motor esta' preso?\r\n");
        port_debug_printf("  Confirme com APP_MODE_ENCODER_CAL: motores\r\n");
        port_debug_printf("  desligados, gire cada roda na mao e veja os ticks.\r\n\r\n");
        return;
    }
#endif /* BOARD_USE_ENCODERS */

    /* --- controle --- */
    for (int i = 0; i < WHEELS; i++)
    {
        wheel_ctl_t *w = &wheel[i];

        if (w->target_mms == 0.0f)
        {
            /* Alvo zero e' parada de verdade: sem integral acumulando,
             * senao o robo "empurra" parado e sai andando ao religar. */
            w->integral = 0.0f;
            w->duty     = 0;
            port_motor_set((motor_id_t)i, 0);
            continue;
        }

        float error_mms = w->target_mms - w->meas_mms;
        float error_rpm = MMS_TO_RPM(error_mms);

        w->integral += error_rpm * PID_KI * dt;
        w->integral  = clampf(w->integral, -PID_I_LIMIT, PID_I_LIMIT);

        float derivative = (error_rpm - w->prev_error) / dt;
        w->prev_error = error_rpm;

        float out = feedforward_duty(w->target_mms)
                  + (error_rpm * PID_KP)
                  + w->integral
                  + (derivative * PID_KD);

        out = clampf(out, -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX);

        w->duty = (int32_t)out;
        port_motor_set((motor_id_t)i, w->duty);
    }
}

/* ---------------------------------------------------------------------------
 * Leitura de estado
 * ------------------------------------------------------------------------- */

float speed_ctl_get_mms(int w)
{
    return (w >= 0 && w < WHEELS) ? wheel[w].meas_mms : 0.0f;
}

float speed_ctl_get_target_mms(int w)
{
    return (w >= 0 && w < WHEELS) ? wheel[w].target_mms : 0.0f;
}

int32_t speed_ctl_get_duty(int w)
{
    return (w >= 0 && w < WHEELS) ? wheel[w].duty : 0;
}

float speed_ctl_get_distance_mm(void)
{
    return odo_distance_mm;
}

float speed_ctl_get_heading_deg(void)
{
    return odo_heading_deg;
}

int speed_ctl_get_fault(void)
{
    return fault;
}

void speed_ctl_clear_fault(void)
{
    fault      = 0;
    fault_sign = 0;

    for (int i = 0; i < WHEELS; i++)
    {
        wheel[i].silent_ms = 0u;
        wheel[i].wrong_ms  = 0u;
    }
}

int32_t speed_ctl_get_turn_ticks(void)
{
    return odo_ticks_r - odo_ticks_l;
}

/* Rumo pelos encoders (SPEED_HOLD_KP no board.h) */
static int32_t hold_ref = 0;

void speed_ctl_hold_mark(void)
{
    hold_ref = odo_ticks_r - odo_ticks_l;
}

float speed_ctl_hold_turn(void)
{
    /* + = roda direita andou mais = robo virou p/ a esquerda -> volta p/ a
     * direita (turn negativo acelera a esquerda e freia a direita) */
    float err_mm = (float)((odo_ticks_r - odo_ticks_l) - hold_ref) * MM_PER_TICK;
    float t = -SPEED_HOLD_KP * err_mm;
    if (t >  SPEED_HOLD_MAX_MMS) { t =  SPEED_HOLD_MAX_MMS; }
    if (t < -SPEED_HOLD_MAX_MMS) { t = -SPEED_HOLD_MAX_MMS; }
    return t;
}

int32_t speed_ctl_get_fwd_ticks(void)
{
    return (odo_ticks_r + odo_ticks_l) / 2;
}

void speed_ctl_reset_odometry(void)
{
    odo_distance_mm = 0.0f;
    odo_heading_deg = 0.0f;
    odo_ticks_l     = 0;
    odo_ticks_r     = 0;
    port_encoder_reset();
}
