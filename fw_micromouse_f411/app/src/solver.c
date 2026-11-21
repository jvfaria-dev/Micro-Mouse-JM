/**
 ******************************************************************************
 * @file    solver.c
 * @brief   Resolucao do labirinto: movimento celula a celula + flood fill.
 *          A logica do mapa esta' em maze.c; aqui fica so' o movimento e a
 *          maquina de fases. Ver solver.h.
 ******************************************************************************
 */

#include "solver.h"
#include "maze.h"
#include "sensors.h"
#include "speed_ctl.h"
#include "debug.h"
#include "ihm.h"
#include "imu.h"

#include <stdbool.h>
#include <stddef.h>

/* ===========================================================================
 * Estado
 * ======================================================================== */

typedef enum
{
    ST_WAIT = 0,    /* contagem antes de sair                   */
    ST_SENSE,       /* parado no centro: le', mapeia, decide    */
    ST_TURN,        /* gira no lugar                            */
    ST_MOVE,        /* anda N celulas                           */
    ST_ALIGN,       /* acerta a distancia pela parede da frente */
    ST_PAUSE,       /* pausa entre fases                        */
    ST_DONE,
    ST_ABORT,
    ST_UNSTICK      /* recua um pouco e retoma o que fazia      */
} st_t;

typedef enum { PH_EXPLORE = 0, PH_RETURN, PH_RUN } phase_t;

static const char *const st_name[] =
    { "ESPERA", "LE", "GIRA", "ANDA", "ALINHA", "PAUSA", "FIM", "ABORTOU", "DESENCALHA" };
static const char *const ph_name[] = { "EXPLORA", "VOLTA", "CORRE" };
static const char dir_ch[4] = { 'N', 'L', 'S', 'O' };   /* norte leste sul oeste */

static st_t     st       = ST_WAIT;
static uint32_t st_ms    = 0u;
static phase_t  phase    = PH_EXPLORE;

static uint8_t    cx = 0u, cy = 0u;     /* celula atual            */
static maze_dir_t ch = DIR_N;           /* para onde esta' virado  */
static uint8_t    sx = 0u, sy = 0u;     /* largada                 */

static int       steps        = 0;
static maze_dir_t want_dir    = DIR_N;
static uint8_t   move_cells   = 1u;
static int32_t   move_start   = 0;
static int32_t   move_target  = 0;
static float     creep_mm     = 0.0f;
static bool      aligned      = false;  /* ja' alinhou nesta celula    */
static int32_t   align_start  = 0;
static bool      edge_l = false, edge_r = false;
static uint16_t  edge_fixes   = 0u;
/* erro medio das bordas, por tipo (0 = parede comecando, 1 = acabando) */
static float     edge_err_sum[2] = { 0.0f, 0.0f };
static uint16_t  edge_err_n[2]   = { 0u, 0u };
static float     edge_ema[2]     = { 0.0f, 0.0f };
static float     edge_beam       = 0.0f;    /* meia largura do feixe, mm */
static uint16_t  align_count  = 0u;

/* ---------------------------------------------------------------------------
 * Escala da odometria, aprendida no caminho.
 *
 * Se MM_PER_TICK estiver errado (roda com outro diametro, reducao diferente
 * da de catalogo), o robo para antes ou depois do centro em toda celula sem
 * parede na frente, e o erro soma. Em vez de depender so' da calibracao, o
 * robo mede a escala real com o que o labirinto da' de graca:
 *   - parada pelo sensor frontal: andou exatamente n celulas;
 *   - borda de parede lateral: esta' exatamente numa divisa de celulas.
 * odo_k multiplica MM_PER_TICK. 1,0 = calibracao certa.
 * ------------------------------------------------------------------------- */
static float     odo_k        = 1.0f;
static int32_t   raw_start    = 0;      /* ticks na saida, sem correcao  */
/* telemetria do ultimo movimento (vai na linha do log da celula) */
static uint8_t   mv_fix_n     = 0u;     /* bordas usadas neste movimento */
static float     mv_fix_mm    = 0.0f;   /* soma das correcoes, mm         */
static int16_t   mv_raw_mm    = -1;     /* encoder cru do ultimo mov., mm */

/* Ultima posicao ABSOLUTA conhecida dentro do movimento atual (borda de
 * parede). A escala so' e' aprendida entre duas posicoes absolutas -- o
 * ponto de partida pode estar fora do centro e contaminaria a medida. */
static bool      fix_valid    = false;
static float     fix_real_mm  = 0.0f;   /* posicao do eixo desde a saida */
static int32_t   fix_raw      = 0;      /* ticks crus naquele instante   */

static float mm_per_tick(void)
{
    return MM_PER_TICK * odo_k;
}

static void odo_learn(float k_meas, float alpha)
{
#if (SOLVER_ODO_LEARN == 0)
    (void)k_meas; (void)alpha;
    return;
#endif
    if (k_meas < 0.6f || k_meas > 1.6f)
    {
        return;                         /* medida absurda: ignora */
    }
    odo_k += (k_meas - odo_k) * alpha;
    if (odo_k < SOLVER_ODO_K_MIN) { odo_k = SOLVER_ODO_K_MIN; }
    if (odo_k > SOLVER_ODO_K_MAX) { odo_k = SOLVER_ODO_K_MAX; }
}
static int32_t   turn_target  = 0;
static const char *abort_why  = "";

/* ticks de cada roda na entrada do estado (diagnostico) */
static int32_t   st_l0 = 0, st_r0 = 0;

/* detector de giro parado */
static int32_t   stall_ref = 0;
static uint32_t  stall_ms  = 0u;

/* desencalhe */
static st_t      resume_st     = ST_SENSE;
static uint32_t  resume_ms     = 0u;
static int32_t   unstick_start = 0;
static float     unstick_dir   = -1.0f;
static uint8_t   unstick_cell  = 0u;    /* tentativas nesta celula   */
static uint16_t  unstick_total = 0u;    /* no percurso todo          */
/* desvio de pino (SOLVER_SIDESTEP): 0 = recuo simples, +1 esq, -1 dir */
static int8_t    us_side       = 0;
static uint8_t   us_phase      = 0u;    /* 0 recua, 1 vira, 2 anda, 3 desvira */
static uint32_t  us_ms         = 0u;
static float     us_head0      = 0.0f;
/* ajuste pela parede da frente nesta celula (mm, -1 = nao houve) */
static int16_t   falign_mm     = -1;

/* ---------------------------------------------------------------------------
 * Rumo pelo giroscopio. Tudo em graus, + = esquerda.
 *   tgt_head   rumo que o robo deveria ter (0 na largada, +-90 a cada giro)
 *   head_bias  correcao da referencia (deriva do giroscopio), aprendida
 *              andando paralelo entre duas paredes
 * ------------------------------------------------------------------------- */
static bool      use_imu      = false;
static bool      imu_cal_done = false;
static uint8_t   imu_cal_tries = 0u;
static bool      imu_checked  = false;
/* largada: confere uma vez se o robo foi posto de frente para a saida */
static bool      start_checked = false;
static bool      start_turn    = false;   /* giro so' de orientacao, sem mudar ch */
static float     imu_sign     = 1.0f;
static float     tgt_head     = 0.0f;
static float     head_bias    = 0.0f;
static int32_t   turn_tt0     = 0;
static float     turn_gh0     = 0.0f;
static uint32_t  settle_ms    = 0u;
static float     turn_err_max = 0.0f;
static uint16_t  turn_fixes   = 0u;     /* correcoes finas de giro */

static float absf(float v) { return (v < 0.0f) ? -v : v; }

static float gyro_head(void)
{
    return imu_sign * port_imu_get_heading_deg();
}

/** Erro de rumo (graus): quanto falta girar para a esquerda. */
static float head_err(void)
{
    return (tgt_head + head_bias) - gyro_head();
}

/** Termo de giro (mm/s) para segurar o rumo andando reto. */
static float head_turn(void)
{
    if (!use_imu)
    {
        return 0.0f;
    }
    float t = head_err() * SOLVER_HEADING_KP;
    if (t >  SOLVER_HEADING_MAX_MMS) { t =  SOLVER_HEADING_MAX_MMS; }
    if (t < -SOLVER_HEADING_MAX_MMS) { t = -SOLVER_HEADING_MAX_MMS; }
    return t;
}

/* centralizador (P+D), mesma ideia do nav.c */
static float wf_prev  = 0.0f;
static float wf_deriv = 0.0f;

/* ---------------------------------------------------------------------------
 * Registro em RAM (sobrevive ao fim do percurso; RESET apaga)
 * ------------------------------------------------------------------------- */
#define LOG_N   400

typedef struct
{
    uint8_t x, y, h;
    uint8_t walls;      /* bit0 esquerda, bit1 frente, bit2 direita        */
    uint8_t phase;
    uint8_t dir;        /* direcao escolhida (0..3), 0xFF = nenhuma        */
    uint8_t n;          /* celulas pedidas                                  */
    uint8_t note;       /* 0 ok, 1 bloqueio na saida, 2 andou menos         */
} log_t;

static log_t    lg[LOG_N];
static uint16_t lg_n = 0u;

/* ===========================================================================
 * Helpers
 * ======================================================================== */

static int32_t ticks_l(void)
{
    return speed_ctl_get_fwd_ticks() - speed_ctl_get_turn_ticks() / 2;
}

static int32_t ticks_r(void)
{
    return speed_ctl_get_fwd_ticks() + speed_ctl_get_turn_ticks() / 2;
}

static void enter(st_t s)
{
    st        = s;
    st_ms     = 0u;
    st_l0     = ticks_l();
    st_r0     = ticks_r();
    stall_ref = speed_ctl_get_turn_ticks();
    stall_ms  = 0u;
}

static void out(const char *s)
{
    port_debug_write(s);
}

static void drive_centered(float v)
{
    float turn;
    bool  wl = port_sensors_wall(SENSOR_LEFT);
    bool  wr = port_sensors_wall(SENSOR_RIGHT);
    bool  colado = (port_sensors_get_prox(SENSOR_LEFT)  > SOLVER_SIDE_CLOSE_PROX)
                || (port_sensors_get_prox(SENSOR_RIGHT) > SOLVER_SIDE_CLOSE_PROX);

    if (wl || wr)
    {
        const float dt = (float)NAV_PERIOD_MS / 1000.0f;
        float e  = (float)port_sensors_center_error();
        float de = (e - wf_prev) / dt;
        wf_prev  = e;
        wf_deriv += (de - wf_deriv) * WALL_FOLLOW_D_ALPHA;
        turn = (e * WALL_FOLLOW_KP) + (wf_deriv * WALL_FOLLOW_KD);

        /* Com parede do lado, quem manda e' a parede. A referencia de rumo
         * do giroscopio vai sendo puxada para o rumo atual -- devagar com
         * uma parede, rapido com duas e o robo paralelo a elas (distancia
         * que nao muda). Assim a deriva e o erro de escala do giroscopio
         * nunca brigam com o centralizador, e nas celulas abertas o robo
         * segura o ultimo rumo que as paredes confirmaram. */
        float v_med = 0.5f * (speed_ctl_get_mms(0) + speed_ctl_get_mms(1));
        float h_err = gyro_head() - (tgt_head + head_bias);
        /* duas paredes e paralelo: o robo esta' certamente no rumo da
         * celula -- aceita corrigir erro maior (escala acumulada nos giros) */
        bool  par2 = wl && wr && absf(wf_deriv) < SOLVER_HEADING_PARALLEL;
        float gate = par2 ? SOLVER_HEADING_ABSORB2_DEG : SOLVER_HEADING_ABSORB_DEG;
        if (use_imu && !colado && v > 50.0f && v_med > SOLVER_HEADING_MIN_MMS
            && absf(h_err) < gate)
        {
            float rate = SOLVER_HEADING_WALL_RATE * 0.25f;
            if (par2)
            {
                rate = SOLVER_HEADING_WALL_RATE;
            }
            head_bias += h_err * rate;
            if (head_bias >  SOLVER_HEADING_BIAS_MAX) { head_bias =  SOLVER_HEADING_BIAS_MAX; }
            if (head_bias < -SOLVER_HEADING_BIAS_MAX) { head_bias = -SOLVER_HEADING_BIAS_MAX; }
        }
    }
    else
    {
        wf_prev  = 0.0f;
        wf_deriv = 0.0f;
        turn     = use_imu ? 0.0f : STRAIGHT_TRIM_MMS;
    }

    if (!colado)
    {
        /* com parede do lado quem manda e' o centralizador: o giroscopio
         * segurando o rumo com forca total brigava com ele e o robo ficava
         * 20-30 mm fora do centro (20o teste, bateu no pino em (4,2)) */
        turn += (wl || wr) ? (SOLVER_HEADING_WALL_SHARE * head_turn()) : head_turn();
    }

    if (turn >  WALL_FOLLOW_MAX_MMS) { turn =  WALL_FOLLOW_MAX_MMS; }
    if (turn < -WALL_FOLLOW_MAX_MMS) { turn = -WALL_FOLLOW_MAX_MMS; }

    speed_ctl_set_vw(v, turn);
}

/** Empacado andando (atrito, pe' arrastando): pede mais ate' desgrudar. */
static float move_boost(float v)
{
    v += (float)stall_ms * (SOLVER_MOVE_BOOST_MMS_PER_S / 1000.0f);
    return (v > SOLVER_MOVE_BOOST_MAX_MMS) ? SOLVER_MOVE_BOOST_MAX_MMS : v;
}

static void set_target_for_phase(void)
{
    if (phase == PH_RETURN)
    {
        maze_target_cell(sx, sy);
    }
    else
    {
        maze_target_center();
    }
}

static void do_abort(const char *why)
{
    speed_ctl_stop();
    abort_why = why;
    port_debug_printf("ABORTOU: %s\r\n", why);
    port_ihm_led_blink(100u);
    enter(ST_ABORT);
}

/**
 * Robo preso: imprime o diagnostico, recua (ou avanca) um pouco e depois
 * retoma o estado em que estava, do ponto em que parou.
 */
static int16_t front_min(void);

static void begin_unstick(const char *why)
{
    int32_t dl = ticks_l() - st_l0;
    int32_t dr = ticks_r() - st_r0;
    int16_t f  = port_sensors_front();

    /* uma linha so': estado, celula, motivo | ticks E/D | duty E/D |
     * frente (FE/FD) | lateral esq/dir */
    port_debug_printf("PRESO %s %u,%u %s | tk %ld/%ld dt %ld/%ld | F%d(%d/%d) E%d D%d r%d\r\n",
                      st_name[st], cx, cy, why, (long)dl, (long)dr,
                      (long)speed_ctl_get_duty(0), (long)speed_ctl_get_duty(1),
                      f, port_sensors_get_prox(SENSOR_FRONT_L),
                      port_sensors_get_prox(SENSOR_FRONT_R),
                      port_sensors_get_prox(SENSOR_LEFT),
                      port_sensors_get_prox(SENSOR_RIGHT),
                      (int)head_err());

    if (st == ST_UNSTICK || unstick_cell >= SOLVER_UNSTICK_MAX)
    {
        speed_ctl_clear_fault();
        do_abort("preso, desencalhe falhou");
        return;
    }

    unstick_cell++;
    unstick_total++;
    speed_ctl_clear_fault();
    speed_ctl_stop();

    resume_st     = st;
    resume_ms     = st_ms;
    /* Andando: bateu indo para frente, entao recua. Girando: recua se ha'
     * parede perto na frente, senao foi a traseira que encostou -- avanca. */
    /* Girando com a frente livre: alterna avanca / recua a cada tentativa
     * (8o teste: preso no giro com parede colada de lado, avancou 3x igual). */
    if (st != ST_TURN || f > SOLVER_CREEP_DETECT_PROX)
    {
        unstick_dir = -1.0f;
    }
    else
    {
        unstick_dir = ((unstick_cell & 1u) != 0u) ? 1.0f : -1.0f;
    }
    unstick_start = speed_ctl_get_fwd_ticks();

    us_side  = 0;
    us_phase = 0u;
    us_ms    = 0u;
#if (SOLVER_SIDESTEP != 0)
    /* andando, nada na frente: quina do chassi num pino -> desvia de lado */
    if (use_imu && st == ST_MOVE && front_min() < SOLVER_CREEP_DETECT_PROX
        && !(port_sensors_wall(SENSOR_LEFT) && port_sensors_wall(SENSOR_RIGHT)))
    {
        int16_t pl  = port_sensors_get_prox(SENSOR_LEFT);
        int16_t pr  = port_sensors_get_prox(SENSOR_RIGHT);
        int32_t rot = dr - dl;      /* + = girou p/ a esquerda = preso na esquerda */
        /* 24o teste: a quina DIREITA pegou no pino (robo girado 6 graus p/ a
         * direita, r6), mas a roda direita tinha andado mais (o rumo estava
         * sendo corrigido) -> desviou p/ o lado errado 3x. O giroscopio diz
         * para onde o pino girou o robo: girou p/ a direita = preso na direita. */
        float he  = head_err();     /* + = robo girado p/ a direita do rumo */
        /* parede CONHECIDA de um lado so': o pino que prendeu e' a ponta de
         * parede do lado aberto (24o (0,3) e 26o (0,0): robo afastado da
         * borda O, quina direita no pino) -> desvia p/ o lado da parede */
        bool kl = maze_wall(cx, cy, (maze_dir_t)((ch + 3) & 3));
        bool kr = maze_wall(cx, cy, (maze_dir_t)((ch + 1) & 3));
        if (pl > SOLVER_NEAR_WALL_PROX && pl > pr + 200)       { us_side = -1; }
        else if (pr > SOLVER_NEAR_WALL_PROX && pr > pl + 200)  { us_side =  1; }
        else if (he >=  SOLVER_SIDESTEP_ERR_DEG)               { us_side =  1; }
        else if (he <= -SOLVER_SIDESTEP_ERR_DEG)               { us_side = -1; }
        else if (kl && !kr)                                    { us_side =  1; }
        else if (kr && !kl)                                    { us_side = -1; }
        else if (rot >=  SOLVER_SIDESTEP_ROT_TICKS)            { us_side = -1; }
        else if (rot <= -SOLVER_SIDESTEP_ROT_TICKS)            { us_side =  1; }
        else { us_side = ((unstick_cell & 1u) != 0u) ? -1 : 1; }
        us_head0 = tgt_head + head_bias;
    }
#endif

    if (us_side != 0)
    {
        port_debug_printf("  desvia p/ %s (recua %d, %d graus, %d mm), tentativa %u/%d\r\n",
                          (us_side > 0) ? "esq" : "dir", (int)SOLVER_SIDESTEP_BACK_MM,
                          (int)SOLVER_SIDESTEP_DEG, (int)SOLVER_SIDESTEP_FWD_MM,
                          (unsigned)unstick_cell, (int)SOLVER_UNSTICK_MAX);
    }
    else
    {
        port_debug_printf("  %s %d mm, tentativa %u/%d\r\n",
                          (unstick_dir < 0.0f) ? "recua" : "avanca",
                          (int)SOLVER_UNSTICK_MM, (unsigned)unstick_cell,
                          (int)SOLVER_UNSTICK_MAX);
    }
    enter(ST_UNSTICK);
}

static bool side_state(sensor_id_t s, bool prev)
{
    int16_t p = port_sensors_get_prox(s);
    if (prev)  { return p > SOLVER_EDGE_OFF_PROX; }
    return p > SOLVER_EDGE_ON_PROX;
}

static void start_move(void)
{
    move_start  = speed_ctl_get_fwd_ticks();
    raw_start   = move_start;
    fix_valid   = false;
    mv_fix_n    = 0u;
    mv_fix_mm   = 0.0f;
    mv_raw_mm   = -1;
    move_target = (int32_t)((float)move_cells * MAZE_CELL_MM / mm_per_tick());
    creep_mm    = 0.0f;
    wf_prev = 0.0f; wf_deriv = 0.0f;
    edge_l = side_state(SENSOR_LEFT,  port_sensors_wall(SENSOR_LEFT));
    edge_r = side_state(SENSOR_RIGHT, port_sensors_wall(SENSOR_RIGHT));
    enter(ST_MOVE);
}

/**
 * Borda de parede lateral: o sensor lateral cruzou a divisa entre duas
 * celulas. As divisas ficam a 90 + 180k mm do ponto de partida (centro da
 * celula de saida); o sensor esta' SOLVER_SIDE_SENSOR_X_MM a' frente do eixo.
 * Corrige o ponto de partida para a odometria concordar com a divisa.
 */
static void edge_fix(bool rising)
{
#if (SOLVER_EDGE_CORRECTION == 0)
    (void)rising;
#else
    float a = (float)(speed_ctl_get_fwd_ticks() - move_start) * mm_per_tick();
    float s = a + SOLVER_SIDE_SENSOR_X_MM - SOLVER_EDGE_OFFSET_MM;
    int   k = (int)((s - 90.0f) / MAZE_CELL_MM + 0.5f);

    if (k < 0 || k >= (int)move_cells)
    {
        return;
    }

    float edge = 90.0f + MAZE_CELL_MM * (float)k;
    int   t    = rising ? 0 : 1;

    /* erro cru, como se o sensor visse a divisa exatamente */
    float raw_err = (edge - SOLVER_SIDE_SENSOR_X_MM + SOLVER_EDGE_OFFSET_MM) - a;
    if (raw_err <= -SOLVER_EDGE_MAX_FIX_MM || raw_err >= SOLVER_EDGE_MAX_FIX_MM)
    {
        return;
    }

    /* Largura do feixe: a parede "comeca" antes da divisa e "acaba" depois.
     * A metade da diferenca entre os erros medios dos dois tipos de borda e'
     * esse desvio -- ele nao depende da escala, que puxa os dois para o
     * mesmo lado. Medido no caminho, descontado de cada borda. */
    edge_err_sum[t] += raw_err;
    edge_err_n[t]++;
    edge_ema[t] += (raw_err - edge_ema[t]) * 0.15f;
    if (edge_err_n[0] >= 2u && edge_err_n[1] >= 2u)
    {
        edge_beam = (edge_ema[0] - edge_ema[1]) * 0.5f;
        if (edge_beam < 0.0f)                  { edge_beam = 0.0f; }
        if (edge_beam > SOLVER_EDGE_BEAM_MAX)  { edge_beam = SOLVER_EDGE_BEAM_MAX; }
    }

    /* So' bordas de SUBIDA (parede comecando). Nos testes 12-18 as de subida
     * erravam ~+5 mm e as de descida ~-20 mm (o sensor ainda ve' o pino depois
     * que a parede acaba): a descida em (5,1)->(5,2) fazia o robo passar
     * 20-29 mm do ponto e raspar no pino da entrada da armadilha. As medias
     * continuam no registro final ("ini"/"fim") para conferir. */
    if (!rising && SOLVER_EDGE_USE_FALLING == 0)
    {
        return;
    }
    float beam_use  = (SOLVER_EDGE_USE_FALLING != 0) ? edge_beam : 0.0f;
    float sensor_at = edge + (rising ? -beam_use : beam_use);
    float real_now  = sensor_at - SOLVER_SIDE_SENSOR_X_MM + SOLVER_EDGE_OFFSET_MM;
    float err       = real_now - a;

    /* soma das correcoes num movimento tambem limitada (16o teste: tres
     * bordas somaram -40 mm numa celula so') */
    if (absf(mv_fix_mm + err) > SOLVER_EDGE_MOVE_MAX_MM)
    {
        return;
    }
    move_start -= (int32_t)(err / mm_per_tick());
    edge_fixes++;
    mv_fix_n++;
    mv_fix_mm += err;

    /* aprende a escala entre duas posicoes absolutas */
    int32_t raw_now = speed_ctl_get_fwd_ticks();
    if (fix_valid)
    {
        float d_real = real_now - fix_real_mm;
        float d_raw  = (float)(raw_now - fix_raw) * MM_PER_TICK;
        if (d_real >= 100.0f && d_raw > 1.0f)
        {
            odo_learn(d_real / d_raw, 0.15f);
        }
    }
    fix_valid   = true;
    fix_real_mm = real_now;
    fix_raw     = raw_now;
#endif
}

/* ===========================================================================
 * Decisao (parado no centro de uma celula)
 * ======================================================================== */

/* Parede na frente so' se os DOIS frontais a virem (o menor dos dois acima
 * do limiar). Com a media, um frontal com leitura de fundo alta (10o teste:
 * FR-E 306, FR-D 217 com o corredor vazio) bastava para "fechar" a frente. */
static int16_t front_min(void)
{
    int16_t a = port_sensors_get_prox(SENSOR_FRONT_L);
    int16_t b = port_sensors_get_prox(SENSOR_FRONT_R);
    return (a < b) ? a : b;
}

static bool front_wall_seen(void)
{
    return front_min() > SOLVER_FRONT_WALL_PROX;
}

static void begin_turn(int quarters)        /* + = esquerda */
{
    turn_target  = speed_ctl_get_turn_ticks() + quarters * TURN_TICKS_PER_90;
    tgt_head    += 90.0f * (float)quarters;
    turn_tt0     = speed_ctl_get_turn_ticks();
    turn_gh0     = gyro_head();
    settle_ms    = 0u;
    enter(ST_TURN);
}

static void sense_and_decide(void)
{
    bool wl = port_sensors_wall(SENSOR_LEFT);
    bool wr = port_sensors_wall(SENSOR_RIGHT);
    bool wf = front_wall_seen();

    /* LARGADA: a celula de saida e' fechada em 3 lados e aberta em 1.
     *  - Imprime os 4 sensores (prox e cru) uma vez: diagnostico barato.
     *  - Laterais as duas fechadas: a saida so' pode estar na frente ou atras.
     *    So' conta parede na frente se a leitura for de parede COLADA
     *    (>= SOLVER_ALIGN_LO, ~ centro da celula): ai' o robo esta' de costas
     *    e gira 180. Leitura fraca (9o/10o testes: frente "250..850" com o
     *    corredor vazio) e' luz/reflexo/sensor -- considera ABERTA e avisa.
     *  - Uma lateral aberta e frente fechada: foi posto de lado; gira p/ ela.
     * O lado para onde ele ficar virado passa a ser o "N" -- o mapa nao muda. */
    if (!start_checked && phase == PH_EXPLORE && steps == 0 && cx == sx && cy == sy)
    {
        int16_t f = port_sensors_front();
        start_checked = true;
        port_debug_printf("LARGADA sens E%d FE%d FD%d D%d (cru %u %u %u %u)\r\n",
                          port_sensors_get_prox(SENSOR_LEFT),
                          port_sensors_get_prox(SENSOR_FRONT_L),
                          port_sensors_get_prox(SENSOR_FRONT_R),
                          port_sensors_get_prox(SENSOR_RIGHT),
                          port_sensors_get_raw_filtered(SENSOR_LEFT),
                          port_sensors_get_raw_filtered(SENSOR_FRONT_L),
                          port_sensors_get_raw_filtered(SENSOR_FRONT_R),
                          port_sensors_get_raw_filtered(SENSOR_RIGHT));
        int quarters = 0;
        if (wl && wr)
        {
            if (f >= SOLVER_ALIGN_LO)
            {
                quarters = 2;                   /* de costas para a saida */
            }
            else if (wf)
            {
                port_debug_printf("AVISO frente %d sem parede perto: considera ABERTA\r\n", f);
                wf = false;
            }
#if (SOLVER_START_AUTOCAL != 0)
            if (quarters == 0)
            {
                /* AUTOCALIBRACAO NA LARGADA: as duas laterais sao parede com o
                 * robo no centro (posto a' mao) e a frente e' aberta. Com a luz
                 * de AGORA: lateral no centro = 1000, frontal aberto = 0.
                 * 20o teste: no escuro as laterais liam 1284/1127 no centro
                 * (calibracao dizia 1000) e o centralizador perseguia o lado. */
                sensor_id_t sl[2] = { SENSOR_LEFT, SENSOR_RIGHT };
                sensor_id_t sf[2] = { SENSOR_FRONT_L, SENSOR_FRONT_R };
                for (int k = 0; k < 2; k++)
                {
                    int32_t r  = port_sensors_get_raw_filtered(sl[k]);
                    int32_t fa = port_sensors_get_cal_far(sl[k]);
                    if (r < fa - 300) { port_sensors_set_cal(sl[k], fa, r); }
                    int32_t rf = port_sensors_get_raw_filtered(sf[k]);
                    int32_t ne = port_sensors_get_cal_near(sf[k]);
                    if (rf > ne + 800) { port_sensors_set_cal(sf[k], rf, ne); }
                }
                port_debug_printf("CAL largada: lat perto %ld/%ld  frente longe %ld/%ld\r\n",
                                  (long)port_sensors_get_cal_near(SENSOR_LEFT),
                                  (long)port_sensors_get_cal_near(SENSOR_RIGHT),
                                  (long)port_sensors_get_cal_far(SENSOR_FRONT_L),
                                  (long)port_sensors_get_cal_far(SENSOR_FRONT_R));
                wl = port_sensors_wall(SENSOR_LEFT);
                wr = port_sensors_wall(SENSOR_RIGHT);
            }
#endif
        }
        else if (wf)
        {
            quarters = (!wl) ? 1 : -1;          /* posto de lado */
        }
        if (quarters != 0)
        {
            port_debug_printf("LARGADA de frente p/ parede (E%d F%d D%d): gira %s\r\n",
                              wl ? 1 : 0, (int)f, wr ? 1 : 0,
                              (quarters == 1) ? "p/ esq" : (quarters == -1) ? "p/ dir" : "180");
            want_dir   = ch;
            start_turn = true;
            begin_turn(quarters);
            return;
        }
    }

    /* Leu ABERTO onde a parede e' certa (borda externa / largada)? Entao o
     * robo nao esta' onde acha que esta' -- avisa no log (teste 7: em (4,0)
     * virado p/ S leu frente aberta, mas ali e' a borda). */
    {
        maze_dir_t dl = (maze_dir_t)((ch + 3) & 3);
        maze_dir_t dr = (maze_dir_t)((ch + 1) & 3);
        if ((!wl && maze_wall_locked(cx, cy, dl)) ||
            (!wf && maze_wall_locked(cx, cy, ch)) ||
            (!wr && maze_wall_locked(cx, cy, dr)))
        {
            port_debug_printf("AVISO %u,%u %c: aberto onde a parede e' certa (E%d F%d D%d)\r\n",
                              cx, cy, dir_ch[ch], port_sensors_get_prox(SENSOR_LEFT),
                              (int)port_sensors_front(), port_sensors_get_prox(SENSOR_RIGHT));
        }
    }

    maze_set_walls(cx, cy, ch, wl, wf, wr);
    unstick_cell = 0u;              /* chegou numa celula: zera tentativas */

    log_t *e = (lg_n < LOG_N) ? &lg[lg_n] : NULL;
    if (e != NULL)
    {
        e->x = cx; e->y = cy; e->h = (uint8_t)ch;
        e->walls = (uint8_t)((wl ? 1u : 0u) | (wf ? 2u : 0u) | (wr ? 4u : 0u));
        e->phase = (uint8_t)phase;
        e->dir = 0xFFu; e->n = 0u; e->note = 0u;
        lg_n++;
    }

    /* --- troca de fase --- */
    if (phase == PH_EXPLORE && maze_is_target(cx, cy))
    {
        port_debug_printf("CHEGOU %u,%u em %d passos\r\n", cx, cy, steps);
        phase = PH_RETURN;
        set_target_for_phase();
        port_ihm_led_blink(500u);
        enter(ST_PAUSE);
        return;
    }
    if (phase == PH_RETURN && cx == sx && cy == sy)
    {
        port_debug_printf("LARGADA de volta\r\n");
        phase = PH_RUN;
        set_target_for_phase();
        port_ihm_led_set(true);
        enter(ST_PAUSE);
        return;
    }
    if (phase == PH_RUN && maze_is_target(cx, cy))
    {
        speed_ctl_stop();
        port_debug_printf("FIM corrida ok, %d passos\r\n", steps);
        port_ihm_led_set(true);
        enter(ST_DONE);
        return;
    }

    if (++steps > SOLVER_MAX_STEPS)
    {
        do_abort("passos demais (trava de seguranca)");
        return;
    }

    /* --- flood fill e escolha --- */
    bool pess = (phase == PH_RUN);
    maze_flood(pess);
    int d = maze_next_dir(cx, cy, ch, pess);

    if (d < 0 && pess)
    {
        /* corrida sem caminho so' por celulas vistas: usa o otimista */
        pess = false;
        maze_flood(false);
        d = maze_next_dir(cx, cy, ch, false);
    }
    if (d < 0)
    {
        /* Alguma parede lida errado fechou o labirinto. Esquece as paredes
         * vistas uma vez so' e tenta de novo: o robo volta a explorar e
         * confere de perto o que tinha lido mal. */
        port_debug_printf("sem caminho: esquece leituras fracas\r\n");
        maze_forget_weak();
        maze_set_walls(cx, cy, ch, wl, wf, wr);
        pess = false;
        maze_flood(false);
        d = maze_next_dir(cx, cy, ch, false);
    }
    if (d < 0)
    {
        do_abort("sem caminho para o alvo (parede lida errado?)");
        return;
    }

    want_dir   = (maze_dir_t)d;
    /* corrida em retas longas so' com SOLVER_RUN_STRAIGHTS: no 21o teste a
     * exploracao e a volta foram perfeitas celula a celula, e a corrida em
     * retas de 5 celulas saiu torta e encalhou */
    move_cells = (phase == PH_RUN && SOLVER_RUN_STRAIGHTS != 0)
               ? maze_straight_run(cx, cy, want_dir, pess) : 1u;

    if (e != NULL)
    {
        e->dir = (uint8_t)want_dir;
        e->n   = move_cells;
    }

    /* Uma linha curta por celula (legenda no cabecalho):
     *   E 5,0 N 101 d4>N         fase, celula, rumo, paredes Esq/Frente/Dir,
     *                            distancia ao alvo > proxima direcao (xN = retas)
     *   180 b1+3                 ultimo movimento: encoder cru em mm, bordas
     *                            usadas e soma das correcoes delas (mm) */
    port_debug_printf("%c %u,%u %c %d%d%d d%u>%c",
                      ph_name[phase][0], cx, cy, dir_ch[ch],
                      wl ? 1 : 0, wf ? 1 : 0, wr ? 1 : 0,
                      (unsigned)maze_dist(cx, cy), dir_ch[want_dir]);
    if (move_cells > 1u)
    {
        port_debug_printf("x%u", (unsigned)move_cells);
    }
    if (mv_raw_mm >= 0)
    {
        port_debug_printf(" %d b%u%+d", (int)mv_raw_mm, (unsigned)mv_fix_n,
                          (int)mv_fix_mm);
        mv_raw_mm = -1;
    }
    if (wf)
    {
        /* parede na frente: leitura dos dois frontais parado aqui (calibrar
         * o "centro" -- 17o teste: parar em 1000 era ~30 mm antes) */
        port_debug_printf(" F%d/%d", port_sensors_get_prox(SENSOR_FRONT_L),
                          port_sensors_get_prox(SENSOR_FRONT_R));
    }
    {
        /* lateral em zona duvidosa: imprime os dois (diagnostico de luz) */
        int16_t sl = port_sensors_get_prox(SENSOR_LEFT);
        int16_t sr = port_sensors_get_prox(SENSOR_RIGHT);
        if ((sl >= SOLVER_DIAG_SIDE_LO && sl <= SOLVER_DIAG_SIDE_HI)
            || (sr >= SOLVER_DIAG_SIDE_LO && sr <= SOLVER_DIAG_SIDE_HI))
        {
            port_debug_printf(" s%d/%d", sl, sr);
        }
    }
    if (use_imu && absf(head_bias) >= 3.0f)
    {
        /* referencia de rumo corrigida pelas paredes (graus): cresce com o
         * erro de escala do giroscopio */
        port_debug_printf(" h%d", (int)head_bias);
    }
    if (falign_mm >= 0)
    {
        /* ajustou pela parede da frente: andou isto a mais (mm) */
        port_debug_printf(" a%d", (int)falign_mm);
        falign_mm = -1;
    }
    port_debug_printf("\r\n");

    int q = ((int)want_dir - (int)ch) & 3;     /* 0 reto, 1 dir, 2 re', 3 esq */
    if (q == 0)
    {
        start_move();
    }
    else
    {
        int quarters = (q == 1) ? -1 : (q == 3) ? 1 : 2;   /* + = esquerda */
#if (SOLVER_U_TURN_BALANCE != 0)
        /* 23o teste: toda meia-volta era p/ a esquerda -> o rumo total somou
         * +360 graus e o erro de escala do giroscopio (~3%) virou ~12 graus:
         * em (0,0) o robo saiu torto, foi parar ~37 mm de lado e prendeu no
         * giro em (3,1). Meia-volta agora gira para o lado que traz o rumo
         * total de volta a 0 -- o erro de escala nao acumula. */
        if (quarters == 2 && tgt_head > 1.0f)
        {
            quarters = -2;
        }
#endif
        begin_turn(quarters);
    }
}

/* ===========================================================================
 * API
 * ======================================================================== */

void solver_init(void)
{
    maze_init();

    sx = (MAZE_START_OUTER_RIGHT != 0) ? (uint8_t)(MAZE_W - 1) : 0u;
    sy = 0u;
    cx = sx; cy = sy; ch = DIR_N;
    maze_set_start(sx, sy);

    phase = PH_EXPLORE;
    set_target_for_phase();
    steps = 0;
    lg_n  = 0u;
    edge_fixes = 0u;
    odo_k = 1.0f;
    align_count = 0u;
    aligned = false;
    falign_mm = -1;
    us_side   = 0;
    unstick_cell  = 0u;
    unstick_total = 0u;
    use_imu      = false;
    imu_cal_done = false;
    imu_cal_tries = 0u;
    imu_checked  = false;
    start_checked = false;
    start_turn    = false;
    imu_sign     = 1.0f;
    tgt_head     = 0.0f;
    head_bias    = 0.0f;
    turn_err_max = 0.0f;
    turn_fixes   = 0u;
    edge_err_sum[0] = edge_err_sum[1] = 0.0f;
    edge_err_n[0]   = edge_err_n[1]   = 0u;
    edge_ema[0]     = edge_ema[1]     = 0.0f;
    edge_beam       = 0.0f;

    speed_ctl_enable(true);
    speed_ctl_stop();
    speed_ctl_reset_odometry();
    port_ihm_led_blink(200u);

    port_debug_printf("\r\nSOLVE %dx%d largada %u,%u N, externa a' %s, celula %ld tk, "
                      "90g %ld tk, sai em %lu s\r\n",
                      MAZE_W, MAZE_H, sx, sy,
                      (MAZE_START_OUTER_RIGHT != 0) ? "dir" : "esq",
                      (long)TICKS_PER_CELL, (long)TURN_TICKS_PER_90,
                      (unsigned long)(SOLVER_START_DELAY_MS / 1000u));
    port_debug_printf("linha: fase x,y rumo EFD dDist>prox [mm bBordas+corr]\r\n");

    enter(ST_WAIT);
}

void solver_update(void)
{
    st_ms += NAV_PERIOD_MS;

    /* Parado de verdade (lendo paredes ou na pausa entre fases, rodas sem
     * girar): o giroscopio re-mede o proprio offset (ver port_imu_set_still).
     * Assim um offset mal medido na largada se corrige na 1a parada. */
    port_imu_set_still(use_imu && (st == ST_SENSE || st == ST_PAUSE)
                       && st_ms >= 100u
                       && absf(speed_ctl_get_mms(0)) < 3.0f
                       && absf(speed_ctl_get_mms(1)) < 3.0f);

    if (speed_ctl_get_fault() != 0 && st != ST_ABORT && st != ST_DONE)
    {
        if (st == ST_TURN || st == ST_MOVE || st == ST_ALIGN)
        {
            begin_unstick("roda sem andar (falha do speed_ctl)");
        }
        else
        {
            do_abort("falha de roda/encoder (ver mensagem do speed_ctl)");
        }
        return;
    }

    switch (st)
    {
    case ST_WAIT:
        speed_ctl_stop();
        if (!imu_cal_done && st_ms + 1500u >= SOLVER_START_DELAY_MS)
        {
            /* robo parado ha' alguns segundos: mede o offset do giroscopio */
            imu_cal_done = true;
            if (SOLVER_USE_IMU && !port_imu_ok())
            {
                (void)port_imu_init();      /* mais uma chance (destrava o I2C) */
            }
            if (SOLVER_USE_IMU && port_imu_ok())
            {
                bool cal_ok = port_imu_calibrate(SOLVER_IMU_CAL_SAMPLES);
                port_imu_reset_heading();
                use_imu = true;
                port_debug_printf("gyro offset %d (variacao %d)%s\r\n",
                                  (int)port_imu_get_bias_raw(),
                                  (int)port_imu_get_cal_spread(),
                                  cal_ok ? "" : " MEXEU -- repete em 1 s");
                if (!cal_ok && imu_cal_tries < 5u)
                {
                    /* mao no robo: tenta de novo e atrasa a saida */
                    imu_cal_tries++;
                    imu_cal_done = false;
                    st_ms = SOLVER_START_DELAY_MS - 2500u;
                }
            }
            else
            {
                port_debug_printf("SEM gyro: giros so' por encoder\r\n");
            }
        }
        if (st_ms >= SOLVER_START_DELAY_MS)
        {
            port_ihm_led_blink(1000u);
            enter(ST_SENSE);
        }
        break;

    case ST_PAUSE:
        speed_ctl_stop();
        if (st_ms >= SOLVER_PHASE_PAUSE_MS)
        {
            enter(ST_SENSE);
        }
        break;

    case ST_SENSE:
        speed_ctl_stop();
        if (st_ms >= SOLVER_SETTLE_MS)
        {
            /* MENOR dos dois frontais (a luz infla um deles). Alinha a partir
             * de SOLVER_ALIGN_TRIGGER: 16o teste, parado ~20-40 mm antes do
             * centro a parede lia < 500, virou "frente aberta", e o robo
             * saiu deslocado e bateu na quina. Na largada nao alinha. */
            int16_t f = front_min();

            bool want_align = false;
            if (SOLVER_FRONT_STOP_BY_SENSOR != 0)
            {
                want_align = (f < SOLVER_ALIGN_LO || f > SOLVER_ALIGN_HI);
            }
            else if (SOLVER_FRONT_ALIGN != 0)
            {
                /* 22o teste: so' para frente, pelo FR-E (ver solver.h) */
                want_align = (port_sensors_get_prox(SENSOR_FRONT_L) < SOLVER_FALIGN_START)
                          && (port_sensors_get_prox(SENSOR_FRONT_R) < SOLVER_FALIGN_FD_SAFE);
            }
            if (want_align && !aligned && start_checked && f > SOLVER_ALIGN_TRIGGER)
            {
                aligned     = true;
                align_start = speed_ctl_get_fwd_ticks();
                align_count++;
                enter(ST_ALIGN);
                break;
            }
            aligned = false;
            sense_and_decide();
        }
        break;

    case ST_ALIGN:
    {
        int16_t f     = front_min();
        float   moved = (float)(speed_ctl_get_fwd_ticks() - align_start) * mm_per_tick();
        if (moved < 0.0f) { moved = -moved; }

        if (SOLVER_FRONT_STOP_BY_SENSOR == 0)
        {
            /* so' para frente, ate' o FR-E chegar no centro */
            if (port_sensors_get_prox(SENSOR_FRONT_L) >= SOLVER_FALIGN_STOP
                || port_sensors_get_prox(SENSOR_FRONT_R) >= SOLVER_FALIGN_FD_SAFE
                || moved > SOLVER_FALIGN_MAX_MM || st_ms > SOLVER_FALIGN_TIMEOUT_MS)
            {
                speed_ctl_stop();
                falign_mm = (int16_t)moved;
                enter(ST_SENSE);    /* aligned = true: nao alinha de novo */
                break;
            }
            speed_ctl_set_vw(SOLVER_FALIGN_MMS, head_turn());
            break;
        }
        if ((f >= SOLVER_ALIGN_LO && f <= SOLVER_ALIGN_HI)
            || moved > SOLVER_ALIGN_MAX_MM || st_ms > 2000u)
        {
            speed_ctl_stop();
            enter(ST_SENSE);        /* aligned = true: nao alinha de novo */
            break;
        }
        speed_ctl_set_vw((f < SOLVER_ALIGN_LO) ? SOLVER_ALIGN_MMS : -SOLVER_ALIGN_MMS, head_turn());
        break;
    }

    case ST_TURN:
    {
        if (st_ms < SOLVER_TURN_PAUSE_MS)
        {
            speed_ctl_stop();
            break;
        }

        /* Primeiro giro: o giroscopio concorda com os encoders? */
        if (use_imu && !imu_checked)
        {
            float enc_deg = (float)(speed_ctl_get_turn_ticks() - turn_tt0)
                          * 90.0f / (float)TURN_TICKS_PER_90;
            float gyr_deg = gyro_head() - turn_gh0;
            if (absf(enc_deg) > 30.0f)
            {
                imu_checked = true;
                if (absf(gyr_deg) < 8.0f)
                {
                    use_imu = false;
                    port_debug_printf("gyro NAO acompanhou (enc %d gyro %d): so' encoder\r\n",
                                      (int)enc_deg, (int)gyr_deg);
                }
                else if ((gyr_deg > 0.0f) != (enc_deg > 0.0f))
                {
                    imu_sign = -imu_sign;
                    port_debug_printf("gyro com sinal trocado (troque IMU_GZ_INVERT)\r\n");
                }
                else
                {
                    port_debug_printf("gyro ok (enc %d gyro %d)\r\n",
                                      (int)enc_deg, (int)gyr_deg);
                }
            }
        }

        /* quanto falta, em graus (+ = esquerda) */
        float rem = use_imu
                  ? head_err()
                  : (float)(turn_target - speed_ctl_get_turn_ticks())
                    * 90.0f / (float)TURN_TICKS_PER_90;
        float mag = absf(rem);

        /* parado conferindo: assentou dentro da tolerancia? */
        if (settle_ms > 0u)
        {
            speed_ctl_stop();
            settle_ms += NAV_PERIOD_MS;
            if (settle_ms < SOLVER_TURN_SETTLE_MS)
            {
                break;
            }
            if (mag <= SOLVER_TURN_DONE_DEG)
            {
                if (mag > turn_err_max) { turn_err_max = mag; }
                ch = want_dir;
                if (start_turn)
                {
                    /* giro de orientacao na largada: le' as paredes de novo */
                    start_turn = false;
                    aligned    = false;
                    enter(ST_SENSE);
                    break;
                }
                start_move();
                break;
            }
            /* passou (ou faltou): corrige devagar */
            settle_ms = 0u;
            turn_fixes++;
            stall_ref = speed_ctl_get_turn_ticks();
            stall_ms  = 0u;
        }

        if (mag <= SOLVER_TURN_TOL_DEG)
        {
            speed_ctl_stop();
            settle_ms = NAV_PERIOD_MS;
            break;
        }

        float speed = SPEED_TURN_MMS;
        if (mag < SOLVER_TURN_SLOW_DEG)
        {
            speed *= mag / SOLVER_TURN_SLOW_DEG;
            if (speed < 30.0f) { speed = 30.0f; }
        }

        /* nao saiu do lugar ainda: pede mais, ate' desgrudar */
        speed += (float)stall_ms * (SOLVER_TURN_BOOST_MMS_PER_S / 1000.0f);
        if (speed > SOLVER_TURN_BOOST_MAX_MMS) { speed = SOLVER_TURN_BOOST_MAX_MMS; }

        speed_ctl_set_vw(0.0f, (rem > 0.0f) ? speed : -speed);

        /* giro parado mesmo com forca maxima = preso de verdade */
        int32_t tt = speed_ctl_get_turn_ticks();
        int32_t dt = tt - stall_ref;
        if (dt < 0) { dt = -dt; }
        if (dt >= SOLVER_TURN_STALL_TICKS)
        {
            stall_ref = tt;
            stall_ms  = 0u;
        }
        else
        {
            stall_ms += NAV_PERIOD_MS;
            if (stall_ms >= SOLVER_TURN_STALL_MS)
            {
                begin_unstick("giro sem avancar");
                break;
            }
        }

        if (st_ms > 8000u)
        {
            do_abort("giro nao terminou em 8 s");
        }
        break;
    }

    case ST_MOVE:
    {
        /* Primeira passada: parede na frente que nao foi vista? Nao sai. */
        if (st_ms == NAV_PERIOD_MS && front_wall_seen())
        {
            speed_ctl_stop();
            maze_force_wall(cx, cy, ch);
            if (lg_n > 0u) { lg[lg_n - 1u].note = 1u; }
            port_debug_printf("parede na frente ao sair\r\n");
            enter(ST_SENSE);
            break;
        }

        /* bordas de parede lateral -> corrige a odometria */
        bool nl = side_state(SENSOR_LEFT,  edge_l);
        bool nr = side_state(SENSOR_RIGHT, edge_r);
        if (nl != edge_l) { edge_l = nl; edge_fix(nl); }
        if (nr != edge_r) { edge_r = nr; edge_fix(nr); }

        /* bateu em algo: a odometria parou de avancar */
        {
            int32_t ft = speed_ctl_get_fwd_ticks();
            int32_t d  = ft - stall_ref;
            if (d < 0) { d = -d; }
            if (d >= SOLVER_MOVE_STALL_TICKS)
            {
                stall_ref = ft;
                stall_ms  = 0u;
            }
            else
            {
                stall_ms += NAV_PERIOD_MS;
                if (stall_ms >= SOLVER_MOVE_STALL_MS)
                {
                    begin_unstick("andando, mas a odometria parou");
                    break;
                }
            }
        }

        int32_t traveled  = speed_ctl_get_fwd_ticks() - move_start;
        float   remaining = (float)(move_target - traveled) * mm_per_tick();
        /* parada no centro pelo MENOR frontal: com a media, a luz no FR-E
         * fazia chegar a 1000 uns 20 mm antes do centro (14o-16o testes) */
        int16_t front     = front_min();
        bool    stop      = false;

        /* 25o/26o testes: o FR-E chegava a 1400 ~25-30 mm ANTES do centro
         * (becos: 149-160 mm; "a25" em (3,1)). O robo ficava curto, voltava
         * com o erro e perdia parede/batia em pino. Agora a parada pelo FR-E
         * so' vale perto do fim da distancia do encoder; fora dessa janela,
         * so' com a parede praticamente colada. */
        int16_t fe = port_sensors_get_prox(SENSOR_FRONT_L);
        if (SOLVER_FRONT_STOP_BY_SENSOR == 0
            && front >= SOLVER_FRONT_WALL_PROX
            && ((fe >= SOLVER_FRONT_SAFE_FE && remaining <= SOLVER_FRONT_SAFE_WINDOW_MM)
                || fe >= SOLVER_FRONT_GLUED_FE))
        {
            stop = true;                /* seguranca: parede colada na frente */
        }
        else if (SOLVER_FRONT_STOP_BY_SENSOR != 0 && front >= SENSOR_FRONT_CENTER)
        {
            stop = true;                /* parede no fim: centro exato */

            /* parou no centro exato de uma celula: se houve uma borda antes
             * neste movimento, o trecho entre as duas mede a escala */
            if (fix_valid)
            {
                float trav_mm = (float)(speed_ctl_get_fwd_ticks() - move_start) * mm_per_tick();
                int   cells_i = (int)(trav_mm / MAZE_CELL_MM + 0.5f);
                float d_real  = (float)cells_i * MAZE_CELL_MM - fix_real_mm;
                float d_raw   = (float)(speed_ctl_get_fwd_ticks() - fix_raw) * MM_PER_TICK;
                if (cells_i >= 1 && d_real >= 100.0f && d_raw > 1.0f)
                {
                    odo_learn(d_real / d_raw, 0.15f);
                }
            }
        }
        else if (remaining <= 0.0f)
        {
            /* Acabou a distancia. Se a parede da frente esta' chegando mas
             * ainda nao no centro, completa devagar pelo sensor. */
            /* min dos dois frontais: 11o teste, o FR-E sozinho lia 547 de luz
             * (FR-D 241), a media passava de 400 e o robo rastejava 45 mm a
             * mais (andou 225 mm na 1a celula) -> passou do ponto da curva */
            if (SOLVER_FRONT_STOP_BY_SENSOR != 0
                && front_min() > SOLVER_CREEP_DETECT_PROX && creep_mm < SOLVER_CREEP_MAX_MM)
            {
                creep_mm = -remaining;
                drive_centered(move_boost(SPEED_APPROACH_MMS));
            }
            else
            {
                stop = true;
            }
        }
        else
        {
            float vmax = (phase == PH_RUN) ? SOLVER_RUN_MMS : SOLVER_EXPLORE_MMS;
            float v    = SOLVER_MIN_MMS + SOLVER_BRAKE_GAIN * remaining;
            if (v > vmax) { v = vmax; }
            if (front_min() > SOLVER_NEAR_WALL_PROX && v > SPEED_APPROACH_MMS)
            {
                v = SPEED_APPROACH_MMS;     /* parede chegando: devagar */
            }
            drive_centered(move_boost(v));
        }

        if (stop)
        {
            speed_ctl_stop();
            mv_raw_mm = (int16_t)((float)(speed_ctl_get_fwd_ticks() - raw_start) * MM_PER_TICK);

            int32_t tpc   = (int32_t)(MAZE_CELL_MM / mm_per_tick());
            int32_t cells = (traveled + tpc / 2) / tpc;
            if (cells > (int32_t)move_cells) { cells = move_cells; }
            if (cells < 0)                   { cells = 0; }

            if (cells < (int32_t)move_cells && lg_n > 0u)
            {
                lg[lg_n - 1u].note = 2u;
                port_debug_printf("andou %ld de %u\r\n",
                                  (long)cells, (unsigned)move_cells);
            }
            if (cells == 0)
            {
                /* nem saiu da celula: tinha parede ai' */
                maze_force_wall(cx, cy, ch);
            }

            for (int32_t i = 0; i < cells; i++)
            {
                (void)maze_step(&cx, &cy, ch);
            }
            enter(ST_SENSE);
            break;
        }

        uint32_t limit = 3000u + (uint32_t)move_cells * 4000u;
        if (st_ms > limit)
        {
            do_abort("andar demorou demais (preso?)");
        }
        break;
    }

    case ST_UNSTICK:
    {
        if (st_ms < 200u)
        {
            speed_ctl_stop();           /* deixa assentar antes de mexer */
            break;
        }

        float moved = (float)(speed_ctl_get_fwd_ticks() - unstick_start) * mm_per_tick();
        if (moved < 0.0f) { moved = -moved; }

        bool resume = false;
        if (us_side != 0)
        {
            /* desvio de pino: recua, vira p/ o lado livre, anda, desvira */
            us_ms += NAV_PERIOD_MS;
            float tgt = us_head0;
            if (us_phase == 1u || us_phase == 2u)
            {
                tgt += (float)us_side * SOLVER_SIDESTEP_DEG;
            }
            float err  = tgt - gyro_head();     /* + = falta girar p/ esquerda */
            bool  next = (us_ms > SOLVER_SIDESTEP_STEP_MS);

            if (us_phase == 0u || us_phase == 2u)
            {
                float dist = (us_phase == 0u) ? SOLVER_SIDESTEP_BACK_MM : SOLVER_SIDESTEP_FWD_MM;
                if (moved >= dist) { next = true; }
                else
                {
                    float w = err * SOLVER_HEADING_KP;
                    if (w >  SOLVER_HEADING_MAX_MMS) { w =  SOLVER_HEADING_MAX_MMS; }
                    if (w < -SOLVER_HEADING_MAX_MMS) { w = -SOLVER_HEADING_MAX_MMS; }
                    speed_ctl_set_vw((us_phase == 0u) ? -SOLVER_SIDESTEP_MMS : SOLVER_SIDESTEP_MMS, w);
                }
            }
            else
            {
                if (absf(err) < SOLVER_SIDESTEP_TOL_DEG) { next = true; }
                else
                {
                    float w = absf(err) * SOLVER_SIDESTEP_TURN_KP;
                    if (w < SOLVER_SIDESTEP_TURN_MIN) { w = SOLVER_SIDESTEP_TURN_MIN; }
                    if (w > SOLVER_SIDESTEP_TURN_MAX) { w = SOLVER_SIDESTEP_TURN_MAX; }
                    speed_ctl_set_vw(0.0f, (err > 0.0f) ? w : -w);
                }
            }

            if (next)
            {
                speed_ctl_stop();
                us_phase++;
                us_ms = 0u;
                unstick_start = speed_ctl_get_fwd_ticks();
                if (us_phase > 3u) { resume = true; }
            }
            if (!resume) { break; }
        }

        if (resume || moved >= SOLVER_UNSTICK_MM || st_ms > SOLVER_UNSTICK_TIMEOUT_MS)
        {
            speed_ctl_stop();
            /* retoma de onde parou, com o relogio do estado de antes */
            st_t back = resume_st;
            enter(back);
            /* relogio do estado: o giro pula a pausa inicial; o movimento
             * recomeca a contagem do tempo limite (o tempo preso nao conta) */
            st_ms = (back == ST_TURN) ? SOLVER_TURN_PAUSE_MS
                  : (back == ST_MOVE) ? (2u * NAV_PERIOD_MS)
                  : resume_ms;
            if (back == ST_MOVE)
            {
                stall_ref = speed_ctl_get_fwd_ticks();  /* detector do andar usa fwd */
            }
            us_side = 0;
            break;
        }
        /* recuando, puxa a frente para longe da parede lateral mais perto */
        float w = 0.0f;
        if (unstick_dir < 0.0f)
        {
            int16_t pl = port_sensors_get_prox(SENSOR_LEFT);
            int16_t pr = port_sensors_get_prox(SENSOR_RIGHT);
            if (pl > pr + 200)      { w = -SOLVER_UNSTICK_STEER_MMS; }
            else if (pr > pl + 200) { w =  SOLVER_UNSTICK_STEER_MMS; }
        }
        speed_ctl_set_vw(unstick_dir * SOLVER_UNSTICK_MMS, w);
        break;
    }

    case ST_DONE:
        speed_ctl_stop();
        break;

    case ST_ABORT:
    default:
        speed_ctl_stop();
        break;
    }
}

bool solver_finished(void)
{
    return (st == ST_DONE) || (st == ST_ABORT);
}

bool solver_aborted(void)
{
    return st == ST_ABORT;
}

void solver_dump(void)
{
    /* Resumo em 3 linhas + passos agrupados (8 por linha) + mapa. Sai UMA
     * vez no fim; de novo so' quando o terminal abre ou chega uma tecla. */
    port_debug_printf("\r\n== %s %s passos %d", st_name[st], ph_name[phase], steps);
    if (st == ST_ABORT)
    {
        port_debug_printf(" (%s)", abort_why);
    }
    port_debug_printf("\r\nbordas %u (ini %+d fim %+d mm) alinh %u desenc %u feixe %d escala %ld\r\n",
                      (unsigned)edge_fixes,
                      (int)((edge_err_n[0] > 0u) ? edge_err_sum[0] / (float)edge_err_n[0] : 0.0f),
                      (int)((edge_err_n[1] > 0u) ? edge_err_sum[1] / (float)edge_err_n[1] : 0.0f),
                      (unsigned)align_count, (unsigned)unstick_total, (int)edge_beam,
                      (long)(odo_k * 1000.0f));
    port_debug_printf("gyro %s off %d deriva %d giro errmax %d.%d fino %u\r\n",
                      use_imu ? "sim" : "NAO", (int)port_imu_get_bias_raw(), (int)head_bias,
                      (int)turn_err_max, (int)(turn_err_max * 10.0f) % 10,
                      (unsigned)turn_fixes);

    /* passos: fase x,y rumo EFD>dir  ('!' parede na saida, '-' andou menos) */
    for (uint16_t i = 0; i < lg_n; i++)
    {
        const log_t *e = &lg[i];
        char go = (e->dir < 4u) ? dir_ch[e->dir] : '-';
        port_debug_printf("%c%u%u%c%d%d%d>%c%c%s",
                          ph_name[e->phase][0], e->x, e->y, dir_ch[e->h & 3u],
                          (e->walls & 1u) ? 1 : 0, (e->walls & 2u) ? 1 : 0,
                          (e->walls & 4u) ? 1 : 0, go,
                          (e->note == 1u) ? '!' : (e->note == 2u) ? '-' : ' ',
                          ((i % 8u) == 7u || i + 1u == lg_n) ? "\r\n" : " ");
    }

    maze_flood(false);
    maze_print(out, cx, cy, ch);
}
