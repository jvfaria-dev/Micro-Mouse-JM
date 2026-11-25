/**
 ******************************************************************************
 * @file    decide_test.c
 * @brief   Teste da decisao de giro do flood fill com o robo parado.
 *          Ver decide_test.h. Usa o MESMO maze.c do solver e a MESMA regra
 *          de parede do solver.c (sense_and_decide), sem andar.
 ******************************************************************************
 */

#include "decide_test.h"
#include "maze.h"
#include "solver.h"     /* SOLVER_FRONT_WALL_PROX */
#include "nav.h"        /* NAV_PREFER_RIGHT (seguidor de paredes) */
#include "sensors.h"
#include "speed_ctl.h"
#include "motor.h"
#include "debug.h"
#include "ihm.h"
#include "board.h"

static uint32_t elapsed_ms = 0u;
static int      last_q     = -2;

static const char *const dir_name[4] = { "N", "L", "S", "O" };

/* mesma conta do solver.c: parede na frente so' se os DOIS diagonais virem */
static int16_t front_min_diag(void)
{
    int16_t a = port_sensors_get_prox(SENSOR_FRONT_L);
    int16_t b = port_sensors_get_prox(SENSOR_FRONT_R);
    return (a < b) ? a : b;
}

static const char *q_name(int q)
{
    switch (q)
    {
        case 0:  return "RETO";
        case 1:  return "DIREITA";
        case 2:  return "MEIA-VOLTA";
        case 3:  return "ESQUERDA";
        default: return "SEM CAMINHO";
    }
}

/* distancia da vizinha no lado d, ou '#' se nao da' para passar */
static void print_neighbor(const char *tag, uint8_t x, uint8_t y, maze_dir_t d)
{
    uint8_t nx = x, ny = y;
    if (maze_wall(x, y, d) || !maze_step(&nx, &ny, d))
    {
        port_debug_printf(" %s:#", tag);
    }
    else
    {
        port_debug_printf(" %s:%u", tag, (unsigned)maze_dist(nx, ny));
    }
}

/* monta um mapa novo com a leitura de agora e roda a decisao */
static int decide(int8_t tl, bool wf, int8_t tr)
{
    const uint8_t    x = DECIDE_TEST_X, y = DECIDE_TEST_Y;
    const maze_dir_t h = (maze_dir_t)(DECIDE_TEST_H & 3);

    maze_init();                            /* bordas + alvo no centro */
    maze_set_walls3(x, y, h, tl, wf ? 1 : 0, tr);   /* igual ao sense_and_decide */
    maze_flood(false);                      /* otimista, como na EXPLORA */

    int d = maze_next_dir(x, y, h, false);
    return (d < 0) ? -1 : (((int)d - (int)h) & 3);   /* 0 reto 1 dir 2 re' 3 esq */
}

void decide_test_init(void)
{
    speed_ctl_enable(false);
    port_motor_stop_all();
    elapsed_ms = 0u;
    last_q     = -2;

    port_debug_printf("\r\nDECIDE_TEST: motores DESLIGADOS. Robo virtual em %u,%u virado p/ %s, "
                      "%dx%d, alvo no centro.\r\n",
                      (unsigned)DECIDE_TEST_X, (unsigned)DECIDE_TEST_Y,
                      dir_name[DECIDE_TEST_H & 3], MAZE_W, MAZE_H);
    port_debug_printf("Frente: a mesma do solver (%s). Limiares: lateral %d, frente %d.\r\n",
                      SOLVER_FRONT_CENTRAL ? "Sharp CENTRAL" : "diagonais",
                      SENSOR_WALL_THRESHOLD,
                      SOLVER_FRONT_CENTRAL ? SOLVER_FRONT_WALL_C : SOLVER_FRONT_WALL_PROX);
    port_debug_printf("Esperado em 1,1 N:  F aberta -> RETO | F+D fechadas -> ESQUERDA |"
                      " F fechada, D aberta -> DIREITA | tudo fechado -> MEIA-VOLTA\r\n");
    port_debug_printf("Botao: imprime o mapa.\r\n\r\n");
}

void decide_test_update(void)
{
    port_motor_stop_all();                  /* garantia: nunca anda */

    int8_t tl    = solver_side_wall(SENSOR_LEFT);   /* 1/0/-1 (duvida) */
    int8_t tr    = solver_side_wall(SENSOR_RIGHT);
    bool wl      = (tl == 1);
    bool wr      = (tr == 1);
    bool wf_diag = front_min_diag() > SOLVER_FRONT_WALL_PROX;
    bool wf      = solver_front_wall_seen();    /* a mesma regra do solver */
    bool wf_cen  = wf;

    int q = decide(tl, wf, tr);

    /* zona morta do central (24/11): afaste a mao (leitura baixa) para zerar */
    if (port_sensors_get_prox(SENSOR_FRONT) < SENSOR_FRONT_STOP / 2)
    {
        sensors_dead_reset(SENSOR_FRONT);
    }
    sensor_zone_t zc = sensors_dead_update(SENSOR_FRONT, NAV_PERIOD_MS);
    static sensor_zone_t zc_last = SENSOR_ZONE_OK;
    bool zc_changed = (zc != zc_last);
    zc_last = zc;

    if (port_ihm_button_clicked())
    {
        port_debug_printf("\r\n");
        maze_print(port_debug_write, DECIDE_TEST_X, DECIDE_TEST_Y,
                   (maze_dir_t)(DECIDE_TEST_H & 3));
        port_debug_printf("\r\n");
    }

    elapsed_ms += NAV_PERIOD_MS;
    /* imprime no ritmo fixo, ou na hora se a decisao mudou */
    if (elapsed_ms < DECIDE_TEST_PRINT_MS && q == last_q && !zc_changed)
    {
        return;
    }
    elapsed_ms = 0u;
    bool changed = (q != last_q);
    last_q = q;

    const uint8_t    x = DECIDE_TEST_X, y = DECIDE_TEST_Y;
    const maze_dir_t h = (maze_dir_t)(DECIDE_TEST_H & 3);

    port_debug_printf("E%c F%d D%c | prox E %4d FE %4d C %4d FD %4d D %4d |",
                      (tl < 0) ? '?' : (wl ? '1' : '0'), wf ? 1 : 0,
                      (tr < 0) ? '?' : (wr ? '1' : '0'),
                      (int)port_sensors_get_prox(SENSOR_LEFT),
                      (int)port_sensors_get_prox(SENSOR_FRONT_L),
                      (int)port_sensors_get_prox(SENSOR_FRONT),
                      (int)port_sensors_get_prox(SENSOR_FRONT_R),
                      (int)port_sensors_get_prox(SENSOR_RIGHT));
    print_neighbor("F", x, y, h);
    print_neighbor("D", x, y, (maze_dir_t)((h + 1) & 3));
    print_neighbor("E", x, y, (maze_dir_t)((h + 3) & 3));
    print_neighbor("T", x, y, (maze_dir_t)((h + 2) & 3));
    port_debug_printf(" -> %s  [C:%s max %d]", q_name(q), sensors_zone_name(zc),
                      (int)sensors_dead_max(SENSOR_FRONT));

    /* para comparar: o que o seguidor de paredes (modo MAZE) faria */
    {
        const char *wf_txt;
        wl = port_sensors_wall(SENSOR_LEFT);    /* o modo MAZE usa o limiar unico */
        wr = port_sensors_wall(SENSOR_RIGHT);
        if (!wf_cen)            { wf_txt = "reto"; }
        else if (!wl && wr)     { wf_txt = "esq"; }
        else if (!wr && wl)     { wf_txt = "dir"; }
        else if (!wl && !wr)    { wf_txt = NAV_PREFER_RIGHT ? "dir" : "esq"; }
        else                    { wf_txt = "meia-volta"; }
        port_debug_printf("  (MAZE: %s)", wf_txt);
    }

    if (wf_diag != wf)
    {
        port_debug_printf("  !! frente: diag %d / usada %d", wf_diag ? 1 : 0, wf ? 1 : 0);
    }
    if (changed)
    {
        port_debug_printf("  *");
    }
    port_debug_printf("\r\n");
}

/* ===========================================================================
 * APP_MODE_DECIDE_TEST_MOTOR
 * ======================================================================== */

void decide_test_motor_init(void)
{
    solver_set_decision_log(true);
    solver_set_explore_only(DECIDE_TEST_MOTOR_EXPLORE_ONLY != 0);
    solver_init();
    port_debug_printf("DECIDE_TEST_MOTOR: uma linha [DEC] por celula. %s\r\n",
                      DECIDE_TEST_MOTOR_EXPLORE_ONLY ? "Para no centro."
                                                     : "Ciclo completo (explora, volta, corre).");
    port_debug_printf("[DEC] paredes lidas | Sharp E FE C FD D | dist F/D/E/T -> giro (rumo)\r\n");
}

void decide_test_motor_update(void)
{
    solver_update();
}

/* ===========================================================================
 * APP_MODE_LOCAL_TEST
 * ======================================================================== */

void decide_test_local_init(void)
{
    solver_set_local_mode(true);
    solver_set_decision_log(false);
    solver_set_explore_only(false);
    solver_init();
    port_debug_printf("LOCAL_TEST: decide SO' pelos Sharp (sem mapa/bordas/alvo). "
                      "Preferencia: %s\r\n",
                      (SOLVER_LOCAL_PREFER == 1) ? "direita, frente, esquerda"
                    : (SOLVER_LOCAL_PREFER == 2) ? "esquerda, frente, direita"
                                                 : "frente, direita, esquerda");
}

void decide_test_local_update(void)
{
    solver_update();
}
