/**
 ******************************************************************************
 * @file    decide_test.h
 * @brief   APP_MODE_DECIDE_TEST -- confere, com o robo PARADO, para que lado
 *          o flood fill decide virar a partir do que os Sharp leem.
 *
 * MOTORES DESLIGADOS. Nada aqui anda nem gira.
 *
 * COMO USAR
 *   1. Em app_main.h: #define APP_MODE APP_MODE_DECIDE_TEST
 *   2. Grave, abra o terminal serial.
 *   3. Ponha o robo numa celula (ou use a mao/um pedaco de parede na frente
 *      de cada sensor) e veja a linha que sai a cada DECIDE_TEST_PRINT_MS:
 *
 *      E1 F0 D1 | prox E 812 FE 140 C 95 FD 133 D 790 | F:1 D:# E:# T:3 -> RETO
 *
 *        E/F/D       parede lida a Esquerda / Frente / Direita (1 = parede,
 *                    ? = lateral na faixa de duvida, ver solver.h)
 *        prox ...    proximidade de cada Sharp (mesma escala do solver)
 *        F: D: E: T: distancia ate' o alvo da vizinha da Frente, Direita,
 *                    Esquerda e de Tras ('#' = parede, nao pode ir)
 *        -> ...      decisao: RETO, DIREITA, ESQUERDA, MEIA-VOLTA
 *      A frente (F) e' lida pela MESMA funcao do solver
 *      (solver_front_wall_seen: Sharp central desde 23/11). Se os diagonais
 *      discordarem dela, a linha termina com  !! frente: diag X / usada Y
 *      -- so' informativo.
 *
 *   4. Clique no botao: imprime o mapa ASCII com as distancias.
 *
 * O QUE E' "CERTO"
 *   O robo virtual fica na celula (DECIDE_TEST_X, DECIDE_TEST_Y) virado para
 *   DECIDE_TEST_H, num mapa novo (so' as paredes externas + o que os Sharp
 *   veem agora). No padrao -- celula (1,1) virado para N, labirinto 6x6 com
 *   alvo no centro -- a frente e a direita estao a 1 passo do alvo, a
 *   esquerda e tras a 3. Entao a resposta certa e':
 *
 *      E  F  D   ->  decisao
 *      -  0  -   ->  RETO        (frente aberta sempre ganha: empate com a
 *                                 direita, e o desempate prefere reto)
 *      -  1  0   ->  DIREITA
 *      0  1  1   ->  ESQUERDA    (empate com tras: prefere esquerda)
 *      1  1  1   ->  MEIA-VOLTA
 *
 *   Se a leitura de E/F/D estiver certa e a decisao nao bater com a tabela,
 *   o problema e' na logica; se E/F/D ja' estiverem errados, e' sensor/limiar.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Celula e rumo do robo virtual (DIR_N=0, DIR_E=1, DIR_S=2, DIR_W=3). */
#ifndef DECIDE_TEST_X
#define DECIDE_TEST_X           1
#endif
#ifndef DECIDE_TEST_Y
#define DECIDE_TEST_Y           1
#endif
#ifndef DECIDE_TEST_H
#define DECIDE_TEST_H           0       /* N */
#endif

/* APP_MODE_DECIDE_TEST_MOTOR: 1 = para quando chega ao centro (so' a
 * exploracao); 0 = ciclo completo do SOLVE (explora, volta e corre). */
#ifndef DECIDE_TEST_MOTOR_EXPLORE_ONLY
#define DECIDE_TEST_MOTOR_EXPLORE_ONLY  1
#endif

#define DECIDE_TEST_PRINT_MS    500u

void decide_test_init(void);

/** @brief Chamar a cada NAV_PERIOD_MS. */
void decide_test_update(void);

/* ---------------------------------------------------------------------------
 * APP_MODE_DECIDE_TEST_MOTOR -- o mesmo teste, mas ANDANDO no labirinto.
 *
 * E' o solver de verdade (mesmo movimento, mesma decisao do APP_MODE_SOLVE),
 * com uma linha [DEC] a mais em cada celula, impressa na hora em que decide:
 *
 *   E 5,0 N 101 d4>N ...                     (linha normal do solver)
 *     [DEC] E1 F0 D1 | E 812 FE 140 C 95 FD 133 D 790 | F:3 D:# E:# T:5 -> RETO (N)
 *
 *   E/F/D       parede que ele LEU (1 = parede) -- e' o que vai para o mapa
 *   E FE C FD D proximidade dos 5 Sharp naquele instante (parado no centro)
 *   F/D/E/T     distancia ao alvo da vizinha da Frente/Direita/Esquerda/Tras
 *               ('#' parede, '-' sem caminho por ali)
 *   -> giro     RETO / DIREITA / ESQUERDA / MEIA-VOLTA e a direcao no mapa
 *
 * A decisao certa e' sempre: a vizinha com o MENOR numero entre as que nao
 * sao '#'; empate -> reto, depois direita, esquerda, tras.
 *
 * Largada igual ao SOLVE: robo no centro da celula de saida, 5 s de espera.
 * Com DECIDE_TEST_MOTOR_EXPLORE_ONLY 1 ele para no centro do labirinto.
 * ------------------------------------------------------------------------- */
void decide_test_motor_init(void);
void decide_test_motor_update(void);

/* ---------------------------------------------------------------------------
 * APP_MODE_LOCAL_TEST -- anda celula a celula decidindo SO' pelos Sharp.
 * Sem mapa, sem bordas, sem alvo: MAZE_START_OUTER_RIGHT nao influencia.
 * Uma linha por celula:
 *   LOCAL #3  E1 F1 D0 | E 1032 FE 583 C 1076 FD 868 D 0 -> DIREITA  (andou 178 mm)
 * Preferencia entre lados abertos: SOLVER_LOCAL_PREFER (solver.h).
 * Largada igual ao SOLVE (5 s). Para sozinho em SOLVER_MAX_STEPS celulas,
 * ou desligue o robo. Mapa/registro saem no fim, como no SOLVE.
 * ------------------------------------------------------------------------- */
void decide_test_local_init(void);
void decide_test_local_update(void);
