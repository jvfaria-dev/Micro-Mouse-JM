/**
 ******************************************************************************
 * @file    maze.h
 * @brief   Mapa do labirinto e flood fill -- logica pura, sem hardware.
 *
 * Nada aqui mexe em motor, sensor ou HAL: so' guarda paredes e calcula
 * distancias. Por isso o mesmo arquivo compila no PC e foi testado num
 * simulador antes de ir para o robo.
 *
 * ---------------------------------------------------------------------------
 * SISTEMA DE COORDENADAS (do ponto de vista do ROBO na largada)
 * ---------------------------------------------------------------------------
 *   - Norte (N) = para onde o robo esta' virado na largada.
 *   - x cresce para a DIREITA do robo (Leste), y cresce para FRENTE (Norte).
 *   - A largada fica na linha y = 0, encostada num canto.
 *
 * O labirinto real pode estar em qualquer orientacao: o mapa e' sempre
 * relativo a' largada. So' e' preciso dizer DE QUE LADO esta' a parede
 * externa na largada (MAZE_START_OUTER_RIGHT, em solver.h).
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef MAZE_W
#define MAZE_W      6           /* colunas (x) */
#endif
#ifndef MAZE_H
#define MAZE_H      6           /* linhas  (y) */
#endif

#define MAZE_DIST_INF   0xFFFFu

typedef enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 } maze_dir_t;

/* Bits de parede de cada celula */
#define WALL_N      (1u << 0)
#define WALL_E      (1u << 1)
#define WALL_S      (1u << 2)
#define WALL_W      (1u << 3)
#define CELL_SEEN   (1u << 4)   /* o robo ja' esteve aqui e leu as paredes */

/** @brief Zera o mapa e poe as paredes externas. */
void maze_init(void);

/** @brief Celula de largada e alvo atual. */
void maze_set_start(uint8_t x, uint8_t y);

/** @brief Alvo = as 4 celulas do centro (labirinto par) ou a do meio. */
void maze_target_center(void);

/** @brief Alvo = uma celula so'. */
void maze_target_cell(uint8_t x, uint8_t y);

/** @brief true se (x,y) faz parte do alvo atual. */
bool maze_is_target(uint8_t x, uint8_t y);

/**
 * @brief Registra o que o robo viu parado na celula (x,y) virado para h.
 * @param left/front/right  parede vista naquele lado do ROBO
 * Cada leitura e' um VOTO (parede/aberto) na divisa, valendo para as duas
 * celulas que ela separa. Uma leitura errada isolada e' corrigida na proxima
 * passagem pelo mesmo lugar.
 */
void maze_set_walls(uint8_t x, uint8_t y, maze_dir_t h,
                    bool left, bool front, bool right);

/**
 * @brief Igual a maze_set_walls, mas cada lado pode ser -1 = "nao sei"
 *        (leitura na faixa de duvida): esse lado nao recebe voto.
 *        1 = parede, 0 = aberto.
 */
void maze_set_walls3(uint8_t x, uint8_t y, maze_dir_t h,
                     int8_t left, int8_t front, int8_t right);

/** @brief Parede confirmada (bateu nela / nao conseguiu sair). */
void maze_force_wall(uint8_t x, uint8_t y, maze_dir_t d);

/** @brief Parede certa (borda externa ou largada) -- leitura nao muda. */
bool maze_wall_locked(uint8_t x, uint8_t y, maze_dir_t d);

/** @brief Esquece paredes vistas uma vez so' (recuperacao de leitura errada). */
void maze_forget_weak(void);

/**
 * @brief Tira UM voto de toda parede/abertura nao travada (+2 -> +1,
 *        +1 -> 0, -1 -> 0 ...). Recuperacao mais forte que maze_forget_weak:
 *        usada quando, mesmo esquecendo as leituras fracas, nao ha' caminho
 *        (parede fantasma lida duas vezes). Paredes reais voltam a ser lidas
 *        quando o robo passa por elas de novo.
 */
void maze_weaken(void);

/** @brief Ha' parede (conhecida) no lado d da celula? */
bool maze_wall(uint8_t x, uint8_t y, maze_dir_t d);

bool maze_seen(uint8_t x, uint8_t y);
uint8_t maze_raw(uint8_t x, uint8_t y);

/**
 * @brief Flood fill a partir do alvo.
 * @param unknown_is_wall  false = otimista (exploracao: o que nao foi visto
 *        conta como aberto); true = pessimista (corrida: so' passa por onde
 *        ja' andou, ou seja, por celulas vistas).
 */
void maze_flood(bool unknown_is_wall);

uint16_t maze_dist(uint8_t x, uint8_t y);

/**
 * @brief Melhor direcao a partir de (x,y) estando virado para h, segundo o
 *        ultimo flood. Empate: prefere seguir reto (menos giros).
 * @return direcao, ou -1 se nao ha' caminho.
 */
int maze_next_dir(uint8_t x, uint8_t y, maze_dir_t h, bool unknown_is_wall);

/**
 * @brief Quantas celulas seguidas da' para andar em linha reta na direcao d,
 *        partindo de (x,y), seguindo o flood e so' por celulas ja' vistas.
 *        Usado na corrida final para nao parar a cada celula. Minimo 1.
 */
uint8_t maze_straight_run(uint8_t x, uint8_t y, maze_dir_t d, bool unknown_is_wall);

/** @brief Celula vizinha na direcao d. false se sairia do labirinto. */
bool maze_step(uint8_t *x, uint8_t *y, maze_dir_t d);

/** @brief Imprime o mapa em ASCII por uma funcao de saida qualquer. */
typedef void (*maze_print_fn)(const char *s);
void maze_print(maze_print_fn out, uint8_t rx, uint8_t ry, maze_dir_t rh);
