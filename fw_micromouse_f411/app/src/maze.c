/**
 ******************************************************************************
 * @file    maze.c
 * @brief   Mapa e flood fill. Sem hardware -- ver maze.h.
 ******************************************************************************
 */

#include "maze.h"

#include <stdio.h>
#include <string.h>

static uint8_t  cell[MAZE_W][MAZE_H];

/* Votos por parede: +1 cada vez que foi vista, -1 cada vez que foi vista
 * ABERTA. A parede existe se o saldo for positivo. Uma leitura errada no
 * meio de varias certas nao muda nada, e uma parede fantasma vista uma vez
 * some na proxima passagem. Paredes externas ficam travadas em VOTE_FIXED. */
#define VOTE_MAX    3
#define VOTE_FIXED  100
static int8_t   vote[MAZE_W][MAZE_H][4];
static uint16_t dist[MAZE_W][MAZE_H];
static uint8_t  start_x = 0u, start_y = 0u;

/* alvo: ate' 4 celulas */
static uint8_t  tgt_x[4], tgt_y[4];
static uint8_t  tgt_n = 0u;

static const int8_t  DX[4]   = { 0, 1, 0, -1 };
static const int8_t  DY[4]   = { 1, 0, -1, 0 };
static const uint8_t WBIT[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

/* -------------------------------------------------------------------------- */

static bool inside(int x, int y)
{
    return (x >= 0) && (x < MAZE_W) && (y >= 0) && (y < MAZE_H);
}

static void apply_vote(uint8_t x, uint8_t y, maze_dir_t d, int8_t v)
{
    if (v > 0) { cell[x][y] |= WBIT[d]; }
    else       { cell[x][y] &= (uint8_t)~WBIT[d]; }
}

/* soma um voto (+1 parede, -1 aberto) nos DOIS lados da divisa */
static void vote_wall(uint8_t x, uint8_t y, maze_dir_t d, bool wall)
{
    int nx = (int)x + DX[d];
    int ny = (int)y + DY[d];

    if (!inside(nx, ny) || vote[x][y][d] >= VOTE_FIXED)
    {
        return;                         /* borda externa: nao mexe */
    }

    int8_t v = vote[x][y][d];
    if (wall  && v <  VOTE_MAX) { v++; }
    if (!wall && v > -VOTE_MAX) { v--; }

    maze_dir_t od = (maze_dir_t)((d + 2) & 3);
    vote[x][y][d]    = v;
    vote[nx][ny][od] = v;
    apply_vote(x, y, d, v);
    apply_vote((uint8_t)nx, (uint8_t)ny, od, v);
}

/**
 * A passagem entre (x,y) e o vizinho em d esta' livre?
 * Otimista: so' bloqueia parede CONHECIDA.
 * Pessimista: alem disso exige que a passagem tenha sido OBSERVADA, isto e',
 * que uma das duas celulas ja' tenha sido visitada.
 */
static bool passable(uint8_t x, uint8_t y, maze_dir_t d, bool unknown_is_wall)
{
    int nx = (int)x + DX[d];
    int ny = (int)y + DY[d];

    if (!inside(nx, ny) || (cell[x][y] & WBIT[d]) != 0u)
    {
        return false;
    }

    if (unknown_is_wall)
    {
        bool seen = ((cell[x][y] & CELL_SEEN) != 0u)
                 || ((cell[nx][ny] & CELL_SEEN) != 0u);
        if (!seen)
        {
            return false;
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */

void maze_init(void)
{
    memset(cell, 0, sizeof(cell));
    memset(vote, 0, sizeof(vote));

    for (uint8_t x = 0; x < MAZE_W; x++)
    {
        cell[x][0]          |= WALL_S;
        cell[x][MAZE_H - 1] |= WALL_N;
        vote[x][0][DIR_S]          = VOTE_FIXED;
        vote[x][MAZE_H - 1][DIR_N] = VOTE_FIXED;
    }
    for (uint8_t y = 0; y < MAZE_H; y++)
    {
        cell[0][y]          |= WALL_W;
        cell[MAZE_W - 1][y] |= WALL_E;
        vote[0][y][DIR_W]          = VOTE_FIXED;
        vote[MAZE_W - 1][y][DIR_E] = VOTE_FIXED;
    }

    maze_target_center();
}

/* trava uma parede (os dois lados da divisa) como certa: nenhuma leitura
 * nem maze_forget_weak() mexe mais nela */
static void lock_wall(uint8_t x, uint8_t y, maze_dir_t d)
{
    int nx = (int)x + DX[d];
    int ny = (int)y + DY[d];
    maze_dir_t od = (maze_dir_t)((d + 2) & 3);

    vote[x][y][d] = VOTE_FIXED;
    apply_vote(x, y, d, 1);
    if (inside(nx, ny))
    {
        vote[nx][ny][od] = VOTE_FIXED;
        apply_vote((uint8_t)nx, (uint8_t)ny, od, 1);
    }
}

/**
 * Largada: pelo regulamento a celula de saida e' fechada nos 3 lados e so'
 * abre para a frente (N). As duas laterais e o fundo ficam travados -- antes
 * a parede interna era so' um voto (+1), maze_forget_weak() a apagava e o
 * flood fill mandava o robo voltar "pelo lado" da largada, atravessando a
 * parede (teste 7: VOLTA (4,0) -> L).
 */
void maze_set_start(uint8_t x, uint8_t y)
{
    start_x = x;
    start_y = y;

    if (inside(x, y))
    {
        lock_wall(x, y, DIR_W);
        lock_wall(x, y, DIR_E);
        lock_wall(x, y, DIR_S);
        cell[x][y] |= CELL_SEEN;
    }
}

void maze_target_center(void)
{
    uint8_t x0 = (uint8_t)((MAZE_W - 1) / 2), x1 = (uint8_t)(MAZE_W / 2);
    uint8_t y0 = (uint8_t)((MAZE_H - 1) / 2), y1 = (uint8_t)(MAZE_H / 2);

    tgt_n = 0u;
    for (uint8_t x = x0; x <= x1; x++)
    {
        for (uint8_t y = y0; y <= y1; y++)
        {
            tgt_x[tgt_n] = x;
            tgt_y[tgt_n] = y;
            tgt_n++;
        }
    }
}

void maze_target_cell(uint8_t x, uint8_t y)
{
    tgt_x[0] = x;
    tgt_y[0] = y;
    tgt_n    = 1u;
}

bool maze_is_target(uint8_t x, uint8_t y)
{
    for (uint8_t i = 0; i < tgt_n; i++)
    {
        if (tgt_x[i] == x && tgt_y[i] == y)
        {
            return true;
        }
    }
    return false;
}

void maze_set_walls(uint8_t x, uint8_t y, maze_dir_t h,
                    bool left, bool front, bool right)
{
    if (!inside(x, y))
    {
        return;
    }

    maze_dir_t dl = (maze_dir_t)((h + 3) & 3);
    maze_dir_t dr = (maze_dir_t)((h + 1) & 3);

    vote_wall(x, y, dl, left);
    vote_wall(x, y, h,  front);
    vote_wall(x, y, dr, right);

    cell[x][y] |= CELL_SEEN;
}

void maze_force_wall(uint8_t x, uint8_t y, maze_dir_t d)
{
    if (!inside(x, y)) { return; }
    for (int i = 0; i < VOTE_MAX; i++)
    {
        vote_wall(x, y, d, true);
    }
}

void maze_forget_weak(void)
{
    for (uint8_t x = 0; x < MAZE_W; x++)
    {
        for (uint8_t y = 0; y < MAZE_H; y++)
        {
            for (int d = 0; d < 4; d++)
            {
                int8_t v = vote[x][y][d];
                if (v == 1 || v == -1)
                {
                    vote[x][y][d] = 0;
                    apply_vote(x, y, (maze_dir_t)d, 0);
                }
            }
        }
    }
}

bool maze_wall_locked(uint8_t x, uint8_t y, maze_dir_t d)
{
    int nx = (int)x + DX[d];
    int ny = (int)y + DY[d];
    if (!inside(x, y))       { return false; }
    if (!inside(nx, ny))     { return true; }   /* borda externa */
    return vote[x][y][d] >= VOTE_FIXED;
}

bool maze_wall(uint8_t x, uint8_t y, maze_dir_t d)
{
    return inside(x, y) && ((cell[x][y] & WBIT[d]) != 0u);
}

bool maze_seen(uint8_t x, uint8_t y)
{
    return inside(x, y) && ((cell[x][y] & CELL_SEEN) != 0u);
}

uint8_t maze_raw(uint8_t x, uint8_t y)
{
    return inside(x, y) ? cell[x][y] : 0u;
}

void maze_flood(bool unknown_is_wall)
{
    static uint8_t qx[MAZE_W * MAZE_H], qy[MAZE_W * MAZE_H];
    uint16_t head = 0u, tail = 0u;

    for (uint8_t x = 0; x < MAZE_W; x++)
    {
        for (uint8_t y = 0; y < MAZE_H; y++)
        {
            dist[x][y] = MAZE_DIST_INF;
        }
    }

    for (uint8_t i = 0; i < tgt_n; i++)
    {
        dist[tgt_x[i]][tgt_y[i]] = 0u;
        qx[tail] = tgt_x[i];
        qy[tail] = tgt_y[i];
        tail++;
    }

    while (head < tail)
    {
        uint8_t x = qx[head], y = qy[head];
        head++;

        for (int d = 0; d < 4; d++)
        {
            if (!passable(x, y, (maze_dir_t)d, unknown_is_wall))
            {
                continue;
            }
            uint8_t nx = (uint8_t)(x + DX[d]);
            uint8_t ny = (uint8_t)(y + DY[d]);

            if (dist[nx][ny] == MAZE_DIST_INF)
            {
                dist[nx][ny] = (uint16_t)(dist[x][y] + 1u);
                qx[tail] = nx;
                qy[tail] = ny;
                tail++;
            }
        }
    }
}

uint16_t maze_dist(uint8_t x, uint8_t y)
{
    return inside(x, y) ? dist[x][y] : MAZE_DIST_INF;
}

int maze_next_dir(uint8_t x, uint8_t y, maze_dir_t h, bool unknown_is_wall)
{
    if (!inside(x, y) || dist[x][y] == MAZE_DIST_INF)
    {
        return -1;
    }

    /* ordem de preferencia no empate: reto, direita, esquerda, re' */
    const maze_dir_t order[4] =
    {
        h,
        (maze_dir_t)((h + 1) & 3),
        (maze_dir_t)((h + 3) & 3),
        (maze_dir_t)((h + 2) & 3),
    };

    int      best   = -1;
    uint16_t best_d = MAZE_DIST_INF;

    for (int i = 0; i < 4; i++)
    {
        maze_dir_t d = order[i];

        if (!passable(x, y, d, unknown_is_wall))
        {
            continue;
        }

        uint16_t nd = dist[x + DX[d]][y + DY[d]];
        if (nd < best_d)
        {
            best_d = nd;
            best   = (int)d;
        }
    }

    return (best_d < dist[x][y]) ? best : -1;
}

uint8_t maze_straight_run(uint8_t x, uint8_t y, maze_dir_t d, bool unknown_is_wall)
{
    uint8_t n = 0u;

    while (n < (uint8_t)(MAZE_W + MAZE_H))
    {
        if (maze_next_dir(x, y, d, unknown_is_wall) != (int)d)
        {
            break;
        }
        if (!maze_step(&x, &y, d))
        {
            break;
        }
        n++;

        /* para no alvo e antes de entrar em celula nunca visitada */
        if (maze_is_target(x, y) || !maze_seen(x, y))
        {
            break;
        }
    }
    return (n == 0u) ? 1u : n;
}

bool maze_step(uint8_t *x, uint8_t *y, maze_dir_t d)
{
    int nx = (int)*x + DX[d];
    int ny = (int)*y + DY[d];

    if (!inside(nx, ny))
    {
        return false;
    }
    *x = (uint8_t)nx;
    *y = (uint8_t)ny;
    return true;
}

/* -------------------------------------------------------------------------- */

/*
 * Exemplo (6x6), linha de cima = y mais alto:
 *
 *   +---+---+---+
 *   | 5   4 : ? :      ---/|  parede vista
 *   +   +---+ . +       . :   ainda nao observada
 *   | 6 | ^   3 |       ^>v<  robo
 *
 * O numero e' a distancia ate' o alvo no ultimo flood.
 */
void maze_print(maze_print_fn out, uint8_t rx, uint8_t ry, maze_dir_t rh)
{
    static const char arrow[4] = { '^', '>', 'v', '<' };
    char line[MAZE_W * 4 + 4];

    for (int y = MAZE_H - 1; y >= 0; y--)
    {
        /* borda de cima da linha */
        int p = 0;
        for (int x = 0; x < MAZE_W; x++)
        {
            bool known = maze_seen((uint8_t)x, (uint8_t)y)
                      || (y == MAZE_H - 1)
                      || maze_seen((uint8_t)x, (uint8_t)(y + 1));
            const char *seg = (cell[x][y] & WALL_N) ? "---"
                            : (known ? "   " : " . ");
            line[p++] = '+';
            memcpy(&line[p], seg, 3);
            p += 3;
        }
        line[p++] = '+';
        line[p++] = '\r';
        line[p++] = '\n';
        line[p]   = '\0';
        out(line);

        /* celulas e paredes verticais */
        p = 0;
        for (int x = 0; x < MAZE_W; x++)
        {
            bool known = maze_seen((uint8_t)x, (uint8_t)y) || (x == 0)
                      || maze_seen((uint8_t)(x - 1), (uint8_t)y);
            line[p++] = (cell[x][y] & WALL_W) ? '|' : (known ? ' ' : ':');

            char c[4];
            if (x == rx && y == ry)
            {
                c[0] = ' '; c[1] = arrow[rh & 3]; c[2] = ' ';
            }
            else if (!maze_seen((uint8_t)x, (uint8_t)y))
            {
                c[0] = ' ';
                c[1] = maze_is_target((uint8_t)x, (uint8_t)y) ? 'G' : '?';
                c[2] = ' ';
            }
            else if (dist[x][y] == MAZE_DIST_INF)
            {
                c[0] = ' '; c[1] = '-'; c[2] = ' ';
            }
            else
            {
                snprintf(c, sizeof(c), "%3u", (unsigned)(dist[x][y] % 1000u));
            }
            memcpy(&line[p], c, 3);
            p += 3;
        }
        line[p++] = (cell[MAZE_W - 1][y] & WALL_E) ? '|' : ':';
        line[p++] = '\r';
        line[p++] = '\n';
        line[p]   = '\0';
        out(line);
    }

    /* borda de baixo */
    int p = 0;
    for (int x = 0; x < MAZE_W; x++)
    {
        line[p++] = '+';
        memcpy(&line[p], "---", 3);
        p += 3;
    }
    line[p++] = '+';
    line[p++] = '\r';
    line[p++] = '\n';
    line[p]   = '\0';
    out(line);
}
