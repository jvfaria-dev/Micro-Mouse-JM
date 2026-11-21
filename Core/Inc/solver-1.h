/**
 ******************************************************************************
 * @file    solver.h
 * @brief   APP_MODE_SOLVE -- resolve o labirinto com flood fill.
 *
 * O CICLO
 *   1. EXPLORA ate' o alvo (centro 2x2), mapeando as paredes.
 *   2. VOLTA para a largada, explorando o que faltou pelo caminho.
 *   3. CORRE da largada ao alvo pelo caminho mais curto CONHECIDO, sem
 *      parar a cada celula nas retas.
 *
 * COMO ELE ANDA (devagar e confiavel, de proposito)
 *   - sempre para no CENTRO de uma celula, le' as tres paredes parado,
 *     atualiza o mapa, roda o flood fill e decide;
 *   - gira no lugar (90, -90 ou 180) fechando por ticks, igual ao TURN_CAL;
 *   - anda exatamente uma celula (na corrida, varias) centralizando entre as
 *     paredes. Se houver parede no fim, quem manda parar e' o sensor frontal
 *     (prox = 1000 = centro da celula), o que corrige o erro de distancia
 *     acumulado toda vez que ele encontra uma parede de frente.
 *
 * COMO LARGAR
 *   Robo no CENTRO da celula de largada, virado para a unica abertura.
 *   Ele espera SOLVER_START_DELAY_MS e sai. Nao aperte reset no fim: o
 *   registro e o mapa vivem na RAM. Plugue a USB e abra o terminal -- ele
 *   imprime sozinho, ou na hora com qualquer tecla.
 ******************************************************************************
 */
#pragma once

#include "board.h"
#include "maze.h"

#include <stdint.h>
#include <stdbool.h>

/* --- labirinto ----------------------------------------------------------- */
/* Tamanho: MAZE_W / MAZE_H em maze.h (6x6). Alvo: centro.
 *
 * De que lado do ROBO fica a parede EXTERNA na largada.
 * 1 = parede externa a' DIREITA do robo (o labirinto se estende para a
 *     esquerda dele). E' o caso do labirinto da foto: largada no canto
 *     superior direito, robo virado para a esquerda da foto -- a borda de
 *     cima fica a' direita dele.
 * 0 = parede externa a' ESQUERDA. */
#define MAZE_START_OUTER_RIGHT      1

/* --- tempos -------------------------------------------------------------- */
#define SOLVER_START_DELAY_MS       5000u   /* tirar a mao do robo          */
#define SOLVER_SETTLE_MS            250u    /* parado antes de ler paredes  */
#define SOLVER_TURN_PAUSE_MS        150u    /* parado antes de girar        */
#define SOLVER_PHASE_PAUSE_MS       2000u   /* entre explorar/voltar/correr */
#define SOLVER_MAX_STEPS            400     /* trava de seguranca           */

/* --- velocidades --------------------------------------------------------- */
#define SOLVER_EXPLORE_MMS          SPEED_CRUISE_MMS
#define SOLVER_RUN_MMS              150.0f
#define SOLVER_MIN_MMS              30.0f
/* Freio: velocidade = MIN + ganho * distancia_restante (mm). 1,5/s da'
 * ~100 mm/s a 50 mm do fim. */
#define SOLVER_BRAKE_GAIN           1.5f

/* --- leitura de parede (proximidade calibrada, 1000 = centro) ------------ */
/* Lateral: SENSOR_WALL_THRESHOLD do board.h.
 * Frontal parado no centro: parede da propria celula da' ~1000, a da celula
 * seguinte (198 mm) da' perto de zero. 250 separa os dois com folga. */
#define SOLVER_FRONT_WALL_PROX      250

/* Ao fim da distancia, se o frontal ainda ve' a parede chegando mas nao
 * atingiu o centro, avanca devagar ate' atingir -- no maximo isto. */
#define SOLVER_CREEP_MAX_MM         45.0f
/* 6o teste: SEM nada na frente o frontal le' 70-95 (FE ~90, FD ~55: luz
 * ambiente/reflexo). Com 60 o robo achava que vinha parede, andava a 40 mm/s
 * a celula toda e, devagar assim, empacava no atrito. 150 = parede a ~4 cm. */
#define SOLVER_CREEP_DETECT_PROX    150

/* Perto da parede da frente (o frontal ja' a enxerga) a velocidade cai para
 * a de aproximacao, para o robo nao passar do centro por inercia. */
#define SOLVER_NEAR_WALL_PROX       150

/* --- correcao de distancia pela parede da frente ------------------------- */
/* Parado com parede na frente, se a proximidade frontal estiver fora desta
 * faixa o robo anda devagar para frente/tras ate' entrar nela. Corrige o erro
 * de distancia acumulado e a passada de ponto na frenagem. */
#define SOLVER_ALIGN_LO             850
#define SOLVER_ALIGN_HI             1250
#define SOLVER_ALIGN_MMS            20.0f
#define SOLVER_ALIGN_MAX_MM         40.0f

/* --- correcao de distancia pela BORDA da parede lateral ------------------ */
/* Quando uma parede lateral comeca ou termina, o sensor lateral esta'
 * exatamente na divisa entre duas celulas. Como o sensor fica
 * SOLVER_SIDE_SENSOR_X_MM a' frente do eixo, isso diz onde o robo esta' no
 * corredor -- e corrige a odometria mesmo sem parede na frente.
 * SOLVER_EDGE_OFFSET_MM compensa a largura do feixe do sensor (ajuste fino:
 * se ele para sempre um pouco ALEM do centro em corredor aberto, aumente). */
#define SOLVER_EDGE_CORRECTION      1
#define SOLVER_SIDE_SENSOR_X_MM     35.0f
#define SOLVER_EDGE_OFFSET_MM       0.0f
#define SOLVER_EDGE_ON_PROX         400
#define SOLVER_EDGE_OFF_PROX        250
#define SOLVER_EDGE_MAX_FIX_MM      45.0f   /* era 75: borda falsa empurrava meia celula */

/* Escala da odometria aprendida pelas bordas.
 * 5o teste: aprendeu 1194/1000 (19% !). O feixe do sensor lateral tem
 * largura: ele ve' a parede COMECAR um pouco antes da divisa e TERMINAR um
 * pouco depois, e isso virava "escala". Agora o robo estima essa largura no
 * caminho (metade da diferenca entre o erro medio das bordas de subida e o
 * das de descida) e desconta de cada borda. Os ticks por volta foram medidos
 * a' mao (573), entao a escala fica presa perto de 1. */
/* 7o e 8o testes: a escala aprendida bateu no teto (1100/1000) nas duas
 * corridas e, nas duas, o robo terminou ATRAS de onde achava que estava
 * (no 8o, uma celula inteira -> "CHEGOU" falso). O aprendizado esta'
 * enviesado; fica desligado (escala = MM_PER_TICK). Para conferir
 * MM_PER_TICK use o modo ODOM_CAL (500 mm medidos com trena). */
#define SOLVER_ODO_LEARN            0
#define SOLVER_ODO_K_MIN            0.90f
#define SOLVER_ODO_K_MAX            1.10f
#define SOLVER_EDGE_BEAM_MAX        30.0f   /* mm */

/* Andando sem sair do lugar: primeiro pede MAIS velocidade (o controle sobe
 * o esforco, como no giro); so' se continuar parado por SOLVER_MOVE_STALL_MS
 * e' batida de verdade -> desencalhe. */
#define SOLVER_MOVE_STALL_MS        1800u
#define SOLVER_MOVE_STALL_TICKS     10
#define SOLVER_MOVE_BOOST_MMS_PER_S 150.0f
#define SOLVER_MOVE_BOOST_MAX_MMS   200.0f

/* --- desencalhe ----------------------------------------------------------- */
/* Se o robo PRENDE (quina do chassi raspando na parede ao girar, roda
 * travada), em vez de abortar ele:
 *   1. para e imprime o diagnostico ([solve] PRESO ...: quanto cada roda
 *      andou, sensores, duty);
 *   2. recua SOLVER_UNSTICK_MM (girando sem nada perto na frente, avanca);
 *   3. continua o giro/movimento de onde parou.
 * Desiste depois de SOLVER_UNSTICK_MAX tentativas na MESMA celula.
 *
 * O giro tem detector proprio, mais rapido que o do speed_ctl (1800 ms):
 * se a diferenca de ticks nao avanca SOLVER_TURN_STALL_TICKS em
 * SOLVER_TURN_STALL_MS, e' porque esta' empurrando a parede.
 * SOLVER_UNSTICK_MMS fica abaixo de SPEED_FAULT_MIN_TARGET de proposito: o
 * recuo nao dispara a falha de roda de novo. */
#define SOLVER_UNSTICK_MAX          3
#define SOLVER_UNSTICK_MM           20.0f
#define SOLVER_UNSTICK_MMS          30.0f
#define SOLVER_UNSTICK_STEER_MMS    8.0f    /* ao recuar, afasta da parede */
#define SOLVER_UNSTICK_TIMEOUT_MS   1700u
#define SOLVER_TURN_STALL_MS        2500u

/* --- forca extra para DESGRUDAR no giro ---------------------------------- */
/* 3o teste: com duty ~320 (~2,7 V) as rodas ainda nao saiam do lugar no
 * giro -- girar no lugar arrasta os pes de lado e pede bem mais forca que
 * andar reto. Enquanto a diferenca de ticks nao avanca, a velocidade pedida
 * no giro SOBE (SOLVER_TURN_BOOST_MMS_PER_S) ate' o teto; o controle entao
 * chega rapido ao duty maximo (6 V). Quando desgruda, volta ao normal. */
#define SOLVER_TURN_BOOST_MMS_PER_S 200.0f
#define SOLVER_TURN_BOOST_MAX_MMS   250.0f
#define SOLVER_TURN_STALL_TICKS     10

/* --- giroscopio (MPU-6050) ---------------------------------------------- */
/* 4o teste: o encoder nao sabe se a roda escorregou, entao o giro saia
 * as vezes com mais/menos de 90 graus e o robo saia torto da celula, batendo
 * a quina. Com o giroscopio:
 *   - o GIRO fecha pelo angulo medido de verdade (com correcao fina se
 *     passou do ponto);
 *   - ANDANDO, o robo segura o rumo (SOLVER_HEADING_KP), alem de centralizar
 *     entre as paredes;
 *   - entre duas paredes, andando paralelo a elas, a referencia de rumo e'
 *     reacertada aos poucos -- isso cancela a deriva do giroscopio.
 * O offset e' medido nos 5 s de espera (robo parado!). No primeiro giro o
 * sinal do giroscopio e' conferido contra os encoders; se a IMU nao
 * responder, tudo volta a ser so' por encoder, como antes. */
#define SOLVER_USE_IMU              1
#define SOLVER_IMU_CAL_SAMPLES      800u    /* 0,8 s parado                 */
#define SOLVER_TURN_TOL_DEG         2.0f    /* para de girar a menos disto  */
#define SOLVER_TURN_DONE_DEG        3.5f    /* parado, aceita ate' isto     */
#define SOLVER_TURN_SETTLE_MS       150u
#define SOLVER_TURN_SLOW_DEG        25.0f   /* comeca a frear a 25 graus    */
#define SOLVER_HEADING_KP           3.0f    /* mm/s de giro por grau de erro */
#define SOLVER_HEADING_MAX_MMS      30.0f
#define SOLVER_HEADING_WALL_RATE    0.02f   /* reacerto pela parede, /10 ms */
#define SOLVER_HEADING_PARALLEL     150.0f  /* |d(erro)/dt| abaixo disto = paralelo */

void solver_init(void);

/** @brief Chamar a cada NAV_PERIOD_MS. */
void solver_update(void);

/** @brief true quando terminou (ou abortou). */
bool solver_finished(void);

/** @brief true se parou por erro (ver solver_dump). */
bool solver_aborted(void);

/** @brief Imprime o registro passo a passo e o mapa. */
void solver_dump(void);
