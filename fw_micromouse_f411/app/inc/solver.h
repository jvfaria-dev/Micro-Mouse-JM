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
/* 21o teste: exploracao e volta perfeitas (41 + 15 passos, todas as celulas
 * 179-180 mm), corrida em retas de 5 celulas a 150 mm/s saiu torta (parou
 * 37 mm antes, leu paredes erradas) e encalhou. Por enquanto a corrida anda
 * como a exploracao: celula a celula, mesma velocidade. Acelerar depois que
 * estiver confiavel: SOLVER_RUN_STRAIGHTS 1 e/ou SOLVER_RUN_MMS maior. */
#define SOLVER_RUN_STRAIGHTS        0
#define SOLVER_RUN_MMS              150.0f  /* 31o: 100 -> 120 ok; 32o: 120 -> 150 */
#define SOLVER_MIN_MMS              30.0f
/* Freio: velocidade = MIN + ganho * distancia_restante (mm). 1,5/s da'
 * ~100 mm/s a 50 mm do fim. */
#define SOLVER_BRAKE_GAIN           1.5f

/* --- leitura de parede (proximidade calibrada, 1000 = centro) ------------ */
/* Lateral: SENSOR_WALL_THRESHOLD do board.h.
 *
 * FRONTAL -- o espaco entre "tem parede" e "nao tem" e' enorme, entao os
 * limiares ficam no MEIO dele, longe do fundo:
 *   - com parede na frente, toda decisao acontece com o robo parado no
 *     centro, e o movimento so' termina quando a frente chega a 1000
 *     (SENSOR_FRONT_CENTER): a leitura de parede e' ~1000;
 *   - sem parede, o fundo depende da LUZ: 70-95 no 6o teste, 217-306 no 10o
 *     (labirinto novo, mais claro). Com 250/150 o robo via parede onde nao
 *     tinha, freava e "rastejava" 45 mm a mais em toda celula.
 * Se ele parar curto com parede na frente e ler menos que isto, nao ha'
 * problema: ao sair ele ve' a frente chegar a 1000 logo, para sem ter saido
 * da celula e marca a parede (cells == 0 -> maze_force_wall). */
#define SOLVER_FRONT_WALL_PROX      500     /* era 250 */

/* Ao fim da distancia, se o frontal ainda ve' a parede chegando mas nao
 * atingiu o centro, avanca devagar ate' atingir -- no maximo isto. */
#define SOLVER_CREEP_MAX_MM         30.0f   /* era 45 */
/* 6o teste: 60 era baixo demais (fundo 70-95). 10o teste: 150 tambem (fundo
 * 217-306 com mais luz). 400 = parede ja' bem perto, acima de qualquer fundo
 * medido ate' agora. */
#define SOLVER_CREEP_DETECT_PROX    300     /* 150 -> 400 -> 300 (agora pelo MENOR frontal, que a luz quase nao infla) */

/* Perto da parede da frente a velocidade cai para a de aproximacao, para o
 * robo nao passar do centro por inercia. Mesmo criterio do de cima. */
#define SOLVER_NEAR_WALL_PROX       400     /* era 150 */

/* --- correcao de distancia pela parede da frente ------------------------- */
/* Parado com parede na frente, se a proximidade frontal estiver fora desta
 * faixa o robo anda devagar para frente/tras ate' entrar nela. Corrige o erro
 * de distancia acumulado e a passada de ponto na frenagem. */
/* PARADA COM PAREDE NA FRENTE. 14o-17o testes: toda reta que terminava em
 * parede parava em ~150 mm em vez de 180 (E 0,0 O ... 151, E 1,1 ... 150,
 * mesmo no escuro), ou seja ~30 mm ANTES do centro: o FR-D satura cedo
 * (prox maxima ~1150, NEAR cru 518) e "1000" acontece longe da parede. Parado
 * fora do centro, ao girar o robo bate nos pinos (17o: (4,1) e (0,0)).
 * 0 = para pelo ENCODER (180 mm/celula, medido certo) e o frontal so' para por
 *     seguranca, com o FR-E quase saturado (parede colada). Sem rastejo e sem
 *     alinhamento frontal ate' recalibrar o "centro".
 * 1 = comportamento antigo (para quando o menor frontal chega a 1000). */
#define SOLVER_FRONT_STOP_BY_SENSOR 0
#define SOLVER_FRONT_SAFE_FE        1400    /* 19o: no centro FR-E 1435 / FR-D 1107 e
                                             * colado 1445 -- 1450 nunca chegava e o
                                             * robo empurrou a parede */

/* Parado, se o MENOR frontal passar disto (mas fora de LO..HI), anda devagar
 * ate' a parede ficar no centro. Abaixo de ~250 e' corredor aberto mesmo com
 * a luz (FD sem parede: 2-241); com parede 20-40 mm adiante le' mais. */
/* 26o: parada de seguranca pelo FR-E so' nos ultimos WINDOW mm do encoder
 * (antes parava 20-30 mm curto); fora disso so' com FR-E >= GLUED (colada). */
/* 27o-30o: com janela 25 os becos paravam exatamente em ~155 (180-25): o FR-E
 * ja' passa de 1400 (e le' 1420-1433) 25 mm antes do centro -- esta' saturado
 * nesse trecho todo. Em (3,1) o robo voltava curto de novo ("a24", s23/249).
 * Janela 10 e colada 1445 (19o: colado ~1445): quem para e' o encoder. */
#define SOLVER_FRONT_SAFE_WINDOW_MM 10.0f
#define SOLVER_FRONT_GLUED_FE       1445
/* lateral nesta faixa na leitura da celula -> imprime " sE/D" no log */
#define SOLVER_DIAG_SIDE_LO         120
#define SOLVER_DIAG_SIDE_HI         600
#define SOLVER_ALIGN_TRIGGER        300
#define SOLVER_ALIGN_LO             850
#define SOLVER_ALIGN_HI             1250
#define SOLVER_ALIGN_MMS            20.0f
#define SOLVER_ALIGN_MAX_MM         40.0f

/* AJUSTE PELA PAREDE DA FRENTE, SO' PARA FRENTE (22o teste). Parado pelo
 * encoder, com parede na frente mas lendo longe (beco (4,4): F799/795, no
 * centro e' ~1408/1094), o robo ficou ~30-50 mm antes do centro; voltou 3
 * celulas com esse erro, virou para O em (4,1) e a quina pegou no pino.
 * Agora: se o MENOR frontal > SOLVER_ALIGN_TRIGGER (tem parede) e o FR-E
 * < FALIGN_START, anda devagar para frente ate' FR-E >= FALIGN_STOP (o FR-D
 * satura cedo, so' serve de seguranca: >= FALIGN_FD_SAFE para). Nunca recua
 * (a luz infla o FR-E -> no maximo para um pouco antes, como hoje). */
#define SOLVER_FRONT_ALIGN          1
#define SOLVER_FALIGN_START         1340    /* FR-E abaixo disto: esta' curto */
#define SOLVER_FALIGN_STOP          1395    /* FR-E no centro: 1404-1435      */
#define SOLVER_FALIGN_FD_SAFE       1140    /* FR-D no centro 1088-1107       */
#define SOLVER_FALIGN_MAX_MM        50.0f
#define SOLVER_FALIGN_MMS           25.0f
#define SOLVER_FALIGN_TIMEOUT_MS    3000u

/* --- correcao de distancia pela BORDA da parede lateral ------------------ */
/* Quando uma parede lateral comeca ou termina, o sensor lateral esta'
 * exatamente na divisa entre duas celulas. Como o sensor fica
 * SOLVER_SIDE_SENSOR_X_MM a' frente do eixo, isso diz onde o robo esta' no
 * corredor -- e corrige a odometria mesmo sem parede na frente.
 * SOLVER_EDGE_OFFSET_MM compensa a largura do feixe do sensor (ajuste fino:
 * se ele para sempre um pouco ALEM do centro em corredor aberto, aumente). */
/* 19o teste: ate' a borda de SUBIDA errou -32 mm (a parede da direita estava
 * mais longe e a leitura cruzou o limiar tarde) e o robo passou do ponto e
 * bateu na parede do fundo de (4,4). O cruzamento do limiar depende da
 * distancia lateral -- com estes modulos nao da' posicao confiavel. O encoder
 * sozinho fecha 179-180 mm por celula em todos os testes: fica so' ele. */
#define SOLVER_EDGE_CORRECTION      0
#define SOLVER_SIDE_SENSOR_X_MM     35.0f
#define SOLVER_EDGE_OFFSET_MM       0.0f
#define SOLVER_EDGE_ON_PROX         500     /* era 400: luz deixa o lado ABERTO em 250-360 */
#define SOLVER_EDGE_OFF_PROX        380     /* era 250 */
#define SOLVER_EDGE_MOVE_MAX_MM     35.0f   /* soma das bordas num movimento */
#define SOLVER_EDGE_USE_FALLING     0       /* 0 = so' parede comecando (ver edge_fix) */
#define SOLVER_EDGE_MAX_FIX_MM      25.0f   /* 13o teste: com luz a borda falsa dava -29 mm (2 vezes) e jogava o robo na quina. O encoder sozinho fecha 179-180 mm/celula: a borda so' faz ajuste fino */

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

/* DESVIO DE PINO (22o teste). Andando sem parede na frente, preso: e' a
 * quina do chassi num pino/ponta de parede (robo fora do centro de lado).
 * Recuar e tentar na mesma linha bate no mesmo lugar (3x em (4,1)). Agora:
 * recua BACK_MM, vira DEG para o lado livre, anda FWD_MM (desloca ~17 mm de
 * lado), volta ao rumo e continua. Lado livre: o robo gira para o lado que
 * prendeu (roda do outro lado anda mais: tk 156/166 -> preso na esquerda ->
 * desvia para a direita); se nao der para saber, parede lateral perto ou
 * alterna. So' com giroscopio e sem parede dos dois lados. */
#define SOLVER_SIDESTEP             1
#define SOLVER_SIDESTEP_BACK_MM     30.0f
#define SOLVER_SIDESTEP_DEG         25.0f
#define SOLVER_SIDESTEP_FWD_MM      40.0f
#define SOLVER_SIDESTEP_MMS         30.0f   /* < SPEED_FAULT_MIN_TARGET */
#define SOLVER_SIDESTEP_TURN_KP     2.0f    /* mm/s por grau          */
#define SOLVER_SIDESTEP_TURN_MIN    30.0f   /* mm/s (desgrudar)       */
#define SOLVER_SIDESTEP_TURN_MAX    38.0f   /* < SPEED_FAULT_MIN_TARGET */
#define SOLVER_SIDESTEP_TOL_DEG     3.0f
#define SOLVER_SIDESTEP_STEP_MS     2500u   /* limite de cada etapa   */
#define SOLVER_SIDESTEP_ERR_DEG     3.0f    /* erro de rumo p/ decidir o lado (1o criterio) */
#define SOLVER_SIDESTEP_ROT_TICKS   4       /* |dir-esq| p/ decidir o lado */

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
/* 12o teste: a referencia de rumo foi puxada -95 graus. Preso torto contra
 * uma parede, com o "boost" pedindo velocidade e a luz fazendo os laterais
 * verem parede, ela ia absorvendo o angulo do robo encalhado. Agora so'
 * absorve diferenca PEQUENA (deriva de verdade), com o robo ANDANDO de fato,
 * e o total fica limitado. */
#define SOLVER_HEADING_ABSORB_DEG   6.0f    /* so' corrige se o erro for menor */
/* 23o teste: com DUAS paredes e o robo paralelo a elas o rumo e' certo --
 * ai' aceita corrigir ate' isto (erro de escala acumulado, ~6 graus por 180) */
#define SOLVER_HEADING_ABSORB2_DEG  15.0f
/* meia-volta para o lado que traz o rumo total de volta a 0 (escala nao soma) */
#define SOLVER_U_TURN_BALANCE       1
#define SOLVER_HEADING_BIAS_MAX     180.0f  /* 15o teste: 20 era pouco -- o erro de
                                             * escala do giroscopio soma a cada
                                             * giro e passou de 20 no fim da
                                             * exploracao; o robo andou torto na
                                             * corrida e raspou na parede. Quem
                                             * segura o disparo encalhado e' o
                                             * ABSORB de 6 graus + robo andando. */
#define SOLVER_HEADING_MIN_MMS      60.0f   /* velocidade MEDIDA minima        */
/* Lateral colado na parede (prox acima disto): o rumo do giroscopio esta'
 * errado ou o robo esta' torto -- deixa so' o centralizador afastar o robo
 * da parede (sem segurar rumo e sem aprender rumo nesse instante). */
/* Peso do "segurar rumo" do giroscopio quando ha' parede do lado (0..1). */
#define SOLVER_HEADING_WALL_SHARE   0.3f
/* Recalibra laterais (centro) e frontais (aberto) na largada, com a luz atual. */
#define SOLVER_START_AUTOCAL        1
#define SOLVER_SIDE_CLOSE_PROX      1200

void solver_init(void);

/** @brief Chamar a cada NAV_PERIOD_MS. */
void solver_update(void);

/** @brief true quando terminou (ou abortou). */
bool solver_finished(void);

/** @brief true se parou por erro (ver solver_dump). */
bool solver_aborted(void);

/** @brief Imprime o registro passo a passo e o mapa. */
void solver_dump(void);
