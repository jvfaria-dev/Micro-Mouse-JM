/**
 ******************************************************************************
 * @file    cal_modes.h
 * @brief   Dois modos de calibracao que precisam ser rodados ANTES do
 *          labirinto, nesta ordem.
 *
 * ---------------------------------------------------------------------------
 * 1) APP_MODE_SENSOR_CAL -- calibracao dos sensores IR
 * ---------------------------------------------------------------------------
 * Motores parados. Imprime as quatro leituras cruas continuamente, junto com
 * o minimo e o maximo vistos desde que ligou.
 *
 *   a. Deixe o carrinho no meio do corredor, sem nada perto: anote os
 *      valores. Esses sao os RAW_FAR.
 *   b. Encoste uma parede BRANCA na distancia de referencia de cada sensor
 *      (para os laterais, o carrinho centralizado no corredor; para os
 *      frontais, a uma celula de distancia): anote. Esses sao os RAW_NEAR.
 *   c. Ponha os oito numeros no board.h.
 *
 * O firmware nao adivinha se o seu modulo da' tensao alta ou baixa vendo
 * branco -- ele so' interpola entre FAR e NEAR. Entao se NEAR sair MENOR
 * que FAR, tudo bem, e' so' colocar assim mesmo.
 *
 * ---------------------------------------------------------------------------
 * 2) APP_MODE_ODOM_CAL -- calibracao da geometria
 * ---------------------------------------------------------------------------
 * Sem isso, distancia e angulo no labirinto sao chute. Duas etapas
 * alternadas, com pausa entre elas:
 *
 *   RETA  : anda ODOM_CAL_DISTANCE_MM comandados e para.
 *           Meca com regua quanto andou DE VERDADE e corrija:
 *               WHEEL_DIAMETER_MM_novo = atual * (medido / comandado)
 *
 *   GIRO  : gira ODOM_CAL_TURNS voltas completas no proprio eixo e para.
 *           Se parou ANTES de fechar a volta, a base esta' grande demais:
 *               WHEEL_BASE_MM_novo = atual * (girado_real / comandado)
 *           Marque o chao com fita para ver onde parou.
 *
 * Faca a reta PRIMEIRO: o giro depende do diametro da roda ja' estar certo.
 ******************************************************************************
 */
#pragma once

#include "board.h"    /* SPEED_TURN_MMS / SPEED_CRUISE_MMS */

#include <stdint.h>

/* --- calibracao dos sensores --- */
#define SENSOR_CAL_PRINT_MS     250u

/* --- calibracao da odometria --- */

/* Com 1, pula a etapa da reta e fica so' repetindo o giro. Use quando
 * estiver ajustando SO' o WHEEL_BASE_MM: evita o carrinho sair passeando
 * pela sala entre uma medicao e outra, e voce mede varias vezes seguidas
 * no mesmo lugar. */
#define ODOM_CAL_SKIP_STRAIGHT  0

#define ODOM_CAL_DISTANCE_MM    500.0f  /* comprimento da reta            */

/* UMA volta e' mais facil de medir que duas: da' pra ver o quanto faltou
 * com transferidor, e acumula menos escorregamento de roda. */
#define ODOM_CAL_TURNS          1.0f
#define ODOM_CAL_SPEED_MMS      200.0f
#define ODOM_CAL_TURN_MMS       150.0f
#define ODOM_CAL_PAUSE_MS       4000u   /* pausa para voce medir          */

/* ---------------------------------------------------------------------------
 * 3) APP_MODE_TURN_CAL -- calibracao do giro (a que importa de verdade)
 * ---------------------------------------------------------------------------
 * O carrinho da' quatro giros de 90 graus com uma pausa entre eles, ou seja
 * uma volta completa, e para. Marque a posicao inicial com fita no chao.
 *
 *   fechou a volta certinho -> TURN_TICKS_PER_90 esta' bom
 *   parou ANTES de fechar   -> AUMENTE TURN_TICKS_PER_90
 *   passou da volta         -> DIMINUA
 *
 *   valor novo = atual * (360 / graus_girados_de_verdade)
 *
 * Quatro giros e nao um: o erro de cada giro aparece multiplicado por quatro,
 * o que torna visivel a olho nu um desvio que num giro unico seria discutivel.
 * E como sao quatro partidas e quatro paradas, a calibracao ja' inclui a
 * inercia de arrancar e frear -- que e' justamente o que uma conta puramente
 * geometrica deixaria de fora.
 */
#define TURN_CAL_QUARTERS       4       /* 4 x 90 graus = volta completa   */
/* Gira na MESMA velocidade do labirinto, e nao num valor proprio.
 * Escorregamento depende de velocidade: calibrar a 150 mm/s e depois correr a
 * 45 mede uma coisa e usa outra. Amarrado ao SPEED_TURN_MMS, os dois andam
 * juntos sozinhos quando voce mexer em um. */
#define TURN_CAL_SPEED_MMS      SPEED_TURN_MMS
#define TURN_CAL_GAP_MS         700u    /* pausa entre um giro e o proximo */
#define TURN_CAL_REPORT_MS      8000u   /* pausa para voce medir e anotar  */

/* ---------------------------------------------------------------------------
 * A RETA que vem antes dos giros (roda uma vez so', logo apos ligar)
 * ---------------------------------------------------------------------------
 * Serve para responder UMA pergunta que os testes de bancada nao conseguiram:
 * a roda direita conta 13% menos ticks que a esquerda -- e' o ENCODER contando
 * menos, ou e' o MOTOR girando menos?
 *
 * As duas hipoteses pedem correcoes OPOSTAS, e por isso nao da' pra chutar:
 *
 *   MOTOR mais lento    -> a malha fechada ja' resolve sozinha. Nao mexa em
 *                          nada; mexer estragaria.
 *   ENCODER contando a
 *   menos               -> o PID acredita que a roda esta' devagar e a
 *                          acelera demais. O carrinho puxa para a ESQUERDA
 *                          e so' WHEEL_R_SCALE conserta.
 *
 * O teste distingue as duas em tres segundos, porque aqui o PID esta' ligado:
 *
 *   andou reto          -> era o motor. Deixe WHEEL_R_SCALE = 1.000
 *   puxou para ESQUERDA -> era o encoder direito. AUMENTE WHEEL_R_SCALE
 *   puxou para DIREITA  -> AUMENTE WHEEL_L_SCALE
 *
 * Mexa de 0,02 em 0,02 (1.020, 1.040...). Nao precisa ficar perfeito: no
 * corredor os sensores laterais corrigem o resto.
 */
#define TURN_CAL_STRAIGHT_MS    3000u
/* Mesma logica: a reta de aquecimento roda na velocidade de cruzeiro real. */
#define TURN_CAL_STRAIGHT_MMS   SPEED_CRUISE_MMS
#define TURN_CAL_STRAIGHT_PAUSE 4000u

void turn_cal_init(void);
void turn_cal_update(void);
const char *turn_cal_state_name(void);

void sensor_cal_init(void);
void sensor_cal_update(void);

void odom_cal_init(void);
void odom_cal_update(void);

/** @brief Nome do estado atual da calibracao, para a telemetria. */
const char *odom_cal_state_name(void);
