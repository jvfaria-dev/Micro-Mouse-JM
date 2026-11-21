/**
 ******************************************************************************
 * @file    encoder_cal.h
 * @brief   Calibracao do encoder: descobre quantos ticks o encoder da' por
 *          volta completa da RODA, girando na mao.
 *
 * Por que isso importa: TODO o RPM e toda a odometria dependem da constante
 * ENCODER_TICKS_PER_REV (= PPR x 4 x reducao) do board.h. Esses tres numeros
 * sao chutados a partir do datasheet; se qualquer um estiver errado, o RPM
 * mostrado esta errado na mesma proporcao, e o PID vai ser ajustado em cima
 * de um numero que nao existe.
 *
 * COMO USAR
 *   1. Ponha APP_MODE = APP_MODE_ENCODER_CAL, compile e grave.
 *   2. Os motores ficam em roda livre (da' pra girar com o dedo).
 *   3. Marque um ponto da roda com fita, alinhe com uma referencia fixa.
 *   4. Gire a roda DEVAGAR, no sentido de "para frente", contando as voltas.
 *      Dez voltas da' uma media boa.
 *   5. Leia a linha 'ticks' e divida pelo numero de voltas que voce deu.
 *      O firmware ja' imprime a divisao pronta para 1, 5 e 10 voltas.
 *   6. Ponha o resultado no board.h.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* Periodo de impressao (ms) */
#define ENCODER_CAL_PRINT_MS    300u

/* ---------------------------------------------------------------------------
 * TESTE MOTORIZADO (opcional, mas decisivo)
 * ---------------------------------------------------------------------------
 * Com ENCODER_CAL_RUN_DUTY > 0, depois de 3 s parado o firmware liga os dois
 * motores num duty fixo por ENCODER_CAL_RUN_MS e imprime quantos ticks contou.
 *
 * POR QUE ISSO IMPORTA: o teste manual e' feito com o motor DESLIGADO, sem
 * ruido nenhum. Se o numero de ticks por volta medido girando bater com o
 * medido na mao, a contagem e' confiavel. Se NAO bater, o motor esta' injetando
 * pulsos falsos na linha do encoder, e ai' nenhum RPM medido presta.
 *
 * COMO USAR
 *   1. Escolha um duty em que a roda gire devagar o bastante para contar.
 *   2. Filme a roda com o celular durante os 10 s (camera lenta ajuda).
 *   3. Conte as voltas no video e divida os ticks impressos por esse numero.
 *
 * Deixe em 0 para so' usar o modo manual.
 */
#define ENCODER_CAL_RUN_DUTY    0
#define ENCODER_CAL_RUN_MS      10000u
#define ENCODER_CAL_RUN_DELAY_MS 3000u

void encoder_cal_init(void);

/** @brief Chamar a cada CONTROL_PERIOD_MS. */
void encoder_cal_update(void);
