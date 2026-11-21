/**
 ******************************************************************************
 * @file    pwm_test.h
 * @brief   Ensaio de caracterizacao do PWM: varre o duty em degraus e mede
 *          a velocidade resultante em cada patamar.
 *
 * O que este ensaio responde:
 *   1. ZONA MORTA  - a partir de que duty a roda sai do lugar
 *   2. HISTERESE   - em que duty ela para, descendo (sempre menor que o de
 *                    partida; a diferenca e' o atrito estatico)
 *   3. LINEARIDADE - se dobrar o duty dobra o RPM, ou se satura antes
 *   4. CASAMENTO   - se os dois motores dao o mesmo RPM no mesmo duty
 *   5. RPM MAXIMO  - a velocidade de topo real, com a sua bateria
 *
 * A saida sai na UART em formato de tabela com ';' como separador, entao da'
 * pra copiar direto para o Excel e plotar a curva.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Incremento do duty entre um degrau e o proximo (por-mil).
 * 25 da' 41 patamares por rampa; use 50 se quiser um ensaio mais rapido. */
#define PWM_TEST_STEP           25

/* Duty maximo do ensaio = o teto do motor (board.h: 6 V / bateria cheia).
 * Acima disso o motor.c satura e a curva sairia achatada no fim. */
#define PWM_TEST_MAX_DUTY       MOTOR_DUTY_MAX

/* Tempo esperando a velocidade estabilizar antes de medir (ms) */
#define PWM_TEST_SETTLE_MS      500u

/* Janela de medicao da velocidade em cada patamar (ms) */
#define PWM_TEST_MEASURE_MS     250u

/* Acima deste RPM a roda e' considerada "girando" (milesimos de RPM).
 * 5 RPM na roda ja' e' movimento inequivoco e imune a ruido de encoder. */
#define PWM_TEST_MOVE_RPM_MILLI 5000

/* 1 = repete o ensaio em loop; 0 = roda uma vez e para os motores */
#define PWM_TEST_LOOP           1

void pwm_test_init(void);

/** @brief Chamar a cada CONTROL_PERIOD_MS. */
void pwm_test_update(void);

/** @brief Nome da fase atual (para a telemetria). */
const char *pwm_test_phase_name(void);

/** @brief true quando o ensaio terminou (so' relevante com PWM_TEST_LOOP=0). */
bool pwm_test_done(void);
