/**
 ******************************************************************************
 * @file    motor_test.h
 * @brief   Rotina de teste dos motores e encoders (maquina de estados
 *          nao-bloqueante, avanca a cada 1 ms).
 *
 * Objetivo do teste: descobrir, sem escrever codigo novo, se
 *   1. cada motor gira;
 *   2. gira no sentido certo (duty positivo = roda para frente);
 *   3. cada encoder conta;
 *   4. o encoder conta com o sinal certo (frente = positivo);
 *   5. os dois lados tem velocidade parecida no mesmo duty;
 *   6. qual e' a zona morta (duty minimo para sair do lugar).
 *
 * A sequencia roda em loop. Se um dia houver botao de usuario
 * (BOARD_HAS_BUTTON em board.h), ele pula para a proxima etapa.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * MODO BANCADA (depuracao com multimetro)
 * ---------------------------------------------------------------------------
 * Com 1, a sequencia nao roda: os dois motores ficam travados em
 * MOTOR_TEST_STATIC_DUTY para sempre. Serve tanto para medir os pinos com
 * calma quanto para simplesmente deixar o motor girando sem parar.
 * Volte para 0 quando terminar.
 */
#define MOTOR_TEST_STATIC       0

/* Duty do modo bancada, em por-mil. 1000 = 100%, motor a plena tensao.
 *
 * ATENCAO: em 1000 o motor recebe a tensao INTEIRA do VM. Se a sua bateria
 * for maior que os 6 V nominais do motor, reduza na proporcao:
 *     duty = 6000 / tensao_da_bateria
 * Ex.: bateria de 7,4 V (2S LiPo) -> 6000/7,4 = 810.
 *      bateria de 8,4 V           -> 6000/8,4 = 714.
 */
#define MOTOR_TEST_STATIC_DUTY  MOTOR_DUTY_MAX   /* teto de 6 V, board.h */

/* Duty nominal usado nas etapas de sentido, em por-mil (0..1000).
 * Comece baixo com o carrinho no ar e va' subindo. */
#define MOTOR_TEST_DUTY         400

/* Duty maximo da etapa de varredura */
#define MOTOR_TEST_SWEEP_DUTY   MOTOR_DUTY_MAX

/* Tempo de rampa de subida/descida em cada etapa (ms) */
#define MOTOR_TEST_RAMP_MS      600u

void motor_test_init(void);

/** @brief Chamar a cada CONTROL_PERIOD_MS. */
void motor_test_update(void);

/** @brief Nome curto da etapa atual (para a telemetria). */
const char *motor_test_step_name(void);
