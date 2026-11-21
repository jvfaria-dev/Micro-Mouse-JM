/**
 ******************************************************************************
 * @file    app_main.h
 * @brief   Aplicacao SEM RTOS: escalonador cooperativo no laco principal e
 *          escolha do modo.
 *
 * ---------------------------------------------------------------------------
 * COMO SUBSTITUI AS TAREFAS DO FREERTOS
 * ---------------------------------------------------------------------------
 * O SysTick do HAL ja' conta milissegundos (HAL_GetTick). O laco principal
 * olha esse relogio e, para cada milissegundo que passou, roda o que vence
 * naquele milissegundo:
 *
 *   antiga tarefa   periodo   agora roda quando
 *   ---------------------------------------------------------------
 *   tControl          1 ms    todo ms
 *   tSensors          5 ms    ms % 5   == 1
 *   tNav             10 ms    ms % 10  == 3
 *   tTelemetry      100 ms    ms % 100 == 7
 *
 * As fases (1, 3, 7) sao de proposito: as tarefas pesadas nunca caem no
 * mesmo milissegundo, entao nenhum ms acumula sensores + navegacao +
 * telemetria de uma vez.
 *
 * O que fazia o RTOS necessario era o printf bloqueante: 100 caracteres a
 * 115200 travam 8,7 ms. Agora o printf so' copia para um buffer e a UART e'
 * alimentada aos poucos pelo proprio laco (port_debug_poll). Com isso nada
 * no laco bloqueia por mais que ~0,5 ms (a leitura dos 4 sensores), e o
 * controle continua no seu 1 kHz.
 *
 * Se o laco atrasar mesmo assim (calibracao que espera de proposito, por
 * exemplo), os milissegundos perdidos sao recuperados em sequencia, ate' um
 * limite -- passou disso, o relogio e' realinhado em vez de rodar o PID
 * cinquenta vezes seguidas.
 ******************************************************************************
 */
#pragma once

#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "sensors.h"
#include "debug.h"
#include "ihm.h"
#include "imu.h"
#include "speed_ctl.h"
#include "nav.h"
#include "cal_modes.h"
#include "motor_test.h"
#include "pwm_test.h"
#include "encoder_cal.h"
#include "trace.h"
#include "imu_test.h"
#include "solver.h"

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * QUAL MODO RODAR
 * ---------------------------------------------------------------------------
 * ORDEM PARA O CHASSI NOVO NA F411
 *
 *   1. MOTOR_TEST   -> fiacao, sentido de cada roda, encoder certo de cada lado
 *   2. SENSOR_CAL   -> os 8 numeros dos sensores IR (board.h, secao 3d)
 *   3. TURN_CAL     -> TURN_TICKS_PER_90, o giro de 90 graus
 *   4. MAZE         -> seguidor de paredes (labirinto de caminho unico)
 *   5. SOLVE        -> flood fill: explora ate' o centro, volta e corre
 *
 *   IMU_TEST        -> confere o MPU-6050 (opcional, pode ser a qualquer hora)
 *
 * Opcionais: PWM_TEST (refaz o ganho com a LiPo), ENCODER_CAL, ODOM_CAL.
 */
#define APP_MODE_MOTOR_TEST     0
#define APP_MODE_PWM_TEST       1
#define APP_MODE_ENCODER_CAL    2
#define APP_MODE_ODOM_CAL       3
#define APP_MODE_SENSOR_CAL     4
#define APP_MODE_MAZE           5
#define APP_MODE_TURN_CAL       6
#define APP_MODE_IMU_TEST       7
#define APP_MODE_SOLVE          8   /* flood fill: explora, volta e corre */

#ifndef APP_MODE
#define APP_MODE                APP_MODE_MAZE
#endif

/* Modos que usam controle de velocidade, sensores e navegacao. */
#define APP_USES_NAV  ((APP_MODE >= APP_MODE_ODOM_CAL) && (APP_MODE <= APP_MODE_TURN_CAL))

/** @brief Inicializa o hardware e o modo. Chamar no USER CODE 2 do main.c. */
void app_init(void);

/** @brief Uma volta do laco. Chamar dentro do while(1) do main.c. */
void app_loop(void);

/** @brief Para os motores. Usado pelo Error_Handler. */
void app_emergency_stop(void);
