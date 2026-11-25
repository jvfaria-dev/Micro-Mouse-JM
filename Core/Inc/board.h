/**
 ******************************************************************************
 * @file    board.h
 * @brief   Mapa de hardware e constantes de controle do MicroJM -- 
 *          Blue Pill STM32F103C8T6.
 *          Totalmente compativel com a arquitetura de app do fw_micromouse_f411.
 ******************************************************************************
 */
#ifndef BOARD_H
#define BOARD_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 1. MAPA DE PINOS E PERIFERICOS (MicroJM - Blue Pill)
 * ==========================================================================
 *
 *  Ponte H TB6612FNG (TIM1 PWM + GPIOs de direcao):
 *      PA8  -> TIM1_CH1  -> PWM Motor Esquerdo
 *      PA9  -> TIM1_CH2  -> PWM Motor Direito
 *      PB15 -> AIN1_1    -> Direcao 1 Motor Esquerdo
 *      PB14 -> AIN2_1    -> Direcao 2 Motor Esquerdo
 *      PA11 -> AIN1_2    -> Direcao 1 Motor Direito
 *      PA10 -> AIN2_2    -> Direcao 2 Motor Direito
 *
 *  Encoders (TIM2 e TIM3, ambos 16 bits na Blue Pill):
 *      PA15 -> TIM2_CH1  -> Encoder Esquerdo canal A  (TIM2 remap completo)
 *      PB3  -> TIM2_CH2  -> Encoder Esquerdo canal B
 *      PB4  -> TIM3_CH1  -> Encoder Direito canal A
 *      PB5  -> TIM3_CH2  -> Encoder Direito canal B
 *
 *  Sensores Sharp IR (ADC1, 5 canais):
 *      (ligacao real conferida em 20/11/2026)
 *      PA3  -> ADC1_IN3  -> SL1 (Sharp Lateral Direito)
 *      PB0  -> ADC1_IN8  -> SL2 (Sharp Lateral Esquerdo)
 *      PA4  -> ADC1_IN4  -> SL3 (Diagonal Direita)
 *      PA6  -> ADC1_IN6  -> SL4 (Diagonal Esquerda)
 *      PA5  -> ADC1_IN5  -> SL5 (Sharp Central Frontal)
 *      PB1  -> livre (antes era o SL5)
 *
 *  IMU MPU-6050 (I2C1):
 *      PB6  -> I2C1_SCL
 *      PB7  -> I2C1_SDA
 *      Endereco 0x68 (AD0 no GND)
 *
 *  LEDs e IHM:
 *      PA1  -> LED_1 (LED de status principal)
 *      PA0  -> LED_2 (LED de status secundario)
 *      PC13 -> LED onboard Blue Pill (ativo em nivel BAIXO)
 *      PA7  -> SW2 (botao de usuario)
 *      PB12 -> BUZZER
 *
 *  Telemetria serial (USART3, 115200 8N1, 3,3 V):
 *      PB10 -> USART3_TX -> ligar no RX do adaptador USB-TTL
 *      PB11 -> USART3_RX -> ligar no TX do adaptador USB-TTL
 *      GND  -> GND do adaptador (obrigatorio)
 */

#define BOARD_MAP_DESC  "MicroJM Blue Pill (TB6612 TIM1, ENC TIM2 PA15/PB3 + TIM3 PB4/PB5, 5 Sharp ADC1, IMU I2C1)"

/* --- Ponte H TB6612 ------------------------------------------------------- */
#define BOARD_MOTOR_TIM             htim1

#define BOARD_MOTOR_L_PWM_CH        TIM_CHANNEL_1   /* PA8 */
#define BOARD_MOTOR_R_PWM_CH        TIM_CHANNEL_2   /* PA9 */

#define BOARD_MOTA_IN1_PORT         AIN1_1_GPIO_Port /* PB15 */
#define BOARD_MOTA_IN1_PIN          AIN1_1_Pin
#define BOARD_MOTA_IN2_PORT         AIN2_1_GPIO_Port /* PB14 */
#define BOARD_MOTA_IN2_PIN          AIN2_1_Pin

#define BOARD_MOTB_IN1_PORT         AIN1_2_GPIO_Port /* PA11 */
#define BOARD_MOTB_IN1_PIN          AIN1_2_Pin
#define BOARD_MOTB_IN2_PORT         AIN2_2_GPIO_Port /* PA10 */
#define BOARD_MOTB_IN2_PIN          AIN2_2_Pin

#define BOARD_PWM_FREQ_HZ           20000u
#define MOTOR_PWM_PERIOD            3199u           /* ARR do TIM1: 64 MHz / 3200 = 20 kHz (PSC=0) */
#define MOTOR_RATED_MV              6000
#define BATTERY_FULL_MV             8400
#define MOTOR_DUTY_MAX              1000

#define MOTOR_L_DEADBAND            25
#define MOTOR_R_DEADBAND            25

#define MOTOR_L_INVERT              1   /* roda esquerda girava ao contrario (MOTOR_TEST 20/11) */
#define MOTOR_R_INVERT              0

/* --- Encoders ------------------------------------------------------------- */
/* CHAVE DOS ENCODERS
 *   1 = encoders ligados (normal): PID de velocidade, odometria real e
 *       protecao de "roda muda".
 *   0 = encoders DESLIGADOS (placa sem encoder ligado): os timers nao contam,
 *       o controle vira malha aberta (so' feedforward), a odometria e'
 *       ESTIMADA pela velocidade comandada e a protecao de roda muda fica
 *       desligada. Distancias e giros ficam aproximados.
 * Para religar, troque para 1 e recompile. Nada mais precisa mudar. */
#ifndef BOARD_USE_ENCODERS
#define BOARD_USE_ENCODERS          1
#endif
#define BOARD_ENC_L_TIM             htim2           /* TIM2: PA15/PB3  */
#define BOARD_ENC_R_TIM             htim3           /* TIM3: PB4/PB5   */

#define BOARD_ENC_L_IS_32BIT        0               /* TIM2 no STM32F103 e' 16 bits */
#define BOARD_ENC_R_IS_32BIT        0               /* TIM3 no STM32F103 e' 16 bits */
#define BOARD_ENC_INPUT_FILTER      6u

/* Pinos lidos como GPIO para o teste do ENCODER_CAL */
#define BOARD_ENC_L_A_PORT          GPIOA
#define BOARD_ENC_L_A_PIN           GPIO_PIN_15
#define BOARD_ENC_L_B_PORT          GPIOB
#define BOARD_ENC_L_B_PIN           GPIO_PIN_3
#define BOARD_ENC_R_A_PORT          GPIOB
#define BOARD_ENC_R_A_PIN           GPIO_PIN_4
#define BOARD_ENC_R_B_PORT          GPIOB
#define BOARD_ENC_R_B_PIN           GPIO_PIN_5

#define ENCODER_L_INVERT            1   /* MOTOR_TEST 21/11: ESQ FRENTE dava ticks negativos */
#define ENCODER_R_INVERT            0

#define ENCODER_PPR                 7.0f
#define ENCODER_GEAR_RATIO          20.5f
#define ENCODER_QUADRATURE          4.0f

#define ENCODER_TICKS_PER_MOTOR_REV ((int32_t)(ENCODER_PPR * ENCODER_QUADRATURE))
/* ENCODER_CAL 23/11, 10 voltas na mao (ticks ESQ / DIR):
 *   teste 2: 5903 / 5880    teste 3: 5757 / 5781
 * Media dos dois: ESQ 583,0  DIR 583,1 por volta -> rodas iguais (a
 * diferenca troca de lado entre os testes = erro de contagem, ~1/4 volta).
 * Canais A e B iguais = sem pulso perdido. Reducao real ~20,8:1.
 * Era 573. (O 1o teste deu 615/522: rodas giradas alternadas.)
 * O erro que sobrar entra no WHEEL_DIAMETER_MM pelo ODOM_CAL. */
#define ENCODER_TICKS_PER_REV       583

/* --- Geometria do Robo ---------------------------------------------------- */
#define WHEEL_DIAMETER_MM           22.0f   /* ODOM_CAL 23/11: 500 mm comandados = 500 mm medidos (com 583 ticks/volta) */
/* 23/11: base EFETIVA tirada do TURN_CAL (1035 ticks de diferenca = 90 graus):
 *   2 * 1035 * MM_PER_TICK / pi = 2 * 1035 * 0,11855 / 3,1416 = 78,1 mm
 * (a geometrica e' 82,5; a efetiva sai menor por escorregamento no giro).
 * So' afeta o angulo calculado pelos encoders ("ang" nos logs). */
#define WHEEL_BASE_MM               78.1f   /* era 82.5 */
#define WHEEL_CIRC_MM               (3.14159265f * WHEEL_DIAMETER_MM)
#define MM_PER_TICK                 (WHEEL_CIRC_MM / (float)ENCODER_TICKS_PER_REV)

#define WHEEL_L_SCALE               1.000f
#define WHEEL_R_SCALE               1.000f

/* --- Labirinto e Navegacao ------------------------------------------------ */
#define MAZE_CELL_MM                180.0f
#define MAZE_WALL_THICK_MM          15.0f
#define MAZE_CORRIDOR_MM            (MAZE_CELL_MM - MAZE_WALL_THICK_MM)

#define TICKS_PER_CELL              ((int32_t)(MAZE_CELL_MM / MM_PER_TICK))
#define TICKS_PER_90DEG             ((int32_t)((3.14159265f * WHEEL_BASE_MM / 4.0f) / MM_PER_TICK))

/* 23/11: o 691 foi calibrado quando BOARD_USE_ENCODERS era 0 -- os "ticks"
 * eram ESTIMADOS pela velocidade comandada, nao medidos. Com os encoders de
 * verdade, 691 de diferenca = ~57 graus (o MAZE mostrou "ang 54" depois de
 * cada giro de "90": o robo ficava torto, via a quina como parede na frente
 * e virava de novo). Valor GEOMETRICO para 90 graus:
 *   (pi * WHEEL_BASE_MM / 2) / MM_PER_TICK = 82,5 * 583 / (2 * 22) = 1093
 * TURN_CAL 23/11 com 1093: 4 x 90 p/ a esquerda passou ~10 graus (370).
 *   1093 * 360 / 370 = 1063,5 -> 1064
 * 2o TURN_CAL com 1064: passou ~5 graus (365).
 *   1064 * 360 / 365 = 1049,4 -> 1049
 * 3o TURN_CAL com 1049, ja' com o feedforward novo (sem tranco): ~365.
 *   1049 * 360 / 365 = 1034,6 -> 1035
 * (valor novo = atual * 360 / girado de verdade; refaca se precisar) */
#ifndef TURN_TICKS_PER_90
#define TURN_TICKS_PER_90           1035    /* 691 (estimado) -> 1093 (geometrico) -> 1064 -> 1049 -> 1035 (TURN_CAL) */
#endif

#define STRAIGHT_TRIM_MMS           0.0f

/* --- Sensores Sharp IR (ADC1) --------------------------------------------- */
#define BOARD_SENSOR_ADC            hadc1

#define BOARD_SENSOR_CH_SL1         ADC_CHANNEL_3   /* PA3 - lateral direito    */
#define BOARD_SENSOR_CH_SL2         ADC_CHANNEL_8   /* PB0 - lateral esquerdo   */
#define BOARD_SENSOR_CH_SL3         ADC_CHANNEL_4   /* PA4 - diagonal direita   */
#define BOARD_SENSOR_CH_SL4         ADC_CHANNEL_6   /* PA6 - diagonal esquerda  */
#define BOARD_SENSOR_CH_SL5         ADC_CHANNEL_5   /* PA5 - central frontal    */

#define BOARD_SENSOR_SAMPLETIME     ADC_SAMPLETIME_71CYCLES_5
#define SENSOR_RAW_MAX              4095
/* 24/11: os Sharp sao GP2Y0A51SK0F (2-15 cm), que atualizam a cada
 * 16,5 +- 3,7 ms (datasheet, Timing Chart). Constante de tempo ~20 ms
 * (alpha 250) acompanha isso. O 125 de 23/11 era para o GP2Y0A21 (38 ms). */
#define SENSOR_FILTER_ALPHA         250     /* 23/11 125 -> 24/11 de volta a 250 */

/* Calibracao padrao Sharp IR */
#define SENSOR_L_RAW_FAR            500
#define SENSOR_L_RAW_NEAR           2110   /* lateral a 2,5 cm (robo centrado, corredor 16 cm, robo 11 cm) */
#define SENSOR_FL_RAW_FAR           500
#define SENSOR_FL_RAW_NEAR          2800
#define SENSOR_FR_RAW_FAR           500
#define SENSOR_FR_RAW_NEAR          2800
#define SENSOR_R_RAW_FAR            500
#define SENSOR_R_RAW_NEAR           2110   /* idem */
#define SENSOR_CENTER_RAW_FAR       500
#define SENSOR_CENTER_RAW_NEAR      2800

#define SENSOR_WALL_THRESHOLD       550  /* era 400: aberto <=220, parede >=700; 430 dava parede falsa (21/11) */
#define SENSOR_FRONT_STOP           400  /* era 250: parede 1 celula adiante ~250-370, na celula 550-900 (21/11) */
/* "Parede a frente" usa os diagonais (FR_E/FR_D) junto com o central?
 *   0 = so' o central (SL5, PA5). No MicroJM os diagonais enxergam as paredes
 *       LATERAIS do corredor e davam falso "parede a frente" (teste 21/11).
 *   1 = media do central com os diagonais que passarem do limiar (antigo). */
#define SENSOR_FRONT_USE_DIAGONALS  0
#define SENSOR_CENTER_TRIM          0
/* 23/11 (DECIDE_TEST, robo parado): central com parede na frente
 *   robo no CENTRO da celula: 830-836 | COLADO na parede: 650-660
 * Colado le' MENOS que no centro: abaixo de ~6 cm a curva do GP2Y0A21 cai
 * (datasheet Fig. 2, pico ~6 cm). Com 700 o robo parava ANTES do centro
 * (vindo de longe a leitura cruza 700 antes de chegar a 832). Agora para
 * logo abaixo do valor do centro; se passar do pico e entrar na zona
 * morta, ele RECUA (SENSOR_DEAD_*, abaixo). */
#define SENSOR_FRONT_CENTER         800  /* era 700 (antes 550) */
/* ZONA MORTA DO SHARP (24/11). GP2Y0A51SK0F: faixa 2-15 cm, pico da curva
 * em ~1,3 cm; mais perto que isso a tensao DESPENCA ate' zero (datasheet,
 * "Example of output distance characteristics"). Medido no MicroJM: central
 * 2 cm ~840, 4 cm ~452, 6 cm ~303, 8 cm ~178.
 * Deteccao (sensors_dead_update): a leitura estava perto do topo da curva
 * (maximo >= MIN_PEAK) e caiu DROP abaixo desse maximo por CONFIRM_MS
 * seguidos -> o robo entrou na zona morta. Reacao: para e RECUA devagar ate'
 * a leitura subir (volta ao pico) e cair de novo ate' SENSOR_FRONT_CENTER. */
#define SENSOR_DEAD_MIN_PEAK        700     /* ~2,3 cm                       */
#define SENSOR_DEAD_DROP            80      /* queda abaixo do maximo        */
#define SENSOR_DEAD_CONFIRM_MS      35u     /* ~2 atualizacoes do sensor     */
#define SENSOR_BACKOFF_MMS          20.0f   /* recuo (< SPEED_FAULT_MIN_TARGET) */
#define SENSOR_BACKOFF_MAX_MM       25.0f   /* recuo maximo                  */
#define SENSOR_BACKOFF_RISE         30      /* subida minima p/ "achou o pico" */
#define SENSOR_BACKOFF_TIMEOUT_MS   2500u
#define SENSOR_TURN_HUG_PROX        1000

/* --- Controle de Velocidade PID ------------------------------------------- */
/* 23/11 (ODOM_CAL, regime): 200 mm/s -> duty ~268 ; 60 mm/s -> duty ~147.
 * Roda 22 mm: 200 mm/s = 173,6 rpm, 60 mm/s = 52,1 rpm.
 *   inclinacao = (268-147)/(173,6-52,1) = 1,0 duty/rpm -> 1,0 rpm/duty
 *   estatico   = 268 - 173,6 = ~95
 * Antes (0,25 / 40) o feedforward pedia duty ~734 para 200 mm/s: na partida
 * a roda disparava a 350-390 mm/s, PATINAVA e o robo saia torto (o encoder
 * nao ve' patinacao, entao a correcao de rumo nao tinha como consertar). */
#define SPEED_GAIN_RPM_PER_DUTY     1.00f   /* era 0.25 */
#define SPEED_FF_STATIC             95.0f   /* era 40   */

#define PID_KP                      0.60f
#define PID_KI                      12.00f
#define PID_KD                      0.00f
#define PID_I_LIMIT                 500.0f

/* SPEED_FAULT_MS, SPEED_FAULT_MIN_TARGET, SPEED_FAULT_MIN_DUTY e
 * SPEED_FAULT_MOVING_MMS ficam SO' no speed_ctl.h. Estavam definidos aqui
 * tambem, com outros valores, e no speed_ctl.c valia o daqui (800 ms) em
 * vez dos 3000 ms ajustados la'. */
#define SPEED_FAULT_WRONG_MS        500u

/* --- Velocidades ---------------------------------------------------------- */
#define SPEED_CRUISE_MMS            228.0f  /* 24/11: 100 -> 120 (+20%) -> 228 (+90%) */
#define SPEED_TURN_MMS              68.0f   /* 24/11: +50% (era 45). TURN_CAL usa esta mesma velocidade: recalibre TURN_TICKS_PER_90 */
#define SPEED_APPROACH_MMS          40.0f

#define WALL_FOLLOW_KP              0.04f
/* 23/11: robo TREMENDO no corredor (laterais alternando a cada amostra,
 * zigue-zague de ~5 Hz). O Sharp atualiza em degraus (~38 ms) e a derivada
 * e' tirada a cada 10 ms: cada degrau virava um pico que, com KD 0,030,
 * saturava o giro em +-60 mm/s. Derivativo menor e mais filtrado.
 * Se ainda tremer: KD 0.005. Se ficar "mole" (bate de lado): volte p/ 0.020. */
#define WALL_FOLLOW_KD              0.010f  /* era 0.030 */
#define WALL_FOLLOW_D_ALPHA         0.10f   /* era 0.30  */
/* SEGURA O RUMO PELOS ENCODERS (23/11). Cada roda tem o seu PID de
 * velocidade, mas nada devolvia a DISTANCIA que uma roda perdia num tranco:
 * no ODOM_CAL a esquerda caiu p/ 166 mm/s por ~60 mm e o robo terminou a
 * reta 7 graus torto. Agora, sem parede (e sem giroscopio), o robo mantem a
 * diferenca de ticks entre as rodas: giro = -KP * (diferenca em mm).
 * KP em 1/s (3 -> o erro cai a ~1/e em 0,17 s). 0 desliga. */
#define SPEED_HOLD_KP               3.0f
#define SPEED_HOLD_MAX_MMS          30.0f
#define WALL_FOLLOW_MAX_MMS         60.0f

/* --- IMU MPU-6050 (I2C1) -------------------------------------------------- */
#define BOARD_IMU_I2C               hi2c1
#define BOARD_IMU_ADDR              0x68u
#define BOARD_IMU_SCL_PORT          GPIOB
#define BOARD_IMU_SCL_PIN           GPIO_PIN_6
#define BOARD_IMU_SDA_PORT          GPIOB
#define BOARD_IMU_SDA_PIN           GPIO_PIN_7

#define IMU_GZ_INVERT               1   /* IMU soldada de cabeca para baixo: esquerda = + */
#define IMU_GYRO_LSB_PER_DPS        65.5f
#define IMU_CAL_MAX_SPREAD          120
#define IMU_ZUPT_ALPHA              0.03f
#define IMU_ZUPT_GATE               40.0f
#define IMU_ZUPT_MAX_DRIFT          15.0f

/* --- LEDs e IHM ----------------------------------------------------------- */
#define BOARD_HAS_LED               1
#define BOARD_LED_PORT              GPIOA
#define BOARD_LED_PIN               GPIO_PIN_1      /* LED_1 */
#define BOARD_LED_ACTIVE_LOW        0

#define BOARD_LED2_PORT             GPIOA
#define BOARD_LED2_PIN              GPIO_PIN_0      /* LED_2 */
#define BOARD_LED2_ACTIVE_LOW       0

#define BOARD_HAS_BUTTON            1
#define BOARD_BTN_PORT              GPIOA
#define BOARD_BTN_PIN               GPIO_PIN_7      /* SW2 */
#define BOARD_BTN_ACTIVE_LOW        1

/* DIP switch SW1 (esquematico: cada chave liga o pino ao GND por R1/R2; o
 * CubeMX ja' configura PC14/PC15 como entrada com pull-up -> ON = 0).
 * SW1-1 (PC14): ONDE E' O ALVO do solver (lida na largada)
 *   OFF = centro do labirinto (2x2 do meio)
 *   ON  = canto OPOSTO a' largada (na diagonal)
 * Se ficar ao contrario no seu robo, troque BOARD_DIP_ACTIVE_LOW. */
#define BOARD_DIP_GOAL_PORT         SW1_1_GPIO_Port     /* PC14 */
#define BOARD_DIP_GOAL_PIN          SW1_1_Pin
#define BOARD_DIP_ACTIVE_LOW        1

/* --- Telemetria serial (USART3) ------------------------------------------ */
#define BOARD_DEBUG_UART            USART3          /* PB10 TX, PB11 RX */
#define BOARD_DEBUG_BAUD            115200u

/* --- Tempos do Escalonador Cooperativo ------------------------------------ */
#define CONTROL_PERIOD_MS           1u
#define SENSOR_PERIOD_MS            5u
#define NAV_PERIOD_MS               10u
#define TELEMETRY_PERIOD_MS         100u
#define ENCODER_SPEED_WINDOW_MS     50u

#endif /* BOARD_H */
