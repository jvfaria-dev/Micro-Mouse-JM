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
#define BOARD_USE_ENCODERS          0
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

#define ENCODER_L_INVERT            0
#define ENCODER_R_INVERT            0

#define ENCODER_PPR                 7.0f
#define ENCODER_GEAR_RATIO          20.5f
#define ENCODER_QUADRATURE          4.0f

#define ENCODER_TICKS_PER_MOTOR_REV ((int32_t)(ENCODER_PPR * ENCODER_QUADRATURE))
#define ENCODER_TICKS_PER_REV       573

/* --- Geometria do Robo ---------------------------------------------------- */
#define WHEEL_DIAMETER_MM           43.0f
#define WHEEL_BASE_MM               102.0f
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

#ifndef TURN_TICKS_PER_90
#define TURN_TICKS_PER_90           691
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
#define SENSOR_FILTER_ALPHA         250

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

#define SENSOR_WALL_THRESHOLD       400
#define SENSOR_FRONT_STOP           250
#define SENSOR_CENTER_TRIM          0
#define SENSOR_FRONT_CENTER         1000
#define SENSOR_TURN_HUG_PROX        1000

/* --- Controle de Velocidade PID ------------------------------------------- */
#define SPEED_GAIN_RPM_PER_DUTY     0.25f
#define SPEED_FF_STATIC             40.0f

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
#define SPEED_CRUISE_MMS            100.0f
#define SPEED_TURN_MMS              45.0f
#define SPEED_APPROACH_MMS          40.0f

#define WALL_FOLLOW_KP              0.04f
#define WALL_FOLLOW_KD              0.030f
#define WALL_FOLLOW_D_ALPHA         0.30f
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
