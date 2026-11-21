/**
 ******************************************************************************
 * @file    board.h
 * @brief   Mapa de hardware do micromouse TUR -- Black Pill STM32F411CEU6,
 *          chassi v2 (impresso), sem RTOS.
 *
 * Este e' o UNICO arquivo que precisa ser mexido quando um fio muda de lugar,
 * quando um motor gira ao contrario ou quando o encoder tem outra resolucao.
 * Nenhum outro arquivo de port/ ou app/ deve conter numero de pino "cru".
 *
 * Historico: o firmware nasceu numa Black Pill STM32U585 com FreeRTOS. A
 * placa queimou e ele foi portado para a F411. Os valores MEDIDOS na U5
 * (encoder, motores) continuam valendo porque sao do motor, nao do micro.
 * Os que dependem do micro (ADC de 14 -> 12 bits, clock dos timers) ou do
 * chassi (roda, bitola, sensores) foram refeitos e estao marcados.
 ******************************************************************************
 */
#pragma once

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 * 1. MAPA DE PINOS  (confere com fw_micromouse_f411.ioc)
 * ==========================================================================
 *
 *  CONFERIDO NO MOTOR_TEST da F411 (chassi v2). Em relacao a' U5 os LADOS
 *  INTEIROS trocaram -- motor e encoder juntos, por isso o teste nao acusou
 *  CRUZADO: cada motor contava no seu encoder, so' que com o nome do outro.
 *
 *  Ponte H (TIM4, 4 canais de PWM):
 *      PB6  -> TIM4_CH1  -> BIN1   (motor DIREITO)
 *      PB7  -> TIM4_CH2  -> BIN2   (motor DIREITO)
 *      PB8  -> TIM4_CH3  -> AIN1   (motor ESQUERDO)
 *      PB9  -> TIM4_CH4  -> AIN2   (motor ESQUERDO)
 *
 *  Encoders (quadratura x4):
 *      PA5  -> TIM2_CH1  -> ENC ESQUERDO canal A   (TIM2 = 32 bits)
 *      PB3  -> TIM2_CH2  -> ENC ESQUERDO canal B
 *      PA6  -> TIM3_CH1  -> ENC DIREITO  canal A   (TIM3 = 16 bits)
 *      PA7  -> TIM3_CH2  -> ENC DIREITO  canal B
 *
 *  Sensores IR (ADC1), A0..A3 da DIREITA para a ESQUERDA do robo:
 *      PA0 (A0) -> ADC1_IN0  -> LATERAL DIREITA  (olha a 90 graus)
 *      PA1 (A1) -> ADC1_IN1  -> FRONTAL DIREITA
 *      PA2 (A2) -> ADC1_IN2  -> FRONTAL ESQUERDA
 *      PA3 (A3) -> ADC1_IN3  -> LATERAL ESQUERDA (olha a 90 graus)
 *  Na F411 o canal e' o numero do pino (PA0=IN0 ... PA3=IN3). Na U5 era
 *  deslocado de 5 -- cuidado ao comparar com o firmware antigo.
 *
 *  IMU MPU-6050 (GY-521) no I2C3 -- o unico bloco que mudou de pino:
 *      PA8  -> I2C3_SCL
 *      PB4  -> I2C3_SDA
 *      AD0 do modulo no GND -> endereco 0x68.
 *
 *  Debug / IHM
 *      PA11/PA12 -> USB OTG FS (USB-C da Black Pill) -> porta COM virtual
 *      PA9  -> USART1_TX (115200 8N1)  -- reserva
 *      PA10 -> USART1_RX
 *      PC13 -> LED da Black Pill (ativo em nivel BAIXO). Nao esta' no .ioc:
 *              o port_ihm_init() configura o pino sozinho.
 *
 *  ATENCAO: PB3 e' SWO e PB4 e' NJTRST. Usar os dois como funcao alternada
 *  nao atrapalha a gravacao/depuracao por SWD (PA13/PA14).
 */

/* --- Ponte H -------------------------------------------------------------- */
#define BOARD_MOTOR_TIM             htim4

/* Medido no MOTOR_TEST:
 *   PB6/PB7 move a roda DIREITA, ja' no sentido certo   -> mantido
 *   PB8/PB9 move a roda ESQUERDA, ao contrario          -> IN1/IN2 trocados */
#define BOARD_MOTOR_L_IN1_CH        TIM_CHANNEL_4   /* PB9 = AIN2 */
#define BOARD_MOTOR_L_IN2_CH        TIM_CHANNEL_3   /* PB8 = AIN1 */
#define BOARD_MOTOR_R_IN1_CH        TIM_CHANNEL_1   /* PB6 = BIN1 */
#define BOARD_MOTOR_R_IN2_CH        TIM_CHANNEL_2   /* PB7 = BIN2 */

/* --- Encoders --------------------------------------------------------------
 * O _IS_32BIT tem que acompanhar o handle: TIM2 e' de 32 bits, TIM3 de 16.
 * Se um dia trocar os lados, troque os dois juntos. */
#define BOARD_ENC_L_TIM             htim2           /* 32 bits, PA5/PB3 */
#define BOARD_ENC_R_TIM             htim3           /* 16 bits, PA6/PA7 */

#define BOARD_ENC_L_IS_32BIT        1
#define BOARD_ENC_R_IS_32BIT        0

/* Filtro digital das entradas do encoder (campo ICxF do timer, 0..15).
 * 6 = amostra a fCK/8 e exige 6 iguais seguidas. A 16 MHz isso rejeita
 * pulso menor que ~3 us -- ruido dos motores -- e passa folgado os ~30 kHz
 * de borda do encoder na rotacao maxima. */
#define BOARD_ENC_INPUT_FILTER      6u

/* Os mesmos pinos lidos como GPIO, para o diagnostico do ENCODER_CAL. */
#define BOARD_ENC_L_A_PORT          GPIOA
#define BOARD_ENC_L_A_PIN           GPIO_PIN_5
#define BOARD_ENC_L_B_PORT          GPIOB
#define BOARD_ENC_L_B_PIN           GPIO_PIN_3
#define BOARD_ENC_R_A_PORT          GPIOA
#define BOARD_ENC_R_A_PIN           GPIO_PIN_6
#define BOARD_ENC_R_B_PORT          GPIOA
#define BOARD_ENC_R_B_PIN           GPIO_PIN_7

/* Impresso no cabecalho do boot: prova no log QUAL mapa foi gravado. */
#define BOARD_MAP_DESC  "ESQ: PB9/PB8 + TIM2   DIR: PB6/PB7 + TIM3  (v3)"

/* --- Debug ----------------------------------------------------------------
 * A saida vai pela USB-C (porta COM virtual) sempre que o projeto tiver a
 * USB_DEVICE/CDC gerada pelo CubeMX -- a deteccao e' automatica. A USART1
 * continua configurada como reserva. Ponha 1 abaixo para forcar a UART. */
#define BOARD_DEBUG_UART            huart1
#define BOARD_DEBUG_FORCE_UART      0

/* --- Sensores IR ---------------------------------------------------------- */
#define BOARD_SENSOR_ADC            hadc1

#define BOARD_SENSOR_CH_LEFT        ADC_CHANNEL_3   /* PA3 (A3) */
#define BOARD_SENSOR_CH_FRONT_L     ADC_CHANNEL_2   /* PA2 (A2) */
#define BOARD_SENSOR_CH_FRONT_R     ADC_CHANNEL_1   /* PA1 (A1) */
#define BOARD_SENSOR_CH_RIGHT       ADC_CHANNEL_0   /* PA0 (A0) */

/* Tempo de amostragem. A saida A0 do modulo TCRT5000 tem impedancia alta
 * (resistor de ~10k no fototransistor), e o capacitor de amostragem do ADC
 * precisa de tempo para carregar -- curto demais e a leitura "puxa" para o
 * canal anterior. O .ioc veio com 3 ciclos, que e' curto demais: o driver
 * sobrescreve com este valor a cada leitura.
 * 480 ciclos a 4 MHz de ADC = 120 us por canal, ~0,5 ms pelos quatro. */
#define BOARD_SENSOR_SAMPLETIME     ADC_SAMPLETIME_480CYCLES

/* --- IMU ------------------------------------------------------------------ */
#define BOARD_IMU_I2C               hi2c3
#define BOARD_IMU_ADDR              0x68u           /* AD0 no GND */

/* --- LED / botao ---------------------------------------------------------- */
#define BOARD_HAS_LED               1
#define BOARD_LED_PORT              GPIOC
#define BOARD_LED_PIN               GPIO_PIN_13
#define BOARD_LED_ACTIVE_LOW        1

/* Sem botao por enquanto. Para ligar um, use um pino livre (PB12, PB13...),
 * ponha 1 aqui e ligue o botao entre o pino e GND (pull-up interno). */
#define BOARD_HAS_BUTTON            0
#define BOARD_BTN_PORT              GPIOB
#define BOARD_BTN_PIN               GPIO_PIN_12
#define BOARD_BTN_ACTIVE_LOW        1

/* ==========================================================================
 * 2. PWM
 * ==========================================================================
 * A frequencia e' fixa; o ARR e' calculado em tempo de execucao a partir do
 * clock real do TIM4 (port_motor_init). Assim o firmware funciona igual com
 * o clock do jeito que o CubeMX gerou (HSI 16 MHz) ou se voce subir para
 * 100 MHz depois -- nada aqui precisa mudar.
 *   16 MHz  -> ARR =  799  (800 degraus de duty)
 *   100 MHz -> ARR = 4999
 * O duty continua em "por mil" (0..1000).
 */
#define BOARD_PWM_FREQ_HZ           20000u

/* Duty maximo em modulo (0..1000) -- o que chega DE FATO no motor.
 *
 * Motor de 6 V numa LiPo 2S: a bateria cheia da' 8,4 V, entao 100% seria
 * 40% acima do nominal (esquenta e gasta as escovas). O teto e' calculado
 * pela bateria CHEIA, que e' o pior caso:
 *      6,0 V / 8,4 V = 714
 * Com a bateria descarregando (7,4 V nominal, ~6,6 V vazia) o motor recebe
 * menos que 6 V no teto -- o PID compensa subindo o duty, mas nunca passa
 * daqui.
 *
 * O limite vale para TUDO: PID, testes e modo bancada. O motor.c aplica o
 * teto depois da compensacao de zona morta, entao nem ela passa daqui. */
#define MOTOR_RATED_MV              6000
#define BATTERY_FULL_MV             8400
#define MOTOR_DUTY_MAX              ((MOTOR_RATED_MV * 1000) / BATTERY_FULL_MV)   /* 714 */

/* Zona morta (offset do atuador), por-mil. MEDIDO na U5 com slow decay:
 * a reta duty x RPM cruza o zero em 25 (esq) e 29 (dir). E' propriedade do
 * motor + ponte, entao continua valendo. Refaca com PWM_TEST se mudar a
 * bateria e quiser precisao. */
#define MOTOR_L_DEADBAND            25
#define MOTOR_R_DEADBAND            29

/* Inversao de sentido por software: troque 0 <-> 1 se a roda girar ao
 * contrario do esperado, em vez de trocar os fios do motor.
 *
 * ATENCAO: inverter o MOTOR nao inverte o ENCODER. Sao dois ajustes
 * independentes (secao 3). Se os dois discordarem, o PID vira
 * realimentacao positiva -- e o sintoma nao e' "anda torto", e' "dispara".
 *
 * Valores herdados da U5 (mesma fiacao). CONFIRA com o MOTOR_TEST: o chassi
 * novo pode ter montado os motores virados. */
/* Seguem o CANAL: PB6/PB7 andava certo com 1 (agora roda direita);
 * PB8/PB9 tinha 0 e o sentido foi corrigido trocando IN1/IN2 acima. */
#define MOTOR_L_INVERT              0
#define MOTOR_R_INVERT              1

/* 1 = slow decay (IN1 = 100%, IN2 = 100% - duty). Medido: em fast decay a
 * roda so' saia do lugar em duty 550. Mantenha 1. */
#define MOTOR_SLOW_DECAY            1

/* ==========================================================================
 * 3. ENCODERS  (MEDIDOS -- sao do motor, valem na F411)
 * ==========================================================================
 *   - 10 voltas no ima do encoder = 280 ticks -> 28 ticks/volta do motor
 *     = 7 polos x 4 bordas.
 *   - duty 1000 -> 7152 ticks/s = 15326 RPM no motor; com 750 RPM nominais
 *     na saida, reducao ~20,8 (estimativa).
 *   - MEDIDO (ENCODER_CAL, 10 voltas na roda com a mao): ESQ 5712..5768,
 *     DIR 5749 -> ~573 ticks/volta da roda -> reducao 573/28 = 20,5.
 * O .ioc gerou os timers em modo TI1 (x2). O port_encoder_init() troca
 * para TI12 (x4), que e' o que estes numeros pressupoem.
 */
#define ENCODER_PPR                 7.0f
#define ENCODER_GEAR_RATIO          20.5f
#define ENCODER_QUADRATURE          4.0f

#define ENCODER_TICKS_PER_MOTOR_REV \
    ((int32_t)(ENCODER_PPR * ENCODER_QUADRATURE))

#define ENCODER_TICKS_PER_REV \
    ((int32_t)(ENCODER_PPR * ENCODER_QUADRATURE * ENCODER_GEAR_RATIO))

/* Inversao da contagem. Valores da U5; CONFIRA com o MOTOR_TEST:
 * roda girando PARA FRENTE tem que dar ticks POSITIVOS. */
/* Do log do MOTOR_TEST, com o mapa antigo:
 *   TIM3 (roda direita) contou NEGATIVO com a roda indo para frente, com
 *        invert 1                                    -> agora 0
 *   TIM2 (roda esquerda) contou POSITIVO com a roda indo para TRAS (o motor
 *        dela estava invertido), com invert 0        -> agora 1 */
#define ENCODER_L_INVERT            1
#define ENCODER_R_INVERT            0

/* ==========================================================================
 * 3b. GEOMETRIA -- CHASSI V2  (do CAD, hardware/chassi_v2)
 * ==========================================================================
 *   roda 43 mm; face interna em y = 41,5 e externa em 60,5
 *   -> centro do pneu a 51 mm do eixo -> bitola 102 mm.
 * Medida de CAD, nao de chao. O ODOM_CAL acerta os dois se precisar.
 */
#define WHEEL_DIAMETER_MM           43.0f
#define WHEEL_BASE_MM               102.0f

#define WHEEL_CIRC_MM               (3.14159265f * WHEEL_DIAMETER_MM)

/* 0,235 mm por tick (135 mm de roda / 574 ticks por volta). */
#define MM_PER_TICK                 (WHEEL_CIRC_MM / (float)ENCODER_TICKS_PER_REV)

/* Correcao de assimetria entre rodas (ver ODOM_CAL). Zerada para o chassi
 * novo: a correcao de 1.020 da U5 era do pneu antigo. */
#define WHEEL_L_SCALE               1.000f
#define WHEEL_R_SCALE               1.000f

/* ==========================================================================
 * 3c. LABIRINTO
 * ========================================================================== */
#define MAZE_CELL_MM                180.0f
#define MAZE_WALL_THICK_MM          15.0f
#define MAZE_CORRIDOR_MM            (MAZE_CELL_MM - MAZE_WALL_THICK_MM)

#define TICKS_PER_CELL              ((int32_t)(MAZE_CELL_MM / MM_PER_TICK))

/* Referencia teorica (nao usada pela navegacao). */
#define TICKS_PER_90DEG \
    ((int32_t)((3.14159265f * WHEEL_BASE_MM / 4.0f) / MM_PER_TICK))

/* --------------------------------------------------------------------------
 * TURN_TICKS_PER_90 -- a UNICA constante que o giro precisa
 * --------------------------------------------------------------------------
 * O giro fecha quando (ticks_direita - ticks_esquerda) atinge este numero.
 *
 * Valor TEORICO para o chassi v2: cada roda anda pi*102/4 = 80,1 mm,
 * a diferenca e' o dobro, 160,2 mm / 0,232 mm/tick = 691.
 *
 * CALIBRE com APP_MODE_TURN_CAL (quatro giros = uma volta):
 *     novo = atual * 360 / graus_girados_de_verdade
 */
#ifndef TURN_TICKS_PER_90
#define TURN_TICKS_PER_90           691
#endif

/* Correcao fina do rumo em reta sem parede. Positivo puxa para a ESQUERDA. */
#define STRAIGHT_TRIM_MMS           0.0f

/* ==========================================================================
 * 3d. SENSORES IR
 * ==========================================================================
 * Cada sensor e' calibrado por DOIS pontos, medidos com APP_MODE_SENSOR_CAL:
 *   RAW_NEAR = robo parado no CENTRO de uma celula com parede dos dois lados
 *              e na frente. Por construcao, ai' a proximidade vale 1000.
 *   RAW_FAR  = robo em area ABERTA, nada ao alcance.
 * Se NEAR sair menor que FAR (estes modulos dao tensao ALTA vendo o vazio),
 * ponha assim mesmo -- a interpolacao lida com o sinal.
 *
 * MEDIDOS no chassi v2 (SENSOR_CAL, F411, coluna "filt"):
 *   FAR  = robo em area aberta, nada a menos de ~30 cm
 *   NEAR = robo parado no centro de uma celula sem saida (parede dos dois
 *          lados e na frente) -- a posicao de referencia, prox = 1000
 *
 *            FAR    NEAR
 *   ESQ     4032    1116      (PA3 -- o pino que travava na U5 funciona)
 *   FR-E    3927    1311
 *   FR-D    3975     518      <- bem mais forte que o FR-E na mesma parede:
 *   DIR     3929     650         descasamento de modulo e/ou robo um pouco
 *                                torto. A calibracao por sensor cancela.
 *
 * Como o centro foi "mais ou menos no meio", o ponto de 1000 pode estar uns
 * milimetros fora. Se o robo parar para girar visivelmente antes ou depois
 * do centro da celula, refaca so' o NEAR com mais cuidado.
 */
#define SENSOR_RAW_MAX              4095    /* ADC de 12 bits na F411 */

#define SENSOR_L_RAW_FAR            4032
#define SENSOR_L_RAW_NEAR           1116
#define SENSOR_FL_RAW_FAR           3927
#define SENSOR_FL_RAW_NEAR          1311
#define SENSOR_FR_RAW_FAR           3975
#define SENSOR_FR_RAW_NEAR           518
#define SENSOR_R_RAW_FAR            3929
#define SENSOR_R_RAW_NEAR            650

#define SENSOR_FILTER_ALPHA         250

/* Acima desta proximidade ha' parede daquele lado. */
#define SENSOR_WALL_THRESHOLD       400     /* era 350: com luz o lado aberto chegou a 363 */

/* Proximidade frontal que dispara "parede na frente, comece a frear".
 * Acima do ruido dos frontais no corredor (~54 na U5) com folga. */
#define SENSOR_FRONT_STOP           180

/* Descasamento esquerda/direita, medido no SENSOR_CAL ("erro centro="). */
#define SENSOR_CENTER_TRIM          0

/* Proximidade frontal em que o robo para para girar.
 * 1000 = centro da celula, por construcao da calibracao. Com o chassi v2 o
 * giro tem 13,5 mm de folga em qualquer ponto da celula, entao parar no
 * centro e' o certo -- os 850 da U5 eram aperto do chassi antigo. */
#define SENSOR_FRONT_CENTER         1000

/* Alvo na parede do lado FECHADO durante a aproximacao da curva.
 * Na U5 isto era 600 para o robo encostar no lado aberto antes de girar,
 * porque o canto do chassi antigo varria 83,3 mm e o corredor tem 82,5.
 * O chassi v2 varre 69 mm: nao precisa mais. 1000 = centralizado. */
#define SENSOR_TURN_HUG_PROX        1000

/* ==========================================================================
 * 3e. CONTROLE DE VELOCIDADE (PID por roda)
 * ==========================================================================
 * duty_ff = rpm_alvo / SPEED_GAIN_RPM_PER_DUTY
 *
 * 0,652 foi MEDIDO no PWM_TEST na U5. O ganho escala com a tensao da
 * bateria; com a LiPo 2S ele deve sair maior. O integrador cobre a
 * diferenca, mas rode o PWM_TEST de novo quando der e ponha o numero novo.
 */
/* 1o TESTE NO LABIRINTO (chassi v2, LiPo 2S): reto a 100 mm/s (44 rpm) o
 * duty assentou em ~189 -- o 0,652 da U5 previa 68. Com isso, no giro a
 * 45 mm/s a malha so' entregava ~80 de duty depois de meio segundo
 * (feedforward 31 + integrador lento), pouco para tirar o robo do lugar
 * girando (os pes arrastam de lado): ele parava na frente da parede e nao
 * virava, e nem conseguia recuar a 30 mm/s.
 *   - ganho medido no chassi: 44 rpm / 189 = 0,235 -> 0,25
 *   - SPEED_FF_STATIC: esforco extra para vencer o atrito parado
 *   - PID_KI 4 -> 12: roda presa ganha esforco 3x mais rapido */
#define SPEED_GAIN_RPM_PER_DUTY     0.25f
#define SPEED_FF_STATIC             40.0f

#define PID_KP                      0.60f
#define PID_KI                      12.00f
#define PID_KD                      0.00f
/* Teto de esforco do integrador (duty). Com a roda de 43 mm o mesmo mm/s
 * pede ~metade da rotacao da U5, entao sobra margem. */
#define PID_I_LIMIT                 500.0f

/* ==========================================================================
 * 3f. VELOCIDADES DO PERCURSO
 * ==========================================================================
 * O robo nao compete: e' para testar o labirinto. Devagar e funcional.
 */
#define SPEED_CRUISE_MMS            100.0f
#define SPEED_TURN_MMS               45.0f
#define SPEED_APPROACH_MMS           40.0f

/* Centralizador P+D entre paredes. O D e' obrigatorio: sensor mede posicao,
 * atuador comanda angulo -- sem amortecimento o laco oscila.
 *   serpenteia -> suba KD de 0,01 em 0,01
 *   lerdo      -> suba KP de 0,01 em 0,01
 *   tremendo   -> desca KD */
#define WALL_FOLLOW_KP              0.04f
#define WALL_FOLLOW_KD              0.030f
#define WALL_FOLLOW_D_ALPHA         0.30f
#define WALL_FOLLOW_MAX_MMS         60.0f

/* ==========================================================================
 * 4. TEMPOS DO SISTEMA  (laco principal, sem RTOS -- ver app_main.c)
 * ========================================================================== */
#define CONTROL_PERIOD_MS           1u      /* controle: 1 kHz              */
#define SENSOR_PERIOD_MS            5u      /* sensores: 200 Hz             */
#define NAV_PERIOD_MS               10u     /* navegacao: 100 Hz            */
#define TELEMETRY_PERIOD_MS         100u    /* telemetria: 10 Hz            */
#define ENCODER_SPEED_WINDOW_MS     50u     /* janela p/ calcular velocidade */
