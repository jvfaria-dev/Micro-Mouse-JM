/**
 ******************************************************************************
 * @file    app_main.c
 * @brief   Laco principal sem RTOS e despacho dos modos.
 ******************************************************************************
 */

#include "app_main.h"

/* Ate' quantos milissegundos atrasados o laco recupera um a um. Passou
 * disso (alguma espera longa de proposito), realinha o relogio. */
#define APP_MAX_CATCHUP_MS      20u

/* Se o board.h no disco for uma versao antiga (sem esta linha), compila do
 * mesmo jeito e o cabecalho do boot denuncia. */
#ifndef BOARD_MAP_DESC
#define BOARD_MAP_DESC  "?? board.h ANTIGO -- a IDE gravou por cima, feche a aba e recompile"
#endif

static uint32_t last_ms = 0u;

/* ===========================================================================
 * "Tarefas" -- as mesmas do firmware FreeRTOS, agora chamadas pelo laco
 * ======================================================================== */

/* --- controle, 1 kHz ------------------------------------------------------ */
static void task_control(void)
{
    port_ihm_update();
    port_encoder_update();

#if   (APP_MODE == APP_MODE_MOTOR_TEST)
    motor_test_update();
#elif (APP_MODE == APP_MODE_PWM_TEST)
    pwm_test_update();
#elif (APP_MODE == APP_MODE_ENCODER_CAL)
    encoder_cal_update();
#elif (APP_MODE == APP_MODE_IMU_TEST)
    /* motores parados */
#else
    speed_ctl_update();
#endif
}

/* --- sensores, 200 Hz ----------------------------------------------------- */
static void task_sensors(void)
{
#if APP_USES_NAV || (APP_MODE == APP_MODE_SOLVE)
    port_sensors_update();
#if (APP_MODE == APP_MODE_SOLVE)
    port_imu_update(SENSOR_PERIOD_MS);      /* giroscopio, 200 Hz */
#endif
#elif (APP_MODE == APP_MODE_IMU_TEST)
    imu_test_update();
#endif
}

/* --- navegacao, 100 Hz ---------------------------------------------------- */
static void task_nav(void)
{
#if   (APP_MODE == APP_MODE_ODOM_CAL)
    odom_cal_update();
#elif (APP_MODE == APP_MODE_SENSOR_CAL)
    sensor_cal_update();
#elif (APP_MODE == APP_MODE_TURN_CAL)
    turn_cal_update();
#elif (APP_MODE == APP_MODE_MAZE)
    nav_update();
#elif (APP_MODE == APP_MODE_SOLVE)
    solver_update();
#endif
}

/* --- telemetria, 10 Hz ---------------------------------------------------- */
#if (APP_MODE == APP_MODE_MAZE)
/*
 * No labirinto NAO imprime enquanto o robo anda: as amostras vao para a RAM
 * (trace) e saem depois. O despejo acontece:
 *   - 3 s depois que o robo parou, e de novo a cada 15 s;
 *   - na hora, se chegar qualquer tecla pela serial.
 * Quem decide se parou e' o ENCODER, nao a navegacao: encostado numa parede
 * a navegacao nao "termina", mas o encoder para de contar.
 * ATENCAO: o registro vive em RAM. RESET apaga tudo.
 */
static uint32_t tr_idle_ms  = 0u;
static uint32_t tr_still_ms = 0u;
static int32_t  tr_last_fwd = 0;
static bool     tr_dumped   = false;

static void task_telemetry(void)
{
    int32_t fwd  = speed_ctl_get_fwd_ticks();
    int32_t move = fwd - tr_last_fwd;
    tr_last_fwd = fwd;

    /* Em ESPERA o robo esta' parado de proposito: nao conta como parado. */
    if (nav_state_index() == 0)
    {
        tr_still_ms = 0u;
    }
    else if ((move > TRACE_STILL_TICKS) || (move < -TRACE_STILL_TICKS))
    {
        tr_still_ms = 0u;
    }
    else
    {
        tr_still_ms += TELEMETRY_PERIOD_MS;
    }

    bool parado = nav_finished() || (tr_still_ms >= TRACE_STILL_MS);

    if (!parado)
    {
        trace_sample();
        tr_idle_ms = 0u;
        tr_dumped  = false;
    }
    else
    {
        tr_idle_ms += TELEMETRY_PERIOD_MS;

        if (!tr_dumped && tr_idle_ms >= 3000u)
        {
            trace_dump();
            tr_dumped  = true;
            tr_idle_ms = 0u;
        }
        else if (tr_dumped && tr_idle_ms >= 15000u)
        {
            trace_dump();
            tr_idle_ms = 0u;
        }
    }

    if (port_debug_key())
    {
        trace_dump();
        tr_idle_ms = 0u;
        tr_dumped  = true;
    }
}
#elif (APP_MODE == APP_MODE_ODOM_CAL)
static void task_telemetry(void)
{
    port_debug_printf("[%-10s] d=%5ld mm  ang=%5ld  alvo E=%+4ld D=%+4ld  "
                      "medido E=%+4ld D=%+4ld  duty E=%+5ld D=%+5ld\r\n",
                      odom_cal_state_name(),
                      (long)speed_ctl_get_distance_mm(),
                      (long)speed_ctl_get_heading_deg(),
                      (long)speed_ctl_get_target_mms(0),
                      (long)speed_ctl_get_target_mms(1),
                      (long)speed_ctl_get_mms(0),
                      (long)speed_ctl_get_mms(1),
                      (long)speed_ctl_get_duty(0),
                      (long)speed_ctl_get_duty(1));
}
#elif (APP_MODE == APP_MODE_IMU_TEST)
static void task_telemetry(void)
{
    imu_test_print();
}
#elif (APP_MODE == APP_MODE_SOLVE)
/* O registro e o mapa saem UMA vez, 3 s depois do fim. Depois so' de novo
 * quando o terminal (re)abre a porta -- nunca sozinho. RESET apaga. */
static uint32_t sv_idle_ms = 0u;
static bool     sv_dumped  = false;

static void task_telemetry(void)
{
    /* Reimprime so' quando o terminal (re)abre a porta USB -- tecla nao conta
     * mais: alguma coisa (RX da UART solto ou o proprio monitor) gerava
     * "tecla" continua e o registro saia dezenas de vezes por segundo
     * (11o teste). E no maximo 1 a cada 3 s. */
    bool key = port_debug_port_opened();
    if (sv_dumped && sv_idle_ms < 3000u)
    {
        sv_idle_ms += TELEMETRY_PERIOD_MS;
        key = false;
    }

    if (!solver_finished())
    {
        return;
    }
    if (key)
    {
        solver_dump();
        sv_dumped  = true;
        sv_idle_ms = 0u;
        return;
    }
    if (!sv_dumped)
    {
        sv_idle_ms += TELEMETRY_PERIOD_MS;
        if (sv_idle_ms >= 3000u)
        {
            solver_dump();
            sv_dumped = true;
        }
    }
}
#else
static void task_telemetry(void) { }
#endif

/* ===========================================================================
 * Um milissegundo
 * ======================================================================== */

static void app_tick(uint32_t ms)
{
    task_control();

    if ((ms % SENSOR_PERIOD_MS) == 1u)
    {
        task_sensors();
    }

    if ((ms % NAV_PERIOD_MS) == 3u)
    {
        task_nav();
    }

    if ((ms % TELEMETRY_PERIOD_MS) == 7u)
    {
        task_telemetry();
    }

    port_debug_poll();
}

/* ===========================================================================
 * API
 * ======================================================================== */

static const char *mode_name(void)
{
    switch (APP_MODE)
    {
        case APP_MODE_MOTOR_TEST:  return "MOTOR_TEST";
        case APP_MODE_PWM_TEST:    return "PWM_TEST";
        case APP_MODE_ENCODER_CAL: return "ENCODER_CAL";
        case APP_MODE_ODOM_CAL:    return "ODOM_CAL";
        case APP_MODE_SENSOR_CAL:  return "SENSOR_CAL";
        case APP_MODE_MAZE:        return "MAZE";
        case APP_MODE_TURN_CAL:    return "TURN_CAL";
        case APP_MODE_IMU_TEST:    return "IMU_TEST";
        case APP_MODE_SOLVE:       return "SOLVE";
        default:                   return "?";
    }
}

void app_init(void)
{
    port_debug_init();
    port_ihm_init();
    port_motor_init();
    port_encoder_init();
    port_sensors_init();
    speed_ctl_init();
    port_motor_stop_all();

    bool imu = port_imu_init();

    port_debug_printf("\r\n==============================================\r\n");
    port_debug_printf(" MICROMOUSE TUR - STM32F411, sem RTOS\r\n");
    port_debug_printf("==============================================\r\n");
    port_debug_printf(" SYSCLK ........ %lu Hz\r\n",
                      (unsigned long)HAL_RCC_GetSysClockFreq());
    port_debug_printf(" PWM ........... %lu Hz (ARR %lu)\r\n",
                      (unsigned long)BOARD_PWM_FREQ_HZ,
                      (unsigned long)port_motor_get_pwm_arr());
    port_debug_printf(" ticks/volta ... %ld\r\n", (long)ENCODER_TICKS_PER_REV);
    port_debug_printf(" serial ........ %s\r\n", port_debug_output_name());
    port_debug_printf(" rodas ......... %s\r\n", BOARD_MAP_DESC);
    port_debug_printf(" IMU ........... %s",
                      imu ? "ok, WHO_AM_I=0x" : "NAO respondeu");
    if (imu)
    {
        port_debug_printf("%02X", (unsigned)port_imu_whoami());
    }
    if (port_imu_init_tries() > 1u)
    {
        port_debug_printf("  (I2C estava travado: %u tentativas)",
                          (unsigned)port_imu_init_tries());
    }
    port_debug_printf("\r\n modo .......... %d (%s)\r\n", (int)APP_MODE, mode_name());
    port_debug_printf("----------------------------------------------\r\n");
    port_debug_flush();

    /* init do modo -- na versao RTOS acontecia dentro de cada tarefa */
#if   (APP_MODE == APP_MODE_MOTOR_TEST)
    motor_test_init();
#elif (APP_MODE == APP_MODE_PWM_TEST)
    pwm_test_init();
#elif (APP_MODE == APP_MODE_ENCODER_CAL)
    encoder_cal_init();
#elif (APP_MODE == APP_MODE_ODOM_CAL)
    odom_cal_init();
#elif (APP_MODE == APP_MODE_SENSOR_CAL)
    sensor_cal_init();
#elif (APP_MODE == APP_MODE_TURN_CAL)
    turn_cal_init();
#elif (APP_MODE == APP_MODE_MAZE)
    nav_init();
    trace_init();
#elif (APP_MODE == APP_MODE_IMU_TEST)
    imu_test_init();
#elif (APP_MODE == APP_MODE_SOLVE)
    solver_init();
#endif

    last_ms = HAL_GetTick();
}

void app_loop(void)
{
    uint32_t now    = HAL_GetTick();
    uint32_t behind = now - last_ms;

    if (behind > APP_MAX_CATCHUP_MS)
    {
        /* Atraso grande (espera proposital): realinha em vez de rodar o PID
         * dezenas de vezes seguidas com o mesmo encoder. */
        last_ms = now - 1u;
    }

    while (last_ms != now)
    {
        last_ms++;
        app_tick(last_ms);
    }

    /* Fora do tick tambem: o laco gira muito mais rapido que 1 kHz, e cada
     * volta e' uma chance de mandar mais um byte pela UART. */
    port_debug_poll();
}

void app_emergency_stop(void)
{
    port_motor_stop_all();
}
