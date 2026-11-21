/**
 ******************************************************************************
 * @file    nav.c
 * @brief   Maquina de estados da navegacao no labirinto.
 ******************************************************************************
 */

#include "nav.h"
#include "board.h"
#include "sensors.h"
#include "speed_ctl.h"
#include "debug.h"
#include "ihm.h"

#include <stdbool.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Estados
 * ------------------------------------------------------------------------- */

typedef enum
{
    NAV_WAIT = 0,       /* parado, esperando voce soltar o carrinho     */
    NAV_FOLLOW,         /* andando no corredor, centralizando           */
    NAV_APPROACH,       /* achou parede na frente, avanca ate o centro  */
    NAV_BACKUP,         /* recua um pouco para ter espaco de girar      */
    NAV_TURN,           /* girando 90 graus no proprio eixo             */
    NAV_SETTLE,         /* pausa curta depois do giro, para estabilizar */
    NAV_DONE,           /* saiu do labirinto                            */
    NAV_ABORT           /* parou por seguranca                          */
} nav_state_t;

static const char *const state_name[] =
{
    "ESPERA", "CORREDOR", "APROXIMA", "RECUA", "GIRO", "ASSENTA", "FIM",
    "ABORTADO"
};

/* ---------------------------------------------------------------------------
 * Estado interno
 * ------------------------------------------------------------------------- */

static nav_state_t state       = NAV_WAIT;
static uint32_t    state_ms    = 0u;
static float       mark_mm     = 0.0f;    /* odometria no inicio do estado  */
static float       mark_deg    = 0.0f;
/* Alvo de giro ABSOLUTO desde o reset da odometria, em DIFERENCA DE TICKS
 * entre as rodas -- nao em graus.
 *
 * Em ticks porque grau depende do diametro da roda e da distancia entre elas,
 * dois numeros que aqui sao estimativa, e ainda e' distorcido pela assimetria
 * dos encoders. A diferenca crua nao depende de nenhum dos tres: uma unica
 * constante calibrada (TURN_TICKS_PER_90) cobre tudo.
 *
 * Absoluto (e nao "gire mais 90") porque assim o erro residual de um giro nao
 * se acumula no proximo: se um parar curto, o seguinte ja' nasce sabendo. */
static int32_t     turn_target_ticks = 0;
static int         turn_quarters     = 0;   /* lado escolhido, em 90 graus */
static int         turns_done        = 0;
static float       open_run_mm = 0.0f;    /* quanto anda sem ver parede     */

/* Memoria do derivativo do centralizador. Precisa ser zerada junto com o
 * resto em nav_init, senao o primeiro passo de um percurso novo herda a
 * derivada do anterior e da' um chute de comando na largada. */
static float       wf_err_prev = 0.0f;
static float       wf_deriv    = 0.0f;

/* Mesma coisa para a aproximacao da curva. Memoria separada porque o ERRO ali
 * e' outra grandeza (distancia ate' uma parede so', e nao diferenca entre as
 * duas): misturar as duas derivadas daria um chute de comando na troca de
 * estado, justamente no trecho mais critico do percurso. */
static float       ap_err_prev = 0.0f;
static float       ap_deriv    = 0.0f;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static void enter(nav_state_t s)
{
    state    = s;
    state_ms = 0u;
    mark_mm  = speed_ctl_get_distance_mm();
    mark_deg = speed_ctl_get_heading_deg();

    port_debug_printf("[nav] -> %s   (dist %ld mm, ang %ld graus)\r\n",
                      state_name[s],
                      (long)mark_mm,
                      (long)mark_deg);
}

/**
 * Ticks de giro a somar no alvo para uma curva de `quarters` quartos de volta,
 * ja' com o aparo de NAV_TURN_TRIM_DEG aplicado uma vez, no sentido do giro.
 */
static int32_t turn_ticks_for(int quarters)
{
    int32_t base = quarters * TURN_TICKS_PER_90;
    int32_t trim = (int32_t)((NAV_TURN_TRIM_DEG / 90.0f)
                             * (float)TURN_TICKS_PER_90);

    /* O aparo acompanha o SENTIDO do giro, nao o sinal do proprio aparo:
     * girando para a direita (quarters < 0) tirar graus significa somar. */
    return (quarters >= 0) ? (base + trim) : (base - trim);
}

static float dist_since_mark(void)
{
    return speed_ctl_get_distance_mm() - mark_mm;
}

/**
 * Anda para frente mantendo o carrinho no meio do corredor.
 * O erro de centralizacao vira uma diferenca de velocidade entre as rodas.
 */
static void drive_centered(float speed_mms)
{
    int16_t err = port_sensors_center_error();

    float turn;

    if (port_sensors_wall(SENSOR_LEFT) || port_sensors_wall(SENSOR_RIGHT))
    {
        /* Proporcional + DERIVATIVO.
         *
         * O derivativo nao e' afinacao fina, e' o que torna este laco
         * estavel. O sensor mede posicao lateral e o atuador comanda angulo:
         * sao duas integracoes entre um e outro, e proporcional puro em cima
         * disso oscila para sempre -- foi exatamente o que o registro mostrou,
         * centro -> parede esquerda -> parede direita -> travou, com periodo
         * de 2,4 s.
         *
         * O termo derivativo reage a QUE VELOCIDADE o carrinho esta' cruzando
         * o corredor, e comeca a frear a correcao antes de chegar no centro.
         * E' o que impede a ultrapassagem. */
        const float dt = (float)NAV_PERIOD_MS / 1000.0f;

        float e  = (float)err;
        float de = (e - wf_err_prev) / dt;
        wf_err_prev = e;

        /* Derivada crua de sinal de sensor e' ruido amplificado; filtra. */
        wf_deriv += (de - wf_deriv) * WALL_FOLLOW_D_ALPHA;

        turn = (e * WALL_FOLLOW_KP) + (wf_deriv * WALL_FOLLOW_KD);
    }
    else
    {
        /* Sem parede o erro nao existe: zera a memoria para nao carregar uma
         * derivada velha para dentro do proximo corredor. */
        wf_err_prev = 0.0f;
        wf_deriv    = 0.0f;

        /* Sem parede nenhuma nao existe referencia externa: o carrinho anda
         * em malha aberta e qualquer assimetria vira arco. STRAIGHT_TRIM_MMS
         * e' o unico contrapeso disponivel aqui. */
        turn = STRAIGHT_TRIM_MMS;
    }

    if (turn >  WALL_FOLLOW_MAX_MMS) { turn =  WALL_FOLLOW_MAX_MMS; }
    if (turn < -WALL_FOLLOW_MAX_MMS) { turn = -WALL_FOLLOW_MAX_MMS; }

    /* err positivo = perto demais da direita => precisa ir para a esquerda,
     * ou seja roda direita mais rapida que a esquerda. Em speed_ctl_set_vw
     * o termo de giro soma na direita e subtrai na esquerda, entao o sinal
     * ja' esta' correto. */
    speed_ctl_set_vw(speed_mms, turn);
}

/**
 * Guia a APROXIMACAO da curva.
 *
 * Ate' agora este trecho mandava `speed_ctl_set_vw(v, 0.0f)`: zero correcao.
 * Ou seja, nos ultimos centimetros antes do giro -- justamente onde a posicao
 * decide se o carrinho cabe na curva ou raspa -- ele andava em malha ABERTA.
 * Os sensores enxergavam a parede chegando e ninguem lia. Era esse o "mesmo
 * com o sensor se aproximando, ele bate".
 *
 * Agora a aproximacao e' guiada, e com um alvo diferente do corredor:
 *
 *   - dois lados com parede -> centraliza, como no corredor
 *   - um lado aberto        -> ENCOSTA NO LADO ABERTO, mantendo a parede do
 *                              lado fechado em SENSOR_TURN_HUG_PROX
 *   - nenhuma parede        -> segue reto
 *
 * O segundo caso e' o que resolve a curva. Ver a explicacao da geometria em
 * SENSOR_TURN_HUG_PROX, no board.h.
 */
static void drive_approach(float speed_mms)
{
    bool wl = port_sensors_wall(SENSOR_LEFT);
    bool wr = port_sensors_wall(SENSOR_RIGHT);

    float turn;

    if (wl && wr)
    {
        turn = (float)port_sensors_center_error() * WALL_FOLLOW_KP;
    }
    else if (wl || wr)
    {
        /* Erro positivo = perto demais da parede do lado fechado. */
        int16_t p = port_sensors_get_prox(wr ? SENSOR_RIGHT : SENSOR_LEFT);
        float   e = (float)(p - SENSOR_TURN_HUG_PROX);

        /* Proporcional + derivativo, pelo mesmo motivo do corredor: o sensor
         * mede POSICAO e o comando e' ANGULO. Sem o derivativo o carrinho
         * chega no ponto de giro ainda girando -- e entrar torto na curva
         * custa mais do que o deslocamento lateral ganhou. */
        const float dt = (float)NAV_PERIOD_MS / 1000.0f;
        float de = (e - ap_err_prev) / dt;
        ap_err_prev = e;
        ap_deriv += (de - ap_deriv) * WALL_FOLLOW_D_ALPHA;

        /* turn positivo gira para a ESQUERDA. Com a parede a' direita, fugir
         * dela e' ir para a esquerda; com a parede a' esquerda, e' o oposto. */
        turn = (e * WALL_FOLLOW_KP) + (ap_deriv * WALL_FOLLOW_KD);

        if (wl) { turn = -turn; }
    }
    else
    {
        turn = STRAIGHT_TRIM_MMS;
    }

    if (turn >  WALL_FOLLOW_MAX_MMS) { turn =  WALL_FOLLOW_MAX_MMS; }
    if (turn < -WALL_FOLLOW_MAX_MMS) { turn = -WALL_FOLLOW_MAX_MMS; }

    speed_ctl_set_vw(speed_mms, turn);
}

/** Decide o lado do giro, em QUARTOS de volta: +1 esquerda, -1 direita,
 *  +2 meia-volta. */
static int choose_turn_quarters(void)
{
    bool left_open  = !port_sensors_wall(SENSOR_LEFT);
    bool right_open = !port_sensors_wall(SENSOR_RIGHT);

    if (left_open && !right_open)  { return  1; }
    if (right_open && !left_open)  { return -1; }

    if (left_open && right_open)
    {
#if (NAV_PREFER_RIGHT != 0)
        return -1;
#else
        return  1;
#endif
    }

    /* Beco sem saida: os dois lados e a frente fechados -> meia-volta. */
    port_debug_printf("[nav] beco sem saida, meia-volta\r\n");
    return 2;
}

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

void nav_init(void)
{
    turns_done        = 0;
    open_run_mm       = 0.0f;
    turn_target_ticks = 0;
    turn_quarters     = 0;
    wf_err_prev       = 0.0f;
    wf_deriv          = 0.0f;
    ap_err_prev       = 0.0f;
    ap_deriv          = 0.0f;

    speed_ctl_reset_odometry();
    speed_ctl_enable(true);
    speed_ctl_stop();

    port_ihm_led_blink(1000u);

    port_debug_printf("\r\n==================================================\r\n");
    port_debug_printf(" NAVEGACAO NO LABIRINTO\r\n");
    port_debug_printf("==================================================\r\n");
    port_debug_printf(" celula ............ %ld mm\r\n", (long)MAZE_CELL_MM);
    port_debug_printf(" ticks por celula .. %ld\r\n",    (long)TICKS_PER_CELL);
    port_debug_printf(" giro de 90 graus .. %ld ticks de diferenca\r\n",
                      (long)TURN_TICKS_PER_90);
    port_debug_printf(" mm por tick ....... 0.%03ld\r\n",
                      (long)(MM_PER_TICK * 1000.0f));
    port_debug_printf(" velocidade ........ %ld mm/s\r\n", (long)SPEED_CRUISE_MMS);
    port_debug_printf("\r\n Solte o carrinho. Partida em %lu ms.\r\n",
                      (unsigned long)NAV_START_DELAY_MS);

    enter(NAV_WAIT);
}

void nav_update(void)
{
    state_ms += NAV_PERIOD_MS;

    /* --- travas de seguranca, valem em qualquer estado --- */
    if (NAV_MAX_DISTANCE_MM > 0.0f
        && state != NAV_DONE && state != NAV_ABORT && state != NAV_WAIT)
    {
        if (fabsf(speed_ctl_get_distance_mm()) > NAV_MAX_DISTANCE_MM)
        {
            port_debug_printf("[nav] !! limite de distancia atingido\r\n");
            speed_ctl_stop();
            enter(NAV_ABORT);
            return;
        }
    }

    switch (state)
    {
    /* ------------------------------------------------------------------ */
    case NAV_WAIT:
        speed_ctl_stop();

        /* --- MIRA DE PARTIDA ------------------------------------------------
         * O registro de 117 amostras mostrou o carrinho ja' descentralizado no
         * instante da largada (esquerda 690, direita 1174) e encostado na
         * parede esquerda nos primeiros 100 mm. Com ~20 mm de folga por lado,
         * um erro de partida come quase toda a margem antes do centralizador
         * ter tempo de agir -- e nao adianta ajustar ganho para compensar mao
         * torta.
         *
         * Entao o LED vira mira: aceso fixo = centralizado, pode soltar.
         * Piscando rapido = torto, ajuste antes que a contagem acabe. O
         * criterio e' o MESMO erro que o centralizador usa, entao o que o LED
         * mostra e' exatamente o que o controle vai ver. */
        {
            int16_t e0 = port_sensors_center_error();
            if (e0 < 0) { e0 = (int16_t)-e0; }

            if (e0 <= NAV_START_CENTER_TOL)  { port_ihm_led_blink(0u);
                                               port_ihm_led_set(true); }
            else if (e0 <= (2 * NAV_START_CENTER_TOL)) { port_ihm_led_blink(300u); }
            else                                       { port_ihm_led_blink(100u); }
        }

        if (state_ms >= NAV_START_DELAY_MS)
        {
            speed_ctl_reset_odometry();
            port_ihm_led_blink(200u);
            enter(NAV_FOLLOW);
        }
        break;

    /* ------------------------------------------------------------------ */
    case NAV_FOLLOW:
    {
        /* Parede na frente? Passa a se aproximar devagar. */
        if (port_sensors_wall_ahead())
        {
            port_debug_printf("[nav] parede a frente (prox %d)\r\n",
                              (int)port_sensors_front());
            ap_err_prev = 0.0f;
            ap_deriv    = 0.0f;
            enter(NAV_APPROACH);
            break;
        }

        drive_centered(SPEED_CRUISE_MMS);

        /* Deteccao de saida: andou bastante sem parede nenhuma dos lados. */
        bool any_wall = port_sensors_wall(SENSOR_LEFT)
                     || port_sensors_wall(SENSOR_RIGHT);

        if (any_wall)
        {
            open_run_mm = 0.0f;
        }
        else
        {
            open_run_mm += (SPEED_CRUISE_MMS * (float)NAV_PERIOD_MS / 1000.0f);

            if (open_run_mm > NAV_EXIT_OPEN_MM)
            {
                port_debug_printf("[nav] sem paredes por %ld mm: saida\r\n",
                                  (long)open_run_mm);
                speed_ctl_stop();
                enter(NAV_DONE);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case NAV_APPROACH:
        /* Avanca devagar ate o sensor frontal dizer que o carrinho chegou
         * no centro da celula. Fechar pelo sensor e nao por distancia
         * comandada e' o que garante que o eixo de giro fique sempre no
         * mesmo lugar -- foi exatamente isso que a simulacao mostrou ser
         * necessario: girando fora do centro, o carrinho entrava torto no
         * corredor seguinte e raspava a parede. */
        if (port_sensors_front() >= SENSOR_FRONT_CENTER
            || dist_since_mark() >= NAV_APPROACH_MAX_MM)
        {
            speed_ctl_stop();

            /* O lado do giro tem que ser decidido AGORA, antes de recuar:
             * daqui o carrinho esta' no centro da celula, que e' de onde os
             * sensores laterais enxergam direito qual lado esta' aberto.
             * Recuado, o lateral fica olhando para a parede do corredor de
             * tras e a leitura deixa de significar o que a gente quer. */
            turn_quarters = choose_turn_quarters();

            enter((NAV_TURN_BACKUP_MM > 0.0f) ? NAV_BACKUP : NAV_TURN);

            if (NAV_TURN_BACKUP_MM <= 0.0f)
            {
                turn_target_ticks += turn_ticks_for(turn_quarters);
            }
            break;
        }

        /* Freia PROPORCIONALMENTE ao que falta, em vez de vir a velocidade
         * cheia e cortar de uma vez. No registro do percurso a aproximacao
         * inteira aconteceu em um unico ciclo de navegacao -- de prox 678 para
         * 1437 -- e o carrinho parou 4 mm depois do centro da celula. A curva
         * do sensor e' ingreme demais perto da parede para uma velocidade so'.
         *
         * O piso de 15 mm/s existe porque abaixo disso o atrito estatico ganha
         * e a roda simplesmente para de girar. */
        {
            float falta = (float)SENSOR_FRONT_CENTER
                        - (float)port_sensors_front();
            float v = SPEED_APPROACH_MMS * (falta / 600.0f);

            if (v > SPEED_APPROACH_MMS) { v = SPEED_APPROACH_MMS; }
            if (v < 15.0f)              { v = 15.0f; }

            drive_approach(v);
        }
        break;

    /* ------------------------------------------------------------------ */
    case NAV_BACKUP:
        /* Recuo antes do giro. Nao e' capricho: girando a partir do centro
         * exato da celula, o canto dianteiro do carrinho descreve um arco
         * que passa por cima da parede da frente. Recuando um pouco, esse
         * arco cabe -- e o canto traseiro, que passa a ser o critico, ganha
         * o espaco do corredor de onde ele veio.
         *
         * Com NAV_TURN_BACKUP_MM em zero este estado nem e' visitado. */
        if (-dist_since_mark() >= NAV_TURN_BACKUP_MM)
        {
            speed_ctl_stop();
            turn_target_ticks += turn_ticks_for(turn_quarters);
            enter(NAV_TURN);
            break;
        }

        speed_ctl_set_vw(-SPEED_APPROACH_MMS, 0.0f);
        break;

    /* ------------------------------------------------------------------ */
    case NAV_TURN:
    {
        /* Primeiro PARA de verdade, depois gira. Sem esta pausa o giro
         * comecava com o carrinho ainda deslizando para frente, e o
         * deslizamento entrava como erro no fechamento por ticks. */
        if (state_ms < NAV_TURN_PAUSE_MS)
        {
            speed_ctl_stop();
            break;
        }

        /* Fechado por DIFERENCA DE TICKS, nao por graus: uma constante so'
         * (TURN_TICKS_PER_90), calibrada por tentativa, no lugar de tres
         * grandezas mal medidas multiplicadas entre si. */
        int32_t left = turn_target_ticks - speed_ctl_get_turn_ticks();
        int32_t mag  = (left < 0) ? -left : left;

        if (mag <= NAV_TURN_TOLERANCE_TICKS)
        {
            speed_ctl_stop();
            turns_done++;
            port_debug_printf("[nav] giro %d concluido (%ld ticks, alvo %ld)\r\n",
                              turns_done,
                              (long)speed_ctl_get_turn_ticks(),
                              (long)turn_target_ticks);
            enter(NAV_SETTLE);
            break;
        }

        /* Giro no proprio eixo: rodas em sentidos opostos.
         * Desacelera perto do alvo para nao passar do ponto. */
        float speed = SPEED_TURN_MMS;
        const int32_t slow_zone = (TURN_TICKS_PER_90 / 4);

        if (mag < slow_zone)
        {
            speed *= ((float)mag / (float)slow_zone);

            /* Piso baixado de 40 para 30 junto com SPEED_TURN_MMS: com o
             * giro a 45 mm/s, um piso de 40 praticamente eliminava a
             * desaceleracao no fim do giro. 30 mm/s ainda vence o atrito
             * (o feedforward sozinho ja' pede ~55 de duty, e a zona morta
             * medida no PWM_TEST e' 75 com o integrador cobrindo o resto). */
            if (speed < 30.0f) { speed = 30.0f; }
        }

        /* left > 0 -> falta girar para a esquerda (anti-horario) ->
         * roda direita para frente, esquerda para tras */
        speed_ctl_set_vw(0.0f, (left > 0) ? speed : -speed);
        break;
    }

    /* ------------------------------------------------------------------ */
    case NAV_SETTLE:
        speed_ctl_stop();

        if (state_ms >= 300u)
        {
            open_run_mm = 0.0f;
            enter(NAV_FOLLOW);
        }
        break;

    /* ------------------------------------------------------------------ */
    case NAV_DONE:
        speed_ctl_stop();
        port_ihm_led_set(true);
        break;

    /* ------------------------------------------------------------------ */
    case NAV_ABORT:
    default:
        speed_ctl_stop();
        port_ihm_led_blink(100u);
        break;
    }
}

const char *nav_state_name(void)
{
    return state_name[state];
}

int nav_state_index(void)
{
    return (int)state;
}

bool nav_finished(void)
{
    return (state == NAV_DONE) || (state == NAV_ABORT);
}
