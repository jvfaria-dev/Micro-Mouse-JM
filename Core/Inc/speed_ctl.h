/**
 ******************************************************************************
 * @file    speed_ctl.h
 * @brief   Controle de velocidade das duas rodas: PID com realimentacao
 *          direta (feedforward), rodando no laco de 1 kHz.
 *
 * A velocidade e' comandada em MILIMETROS POR SEGUNDO, nao em duty nem em
 * RPM. Isso deixa a navegacao independente do motor: se um dia trocar a
 * roda ou a caixa, so' o board.h muda.
 *
 * Por que feedforward: o ensaio de PWM mediu rpm = 0,773*duty - 19,5 com
 * R2 = 0,99999. Como o atuador e' praticamente linear, da' pra calcular o
 * duty necessario direto da conta e deixar o PID so' corrigindo o que
 * sobra (atrito variavel, bateria caindo, uma roda mais dura que a outra).
 * O resultado e' resposta rapida sem ganho alto -- e ganho baixo e' o que
 * evita oscilacao.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/** @brief Zera integradores e para os motores. */
void speed_ctl_init(void);

/**
 * @brief Define a velocidade alvo de cada roda, em mm/s.
 *        Positivo = para frente. Chamar de qualquer tarefa.
 */
void speed_ctl_set(float left_mms, float right_mms);

/** @brief Comanda velocidade linear e rotacional do carrinho. */
void speed_ctl_set_vw(float linear_mms, float turn_mms);

/** @brief Para tudo e zera os integradores. */
void speed_ctl_stop(void);

/**
 * @brief Liga/desliga o controlador. Desligado, os testes antigos podem
 *        comandar os motores direto sem o PID brigar com eles.
 */
void speed_ctl_enable(bool on);

/** @brief Um passo do controlador. Chamar a cada CONTROL_PERIOD_MS. */
void speed_ctl_update(void);

/* --- leitura de estado, para telemetria e navegacao --------------------- */

float speed_ctl_get_mms(int wheel);        /* velocidade medida, mm/s   */
float speed_ctl_get_target_mms(int wheel); /* alvo, mm/s                */
int32_t speed_ctl_get_duty(int wheel);     /* duty aplicado             */

/** @brief Distancia percorrida pelo centro do carrinho desde o reset, em mm. */
float speed_ctl_get_distance_mm(void);

/** @brief Angulo acumulado desde o reset, em graus (positivo = anti-horario). */
float speed_ctl_get_heading_deg(void);

/**
 * @brief Diferenca CRUA de ticks entre as rodas desde o reset da odometria:
 *        (direita - esquerda). Positivo = girou para a esquerda.
 *
 * E' isto, e nao o angulo em graus, que a navegacao usa para fechar o giro
 * de 90 graus. O angulo em graus passa por WHEEL_DIAMETER_MM e
 * WHEEL_BASE_MM, dois valores que aqui nao estao medidos com confianca, e
 * ainda por cima e' distorcido pela assimetria entre os dois encoders. A
 * diferenca crua nao passa por nenhum deles: basta calibrar UM numero
 * (TURN_TICKS_PER_90) por tentativa e os quatro erros somem juntos.
 */
int32_t speed_ctl_get_turn_ticks(void);

/** @brief Ticks acumulados no sentido de avanco (media das duas rodas). */
int32_t speed_ctl_get_fwd_ticks(void);

/** @brief Zera odometria (distancia, angulo e contagem crua). */
void speed_ctl_reset_odometry(void);

/* ---------------------------------------------------------------------------
 * DETECCAO DE RODA MUDA
 * ---------------------------------------------------------------------------
 * Se uma roda esta' sendo comandada com esforco significativo e o encoder
 * dela nao acusa movimento nenhum por SPEED_FAULT_MS, alguma coisa esta'
 * errada: fio de encoder solto, modulo sem alimentacao, motor travado ou
 * roda presa.
 *
 * Isso PRECISA ser detectado, e nao so' ignorado, porque a malha fechada
 * reage a ausencia de movimento exatamente do jeito errado: o integrador
 * cresce ate saturar e o motor passa a ser empurrado com esforco maximo
 * contra um problema que nao e' de esforco. Se a roda estiver travada de
 * verdade, isso cozinha o motor e a ponte H.
 *
 * Ao detectar, o controlador para tudo e trava ate speed_ctl_clear_fault().
 */
/* SUBIU DE 800 ms PARA 1800 ms.
 *
 * O 800 estava julgando antes do controlador terminar de tentar. O esforco
 * nao aparece de uma vez: feedforward e proporcional entram no primeiro
 * milissegundo (uns 170 de duty a 100 mm/s), mas o integrador sobe a
 * PID_KI*erro por segundo -- uns 318 de duty por segundo -- e so' encosta no
 * PID_I_LIMIT depois de 1,5 s. Aos 800 ms o duty aplicado era ~425 de um teto
 * de 670: a trava disparava com o controlador ainda na metade do caminho.
 *
 * 1800 ms deixa a malha chegar no teto antes de alguem declarar roda travada.
 * Nao ha' risco termico nisso: sao dois segundos, nao dois minutos. */
/* 1800 -> 3000: o giro agora sobe o esforco de proposito ate' desgrudar
 * (SOLVER_TURN_BOOST_*) e desiste sozinho em 2,5 s; este fica de reserva. */
#define SPEED_FAULT_MS          3000u   /* tempo mudo para acusar falha  */
#define SPEED_FAULT_MIN_TARGET  40.0f   /* mm/s: abaixo disso nem checa  */
#define SPEED_FAULT_MIN_DUTY    150     /* duty: abaixo disso nem checa  */
#define SPEED_FAULT_MOVING_MMS  8.0f    /* acima disso considera que moveu */

/* ---------------------------------------------------------------------------
 * DETECCAO DE SENTIDO INVERTIDO
 * ---------------------------------------------------------------------------
 * Caso irmao do anterior, e mais perigoso. Se a roda anda de verdade, mas
 * para o lado OPOSTO ao comandado, e' porque MOTOR_*_INVERT e
 * ENCODER_*_INVERT discordam: a medida chega com o sinal trocado e a malha
 * fechada vira realimentacao POSITIVA. O erro cresce, o duty cresce junto e
 * o carrinho sai em disparada com esforco maximo -- o contrario exato do que
 * um controlador deveria fazer.
 *
 * Nenhum ajuste de PID conserta: o problema esta' no sinal da medida, nao no
 * ganho. Meio segundo separa isso com folga de um transitorio de partida ou
 * de uma inversao de sentido comandada.
 */
#define SPEED_FAULT_WRONG_MS    500u

/** @brief 0 = tudo bem; 1 = roda esquerda muda; 2 = direita; 3 = as duas. */
int speed_ctl_get_fault(void);

/** @brief Limpa a falha e reabilita o controle. */
void speed_ctl_clear_fault(void);
