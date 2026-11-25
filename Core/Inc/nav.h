/**
 ******************************************************************************
 * @file    nav.h
 * @brief   Navegacao do percurso: corredor -> curva 90 -> curva 90 -> saida.
 *
 * A maquina de estados nao sabe qual e' o percurso. Ela segue esta regra
 * geral, que resolve o seu trajeto e qualquer outro parecido:
 *
 *   1. Anda para frente centralizando entre as paredes.
 *   2. Achou parede na frente? Para, olha para os lados e vira para o lado
 *      que estiver aberto (se os dois estiverem, prefere um lado fixo).
 *   3. Nao ha' parede em lugar nenhum e ja' andou o suficiente? Saiu.
 *
 * O giro de 90 graus e' fechado por ENCODER, nao por tempo -- mas pela
 * DIFERENCA CRUA de ticks entre as rodas, nao por angulo calculado. Com isso
 * a navegacao nao depende de WHEEL_DIAMETER_MM nem de WHEEL_BASE_MM, e nem da
 * assimetria entre os dois encoders: tudo isso e' absorvido por uma unica
 * constante, TURN_TICKS_PER_90, calibrada com APP_MODE_TURN_CAL.
 *
 * A distancia percorrida continua em milimetros, mas so' alimenta redes de
 * seguranca (limite de aproximacao, limite de percurso). Quem decide quando
 * parar, virar e sair sao os SENSORES. Por isso um erro no diametro da roda
 * atrasa ou adianta um pouco os limites, mas nao quebra o percurso.
 ******************************************************************************
 */
#pragma once

#include "board.h"

#include <stdint.h>
#include <stdbool.h>

void nav_init(void);

/** @brief Chamar a cada NAV_PERIOD_MS. */
void nav_update(void);

/** @brief Nome do estado atual, para a telemetria. */
const char *nav_state_name(void);

/** @brief Indice do estado atual. Usado pelo registro em RAM, que guarda um
 *         byte por amostra em vez de um ponteiro para texto. */
int nav_state_index(void);

/** @brief true quando o percurso terminou (motores parados). */
bool nav_finished(void);

/* --- ajustes do percurso ------------------------------------------------ */

/* O periodo da tarefa de navegacao (NAV_PERIOD_MS) vive no board.h, junto
 * com os outros tempos do sistema. */

/* Espera antes de sair andando, para voce tirar a mao do carrinho.
 * Subiu de 3 s para 6 s porque agora esse tempo tem uso: o LED indica se o
 * carrinho esta' centralizado, e 3 s nao davam para posicionar e soltar. */
#define NAV_START_DELAY_MS      6000u

/* Erro lateral que ainda conta como "centralizado" na mira de partida.
 * A calibracao faz cada lado valer 1000 no centro da celula, e o registro
 * mostra a escala em ~40 unidades de erro por milimetro de deslocamento --
 * entao 150 sao uns 4 mm de desvio. Dentro disso o LED fica aceso fixo. */
#define NAV_START_CENTER_TOL    150

/* A aproximacao da parede e' fechada pelo SENSOR, nao por distancia
 * comandada: avanca devagar ate a proximidade frontal chegar em
 * SENSOR_FRONT_CENTER, que por construcao da calibracao e' o centro da
 * celula. Assim o eixo de giro fica no lugar certo independentemente de
 * onde a parede foi detectada -- que muda com a velocidade e com a bateria.
 *
 * O limite de distancia abaixo e' so' rede de seguranca: se o sensor nunca
 * chegar no alvo (parede escura, sensor sujo), o carrinho para mesmo assim
 * em vez de seguir empurrando a parede. */
#define NAV_APPROACH_MAX_MM     140.0f

/* ---------------------------------------------------------------------------
 * APARO DO GIRO -- quantos graus TIRAR de cada curva
 * ---------------------------------------------------------------------------
 * O TURN_TICKS_PER_90 fica como esta': ele foi CALIBRADO, e no TURN_CAL quatro
 * giros seguidos deram 355 graus, ou seja 88,75 por giro. Mexer nele seria
 * apagar uma medida boa.
 *
 * Este numero e' outra coisa: uma correcao aplicada UMA VEZ por curva, com o
 * sinal do giro. Serve para o que a calibracao na bancada nao pega -- no
 * labirinto o carrinho encosta, a roda escorrega na quina, e o giro efetivo
 * sai maior do que os ticks dizem.
 *
 * Negativo = gira MENOS. -20 pede 70 graus onde antes pedia 90.
 *
 * Aplicado uma vez por curva (e nao proporcional) de proposito: se a origem e'
 * escorregao no fim do giro, o erro e' um valor fixo por curva, nao uma
 * porcentagem. Por isso a meia-volta de 180 graus tambem leva -20, e nao -40.
 */
/* ZERADO no chassi v2. Os -20 eram do chassi antigo, que raspava na quina e
 * escorregava no fim do giro. Com 13,5 mm de folga isso nao deve acontecer;
 * so' volte a mexer aqui se o TURN_CAL fechar a volta e o labirinto nao. */
#define NAV_TURN_TRIM_DEG       (0.0f)

/* Pausa entre PARAR na parede e COMECAR a girar.
 *
 * A pedido, e util por conta propria: sem ela o giro comecava enquanto o
 * carrinho ainda escorregava para frente, e esse deslizamento residual entra
 * como erro no giro fechado por ticks. Parado de verdade, o giro parte de uma
 * pose conhecida.
 *
 * O alvo de giro e' ABSOLUTO, entao qualquer tick contado durante a pausa
 * (roda acomodando) e' descontado sozinho -- a pausa nao introduz erro. */
#define NAV_TURN_PAUSE_MS       400u

/* Tolerancia do giro, em TICKS de diferenca entre as rodas.
 * TURN_TICKS_PER_90/30 equivale a uns 3 graus -- apertar mais do que isso so'
 * faz o carrinho ficar caçando o alvo, porque o encoder nem tem resolucao
 * util abaixo disso depois da inercia da parada. */
#define NAV_TURN_TOLERANCE_TICKS  (TURN_TICKS_PER_90 / 30)

/* ---------------------------------------------------------------------------
 * RECUO ANTES DO GIRO -- DESLIGADO, e o motivo importa
 * ---------------------------------------------------------------------------
 * Para girar no proprio eixo, TODO o corpo tem que caber num circulo de raio
 * 82,5 mm (a meia-largura do corredor livre) centrado no EIXO DAS RODAS.
 *
 * Com 125 x 110 e o eixo no meio do comprimento, o canto fica a
 *
 *      raiz(62,5^2 + 55^2) = 83,3 mm
 *
 * contra 82,5 disponiveis. Ou seja: o retangulo NAO CABE, nem centralizado,
 * nem recuado. Recuar desloca o eixo para tras, mas a parede LATERAL continua
 * a 82,5 mm -- entao recuo nao compra folga lateral nenhuma. Foi por isso que
 * a lateral esquerda bateu e a roda prendeu na quina.
 *
 * Duas saidas, e elas se somam:
 *
 *   a) ENCOSTAR NO LADO ABERTO antes de girar (SENSOR_TURN_HUG_PROX, no
 *      board.h). Deslocando ~10,5 mm para o lado que esta' aberto, a parede
 *      de fora vai de 82,5 para 93 mm e o canto passa com 9,7 mm de folga.
 *      E' a mesma folga que o chanfro daria, sem serra.
 *
 *   b) CHANFRAR os cantos. Continua valendo, e e' o unico caminho se um dia
 *      o lado de dentro da curva nao estiver aberto (beco, meia-volta).
 *      Cortando a 45 graus:
 *
 *      corte      canto mais distante do eixo      folga
 *      nenhum            83,3 mm                   BATE
 *      10 mm             77,0 mm                   5,5 mm
 *      15 mm             74,2 mm                   8,3 mm
 *      20 mm             71,6 mm                  10,9 mm
 *
 * 15 mm nos QUATRO cantos e' o alvo. Como o eixo pode nao estar no meio do
 * comprimento, o criterio pratico e' direto: meca com regua do EIXO ate' cada
 * canto, e corte todo canto que passar de 78 mm.
 *
 * ---------------------------------------------------------------------------
 * E o recuo em si?
 * ---------------------------------------------------------------------------
 * O carrinho precisa mesmo girar uns 10 mm atras do centro da celula, senao o
 * canto DIANTEIRO varre a parede da frente. Mas isso agora e' feito PARANDO
 * ANTES (SENSOR_FRONT_CENTER, hoje em 850) e, principalmente, ENCOSTANDO NO
 * LADO ABERTO durante a aproximacao (SENSOR_TURN_HUG_PROX) -- e nao andando
 * de re'.
 *
 * Motivo medido no PWM_TEST: de re' a roda esquerda so' arranca com duty 125,
 * contra 75 para frente. No registro do percurso o RECUA pediu 10 mm, ficou um
 * segundo com o duty subindo ate' -500, e entregou 5 mm. Um estado que depende
 * do sentido mais fraco do carrinho, bem no ponto mais critico do trajeto, e'
 * fragilidade de graca.
 *
 * Zero desliga o estado NAV_BACKUP inteiro.
 */
#define NAV_TURN_BACKUP_MM       0.0f

/* Se preferir virar sempre para um lado quando os dois estiverem abertos:
 * 0 = esquerda, 1 = direita */
#define NAV_PREFER_RIGHT        0

/* LATERAL NA DUVIDA na hora de escolher o giro (23/11). Parede de verdade
 * le' 685+, aberto ate' ~395, mas um lado ABERTO visto de vies (robo torto)
 * le' 530-590 -- logo acima do SENSOR_WALL_THRESHOLD (550). Assim o robo
 * achou "beco sem saida" com ESQ 557 e deu meia-volta. Agora:
 *   >= WALL_MIN parede | <= OPEN_MAX aberto | entre os dois: duvida.
 * Escolha: lado ABERTO certo primeiro; senao, lado na DUVIDA (tenta -- se
 * for parede, o frontal ve' ao sair e ele decide de novo); meia-volta so'
 * com os dois lados parede certa. Mesmos numeros do solver.h. */
#define NAV_SIDE_OPEN_MAX       420
#define NAV_SIDE_WALL_MIN       600     /* 23/11: parede real leu 640-660 (era 650) */

/* Distancia sem nenhuma parede que caracteriza a saida do labirinto */
#define NAV_EXIT_OPEN_MM        250.0f

/* Trava de distancia: se o percurso passar disso, para tudo. Serve para o
 * carrinho nao sair correndo pela sala se a logica travar.
 *
 * ZERO DESLIGA a trava -- e' o valor atual, a pedido, para o carrinho andar
 * livre enquanto a gente investiga o movimento estranho.
 *
 * O que continua ligado, e nao deveria ser desligado: as duas travas de
 * speed_ctl.c, que pegam roda travada (encoder mudo com esforco aplicado) e
 * sentido invertido (realimentacao positiva). Aquelas protegem o HARDWARE.
 * Esta aqui so' limita o passeio, entao desligar custa no maximo ter que
 * correr atras dele. */
#define NAV_MAX_DISTANCE_MM     0.0f
