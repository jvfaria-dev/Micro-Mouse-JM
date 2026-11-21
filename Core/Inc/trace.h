/**
 ******************************************************************************
 * @file    trace.h
 * @brief   Registro do percurso em RAM, para ser lido DEPOIS da corrida.
 *
 * ---------------------------------------------------------------------------
 * POR QUE ISTO EXISTE
 * ---------------------------------------------------------------------------
 * A telemetria por UART so' serve quando da' para ficar com o fio ligado, e
 * com o carrinho andando pelo labirinto nao da'. Sem o fio, todo printf feito
 * durante a corrida cai no vazio: a UART transmite do mesmo jeito, ninguem
 * escuta, e a informacao que interessa -- justamente a do instante em que ele
 * errou -- some.
 *
 * A solucao e' desacoplar QUANDO o dado e' produzido de QUANDO ele e' lido.
 * Durante a corrida cada amostra vai para um vetor em RAM, o que custa uma
 * copia de 20 bytes e nenhuma espera. Terminado o percurso, voce conecta o
 * conversor serial com calma e manda despejar.
 *
 * ---------------------------------------------------------------------------
 * COMO USAR
 * ---------------------------------------------------------------------------
 *   1. Solte o carrinho SEM fio nenhum e deixe ele fazer o percurso.
 *   2. Pegue o carrinho. NAO APERTE RESET -- o registro vive em RAM e o
 *      reset apaga tudo.
 *   3. Ligue o conversor serial (PA9/PA10/GND) e abra o terminal.
 *   4. Aperte qualquer tecla no terminal. O firmware despeja o registro
 *      inteiro. Pode repetir quantas vezes quiser.
 *
 * Ao chegar em FIM ou ABORTADO ele tambem despeja sozinho, de tempos em
 * tempos, para o caso de voce conectar sem ter nada para digitar.
 *
 * ---------------------------------------------------------------------------
 * O BUFFER E' CIRCULAR
 * ---------------------------------------------------------------------------
 * Passando de TRACE_CAPACITY amostras, as mais antigas sao sobrescritas. Isso
 * e' proposital: quando alguma coisa da' errado, o que explica o erro sao os
 * ultimos segundos, nao os primeiros. A 10 Hz o buffer guarda um minuto de
 * corrida, varias vezes o seu percurso.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* 150 amostras a 10 Hz = 15 s. Otimizado para os 20 KB de RAM da Blue Pill STM32F103C8 */
#define TRACE_CAPACITY      150u

/* Quando considerar que o carrinho PAROU, para despejar o registro sozinho.
 *
 * Encostado numa parede a navegacao nao termina e, com a trava de distancia
 * desligada, tambem nao aborta -- ela fica em CORREDOR para sempre. Perguntar
 * "ja' acabou?" nunca responde sim, e o registro da batida nunca sai. Quem
 * responde direito e' o encoder: parou de contar, parou de andar. */
#define TRACE_STILL_TICKS   20      /* movimento por amostra que ainda conta */
#define TRACE_STILL_MS      4000u   /* tempo parado para valer como parado   */

/** @brief Zera o registro. Chamar uma vez, antes de comecar. */
void trace_init(void);

/**
 * @brief Grava uma amostra do estado atual. Chamar periodicamente.
 *        Barata de proposito: so' copia numeros, nao formata nada.
 */
void trace_sample(void);

/** @brief Despeja o registro inteiro pela UART. Demora alguns segundos. */
void trace_dump(void);

/** @brief Quantas amostras estao guardadas agora. */
uint32_t trace_count(void);
