/**
 ******************************************************************************
 * @file    debug.h
 * @brief   Saida de texto de debug: pela USB-C da Black Pill (porta COM
 *          virtual) se a USB estiver ligada no CubeMX; senao pela USART1
 *          (PA9/PA10, 115200 8N1). Ver port/src/debug.c.
 *
 * Use um conversor USB-Serial ligado assim:
 *      PA9  (TX do STM32) -> RX do conversor
 *      PA10 (RX do STM32) -> TX do conversor
 *      GND                -> GND
 *
 * OBS: port_debug_printf NAO suporta %f. Para imprimir numero fracionario use
 * o helper de milesimos (ver DEBUG_MILLI_FMT / DEBUG_MILLI_ARGS abaixo) ou
 * habilite "-u _printf_float" nas opcoes do linker.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Habilita/desabilita toda a telemetria em tempo de compilacao.
 * Com 0 as chamadas viram no-op e nao custam nada. */
#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED   1
#endif

/**
 * Imprime um valor guardado em milesimos como "-12.345", sem usar %f.
 * Uso:  port_debug_printf("rpm=" DEBUG_MILLI_FMT "\r\n", DEBUG_MILLI_ARGS(v));
 */
#define DEBUG_MILLI_FMT         "%s%ld.%03ld"
#define DEBUG_MILLI_ARGS(v)     (((v) < 0) ? "-" : ""),           \
                                (long)(((v) < 0 ? -(v) : (v)) / 1000), \
                                (long)(((v) < 0 ? -(v) : (v)) % 1000)

void port_debug_init(void);

/** Mantidas por compatibilidade com o codigo da versao FreeRTOS; sem RTOS
 *  nao fazem nada. */
void port_debug_lock(void);
void port_debug_unlock(void);

/**
 * @brief Esvazia o buffer de saida na UART. Chamar a cada volta do laco
 *        principal (app_loop ja' faz isso). Nao bloqueia.
 */
void port_debug_poll(void);

/** @brief Espera o texto pendente sair (ate' 2 s). Nao espera se ninguem
 *         estiver escutando -- o texto fica guardado no buffer. */
void port_debug_flush(void);

/** @brief "USB (porta COM virtual)" ou "USART1 ...", para o cabecalho. */
const char *port_debug_output_name(void);

/** @brief printf NAO bloqueante: copia para o buffer e volta. */
void port_debug_printf(const char *fmt, ...);

/** @brief Escreve uma string crua (mais barato que printf). */
/** @brief true uma vez cada vez que o terminal (re)abre a porta USB. Nao
 *         depende de tecla nem da UART (RX solto "recebe" ruido sem parar). */
bool port_debug_port_opened(void);

void port_debug_write(const char *str);

/**
 * @brief Ha' alguma tecla esperando na UART? Consome UM caractere e devolve
 *        true; devolve false imediatamente se nao houver nada.
 *
 * Nao bloqueia (timeout zero), entao pode ser chamada de dentro de um laco
 * periodico sem atrapalhar. Serve para pedir alguma coisa ao firmware DEPOIS
 * de ele ja' estar rodando -- por exemplo, despejar o registro do percurso
 * quando voce finalmente conseguir conectar o conversor serial.
 */
bool port_debug_key(void);
