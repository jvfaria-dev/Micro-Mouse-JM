/**
 ******************************************************************************
 * @file    debug.h
 * @brief   Telemetria e printf nao-bloqueante para o MicroJM (Blue Pill).
 *          Suporta buffer de RAM (Live Expressions na IDE) e USART1 se presente.
 ******************************************************************************
 */
#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include <stdbool.h>

#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED   1
#endif

/** Imprime valor em milesimos como "-12.345" sem usar float */
#define DEBUG_MILLI_FMT         "%s%ld.%03ld"
#define DEBUG_MILLI_ARGS(v)     (((v) < 0) ? "-" : ""),           \
                                (long)(((v) < 0 ? -(v) : (v)) / 1000), \
                                (long)(((v) < 0 ? -(v) : (v)) % 1000)

void        port_debug_init(void);
void        port_debug_lock(void);
void        port_debug_unlock(void);
void        port_debug_poll(void);
void        port_debug_flush(void);
const char *port_debug_output_name(void);
void        port_debug_printf(const char *fmt, ...);
void        port_debug_write(const char *str);
bool        port_debug_key(void);
bool        port_debug_port_opened(void);

#endif /* DEBUG_H */

