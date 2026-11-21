/**
 ******************************************************************************
 * @file    ihm.h
 * @brief   Interface homem-maquina minima: LED de status e botao de usuario.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

void port_ihm_init(void);

/** @brief Deve ser chamada a cada CONTROL_PERIOD_MS (faz o debounce). */
void port_ihm_update(void);

void port_ihm_led_set(bool on);
void port_ihm_led_toggle(void);

/**
 * @brief Pisca o LED sozinho.
 * @param period_ms periodo do piscar; 0 desliga o modo automatico.
 */
void port_ihm_led_blink(uint32_t period_ms);

/** @brief Estado atual do botao, ja' com debounce. */
bool port_ihm_button_pressed(void);

/**
 * @brief Retorna true UMA vez a cada borda de descida do botao.
 *        A leitura consome o evento.
 */
bool port_ihm_button_clicked(void);
