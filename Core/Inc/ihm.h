/**
 ******************************************************************************
 * @file    ihm.h
 * @brief   Interface homem-maquina (LEDs e botao) para o MicroJM (Blue Pill).
 *          Compativel com a API port_ihm_... do fw_micromouse_f411.
 ******************************************************************************
 */
#ifndef IHM_H
#define IHM_H

#include <stdint.h>
#include <stdbool.h>

void port_ihm_init(void);

/** @brief Atualiza temporizadores e debounce. Chamar a cada CONTROL_PERIOD_MS (1 kHz). */
void port_ihm_update(void);

void port_ihm_led_set(bool on);
void port_ihm_led_toggle(void);
void port_ihm_led_blink(uint32_t period_ms);

/** @brief Controle do segundo LED (PA0 - LED_2) */
void port_ihm_led2_set(bool on);
void port_ihm_led2_toggle(void);

/** @brief Estado atual do botao com debounce */
bool port_ihm_button_pressed(void);

/** @brief Retorna true UMA vez a cada clique (borda de descida) */
bool port_ihm_button_clicked(void);

#endif /* IHM_H */

