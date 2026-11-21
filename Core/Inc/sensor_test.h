/*
 * sensor_test.h
 *
 * Modo de teste isolado para validacao dos sensores ADC.
 * Motores parados, LEDs indicam deteccao, valores disponiveis
 * via Live Expressions no debugger (adc_test_values[]).
 *
 * Mapeamento dos canais ADC1 (conforme .ioc do MicroJM):
 *   adc_data_avg[0] = CH4  (PA4) = SL2 sharp esquerda
 *   adc_data_avg[1] = CH5  (PA5) = SL3 borda direita
 *   adc_data_avg[2] = CH3  (PA3) = SL1 sharp direita
 *   adc_data_avg[3] = CH6  (PA6) = SL4 borda esquerda
 *   adc_data_avg[4] = CH2  (PA2) = CargBat bateria
 *   adc_data_avg[5] = CH9  (PB1) = SL5 sensor central
 */
#ifndef SENSOR_TEST_H_
#define SENSOR_TEST_H_

#include <stdint.h>

/* Inicializa o modo de teste (motores parados, ADC rodando) */
void sensor_test_init(void);

/* Loop do modo de teste: le os sensores, atualiza LEDs e variaveis de debug */
void sensor_test_loop(void);

#endif /* SENSOR_TEST_H_ */

