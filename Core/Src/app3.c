/*
 * app3.c
 *
 *  Created on: 28 de out de 2019
 *      Author: lukas
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "app3.h"
#include "motor3.h"
#include "adc.h"
#include "imu.h"

// bateria adc-2410 - 8v

volatile uint8_t estado;
volatile uint8_t ligar = 0; // definicao de ligar
static uint32_t conta;

// Parameters for sensor distances
#define SETPOINT_LUZ_ESQ 700 // abaixo desse valor e branco
#define SETPOINT_LUZ_DIR 700

// Sensor distance - central 4 to 30 cm / side sensors 10 to 80 cm
#define DETECTED_RANGE_ESQ 1200 // object detected - target
#define DETECTED_RANGE_DIR 1200
#define DETECTED_RANGE_CENTRO 1000
#define CLOSE_RANGE_ESQ   2800 // object detected left sensor - attack
#define CLOSE_RANGE_DIR   2800
#define CLOSE_RANGE_CENTRO   2000

// Motor speed parameters
#define REVERSE_SPEED     12000 // 200 rpm
#define TURN_SPEED        15000
#define FORWARD_SPEED     17500
#define FORWARD_SPEED_2   19000
#define ATTACK_SPEED      19999

// Duration parameters (ms)
#define REVERSE_DURATION  150
#define TURN_DURATION     200
#define HUNT_DURATION     250

uint32_t hw_timer_get_tick_ms(void)
{
    return HAL_GetTick();
}

// Command queue definitions
#define MAX_COMMANDS 10
uint32_t startTime;
uint32_t nextCommand;
unsigned int commandIndex;
unsigned int numberOfCommands;
int leftSpeed[MAX_COMMANDS];
int rightSpeed[MAX_COMMANDS];
unsigned int commandDuration[MAX_COMMANDS];
unsigned int turnDirection = 1;

enum State { SURVIVE, HUNT, TARGET, ATTACK } state;

bool oponenteDetectado;
bool bordaDetectada;
bool oponenteDetectado_esq;
bool oponenteDetectado_dir;
bool detectado_oponente;
bool executandoComandos;

volatile extern uint32_t adc_data_avg[ADC_NUM_CHANNELS];

// Callback de interrupcao do botao (USER_KEY / SW2 no PA7)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SW2_Pin)
    {
        ligar = 1;
    }
}

void updateFromSensors(void)
{
    bordaDetectada = ((adc_data_avg[2] > SETPOINT_LUZ_DIR) || (adc_data_avg[3] > SETPOINT_LUZ_ESQ));
    oponenteDetectado = adc_data_avg[5] >= DETECTED_RANGE_CENTRO;
    oponenteDetectado_esq = adc_data_avg[0] >= DETECTED_RANGE_ESQ;
    oponenteDetectado_dir = adc_data_avg[1] >= DETECTED_RANGE_DIR;
    detectado_oponente = (oponenteDetectado_esq || oponenteDetectado_dir || oponenteDetectado);
}

void maquina_de_estado(void)
{
    switch (state)
    {
    case SURVIVE:
        if (!executandoComandos) hunt();
        else if (bordaDetectada) survive();
        break;
    case ATTACK:
        if (bordaDetectada) survive();
        else if (!detectado_oponente) hunt();
        else if (oponenteDetectado) target();
        break;
    case TARGET:
        if (bordaDetectada) survive();
        else if (!detectado_oponente) hunt();
        else if (adc_data_avg[5] >= CLOSE_RANGE_CENTRO) attack();
        else
            ;
        break;
    default: // HUNT
        if (bordaDetectada) survive();
        else if (detectado_oponente)
        {
            if (adc_data_avg[5] >= CLOSE_RANGE_CENTRO) attack();
            else target();
        }
        else if (!executandoComandos) hunt();
        break;
    }
}

void survive(void)
{
    state = SURVIVE;
    if (adc_data_avg[2] > SETPOINT_LUZ_DIR) turnDirection = -1; else turnDirection = 1;
    clearCommands();
    if ((adc_data_avg[2] > SETPOINT_LUZ_DIR) && (adc_data_avg[3] > SETPOINT_LUZ_ESQ))
        addCommand(turnDirection * TURN_SPEED, -turnDirection * TURN_SPEED, TURN_DURATION);
    addCommand(-REVERSE_SPEED, -REVERSE_SPEED, REVERSE_DURATION);
    addCommand(turnDirection * TURN_SPEED, -turnDirection * TURN_SPEED, TURN_DURATION);
    addCommand(FORWARD_SPEED, FORWARD_SPEED, REVERSE_DURATION);
    startExecutingCommands();
}

void hunt(void)
{
    state = HUNT;
    clearCommands();
    addCommand(FORWARD_SPEED, FORWARD_SPEED, HUNT_DURATION);
    startExecutingCommands();
}

void target(void)
{
    state = TARGET;
    clearCommands();
    if (oponenteDetectado_esq && !oponenteDetectado)
        addCommand(16000, -16000, 400);
    else if (oponenteDetectado_dir && !oponenteDetectado)
        addCommand(-16000, 16000, 400);
    else
        addCommand(FORWARD_SPEED_2, FORWARD_SPEED_2, 0);
    startExecutingCommands();
}

void attack(void)
{
    state = ATTACK;
    clearCommands();
    addCommand(ATTACK_SPEED, ATTACK_SPEED, 300);
    startExecutingCommands();
}

void clearCommands(void)
{
    executandoComandos = false;
    numberOfCommands = 0;
    nextCommand = hw_timer_get_tick_ms();
}

void startExecutingCommands(void)
{
    executandoComandos = true;
    commandIndex = 0;
    startTime = hw_timer_get_tick_ms();
    nextCommand = startTime;
}

void addCommand(int left, int right, int duration)
{
    if (numberOfCommands < MAX_COMMANDS)
    {
        leftSpeed[numberOfCommands] = left;
        rightSpeed[numberOfCommands] = right;
        commandDuration[numberOfCommands] = duration;
        numberOfCommands++;
    }
}

void executeCommands(void)
{
    if (hw_timer_get_tick_ms() >= nextCommand)
    {
        if (commandIndex < numberOfCommands)
        {
            setSpeeds(leftSpeed[commandIndex], rightSpeed[commandIndex]);
            nextCommand = nextCommand + commandDuration[commandIndex];
            commandIndex++;
        }
        else
            executandoComandos = false;
    }
}

void app3_init(void)
{
    motor3_init();
    //adc_init();
    imu_init();
    conta = 0;
    ligar = 0;
}

void app3_loop(void)
{
        setSpeeds(15000,15000);
        HAL_Delay(5000);
        setSpeeds(-15000,-15000);
        HAL_Delay(5000);
}
