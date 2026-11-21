# Micromouse TUR — firmware STM32F411 (sem RTOS)

Porte do firmware da Black Pill STM32U585 (FreeRTOS) para a Black Pill
STM32F411CEU6, chassi v2.

## Estrutura

```
Core/    gerado pelo CubeMX — só os blocos USER CODE do main.c foram tocados
port/    hardware: board.h (TODOS os pinos e constantes), motor, encoder,
         sensores IR, IMU, LED, serial
app/     lógica: app_main (laço + escolha do modo), controle de velocidade,
         navegação, modos de teste e calibração
```

`main.c` chama só três coisas: `app_init()` no USER CODE 2, `app_loop()` no
`while(1)` e `app_emergency_stop()` no `Error_Handler`. Regerar o projeto no
CubeMX não apaga nada disso.

## Como escolher o modo

`app/inc/app_main.h` → `#define APP_MODE ...`

| modo | para quê |
|---|---|
| `APP_MODE_MOTOR_TEST` | **primeiro teste**: fiação, sentido das rodas, encoder de cada lado (rodas no ar) |
| `APP_MODE_SENSOR_CAL` | os 8 números dos sensores IR |
| `APP_MODE_TURN_CAL` | `TURN_TICKS_PER_90` (giro de 90°) |
| `APP_MODE_MAZE` | labirinto |
| `APP_MODE_IMU_TEST` | confere o MPU-6050 (motores desligados) |
| `APP_MODE_PWM_TEST`, `ENCODER_CAL`, `ODOM_CAL` | opcionais |

## O que mudou em relação à U5

- **Sem RTOS.** As quatro tarefas viraram um escalonador no laço principal
  (`app_main.c`): controle a 1 kHz, sensores a 200 Hz, navegação a 100 Hz,
  telemetria a 10 Hz, em fases diferentes para não caírem no mesmo ms.
- **printf não bloqueia mais.** Copia para um buffer circular de 2 KB e a UART
  é alimentada pelo próprio laço. Sem isso, uma linha de log travaria o PID
  por 8 ms.
- **Clock independente.** O PWM (20 kHz) e o prescaler do ADC são calculados
  a partir do clock real. Funciona com o HSI de 16 MHz do jeito que o CubeMX
  gerou, ou se você subir para 100 MHz depois.
- **Correções sobre o .ioc**, feitas em código para sobreviver a uma
  regeração: encoder em x4 (o .ioc gerou x2), filtro nas entradas do encoder,
  amostragem de 480 ciclos no ADC (o .ioc gerou 3), PWM de 20 kHz (o .ioc
  gerou 244 Hz), LED do PC13.
- **Sensores em PA0..PA3 = ADC1_IN0..IN3** (na U5 os canais eram outros).
- **IMU no I2C3: SCL = PA8, SDA = PB4.** Driver novo (`port/imu.c`) só do
  giroscópio Z, mais o modo `IMU_TEST`. Ainda não entra na navegação.
- **Chassi v2**: roda 43 mm, bitola 102 mm, `TURN_TICKS_PER_90` teórico 691,
  e removidos os remendos do chassi antigo (aparo de −20° no giro, encostar
  no lado aberto antes da curva, parar antes do centro da célula).

## Valores que AINDA precisam de medida

- sensores IR (`board.h` 3d) — palpite, U5 ÷ 4
- `TURN_TICKS_PER_90` — teórico
- `MOTOR_*_INVERT` / `ENCODER_*_INVERT` — herdados, conferir no MOTOR_TEST
- `SPEED_GAIN_RPM_PER_DUTY` — medido com outra bateria (PWM_TEST, opcional)
