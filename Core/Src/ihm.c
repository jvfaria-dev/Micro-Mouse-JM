/**
 ******************************************************************************
 * @file    ihm.c
 * @brief   LEDs de status e botao de usuario para MicroJM (Blue Pill).
 ******************************************************************************
 */
#include "ihm.h"
#include "board.h"
#include "main.h"

#define IHM_DEBOUNCE_MS     20u

static uint32_t led_blink_period = 0u;
static uint32_t led_blink_acc    = 0u;

static bool     btn_stable       = false;
static bool     btn_raw_last     = false;
static uint32_t btn_stable_ms    = 0u;
static bool     btn_click_event  = false;

void port_ihm_init(void)
{
    /* Garante clocks habilitados */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Inicializa LED onboard PC13 como saida caso nao esteja */
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);

    port_ihm_led_set(false);
    port_ihm_led2_set(false);

    led_blink_period = 0u;
    led_blink_acc    = 0u;

    btn_stable      = false;
    btn_raw_last    = false;
    btn_stable_ms   = 0u;
    btn_click_event = false;
}

void port_ihm_led_set(bool on)
{
    /* Aciona tanto LED_1 (PA1) quanto o LED onboard (PC13, ativo baixo) */
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void port_ihm_led_toggle(void)
{
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void port_ihm_led2_set(bool on)
{
    HAL_GPIO_WritePin(BOARD_LED2_PORT, BOARD_LED2_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void port_ihm_led2_toggle(void)
{
    HAL_GPIO_TogglePin(BOARD_LED2_PORT, BOARD_LED2_PIN);
}

void port_ihm_led_blink(uint32_t period_ms)
{
    led_blink_period = period_ms;
    led_blink_acc    = 0u;

    if (period_ms == 0u)
    {
        port_ihm_led_set(false);
    }
}

static bool port_ihm_button_raw(void)
{
#if (BOARD_HAS_BUTTON != 0)
    GPIO_PinState state = HAL_GPIO_ReadPin(BOARD_BTN_PORT, BOARD_BTN_PIN);
#if (BOARD_BTN_ACTIVE_LOW != 0)
    return (state == GPIO_PIN_RESET);
#else
    return (state == GPIO_PIN_SET);
#endif
#else
    return false;
#endif
}

void port_ihm_update(void)
{
    /* Pisca automatico do LED */
    if (led_blink_period > 0u)
    {
        led_blink_acc += CONTROL_PERIOD_MS;
        if (led_blink_acc >= (led_blink_period / 2u))
        {
            led_blink_acc = 0u;
            port_ihm_led_toggle();
        }
    }

    /* Debounce do botao */
    bool raw = port_ihm_button_raw();

    if (raw != btn_raw_last)
    {
        btn_raw_last  = raw;
        btn_stable_ms = 0u;
    }
    else
    {
        if (btn_stable_ms < IHM_DEBOUNCE_MS)
        {
            btn_stable_ms += CONTROL_PERIOD_MS;
            if (btn_stable_ms >= IHM_DEBOUNCE_MS)
            {
                if (!btn_stable && raw)
                {
                    btn_click_event = true;
                }
                btn_stable = raw;
            }
        }
    }
}

bool port_ihm_button_pressed(void)
{
    return btn_stable;
}

bool port_ihm_button_clicked(void)
{
    bool c = btn_click_event;
    btn_click_event = false;
    return c;
}

