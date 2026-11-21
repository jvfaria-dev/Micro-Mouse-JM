/**
 ******************************************************************************
 * @file    ihm.c
 * @brief   LED de status e botao de usuario com debounce.
 ******************************************************************************
 */

#include "ihm.h"
#include "board.h"

#define IHM_DEBOUNCE_MS     20u

static uint32_t led_blink_period = 0u;
static uint32_t led_blink_acc    = 0u;

static bool     btn_stable       = false;
static bool     btn_raw_last     = false;
static uint32_t btn_stable_ms    = 0u;
static bool     btn_click_event  = false;

/* -------------------------------------------------------------------------- */

void port_ihm_init(void)
{
    /* Nem o LED nem o botao estao no .ioc da F411: os pinos sao
     * configurados aqui, para nao depender do CubeMX. */
    GPIO_InitTypeDef gpio = {0};

#if (BOARD_HAS_LED != 0)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin   = BOARD_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_PORT, &gpio);
#endif

#if (BOARD_HAS_BUTTON != 0)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = BOARD_BTN_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = (BOARD_BTN_ACTIVE_LOW != 0) ? GPIO_PULLUP : GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_BTN_PORT, &gpio);
#endif
    (void)gpio;

    port_ihm_led_set(false);

    led_blink_period = 0u;
    led_blink_acc    = 0u;

    btn_stable      = false;
    btn_raw_last    = false;
    btn_stable_ms   = 0u;
    btn_click_event = false;
}

void port_ihm_led_set(bool on)
{
#if (BOARD_HAS_LED != 0)
#if (BOARD_LED_ACTIVE_LOW != 0)
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#else
    (void)on;
#endif
}

void port_ihm_led_toggle(void)
{
#if (BOARD_HAS_LED != 0)
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
#endif
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
    /* --- LED automatico --- */
    if (led_blink_period > 0u)
    {
        led_blink_acc += CONTROL_PERIOD_MS;

        if (led_blink_acc >= (led_blink_period / 2u))
        {
            led_blink_acc = 0u;
            port_ihm_led_toggle();
        }
    }

    /* --- Botao com debounce --- */
    bool raw = port_ihm_button_raw();

    if (raw != btn_raw_last)
    {
        btn_raw_last  = raw;
        btn_stable_ms = 0u;
    }
    else if (btn_stable_ms < IHM_DEBOUNCE_MS)
    {
        btn_stable_ms += CONTROL_PERIOD_MS;

        if (btn_stable_ms >= IHM_DEBOUNCE_MS && raw != btn_stable)
        {
            btn_stable = raw;

            if (btn_stable)   /* borda de "apertou" */
            {
                btn_click_event = true;
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
    bool event = btn_click_event;
    btn_click_event = false;
    return event;
}
