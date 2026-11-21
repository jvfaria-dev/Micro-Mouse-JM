/**
 ******************************************************************************
 * @file    debug.c
 * @brief   Telemetria pela serial (USART3: PB10 TX, PB11 RX, 115200 8N1)
 *          com printf nao-bloqueante, para a Blue Pill.
 *
 * Como funciona:
 *   - port_debug_printf() so' formata e copia para um anel de 1 KB na RAM.
 *   - port_debug_poll(), chamado pelo laco principal (todo ms e a cada volta),
 *     entrega os bytes a' USART3 enquanto o registrador de transmissao estiver
 *     livre. Assim o controle de 1 kHz nao trava esperando a serial.
 *   - Se o anel encher (despejo grande do trace/solver, com o robo parado),
 *     a escrita espera a serial abrir espaco, em vez de perder linhas.
 *   - Qualquer byte recebido no RX vira uma "tecla" (port_debug_key) e um
 *     pedido de reimpressao (port_debug_port_opened).
 *   - A ultima linha continua em debug_last_line (Live Expressions).
 *
 * A USART3 e' configurada aqui mesmo (nao pelo CubeMX), entao nao se perde
 * se o main.c for gerado de novo.
 ******************************************************************************
 */
#include "debug.h"
#include "board.h"
#include "main.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_RING_SIZE         1024u   /* buffer circular em RAM (potencia de 2) */
#define DEBUG_FMT_BUF_SIZE      192u
#define DEBUG_FULL_WAIT_MS      50u     /* espera maxima por espaco no anel     */
#define DEBUG_FLUSH_WAIT_MS     300u    /* espera maxima do port_debug_flush()  */

static char              fmt_buffer[DEBUG_FMT_BUF_SIZE];
static uint8_t           ring[DEBUG_RING_SIZE];
static volatile uint32_t ring_head   = 0u;
static volatile uint32_t ring_tail   = 0u;
static volatile bool     key_flag    = false;
static volatile bool     opened_flag = false;
static bool              uart_ready  = false;

/* Buffer de visualizacao no Live Expressions do STM32CubeIDE */
char debug_last_line[128] = {0};

void port_debug_lock(void)   { }
void port_debug_unlock(void) { }

static uint32_t ring_used(void)
{
    return (ring_head - ring_tail) & (DEBUG_RING_SIZE - 1u);
}

static uint32_t ring_free(void)
{
    return (DEBUG_RING_SIZE - 1u) - ring_used();
}

/* -------------------------------------------------------------------------- */
/* USART3 (PB10 TX / PB11 RX)                                                 */
/* -------------------------------------------------------------------------- */

static void uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* PB10 = TX: funcao alternativa push-pull */
    g.Pin   = GPIO_PIN_10;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB11 = RX: entrada com pull-up, para o pino solto nao virar "tecla" */
    g.Pin  = GPIO_PIN_11;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);

    /* 8N1, sem controle de fluxo. USART3 fica no APB1 (32 MHz aqui). */
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    BOARD_DEBUG_UART->CR1 = 0u;
    BOARD_DEBUG_UART->CR2 = 0u;
    BOARD_DEBUG_UART->CR3 = 0u;
    BOARD_DEBUG_UART->BRR = (pclk + (BOARD_DEBUG_BAUD / 2u)) / BOARD_DEBUG_BAUD;
    BOARD_DEBUG_UART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    uart_ready = true;
}

void port_debug_init(void)
{
    ring_head   = 0u;
    ring_tail   = 0u;
    key_flag    = false;
    opened_flag = false;
    memset(debug_last_line, 0, sizeof(debug_last_line));
    uart_init();
}

void port_debug_poll(void)
{
    if (!uart_ready)
    {
        ring_tail = ring_head;      /* sem serial: descarta para nao acumular */
        return;
    }

    /* Recepcao: qualquer byte conta como tecla. Ler SR e depois DR tambem
     * limpa o erro de overrun (ORE). */
    uint32_t sr = BOARD_DEBUG_UART->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        (void)BOARD_DEBUG_UART->DR;
        if (sr & USART_SR_RXNE)
        {
            key_flag    = true;
            opened_flag = true;
        }
    }

    /* Transmissao: um byte por vez enquanto o registrador estiver livre */
    while ((ring_tail != ring_head) && ((BOARD_DEBUG_UART->SR & USART_SR_TXE) != 0u))
    {
        BOARD_DEBUG_UART->DR = ring[ring_tail];
        ring_tail = (ring_tail + 1u) & (DEBUG_RING_SIZE - 1u);
    }
}

void port_debug_flush(void)
{
    /* Espera o anel esvaziar (usado antes de esperas longas e no boot). */
    uint32_t t0 = HAL_GetTick();
    while ((ring_tail != ring_head) && ((HAL_GetTick() - t0) < DEBUG_FLUSH_WAIT_MS))
    {
        port_debug_poll();
    }
    /* e o ultimo byte sair do registrador de deslocamento */
    if (uart_ready)
    {
        while (((BOARD_DEBUG_UART->SR & USART_SR_TC) == 0u)
               && ((HAL_GetTick() - t0) < DEBUG_FLUSH_WAIT_MS)) { }
    }
}

const char *port_debug_output_name(void)
{
    return uart_ready ? "USART3 115200 8N1 (PB10 TX, PB11 RX)"
                      : "RAM Buffer / Live Expressions";
}

static void ring_put(uint8_t c)
{
    if (ring_free() == 0u)
    {
        if (!uart_ready)
        {
            return;
        }
        /* Anel cheio: so' acontece em despejos grandes, com o robo parado.
         * Espera a serial abrir espaco em vez de perder o texto. */
        uint32_t t0 = HAL_GetTick();
        while ((ring_free() == 0u) && ((HAL_GetTick() - t0) < DEBUG_FULL_WAIT_MS))
        {
            port_debug_poll();
        }
        if (ring_free() == 0u)
        {
            return;     /* serial travada: desiste deste byte */
        }
    }
    ring[ring_head] = c;
    ring_head = (ring_head + 1u) & (DEBUG_RING_SIZE - 1u);
}

void port_debug_write(const char *str)
{
    if (str == NULL) return;

    size_t len = strlen(str);
    if (len > 0u)
    {
        strncpy(debug_last_line, str, sizeof(debug_last_line) - 1u);
        debug_last_line[sizeof(debug_last_line) - 1u] = '\0';
    }

    for (size_t i = 0u; i < len; i++)
    {
        ring_put((uint8_t)str[i]);
    }
}

void port_debug_printf(const char *fmt, ...)
{
#if (DEBUG_ENABLED != 0)
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(fmt_buffer, sizeof(fmt_buffer), fmt, args);
    va_end(args);

    if (n > 0)
    {
        port_debug_write(fmt_buffer);
    }
#else
    (void)fmt;
#endif
}

/* printf() "normal" da newlib tambem sai pela serial (via syscalls.c). */
int __io_putchar(int ch)
{
    ring_put((uint8_t)ch);
    return ch;
}

bool port_debug_key(void)
{
    bool k = key_flag;
    key_flag = false;
    return k;
}

/* Na F411 isto era "o terminal abriu a porta USB". Numa UART nao da' para
 * saber, entao vale "chegou um byte": aperte qualquer tecla no terminal para
 * pedir a reimpressao do registro. */
bool port_debug_port_opened(void)
{
    bool o = opened_flag;
    opened_flag = false;
    return o;
}
