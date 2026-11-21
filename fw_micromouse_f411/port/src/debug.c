/**
 ******************************************************************************
 * @file    debug.c
 * @brief   Saida de texto de debug -- NAO bloqueante, pela USB (CDC) ou UART.
 *
 * QUAL SAIDA E' USADA
 *
 * Se o projeto tiver a USB ligada no CubeMX (USB_OTG_FS + USB_DEVICE classe
 * CDC), o arquivo usbd_cdc_if.h existe e a saida vai pela USB-C da Black
 * Pill: aparece no PC como uma porta COM, sem conversor nenhum. Senao, cai
 * de volta na USART1 (PA9/PA10). A escolha e' automatica, em tempo de
 * compilacao -- o projeto compila nos dois estados. Para forcar a UART mesmo
 * com USB, ponha BOARD_DEBUG_FORCE_UART 1 no board.h.
 *
 * POR QUE NAO BLOQUEIA
 *
 * Sem RTOS tudo roda no mesmo laco, e um printf que esperasse a transmissao
 * travaria o PID. Aqui o printf so' COPIA o texto para um buffer circular;
 * quem esvazia e' port_debug_poll(), chamada a cada volta do laco.
 *
 * O CASO QUE A USB TRAZ: NINGUEM ESCUTANDO
 *
 * Pela UART, o byte sai pelo fio mesmo sem ninguem do outro lado. Pela USB,
 * nao: sem cabo, ou com o cabo ligado e o terminal FECHADO, o PC nao busca
 * os pacotes e a transmissao fica parada. Um printf que esperasse espaco no
 * buffer travaria o robo para sempre. Entao:
 *   - sem USB configurada, ou com a transmissao parada ha' mais de
 *     DEBUG_USB_STALL_MS, o texto que nao couber e' DESCARTADO, sem esperar;
 *   - o que ja' esta' no buffer fica guardado -- inclusive o cabecalho do
 *     boot, que aparece quando voce abrir o terminal.
 * O registro do percurso (trace) nao corre esse risco: ele vive na RAM e so'
 * e' impresso quando voce pede.
 ******************************************************************************
 */

#include "debug.h"
#include "board.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* --- escolha da saida ----------------------------------------------------- */
#ifndef BOARD_DEBUG_FORCE_UART
#define BOARD_DEBUG_FORCE_UART  0
#endif

#if (BOARD_DEBUG_FORCE_UART == 0) && defined(__has_include)
#  if __has_include("usbd_cdc_if.h")
#    define DEBUG_USE_USB   1
#  endif
#endif
#ifndef DEBUG_USE_USB
#  define DEBUG_USE_USB     0
#endif

extern UART_HandleTypeDef BOARD_DEBUG_UART;

#define DEBUG_FMT_BUFFER_SIZE   192u
#define DEBUG_RING_SIZE         4096u       /* potencia de 2 */
#define DEBUG_FULL_WAIT_MS      100u        /* espera maxima com buffer cheio */

static char     fmt_buffer[DEBUG_FMT_BUFFER_SIZE];
static uint8_t  ring[DEBUG_RING_SIZE];
static volatile uint32_t ring_head = 0u;    /* proxima posicao livre       */
static volatile uint32_t ring_tail = 0u;    /* proximo byte a transmitir   */
static volatile bool     key_flag  = false; /* chegou algo pela USB        */
static volatile bool     open_flag = false; /* terminal abriu a porta USB   */

/* Sem RTOS nao ha' o que travar. Mantidas __weak para compatibilidade. */
__weak void port_debug_lock(void)   { }
__weak void port_debug_unlock(void) { }

static uint32_t ring_used(void)
{
    return (ring_head - ring_tail) & (DEBUG_RING_SIZE - 1u);
}

static uint32_t ring_free(void)
{
    return (DEBUG_RING_SIZE - 1u) - ring_used();
}

/* ===========================================================================
 * Saida pela USB (CDC)
 * ======================================================================== */
#if (DEBUG_USE_USB != 0)

#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Transmissao parada por mais que isto = nao tem ninguem lendo. */
#define DEBUG_USB_STALL_MS      100u

/* Pacote de 63 bytes, nunca 64: uma transferencia de tamanho multiplo de 64
 * exige um pacote vazio no fim, e versoes antigas da biblioteca da ST nao o
 * mandam -- o PC fica esperando e a saida empaca. Com 63 o problema nao
 * existe em versao nenhuma. */
#define DEBUG_USB_CHUNK         63u

/* Dois buffers: um pode estar sendo lido pela USB enquanto o outro e'
 * preparado. So' se escreve no que NAO esta' em voo. */
static uint8_t  usb_buf[2][DEBUG_USB_CHUNK];
static uint8_t  usb_active  = 0u;       /* buffer da transferencia em voo   */
static uint16_t usb_staged  = 0u;       /* bytes prontos no outro buffer    */
static uint32_t usb_busy_t0 = 0u;       /* desde quando a USB diz "ocupada" */
static volatile bool usb_stalled = false;

/* Interface CDC original (gerada pelo CubeMX) e a nossa, que a envolve para
 * ouvir o que chega do PC sem precisar editar o usbd_cdc_if.c gerado. */
static USBD_CDC_ItfTypeDef usb_fops;
static int8_t (*orig_receive)(uint8_t *buf, uint32_t *len) = NULL;
static int8_t (*orig_control)(uint8_t cmd, uint8_t *pbuf, uint16_t length) = NULL;

static int8_t wrap_receive(uint8_t *buf, uint32_t *len)
{
    if ((len != NULL) && (*len > 0u))
    {
        key_flag = true;
    }
    return orig_receive(buf, len);      /* a original rearma o endpoint */
}

static int8_t wrap_control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    /* Terminal abrindo a porta (DTR ligado): recomeca a tentar mandar. */
    if (cmd == CDC_SET_CONTROL_LINE_STATE)
    {
        usb_stalled = false;
        usb_busy_t0 = 0u;
        open_flag   = true;     /* terminal abriu: reimprime o registro */
    }
    return orig_control(cmd, pbuf, length);
}

static bool usb_configured(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

static bool out_listening(void)
{
    return usb_configured() && !usb_stalled;
}

static void out_init(void)
{
    usb_fops        = USBD_Interface_fops_FS;   /* copia tudo, troca dois */
    orig_receive    = USBD_Interface_fops_FS.Receive;
    orig_control    = USBD_Interface_fops_FS.Control;
    usb_fops.Receive = wrap_receive;
    usb_fops.Control = wrap_control;
    (void)USBD_CDC_RegisterInterface(&hUsbDeviceFS, &usb_fops);
}

static void out_poll(void)
{
    if (!usb_configured())
    {
        return;
    }

    /* Prepara o proximo pedaco no buffer que NAO esta' em voo. */
    if (usb_staged == 0u)
    {
        uint8_t *dst = usb_buf[usb_active ^ 1u];
        uint16_t n   = 0u;

        while ((n < DEBUG_USB_CHUNK) && (ring_tail != ring_head))
        {
            dst[n++] = ring[ring_tail];
            ring_tail = (ring_tail + 1u) & (DEBUG_RING_SIZE - 1u);
        }
        usb_staged = n;
    }

    if (usb_staged == 0u)
    {
        return;
    }

    uint8_t st = CDC_Transmit_FS(usb_buf[usb_active ^ 1u], usb_staged);

    if (st == USBD_OK)
    {
        usb_active ^= 1u;
        usb_staged   = 0u;
        usb_busy_t0  = 0u;
        usb_stalled  = false;
    }
    else
    {
        /* Ocupada: o pedaco continua preparado e sera' tentado de novo.
         * Se ficar assim tempo demais, ninguem esta' lendo. */
        uint32_t now = HAL_GetTick();
        if (usb_busy_t0 == 0u)
        {
            usb_busy_t0 = now | 1u;
        }
        else if ((now - usb_busy_t0) > DEBUG_USB_STALL_MS)
        {
            usb_stalled = true;
        }
    }
}

static bool out_idle(void)
{
    return (ring_tail == ring_head) && (usb_staged == 0u);
}

static const char *out_name(void)
{
    return "USB (porta COM virtual)";
}

/* ===========================================================================
 * Saida pela UART (fallback)
 * ======================================================================== */
#else

static bool out_listening(void)
{
    return true;    /* o fio sempre "escuta" */
}

static void out_init(void)
{
}

static void out_poll(void)
{
    USART_TypeDef *u = BOARD_DEBUG_UART.Instance;

    while ((ring_tail != ring_head) && ((u->SR & USART_SR_TXE) != 0u))
    {
        u->DR = ring[ring_tail];
        ring_tail = (ring_tail + 1u) & (DEBUG_RING_SIZE - 1u);
    }
}

static bool out_idle(void)
{
    return (ring_tail == ring_head)
        && ((BOARD_DEBUG_UART.Instance->SR & USART_SR_TC) != 0u);
}

static const char *out_name(void)
{
    return "USART1 (PA9/PA10, 115200)";
}

#endif

/* ===========================================================================
 * Comum
 * ======================================================================== */

static void ring_put(const char *s, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        if (ring_free() == 0u)
        {
            /* Buffer cheio. Com alguem escutando, espera esvaziar (so'
             * acontece em despejo grande, com o robo parado). Sem ninguem,
             * descarta o resto na hora -- nunca trava o robo. */
            uint32_t t0 = HAL_GetTick();

            while ((ring_free() == 0u) && out_listening()
                   && ((HAL_GetTick() - t0) < DEBUG_FULL_WAIT_MS))
            {
                out_poll();
            }

            if (ring_free() == 0u)
            {
                return;
            }
        }

        ring[ring_head] = (uint8_t)s[i];
        ring_head = (ring_head + 1u) & (DEBUG_RING_SIZE - 1u);
    }
}

void port_debug_init(void)
{
    ring_head = 0u;
    ring_tail = 0u;
    key_flag  = false;
#if (DEBUG_ENABLED != 0)
    out_init();
    port_debug_write("\r\n\r\n");
#endif
}

const char *port_debug_output_name(void)
{
    return out_name();
}

void port_debug_poll(void)
{
#if (DEBUG_ENABLED != 0)
    out_poll();
#endif
}

void port_debug_flush(void)
{
#if (DEBUG_ENABLED != 0)
    /* Sem ninguem escutando nao adianta esperar: o texto fica no buffer
     * e sai quando o terminal abrir. */
    uint32_t t0 = HAL_GetTick();

    while (!out_idle() && out_listening() && ((HAL_GetTick() - t0) < 2000u))
    {
        out_poll();
    }
#endif
}

void port_debug_write(const char *str)
{
#if (DEBUG_ENABLED != 0)
    if (str == NULL)
    {
        return;
    }
    ring_put(str, strlen(str));
#else
    (void)str;
#endif
}

bool port_debug_port_opened(void)
{
#if (DEBUG_ENABLED != 0)
    bool got = open_flag;
    open_flag = false;
    return got;
#else
    return false;
#endif
}

bool port_debug_key(void)
{
#if (DEBUG_ENABLED != 0)
    bool got = false;

    if (key_flag)
    {
        key_flag = false;
        got = true;
    }

    /* A UART continua sendo ouvida mesmo com saida pela USB. Overrun trava a
     * recepcao na F4 ate' ler SR e depois DR, entao le' os dois sempre. */
    USART_TypeDef *u = BOARD_DEBUG_UART.Instance;
    if ((u->SR & (USART_SR_RXNE | USART_SR_ORE)) != 0u)
    {
        (void)u->DR;
        got = true;
    }
    return got;
#else
    return false;
#endif
}

void port_debug_printf(const char *fmt, ...)
{
#if (DEBUG_ENABLED != 0)
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(fmt_buffer, sizeof(fmt_buffer), fmt, args);
    va_end(args);

    if (len > 0)
    {
        if ((size_t)len >= sizeof(fmt_buffer))
        {
            len = (int)sizeof(fmt_buffer) - 1;   /* truncou */
        }
        ring_put(fmt_buffer, (size_t)len);
    }
#else
    (void)fmt;
#endif
}
