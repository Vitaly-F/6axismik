#include "serial.h"
#include "config.h"
#include "board.h"
#include "stepper.h"

#ifndef HOST_TEST
#include "uart_lib.h"
#endif

#define RX_SIZE 256u
#define TX_SIZE 1024u

static volatile uint8_t rx_buf[RX_SIZE];
static volatile uint32_t rx_head, rx_tail;
static uint8_t tx_buf[TX_SIZE];
static volatile uint32_t tx_head, tx_tail;
static bool tx_busy;
static uint32_t tx_t0;

volatile uint8_t rt_status_req;
volatile uint8_t rt_halt_req;
volatile uint8_t rx_overflow;

/* Если флаг "передача завершена" почему-то не придёт, через 1 мс считаем байт отправленным */
#define TX_TIMEOUT_TICKS (STEP_TICK_HZ / 1000u + 1u)

void serial_init(void)
{
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    tx_busy = false;
#ifndef HOST_TEST
    uint32_t divider = ((uint32_t)OSC_SYSTEM_VALUE + UART_BAUD / 2u) / UART_BAUD;
    UART_Init(UART_0, divider,
              UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_RXNEIE_M | UART_CONTROL1_M_8BIT_M,
              0, 0);
#endif
}

/* Обработка одного принятого байта (из прерывания) */
static inline __attribute__((always_inline)) void rx_char(uint8_t c)
{
    if (c == '!')
    {
        st_halt_req = 1; /* генератор шагов остановится на ближайшем тике */
        rt_halt_req = 1;
    }
    else if (c == '?')
    {
        rt_status_req = 1;
    }
    else if ((uint32_t)(rx_head - rx_tail) < RX_SIZE)
    {
        rx_buf[rx_head & (RX_SIZE - 1u)] = c;
        rx_head = rx_head + 1u;
    }
    else
    {
        rx_overflow = 1;
    }
}

#ifdef HOST_TEST
void host_rx_char(uint8_t c) { rx_char(c); }
#endif

/* Прерывание приёма. Выполняется из ОЗУ. */
RAM_FUNC void serial_isr(void)
{
#ifndef HOST_TEST
    uint32_t f = UART_0->FLAGS;
    if (f & UART_FLAGS_RXNE_M)
    {
        uint8_t c = (uint8_t)UART_0->RXDATA;
        /* Сбрасываем только RXNE: остальные флаги (TC) нужны передатчику */
        UART_0->FLAGS = UART_FLAGS_RXNE_M;
        rx_char(c);
    }
    if (f & UART_FLAGS_ORE_M)
    {
        UART_0->FLAGS = UART_FLAGS_ORE_M;
        rx_overflow = 1;
    }
#endif
}

int serial_getc(void)
{
    if (rx_tail == rx_head)
        return -1;
    uint8_t c = rx_buf[rx_tail & (RX_SIZE - 1u)];
    rx_tail = rx_tail + 1u;
    return c;
}

void serial_poll(void)
{
#ifndef HOST_TEST
    if (tx_busy)
    {
        if (!(UART_0->FLAGS & UART_FLAGS_TC_M) && (stepper_ticks() - tx_t0) < TX_TIMEOUT_TICKS)
            return; /* предыдущий байт ещё уходит */
        tx_busy = false;
    }
    if (tx_tail != tx_head)
    {
        UART_0->FLAGS = UART_FLAGS_TC_M; /* сброс "передача завершена" перед новым байтом */
        UART_0->TXDATA = tx_buf[tx_tail & (TX_SIZE - 1u)];
        tx_tail = tx_tail + 1u;
        tx_busy = true;
        tx_t0 = stepper_ticks();
    }
#else
    extern void host_uart_out(char c);
    while (tx_tail != tx_head)
    {
        host_uart_out((char)tx_buf[tx_tail & (TX_SIZE - 1u)]);
        tx_tail = tx_tail + 1u;
    }
#endif
}

void serial_putc(char c)
{
    while ((uint32_t)(tx_head - tx_tail) >= TX_SIZE)
        serial_poll(); /* буфер полон — ждём, пока уйдёт часть */
    tx_buf[tx_head & (TX_SIZE - 1u)] = (uint8_t)c;
    tx_head = tx_head + 1u;
}

void serial_puts(const char *s)
{
    while (*s)
        serial_putc(*s++);
}

void serial_put_uint(uint32_t v)
{
    char b[11];
    int n = 0;
    do
    {
        b[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    while (n)
        serial_putc(b[--n]);
}

void serial_put_int(int32_t v)
{
    if (v < 0)
    {
        serial_putc('-');
        serial_put_uint((uint32_t)(-(int64_t)v));
    }
    else
        serial_put_uint((uint32_t)v);
}

void serial_flush(void)
{
    while (tx_tail != tx_head || tx_busy)
        serial_poll();
}
