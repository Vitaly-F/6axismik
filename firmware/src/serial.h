/*
 * Связь с компьютером: UART0 -> CH340N -> USB-C.
 * Приём — по прерыванию в кольцевой буфер, передача — из буфера в основном цикле,
 * чтобы основной цикл никогда не ждал медленный порт.
 *
 * Символы '!' и '?' обрабатываются сразу при приёме (как в Grbl):
 *   '!' — немедленная остановка, '?' — запрос состояния.
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>

void serial_init(void);
void serial_poll(void);          /* вызывать в основном цикле */
int serial_getc(void);           /* -1, если ничего не пришло */
void serial_putc(char c);
void serial_puts(const char *s);
void serial_put_int(int32_t v);
void serial_put_uint(uint32_t v);
void serial_flush(void);         /* дождаться отправки всего буфера */

extern volatile uint8_t rt_status_req; /* пришёл '?' */
extern volatile uint8_t rt_halt_req;   /* пришёл '!' */
extern volatile uint8_t rx_overflow;   /* потеряны принятые байты */

void serial_isr(void); /* вызывается из обработчика прерываний */

#endif
