/*
 * Состояние станка: включены ли драйверы, есть ли авария,
 * опрос концевиков и аварийной кнопки, светодиоды.
 */
#ifndef MACHINE_H
#define MACHINE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    ALARM_NONE = 0,
    ALARM_LIMIT,  /* сработал концевик во время движения */
    ALARM_ESTOP,  /* нажата аварийная кнопка */
    ALARM_INPUTS, /* не читаются входы (74HC165 / буфер) */
} alarm_t;

void machine_init(void);
void machine_poll(void); /* вызывать в основном цикле */

alarm_t machine_alarm(void);
const char *machine_alarm_name(void);
void machine_clear_alarm(void);

/* Включить драйверы. Возвращает false, если нажата аварийная кнопка или есть авария. */
bool machine_enable(const char **why);
void machine_disable(void);

uint8_t machine_limits(void);  /* какие концевики сейчас нажаты (бит 0 = ось 1) */
int machine_estop_state(void); /* 1 = кнопка отпущена, 0 = нажата, -1 = неизвестно (драйверы выключены) */

void machine_print_status(void);

#endif
