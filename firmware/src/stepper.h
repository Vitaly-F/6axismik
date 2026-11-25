/*
 * Генератор шагов для 6 осей.
 *
 * Компьютер присылает отрезки движения: "ось i должна сделать d[i] шагов за N тиков".
 * Отрезки складываются в очередь. Таймер с частотой STEP_TICK_HZ на каждом тике
 * решает, какие оси делают шаг (алгоритм Брезенхема/DDA), поэтому все оси
 * начинают и заканчивают отрезок одновременно и движутся равномерно.
 */
#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

void stepper_init(void);

/* Положить отрезок в очередь. false — очередь заполнена (повторите позже). */
bool stepper_push(const int32_t delta[NAXES], uint32_t ticks);

uint32_t stepper_queue_count(void); /* сколько отрезков ждёт в очереди */
bool stepper_busy(void);            /* идёт движение или очередь не пуста */
void stepper_halt(void);            /* немедленно остановить всё и очистить очередь */

void stepper_get_pos(int32_t pos[NAXES]);
void stepper_set_pos(const int32_t pos[NAXES]);

/* Счётчик тиков генератора (STEP_TICK_HZ в секунду) — используется как часы */
uint32_t stepper_ticks(void);

/* Флаг "стоп", может выставляться прямо из прерывания приёма UART */
extern volatile uint32_t st_halt_req;

/* Вызывается из обработчика прерываний на каждом тике таймера */
void stepper_isr(void);

#endif
