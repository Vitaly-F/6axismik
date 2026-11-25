/*
 * Периферия платы: разрешение драйверов, светодиоды,
 * чтение концевиков/аварийной кнопки (74HC165) и энкодеров MT6701.
 */
#ifndef IO_H
#define IO_H

#include <stdint.h>
#include <stdbool.h>

void io_init(void);

void drv_enable(bool on); /* DRV_EN: 1 = разрешить драйверы */
bool drv_is_enabled(void);

void led_run(bool on);
void led_err(bool on);

/* Прочитать 8 входов 74HC165: биты 0..5 = LIM1..LIM6, бит 6 = ESTOP_B, бит 7 = земля */
uint8_t io_read_inputs(void);

/* Прочитать энкодер k (0..5).
 * Возвращает true, если данные корректны (CRC сошлась), angle = 0..16383 (14 бит на оборот). */
bool io_read_encoder(int k, uint16_t *angle, uint8_t *status);

#endif
