/*
 * Выводы микроконтроллера на нашей плате (лист схемы 2_MCU).
 *
 *  Цепь        Вывод корпуса  Порт
 *  STEP1         1            P1.12
 *  STEP2         2            P1.11
 *  STEP3         3            P1.10
 *  STEP4         4            P1.9
 *  STEP5         7            P1.8
 *  STEP6        64            P1.13
 *  DIR1         21            P1.7
 *  DIR2         23            P1.5
 *  DIR3         54            P0.4
 *  DIR4         27            P1.2
 *  DIR5         28            P1.1
 *  DIR6         30            P1.0
 *  DRV_EN       26            P1.3   1 = драйверы включены (если не нажата аварийная кнопка)
 *  SEL_A0       25            P1.4   адрес дешифратора 74HC138
 *  SEL_A1       22            P1.6
 *  SEL_A2       55            P0.3
 *  SEL_EN       60            P1.15  1 = выбранный выход дешифратора активен
 *  SPI0_SCK     56            P0.2   тактовый сигнал 74HC165 и энкодеров
 *  SPI0_MISO    58            P0.0   данные от 74HC165 и энкодеров
 *  UART0_TX     51            P0.6   к CH340N
 *  UART0_RX     53            P0.5   от CH340N
 *  LED_RUN      31            P2.7
 *  LED_ERR      32            P2.6
 *  P0.11..P0.15 — JTAG, P2.0..P2.5 — внешняя флеш. Их программа не трогает.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifndef HOST_TEST
#include "mik32_memory_map.h"
#include "gpio.h"
#else
#include "host_mock.h"
#endif

#define NAXES 6
#define PIN(n) (1u << (n))

/* ---- STEP: все шесть на порту 1 ---- */
#define STEP_GPIO GPIO_1
#define STEP1_PIN PIN(12)
#define STEP2_PIN PIN(11)
#define STEP3_PIN PIN(10)
#define STEP4_PIN PIN(9)
#define STEP5_PIN PIN(8)
#define STEP6_PIN PIN(13)
#define STEP_ALL_PINS (STEP1_PIN | STEP2_PIN | STEP3_PIN | STEP4_PIN | STEP5_PIN | STEP6_PIN)

/* ---- DIR: DIR3 на порту 0, остальные на порту 1 ---- */
#define DIR1_P1_PIN PIN(7)
#define DIR2_P1_PIN PIN(5)
#define DIR3_P0_PIN PIN(4)
#define DIR4_P1_PIN PIN(2)
#define DIR5_P1_PIN PIN(1)
#define DIR6_P1_PIN PIN(0)
#define DIR_P1_PINS (DIR1_P1_PIN | DIR2_P1_PIN | DIR4_P1_PIN | DIR5_P1_PIN | DIR6_P1_PIN)
#define DIR_P0_PINS (DIR3_P0_PIN)

/* ---- Прочие выводы ---- */
#define DRV_EN_PIN PIN(3)  /* GPIO_1 */
#define SEL_A0_PIN PIN(4)  /* GPIO_1 */
#define SEL_A1_PIN PIN(6)  /* GPIO_1 */
#define SEL_A2_PIN PIN(3)  /* GPIO_0 */
#define SEL_EN_PIN PIN(15) /* GPIO_1 */
#define SCK_PIN PIN(2)     /* GPIO_0 */
#define MISO_PIN PIN(0)    /* GPIO_0 */
#define LED_RUN_PIN PIN(7) /* GPIO_2 */
#define LED_ERR_PIN PIN(6) /* GPIO_2 */

/* ---- Адреса дешифратора 74HC138 (лист 3_IO) ---- */
#define SEL_ADDR_ENC(k) (k) /* 0..5 -> CS энкодеров 1..6 */
#define SEL_ADDR_INPUTS 6   /* Y6 -> CS_IN: буфер 74LVC1G125 выдаёт Q7 регистра на MISO */
#define SEL_ADDR_LOAD 7     /* Y7 -> IN_LOAD: защёлкнуть входы в 74HC165 */

/* ---- Биты, которые читаются из 74HC165 ---- */
#define IN_LIM_MASK 0x3Fu      /* D0..D5 = LIM1..LIM6 */
#define IN_ESTOP_BIT (1u << 6) /* D6 = ESTOP_B: 1 = кнопка отпущена И DRV_EN = 1 */
#define IN_GND_BIT (1u << 7)   /* D7 = земля, всегда 0 */

/* Функции, которые должны выполняться из ОЗУ (обработчики прерываний) */
#ifndef HOST_TEST
/* no-tree-loop-distribute-patterns: не давать компилятору заменять циклы вызовом memcpy/memset,
 * которые лежат во внешней флеш */
#define RAM_FUNC __attribute__((section(".ram_text"), noinline, optimize("no-tree-loop-distribute-patterns")))
#else
#define RAM_FUNC
#endif

/* Запрет/разрешение прерываний на короткое время */
#ifndef HOST_TEST
static inline uint32_t irq_save(void)
{
    uint32_t m;
    __asm__ volatile("csrrci %0, mstatus, 8" : "=r"(m)::"memory");
    return m;
}
static inline void irq_restore(uint32_t m)
{
    if (m & 8u)
        __asm__ volatile("csrsi mstatus, 8" ::: "memory");
}
#else
static inline uint32_t irq_save(void) { return 0; }
static inline void irq_restore(uint32_t m) { (void)m; }
#endif

#define COMPILER_BARRIER() __asm__ volatile("" ::: "memory")

/* Для проверки логики на компьютере (без платы). На плате ничего не делает. */
#ifdef HOST_TEST
void host_idle(void);
#define HOST_IDLE() host_idle()
#else
#define HOST_IDLE() ((void)0)
#endif

#endif
