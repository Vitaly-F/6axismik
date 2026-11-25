/*
 * Прошивка платы управления 6 шаговыми двигателями на MIK32 АМУР.
 *
 * Что делает:
 *  - принимает от компьютера отрезки движения (команда M) и выдаёт STEP/DIR на 6 драйверов;
 *  - следит за концевиками и аварийной кнопкой, при срабатывании останавливает движение;
 *  - читает магнитные энкодеры MT6701 (если подключены);
 *  - отвечает на запросы состояния.
 * Кинематику манипулятора (углы суставов, траектории) считает компьютер.
 */
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_spifi.h"
#include "timer32.h"
#include "epic.h"

#include "config.h"
#include "board.h"
#include "io.h"
#include "serial.h"
#include "stepper.h"
#include "machine.h"
#include "commands.h"

#if (STEP_TICK_HZ % 1000) != 0
#error "STEP_TICK_HZ must be a multiple of 1000"
#endif

/* Тактирование: ВНУТРЕННИЙ генератор HSI32M (32 МГц). Внешний кварц ZQ1 не используется
 * и может быть не запаян. Кварц 32,768 кГц (ZQ2) тоже не нужен: служебные 32 кГц — от
 * внутреннего LSI32K.
 * Если в мониторе пойдут "кракозябры" (частота HSI ушла), подстройте HSI32MCalibrationValue
 * (0..255, по умолчанию 128) или временно снизьте UART_BAUD в config.h. */
static void SystemClock_Config(void)
{
    PCC_InitTypeDef pcc = {0};
    pcc.OscillatorEnable = PCC_OSCILLATORTYPE_HSI32M | PCC_OSCILLATORTYPE_LSI32K;
    pcc.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_HSI32M;
    pcc.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_FIXED;
    pcc.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_LSI32K;
    pcc.AHBDivider = 0;
    pcc.APBMDivider = 0;
    pcc.APBPDivider = 0;
    pcc.HSI32MCalibrationValue = 200; /* ПРОБА: было 128. При 128 частота была ≈ 35,1 МГц (+9,8 %) */
    pcc.LSI32KCalibrationValue = 8;
    pcc.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_LSI32K;
    pcc.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_LSI32K;
    HAL_PCC_Config(&pcc);
}

#if ENABLE_SPIFI_CACHE
/* Включаем кэш внешней флеш (по примеру Микрона HAL_Blink_SPIFI).
 * Функция целиком в ОЗУ: пока контроллер флеш перенастраивается, читать код из флеш нельзя. */
RAM_FUNC static void spifi_cache_enable(void)
{
    const uint32_t cmd_read =
        SPIFI_DIRECTION_INPUT |
        SPIFI_CONFIG_CMD_INTLEN(0) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE_3ADDR) |
        SPIFI_CONFIG_CMD_OPCODE(0x03);

    SPIFI_CONFIG->STAT = SPIFI_CONFIG_STAT_RESET_M;
    for (volatile uint32_t t = 0; (SPIFI_CONFIG->STAT & SPIFI_CONFIG_STAT_RESET_M) && t < 100000u; t++)
    {
    }
    SPIFI_CONFIG->CLIMIT = 0x90000000;
    SPIFI_CONFIG->CTRL = 0x400FFFFF | SPIFI_CONFIG_CTRL_CACHE_EN_M;
    SPIFI_CONFIG->MCMD = cmd_read;
}
#endif

/* Единый обработчик прерываний и исключений (вызывается из crt0). Выполняется из ОЗУ. */
RAM_FUNC void trap_handler(void)
{
    uint32_t cause;
    __asm__ volatile("csrr %0, mcause" : "=r"(cause));
    if ((int32_t)cause >= 0)
    {
        /* Исключение (ошибка в программе): выключаем драйверы, зажигаем ERR и стоим */
        GPIO_1->CLEAR = DRV_EN_PIN | STEP_ALL_PINS;
        GPIO_2->SET = LED_ERR_PIN;
        for (;;)
        {
        }
    }

    uint32_t st = EPIC->RAW_STATUS;
    if (st & (1u << EPIC_LINE_UART_0_S))
    {
        serial_isr();
        EPIC->CLEAR = 1u << EPIC_LINE_UART_0_S;
    }
    if (st & (1u << EPIC_LINE_TIMER32_1_S))
    {
        TIMER32_1->INT_CLEAR = 0x3FFu;
        stepper_isr();
        EPIC->CLEAR = 1u << EPIC_LINE_TIMER32_1_S;
    }
}

int main(void)
{
    SystemClock_Config();
#if ENABLE_SPIFI_CACHE
    spifi_cache_enable();
#endif
    HAL_Init(); /* тактирование PAD_CONFIG и контроллера прерываний EPIC */

    io_init();
    serial_init();
    stepper_init();
    machine_init();
    commands_init();

    HAL_EPIC_MaskLevelSet(HAL_EPIC_TIMER32_1_MASK | HAL_EPIC_UART_0_MASK);
    HAL_IRQ_EnableInterrupts();

    serial_puts("\r\n" FW_VERSION " ready. Send H for help.\r\n");

    for (;;)
    {
        serial_poll();
        commands_poll();
        machine_poll();
    }
}
