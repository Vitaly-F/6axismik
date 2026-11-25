#include "machine.h"
#include "config.h"
#include "board.h"
#include "io.h"
#include "serial.h"
#include "stepper.h"

static alarm_t alarm_state;
static int alarm_axis;

static uint8_t lim_stable;  /* отфильтрованное состояние концевиков (1 = нажат) */
static uint8_t lim_prev;    /* прошлое сырое чтение — для фильтра */
static uint8_t lim_ignore;  /* концевики, на которых стоим после сброса аварии (X) */
static uint8_t estop_low_cnt;
static uint8_t chain_bad_cnt;
static uint8_t last_raw;
static uint32_t last_poll_tick;

#define TICKS_PER_MS (STEP_TICK_HZ / 1000u)

static void raise_alarm(alarm_t a, int axis)
{
    if (alarm_state != ALARM_NONE)
        return;
    alarm_state = a;
    alarm_axis = axis;
    stepper_halt();
    if (a == ALARM_ESTOP)
        drv_enable(false); /* защёлкиваем: после отпускания кнопки драйверы сами не включатся */
    led_err(true);
    serial_puts("ALARM: ");
    serial_puts(machine_alarm_name());
    if (a == ALARM_LIMIT)
    {
        serial_putc(' ');
        serial_put_int(axis);
    }
    serial_puts("\r\n");
}

/* Сырые биты концевиков -> "нажат" = 1 с учётом полярности */
static uint8_t raw_to_pressed(uint8_t raw)
{
    return (uint8_t)(((raw & IN_LIM_MASK) ^ LIMIT_ACTIVE_LOW_MASK) & IN_LIM_MASK);
}

static void poll_inputs(void)
{
    uint8_t raw = io_read_inputs();
    last_raw = raw;

    /* 1. Исправность цепочки входов */
#if CHECK_INPUT_CHAIN
    if (raw & IN_GND_BIT)
    {
        if (chain_bad_cnt < 255)
            chain_bad_cnt++;
        if (chain_bad_cnt >= 3)
            raise_alarm(ALARM_INPUTS, 0);
        return; /* остальные биты в такой ситуации доверия не вызывают */
    }
    chain_bad_cnt = 0;
#endif

    /* 2. Аварийная кнопка. ESTOP_B = 1 только если DRV_EN = 1 и кнопка не нажата,
     *    поэтому смотрим на неё только при включённых драйверах. */
    if (drv_is_enabled())
    {
        if (!(raw & IN_ESTOP_BIT))
        {
            if (++estop_low_cnt >= 2)
                raise_alarm(ALARM_ESTOP, 0);
        }
        else
            estop_low_cnt = 0;
    }
    else
        estop_low_cnt = 0;

    /* 3. Концевики: два одинаковых чтения подряд = устойчивое состояние */
    uint8_t now = raw_to_pressed(raw);
    if (now == lim_prev)
        lim_stable = now;
    lim_prev = now;

    lim_ignore &= lim_stable; /* отпущенный концевик снова охраняется */
    uint8_t active = (uint8_t)(lim_stable & LIMIT_ENABLE_MASK & (uint8_t)~lim_ignore);
    if (active && stepper_busy())
    {
        int axis = 1;
        while (!(active & 1u))
        {
            active >>= 1;
            axis++;
        }
        raise_alarm(ALARM_LIMIT, axis);
    }
}

static void update_leds(void)
{
    uint32_t ms = stepper_ticks() / TICKS_PER_MS;
    bool run;
    if (stepper_busy())
        run = true;                       /* движение — горит постоянно */
    else if (drv_is_enabled())
        run = (ms % 1000u) < 500u;        /* драйверы включены, стоим — мигает 1 раз в секунду */
    else
        run = (ms % 2000u) < 100u;        /* драйверы выключены — короткая вспышка раз в 2 с */
    led_run(run);
    led_err(alarm_state != ALARM_NONE);
}

void machine_init(void)
{
    alarm_state = ALARM_NONE;
    alarm_axis = 0;
    lim_stable = lim_prev = lim_ignore = 0;
    estop_low_cnt = chain_bad_cnt = 0;
    last_poll_tick = stepper_ticks();
    drv_enable(false);
}

void machine_poll(void)
{
    uint32_t now = stepper_ticks();
    if ((uint32_t)(now - last_poll_tick) >= INPUT_POLL_MS * TICKS_PER_MS)
    {
        last_poll_tick = now;
        poll_inputs();
        update_leds();
    }
}

alarm_t machine_alarm(void)
{
    return alarm_state;
}

const char *machine_alarm_name(void)
{
    switch (alarm_state)
    {
    case ALARM_LIMIT:
        return "LIMIT";
    case ALARM_ESTOP:
        return "ESTOP";
    case ALARM_INPUTS:
        return "INPUTS";
    default:
        return "NONE";
    }
}

void machine_clear_alarm(void)
{
    alarm_state = ALARM_NONE;
    alarm_axis = 0;
    estop_low_cnt = 0;
    chain_bad_cnt = 0;
    /* Концевики, нажатые прямо сейчас, перестают охраняться, пока их не отпустят:
     * так можно съехать с концевика после остановки. */
    lim_ignore = lim_stable;
    led_err(false);
}

static void wait_ms(uint32_t ms)
{
    uint32_t t0 = stepper_ticks();
    while ((uint32_t)(stepper_ticks() - t0) < ms * TICKS_PER_MS)
    {
        serial_poll();
        HOST_IDLE();
    }
}

bool machine_enable(const char **why)
{
    if (alarm_state != ALARM_NONE)
    {
        *why = "alarm active, send X first";
        return false;
    }
    drv_enable(true);
    wait_ms(3); /* RC-цепочка затвора (1 кОм * 100 нФ) и фильтр — с запасом */
    uint8_t raw = io_read_inputs();
#if CHECK_INPUT_CHAIN
    if (raw & IN_GND_BIT)
    {
        drv_enable(false);
        *why = "inputs not readable (74HC165)";
        return false;
    }
#endif
    if (!(raw & IN_ESTOP_BIT))
    {
        drv_enable(false);
        *why = "E-stop pressed or not connected";
        return false;
    }
    estop_low_cnt = 0;
    return true;
}

void machine_disable(void)
{
    stepper_halt();
    drv_enable(false);
}

uint8_t machine_limits(void)
{
    return lim_stable;
}

int machine_estop_state(void)
{
    if (!drv_is_enabled())
        return -1;
    return (last_raw & IN_ESTOP_BIT) ? 1 : 0;
}

void machine_print_status(void)
{
    int32_t p[NAXES];
    stepper_get_pos(p);

    if (alarm_state != ALARM_NONE)
        serial_puts("<Alarm");
    else if (stepper_busy())
        serial_puts("<Run");
    else if (drv_is_enabled())
        serial_puts("<Idle");
    else
        serial_puts("<Off");

    serial_puts("|POS:");
    for (int i = 0; i < NAXES; i++)
    {
        if (i)
            serial_putc(',');
        serial_put_int(p[i]);
    }
    serial_puts("|Q:");
    serial_put_uint(stepper_queue_count());
    serial_puts("|LIM:");
    uint8_t l = lim_stable;
    for (int i = 0; i < NAXES; i++)
        serial_putc((l >> i) & 1u ? '1' : '0');
    serial_puts("|EN:");
    serial_putc(drv_is_enabled() ? '1' : '0');
    if (alarm_state != ALARM_NONE)
    {
        serial_puts("|ALARM:");
        serial_puts(machine_alarm_name());
        if (alarm_state == ALARM_LIMIT)
        {
            serial_putc(' ');
            serial_put_int(alarm_axis);
        }
    }
    serial_puts(">\r\n");
}
