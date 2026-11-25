#include "commands.h"
#include "config.h"
#include "board.h"
#include "io.h"
#include "machine.h"
#include "serial.h"
#include "stepper.h"

#include <stdint.h>
#include <stdbool.h>

#define LINE_MAX 120

static char line[LINE_MAX];
static uint32_t line_len;
static bool line_too_long;

/* Команда M, которая ждёт свободного места в очереди */
static bool pending;
static int32_t pend_delta[NAXES];
static uint32_t pend_ticks;

static void reply_ok(void) { serial_puts("ok\r\n"); }

static void reply_err(const char *msg)
{
    serial_puts("error: ");
    serial_puts(msg);
    serial_puts("\r\n");
}

/* ---------- разбор чисел ---------- */
static const char *skip_sep(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == ',')
        p++;
    return p;
}

static bool parse_int(const char **pp, int32_t *out)
{
    const char *p = skip_sep(*pp);
    bool neg = false;
    if (*p == '-' || *p == '+')
    {
        neg = (*p == '-');
        p++;
    }
    if (*p < '0' || *p > '9')
        return false;
    uint32_t v = 0;
    while (*p >= '0' && *p <= '9')
    {
        uint32_t d = (uint32_t)(*p - '0');
        if (v > (0x7FFFFFFFu - d) / 10u)
            return false; /* слишком большое число */
        v = v * 10u + d;
        p++;
    }
    *out = neg ? -(int32_t)v : (int32_t)v;
    *pp = p;
    return true;
}

static bool at_end(const char *p)
{
    p = skip_sep(p);
    return *p == '\0';
}

/* ---------- команды ---------- */
static void cmd_help(void)
{
    serial_puts(FW_VERSION "\r\n"
                "M d1 d2 d3 d4 d5 d6 t  move: steps per axis (+/-), duration t in ms\r\n"
                "E   enable drivers        D   disable drivers (stops motion)\r\n"
                "X   reset alarm           Z   zero all positions\r\n"
                "P p1 p2 p3 p4 p5 p6       set positions\r\n"
                "R   read encoders         H   this help\r\n"
                "?   status (instant)      !   stop now, clear queue (instant)\r\n");
}

static void cmd_move(const char *p)
{
    int32_t d[NAXES];
    int32_t t_ms;
    for (int i = 0; i < NAXES; i++)
    {
        if (!parse_int(&p, &d[i]))
        {
            reply_err("M needs 6 step counts and time in ms");
            return;
        }
    }
    if (!parse_int(&p, &t_ms) || !at_end(p) || t_ms < 0 || t_ms > 600000)
    {
        reply_err("M needs time in ms (0..600000) at the end");
        return;
    }
    if (machine_alarm() != ALARM_NONE)
    {
        reply_err("alarm active, send X first");
        return;
    }
    if (!drv_is_enabled())
    {
        reply_err("drivers are off, send E first");
        return;
    }
    uint32_t ticks = (uint32_t)t_ms * (STEP_TICK_HZ / 1000u);
    if (rt_halt_req || !stepper_push(d, ticks))
    {
        /* Очередь заполнена — ответим "ok", когда освободится место */
        for (int i = 0; i < NAXES; i++)
            pend_delta[i] = d[i];
        pend_ticks = ticks;
        pending = true;
        return;
    }
    reply_ok();
}

static void cmd_set_pos(const char *p)
{
    int32_t v[NAXES];
    for (int i = 0; i < NAXES; i++)
    {
        if (!parse_int(&p, &v[i]))
        {
            reply_err("P needs 6 positions");
            return;
        }
    }
    if (stepper_busy())
    {
        reply_err("busy, wait until motion ends");
        return;
    }
    stepper_set_pos(v);
    reply_ok();
}

static void cmd_zero(void)
{
    int32_t v[NAXES] = {0};
    if (stepper_busy())
    {
        reply_err("busy, wait until motion ends");
        return;
    }
    stepper_set_pos(v);
    reply_ok();
}

static void cmd_encoders(void)
{
    serial_puts("ENC:");
    for (int k = 0; k < NAXES; k++)
    {
        uint16_t a;
        uint8_t st;
        serial_putc(' ');
        if (io_read_encoder(k, &a, &st))
        {
            serial_put_uint(a);
            if (st & 0x3u) /* слишком сильное/слабое поле магнита */
                serial_puts("(mag)");
        }
        else
            serial_putc('-');
    }
    serial_puts("\r\n");
    reply_ok();
}

static void execute(char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    if (*s == '\0')
        return; /* пустая строка — без ответа */

    char c = *s;
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 'a' + 'A');
    const char *args = s + 1;

    switch (c)
    {
    case 'M':
        cmd_move(args);
        break;
    case 'E':
    {
        const char *why = "";
        if (machine_enable(&why))
            reply_ok();
        else
            reply_err(why);
        break;
    }
    case 'D':
        machine_disable();
        reply_ok();
        break;
    case 'X':
        machine_clear_alarm();
        reply_ok();
        break;
    case 'Z':
        cmd_zero();
        break;
    case 'P':
        cmd_set_pos(args);
        break;
    case 'R':
        cmd_encoders();
        break;
    case 'H':
        cmd_help();
        reply_ok();
        break;
    default:
        reply_err("unknown command, H for help");
        break;
    }
}

void commands_init(void)
{
    line_len = 0;
    line_too_long = false;
    pending = false;
}

void commands_poll(void)
{
    /* Мгновенные команды, пойманные прерыванием приёма */
    if (rt_halt_req)
    {
        rt_halt_req = 0;
        stepper_halt();
        if (pending)
        {
            pending = false;
            reply_err("halted");
        }
        serial_puts("HALT\r\n");
    }
    if (rt_status_req)
    {
        rt_status_req = 0;
        machine_print_status();
    }
    if (rx_overflow)
    {
        rx_overflow = 0;
        serial_puts("WARNING: input overflow, some characters lost\r\n");
    }

    /* Команда M ждёт места в очереди — новые строки пока не читаем */
    if (pending)
    {
        if (machine_alarm() != ALARM_NONE)
        {
            pending = false;
            reply_err("alarm active, send X first");
        }
        else if (!drv_is_enabled())
        {
            pending = false;
            reply_err("drivers are off, send E first");
        }
        else if (stepper_push(pend_delta, pend_ticks))
        {
            pending = false;
            reply_ok();
        }
        return;
    }

    int ch;
    while ((ch = serial_getc()) >= 0)
    {
        if (ch == '\r')
            continue;
        if (ch == '\n')
        {
            line[line_len] = '\0';
            if (line_too_long)
                reply_err("line too long");
            else
                execute(line);
            line_len = 0;
            line_too_long = false;
            return; /* одна строка за проход цикла — остальное в следующий раз */
        }
        if (line_len < LINE_MAX - 1)
            line[line_len++] = (char)ch;
        else
            line_too_long = true;
    }
}
