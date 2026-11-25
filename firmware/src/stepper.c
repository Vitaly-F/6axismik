#include "stepper.h"
#include "config.h"

#ifndef HOST_TEST
#include "mik32_hal_timer32.h"
#endif

#if (QUEUE_LEN & (QUEUE_LEN - 1)) != 0
#error "QUEUE_LEN must be a power of two"
#endif

/* Один отрезок движения. Маски для выводов DIR считаются заранее,
 * чтобы прерывание делало как можно меньше работы. */
typedef struct
{
    uint32_t steps[NAXES]; /* модуль числа шагов по каждой оси */
    uint32_t ticks;        /* длительность отрезка в тиках */
    uint32_t neg_mask;     /* бит i = 1: ось i едет в минус */
    uint32_t dir0_set, dir0_clr; /* что выставить/сбросить на порту 0 */
    uint32_t dir1_set, dir1_clr; /* что выставить/сбросить на порту 1 */
} segment_t;

static segment_t queue[QUEUE_LEN];
static volatile uint32_t q_head; /* куда пишет основная программа */
static volatile uint32_t q_tail; /* откуда читает прерывание */

/* Состояние текущего отрезка (меняется только в прерывании) */
static uint32_t c_steps[NAXES];
static uint32_t c_acc[NAXES];
static int32_t c_inc[NAXES];
static uint32_t c_ticks;
static volatile uint32_t c_left;     /* сколько тиков осталось в текущем отрезке */
static volatile uint32_t pend_step;  /* какие STEP поднять в начале следующего тика */

static volatile int32_t pos[NAXES];
static volatile uint32_t tick_count;
volatile uint32_t st_halt_req;

/* Таблица выводов STEP. volatile — чтобы компилятор не перенёс её во внешнюю флеш
 * (как константу): прерывание обращается к ней на каждом тике, она должна лежать в ОЗУ. */
static volatile uint32_t step_pin[NAXES] = {STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN, STEP5_PIN, STEP6_PIN};

/* Где находится DIR каждой оси: порт 0 или порт 1 */
static const uint32_t dir_p0[NAXES] = {0, 0, DIR3_P0_PIN, 0, 0, 0};
static const uint32_t dir_p1[NAXES] = {DIR1_P1_PIN, DIR2_P1_PIN, 0, DIR4_P1_PIN, DIR5_P1_PIN, DIR6_P1_PIN};

#ifndef HOST_TEST
static TIMER32_HandleTypeDef htim;
#endif

void stepper_init(void)
{
    q_head = q_tail = 0;
    c_left = 0;
    pend_step = 0;
    st_halt_req = 0;
    for (int i = 0; i < NAXES; i++)
        pos[i] = 0;

#ifndef HOST_TEST
    /* TIMER32_1 считает такты шины (32 МГц) и переполняется STEP_TICK_HZ раз в секунду */
    htim.Instance = TIMER32_1;
    htim.Top = (uint32_t)(OSC_SYSTEM_VALUE / STEP_TICK_HZ) - 1u;
    htim.State = TIMER32_STATE_DISABLE;
    htim.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htim.Clock.Prescaler = 0;
    htim.InterruptMask = 0;
    htim.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htim);
    HAL_Timer32_Value_Clear(&htim);
    HAL_Timer32_Base_Start_IT(&htim); /* прерывание по переполнению */
#endif
}

/*
 * Один тик генератора шагов. Выполняется из ОЗУ.
 *
 * 1. Сразу поднимаем STEP тех осей, шаг которых был рассчитан на прошлом тике —
 *    так фронты идут ровно с периодом таймера, без дрожания.
 * 2. Считаем шаги для следующего тика (пока STEP поднят — это и есть ширина импульса).
 * 3. Опускаем STEP.
 * 4. Если начался новый отрезок — меняем DIR (после опускания STEP, до следующего фронта
 *    остаётся почти целый тик — с запасом для любого драйвера).
 */
RAM_FUNC void stepper_isr(void)
{
    uint32_t out = pend_step;
    if (out)
        STEP_GPIO->SET = out;
    tick_count++;

    uint32_t next = 0;
    uint32_t new_dir = 0, d0s = 0, d0c = 0, d1s = 0, d1c = 0;

    if (st_halt_req)
    {
        /* Остановка: выбрасываем текущий отрезок и всю очередь */
        c_left = 0;
        q_tail = q_head;
        st_halt_req = 0;
    }
    else
    {
        uint32_t left = c_left;
        if (left == 0 && q_tail != q_head)
        {
            /* Берём следующий отрезок из очереди */
            segment_t *s = &queue[q_tail & (QUEUE_LEN - 1u)];
            uint32_t n = s->ticks;
            uint32_t neg = s->neg_mask;
            c_ticks = n;
            left = n;
            for (int i = 0; i < NAXES; i++)
            {
                c_steps[i] = s->steps[i];
                c_acc[i] = n >> 1; /* начинаем с середины — шаги распределяются симметрично */
                c_inc[i] = (neg & (1u << i)) ? -1 : 1;
            }
            d0s = s->dir0_set;
            d0c = s->dir0_clr;
            d1s = s->dir1_set;
            d1c = s->dir1_clr;
            new_dir = 1;
            q_tail = q_tail + 1u;
        }
        if (left)
        {
            uint32_t n = c_ticks;
            for (int i = 0; i < NAXES; i++)
            {
                uint32_t a = c_acc[i] + c_steps[i];
                if (a >= n)
                {
                    a -= n;
                    next |= step_pin[i];
                    pos[i] += c_inc[i];
                }
                c_acc[i] = a;
            }
            left--;
        }
        c_left = left;
    }
    pend_step = next;

    if (out)
    {
        /* Гарантируем минимальную ширину импульса STEP */
        for (volatile uint32_t k = 0; k < STEP_PULSE_LOOPS; k++)
        {
        }
        STEP_GPIO->CLEAR = out;
    }
    if (new_dir)
    {
        GPIO_0->SET = d0s;
        GPIO_0->CLEAR = d0c;
        GPIO_1->SET = d1s;
        GPIO_1->CLEAR = d1c;
    }
}

bool stepper_push(const int32_t delta[NAXES], uint32_t ticks)
{
    if ((uint32_t)(q_head - q_tail) >= QUEUE_LEN)
        return false;

    segment_t *s = &queue[q_head & (QUEUE_LEN - 1u)];
    uint32_t maxs = 0, neg = 0;
    uint32_t d0s = 0, d0c = 0, d1s = 0, d1c = 0;

    for (int i = 0; i < NAXES; i++)
    {
        int32_t d = delta[i];
        uint32_t a = (d < 0) ? (uint32_t)(-(int64_t)d) : (uint32_t)d;
        s->steps[i] = a;
        if (a > maxs)
            maxs = a;
        if (a == 0)
            continue; /* ось стоит — её DIR не трогаем */
        bool minus = (d < 0);
        if (minus)
            neg |= 1u << i;
        /* По умолчанию DIR = 1 — движение в плюс. DIR_INVERT_MASK меняет это для оси. */
        bool inverted = ((DIR_INVERT_MASK >> i) & 1u) != 0;
        bool level = !(minus ^ inverted);
        if (level)
        {
            d0s |= dir_p0[i];
            d1s |= dir_p1[i];
        }
        else
        {
            d0c |= dir_p0[i];
            d1c |= dir_p1[i];
        }
    }

    /* Ось не может делать больше одного шага за тик: при необходимости растягиваем отрезок */
    if (ticks < maxs)
        ticks = maxs;
    if (ticks == 0)
        return true; /* пустой отрезок — делать нечего */
    if (ticks > 0x7FFFFFFFu)
        ticks = 0x7FFFFFFFu;

    s->ticks = ticks;
    s->neg_mask = neg;
    s->dir0_set = d0s;
    s->dir0_clr = d0c;
    s->dir1_set = d1s;
    s->dir1_clr = d1c;

    COMPILER_BARRIER(); /* сначала полностью записываем отрезок, потом публикуем его */
    q_head = q_head + 1u;
    return true;
}

uint32_t stepper_queue_count(void)
{
    return (uint32_t)(q_head - q_tail);
}

bool stepper_busy(void)
{
    return (q_head != q_tail) || (c_left != 0) || (pend_step != 0);
}

void stepper_halt(void)
{
    st_halt_req = 1;
    /* Прерывание подтвердит остановку на ближайшем тике (через 40 мкс) */
    for (volatile uint32_t i = 0; st_halt_req && i < 200000u; i++)
    {
        HOST_IDLE();
    }
    st_halt_req = 0;
}

void stepper_get_pos(int32_t out[NAXES])
{
    uint32_t m = irq_save();
    for (int i = 0; i < NAXES; i++)
        out[i] = pos[i];
    irq_restore(m);
}

void stepper_set_pos(const int32_t in[NAXES])
{
    uint32_t m = irq_save();
    for (int i = 0; i < NAXES; i++)
        pos[i] = in[i];
    irq_restore(m);
}

uint32_t stepper_ticks(void)
{
    return tick_count;
}
