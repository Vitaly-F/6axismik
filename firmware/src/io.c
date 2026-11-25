#include "io.h"
#include "board.h"
#include "config.h"

#ifndef HOST_TEST
#include "mik32_hal_gpio.h"
#include "mik32_hal_pcc.h"
#endif

static bool drv_on = false;

static inline void short_delay(uint32_t n)
{
    for (volatile uint32_t i = 0; i < n; i++)
    {
    }
}

void io_init(void)
{
#ifndef HOST_TEST
    __HAL_PCC_GPIO_0_CLK_ENABLE();
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    __HAL_PCC_GPIO_2_CLK_ENABLE();
#endif

    /* Сначала задаём уровни, потом включаем выходы — чтобы не было ложных импульсов */
    GPIO_1->CLEAR = STEP_ALL_PINS | DIR_P1_PINS | DRV_EN_PIN | SEL_A0_PIN | SEL_A1_PIN | SEL_EN_PIN;
    GPIO_0->CLEAR = DIR_P0_PINS | SEL_A2_PIN;
    GPIO_0->SET = SCK_PIN; /* SCK в покое = 1 (так нужно энкодерам MT6701) */
    GPIO_2->CLEAR = LED_RUN_PIN | LED_ERR_PIN;

#ifndef HOST_TEST
    GPIO_InitTypeDef g = {0};

    g.Pin = (HAL_PinsTypeDef)(STEP_ALL_PINS | DIR_P1_PINS | DRV_EN_PIN | SEL_A0_PIN | SEL_A1_PIN | SEL_EN_PIN);
    g.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    g.Pull = HAL_GPIO_PULL_NONE;
    g.DS = HAL_GPIO_DS_4MA;
    HAL_GPIO_Init(GPIO_1, &g);

    g.Pin = (HAL_PinsTypeDef)(DIR_P0_PINS | SEL_A2_PIN | SCK_PIN);
    HAL_GPIO_Init(GPIO_0, &g);

    g.Pin = (HAL_PinsTypeDef)(LED_RUN_PIN | LED_ERR_PIN); /* только P2.6/P2.7: P2.0..P2.5 заняты флеш */
    g.DS = HAL_GPIO_DS_2MA;
    HAL_GPIO_Init(GPIO_2, &g);

    g.Pin = (HAL_PinsTypeDef)MISO_PIN;
    g.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    g.Pull = HAL_GPIO_PULL_UP; /* если никто не отвечает — читаем единицы */
    HAL_GPIO_Init(GPIO_0, &g);
#endif

    /* повторно, уже после настройки направления */
    GPIO_1->CLEAR = STEP_ALL_PINS | DRV_EN_PIN | SEL_EN_PIN;
    GPIO_0->SET = SCK_PIN;
    drv_on = false;
}

void drv_enable(bool on)
{
    if (on)
        GPIO_1->SET = DRV_EN_PIN;
    else
        GPIO_1->CLEAR = DRV_EN_PIN;
    drv_on = on;
}

bool drv_is_enabled(void)
{
    return drv_on;
}

void led_run(bool on)
{
    if (on)
        GPIO_2->SET = LED_RUN_PIN;
    else
        GPIO_2->CLEAR = LED_RUN_PIN;
}

void led_err(bool on)
{
    if (on)
        GPIO_2->SET = LED_ERR_PIN;
    else
        GPIO_2->CLEAR = LED_ERR_PIN;
}

/* ---------- Дешифратор 74HC138 ----------
 * Адрес меняем только при SEL_EN = 0, иначе на соседних выходах проскочат ложные импульсы. */
static void sel_on(uint32_t addr)
{
    GPIO_1->CLEAR = SEL_EN_PIN;
    if (addr & 1u) GPIO_1->SET = SEL_A0_PIN; else GPIO_1->CLEAR = SEL_A0_PIN;
    if (addr & 2u) GPIO_1->SET = SEL_A1_PIN; else GPIO_1->CLEAR = SEL_A1_PIN;
    if (addr & 4u) GPIO_0->SET = SEL_A2_PIN; else GPIO_0->CLEAR = SEL_A2_PIN;
    short_delay(1);
    GPIO_1->SET = SEL_EN_PIN;
}

static void sel_off(void)
{
    GPIO_1->CLEAR = SEL_EN_PIN;
}

static inline uint32_t miso(void)
{
    return (GPIO_0->STATE & MISO_PIN) ? 1u : 0u;
}

static inline void sck_low(void) { GPIO_0->CLEAR = SCK_PIN; }
static inline void sck_high(void) { GPIO_0->SET = SCK_PIN; }

/* ---------- 74HC165: концевики и аварийная кнопка ----------
 * 1. Адрес 7: на PL# регистра приходит 0 — входы защёлкиваются.
 * 2. Адрес 6: открывается буфер, на MISO появляется Q7 = D7.
 * 3. Каждый фронт SCK 0->1 сдвигает регистр: дальше идут D6, D5 ... D0. */
uint8_t io_read_inputs(void)
{
    uint32_t v = 0;

    sck_high();
    sel_on(SEL_ADDR_LOAD);
    short_delay(2);
    sel_off();

    sel_on(SEL_ADDR_INPUTS);
    short_delay(2);
    for (int i = 0; i < 8; i++)
    {
        v = (v << 1) | miso();
        sck_low();
        short_delay(1);
        sck_high();
        short_delay(1);
    }
    sel_off();
    return (uint8_t)v;
}

/* ---------- Энкодеры MT6701, интерфейс SSI ----------
 * CSN = 0 (через дешифратор), SCK в покое = 1, датчик меняет DO по фронту 0->1,
 * читаем после спада 1->0. Кадр: 1 пустой бит + 14 бит угла + 4 бита статуса + 6 бит CRC. */
static uint8_t crc6_43(uint32_t data18)
{
    /* CRC-6, полином x^6 + x + 1, старшим битом вперёд, начальное значение 0 */
    uint8_t crc = 0;
    for (int i = 17; i >= 0; i--)
    {
        uint8_t bit = (uint8_t)(((data18 >> i) & 1u) ^ ((crc >> 5) & 1u));
        crc = (uint8_t)((crc << 1) & 0x3Fu);
        if (bit)
            crc ^= 0x03u;
    }
    return crc;
}

static bool frame_ok(uint32_t frame24)
{
    return crc6_43(frame24 >> 6) == (frame24 & 0x3Fu);
}

bool io_read_encoder(int k, uint16_t *angle, uint8_t *status)
{
    uint32_t raw = 0;

    sck_high();
    sel_on(SEL_ADDR_ENC(k));
    short_delay(8); /* от спада CSN до первого такта нужно >= 100 нс */
    for (int i = 0; i < 25; i++)
    {
        sck_low();
        short_delay(4);
        raw = (raw << 1) | miso();
        sck_high();
        short_delay(4);
    }
    sel_off();

    /* Все единицы — датчика нет: линия MISO просто подтянута к питанию */
    if (raw == 0x1FFFFFFu)
        return false;

    /* Обычно первый бит пустой. На всякий случай проверяем и сдвиг на один бит. */
    uint32_t frame = raw & 0xFFFFFFu;
#if ENCODER_CHECK_CRC
    if (!frame_ok(frame))
    {
        uint32_t alt = raw >> 1;
        if (!frame_ok(alt))
            return false;
        frame = alt;
    }
#endif
    *angle = (uint16_t)((frame >> 10) & 0x3FFFu);
    *status = (uint8_t)((frame >> 6) & 0x0Fu);
    return true;
}
