#include "r08_avoidance.h"
#include "stm32f1xx.h"

enum {
    TRIG_PULSE_US = 12,
    SLOT_US = 45000,
    RISE_TIMEOUT_US = 3000,
    ECHO_TIMEOUT_US = 42500,
    MAX_RANGE_MM = 4000,
    FIRST_PING_DELAY_MS = 80,
    ECHO_LINES = (1u << 13) | (1u << 15) | (1u << 1)
};

typedef struct { uint8_t trig_pin; uint8_t echo_pin; } SonarPins;
typedef enum { SONAR_IDLE, SONAR_RISE, SONAR_FALL, SONAR_DONE } SonarPhase;

static const SonarPins sonar_pins[CAR_SENSOR_COUNT] = {
    [CAR_LEFT] = {12, 13}, [CAR_FRONT] = {14, 15}, [CAR_RIGHT] = {0, 1}
};
static const CarSensor ping_order[] = {CAR_FRONT, CAR_LEFT, CAR_FRONT, CAR_RIGHT};
static CarDrive drive;
static uint32_t cycles_per_us;
static uint32_t init_ms;
static uint8_t next_slot;
static volatile bool enabled;
static volatile uint32_t trigger_at;
static volatile CarSensor active_sensor;
static volatile SonarPhase phase;
static volatile uint32_t rise_at;
static volatile uint32_t echo_cycles;

static void gpio_mode(GPIO_TypeDef *port, unsigned pin, uint32_t mode)
{
    volatile uint32_t *const conf = pin < 8u ? &port->CRL : &port->CRH;
    const unsigned shift = (pin % 8u) * 4u;
    *conf = (*conf & ~(0xFu << shift)) | (mode << shift);
}

bool r08_avoidance_init(const CarConfig *car_config, const CarDriveConfig *drive_config,
                         uint32_t now_ms)
{
    enabled = false;
    car_drive_init(&drive, car_config, drive_config, now_ms);
    cycles_per_us = SystemCoreClock / 1000000u;
    if (cycles_per_us == 0u) {
        return false;
    }

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    (void)RCC->APB2ENR;
    EXTI->IMR &= ~(uint32_t)ECHO_LINES;
    for (unsigned i = 0; i < CAR_SENSOR_COUNT; ++i) {
        const unsigned trig = sonar_pins[i].trig_pin;
        const unsigned echo = sonar_pins[i].echo_pin;
        GPIOB->BRR = (1u << trig) | (1u << echo);
        gpio_mode(GPIOB, trig, 0x2u);  /* 2 MHz push-pull. */
        gpio_mode(GPIOB, echo, 0x8u);  /* Pull-down via ODR. */
        const unsigned shift = (echo % 4u) * 4u;
        AFIO->EXTICR[echo / 4u] = (AFIO->EXTICR[echo / 4u] & ~(0xFu << shift)) |
                                  (1u << shift);  /* Port B. */
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t start = DWT->CYCCNT;
    __NOP();
    __NOP();
    if (DWT->CYCCNT == start) {
        return false;
    }

    phase = SONAR_IDLE;
    next_slot = 0;
    init_ms = now_ms;
    trigger_at = DWT->CYCCNT - SLOT_US * cycles_per_us;
    EXTI->RTSR |= ECHO_LINES;
    EXTI->FTSR |= ECHO_LINES;
    EXTI->PR = ECHO_LINES;
    EXTI->IMR |= ECHO_LINES;
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    enabled = true;
    return true;
}

void r08_avoidance_echo_irq(uint32_t irq_lines)
{
    const uint32_t pending = EXTI->PR & ECHO_LINES & irq_lines;
    EXTI->PR = pending;
    if (!enabled || (phase != SONAR_RISE && phase != SONAR_FALL)) {
        return;
    }
    const uint32_t bit = 1u << sonar_pins[active_sensor].echo_pin;
    if ((pending & bit) == 0u) {
        return;
    }
    const uint32_t edge = DWT->CYCCNT;
    const bool high = (GPIOB->IDR & bit) != 0u;
    if (phase == SONAR_RISE && high &&
        edge - trigger_at < RISE_TIMEOUT_US * cycles_per_us) {
        rise_at = edge;
        phase = SONAR_FALL;
    } else if (phase == SONAR_FALL && !high &&
               edge - trigger_at < ECHO_TIMEOUT_US * cycles_per_us) {
        echo_cycles = edge - rise_at;
        phase = SONAR_DONE;
    }
}

static void finish_echo(uint32_t now_ms, uint32_t now_cycles)
{
    const uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    const SonarPhase current = phase;
    const uint32_t age = now_cycles - trigger_at;
    uint32_t duration = 0;
    RangeStatus status = RANGE_PENDING;
    if (current == SONAR_DONE) {
        duration = echo_cycles;
        status = RANGE_ECHO;
    } else if ((current == SONAR_RISE && age >= RISE_TIMEOUT_US * cycles_per_us) ||
               (current == SONAR_FALL && age >= ECHO_TIMEOUT_US * cycles_per_us)) {
        status = RANGE_FAULT;
    }
    if (status != RANGE_PENDING) {
        phase = SONAR_IDLE;
    }
    __set_PRIMASK(irq_mask);
    if (status == RANGE_PENDING) {
        return;
    }

    uint32_t mm = (duration / cycles_per_us * 10u + 29u) / 58u;
    if (status == RANGE_ECHO && mm > MAX_RANGE_MM) {
        status = RANGE_CLEAR;
    }
    if (status == RANGE_ECHO && mm < 20u) {
        mm = 20u;
    }
    car_set_range(&drive.controller, active_sensor, status, (uint16_t)mm, now_ms);
}

static void trigger_sonar(uint32_t now_ms)
{
    const CarSensor sensor = ping_order[next_slot];
    next_slot = (uint8_t)((next_slot + 1u) % (sizeof ping_order / sizeof ping_order[0]));
    const uint32_t trig_bit = 1u << sonar_pins[sensor].trig_pin;
    const uint32_t echo_bit = 1u << sonar_pins[sensor].echo_pin;
    trigger_at = DWT->CYCCNT;
    if ((GPIOB->IDR & echo_bit) != 0u) {
        car_set_range(&drive.controller, sensor, RANGE_FAULT, 0, now_ms);
        return;
    }

    const uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    active_sensor = sensor;
    EXTI->PR = echo_bit;
    phase = SONAR_RISE;
    __set_PRIMASK(irq_mask);
    GPIOB->BSRR = trig_bit;
    const uint32_t high_at = DWT->CYCCNT;
    while (DWT->CYCCNT - high_at < TRIG_PULSE_US * cycles_per_us) {
        __NOP();
    }
    GPIOB->BRR = trig_bit;
}

void r08_avoidance_poll(uint32_t now_ms, uint8_t black_mask)
{
    if (!enabled) {
        return;
    }
    const uint32_t cycles = DWT->CYCCNT;
    finish_echo(now_ms, cycles);
    if (phase == SONAR_IDLE && cycles - trigger_at >= SLOT_US * cycles_per_us &&
        now_ms - init_ms >= FIRST_PING_DELAY_MS) {
        trigger_sonar(now_ms);
    }
    car_drive_step(&drive, now_ms, black_mask);
}

const CarDrive *r08_avoidance_status(void)
{
    return &drive;
}

void r08_gray_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    (void)RCC->APB2ENR;
    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG_Msk) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    gpio_mode(GPIOB, 10, 0x4u);
    gpio_mode(GPIOB, 11, 0x4u);
    gpio_mode(GPIOB, 8, 0x4u);
    gpio_mode(GPIOB, 9, 0x4u);
    gpio_mode(GPIOB, 5, 0x4u);
    gpio_mode(GPIOA, 5, 0x4u);
    gpio_mode(GPIOA, 12, 0x4u);
    gpio_mode(GPIOA, 15, 0x4u);
}

uint8_t r08_gray_mask(bool black_high, bool ch1_is_left)
{
    const uint32_t a = GPIOA->IDR;
    const uint32_t b = GPIOB->IDR;
    const uint8_t channel_bits[] = {
        (uint8_t)(b >> 10), (uint8_t)(b >> 11), (uint8_t)(b >> 8), (uint8_t)(b >> 9),
        (uint8_t)(b >> 5), (uint8_t)(a >> 5), (uint8_t)(a >> 12), (uint8_t)(a >> 15)
    };
    uint8_t mask = 0;
    for (unsigned i = 0; i < 8u; ++i) {
        if (((channel_bits[i] & 1u) != 0u) == black_high) {
            mask |= (uint8_t)(1u << (ch1_is_left ? i : 7u - i));
        }
    }
    return mask;
}
