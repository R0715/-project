#include "line_follow.h"
#include "r08_avoidance.h"
#include "stm32f1xx.h"

enum { BAUD_RATE = 115200, REPORT_PERIOD_MS = 250 };

const uint32_t SystemCoreClock = 8000000u; /* HSI after hardware reset. */
static volatile uint32_t tick_ms;

/* The R08 sensor code takes a CarDrive. This build supplies only its range storage. */
void car_drive_init(CarDrive *drive, const CarConfig *car_config,
                    const CarDriveConfig *drive_config, uint32_t now_ms)
{
    (void)drive_config;
    car_init(&drive->controller, car_config, now_ms);
}

void car_drive_step(CarDrive *drive, uint32_t now_ms, uint8_t black_mask)
{
    (void)drive;
    (void)now_ms;
    (void)black_mask;
}

void SysTick_Handler(void)
{
    ++tick_ms;
}

void EXTI1_IRQHandler(void)
{
    r08_avoidance_echo_irq(1u << 1);
}

void EXTI15_10_IRQHandler(void)
{
    r08_avoidance_echo_irq((1u << 13) | (1u << 15));
}

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    /* PA9 is USART1 TX and is wired to the core board's CH340. */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 4)) | (0xAu << 4);
    USART1->BRR = (SystemCoreClock + BAUD_RATE / 2u) / BAUD_RATE;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart_char(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0u) {
    }
    USART1->DR = (uint8_t)c;
}

static void uart_write(const char *message)
{
    while (*message != '\0') {
        uart_char(*message++);
    }
}

static void uart_decimal(uint16_t value)
{
    char digits[5];
    unsigned count = 0;
    do {
        digits[count++] = (char)('0' + value % 10u);
        value = (uint16_t)(value / 10u);
    } while (value != 0u);
    while (count != 0u) {
        uart_char(digits[--count]);
    }
}

static void report_range(const char *label, const CarRange *range)
{
    uart_write(label);
    switch (range->status) {
    case RANGE_ECHO:
        uart_decimal(range->mm);
        uart_write("mm");
        break;
    case RANGE_CLEAR:
        uart_write(">4000mm");
        break;
    case RANGE_FAULT:
        uart_write("NO_ECHO");
        break;
    case RANGE_PENDING:
        uart_write("WAIT");
        break;
    }
}

static void report_sample(uint8_t gray, const CarRange ranges[CAR_SENSOR_COUNT])
{
    uart_write("GRAY CH1..CH8=");
    for (unsigned i = 0; i < 8u; ++i) {
        uart_char((gray & (1u << i)) != 0u ? '1' : '0');
    }
    report_range(" | L=", &ranges[CAR_LEFT]);
    report_range(" F=", &ranges[CAR_FRONT]);
    report_range(" R=", &ranges[CAR_RIGHT]);
    uart_write("\r\n");
}

int main(void)
{
    static const CarDriveConfig bench_drive = {0};
    uart_init();
    uart_write("\r\nR08 sensor test | STM32F103C8 | 115200 8N1\r\n");
    r08_gray_init();
    if (!r08_avoidance_init(&CAR_DEFAULT_CONFIG, &bench_drive, 0u)) {
        uart_write("ERROR: ultrasonic timer did not start\r\n");
        for (;;) {
        }
    }

    SysTick->LOAD = SystemCoreClock / 1000u - 1u;
    SysTick->VAL = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;

    uint32_t last_report = 0u;
    for (;;) {
        const uint32_t now = tick_ms;
        r08_avoidance_poll(now, 0u);
        if (now - last_report >= REPORT_PERIOD_MS) {
            last_report = now;
            const uint8_t gray = line_follow_pack_r03((uint16_t)GPIOA->IDR,
                                                      (uint16_t)GPIOB->IDR);
            report_sample(gray, r08_avoidance_status()->controller.range);
        }
    }
}
