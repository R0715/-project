#ifndef SENSOR_TEST_STM32F1XX_H
#define SENSOR_TEST_STM32F1XX_H

#include <stdint.h>

/* STM32F103 and Cortex-M3 register subset for the standalone bench build. */
typedef struct {
    volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t EVCR, MAPR, EXTICR[4];
} AFIO_TypeDef;

typedef struct {
    volatile uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR;
} EXTI_TypeDef;

typedef struct {
    volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t SR, DR, BRR, CR1;
} USART_TypeDef;

typedef struct {
    volatile uint32_t DHCSR, DCRSR, DCRDR, DEMCR;
} CoreDebug_Type;

typedef struct {
    volatile uint32_t CTRL, CYCCNT;
} DWT_Type;

typedef struct {
    volatile uint32_t CTRL, LOAD, VAL;
} SysTick_Type;

#define AFIO      ((AFIO_TypeDef *)0x40010000u)
#define EXTI      ((EXTI_TypeDef *)0x40010400u)
#define GPIOA     ((GPIO_TypeDef *)0x40010800u)
#define GPIOB     ((GPIO_TypeDef *)0x40010C00u)
#define USART1    ((USART_TypeDef *)0x40013800u)
#define RCC       ((RCC_TypeDef *)0x40021000u)
#define DWT       ((DWT_Type *)0xE0001000u)
#define CoreDebug ((CoreDebug_Type *)0xE000EDF0u)
#define SysTick   ((SysTick_Type *)0xE000E010u)

#define AFIO_MAPR_SWJ_CFG_Msk         (7u << 24)
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE (2u << 24)
#define RCC_APB2ENR_AFIOEN            (1u << 0)
#define RCC_APB2ENR_IOPAEN            (1u << 2)
#define RCC_APB2ENR_IOPBEN            (1u << 3)
#define RCC_APB2ENR_USART1EN          (1u << 14)
#define USART_SR_TXE                 (1u << 7)
#define USART_CR1_TE                 (1u << 3)
#define USART_CR1_UE                 (1u << 13)
#define DWT_CTRL_CYCCNTENA_Msk        (1u << 0)
#define CoreDebug_DEMCR_TRCENA_Msk    (1u << 24)
#define SysTick_CTRL_ENABLE_Msk       (1u << 0)
#define SysTick_CTRL_TICKINT_Msk      (1u << 1)
#define SysTick_CTRL_CLKSOURCE_Msk    (1u << 2)

enum { EXTI1_IRQn = 7, EXTI15_10_IRQn = 40 };
extern const uint32_t SystemCoreClock;

static inline void NVIC_EnableIRQ(unsigned irq)
{
    volatile uint32_t *const iser = (volatile uint32_t *)0xE000E100u;
    iser[irq / 32u] = 1u << (irq % 32u);
}

static inline void __NOP(void)
{
    __asm__ volatile ("nop");
}

static inline uint32_t __get_PRIMASK(void)
{
    uint32_t mask;
    __asm__ volatile ("mrs %0, primask" : "=r"(mask));
    return mask;
}

static inline void __disable_irq(void)
{
    __asm__ volatile ("cpsid i" ::: "memory");
}

static inline void __set_PRIMASK(uint32_t mask)
{
    __asm__ volatile ("msr primask, %0" :: "r"(mask) : "memory");
}

#endif
