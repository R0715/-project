#include "stm32f1xx.h"
#include <stddef.h>

enum { SRAM_TOP = 0x20005000u, CORE_VECTORS = 16 };

extern uint8_t bss_start[] __asm__("section$start$__DATA$__bss");
extern uint8_t bss_end[] __asm__("section$end$__DATA$__bss");
int main(void);
void SysTick_Handler(void);
void EXTI1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

/* Freestanding support for zero-initialization emitted by the compiler. */
void *memset(void *dest, int value, size_t count)
{
    volatile uint8_t *bytes = dest;
    while (count != 0u) {
        *bytes++ = (uint8_t)value;
        --count;
    }
    return dest;
}

void Reset_Handler(void)
{
    for (uintptr_t address = (uintptr_t)bss_start; address < (uintptr_t)bss_end; ++address) {
        *(volatile uint8_t *)address = 0u;
    }
    /* Use the Flash vector table even when entered through the boot alias. */
    *(volatile uint32_t *)0xE000ED08u = 0x08000000u;
    __asm__ volatile ("dsb\n\tisb" ::: "memory");
    (void)main();
    for (;;) {
    }
}

static void Default_Handler(void)
{
    for (;;) {
    }
}

__attribute__((section("__VECTORS,__isr_vector"), used))
const uintptr_t vector_table[CORE_VECTORS + 43] = {
    [0] = SRAM_TOP,
    [1] = (uintptr_t)Reset_Handler,
    [2] = (uintptr_t)Default_Handler,
    [3] = (uintptr_t)Default_Handler,
    [4] = (uintptr_t)Default_Handler,
    [5] = (uintptr_t)Default_Handler,
    [6] = (uintptr_t)Default_Handler,
    [11] = (uintptr_t)Default_Handler,
    [12] = (uintptr_t)Default_Handler,
    [14] = (uintptr_t)Default_Handler,
    [15] = (uintptr_t)SysTick_Handler,
    [CORE_VECTORS + EXTI1_IRQn] = (uintptr_t)EXTI1_IRQHandler,
    [CORE_VECTORS + EXTI15_10_IRQn] = (uintptr_t)EXTI15_10_IRQHandler,
};
