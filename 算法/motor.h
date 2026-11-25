#ifndef CAR_MOTOR_H
#define CAR_MOTOR_H

#include <stdint.h>

/* Initialize motor outputs and encoder counters. Start with both motors stopped. */
void Motor_Init(void);

/*
 * Set each wheel's open-loop PWM command in per mille, from -1000 to 1000.
 * Positive moves the car forward on that side; negative reverses it.
 * Zero requests a stop for that wheel. Clamp inputs outside the range.
 */
void Motor_SetPWM(int16_t left, int16_t right);

/* Request a stop on both wheels. The driver determines the electrical stop mode. */
void Motor_Stop(void);

/*
 * Read cumulative quadrature counts since initialization into two valid pointers.
 * Forward rotation increases each wheel's count. Reading does not reset it.
 */
void Encoder_GetCounts(int32_t *left, int32_t *right);

#endif
