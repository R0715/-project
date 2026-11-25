#ifndef R08_AVOIDANCE_H
#define R08_AVOIDANCE_H

#include "car_drive.h"

/* Call after system clocks, GPIO setup and Motor_Init() and before running the loop. */
bool r08_avoidance_init(const CarConfig *car_config, const CarDriveConfig *drive_config,
                         uint32_t now_ms);
/* Call continuously in the main loop (preferably every 1 ms or faster). */
void r08_avoidance_poll(uint32_t now_ms, uint8_t black_mask);
/* Call from the two EXTI handlers with the mask of the handler's EXTI lines. */
void r08_avoidance_echo_irq(uint32_t irq_lines);
const CarDrive *r08_avoidance_status(void);

/* Optional GPIO helper for the eight digital gray inputs on J4. */
void r08_gray_init(void);
uint8_t r08_gray_mask(bool black_high, bool ch1_is_left);

#endif
