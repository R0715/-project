#ifndef CAR_DRIVE_H
#define CAR_DRIVE_H

#include "car_control.h"

typedef struct {
    uint16_t left_counts_per_turn;
    uint16_t right_counts_per_turn;
    uint16_t wheel_circumference_mm;
    uint16_t pwm_limit;
} CarDriveConfig;

typedef struct {
    int32_t start_count;
    int32_t prev_count;
    int32_t speed_mm_s;
    int32_t integral;
    int32_t check_start_mm;
    uint32_t check_start_ms;
    int8_t direction;
} CarWheel;

typedef struct {
    CarController controller;
    CarDriveConfig config;
    CarWheel left;
    CarWheel right;
    uint32_t last_update_ms;
    bool calibrated;
} CarDrive;

extern const CarDriveConfig CAR_DEFAULT_DRIVE_CONFIG;

/* Call after Motor_Init(). Zero counts per turn allow ranging with the car braked. */
void car_drive_init(CarDrive *drive, const CarConfig *car_config,
                    const CarDriveConfig *config, uint32_t now_ms);
void car_drive_step(CarDrive *drive, uint32_t now_ms, uint8_t black_mask);

#endif
