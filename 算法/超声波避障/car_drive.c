#include "car_drive.h"
#include "../motor.h"

enum {
    CONTROL_PERIOD_MS = 10,
    CONTROL_DEADLINE_MS = 80,
    ENCODER_CHECK_MS = 520,
    ENCODER_MIN_SPEED_MM_S = 45,
    ENCODER_PROGRESS_MM = 5
};

const CarDriveConfig CAR_DEFAULT_DRIVE_CONFIG = {0, 0, 151, 550};

static int32_t clip(int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo) {
        return lo;
    }
    return value > hi ? hi : value;
}

void car_drive_init(CarDrive *drive, const CarConfig *car_config,
                    const CarDriveConfig *config, uint32_t now_ms)
{
    *drive = (CarDrive){0};
    drive->config = *config;
    drive->last_update_ms = now_ms;
    drive->calibrated = config->left_counts_per_turn != 0u &&
                        config->right_counts_per_turn != 0u &&
                        config->wheel_circumference_mm != 0u &&
                        config->pwm_limit != 0u && config->pwm_limit <= 1000u;
    car_init(&drive->controller, car_config, now_ms);
    Motor_Stop();
    Encoder_GetCounts(&drive->left.start_count, &drive->right.start_count);
    drive->left.prev_count = drive->left.start_count;
    drive->right.prev_count = drive->right.start_count;
}

static int32_t read_wheel(CarWheel *wheel, int32_t count, uint16_t counts_per_turn,
                          uint16_t circumference_mm, uint32_t elapsed_ms)
{
    const int32_t delta = (int32_t)((uint32_t)count - (uint32_t)wheel->prev_count);
    const int32_t total = (int32_t)((uint32_t)count - (uint32_t)wheel->start_count);
    wheel->prev_count = count;
    const int32_t sample = (int32_t)((int64_t)delta * circumference_mm * 1000 /
                                      ((int32_t)counts_per_turn * (int32_t)elapsed_ms));
    wheel->speed_mm_s = (2 * wheel->speed_mm_s + clip(sample, -3000, 3000)) / 3;
    return (int32_t)((int64_t)total * circumference_mm / counts_per_turn);
}

static bool encoder_fault(CarWheel *wheel, int32_t travel_mm, uint32_t now_ms)
{
    if (wheel->direction == 0 || now_ms - wheel->check_start_ms < ENCODER_CHECK_MS) {
        return false;
    }
    const int32_t progress = wheel->direction * (travel_mm - wheel->check_start_mm);
    wheel->check_start_mm = travel_mm;
    wheel->check_start_ms = now_ms;
    return progress < ENCODER_PROGRESS_MM;
}

static int16_t wheel_pwm(CarWheel *wheel, int16_t target, int32_t travel_mm,
                          uint32_t now_ms, uint32_t elapsed_ms, int32_t pwm_limit)
{
    const int8_t direction = target > 0 ? 1 : (target < 0 ? -1 : 0);
    const int32_t speed = direction * target;
    if (direction != wheel->direction) {
        wheel->direction = direction;
        wheel->integral = 0;
        wheel->check_start_ms = now_ms;
        wheel->check_start_mm = travel_mm;
    } else if (speed < ENCODER_MIN_SPEED_MM_S) {
        wheel->check_start_ms = now_ms;
        wheel->check_start_mm = travel_mm;
    }
    if (direction == 0) {
        return 0;
    }

    const int32_t error = speed - direction * wheel->speed_mm_s;
    const int32_t next_integral = clip(wheel->integral + error * (int32_t)elapsed_ms,
                                       -30000, 30000);
    const int32_t pwm = 2 * speed + 2 * error + next_integral / 150;
    if ((pwm >= 0 && pwm <= pwm_limit) || (pwm > pwm_limit && error < 0) ||
        (pwm < 0 && error > 0)) {
        wheel->integral = next_integral;
    }
    return (int16_t)(direction * clip(pwm, 0, pwm_limit));
}

void car_drive_step(CarDrive *drive, uint32_t now_ms, uint8_t black_mask)
{
    if (!drive->calibrated) {
        return;
    }
    const uint32_t elapsed_ms = now_ms - drive->last_update_ms;
    if (elapsed_ms < CONTROL_PERIOD_MS) {
        return;
    }
    drive->last_update_ms = now_ms;
    if (elapsed_ms > CONTROL_DEADLINE_MS &&
        drive->controller.state != CAR_STOPPED &&
        drive->controller.state != CAR_FINISHED) {
        drive->controller.state = CAR_STOPPED;
        drive->controller.stop_reason = STOP_CONTROL_PERIOD;
        Motor_Stop();
        return;
    }
    if (drive->controller.state == CAR_STOPPED || drive->controller.state == CAR_FINISHED) {
        Motor_Stop();
        return;
    }

    int32_t left_count;
    int32_t right_count;
    Encoder_GetCounts(&left_count, &right_count);
    const CarDriveConfig *config = &drive->config;
    CarInput in = {
        .now_ms = now_ms,
        .left_mm = read_wheel(&drive->left, left_count, config->left_counts_per_turn,
                               config->wheel_circumference_mm, elapsed_ms),
        .right_mm = read_wheel(&drive->right, right_count, config->right_counts_per_turn,
                                config->wheel_circumference_mm, elapsed_ms),
        .black_mask = black_mask
    };
    const bool left_fault = encoder_fault(&drive->left, in.left_mm, now_ms);
    const bool right_fault = encoder_fault(&drive->right, in.right_mm, now_ms);
    in.drive_fault = left_fault || right_fault;

    const CarCommand cmd = car_update(&drive->controller, &in);
    if (cmd.brake) {
        drive->left.direction = 0;
        drive->right.direction = 0;
        Motor_Stop();
        return;
    }
    const int16_t left_pwm = wheel_pwm(&drive->left, cmd.left_mm_s, in.left_mm, now_ms,
                                        elapsed_ms, config->pwm_limit);
    const int16_t right_pwm = wheel_pwm(&drive->right, cmd.right_mm_s, in.right_mm, now_ms,
                                         elapsed_ms, config->pwm_limit);
    Motor_SetPWM(left_pwm, right_pwm);
}
