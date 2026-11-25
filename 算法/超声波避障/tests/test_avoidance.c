#include "../car_drive.h"
#include "../../motor.h"

#include <assert.h>
#include <stdio.h>

static int32_t encoder_left, encoder_right;
static int16_t motor_left, motor_right;
static bool motor_braked;

void Motor_SetPWM(int16_t left, int16_t right)
{
    motor_left = left;
    motor_right = right;
    motor_braked = false;
}

void Motor_Stop(void)
{
    motor_left = 0;
    motor_right = 0;
    motor_braked = true;
}

void Encoder_GetCounts(int32_t *left, int32_t *right)
{
    *left = encoder_left;
    *right = encoder_right;
}

static void clear_ranges(CarController *car, uint32_t now_ms)
{
    for (unsigned i = 0; i < CAR_SENSOR_COUNT; ++i) {
        car_set_range(car, (CarSensor)i, RANGE_CLEAR, 0, now_ms);
    }
}

static CarCommand step(CarController *car, uint32_t now_ms, int32_t travel_mm,
                        int32_t yaw_mm, uint8_t mask)
{
    const CarInput in = {now_ms, travel_mm + yaw_mm / 2, travel_mm - yaw_mm / 2,
                         mask, false};
    return car_update(car, &in);
}

static void test_start_and_ranging(void)
{
    CarController car;
    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    assert(step(&car, 10, 0, 0, 0x18).brake && car.state == CAR_WAIT_READY);
    clear_ranges(&car, 220);
    assert(step(&car, 220, 0, 0, 0).brake && car.state == CAR_START_SEEK);
    const CarCommand seek = step(&car, 230, 0, 0, 0);
    assert(!seek.brake && seek.left_mm_s == 75 && seek.right_mm_s == 75);
    assert(!step(&car, 240, 25, 0, 0xFF).brake && car.state == CAR_FOLLOW);
    assert(!step(&car, 250, 30, 0, 0x18).brake);
    car_set_range(&car, CAR_FRONT, RANGE_ECHO, 310, 260);
    const CarCommand slow = step(&car, 260, 31, 0, 0x18);
    assert(!slow.brake && slow.left_mm_s < CAR_DEFAULT_CONFIG.cruise_mm_s);
    car_set_range(&car, CAR_FRONT, RANGE_CLEAR, 0, 350);
    assert(!step(&car, 350, 46, 0, 0x18).brake);
    car_set_range(&car, CAR_FRONT, RANGE_ECHO, 89, 440);
    assert(step(&car, 440, 62, 0, 0x18).brake && car.state == CAR_CHECK_OBSTACLE);
    assert(step(&car, 490, 62, 0, 0x18).brake);  /* Side samples expired. */
    clear_ranges(&car, 600);
    assert(step(&car, 600, 62, 0, 0x18).brake && car.state == CAR_FOLLOW);
    assert(!step(&car, 610, 62, 0, 0x18).brake);

    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    clear_ranges(&car, 200);
    step(&car, 200, 0, 0, 0);
    assert(step(&car, 220, 65, 0, 0).brake && car.stop_reason == STOP_LINE);
}

static void test_fault_and_blocked(void)
{
    CarController car;
    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    clear_ranges(&car, 10);
    assert(!step(&car, 10, 0, 0, 0x18).brake);
    car_set_range(&car, CAR_RIGHT, RANGE_FAULT, 0, 190);
    assert(step(&car, 190, 30, 0, 0x18).brake);
    clear_ranges(&car, 300);
    assert(!step(&car, 300, 30, 0, 0x18).brake);
    assert(step(&car, 546, 40, 0, 0x18).brake);
    assert(step(&car, 1196, 40, 0, 0x18).brake && car.stop_reason == STOP_SENSOR);
    clear_ranges(&car, 1200);
    assert(step(&car, 1200, 40, 0, 0x18).brake);

    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    clear_ranges(&car, 0);
    step(&car, 0, 0, 0, 0x18);
    for (uint32_t ms = 100; ms <= 2400; ms += 100) {
        for (unsigned i = 0; i < CAR_SENSOR_COUNT; ++i) {
            car_set_range(&car, (CarSensor)i, RANGE_ECHO, 161, ms);
        }
        step(&car, ms, 1, 0, 0x18);
    }
    assert(car.state == CAR_STOPPED && car.stop_reason == STOP_BLOCKED);
}

static void test_bypass(int side)
{
    CarController car;
    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    clear_ranges(&car, 0);
    step(&car, 0, 0, 0, 0x18);
    const CarSensor occupied_side = side > 0 ? CAR_LEFT : CAR_RIGHT;
    for (uint32_t ms = 100; ms <= 200; ms += 100) {
        car_set_range(&car, occupied_side, RANGE_ECHO, 190, ms);
        car_set_range(&car, CAR_FRONT, RANGE_ECHO, 305, ms);
        step(&car, ms, 50, 0, 0x18);
    }
    assert(car.state == CAR_CHECK_OBSTACLE);

    car_set_range(&car, side > 0 ? CAR_RIGHT : CAR_LEFT, RANGE_CLEAR, 0, 320);
    assert(step(&car, 320, 50, 0, 0x18).brake && car.state == CAR_TURN_OUT);
    const CarCommand turn = step(&car, 330, 50, 0, 0x18);
    assert(!turn.brake && turn.left_mm_s * side > 0 && turn.right_mm_s * side < 0);
    const int32_t angle = side * car.config.wheel_track_mm * 73 / 100;
    const int32_t home_angle = -side * car.config.wheel_track_mm * 56 / 100;
    const int32_t offset_end = 50 + car.config.diagonal_mm;
    const int32_t pass_end = offset_end + car.pass_mm;
    clear_ranges(&car, 880);
    assert(step(&car, 880, 50, angle, 0).brake && car.state == CAR_OFFSET);
    assert(!step(&car, 890, 50, angle, 0).brake);
    assert(!step(&car, 900, 51, angle, 0).brake && car.left_track);
    clear_ranges(&car, 2940);
    assert(step(&car, 2940, offset_end, angle, 0).brake && car.state == CAR_ALIGN_TO_TRACK);
    clear_ranges(&car, 3510);
    assert(step(&car, 3510, offset_end, 0, 0).brake && car.state == CAR_PASS);
    assert(!step(&car, 3520, offset_end, 0, 0).brake);
    clear_ranges(&car, 7900);
    assert(step(&car, 7900, pass_end, 0, 0).brake && car.state == CAR_TURN_HOME);
    clear_ranges(&car, 8390);
    assert(step(&car, 8390, pass_end, home_angle, 0).brake && car.state == CAR_REJOIN);
    clear_ranges(&car, 8650);
    assert(!step(&car, 8650, pass_end + 12, home_angle, 0x18).brake);
    assert(!car.finding_line);
    assert(!step(&car, 8750, pass_end + 30, home_angle, 0x18).brake);
    assert(!step(&car, 8760, pass_end + 31, home_angle, 0x18).brake);
    assert(!step(&car, 8770, pass_end + 32, home_angle, 0x18).brake);
    assert(car.state == CAR_FOLLOW);
}

static void test_finish_and_line_loss(void)
{
    CarController car;
    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    clear_ranges(&car, 10);
    assert(!step(&car, 10, 10, 0, 0x18).brake);
    assert(!step(&car, 20, 12, 0, 0xFF).brake);
    assert(!step(&car, 30, 14, 0, 0xFF).brake && car.state == CAR_FOLLOW);

    clear_ranges(&car, 16000);
    assert(!step(&car, 16000, 2550, 0, 0xFF).brake);
    assert(!step(&car, 16010, 2551, 0, 0x00).brake);
    assert(!step(&car, 16020, 2552, 0, 0x0F).brake);
    assert(step(&car, 16090, 2562, 0, 0xF0).brake && car.state == CAR_FINISH_CREEP);
    assert(!step(&car, 16100, 2565, 0, 0).brake);
    assert(step(&car, 16180, 2587, 0, 0).brake && car.state == CAR_FINISHED);
    clear_ranges(&car, 16500);
    assert(step(&car, 16500, 2600, 0, 0x18).brake);

    car_init(&car, &CAR_DEFAULT_CONFIG, 0);
    clear_ranges(&car, 10);
    step(&car, 10, 10, 0, 0x60);
    assert(!step(&car, 20, 12, 0, 0x00).brake);
    clear_ranges(&car, 350);
    assert(step(&car, 350, 12, 0, 0x00).brake && car.stop_reason == STOP_LINE);
}

static void test_drive_adapter(void)
{
    CarDrive drive;
    car_drive_init(&drive, &CAR_DEFAULT_CONFIG, &CAR_DEFAULT_DRIVE_CONFIG, 0);
    clear_ranges(&drive.controller, 10);
    car_drive_step(&drive, 10, 0x18);
    assert(motor_braked && !drive.calibrated);

    CarDriveConfig config = CAR_DEFAULT_DRIVE_CONFIG;
    config.left_counts_per_turn = 1040;
    config.right_counts_per_turn = 1040;
    car_drive_init(&drive, &CAR_DEFAULT_CONFIG, &config, 0);
    for (uint32_t ms = 10; ms <= 1000; ms += 10) {
        encoder_left += 10;
        encoder_right += 10;
        clear_ranges(&drive.controller, ms);
        car_drive_step(&drive, ms, 0x18);
        assert(!motor_braked && motor_left > 0 && motor_left == motor_right);
        assert(motor_left <= (int16_t)config.pwm_limit);
    }
    car_drive_step(&drive, 1090, 0x18);
    assert(motor_braked && drive.controller.stop_reason == STOP_CONTROL_PERIOD);

    car_drive_init(&drive, &CAR_DEFAULT_CONFIG, &config, 0);
    for (uint32_t ms = 10; ms <= 600; ms += 10) {
        encoder_right += 10;
        clear_ranges(&drive.controller, ms);
        car_drive_step(&drive, ms, 0x18);
    }
    assert(motor_braked && drive.controller.stop_reason == STOP_ENCODER);
}

int main(void)
{
    test_start_and_ranging();
    test_fault_and_blocked();
    test_bypass(-1);
    test_bypass(1);
    test_finish_and_line_loss();
    test_drive_adapter();
    puts("6 avoidance and stop scenarios passed");
    return 0;
}
