#include "car_control.h"

enum {
    FRONT_DETECT_MM = 350,
    SIDE_DETECT_MM = 230,
    FRONT_SLOW_MM = 530,
    SIDE_SLOW_MM = 325,
    CLEAR_MARGIN_MM = 55,
    HARD_STOP_MM = 90,
    PIVOT_STOP_MM = 120,
    BYPASS_FRONT_MM = 170,
    BYPASS_OUTER_MM = 140,
    TURN_SIDE_FREE_MM = 320,
    RANGE_STALE_MS = 245,
    SENSOR_FAULT_MS = 650,
    OBS_SETTLE_MS = 120,
    OBS_TIMEOUT_MS = 2100,
    FINISH_CREEP_SPEED = 80,
    LINE_LOSS_MS = 330,
    RACE_LIMIT_MS = 300000
};

const CarConfig CAR_DEFAULT_CONFIG = {
    .cruise_mm_s = 175,
    .bypass_mm_s = 115,
    .wheel_track_mm = 161,
    .diagonal_mm = 270,
    .clearance_run_mm = 405,
    .start_seek_mm = 65,
    .finish_min_mm = 2500,
    .finish_roll_mm = 25,
    .finish_window_mm = 45
};

static const CarCommand brake = {0, 0, true};

static int32_t clamp(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

static int32_t magnitude(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t forward_mm(const CarInput *in)
{
    return (in->left_mm + in->right_mm) / 2;
}

static int32_t yaw_mm(const CarInput *in)
{
    return in->left_mm - in->right_mm;
}

static void enter(CarController *car, CarState state, const CarInput *in)
{
    car->state = state;
    car->state_ms = in->now_ms;
    car->entry_mm = forward_mm(in);
    car->mark_samples = 0;
    car->mark_mask = 0;
}

static CarCommand stop(CarController *car, CarStopReason reason)
{
    car->state = CAR_STOPPED;
    car->stop_reason = reason;
    return brake;
}

void car_init(CarController *car, const CarConfig *config, uint32_t now_ms)
{
    *car = (CarController){0};
    car->config = *config;
    car->run_ms = now_ms;
    car->sensor_fault_ms = now_ms;
    car->sensor_unready = true;
}

void car_set_range(CarController *car, CarSensor sensor, RangeStatus status,
                   uint16_t mm, uint32_t now_ms)
{
    CarRange *range = &car->range[sensor];
    const uint16_t limit = sensor == CAR_FRONT ? FRONT_DETECT_MM : SIDE_DETECT_MM;

    range->status = status;
    range->sample_ms = now_ms;
    range->mm = status == RANGE_CLEAR ? 4000u : mm;
    if (status == RANGE_PENDING || status == RANGE_FAULT) {
        range->close_count = 0;
        range->clear_count = 0;
    } else if (range->mm <= limit) {
        range->clear_count = 0;
        if (range->close_count < 2u && ++range->close_count == 2u) {
            range->blocked = true;
        }
    } else {
        range->close_count = 0;
        if (range->mm >= limit + CLEAR_MARGIN_MM) {
            if (range->clear_count < 2u && ++range->clear_count == 2u) {
                range->blocked = false;
            }
        } else {
            range->clear_count = 0;
        }
    }
}

static bool ranges_ready(const CarController *car, uint32_t now_ms)
{
    for (unsigned i = 0; i < CAR_SENSOR_COUNT; ++i) {
        const CarRange *range = &car->range[i];
        if ((range->status != RANGE_ECHO && range->status != RANGE_CLEAR) ||
            now_ms - range->sample_ms > RANGE_STALE_MS) {
            return false;
        }
    }
    return true;
}

static bool within(const CarController *car, uint16_t distance_mm)
{
    return car->range[CAR_LEFT].mm <= distance_mm ||
           car->range[CAR_FRONT].mm <= distance_mm ||
           car->range[CAR_RIGHT].mm <= distance_mm;
}

static bool blocked(const CarController *car)
{
    return car->range[CAR_LEFT].blocked || car->range[CAR_FRONT].blocked ||
           car->range[CAR_RIGHT].blocked;
}

static CarCommand steer(int32_t speed, int32_t correction)
{
    correction = clamp(correction, -speed, speed);
    return (CarCommand){
        (int16_t)(speed + correction), (int16_t)(speed - correction), false
    };
}

static CarCommand hold_heading(const CarController *car, const CarInput *in,
                               int16_t speed, int32_t angle_mm)
{
    const int32_t error = car->original_yaw_mm + angle_mm - yaw_mm(in);
    return steer(speed, clamp(error * 2, -speed / 2, speed / 2));
}

static unsigned black_count(uint8_t mask)
{
    unsigned count = 0;
    while (mask != 0u) {
        mask &= (uint8_t)(mask - 1u);
        ++count;
    }
    return count;
}

static bool thin_line(uint8_t mask)
{
    const unsigned n = black_count(mask);
    if (n == 0u || n > 3u) {
        return false;
    }
    while ((mask & 1u) == 0u) {
        mask >>= 1u;
    }
    return (mask & (uint8_t)(mask + 1u)) == 0u;
}

static bool finish_mark(CarController *car, const CarInput *in)
{
    if (!car->thin_line_seen || forward_mm(in) < car->config.finish_min_mm) {
        return false;
    }

    const uint8_t mask = in->black_mask;
    const int32_t distance = forward_mm(in) - car->mark_mm;
    if (mask == 0u || distance < 0 || distance > car->config.finish_window_mm) {
        car->mark_mask = 0;
        car->mark_samples = 0;
    }
    if (black_count(mask) >= (car->mark_samples == 0u ? 4u : 3u) &&
        (mask & 0xC3u) != 0u) {
        if (car->mark_samples == 0u) {
            car->mark_mm = forward_mm(in);
        }
        car->mark_mask |= mask;
        if (car->mark_samples < 2u) {
            ++car->mark_samples;
        }
    }
    return car->mark_samples >= 2u && black_count(car->mark_mask) >= 6u &&
           (car->mark_mask & 0x03u) != 0u && (car->mark_mask & 0xC0u) != 0u;
}

static CarCommand start_finish(CarController *car, const CarInput *in)
{
    enter(car, CAR_FINISH_CREEP, in);
    car->original_yaw_mm = yaw_mm(in);
    return brake;
}

static CarCommand follow(CarController *car, const CarInput *in)
{
    if (blocked(car) || within(car, HARD_STOP_MM)) {
        enter(car, CAR_CHECK_OBSTACLE, in);
        return brake;
    }
    if (finish_mark(car, in)) {
        return start_finish(car, in);
    }

    const unsigned count = black_count(in->black_mask);
    if (count == 0u) {
        if (!car->line_lost) {
            car->lost_ms = in->now_ms;
            car->line_lost = true;
        }
        if (in->now_ms - car->lost_ms >= LINE_LOSS_MS) {
            return stop(car, STOP_LINE);
        }
        return steer(75, clamp(14 * car->last_line_error, -65, 65));
    }

    car->line_lost = false;
    if (thin_line(in->black_mask)) {
        car->thin_line_seen = true;
    }

    int32_t sum = 0;
    for (unsigned i = 0; i < 8u; ++i) {
        if ((in->black_mask & (1u << i)) != 0u) {
            sum += (int32_t)(i * 2u) - 7;
        }
    }
    const int32_t error = sum / (int32_t)count;
    car->last_line_error = (int8_t)error;
    int32_t speed = car->config.cruise_mm_s - 9 * magnitude(error);
    if (car->range[CAR_FRONT].mm <= FRONT_SLOW_MM ||
        car->range[CAR_LEFT].mm <= SIDE_SLOW_MM ||
        car->range[CAR_RIGHT].mm <= SIDE_SLOW_MM || car->mark_samples != 0u) {
        speed = clamp(speed, 60, 105);
    }
    return steer(speed, 15 * error);
}

static CarCommand check_obstacle(CarController *car, const CarInput *in)
{
    const uint32_t elapsed = in->now_ms - car->state_ms;
    if (elapsed < OBS_SETTLE_MS) {
        return brake;
    }
    if (!blocked(car) && !within(car, HARD_STOP_MM)) {
        enter(car, CAR_FOLLOW, in);
        return brake;
    }
    if (elapsed >= OBS_TIMEOUT_MS) {
        return stop(car, STOP_BLOCKED);
    }
    if (within(car, PIVOT_STOP_MM)) {
        return brake;
    }
    if (car->range[CAR_FRONT].mm > FRONT_DETECT_MM + CLEAR_MARGIN_MM &&
        car->range[CAR_LEFT].mm > SIDE_DETECT_MM + CLEAR_MARGIN_MM &&
        car->range[CAR_RIGHT].mm > SIDE_DETECT_MM + CLEAR_MARGIN_MM) {
        return brake;
    }

    const uint16_t left = car->range[CAR_LEFT].mm;
    const uint16_t right = car->range[CAR_RIGHT].mm;
    if (left < TURN_SIDE_FREE_MM && right < TURN_SIDE_FREE_MM) {
        return brake;
    }

    car->bypass_side = left >= right ? -1 : 1;
    uint16_t detection_mm = 0;
    for (unsigned i = 0; i < CAR_SENSOR_COUNT; ++i) {
        const CarRange *range = &car->range[i];
        if (range->blocked && range->mm > detection_mm) {
            detection_mm = range->mm;
        }
    }
    detection_mm = (uint16_t)clamp(detection_mm, SIDE_DETECT_MM, FRONT_DETECT_MM);
    car->pass_mm = (int16_t)(detection_mm + car->config.clearance_run_mm -
                            (car->config.diagonal_mm * 74) / 100);
    car->original_yaw_mm = yaw_mm(in);
    car->left_track = false;
    car->clear_line_samples = 0;
    car->finding_line = false;
    enter(car, CAR_TURN_OUT, in);
    return brake;
}

static CarCommand turn(CarController *car, const CarInput *in, int32_t target_mm,
                       CarState next)
{
    const int32_t remaining = car->original_yaw_mm + target_mm - yaw_mm(in);
    if (magnitude(remaining) <= 6) {
        enter(car, next, in);
        return brake;
    }
    if (within(car, PIVOT_STOP_MM)) {
        return brake;
    }
    const int16_t speed = (int16_t)clamp(magnitude(remaining) * 3, 55, 100);
    const int16_t signed_speed = remaining > 0 ? speed : (int16_t)-speed;
    return (CarCommand){signed_speed, (int16_t)-signed_speed, false};
}

static CarCommand bypass(CarController *car, const CarInput *in)
{
    const int32_t advance = forward_mm(in) - car->entry_mm;
    const int32_t out_angle = car->bypass_side * car->config.wheel_track_mm * 73 / 100;
    const int32_t home_angle = -car->bypass_side * car->config.wheel_track_mm * 56 / 100;
    const unsigned outer = car->bypass_side < 0 ? CAR_LEFT : CAR_RIGHT;
    const bool pivot = car->state == CAR_TURN_OUT || car->state == CAR_ALIGN_TO_TRACK ||
                       car->state == CAR_TURN_HOME;
    const uint32_t elapsed = in->now_ms - car->state_ms;
    if (elapsed >= (pivot ? 1800u : 6200u)) {
        return stop(car, STOP_MANEUVER);
    }
    if (in->black_mask == 0u) {
        if (car->clear_line_samples < 3u && ++car->clear_line_samples == 3u) {
            car->left_track = true;
        }
    } else {
        car->clear_line_samples = 0;
    }
    if (within(car, HARD_STOP_MM)) {
        return brake;
    }
    if ((car->state == CAR_PASS || car->state == CAR_REJOIN) &&
        !blocked(car) && finish_mark(car, in)) {
        return start_finish(car, in);
    }

    switch (car->state) {
    case CAR_TURN_OUT:
        return turn(car, in, out_angle, CAR_OFFSET);
    case CAR_OFFSET:
        if (advance >= car->config.diagonal_mm) {
            enter(car, CAR_ALIGN_TO_TRACK, in);
            return brake;
        }
        if (car->range[CAR_FRONT].mm <= BYPASS_FRONT_MM ||
            car->range[outer].mm <= BYPASS_OUTER_MM) {
            return brake;
        }
        return hold_heading(car, in, car->config.bypass_mm_s, out_angle);
    case CAR_ALIGN_TO_TRACK:
        return turn(car, in, 0, CAR_PASS);
    case CAR_PASS:
        if (advance >= car->pass_mm) {
            enter(car, CAR_TURN_HOME, in);
            return brake;
        }
        if (car->range[CAR_FRONT].mm <= BYPASS_FRONT_MM ||
            car->range[outer].mm <= BYPASS_OUTER_MM) {
            return brake;
        }
        return hold_heading(car, in, car->config.bypass_mm_s, 0);
    case CAR_TURN_HOME:
        return turn(car, in, home_angle, CAR_REJOIN);
    case CAR_REJOIN:
        if (advance >= 380) {
            return stop(car, STOP_MANEUVER);
        }
        if (car->range[CAR_FRONT].mm <= BYPASS_FRONT_MM) {
            return brake;
        }
        if (car->left_track && advance >= 30 && thin_line(in->black_mask)) {
            if (!car->finding_line) {
                car->rejoin_ms = in->now_ms;
                car->finding_line = true;
            }
            if (in->now_ms - car->rejoin_ms >= 20u) {
                car->line_lost = false;
                enter(car, CAR_FOLLOW, in);
                return follow(car, in);
            }
        } else {
            car->finding_line = false;
        }
        return hold_heading(car, in, car->finding_line ? 75 : car->config.bypass_mm_s,
                            home_angle);
    default:
        return stop(car, STOP_MANEUVER);
    }
}

CarCommand car_update(CarController *car, const CarInput *in)
{
    if (car->state == CAR_FINISHED || car->state == CAR_STOPPED) {
        return brake;
    }
    if (in->drive_fault) {
        return stop(car, STOP_ENCODER);
    }
    if (in->now_ms - car->run_ms >= RACE_LIMIT_MS) {
        return stop(car, STOP_RACE_TIMEOUT);
    }
    if (!ranges_ready(car, in->now_ms)) {
        if (!car->sensor_unready) {
            car->sensor_fault_ms = in->now_ms;
            car->sensor_unready = true;
        }
        if (in->now_ms - car->sensor_fault_ms >= SENSOR_FAULT_MS) {
            return stop(car, STOP_SENSOR);
        }
        return brake;
    }
    car->sensor_unready = false;

    switch (car->state) {
    case CAR_WAIT_READY:
        if (in->black_mask == 0u) {
            enter(car, CAR_START_SEEK, in);
            car->original_yaw_mm = yaw_mm(in);
            return brake;
        }
        enter(car, CAR_FOLLOW, in);
        return follow(car, in);
    case CAR_START_SEEK:
        if (blocked(car) || within(car, HARD_STOP_MM)) {
            return stop(car, STOP_BLOCKED);
        }
        if (in->black_mask != 0u) {
            enter(car, CAR_FOLLOW, in);
            return follow(car, in);
        }
        if (forward_mm(in) - car->entry_mm >= car->config.start_seek_mm ||
            in->now_ms - car->state_ms >= 1300u) {
            return stop(car, STOP_LINE);
        }
        return hold_heading(car, in, 75, 0);
    case CAR_FOLLOW:
        return follow(car, in);
    case CAR_CHECK_OBSTACLE:
        return check_obstacle(car, in);
    case CAR_FINISH_CREEP:
        if (forward_mm(in) - car->entry_mm >= car->config.finish_roll_mm) {
            enter(car, CAR_FINISHED, in);
            return brake;
        }
        if (blocked(car) || within(car, HARD_STOP_MM) ||
            in->now_ms - car->state_ms >= 900u) {
            return stop(car, STOP_BLOCKED);
        }
        return hold_heading(car, in, FINISH_CREEP_SPEED, 0);
    default:
        return bypass(car, in);
    }
}
