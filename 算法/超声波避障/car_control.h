#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { CAR_LEFT, CAR_FRONT, CAR_RIGHT, CAR_SENSOR_COUNT } CarSensor;
typedef enum { RANGE_PENDING, RANGE_ECHO, RANGE_CLEAR, RANGE_FAULT } RangeStatus;

typedef enum {
    CAR_WAIT_READY, CAR_START_SEEK, CAR_FOLLOW, CAR_CHECK_OBSTACLE, CAR_TURN_OUT,
    CAR_OFFSET, CAR_ALIGN_TO_TRACK, CAR_PASS, CAR_TURN_HOME, CAR_REJOIN,
    CAR_FINISH_CREEP, CAR_FINISHED, CAR_STOPPED
} CarState;

typedef enum {
    STOP_NONE, STOP_SENSOR, STOP_BLOCKED, STOP_MANEUVER, STOP_LINE,
    STOP_ENCODER, STOP_RACE_TIMEOUT, STOP_CONTROL_PERIOD
} CarStopReason;

typedef struct {
    int16_t cruise_mm_s;
    int16_t bypass_mm_s;
    int16_t wheel_track_mm;
    int16_t diagonal_mm;
    int16_t clearance_run_mm;
    int16_t start_seek_mm;
    int32_t finish_min_mm;
    int16_t finish_roll_mm;
    int16_t finish_window_mm;
} CarConfig;

typedef struct {
    uint32_t now_ms;
    int32_t left_mm;                 /* Cumulative signed travel, forward positive. */
    int32_t right_mm;
    uint8_t black_mask;              /* Bit 0 = vehicle leftmost sensor. */
    bool drive_fault;
} CarInput;

typedef struct {
    int16_t left_mm_s;
    int16_t right_mm_s;
    bool brake;
} CarCommand;

typedef struct {
    uint32_t sample_ms;
    uint16_t mm;
    RangeStatus status;
    uint8_t close_count;
    uint8_t clear_count;
    bool blocked;
} CarRange;

typedef struct {
    CarConfig config;
    CarRange range[CAR_SENSOR_COUNT];
    CarState state;
    CarStopReason stop_reason;
    int8_t bypass_side;              /* -1 = left, +1 = right. */
    int16_t pass_mm;
    int32_t entry_mm;
    int32_t original_yaw_mm;
    int32_t mark_mm;
    uint32_t state_ms;
    uint32_t run_ms;
    uint32_t sensor_fault_ms;
    uint32_t lost_ms;
    uint32_t rejoin_ms;
    uint8_t mark_mask;
    uint8_t mark_samples;
    uint8_t clear_line_samples;
    int8_t last_line_error;
    bool sensor_unready;
    bool line_lost;
    bool thin_line_seen;
    bool left_track;
    bool finding_line;
} CarController;

extern const CarConfig CAR_DEFAULT_CONFIG;

void car_init(CarController *car, const CarConfig *config, uint32_t now_ms);
void car_set_range(CarController *car, CarSensor sensor, RangeStatus status,
                   uint16_t mm, uint32_t now_ms);
CarCommand car_update(CarController *car, const CarInput *input);

#endif
