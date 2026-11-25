#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_FOLLOW_TRACKING,
    LINE_FOLLOW_SEARCHING,
    LINE_FOLLOW_LOST,
    LINE_FOLLOW_AMBIGUOUS
} LineFollowStatus;

typedef struct {
    bool black_level_is_high;
    bool ch1_is_left;
    uint8_t base_percent;
    uint8_t max_percent;
    uint8_t search_percent;
    uint32_t search_timeout_ms;
    float kp;
    float kd;
} LineFollowConfig;

typedef struct {
    bool has_line_history;
    bool previous_sample_tracked;
    float last_error_mm;
    uint32_t last_seen_ms;
} LineFollowState;

typedef struct {
    LineFollowStatus status;
    uint8_t black_mask;       /* Bit 0 = CH1. Bit value 1 = detected black. */
    float error_mm;           /* Valid in TRACKING. Positive means line to vehicle right. */
    uint8_t left_percent;     /* Forward request, 0 to 100. */
    uint8_t right_percent;
} LineFollowResult;

/* Set the two sensor flags after checking the assembled sensor. */
LineFollowConfig line_follow_default_config(bool black_level_is_high, bool ch1_is_left);

/* Packs CH1 through CH8 into bits 0 through 7. R03 and R08 share this pin map. */
uint8_t line_follow_pack_r03(uint16_t gpioa_idr, uint16_t gpiob_idr);

/* Initialize on startup and reset whenever another mode takes over the motors. */
void line_follow_reset(LineFollowState *state);

/* Pass a monotonically increasing millisecond tick, such as HAL_GetTick(). */
LineFollowResult line_follow_step(LineFollowState *state, const LineFollowConfig *config,
                                  uint8_t raw_mask, uint32_t now_ms);

#endif
