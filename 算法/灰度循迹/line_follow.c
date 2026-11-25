#include "line_follow.h"

enum {
    SENSOR_COUNT = 8,
    SENSOR_PITCH_MM = 12,
    FIRST_SENSOR_MM = -42,
    MAX_TRACKED_SENSORS = 3
};

LineFollowConfig line_follow_default_config(bool black_level_is_high, bool ch1_is_left)
{
    return (LineFollowConfig){
        .black_level_is_high = black_level_is_high,
        .ch1_is_left = ch1_is_left,
        .base_percent = 24,
        .max_percent = 36,
        .search_percent = 16,
        .search_timeout_ms = 120,
        .kp = 0.45f,
        .kd = 0.0f,
    };
}

uint8_t line_follow_pack_r03(uint16_t gpioa_idr, uint16_t gpiob_idr)
{
    uint8_t channels = 0;
    channels |= (uint8_t)(((gpiob_idr >> 10) & 1u) << 0); /* CH1: PB10 */
    channels |= (uint8_t)(((gpiob_idr >> 11) & 1u) << 1); /* CH2: PB11 */
    channels |= (uint8_t)(((gpiob_idr >> 8)  & 1u) << 2); /* CH3: PB8  */
    channels |= (uint8_t)(((gpiob_idr >> 9)  & 1u) << 3); /* CH4: PB9  */
    channels |= (uint8_t)(((gpiob_idr >> 5)  & 1u) << 4); /* CH5: PB5  */
    channels |= (uint8_t)(((gpioa_idr >> 5)  & 1u) << 5); /* CH6: PA5  */
    channels |= (uint8_t)(((gpioa_idr >> 12) & 1u) << 6); /* CH7: PA12 */
    channels |= (uint8_t)(((gpioa_idr >> 15) & 1u) << 7); /* CH8: PA15 */
    return channels;
}

void line_follow_reset(LineFollowState *state)
{
    *state = (LineFollowState){0};
}

static uint8_t bounded_percent(float percent, uint8_t maximum)
{
    if (percent <= 0.0f) {
        return 0;
    }
    if (percent >= maximum) {
        return maximum;
    }
    return (uint8_t)(percent + 0.5f);
}

LineFollowResult line_follow_step(LineFollowState *state, const LineFollowConfig *config,
                                  uint8_t raw_mask, uint32_t now_ms)
{
    const uint8_t max_percent = config->max_percent < 100u ? config->max_percent : 100u;
    LineFollowResult result = {
        .status = LINE_FOLLOW_LOST,
        .black_mask = config->black_level_is_high ? raw_mask : (uint8_t)~raw_mask,
    };

    if (result.black_mask == 0u) {
        state->previous_sample_tracked = false;
        if (state->has_line_history && state->last_error_mm != 0.0f &&
            now_ms - state->last_seen_ms < config->search_timeout_ms) {
            const uint8_t speed = bounded_percent(config->search_percent, max_percent);
            result.status = LINE_FOLLOW_SEARCHING;
            if (state->last_error_mm > 0.0f) {
                result.left_percent = speed;
            } else {
                result.right_percent = speed;
            }
        } else {
            line_follow_reset(state);
        }
        return result;
    }

    int count = 0;
    int first_channel = -1;
    int last_channel = -1;
    int total_mm = 0;
    for (int channel = 0; channel < SENSOR_COUNT; ++channel) {
        if ((result.black_mask & (1u << channel)) != 0u) {
            if (first_channel < 0) {
                first_channel = channel;
            }
            last_channel = channel;
            total_mm += FIRST_SENSOR_MM + channel * SENSOR_PITCH_MM;
            ++count;
        }
    }

    if (count > MAX_TRACKED_SENSORS || count != last_channel - first_channel + 1) {
        line_follow_reset(state);
        result.status = LINE_FOLLOW_AMBIGUOUS;
        return result;
    }

    float error_mm = (float)total_mm / (float)count;
    if (!config->ch1_is_left) {
        error_mm = -error_mm;
    }

    float error_rate_mm_per_second = 0.0f;
    const uint32_t elapsed_ms = now_ms - state->last_seen_ms;
    if (state->previous_sample_tracked && elapsed_ms > 0u && elapsed_ms <= 50u) {
        error_rate_mm_per_second = (error_mm - state->last_error_mm) * 1000.0f / (float)elapsed_ms;
    }
    const float steering = config->kp * error_mm + config->kd * error_rate_mm_per_second;
    const uint8_t base = bounded_percent(config->base_percent, max_percent);

    result.status = LINE_FOLLOW_TRACKING;
    result.error_mm = error_mm;
    result.left_percent = bounded_percent(base + steering, max_percent);
    result.right_percent = bounded_percent(base - steering, max_percent);
    state->has_line_history = true;
    state->previous_sample_tracked = true;
    state->last_error_mm = error_mm;
    state->last_seen_ms = now_ms;
    return result;
}
