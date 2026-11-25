#include "line_follow_motor.h"
#include "motor.h"

void line_follow_apply_motor(const LineFollowResult *result)
{
    const bool driving = result->status == LINE_FOLLOW_TRACKING ||
                         result->status == LINE_FOLLOW_SEARCHING;
    if (driving && (result->left_percent != 0u || result->right_percent != 0u)) {
        Motor_SetPWM((int16_t)(result->left_percent * 10u),
                     (int16_t)(result->right_percent * 10u));
    } else {
        Motor_Stop();
    }
}
