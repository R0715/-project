#ifndef LINE_FOLLOW_MOTOR_H
#define LINE_FOLLOW_MOTOR_H

#include "line_follow.h"

/* Apply a fresh result only while the main controller gives tracking motor control. */
void line_follow_apply_motor(const LineFollowResult *result);

#endif
