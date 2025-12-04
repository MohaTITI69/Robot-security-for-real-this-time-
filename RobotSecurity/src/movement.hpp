#ifndef movement_hpp
#define movement_hpp

#include "position.hpp"

static constexpr float UPDATE_INTERVAL = 0.05f;
static constexpr float METERS_PER_SEC = 0.5f;
static constexpr float ROTATIONS_PER_SEC = METERS_PER_SEC / WHEEL_CIRCUMFERENCE;
static constexpr float TURN_SPEED = 22.5f;

void move_forward(Position* pos, const float target_x, const float target_y);
void turn(Position* pos, const float target_theta);

#endif
