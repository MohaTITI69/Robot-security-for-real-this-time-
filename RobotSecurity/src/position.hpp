#ifndef position_hpp
#define position_hpp

#include <math.h>

struct Position {
    float x;
    float y;
    float theta;
};

static constexpr float WHEEL_DIAMETER = 0.063661977236758f;
static constexpr float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
static constexpr int TICKS_PER_ROTATION = 20;
static constexpr float DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;

Position* init_position(void);
void update_position(Position* pos, const float speed, const float dt);
void update_heading(Position* pos, const float speed, const float dt, const float target_theta, const int direction);

#endif
