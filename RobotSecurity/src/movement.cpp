//#include <Arduino.h>
#include <iostream>
#include <math.h>
#include "movement.hpp"
#include "position.hpp"

void move_forward(Position* pos, const float target_x, const float target_y) {
    while (true) {
        float distance = sqrt(pow(target_x - pos->x, 2) + pow(target_y - pos->y, 2));
        if (distance <= 0.5f) {
            pos->x = target_x;
            pos->y = target_y;
            break;
        }
        //delay(UPDATE_INTERVAL);
        update_position(pos, ROTATIONS_PER_SEC, UPDATE_INTERVAL);
        std::cout << "Position: " << pos->x << ", " << pos->y << "\n";
    }
}

void turn(Position* pos, const float target_theta) {
    float difference = target_theta - pos->theta;
    if (difference > 180.f) difference -= 360.f;
    if (difference < -180.f) difference += 360.f;
    int direction = 0;
    if (difference > 0.f) direction = 1;
    if (difference < 0.f) direction = -1;
    while(true) {
        float difference = fmodf(target_theta - pos->theta + 540.0f, 360.0f) - 180.0f;
        if (fabs(difference) <= 1.25f) {
            pos->theta = target_theta;
            break;
        }
        //delay(UPDATE_INTERVAL);
        update_heading(pos, TURN_SPEED, UPDATE_INTERVAL, target_theta, direction);
        std::cout << "Heading: " << pos->theta << "\n";
    }
}
