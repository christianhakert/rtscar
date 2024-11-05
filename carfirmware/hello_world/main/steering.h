#pragma once

#define ENGINE_SPEED 0.5 //m/s
#define WHEEL_DISTANCE 0.15 //m

void update_steering_input(unsigned int speed, double direction);

void steering_task();