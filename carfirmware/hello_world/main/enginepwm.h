#pragma once

#define ENGINE_PWM_PERIOD 100
#define ENGINE_GPIO_A 1
#define ENGINE_GPIO_B 15


void setup_engine_A(int level);
void setup_engine_B(int level);

void setup_engine_pwm();