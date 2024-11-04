#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ULTRASONIC_TRIG_PIN 3
#define ULTRASONIC_ECHO_PIN 4

void ultrasonic_configure();
void ultrasonic_trigger(SemaphoreHandle_t l_notify_semaphore);

double ultrasonic_get_distance();