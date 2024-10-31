#ifndef SERVOPOSITION_H
#define SERVOPOSITION_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

void servo_position_task_function();

/**
 * betwen -45 and -45 degrees
 */
void set_servo_position(double pos, SemaphoreHandle_t l_notify_semaphore);

#endif // SERVOPOSITION_H