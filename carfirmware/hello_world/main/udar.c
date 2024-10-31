#include "udar.h"
#include "servoposition.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SCAN_ANGLE_INTV 1

double current_position=0;
int current_direction=SCAN_ANGLE_INTV;

void set_next_position(){
    current_position+=current_direction;
    if (current_position >= 45){
        current_direction*=-1;
        current_position=45;
    }
    if (current_position <= -45){
        current_direction*=-1;
        current_position=-45;
    }
}

SemaphoreHandle_t pos_notify_semaphore;

void udar_control_task(){
    pos_notify_semaphore = xSemaphoreCreateBinary();
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1){
        //Move the servo forward
        set_next_position();
        set_servo_position(current_position, pos_notify_semaphore);

        //Wait for the position to arrive
        xSemaphoreTake(pos_notify_semaphore, portMAX_DELAY);

        //Now that the next position arrived, the US sensor can be scanned synchronously
        
        vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_PERIOD_MS);
    }
}