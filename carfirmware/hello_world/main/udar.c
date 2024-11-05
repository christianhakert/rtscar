#include "udar.h"
#include "servoposition.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "ultrasonic.h"

double udar_map[UDAR_MAP_SIZE];

double current_position=0;
int current_direction=SCAN_ANGLE_INTV;

double *get_udar_map(){
    return udar_map;
}

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
SemaphoreHandle_t ultrasonic_sempahore;

void udar_control_task(){
    ultrasonic_configure();
    pos_notify_semaphore = xSemaphoreCreateBinary();
    ultrasonic_sempahore = xSemaphoreCreateBinary();
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1){
        //Move the servo forward
        set_next_position();
        set_servo_position(current_position, pos_notify_semaphore);

        //Wait for the position to arrive (at most 1 second)
        xSemaphoreTake(pos_notify_semaphore, 1000/portTICK_PERIOD_MS);

        //Give the servo at least 10ms to move
        vTaskDelay(10/portTICK_PERIOD_MS);

        //Now that the next position arrived, the US sensor can be scanned synchronously
        ultrasonic_trigger(ultrasonic_sempahore);
        //Wait at most 2 seconds for the US sensor to finish
        xSemaphoreTake(ultrasonic_sempahore, 2000/portTICK_PERIOD_MS);
        double distance=ultrasonic_get_distance();

        //Store the distance in the map
        int idx=(current_position+45)/SCAN_ANGLE_INTV;
        udar_map[idx]=distance;
        
        // printf("UDAR Map:\n");
        // for (int i=0; i<UDAR_MAP_SIZE; i++){
        //     printf("%d: %f: ", i, udar_map[i]);
        //     int emptypos=udar_map[i];
        //     if (emptypos>50){
        //         emptypos=50;
        //     }
        //     int full_pos=50-emptypos;
        //     for(int x=0;x<emptypos;x++){
        //         printf(" ");
        //     }
        //     for(int x=0;x<full_pos;x++){
        //         printf("#");
        //     }
        //     printf("\n");
        // }
        // //Clear the terminal
        // printf("\033[2J\033[1;1H");
        
        vTaskDelayUntil(&xLastWakeTime, UDAR_PERIOD/portTICK_PERIOD_MS);
    }
}