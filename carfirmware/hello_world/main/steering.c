#include "steering.h"

#include "enginepwm.h"
#include "udar.h"

#include "freertos/FreeRTOS.h"

double s_speed;
double s_direction;

double brake_factor=1;

#define PI 3.14159265358979323846

void update_steering_input(unsigned int speed, double direction){
    s_speed=speed*ENGINE_SPEED/100.0;
    s_direction=direction;
}

void update_engs(){
    //get the absolute steering angle first
    double abs_angle=s_direction;
    if (abs_angle<0){
        abs_angle*=-1;
    }
    double abs_speed=s_speed*brake_factor;

    double outer_speed=abs_speed;
    double inner_speed=outer_speed-(2*PI*WHEEL_DISTANCE*abs_angle)/360.0;
    if (inner_speed<0){
        inner_speed=0;
    }

    //reseemble engine control vals
    int eng_outer_speed=100*outer_speed/ENGINE_SPEED;
    int eng_inner_speed=100*inner_speed/ENGINE_SPEED;

    printf("Outer speed: %d\n", eng_outer_speed);
    printf("Inner speed: %d\n", eng_inner_speed);

    //If the direction is negative, the inner wheel is the left one
    if (s_direction<0){
        setup_engine_B(eng_inner_speed);
        setup_engine_A(eng_outer_speed);
    } else {
        setup_engine_B(eng_outer_speed);
        setup_engine_A(eng_inner_speed);
    }
}

#define COLLISION_PRED 3
#include <math.h>

void predict_collisions(){
    double x_offset[COLLISION_PRED];
    double y_offset[COLLISION_PRED];

    //compute current movement radius
    double abs_angle=s_direction;
    if (abs_angle<0){
        abs_angle*=-1;
    }

    double r=1000;
    if(abs_angle!=0)
        r=s_speed*360.0/(2*PI*abs_angle);

    for (int s=0;s<COLLISION_PRED;s++){
        x_offset[s]=r-r*cos((s+1)*abs_angle*PI/180.0);
        y_offset[s]=r*sin((s+1)*abs_angle*PI/180.0);

        if (s_direction<0){
            x_offset[s]*=-1;
        }
    }

    //Now compute the polar coordinates of the predicted collision points
    double ang[COLLISION_PRED];
    double dist[COLLISION_PRED];

    for (int s=0;s<COLLISION_PRED;s++){
        ang[s]=atan2(y_offset[s], x_offset[s])*180.0/PI;
        dist[s]=sqrt(x_offset[s]*x_offset[s]+y_offset[s]*y_offset[s]);

        //Transform angle
        ang[s]=90-ang[s];

        // printf("Predicted collision point %d: %f, %f\n", s, ang[s], dist[s]);
    }

    //Now check if the predicted collision points are within the UDAR map
    for (int s=0;s<COLLISION_PRED;s++){
        int idx=(ang[s]+45)/SCAN_ANGLE_INTV;
        if (idx<0 || idx>=UDAR_MAP_SIZE){
            // printf("Collision prediction out of bounds\n");
            continue;
        }

        double udar_dist=get_udar_map()[idx];
        udar_dist/=100; //cm
        // printf("UDAR dist is %f\n", udar_dist);
        // printf("Mydist is %f\n", dist[s]);
        printf("\n");

        if (udar_dist<(dist[s]+0.2)){
            brake_factor=1.0*(s+1)/(COLLISION_PRED);
            printf("Brake factor: %f\n", brake_factor);
            break;
        }
        brake_factor=1;
    }
}

void steering_task(){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    // update_steering_input(90, -1);
    while(1){
        predict_collisions();
        update_engs();

        vTaskDelayUntil(&xLastWakeTime, 500/portTICK_PERIOD_MS);
    }
}