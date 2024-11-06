/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "neopixel.h"

#include "driver/gpio.h"

PRIVILEGED_DATA static portMUX_TYPE xKernelLock = portMUX_INITIALIZER_UNLOCKED;


#include "driver/gpio.h"
#include <string.h>

#include "servoposition.h"
#include "udar.h"
#include "ultrasonic.h"
#include "enginepwm.h"
#include "steering.h"
#include "wifi.h"

void app_main(void)
{
    //Disable watchdog

    printf("Hello world!\n");


    //Configure GPIO 1 as output
    // gpio_set_direction(1, GPIO_MODE_OUTPUT);
    // gpio_set_direction(2, GPIO_MODE_INPUT);
    //Servo
    // gpio_set_direction(3, GPIO_MODE_OUTPUT);
    // gpio_set_direction(4, GPIO_MODE_INPUT);
    //Servo Pin
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    //Ultrasonic Pins
    gpio_set_direction(ULTRASONIC_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_level(4, 0);

    setup_wifi();

    xTaskCreate(servo_position_task_function, "servo_position_task", 2048, NULL, 5, NULL);
    xTaskCreate(udar_control_task, "servo_turn_task", 2048, NULL, 4, NULL);
    xTaskCreate(steering_task, "steering_task", 2048, NULL, 1, NULL);
    xTaskCreate(udar_map_send_task, "udar_map_send_task", 4096, NULL, 1, NULL);
    setup_engine_pwm();

    xTaskCreate(kaputt_task, "kaputt_task", 2048, NULL, 10, NULL);


    // xTaskCreate(led_task, "led_task", 4096, NULL, 6, NULL);

    // gpio_set_direction(1, GPIO_MODE_OUTPUT);
    // gpio_set_direction(15, GPIO_MODE_OUTPUT);

    // int enginestat=0;

    // gpio_set_level(1, 1);
    // gpio_set_level(15, 1);

    // while(1){
    //     gpio_set_level(15, !enginestat);
    //     gpio_set_level(1, enginestat);
    //     enginestat=!enginestat;
    //     vTaskDelay(100);
    // }


    vTaskDelay(portMAX_DELAY);
}
