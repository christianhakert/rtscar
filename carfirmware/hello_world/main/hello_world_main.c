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

#include "esp_now.h"
#include "esp_wifi.h"
#include "espnow_example.h"

#include "nvs_flash.h"

static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(7, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

PRIVILEGED_DATA static portMUX_TYPE xKernelLock = portMUX_INITIALIZER_UNLOCKED;

int servo_pos=1000;
int servo_max=2000;
int servo_min=1000;

void servo_task_fun(){
    int servo_change=15;
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1){
        //Enter critical section
        taskENTER_CRITICAL(&xKernelLock);
        //set the servo to the current position
        gpio_set_level(2, 1);
        //get a current mictosecond counter
        int64_t start=esp_timer_get_time();
        int64_t stop=start;
        //wait for 1ms
        while(stop-start<servo_pos){
            stop=esp_timer_get_time();
        }
        gpio_set_level(2, 0);

        servo_pos+=servo_change;
        if(servo_pos>servo_max){
            servo_change*=-1;
            servo_pos=servo_max;
        }
        if(servo_pos<servo_min){
            servo_change*=-1;
            servo_pos=servo_min;
        }
        taskEXIT_CRITICAL(&xKernelLock);
        vTaskDelayUntil(&xLastWakeTime, 2);
        // printf("SP: %d\n", servo_pos);
    }
}

void ultrasonic_task_fun(){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1){
        taskENTER_CRITICAL(&xKernelLock);
        gpio_set_level(3, 1);
        //get a current mictosecond counter
        int64_t start=esp_timer_get_time();
        int64_t stop=start;
        //wait for 10us
        while(stop-start<10){
            stop=esp_timer_get_time();
        }
        gpio_set_level(3, 0);

        //wait at most a second until gpio 2 becomes hight
        int gval=gpio_get_level(4);
        int64_t now=esp_timer_get_time();
        while(gval==0 && now-start<10000){
            gval=gpio_get_level(4);
            now=esp_timer_get_time();
        }
        //wait until it becomes low again at most a second
        int64_t now2=esp_timer_get_time();
        while(gval==1 && now2-now<10000){
            gval=gpio_get_level(4);
            now2=esp_timer_get_time();
        }

        //now-start is the time of an ultrasonice sensor, convert to centimeters
        int dist=(now2-start)/58;

        taskEXIT_CRITICAL(&xKernelLock);

        printf("Distance is %d cm\n", dist);
        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}

#define PIXEL_COUNT 8
#define NEOPIXEL_PIN GPIO_NUM_5

void led_task(){
    //Run every second

    tNeopixelContext neopixel = neopixel_Init(PIXEL_COUNT, NEOPIXEL_PIN);

   unsigned int rval=0;

    while(1){
        // taskENTER_CRITICAL(&xKernelLock);

        tNeopixel pixel[] =
   {
       { 0, NP_RGB(rval, 255-rval,  0) }, /* red */
       { 1, NP_RGB(rval, 255-rval, 0) }, /* green */
       { 2, NP_RGB(0, 255-rval,  rval) }, /* red */
       { 3, NP_RGB(0,  rval, 255-rval) }, /* green */
         { 4, NP_RGB(255-rval, 0, rval) }, /* red */
         { 5, NP_RGB(255-rval, 0, rval) }, /* green */
         { 6, NP_RGB(255-rval, rval, 0) }, /* red */
         { 7, NP_RGB(255-rval, rval, 0) }, /* green */
   };
        neopixel_SetPixel(neopixel, pixel, 8 );
        rval+=10;
        rval=rval%255;

        // taskEXIT_CRITICAL(&xKernelLock);
        vTaskDelay(100);
    }
}

#include "driver/gpio.h"
#include <string.h>

uint8_t broadcastAddress[] = {0x4c, 0x75, 0x25, 0xa7, 0x5a, 0x8c};

struct struct_message {
  double d;
} __attribute__((packed));

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  printf("Last Packet Send Status:\n");
  printf(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len){
  printf("Bytes received: \n");
  struct struct_message* data = (struct struct_message*) incomingData;
  printf("Bytes received: %f",data->d);
}


void app_main(void)
{
    //Disable watchdog

    printf("Hello world!\n");

    nvs_flash_init();
    example_wifi_init();
    printf("WiFi initialized\n");

    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
        printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                    baseMac[0], baseMac[1], baseMac[2],
                    baseMac[3], baseMac[4], baseMac[5]);
    }

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        printf("Error initializing ESP-NOW");
    }


    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo={};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 7;
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        printf("Failed to add peer\n");
    }
    printf("Peer added\n");

    esp_now_register_recv_cb(OnDataRecv);

    struct struct_message myData;
    myData.d = 42;

    while (1)
    {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
            printf("Send Status: %d\n", result);
    printf("ESP-OK: %d\n", ESP_OK);
            vTaskDelay(100);
    }


    //Configure GPIO 1 as output
    // gpio_set_direction(1, GPIO_MODE_OUTPUT);
    // gpio_set_direction(2, GPIO_MODE_INPUT);
    //Servo
    gpio_set_direction(3, GPIO_MODE_OUTPUT);
    gpio_set_direction(4, GPIO_MODE_INPUT);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    // gpio_set_level(4, 0);

    // xTaskCreate(servo_task_fun, "servo_task", 2048, NULL, 5, NULL);
    // xTaskCreate(ultrasonic_task_fun, "ultrasonic_task", 2048, NULL, 1, NULL);
    // xTaskCreate(led_task, "led_task", 4096, NULL, 6, NULL);

    // gpio_set_direction(1, GPIO_MODE_OUTPUT);
    // gpio_set_direction(15, GPIO_MODE_OUTPUT);

    // int enginestat=0;

    // while(1){
    //     gpio_set_level(15, !enginestat);
    //     gpio_set_level(1, enginestat);
    //     enginestat=!enginestat;
    //     vTaskDelay(100);
    // }


    vTaskDelay(portMAX_DELAY);
}
