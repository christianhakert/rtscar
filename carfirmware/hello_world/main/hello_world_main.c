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

//serious functions from here

#include "servoposition.h"
#include "udar.h"
#include "ultrasonic.h"
#include "enginepwm.h"
#include "steering.h"

void app_main(void)
{
    //Disable watchdog

    printf("Hello world!\n");

    //WIFI Stuff
    // nvs_flash_init();
    // example_wifi_init();
    // printf("WiFi initialized\n");

    // uint8_t baseMac[6];
    // esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    // if (ret == ESP_OK) {
    //     printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
    //                 baseMac[0], baseMac[1], baseMac[2],
    //                 baseMac[3], baseMac[4], baseMac[5]);
    // }

    // // Init ESP-NOW
    // if (esp_now_init() != ESP_OK) {
    //     printf("Error initializing ESP-NOW");
    // }


    // esp_now_register_send_cb(OnDataSent);

    // esp_now_peer_info_t peerInfo={};
    // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    // peerInfo.channel = 7;
    // peerInfo.encrypt = false;

    // // Add peer        
    // if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //     printf("Failed to add peer\n");
    // }
    // printf("Peer added\n");

    // esp_now_register_recv_cb(OnDataRecv);

    // struct struct_message myData;
    // myData.d = 42;

    // while (1)
    // {
    //         esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    //         printf("Send Status: %d\n", result);
    // printf("ESP-OK: %d\n", ESP_OK);
    //         vTaskDelay(100);
    // }
    //EOF WIFI Stuff


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

    xTaskCreate(servo_position_task_function, "servo_position_task", 2048, NULL, 5, NULL);
    xTaskCreate(udar_control_task, "servo_turn_task", 2048, NULL, 4, NULL);
    xTaskCreate(steering_task, "steering_task", 2048, NULL, 1, NULL);

    setup_engine_pwm();


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
