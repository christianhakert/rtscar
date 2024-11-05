#include <Arduino.h>
#include <Display.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <GxEPD2_BW.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "espnow_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "interface.h"
#include "nvs_flash.h"

#define TICKS_PER_MS 1000
#define BOTTOM_LEFT 26
#define TOP_LEFT 25
#define BOTTOM_RIGHT 4
#define TOP_RIGHT 35
#define DISPLAY_CS 5
#define DISPLAY_RES 9
#define DISPLAY_DC 10
#define DISPLAY_BUSY 19

GxEPD2_BW<WatchyDisplay, WatchyDisplay::HEIGHT> display(WatchyDisplay(DISPLAY_CS, DISPLAY_DC, DISPLAY_RES, DISPLAY_BUSY));

void initDisplay(void *pvParameters) {
    ESP_LOGI("initDisplay", "initializing display");

    /* Setting gpio pin types, always necessary at the start. */
    pinMode(DISPLAY_CS, OUTPUT);
    pinMode(DISPLAY_RES, OUTPUT);
    pinMode(DISPLAY_DC, OUTPUT);
    pinMode(DISPLAY_BUSY, OUTPUT);
    pinMode(BOTTOM_LEFT, INPUT);
    pinMode(BOTTOM_RIGHT, INPUT);
    pinMode(TOP_LEFT, INPUT);
    pinMode(TOP_RIGHT, INPUT);

    /* Init the display. */
    display.epd2.selectSPI(SPI, SPISettings(20000000, MSBFIRST, SPI_MODE0));
    display.init(0, true, 10, true);
    display.setFullWindow();
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold24pt7b);
    display.setCursor(0, 90);
    display.print("Hello\nWorld!");
    display.display(false);

    /* Delete the display initialization task. */
    ESP_LOGI("initDisplay", "finished display initialization");
    // vTaskDelete(NULL);
}

// // REPLACE WITH YOUR RECEIVER MAC Address
// uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// // Structure example to send data
// // Must match the receiver structure
// typedef struct struct_message {
//   char a[32];
//   int b;
//   float c;
//   bool d;
// } struct_message;

// // Create a struct_message called myData
// struct_message myData;

// esp_now_peer_info_t peerInfo;
// 68:b6:b3:3e:34:e8
uint8_t broadcastAddress[] = {0x68, 0xb6, 0xb3, 0x3e, 0x34, 0xe8};

struct struct_message {
    double d;
} __attribute__((packed));

void readMacAddress() {
    uint8_t baseMac[6];
    printf("1\n");
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    printf("1\n");
    if (ret == ESP_OK) {
        printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
               baseMac[0], baseMac[1], baseMac[2],
               baseMac[3], baseMac[4], baseMac[5]);
    } else {
        printf("Failed to read MAC address\n");
    }
}

// Structure example to receive data
// Must match the sender structure
// typedef struct struct_message {
//     char a[32];
//     int b;
//     float c;
//     bool d;
// } struct_message;

// Create a struct_message called myData
// struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    printf("Bytes received: \n");
    struct struct_message *data = (struct struct_message *)incomingData;
    //   memcpy(&myData, incomingData, sizeof(myData));
    printf("Bytes received: %f", data->d);
    //   printf(len);
    printf("Char: ");
    //   printf(myData.a);
    printf("Int: ");
    //   printf(myData.b);
    printf("Float: ");
    //   printf(myData.c);
    printf("Bool: ");
    //   printf(myData.d);
    printf("\n");
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
    printf("1\n");
    ESP_ERROR_CHECK(esp_netif_init());
    printf("2\n");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    printf("3\n");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    printf("4\n");
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    printf("5\n");
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    printf("6\n");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    printf("7\n");
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("8\n");
    ESP_ERROR_CHECK(esp_wifi_set_channel(7, WIFI_SECOND_CHAN_NONE));
    printf("9\n");
#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    printf("Last Packet Send Status:\n");
    printf(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup(void *pvParameters) {
    initDisplay(NULL);

    nvs_flash_init();
    example_wifi_init();
    // readMacAddress();
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);
    printf("cb add\n");

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 7;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        printf("Failed to add peer\n");
    }
    printf("Peer added\n");

    // while(1){
    struct struct_message myData;
    myData.d = 11;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    printf("Send Status: %d\n", result);
    printf("ESP-OK: %d\n", ESP_OK);

    vTaskDelay(200);
    // }

    while (1) {
        struct car2watch incomingData;
        incomingData.magic = 0x42;
        incomingData.angle_granularity = 5;
        // generate random number for udar_map
        for (size_t i = 0; i < 90 / incomingData.angle_granularity + 1; i++) {
            incomingData.udar_map[i] = rand() % 50+20;
        }

        // incomingData.udar_map[0] = random; // in cm
        // show only up to 1 m
        // 0,0 is top left corner
        int car_x = 100;
        int car_y = 150;
        display.fillRect(0, 0, 200, 200, GxEPD_WHITE);
        display.fillCircle(car_x, car_y, 5, GxEPD_BLACK);
        // for (size_t i = 0; i < 90 / incomingData.angle_granularity + 1; i++) {
        //     int x = incomingData.udar_map[i] * cos(-45 + i * incomingData.angle_granularity);
        //     int y = incomingData.udar_map[i] * sin(-45 + i * incomingData.angle_granularity);
        //     // printf("dist: %f, x: %d, y: %d\n", incomingData.udar_map[i],x, y);
        //     display.fillRect(car_x-x, car_y-y, incomingData.angle_granularity, 5, GxEPD_BLACK);
        //     printf("dist: %f, x: %d, y: %d\n", incomingData.udar_map[i],car_x-x, car_y-y);
        // }
        for (size_t i = 0; i < 90 / incomingData.angle_granularity + 1; i++) {
            // for (size_t i = 0; i < 1; i++) {
            // int x = incomingData.udar_map[i] * cos(-45 + i * incomingData.angle_granularity);
            // int y = incomingData.udar_map[i] * sin(-45 + i * incomingData.angle_granularity);
            // // printf("dist: %f, x: %d, y: %d\n", incomingData.udar_map[i],x, y);
            // display.fillRect(car_x-x, car_y-y, incomingData.angle_granularity, 5, GxEPD_BLACK);
            double angle = 45+(double)i * (double)incomingData.angle_granularity;
            double angle_line = (double)angle + (double)90;
            double line_length = incomingData.angle_granularity;
            int x = incomingData.udar_map[i] * cos(angle * DEG_TO_RAD);
            int y = incomingData.udar_map[i] * sin(angle * DEG_TO_RAD);
            printf("dist: %f, x: %d, y: %d\n", incomingData.udar_map[i], x, y);
            int x_start = car_x - x - line_length / 2 * cos(angle_line * DEG_TO_RAD);
            int y_start = car_y - y - line_length / 2 * sin(angle_line * DEG_TO_RAD);
            int x_end = car_x - x + line_length / 2 * cos(angle_line * DEG_TO_RAD);
            int y_end = car_y - y + line_length / 2 * sin(angle_line * DEG_TO_RAD);
            display.drawLine(x_start, y_start, x_end, y_end, GxEPD_BLACK);
        }

        display.display(true);

        vTaskDelay(2000);
        printf("updated map\n");
    }
}

extern "C" void app_main() {
    /* Only priorities from 1-25 (configMAX_PRIORITIES) possible. */
    /* Initialize the display first. */
    xTaskCreate(setup, "setup", 4096, NULL, configMAX_PRIORITIES, NULL);

    ESP_LOGI("app_main", "Starting scheduler from app_main()");
    vTaskStartScheduler();
    /* vTaskStartScheduler is blocking - this should never be reached */
    ESP_LOGE("app_main", "insufficient RAM! aborting");
    abort();
}