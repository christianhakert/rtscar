#include <Arduino.h>
#include <Display.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <GxEPD2_BW.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "espnow_example.h"
#include "helper.h"
#include "interface.h"
#include "nvs_flash.h"

GxEPD2_BW<WatchyDisplay, WatchyDisplay::HEIGHT> display(WatchyDisplay(DISPLAY_CS, DISPLAY_DC, DISPLAY_RES, DISPLAY_BUSY));

BMA423 sensor;

SemaphoreHandle_t xSemaphoreCarUpdate, xSemaphoreMotorControl, xSemaphoreKaputt, xSemaphoreFreedom;

unsigned int angle_granularity;
double udar_map[90];

// esp_now_peer_info_t peerInfo;
// 68:b6:b3:3e:34:e8
uint8_t broadcastAddress[] = {0x68, 0xb6, 0xb3, 0x3e, 0x34, 0xe8};

bool initial = true;
bool motor_on = false;
bool kaputt = false;
bool freedom = false;

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
    display.setFont(&FreeMonoBold18pt7b);
    display.setCursor(0, 75);
    display.print("Press\nTop Right\nto Start!");
    display.display(false);

    ESP_LOGI("initDisplay", "finished display initialization");
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(7, WIFI_SECOND_CHAN_NONE));
#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

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

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    struct car2watch *data = (struct car2watch *)incomingData;
    if (data->magic != 0x42) {
        printf("Invalid magic number\n");
        return;
    }
    printf("Received data\n");
    angle_granularity = data->angle_granularity;
    for (size_t i = 0; i < 90; i++) {
        udar_map[i] = data->udar_map[i];
    }
    xSemaphoreGiveFromISR(xSemaphoreCarUpdate, NULL);
}

void map_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreCarUpdate, portMAX_DELAY) == pdTRUE) {
            // struct car2watch incomingData;
            // incomingData.magic = 0x42;
            // incomingData.angle_granularity = 5;
            // // generate random number for udar_map
            // for (size_t i = 0; i < 90 / angle_granularity + 1; i++) {
            //     incomingData.udar_map[i] = rand() % 50 + 20;
            // }

            // incomingData.udar_map[0] = random; // in cm
            // show only up to 1 m
            // 0,0 is top left corner
            int car_x = 100;
            int car_y = 150;
            constexpr unsigned int scaling = 2;
            display.fillScreen(GxEPD_WHITE);
            if (motor_on) {
                display.fillRect(190, 10, 10, 30, GxEPD_BLACK);
            }
            if (kaputt) {
                display.fillRect(0, 190, 20, 10, GxEPD_BLACK);
            }
            display.fillCircle(car_x, car_y, 5, GxEPD_BLACK);
            for (size_t i = 0; i < 90 / angle_granularity + 1; i++) {
                double angle = 45 + (double)i * (double)angle_granularity;
                double angle_line = (double)angle + (double)90;
                double line_length = angle_granularity;
                int x = udar_map[i] * scaling * cos(angle * DEG_TO_RAD);
                int y = udar_map[i] * scaling * sin(angle * DEG_TO_RAD);
                // printf("dist: %f, x: %d, y: %d\n", incomingData.udar_map[i], x, y);
                int x_start = car_x - x - line_length / 2 * cos(angle_line * DEG_TO_RAD);
                int y_start = car_y - y - line_length / 2 * sin(angle_line * DEG_TO_RAD);
                int x_end = car_x - x + line_length / 2 * cos(angle_line * DEG_TO_RAD);
                int y_end = car_y - y + line_length / 2 * sin(angle_line * DEG_TO_RAD);
                display.drawLine(x_start, y_start, x_end, y_end, GxEPD_BLACK);
            }

            display.display(true);
            printf("udpated map\n");
        }
    }
}

void accel_task(void *pvParameters) {
    _bmaConfig();

    while (1) {
        Accel acc;
        // Get acceleration data
        bool res = sensor.getAccel(acc);

        // -x nach vorne neigen
        // x nach hinten neigen
        // y nach links neigen
        // -y nach rechts neigen
        // 90° Neigung = 1000
        if (res == true) {
            double x = acc.x * -1;
            unsigned int speed;
            if (x >= 1000) {
                speed = 100;
            } else if (x >= 0) {
                x = x / 1000 * 100;
                // x = 100 / (1 + exp(-0.1 * (x - 65)));
                speed = (unsigned int)x;
            } else {
                speed = 0;
            }
            speed *= motor_on;
            double y = acc.y * -1;
            double direction;
            if (y >= 1000) {
                direction = 90;
            } else if (y <= -1000) {
                direction = -90;
            } else {
                direction = y / 1000 * 90;
            }
            printf("%d, %f,(%d,%d,%d)\n", speed, direction, acc.x, acc.y, acc.z);

            struct watch2car data;
            data.magic = 0x42;
            data.speed = speed;
            data.direction = direction;
            data.kaputt = kaputt;
            data.freedom = freedom;

            ESP_ERROR_CHECK(esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data)));
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void motor_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreMotorControl, portMAX_DELAY) == pdTRUE) {
            if (motor_on) {
                if (initial) {
                    initial = false;
                    display.fillScreen(GxEPD_WHITE);
                }
                printf("Motor on\n");
                display.fillRect(190, 10, 10, 30, GxEPD_BLACK);
                display.display(true);
            } else {
                printf("Motor off\n");
                display.fillRect(190, 10, 10, 30, GxEPD_WHITE);
                display.display(true);
            }
        }
    }
}
void kaputt_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreKaputt, portMAX_DELAY) == pdTRUE) {
            if (kaputt) {
                printf("Kaputt on\n");
                display.fillRect(0, 190, 20, 10, GxEPD_BLACK);
                display.display(true);
            } else {
                printf("Kaputt off\n");
                display.fillRect(0, 190, 20, 10, GxEPD_WHITE);
                display.display(true);
            }
        }
    }
}
void freedom_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreFreedom, portMAX_DELAY) == pdTRUE) {
            if (freedom) {
                printf("FREEDOM on\n");
                display.fillRect(0, 10, 10, 30, GxEPD_BLACK);
                display.display(true);
            } else {
                printf("FREEDOM off\n");
                display.fillRect(0, 10, 10, 30, GxEPD_WHITE);
                display.display(true);
            }
        }
    }
}

void setup(void *pvParameters) {
    initDisplay(NULL);

    nvs_flash_init();
    example_wifi_init();
    readMacAddress();
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);
    // esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 7;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        printf("Failed to add peer\n");
    }
    printf("Peer added\n");

    xTaskCreate(kaputt_task, "kaputt_task", 4096, NULL, 6, NULL);
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(accel_task, "accel_task", 4096, NULL, 4, NULL);
    xTaskCreate(map_task, "map_task", 4096, NULL, 3, NULL);
    xTaskCreate(freedom_task, "freedom_task", 4096, NULL, 2, NULL);

    vTaskDelete(NULL);
}
static void toggle_kaputt(void *args) {
    if (!initial) {
        kaputt = !kaputt;
        xSemaphoreGiveFromISR(xSemaphoreKaputt, NULL);
    }
}
static void toggle_freedom(void *args) {
    if (!initial) {
        freedom = !freedom;
        xSemaphoreGiveFromISR(xSemaphoreFreedom, NULL);
    }
}
static void toggle_motor(void *args) {
    motor_on = !motor_on;
    xSemaphoreGiveFromISR(xSemaphoreMotorControl, NULL);
}
static void reset_watch(void *args) {
    abort();
}

extern "C" void app_main() {
    if ((xSemaphoreCarUpdate = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE("app_main", "insufficient heap for semaphore");
        abort();
    }
    if ((xSemaphoreMotorControl = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE("app_main", "insufficient heap for semaphore");
        abort();
    }
    if ((xSemaphoreKaputt = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE("app_main", "insufficient heap for semaphore");
        abort();
    }
    if ((xSemaphoreFreedom = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE("app_main", "insufficient heap for semaphore");
        abort();
    }
    /* register interrupts */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)BOTTOM_RIGHT, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)BOTTOM_RIGHT, toggle_freedom, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)BOTTOM_LEFT, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)BOTTOM_LEFT, toggle_kaputt, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)TOP_RIGHT, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)TOP_RIGHT, toggle_motor, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)TOP_LEFT, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)TOP_LEFT, reset_watch, NULL));
    xTaskCreate(setup, "setup", 4096, NULL, configMAX_PRIORITIES, NULL);

    ESP_LOGI("app_main", "Starting scheduler from app_main()");
    vTaskStartScheduler();
    /* vTaskStartScheduler is blocking - this should never be reached */
    ESP_LOGE("app_main", "insufficient RAM! aborting");
    abort();
}