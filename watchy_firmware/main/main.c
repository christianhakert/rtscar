#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include "espnow_example.h"
#include "nvs_flash.h"
#include <string.h>

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
//68:b6:b3:3e:34:e8
uint8_t broadcastAddress[] = {0x68, 0xb6, 0xb3, 0x3e, 0x34, 0xe8};

struct struct_message {
double d;
} __attribute__((packed));

void readMacAddress(){
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
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len){
  printf("Bytes received: \n");
  struct struct_message* data = (struct struct_message*) incomingData;
//   memcpy(&myData, incomingData, sizeof(myData));
  printf("Bytes received: %f",data->d);
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
static void example_wifi_init(void)
{
  printf("1\n");
    ESP_ERROR_CHECK(esp_netif_init());
  printf("2\n");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
  printf("3\n");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  printf("4\n");
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  printf("5\n");
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  printf("6\n");
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  printf("7\n");
    ESP_ERROR_CHECK( esp_wifi_start());
  printf("8\n");
    ESP_ERROR_CHECK( esp_wifi_set_channel(7, WIFI_SECOND_CHAN_NONE));
  printf("9\n");
#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  printf("Last Packet Send Status:\n");
  printf(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}




void app_main() {
    nvs_flash_init();
    example_wifi_init();
    // readMacAddress();
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);
    printf("cb add\n");

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

    while(1){
    struct struct_message myData;
    myData.d = 11;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    printf("Send Status: %d\n", result);
    printf("ESP-OK: %d\n", ESP_OK);


    vTaskDelay(200);
    }

    // ESP_LOGI("app_main", "Starting scheduler from app_main()");
    // vTaskStartScheduler();
    // // this should never be reached
    // ESP_LOGE("app_main", "insufficient RAM! aborting");
    // abort();
}