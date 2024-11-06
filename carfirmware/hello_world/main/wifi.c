#include "wifi.h"
#include "freertos/FreeRTOS.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "espnow_example.h"

#include "nvs_flash.h"

#include <string.h>

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

//paired watch
// uint8_t broadcastAddress[] = {0x4c, 0x75, 0x25, 0xa7, 0x5a, 0x8c};
// 4c:75:25:a7:60:48
uint8_t broadcastAddress[] = {0x4c, 0x75, 0x25, 0xa7, 0x60, 0x48};


struct struct_message {
  double d;
} __attribute__((packed));

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  printf("Last Packet Send Status: ");
  printf(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n" : "Delivery Fail\n");
}

#include "../../../interface.h"
#include "udar.h"
#include "steering.h"

volatile int g_kaputt=0;

void kaputt_task(){
    while(1){
        if(g_kaputt){
            printf("Kaputt\n");
        }
        else{
            vTaskDelay(1000);
        }
    }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len){
    struct watch2car* data = (struct watch2car*) incomingData;
    if (data->magic==0x42){
        printf("Received data\n");
        printf("Set speed to %d\n",data->speed);
        printf("Set direction to %f\n",data->direction);
        update_steering_input(data->speed,data->direction);

        if(data->kaputt){
            printf("Kaputt\n");
            g_kaputt=1;
        } else {
            g_kaputt=0;
        }
    }
}

// struct car2watch {
// unsigned long magic; //0x42
// unsigned int angle_granularity; // how wide are sections
// double udar_map[90]; // per angle section, how far is wall
// };

struct car2watch myData;

void udar_map_send_task(){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1){
        //Send the map

        //prepare udar map to be sent
        myData.magic=0x42;
        myData.angle_granularity=SCAN_ANGLE_INTV;


        double *udar_map=get_udar_map();
        //search max value in udar map
        double max=200;
        // for (int i=0; i<UDAR_MAP_SIZE; i++){
        //     if (udar_map[i]>max){
        //         max=udar_map[i];
        //     }
        // }

        //copy in udar data
        
        for (int i=0; i<UDAR_MAP_SIZE; i++){
            myData.udar_map[i]=udar_map[i]*100.0/max;
        }

        printf("Sending data\n");
        printf("Max data length is %d\n", ESP_NOW_MAX_DATA_LEN);
        printf("My data length is %d\n", sizeof(myData));

        esp_err_t result = esp_now_send(broadcastAddress, (void *) &myData, ESP_NOW_MAX_DATA_LEN);

        printf("result of send %d\n", result);

        vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_PERIOD_MS);
    }
}

void setup_wifi(){
    //WIFI Stuff
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

    //EOF WIFI Stuff
}