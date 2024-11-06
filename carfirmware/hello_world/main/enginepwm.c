#include "enginepwm.h"

#include "freertos/FreeRTOS.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "driver/ledc.h"

int engine_A_level=30;
int engine_B_level=50;

int engine_A_cnt=0;
int engine_B_cnt=0;

#define DUTY_MIN 800
#define DUTY_MAX 1024

void setup_engine_A(int level){
    engine_A_level=level;
    int duty=DUTY_MIN+(DUTY_MAX-DUTY_MIN)*level/100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
void setup_engine_B(int level){
    engine_B_level=level;
    int duty=DUTY_MIN+(DUTY_MAX-DUTY_MIN)*level/100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// gptimer_handle_t epwm_timer;

// _Bool epwm_timer_callback(struct gptimer_t *, const gptimer_alarm_event_data_t *, void *){
//     //start a timer for configured period
//     gptimer_alarm_config_t alarm_config = {
//         .alarm_count = ENGINE_PWM_PERIOD,
//     };
//     gptimer_set_alarm_action(epwm_timer, &alarm_config);
//     gptimer_set_raw_count(epwm_timer, 0);
    
//     if (engine_A_cnt==engine_A_level){
//         gpio_set_level(ENGINE_GPIO_A, 0);
//     } 
//     if (engine_A_cnt==0 && engine_A_level > 0) {
//         gpio_set_level(ENGINE_GPIO_A, 1);
//     }

//     if (engine_B_cnt==engine_B_level){
//         gpio_set_level(ENGINE_GPIO_B, 0);
//     }
//     if (engine_B_cnt==0 && engine_B_level > 0) {
//         gpio_set_level(ENGINE_GPIO_B, 1);
//     }

//     engine_A_cnt++;
//     engine_B_cnt++;

//     if (engine_A_cnt>=100){
//         engine_A_cnt=0;
//     }
//     if (engine_B_cnt>=100){
//         engine_B_cnt=0;
//     }

//     return 0;
// }

// void epwm_timer_setup(){
//     gptimer_config_t timer_config = {
//         .clk_src = GPTIMER_CLK_SRC_DEFAULT,
//         .direction = GPTIMER_COUNT_UP,
//         .resolution_hz = 1000000, // 1MHz, 1 tick=1us
//     };
//     gptimer_new_timer(&timer_config, &epwm_timer);
//     gptimer_event_callbacks_t cbs = {
//         .on_alarm = epwm_timer_callback,
//     };
//     gptimer_register_event_callbacks(epwm_timer, &cbs, 0);
//     gptimer_enable(epwm_timer);
// }

void setup_engine_pwm(){
    //Setup GPIOS
    gpio_set_direction(ENGINE_GPIO_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(ENGINE_GPIO_B, GPIO_MODE_OUTPUT);

    gpio_set_level(ENGINE_GPIO_A, 0);
    gpio_set_level(ENGINE_GPIO_B, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_A = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENGINE_GPIO_A,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_A);
    ledc_channel_config_t ledc_channel_B = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENGINE_GPIO_B,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_B);

    setup_engine_A(0);
    setup_engine_B(0);

    // //Setup PWM timer
    // epwm_timer_setup();

    // //start a timer for configured period
    // gptimer_alarm_config_t alarm_config = {
    //     .alarm_count = ENGINE_PWM_PERIOD,
    // };
    // gptimer_set_alarm_action(epwm_timer, &alarm_config);
    // gptimer_set_raw_count(epwm_timer, 0);
    // gptimer_start(epwm_timer);
}