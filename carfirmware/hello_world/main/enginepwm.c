#include "enginepwm.h"

#include "freertos/FreeRTOS.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

#include "driver/gpio.h"

int engine_A_level=30;
int engine_B_level=50;

int engine_A_cnt=0;
int engine_B_cnt=0;

void setup_engine_A(int level){
    engine_A_level=level;
}
void setup_engine_B(int level){
    engine_B_level=level;
}

gptimer_handle_t epwm_timer;

_Bool epwm_timer_callback(struct gptimer_t *, const gptimer_alarm_event_data_t *, void *){
    //start a timer for configured period
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = ENGINE_PWM_PERIOD,
    };
    gptimer_set_alarm_action(epwm_timer, &alarm_config);
    gptimer_set_raw_count(epwm_timer, 0);
    
    if (engine_A_cnt<engine_A_level){
        gpio_set_level(ENGINE_GPIO_A, 1);
    } else {
        gpio_set_level(ENGINE_GPIO_A, 0);
    }
    if (engine_B_cnt<engine_B_level){
        gpio_set_level(ENGINE_GPIO_B, 1);
    } else {
        gpio_set_level(ENGINE_GPIO_B, 0);
    }
    engine_A_cnt++;
    engine_B_cnt++;

    if (engine_A_cnt>=100){
        engine_A_cnt=0;
    }
    if (engine_B_cnt>=100){
        engine_B_cnt=0;
    }

    return 0;
}

void epwm_timer_setup(){
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    gptimer_new_timer(&timer_config, &epwm_timer);
    gptimer_event_callbacks_t cbs = {
        .on_alarm = epwm_timer_callback,
    };
    gptimer_register_event_callbacks(epwm_timer, &cbs, 0);
    gptimer_enable(epwm_timer);
}

void setup_engine_pwm(){
    //Setup GPIOS
    gpio_set_direction(ENGINE_GPIO_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(ENGINE_GPIO_B, GPIO_MODE_OUTPUT);

    gpio_set_level(ENGINE_GPIO_A, 0);
    gpio_set_level(ENGINE_GPIO_B, 0);

    //Setup PWM timer
    epwm_timer_setup();

    //start a timer for configured period
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = ENGINE_PWM_PERIOD,
    };
    gptimer_set_alarm_action(epwm_timer, &alarm_config);
    gptimer_set_raw_count(epwm_timer, 0);
    gptimer_start(epwm_timer);
}