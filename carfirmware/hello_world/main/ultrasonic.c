#include "ultrasonic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

SemaphoreHandle_t u_notify_semaphore;

PRIVILEGED_DATA static portMUX_TYPE xKernelLock = portMUX_INITIALIZER_UNLOCKED;

gptimer_handle_t us_trig_timer;

_Bool us_timer_callback(struct gptimer_t *, const gptimer_alarm_event_data_t *, void *){
    gptimer_stop(us_trig_timer);
    //set us trigger pin low
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    return 0;
}

void us_setup_timer(){
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    gptimer_new_timer(&timer_config, &us_trig_timer);
    gptimer_event_callbacks_t cbs = {
        .on_alarm = us_timer_callback,
    };
    gptimer_register_event_callbacks(us_trig_timer, &cbs, 0);
    gptimer_enable(us_trig_timer);
}

unsigned long us_rising_ts;
unsigned long us_falling_ts;

double us_last_distance;

double ultrasonic_get_distance(){
    return us_last_distance;
}

_Bool ultrasonic_process_data(){
    us_last_distance=(us_falling_ts-us_rising_ts)*0.017;
    if (u_notify_semaphore != NULL){
        _Bool higherPrioTaskWoken=0;
        // xSemaphoreGiveFromISR(u_notify_semaphore, &higherPrioTaskWoken);
        return higherPrioTaskWoken;
    }
    return 0;
}

void ultrasonic_irq_callback(void *arg){
    if (u_notify_semaphore != NULL){
        xSemaphoreGiveFromISR(u_notify_semaphore, 0);
    }
    //Check if rising or falling edge
    if (gpio_get_level(ULTRASONIC_ECHO_PIN)){
        //Rising edge
        us_rising_ts = esp_timer_get_time();
    } else {
        //Falling edge
        us_falling_ts = esp_timer_get_time();
        ultrasonic_process_data();
    }
}

void ultrasonic_configure(){

    // gpio_config_t io_conf = {};
    // io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // io_conf.pin_bit_mask = (1 << ULTRASONIC_ECHO_PIN);
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_up_en = 1;
    // gpio_config(&io_conf);

    gpio_set_intr_type(ULTRASONIC_ECHO_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_pull_mode(ULTRASONIC_ECHO_PIN, GPIO_PULLUP_ONLY);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ULTRASONIC_ECHO_PIN, ultrasonic_irq_callback, NULL);
    gpio_intr_enable(ULTRASONIC_ECHO_PIN);

    us_setup_timer();
}

void ultrasonic_trigger(SemaphoreHandle_t l_notify_semaphore){
    u_notify_semaphore = l_notify_semaphore;

    taskENTER_CRITICAL(&xKernelLock);
    //set trigger pin high
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    //start a timer for 10us
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 10, //10us
    };
    gptimer_set_alarm_action(us_trig_timer, &alarm_config);
    gptimer_set_raw_count(us_trig_timer, 0);
    gptimer_start(us_trig_timer);
    taskEXIT_CRITICAL(&xKernelLock);   
}