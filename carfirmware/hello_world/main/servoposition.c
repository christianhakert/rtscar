#include "servoposition.h"

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_log.h"

// #include "freertos/queue.h"
// #include "freertos/semphr.h"

PRIVILEGED_DATA static portMUX_TYPE xKernelLock = portMUX_INITIALIZER_UNLOCKED;

//Convert position to PWM pulse length between -45 and 45 degrees

int servo_pos=1000;
int servo_max=2000;
int servo_min=1000;

SemaphoreHandle_t notify_semaphore;
SemaphoreHandle_t next_notify;
SemaphoreHandle_t notify_sync_mutex;

void set_servo_position(double pos, SemaphoreHandle_t l_notify_semaphore){
    pos = pos * -1.0;
    if (pos < -45){
        pos = -45;
    }
    if (pos > 45){
        pos = 45;
    }
    xSemaphoreTake(notify_sync_mutex, portMAX_DELAY);
    servo_pos = (pos + 45) * (servo_max - servo_min) / 90 + servo_min;
    notify_semaphore = l_notify_semaphore;
    xSemaphoreGive(notify_sync_mutex);
}

gptimer_handle_t servo_timer;

_Bool timer_callback(struct gptimer_t *, const gptimer_alarm_event_data_t *, void *){
    gptimer_stop(servo_timer);
    //set servo pin low
    gpio_set_level(2, 0);

    _Bool higherPrioTaskWoken=0;
    if (next_notify != NULL){
        xSemaphoreGiveFromISR(next_notify, &higherPrioTaskWoken);
    }
    return higherPrioTaskWoken;
}

void setup_timer(){
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    gptimer_new_timer(&timer_config, &servo_timer);
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    gptimer_register_event_callbacks(servo_timer, &cbs, 0);
    gptimer_enable(servo_timer);
}

void launch_timer(unsigned long interval){
    //Interval is in microseconds
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = interval,
    };
    gptimer_set_alarm_action(servo_timer, &alarm_config);
    gptimer_set_raw_count(servo_timer, 0);
    gptimer_start(servo_timer);
}

void servo_position_task_function(){
    notify_sync_mutex = xSemaphoreCreateMutex();
    setup_timer();
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1){
        //Enter critical section
        xSemaphoreTake(notify_sync_mutex, portMAX_DELAY);
        next_notify=notify_semaphore;
        notify_semaphore=0;
        taskENTER_CRITICAL(&xKernelLock);
        //set the servo pin high
        gpio_set_level(2, 1);
        launch_timer(servo_pos);
        taskEXIT_CRITICAL(&xKernelLock);
        xSemaphoreGive(notify_sync_mutex);

        //Task should run ever 20 ms
        vTaskDelayUntil(&xLastWakeTime, 20/portTICK_PERIOD_MS);
    }
}