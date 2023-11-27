/*

by Micaël Moreau

mini 4 levels elevator

*/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "ssd1306.h"
#include "esp_log.h"

// elevator number of levels
#define LEVELS 4

// I2C for ssd1306 display 
#define I2C_MASTER_SDA_IO 39
#define I2C_MASTER_SCL_IO 38
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// inputs - push buttons (indoor & outdoor)
#define IN_PB_LVL_1 GPIO_NUM_6
#define IN_PB_LVL_2 GPIO_NUM_7
#define IN_PB_LVL_3 GPIO_NUM_8
#define IN_PB_LVL_4 GPIO_NUM_9

// inputs - limit switches & macros
#define IN_LS_LVL_1 GPIO_NUM_10
#define LVL_1 gpio_get_level(IN_LS_LVL_1)
#define IN_LS_LVL_2 GPIO_NUM_11
#define LVL_2 gpio_get_level(IN_LS_LVL_2)
#define IN_LS_LVL_3 GPIO_NUM_12
#define LVL_3 gpio_get_level(IN_LS_LVL_3)
#define IN_LS_LVL_4 GPIO_NUM_13
#define LVL_4 gpio_get_level(IN_LS_LVL_4)
#define IN_LS_DOOR GPIO_NUM_14
#define DOOR gpio_get_level(IN_LS_DOOR) // if door is closed

#define DEBOUNCE_DELAY 1000    // delay for debouncing (limit switches or push buttons)

// outputs - variable speed, channel for it, and reverse relay
// lift
#define LIFT_PWM_PIN GPIO_NUM_41 // lift motor speed
#define LIFT_PWM_CHAN LEDC_CHANNEL_0
#define LIFT_UP_RELAY GPIO_NUM_42
#define BRAKE_RELEASE_RELAY GPIO_NUM_1
// door
#define DOOR_PWM_PIN GPIO_NUM_2 // door servo motor
#define DOOR_PWM_CHAN LEDC_CHANNEL_1

// outputs for levels light indicators (indoor & outdoor)
#define OUT_LED_LVL_1 GPIO_NUM_40
#define OUT_LED_LVL_2 GPIO_NUM_21
#define OUT_LED_LVL_3 GPIO_NUM_4
#define OUT_LED_LVL_4 GPIO_NUM_5


// global variables
uint8_t current_level = 0;                                  /*!< 1-4, 0=init or between 2 levels */
uint8_t next_level = 0;                                     /*!< 1-4, 0=init or idle */
typedef enum { HIGHER, LOWER, STOPPED } Direction_demands;  /*!< elevator answering demands to levels HIGHER, LOWER, or nothing (STOPPED) */
bool door_closing = false;                                  /*!< to consider door isr only when door is closing */
bool demands[LEVELS] = { false, false, false, false };
typedef enum { UP, DOWN, STOP_L } Direction_lift;           /*!< elevator lift UP, DOWN or STOP_L */
typedef enum { OPEN, CLOSE, STOP_D } Direction_door;        /*!< elevator door OPEN, CLOSE or STOP_D */
typedef enum { SLOW, FAST } Speed;                          /*!< elevator lift/door SLOW or FAST */
Direction_demands direction = STOPPED;                      /*!< current lift direction HIGHER, LOWER or STOPPED*/


// functions prototypes
// - utility and configuration functions
void delay(uint16_t duration_ms);
void configure_isr_pin(gpio_num_t gpio);
void i2c_master_init();
void display_init();
void lift_pwm_init();
void door_pwm_init();
// - application functions without loops
int next_demand();
void lift(Direction_lift dir, Speed spd);                   /*!< UP/DOWN/STOP_L, SLOW/FAST */
void door(Direction_door dir, Speed spd);                   /*!< OPEN/CLOSE/STOP_D, SLOW/FAST */
void display_level();
void display_stop();                                        /*!< remove arrows */
void update_indicators();
// - tasks functions
void main_task(void *params);
void level_isr_task(void *params);
void pushbutton_isr_task(void *params);
void door_isr_task(void *params);
void lift_down_task(void *params);
void display_down_task(void *params);
void lift_up_task(void *params);
void display_up_task(void *params);

// tasks handles
TaskHandle_t main_task_handle = NULL;
TaskHandle_t lift_down_task_handle = NULL;
TaskHandle_t display_down_task_handle = NULL;
TaskHandle_t lift_up_task_handle = NULL;
TaskHandle_t display_up_task_handle = NULL;
TaskHandle_t door_closing_task_handle = NULL;
TaskHandle_t door_opening_task_handle = NULL;

// semaphores for interrrupts
SemaphoreHandle_t level_isr_semaphore = NULL;
SemaphoreHandle_t pushbutton_isr_semaphore = NULL;
SemaphoreHandle_t door_isr_semaphore = NULL;
// semaphores for sub-tasks calls waiting
SemaphoreHandle_t lift_down_semaphore = NULL;
SemaphoreHandle_t lift_up_semaphore = NULL;
SemaphoreHandle_t door_closed_semaphore = NULL;
SemaphoreHandle_t up_to_level_done_semaphore = NULL;
SemaphoreHandle_t down_to_level_done_semaphore = NULL;

// ssd1306 display handle
ssd1306_handle_t ssd1306_dev = NULL;


// interrupts functions

void IRAM_ATTR level_isr_handler(void* arg) {

    static uint32_t last_level_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();
    if ((current_time - last_level_isr_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY)) {
        current_level = (int)arg;
        xSemaphoreGiveFromISR(level_isr_semaphore, NULL);
        last_level_isr_time = current_time;
    }
    portYIELD_FROM_ISR();
}


void IRAM_ATTR pushbutton_isr_handler(void* arg) {

    static uint32_t last_pb_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();
    if ((current_time - last_pb_isr_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY)) {
        int level = (int)arg;
        if (current_level != level) {
            demands[level - 1] = true;
            xSemaphoreGiveFromISR(pushbutton_isr_semaphore, NULL);
            last_pb_isr_time = current_time;
        }
    }
    portYIELD_FROM_ISR();

}


void IRAM_ATTR door_isr_handler(void* arg) {

    static uint32_t last_door_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();
    if ((current_time - last_door_isr_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY)) {
        if (door_closing) { xSemaphoreGiveFromISR(door_isr_semaphore, NULL); }
        last_door_isr_time = current_time;
    }
    portYIELD_FROM_ISR();

}


void app_main() {

    // initialize inputs interrupt service routine
    gpio_install_isr_service(0);

    // I2C for ssd1306 display
    i2c_master_init();
    display_init();

    // GPIO settings
    // interrupts inputs
    gpio_reset_pin(IN_LS_LVL_1);
    configure_isr_pin(IN_LS_LVL_1);                                         // levels limit switches
    gpio_isr_handler_add(IN_LS_LVL_1, level_isr_handler, (void*) 1);        // update current level
    gpio_reset_pin(IN_LS_LVL_2);
    configure_isr_pin(IN_LS_LVL_2);
    gpio_isr_handler_add(IN_LS_LVL_2, level_isr_handler, (void*) 2);
    gpio_reset_pin(IN_LS_LVL_3);
    configure_isr_pin(IN_LS_LVL_3);
    gpio_isr_handler_add(IN_LS_LVL_3, level_isr_handler, (void*) 3);
    gpio_reset_pin(IN_LS_LVL_4);
    configure_isr_pin(IN_LS_LVL_4);
    gpio_isr_handler_add(IN_LS_LVL_4, level_isr_handler, (void*) 4);

    gpio_reset_pin(IN_PB_LVL_1);
    configure_isr_pin(IN_PB_LVL_1);                                         // push buttons in/outdoors
    gpio_isr_handler_add(IN_PB_LVL_1, pushbutton_isr_handler, (void*) 1);   // update levels demands array
    gpio_reset_pin(IN_PB_LVL_2);
    configure_isr_pin(IN_PB_LVL_2);
    gpio_isr_handler_add(IN_PB_LVL_2, pushbutton_isr_handler, (void*) 2);
    gpio_reset_pin(IN_PB_LVL_3);
    configure_isr_pin(IN_PB_LVL_3);
    gpio_isr_handler_add(IN_PB_LVL_3, pushbutton_isr_handler, (void*) 3);
    gpio_reset_pin(IN_PB_LVL_4);
    configure_isr_pin(IN_PB_LVL_4);
    gpio_isr_handler_add(IN_PB_LVL_4, pushbutton_isr_handler, (void*) 4);

    gpio_reset_pin(IN_LS_DOOR);
    configure_isr_pin(IN_LS_DOOR);                                          // door closed
    gpio_isr_handler_add(IN_LS_DOOR, door_isr_handler, (void*) IN_LS_DOOR);

    // outputs
    lift_pwm_init();             // lift up/down speed
    door_pwm_init();             // door open/close servo
    
    gpio_reset_pin(BRAKE_RELEASE_RELAY);
    gpio_set_direction(BRAKE_RELEASE_RELAY, GPIO_MODE_OUTPUT);      // relay to release lift brake (terminals shorting) 
    gpio_reset_pin(LIFT_UP_RELAY);
    gpio_set_direction(LIFT_UP_RELAY, GPIO_MODE_OUTPUT);      // reverse relay to lift up
    gpio_reset_pin(OUT_LED_LVL_1);
    gpio_set_direction(OUT_LED_LVL_1, GPIO_MODE_OUTPUT);    // in/outdoors indicators
    gpio_reset_pin(OUT_LED_LVL_2);
    gpio_set_direction(OUT_LED_LVL_2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT_LED_LVL_3);
    gpio_set_direction(OUT_LED_LVL_3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT_LED_LVL_4);
    gpio_set_direction(OUT_LED_LVL_4, GPIO_MODE_OUTPUT);

    // semaphore instances to their handle
    level_isr_semaphore = xSemaphoreCreateBinary();
    pushbutton_isr_semaphore = xSemaphoreCreateBinary();
    door_isr_semaphore = xSemaphoreCreateBinary();
    door_closed_semaphore = xSemaphoreCreateBinary();
    lift_down_semaphore = xSemaphoreCreateBinary();
    lift_up_semaphore = xSemaphoreCreateBinary();
    up_to_level_done_semaphore = xSemaphoreCreateBinary();
    down_to_level_done_semaphore = xSemaphoreCreateBinary();
    
    // some sub tasks used by main task
    xTaskCreate(level_isr_task, "level_isr_task", 2048, NULL, 1, NULL);
    xTaskCreate(pushbutton_isr_task, "pushbutton_isr_task", 2048, NULL, 1, NULL);
    xTaskCreate(door_isr_task, "door_isr_task", 2048, NULL, 1, NULL);

    xTaskCreate(display_down_task, "display_down_task", 2048, NULL, 1, &display_down_task_handle);
    vTaskSuspend(display_down_task_handle);

    xTaskCreate(display_up_task, "display_up_task", 2048, NULL, 1, &display_up_task_handle);
    vTaskSuspend(display_up_task_handle);

    // machine start-up, close door & bring elevator at 1st level
    display_stop();
    delay(1200);
    printf("machine start-up...\n");
    if (!DOOR) { 
        door(CLOSE, SLOW);
        while (!DOOR) { delay(10); }
        door(STOP_D, 0);
    }
    if (!LVL_1) { 
        lift(DOWN, SLOW);
        while (current_level != 1) { delay(10); }
        lift(STOP_L, 0);
    }
    xTaskCreate(main_task, "main_task", 2048, NULL, 1, &main_task_handle);

}


// utility and configuration functions

void delay(uint16_t duration) {

    vTaskDelay(duration / portTICK_PERIOD_MS);

}


void configure_isr_pin(gpio_num_t gpio) {

    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&conf);

}


void i2c_master_init() {

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);

}


void display_init() {

    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

}


void lift_pwm_init() {

    ledc_timer_config_t timer_config = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 100,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_config);
    delay(10);

    ledc_channel_config_t channel_config = {
        .channel = LIFT_PWM_CHAN,
        .duty = 0,
        .gpio_num = LIFT_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
        .flags.output_invert = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&channel_config);
    delay(10);

}


void door_pwm_init() {

    ledc_timer_config_t timer_config = {
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config(&timer_config);
    delay(10);

    ledc_channel_config_t channel_config = {
        .channel = DOOR_PWM_CHAN,
        .duty = 0,
        .gpio_num = DOOR_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1,
        .hpoint = 0,
        .flags.output_invert = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&channel_config);
    delay(10);

}


// application functions without loops

int next_demand() {

    if (direction == HIGHER) {
        for (int i = current_level; i < LEVELS; i++) {
            if (demands[i]) { return i + 1; }
        }
    }
    else if (direction == LOWER) {
        for (int i = current_level - 2; i >= 0; i--) {
            if (demands[i]) { return i + 1; }
        }
    }
    for (int i = 0; i < LEVELS; i++) {
        if (demands[i]) { return i + 1; }
    }
    direction = STOPPED;
    return 0;

}


void lift(Direction_lift dir, Speed spd) {

    ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_t chan = LIFT_PWM_CHAN;
    switch (dir) {
        case UP: {
            gpio_set_level(BRAKE_RELEASE_RELAY, 1);
            gpio_set_level(LIFT_UP_RELAY, 1);
            ledc_set_duty(mode, chan, spd == SLOW ? 700 : 1024);
            current_level = 0;
            printf("DEBUG: lift() case UP.\n");
            break;
        }
        case DOWN: {
            gpio_set_level(BRAKE_RELEASE_RELAY, 1);
            ledc_set_duty(mode, chan, spd == SLOW ? 500 : 1024);
            current_level = 0;
            printf("DEBUG: lift() case DOWN.\n");
            break;
        }
        case STOP_L: {
            gpio_set_level(BRAKE_RELEASE_RELAY, 0);
            gpio_set_level(LIFT_UP_RELAY, 0);
            ledc_set_duty(mode, chan, 0);
            printf("DEBUG: lift() case STOP_L.\n");
            break;
        }
    }
    ledc_update_duty(mode, chan);
    
}


void door(Direction_door dir, Speed spd) {

    ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_t chan = DOOR_PWM_CHAN;
    switch (dir) {
        case OPEN: {
            ledc_set_duty(mode, chan, spd == SLOW ? 1232 : 1332); // 1232, 1332
            printf("DEBUG: door() case OPEN.\n");
            break;
        }
        case CLOSE: {
            if (DOOR) { return; }
            door_closing = true;
            ledc_set_duty(mode, chan, spd == SLOW ? 1180 : 1080); // 1180, 1080
            printf("DEBUG: door() case CLOSE.\n");
            break;
        }
        case STOP_D: {
            ledc_set_duty(mode, chan, 1205);
            printf("DEBUG: door() case STOP_D.\n");
            break;
        }
    }
    ledc_update_duty(mode, chan);

}


void display_level() {

    ssd1306_fill_rectangle(ssd1306_dev, 39, 31, 52, 49, 1);
    ssd1306_fill_rectangle(ssd1306_dev, 40, 32, 51, 48, 0);
    ssd1306_draw_char(ssd1306_dev, 42, 32, '0' + current_level, 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);

}


void display_stop() {

    ssd1306_fill_rectangle(ssd1306_dev, 65, 16, 95, 63, 0);
    ssd1306_refresh_gram(ssd1306_dev);

}


void update_indicators() {

    gpio_set_level(OUT_LED_LVL_1, demands[0]);
    gpio_set_level(OUT_LED_LVL_2, demands[1]);
    gpio_set_level(OUT_LED_LVL_3, demands[2]);
    gpio_set_level(OUT_LED_LVL_4, demands[3]);

}


// tasks functions

void main_task(void *params) {

    for (;;) { // freertos task loop
        for (;;) {
            next_level = next_demand();
            if (!next_level) { break; }
            if (next_level > current_level) {
                printf("DEBUG: main_task() next_level > current_level.\n");
                xTaskCreate(lift_up_task, "lift_up_task", 2048, NULL, 1, &lift_up_task_handle);
                xSemaphoreTake(lift_up_semaphore, portMAX_DELAY);
                printf("DEBUG: main_task() received lift_up_semaphore (back from lift_up_task()).\n");
                xSemaphoreTake(up_to_level_done_semaphore, portMAX_DELAY);
                printf("DEBUG: main_task() received up_to_level_done_semaphore (back from level_isr_task()).\n");
                break;
            }
            else {
                printf("DEBUG: main_task() next_level < current_level.\n");
                xTaskCreate(lift_down_task, "lift_down_task", 2048, NULL, 1, &lift_down_task_handle);
                xSemaphoreTake(lift_down_semaphore, portMAX_DELAY);
                printf("DEBUG: main_task() received lift_down_semaphore (back from lift_down_task()).\n");
                xSemaphoreTake(down_to_level_done_semaphore, portMAX_DELAY);
                printf("DEBUG: main_task() received down_to_level_done_semaphore (back from level_isr_task()).\n");
                break;
            }
        }
        delay(10);
    }

}


void level_isr_task(void *params) {

    for (;;) { // freertos task loop
        if (xSemaphoreTake(level_isr_semaphore, portMAX_DELAY) == pdTRUE) {
            printf("DEBUG: level_isr_task() received level_isr_semaphore from interrupt.\n");
            display_level();
            if (demands[current_level - 1]) {
                lift(STOP_L, 0);
                vTaskSuspend(direction == HIGHER ? display_up_task_handle : display_down_task_handle);
                display_stop();
                demands[current_level - 1] = false;
                update_indicators();
                delay(500);
                door(OPEN, SLOW);
                delay(2800);
                door(STOP_D, 0);
                delay(1500);
                door(CLOSE, SLOW);
                xSemaphoreTake(door_closed_semaphore, portMAX_DELAY);
                delay(500);
                if (direction == HIGHER) {
                    printf("DEBUG: level_isr_task() giving up_to_level_done_semaphore.\n");
                    xSemaphoreGive(up_to_level_done_semaphore); 
                }
                else { 
                    vTaskSuspend(display_down_task_handle);
                    display_stop();
                    printf("DEBUG: level_isr_task() giving down_to_level_done_semaphore.\n");                    
                    xSemaphoreGive(down_to_level_done_semaphore); 
                }
            }
        }
    }
    
}


void pushbutton_isr_task(void *params) {

    for (;;) { // freertos task loop
        if (xSemaphoreTake(pushbutton_isr_semaphore, portMAX_DELAY) == pdTRUE) {
            printf("DEBUG: pushbutton_isr_task().\n");
            update_indicators();
        }
    }

}


void door_isr_task(void *params) {

    for (;;) { // freertos task loop
        if (xSemaphoreTake(door_isr_semaphore, portMAX_DELAY) == pdTRUE) {
            printf("DEBUG: door_isr_task().\n");
            door(STOP_D, 0);
            door_closing = false;
            xSemaphoreGive(door_closed_semaphore);
        }
    }

}


void lift_down_task(void *params) {

    for (;;) { // freertos task loop
        vTaskResume(display_down_task_handle);
        printf("DEBUG: lift_down_task().\n");
        lift(DOWN, SLOW);
        current_level = 0;
        direction = LOWER;
        xSemaphoreGive(lift_down_semaphore);
        vTaskDelete(NULL);
    }

}


void display_down_task(void *params) {

    for(;;) { // freertos task loop
        ssd1306_draw_line(ssd1306_dev, 66, 17, 76, 33);
        ssd1306_draw_line(ssd1306_dev, 76, 33, 86, 17);
        ssd1306_draw_line(ssd1306_dev, 66, 18, 76, 34);
        ssd1306_draw_line(ssd1306_dev, 76, 34, 86, 18);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(100);
        ssd1306_draw_line(ssd1306_dev, 66, 31, 76, 47);
        ssd1306_draw_line(ssd1306_dev, 76, 47, 86, 31);
        ssd1306_draw_line(ssd1306_dev, 66, 32, 76, 48);
        ssd1306_draw_line(ssd1306_dev, 76, 48, 86, 32);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(100);
        ssd1306_draw_line(ssd1306_dev, 66, 45, 76, 62);
        ssd1306_draw_line(ssd1306_dev, 76, 62, 86, 45);
        ssd1306_draw_line(ssd1306_dev, 66, 46, 76, 63);
        ssd1306_draw_line(ssd1306_dev, 76, 63, 86, 46);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(1000);
        ssd1306_fill_rectangle(ssd1306_dev, 66, 16, 95, 63, 0);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(100);
    }

}


void lift_up_task(void *params) {

    for (;;) { // freertos task loop
        printf("DEBUG: lift_up_task().\n");
        lift(UP, SLOW);
        current_level = 0;
        direction = HIGHER;
        vTaskResume(display_up_task_handle);
        xSemaphoreGive(lift_up_semaphore);
        vTaskDelete(NULL);
    }

}


void display_up_task(void *params) {

    for(;;) { // freertos task loop
        ssd1306_draw_line(ssd1306_dev, 66, 62, 76, 45);
        ssd1306_draw_line(ssd1306_dev, 76, 45, 86, 62);
        ssd1306_draw_line(ssd1306_dev, 66, 63, 76, 46);
        ssd1306_draw_line(ssd1306_dev, 76, 46, 86, 63);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(100);
        ssd1306_draw_line(ssd1306_dev, 66, 47, 76, 31);
        ssd1306_draw_line(ssd1306_dev, 76, 31, 86, 47);
        ssd1306_draw_line(ssd1306_dev, 66, 48, 76, 32);
        ssd1306_draw_line(ssd1306_dev, 76, 32, 86, 48);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(100);
        ssd1306_draw_line(ssd1306_dev, 66, 33, 76, 17);
        ssd1306_draw_line(ssd1306_dev, 76, 17, 86, 33);
        ssd1306_draw_line(ssd1306_dev, 66, 34, 76, 18);
        ssd1306_draw_line(ssd1306_dev, 76, 18, 86, 34);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(1000);
        ssd1306_fill_rectangle(ssd1306_dev, 66, 16, 95, 63, 0);
        ssd1306_refresh_gram(ssd1306_dev);
        delay(100);
    }

}