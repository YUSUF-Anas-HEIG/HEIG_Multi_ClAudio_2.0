
 
#include "app_state.h"
 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
 
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
 
LOG_MODULE_REGISTER(app_state, LOG_LEVEL_INF);
 
 
#define LONG_PRESS_MS       2000U  
#define BLINK_PERIOD_MS      250U   
#define DISPLAY_MS          2000U   
 
/* Switch: P0.16, active low (GND when pressed, pull-up to VCC) */
static const struct gpio_dt_spec sw = GPIO_DT_SPEC_GET(DT_NODELABEL(button_0), gpios);
 
/* WS2812B strip on P0.15 via "led-strip" alias in app.overlay */
static const struct device *led_strip_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(led_strip));
 
#define STRIP_NUM_LEDS      5U   
 
static struct led_rgb strip_buf[STRIP_NUM_LEDS];
 
typedef enum {
    STATE_LOW_POWER,
    STATE_SCANNING,
    STATE_STREAMING,
} app_state_t;
 
static volatile app_state_t current_state = STATE_LOW_POWER;
static volatile bool         disconnect_requested;
 
static K_SEM_DEFINE(sem_long_press, 0, 1);
 
static volatile int64_t press_start_ms;
static volatile bool    press_active;
 

static struct gpio_callback sw_cb_data;
 

static struct k_timer blink_timer;
static volatile bool  blink_on;
static volatile bool  blink_running;
 

static const struct led_rgb COLOR_OFF   = {.r = 0,   .g = 0,   .b = 0};
static const struct led_rgb COLOR_BLUE  = {.r = 0,   .g = 0,   .b = 128};
static const struct led_rgb COLOR_GREEN = {.r = 0,   .g = 128, .b = 0};
static const struct led_rgb COLOR_RED   = {.r = 128, .g = 0,   .b = 0};
 

static void led_set(const struct led_rgb *color)
{
    for (size_t i = 0; i < STRIP_NUM_LEDS; i++) {
        strip_buf[i] = *color;
    }
    led_strip_update_rgb(led_strip_dev, strip_buf, STRIP_NUM_LEDS);
}
 
static void led_off(void)
{
    led_set(&COLOR_OFF);
}
 

static void blink_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
 
    if (!blink_running) {
        return;
    }
 
    blink_on = !blink_on;
    led_set(blink_on ? &COLOR_BLUE : &COLOR_OFF);
}
 
static void blink_start(void)
{
    blink_on      = true;
    blink_running = true;
    led_set(&COLOR_BLUE);
    k_timer_start(&blink_timer,
                  K_MSEC(BLINK_PERIOD_MS),
                  K_MSEC(BLINK_PERIOD_MS));
}
 
static void blink_stop(void)
{
    blink_running = false;
    k_timer_stop(&blink_timer);
    led_off();
}
 

static void sw_isr(const struct device *dev, struct gpio_callback *cb,
                   uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
 
   
    int raw = gpio_pin_get_raw(sw.port, sw.pin);
 
    if (raw == 0) {
        press_start_ms = k_uptime_get();
        press_active   = true;
    } else {
        if (!press_active) {
            return;
        }
        press_active = false;
 
        int64_t held_ms = k_uptime_get() - press_start_ms;
 
        if (held_ms >= LONG_PRESS_MS) {
            if (current_state == STATE_LOW_POWER) {
                k_sem_give(&sem_long_press);
            } else if (current_state == STATE_STREAMING) {
                disconnect_requested = true;
                LOG_INF("Long press while streaming — requesting disconnect");
            }
            
        }
    }
}
 
/*PUBLIC FUNCTIONS*/
int app_state_init(void)
{
    int err;

    led_strip_dev = DEVICE_DT_GET(DT_ALIAS(led_strip));
    if (!device_is_ready(led_strip_dev)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }
    led_off();
 
    if (!gpio_is_ready_dt(&sw)) {
        LOG_ERR("Switch GPIO not ready");
        return -ENODEV;
    }
 
    err = gpio_pin_configure_dt(&sw, GPIO_INPUT | GPIO_PULL_UP);
    if (err != 0) {
        LOG_ERR("Failed to configure switch pin: %d", err);
        return err;
    }
 
    err = gpio_pin_interrupt_configure_dt(&sw, GPIO_INT_EDGE_BOTH);
    if (err != 0) {
        LOG_ERR("Failed to configure switch interrupt: %d", err);
        return err;
    }
 
    gpio_init_callback(&sw_cb_data, sw_isr, BIT(sw.pin));
    gpio_add_callback(sw.port, &sw_cb_data);

    k_timer_init(&blink_timer, blink_timer_cb, NULL);
 
    current_state        = STATE_LOW_POWER;
    disconnect_requested = false;
 
    LOG_INF("App state machine initialised (switch P0.16, LED P0.15)");
    return 0;
}
 
void app_state_wait_for_start(void)
{

    k_sem_reset(&sem_long_press);
    disconnect_requested = false;
    current_state        = STATE_LOW_POWER;
 
    led_off();
 
    LOG_INF("Entering low-power — waiting for 2-second press");

    k_sem_take(&sem_long_press, K_FOREVER);
 
    current_state = STATE_SCANNING;
    blink_start();
 
    LOG_INF("Long press detected — starting BT scan, blue blink running");
}
 
void app_state_notify_connected(void)
{
    blink_stop();
    current_state = STATE_STREAMING;
 
    led_set(&COLOR_GREEN);
    k_sleep(K_MSEC(DISPLAY_MS));
    led_off();
 
    LOG_INF("Connected — green display done, LED off, streaming");
}
 
void app_state_notify_scan_timeout(void)
{
    blink_stop();
    current_state = STATE_LOW_POWER;
 
    led_set(&COLOR_RED);
    k_sleep(K_MSEC(DISPLAY_MS));
    led_off();
 
    LOG_INF("Scan timeout — red display done, returning to low power");
}
 
bool app_state_disconnect_requested(void)
{
    return disconnect_requested;
}