#include "dev_led.h"
#include <devicetree.h>
#include <drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct device *led_dev;
static struct k_timer led_timer;
static led_state_t led_state = LED_STATE_OFF;
static bool led_blink_on = false;

// 定时器回调函数，用于控制LED状态
static void led_timer_cb(struct k_timer *timer)
{
    switch (led_state) {
        case LED_STATE_GREEN_ON: // 绿灯常亮
            gpio_pin_set_dt(&led_green, 0); // 绿灯亮
            gpio_pin_set_dt(&led_red, 1);  // 红灯灭
            break;
        case LED_STATE_RED_ON: // 红灯常亮
            gpio_pin_set_dt(&led_green, 1); // 绿灯灭
            gpio_pin_set_dt(&led_red, 0);  // 红灯亮
            break;
        case LED_STATE_RED_FAST_BLINK: // 红灯快闪
            gpio_pin_set_dt(&led_green, 1);
            gpio_pin_set_dt(&led_red, led_blink_on ? 0 : 1);
            led_blink_on = !led_blink_on;
            k_timer_start(&led_timer, K_MSEC(50), K_NO_WAIT);
            break;
        case LED_STATE_RED_SLOW_BLINK: // 红灯慢闪
            gpio_pin_set_dt(&led_green, 1);
            gpio_pin_set_dt(&led_red, led_blink_on ? 0 : 1);
            led_blink_on = !led_blink_on;
            k_timer_start(&led_timer, K_MSEC(200), K_NO_WAIT);
            break;
        default: // 默认，亮绿灯 正常工作
            gpio_pin_set_dt(&led_green, 0);
            gpio_pin_set_dt(&led_red, 1);
            break;
    }
}

// 设置LED状态
void set_led_state(led_state_t state)
{
    led_state = state;
    led_blink_on = false;
    // 立即刷新一次
    k_timer_start(&led_timer, K_NO_WAIT, K_MSEC(500));
}

// 初始化LED
void user_led_init(void)
{
    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_red, 1);  // 红灯灭
    gpio_pin_set_dt(&led_green, 0); // 绿灯亮

    k_timer_init(&led_timer, led_timer_cb, NULL);
    //set_led_state(LED_STATE_GREEN_ON);

    printk("user_led_init init done \n");
}