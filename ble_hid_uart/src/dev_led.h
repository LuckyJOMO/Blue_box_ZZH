#ifndef DEV_LED_H
#define DEV_LED_H

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_GREEN_ON,
    LED_STATE_RED_ON,
    LED_STATE_RED_FAST_BLINK,
    LED_STATE_RED_SLOW_BLINK
} led_state_t;

void user_led_init(void);
void set_led_state(led_state_t state);

#endif // DEV_LED_H