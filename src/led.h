#pragma once

// Project Omni-Header
#include "config/all_headers.h"

const uint32_t LED_PIN = PICO_DEFAULT_LED_PIN;

void led_init() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

void led_on() {
    gpio_put(LED_PIN, 1);
}

// Turn LED off
void led_off() {
    gpio_put(LED_PIN, 0);
}

// Initialize PWM for LED pulsing
void led_pulse_init() {
    // Set LED pin to PWM mode
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    
    // Get PWM slice for the LED pin
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    
    // Set PWM frequency to ~1kHz
    pwm_set_clkdiv(slice_num, 125.f);
    pwm_set_wrap(slice_num, 999);
    
    // Enable PWM
    pwm_set_enabled(slice_num, true);
}

// Pulse the LED (breathing effect)
void led_pulse() {
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    uint channel = pwm_gpio_to_channel(LED_PIN);
    
    // Fade in
    for (int i = 0; i <= 1000; i += 5) {
        pwm_set_chan_level(slice_num, channel, i);
        sleep_ms(2);
    }
    
    // Fade out
    for (int i = 1000; i >= 0; i -= 5) {
        pwm_set_chan_level(slice_num, channel, i);
        sleep_ms(2);
    }
}