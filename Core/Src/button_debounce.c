/**
 * @file    button_debounce.c
 * @author  Andrei Dochkin
 * @brief   Debounce handling for mechanical buttons.
 *
 * This module provides a simple interface for debouncing button input.
 * It tracks raw and stable states, detects edge transitions, and supports
 * configurable debounce timing and active level (HIGH or LOW).
 */
#include "button_debounce.h"

void button_init(Button* btn, uint8_t active_state, uint32_t debounce_delay) {
    btn->active_state = active_state;
    btn->debounce_delay = debounce_delay;
    btn->stable_state = !active_state; // Start in released state
    btn->last_raw_state = !active_state;
    btn->last_time = 0;
}

void button_update(Button* btn, uint8_t current_state, uint32_t current_time) {
    // Reset debounce timer if state changed
    if (current_state != btn->last_raw_state) {
        btn->last_time = current_time;
        btn->last_raw_state = current_state;
    }

    // Only update stable state if debounce period has passed
    if ((current_time - btn->last_time) >= btn->debounce_delay) {
        btn->stable_state = current_state;
    }
}

uint8_t button_is_pressed(Button* btn) {
    return btn->stable_state == btn->active_state;
}

uint8_t button_just_pressed(Button* btn) {
    static uint8_t last_state = 0;
    uint8_t current_state = button_is_pressed(btn);
    uint8_t result = (current_state && !last_state);
    last_state = current_state;
    return result;
}

uint8_t button_just_released(Button* btn) {
    static uint8_t last_state = 0;
    uint8_t current_state = button_is_pressed(btn);
    uint8_t result = (!current_state && last_state);
    last_state = current_state;
    return result;
}
