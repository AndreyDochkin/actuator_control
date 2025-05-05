/**
 * @file    button_debounce.h
 * @author  Andrei Dochkin
 * @brief   Debounce handling for mechanical buttons.
 *
 * This module provides a simple interface for debouncing button input.
 * It tracks raw and stable states, detects edge transitions, and supports
 * configurable debounce timing and active level (HIGH or LOW).
 */
#ifndef BUTTON_DEBOUNCE_H
#define BUTTON_DEBOUNCE_H

#include <stdint.h>

typedef struct {
    uint8_t stable_state;       // Debounced button state
    uint8_t last_raw_state;     // Previous undebounced state
    uint32_t last_time;         // Last state change timestamp
    uint32_t debounce_delay;    // Debounce time in milliseconds
    uint8_t active_state;       // Button pressed state (LOW/HIGH)
} Button;

// Initialize button parameters
void button_init(Button* btn, uint8_t active_state, uint32_t debounce_delay);

// Update button state - call this regularly
void button_update(Button* btn, uint8_t current_state, uint32_t current_time);

// Check if button is currently pressed
uint8_t button_is_pressed(Button* btn);

// Check if button was just pressed (edge detection)
uint8_t button_just_pressed(Button* btn);

// Check if button was just released (edge detection)
uint8_t button_just_released(Button* btn);

#endif
