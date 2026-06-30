/**
 * @file    button_debounce.c
 * @author  Andrei Dochkin
 * @brief   Debounce handling for mechanical switches / buttons.
 *
 * This module provides a simple, stateful debouncer for mechanical inputs.
 * Edge flags (just_pressed / just_released) are computed inside
 * button_debounce_update() and are valid for one cycle only.
 */
#include "button_debounce.h"

/* -------------------------------------------------------------------------- */
/*   Public API                                                               */
/* -------------------------------------------------------------------------- */

void button_debounce_init(ButtonDebounce_t *p_btn,
                          uint8_t active_state,
                          uint32_t debounce_delay)
{
    if (p_btn == NULL) {
        return;                      /* Defensive — caller must supply valid ptr */
    }

    const uint8_t inactive = (uint8_t)(!active_state);

    p_btn->active_state    = active_state;
    p_btn->debounce_delay  = debounce_delay;
    p_btn->stable_state    = inactive;
    p_btn->last_raw_state  = inactive;
    p_btn->last_stable     = 0U;
    p_btn->last_time       = 0U;
    p_btn->just_pressed    = 0U;
    p_btn->just_released   = 0U;
}

void button_debounce_update(ButtonDebounce_t *p_btn,
                            uint8_t raw_state,
                            uint32_t current_time)
{
    if (p_btn == NULL) {
        return;
    }

    /* ---- Reset one-shot edge flags (valid for one cycle only) ---- */
    p_btn->just_pressed  = 0U;
    p_btn->just_released = 0U;

    /* ---- Debounce filter ---- */
    if (raw_state != p_btn->last_raw_state) {
        p_btn->last_time      = current_time;
        p_btn->last_raw_state = raw_state;
    }

    if ((current_time - p_btn->last_time) >= p_btn->debounce_delay) {
        p_btn->stable_state = raw_state;
    }

    /* ---- Edge detection (always safe now that edge flags are in the struct) ---- */
    {
        const uint8_t currently_pressed = (p_btn->stable_state == p_btn->active_state) ? 1U : 0U;

        if (currently_pressed && (p_btn->last_stable == 0U)) {
            p_btn->just_pressed = 1U;
        } else if ((currently_pressed == 0U) && p_btn->last_stable) {
            p_btn->just_released = 1U;
        }

        p_btn->last_stable = currently_pressed;
    }
}

uint8_t button_debounce_is_pressed(const ButtonDebounce_t *p_btn)
{
    if (p_btn == NULL) {
        return 0U;
    }
    return (p_btn->stable_state == p_btn->active_state) ? 1U : 0U;
}

uint8_t button_debounce_just_pressed(const ButtonDebounce_t *p_btn)
{
    if (p_btn == NULL) {
        return 0U;
    }
    return p_btn->just_pressed;
}

uint8_t button_debounce_just_released(const ButtonDebounce_t *p_btn)
{
    if (p_btn == NULL) {
        return 0U;
    }
    return p_btn->just_released;
}