/**
 * @file    button_debounce.h
 * @author  Andrei Dochkin
 * @brief   Debounce handling for mechanical switches / buttons.
 *
 * This module provides a simple, stateful debouncer for mechanical inputs.
 * It tracks raw and stable states, detects rising/falling edge transitions,
 * and supports configurable debounce timing and active level (HIGH or LOW).
 *
 * @note    Edge flags (just_pressed / just_released) are computed inside
 *          #button_debounce_update() and are valid for one cycle only.
 *          Query functions are **pure** — they do NOT mutate state.
 */
#ifndef BUTTON_DEBOUNCE_H
#define BUTTON_DEBOUNCE_H

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*   Type definitions                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Debounced button state structure.
 * @note   All fields are initialised by #button_debounce_init().
 *         Users should never modify fields directly.
 */
typedef struct {
    uint8_t  stable_state;          /**< Debounced (stable) button state          */
    uint8_t  last_raw_state;        /**< Previous raw (undebounced) state        */
    uint32_t last_time;             /**< Tick timestamp of last raw transition   */
    uint32_t debounce_delay;        /**< Debounce window in ticks                */
    uint8_t  active_state;          /**< Logic level that means "pressed"        */
    uint8_t  last_stable;           /**< Previous stable-is-pressed (0 or 1)     */
    uint8_t  just_pressed;          /**< Set for one cycle after press detected  */
    uint8_t  just_released;         /**< Set for one cycle after release detected*/
} ButtonDebounce_t;

/* -------------------------------------------------------------------------- */
/*   Public API                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialise a debounce instance.
 * @param  p_btn           Pointer to the ButtonDebounce_t struct (out).
 * @param  active_state    0 (LOW) or 1 (HIGH) — the logic level that indicates
 *                         the button is physically pressed.
 * @param  debounce_delay  Debounce window in system ticks (typically ms).
 */
void button_debounce_init(ButtonDebounce_t *p_btn,
                          uint8_t active_state,
                          uint32_t debounce_delay);

/**
 * @brief  Periodic update — call this from the main loop (or timer callback).
 *         This function reads the raw input, applies the debounce filter,
 *         and computes the one-cycle edge flags (#just_pressed, #just_released).
 *
 * @param  p_btn          Pointer to the ButtonDebounce_t struct (in/out).
 * @param  raw_state      Current raw (undebounced) logic level from GPIO.
 * @param  current_time   Current system tick (e.g. HAL_GetTick()).
 */
void button_debounce_update(ButtonDebounce_t *p_btn,
                            uint8_t raw_state,
                            uint32_t current_time);

/**
 * @brief  Return 1 if the button is currently in the pressed (stable) state.
 * @param  p_btn  Pointer to the ButtonDebounce_t struct (read-only).
 * @return 1 if pressed, 0 otherwise.
 */
uint8_t button_debounce_is_pressed(const ButtonDebounce_t *p_btn);

/**
 * @brief  Return 1 if the button was just pressed (rising edge detected).
 *         The flag is valid for one cycle after #button_debounce_update().
 * @param  p_btn  Pointer to the ButtonDebounce_t struct (read-only).
 * @return 1 if a press edge was detected, 0 otherwise.
 */
uint8_t button_debounce_just_pressed(const ButtonDebounce_t *p_btn);

/**
 * @brief  Return 1 if the button was just released (falling edge detected).
 *         The flag is valid for one cycle after #button_debounce_update().
 * @param  p_btn  Pointer to the ButtonDebounce_t struct (read-only).
 * @return 1 if a release edge was detected, 0 otherwise.
 */
uint8_t button_debounce_just_released(const ButtonDebounce_t *p_btn);

#endif /* BUTTON_DEBOUNCE_H */
