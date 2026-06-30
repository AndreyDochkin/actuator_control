/**
 * @file    actuator_control.h
 * @brief   Actuator control interface for extend/shrink/homing operations.
 *
 * This module manages an actuator with two limit switches and provides
 * state-machine-based homing sequence, debounced input, and LED feedback.
 */

#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <stdint.h>
#include "button_debounce.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/*   Macros                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Convert milliseconds to system tick units.
 * @note   Assumes HAL_GetTickFreq() returns the SysTick frequency in Hz.
 */
#define MS_TO_TICKS(ms)     ((ms) * (HAL_GetTickFreq() / 1000U))
#define DEBOUNCE_TIME_MS    3U

/* -------------------------------------------------------------------------- */
/*   Enumerations                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Possible actuator states.
 */
typedef enum {
    ACTUATOR_IDLE       = 0,
    ACTUATOR_EXTENDING  = 1,
    ACTUATOR_SHRINKING  = 2,
    ACTUATOR_ERROR      = 3
} ActuatorState_t;

/**
 * @brief  Phases of the automatic homing sequence.
 */
typedef enum {
    HOMING_PHASE_INIT   = 0, /**< Initial phase — moving to shrink (home) position */
    HOMING_PHASE_EXTEND = 1, /**< Extending while measuring full travel time       */
    HOMING_PHASE_SHRINK = 2, /**< Shrinking while measuring full travel time       */
    HOMING_PHASE_MIDDLE = 3  /**< Moving to the calculated middle position         */
} HomingPhase_t;

/* -------------------------------------------------------------------------- */
/*   Structures                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Actuator hardware configuration.
 * @note   Populated at startup and never modified afterwards.
 */
typedef struct {
    uint8_t       extend_active_level;     /**< GPIO level that drives the extend relay   */
    uint8_t       shrink_active_level;     /**< GPIO level that drives the shrink relay   */
    uint32_t      debounce_time_ms;        /**< Switch debounce window in ticks           */
    GPIO_TypeDef* extend_control_port;     /**< GPIO port for extend control output       */
    uint16_t      extend_control_pin;      /**< GPIO pin  for extend control output       */
    GPIO_TypeDef* shrink_control_port;     /**< GPIO port for shrink control output       */
    uint16_t      shrink_control_pin;      /**< GPIO pin  for shrink control output       */
    GPIO_TypeDef* extend_switch_port;      /**< GPIO port for extend limit switch input   */
    uint16_t      extend_switch_pin;       /**< GPIO pin  for extend limit switch input   */
    GPIO_TypeDef* shrink_switch_port;      /**< GPIO port for shrink limit switch input   */
    uint16_t      shrink_switch_pin;       /**< GPIO pin  for shrink limit switch input   */
    GPIO_TypeDef* led_extend_port;         /**< GPIO port for extend-direction LED        */
    uint16_t      led_extend_pin;          /**< GPIO pin  for extend-direction LED        */
    GPIO_TypeDef* led_shrink_port;         /**< GPIO port for shrink-direction LED        */
    uint16_t      led_shrink_pin;          /**< GPIO pin  for shrink-direction LED        */
} ActuatorConfig_t;

/**
 * @brief  Actuator runtime control structure.
 * @note   All state is held here — no global variables in the module.
 */
typedef struct {
    ActuatorConfig_t config;                 /**< Hardware configuration (immutable at runtime)   */
    ActuatorState_t  state;                  /**< Current actuator state                           */
    uint8_t          is_homing;              /**< Non-zero while homing sequence is active        */
    HomingPhase_t    homing_phase;           /**< Current phase of the homing sequence            */
    uint32_t         homing_last_phase_end_time; /**< Tick timestamp when last homing phase ended  */
    uint32_t         extend_time;            /**< Full-extend travel time measured during homing  */
    uint32_t         shrink_time;            /**< Full-shrink travel time measured during homing  */
    Button           extend_switch;          /**< Debounced extend limit switch                   */
    Button           shrink_switch;          /**< Debounced shrink limit switch                   */
} ActuatorControl_t;

/* -------------------------------------------------------------------------- */
/*   Public API                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialise actuator control structure with a hardware configuration.
 * @param  act_cntrl  Pointer to the actuator control structure (out).
 * @param  config     Pointer to the read-only hardware configuration (in).
 */
void actuator_init(ActuatorControl_t *act_cntrl, const ActuatorConfig_t *config);

/**
 * @brief  Periodic update function — call this from the main loop.
 * @param  act_cntrl    Pointer to the actuator control structure.
 * @param  current_time Current system tick value from HAL_GetTick().
 */
void actuator_update(ActuatorControl_t *act_cntrl, uint32_t current_time);

/**
 * @brief  Start the homing sequence (non-blocking).
 * @param  act_cntrl  Pointer to the actuator control structure.
 */
void actuator_start_homing(ActuatorControl_t *act_cntrl);

/**
 * @brief  Get the current actuator state.
 * @param  act_cntrl  Pointer to the actuator control structure (read-only).
 * @return Current ActuatorState_t value.
 */
ActuatorState_t actuator_get_state(const ActuatorControl_t *act_cntrl);

/**
 * @brief  Check whether the homing sequence is still running.
 * @param  act_cntrl  Pointer to the actuator control structure (read-only).
 * @return Non-zero if homing is in progress, zero otherwise.
 */
uint8_t actuator_is_homing(const ActuatorControl_t *act_cntrl);

/**
 * @brief  Check if the actuator is in the error state.
 * @param  act_cntrl  Pointer to the actuator control structure (read-only).
 * @return Non-zero if in ACTUATOR_ERROR state, zero otherwise.
 */
uint8_t actuator_is_error(const ActuatorControl_t *act_cntrl);

/**
 * @brief  Command the actuator to extend.
 * @param  act_cntrl  Pointer to the actuator control structure.
 */
void actuator_extend(ActuatorControl_t *act_cntrl);

/**
 * @brief  Command the actuator to shrink.
 * @param  act_cntrl  Pointer to the actuator control structure.
 */
void actuator_shrink(ActuatorControl_t *act_cntrl);

/**
 * @brief  Stop the actuator (all outputs de-energised, state -> IDLE).
 * @param  act_cntrl  Pointer to the actuator control structure.
 */
void actuator_stop(ActuatorControl_t *act_cntrl);

#endif /* ACTUATOR_CONTROL_H */