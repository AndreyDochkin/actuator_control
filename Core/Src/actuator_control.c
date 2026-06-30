/**
 * @file    actuator_control.c
 * @brief   Actuator control state machine with non-blocking homing sequence.
 *
 * Implements the full actuator lifecycle: extend, shrink, stop, and an
 * automatic homing routine that measures travel times and parks the actuator
 * at the midpoint.
 *
 * @note    GPIO port pointers arrive as `void*` from the HAL-agnostic config.
 *          They are cast to `GPIO_TypeDef*` at the last responsible moment.
 */

#include "actuator_control.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"          /* HAL_GPIO_WritePin / ReadPin (only in .c) */

/* -------------------------------------------------------------------------- */
/*   Private constants                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Homing safety timeout.
 *         If a limit switch is not pressed within this many ticks after
 *         starting a homing move, the actuator enters the error state.
 */
#define HOMING_TIMEOUT_MS   10000U

/* -------------------------------------------------------------------------- */
/*   Private helpers — forward declarations                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Read and debounce both limit switches.
 * @param  p_act        Actuator control structure.
 * @param  current_time Current tick count.
 */
static void update_switches(ActuatorControl_t *p_act, uint32_t current_time);

/**
 * @brief  Run one iteration of the homing state machine.
 * @param  p_act        Actuator control structure.
 * @param  current_time Current tick count.
 */
static void handle_homing_sequence(ActuatorControl_t *p_act, uint32_t current_time);

/**
 * @brief  Set a GPIO output pair and the two direction LEDs.
 * @param  p_act         Actuator control structure.
 * @param  extend_active Value for the extend control pin.
 * @param  shrink_active Value for the shrink control pin.
 * @param  led_extend    Value for the extend-direction LED.
 * @param  led_shrink    Value for the shrink-direction LED.
 */
static void set_outputs(const ActuatorControl_t *p_act,
                        uint8_t extend_active,
                        uint8_t shrink_active,
                        uint8_t led_extend,
                        uint8_t led_shrink);

/* -------------------------------------------------------------------------- */
/*   Public API                                                               */
/* -------------------------------------------------------------------------- */

void actuator_init(ActuatorControl_t *p_act, const ActuatorConfig_t *p_cfg)
{
    if ((p_act == NULL) || (p_cfg == NULL)) {
        return;
    }

    p_act->config                      = *p_cfg;
    p_act->state                       = ACTUATOR_IDLE;
    p_act->is_homing                   = 0U;
    p_act->homing_phase                = HOMING_PHASE_INIT;
    p_act->homing_last_phase_end_time  = 0U;
    p_act->extend_time                 = 0U;
    p_act->shrink_time                 = 0U;

    button_debounce_init(&p_act->extend_switch,
                         p_cfg->extend_active_level,
                         p_cfg->debounce_time_ms);
    button_debounce_init(&p_act->shrink_switch,
                         p_cfg->shrink_active_level,
                         p_cfg->debounce_time_ms);
}

void actuator_update(ActuatorControl_t *p_act, uint32_t current_time)
{
    if (p_act == NULL) {
        return;
    }

    update_switches(p_act, current_time);

    /* ---- Homing takes priority over normal operation ---- */
    if (p_act->is_homing != 0U) {
        handle_homing_sequence(p_act, current_time);
        return;
    }

    switch (p_act->state) {
        case ACTUATOR_EXTENDING:
            if (button_debounce_is_pressed(&p_act->extend_switch)) {
                actuator_stop(p_act);
            }
            break;

        case ACTUATOR_SHRINKING:
            if (button_debounce_is_pressed(&p_act->shrink_switch)) {
                actuator_stop(p_act);
            }
            break;

        case ACTUATOR_IDLE:
            /* Nothing to do */
            break;

        case ACTUATOR_ERROR:
            actuator_stop(p_act);
            break;
    }
}

/* -------------------------------------------------------------------------- */
/*   Homing sequence                                                          */
/* -------------------------------------------------------------------------- */

static void handle_homing_sequence(ActuatorControl_t *p_act, uint32_t current_time)
{
    if (p_act == NULL) {
        return;
    }

    /* ---- First invocation — initialise ---- */
    if (p_act->homing_last_phase_end_time == 0U) {
        p_act->homing_phase               = HOMING_PHASE_INIT;
        p_act->homing_last_phase_end_time = current_time;
        actuator_shrink(p_act);
        return;
    }

    switch (p_act->homing_phase) {

        case HOMING_PHASE_INIT:
            /* Shrink until the shrink switch is pressed */
            if (p_act->state == ACTUATOR_SHRINKING &&
                button_debounce_is_pressed(&p_act->shrink_switch)) {
                p_act->homing_phase               = HOMING_PHASE_EXTEND;
                p_act->homing_last_phase_end_time = current_time;
                actuator_extend(p_act);
            }
            break;

        case HOMING_PHASE_EXTEND:
            /* Extend until the extend switch is pressed */
            if (p_act->state == ACTUATOR_EXTENDING &&
                button_debounce_is_pressed(&p_act->extend_switch)) {
                p_act->extend_time  = current_time - p_act->homing_last_phase_end_time;
                p_act->homing_phase = HOMING_PHASE_SHRINK;
                p_act->homing_last_phase_end_time = current_time;
                actuator_shrink(p_act);
            }
            break;

        case HOMING_PHASE_SHRINK:
            /* Shrink until the shrink switch is pressed */
            if (p_act->state == ACTUATOR_SHRINKING &&
                button_debounce_is_pressed(&p_act->shrink_switch)) {
                p_act->shrink_time  = current_time - p_act->homing_last_phase_end_time;
                p_act->homing_phase = HOMING_PHASE_MIDDLE;
                p_act->homing_last_phase_end_time = current_time;
                actuator_extend(p_act);
            }
            break;

        case HOMING_PHASE_MIDDLE:
        {
            /* Move half the extend time back toward centre */
            const uint32_t move_time = p_act->extend_time / 2U;
            if ((current_time - p_act->homing_last_phase_end_time) >= move_time) {
                actuator_stop(p_act);
                p_act->is_homing = 0U;
            }
            break;
        }
    }

    /* ---- Homing safety timeout ---- */
    if ((current_time - p_act->homing_last_phase_end_time) > HOMING_TIMEOUT_MS) {
        actuator_stop(p_act);
        p_act->state     = ACTUATOR_ERROR;
        p_act->is_homing = 0U;
    }
}

/* -------------------------------------------------------------------------- */
/*   Commands                                                                 */
/* -------------------------------------------------------------------------- */

void actuator_start_homing(ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return;
    }

    if (p_act->state != ACTUATOR_IDLE) {
        actuator_stop(p_act);
    }

    p_act->is_homing                  = 1U;
    p_act->homing_phase               = HOMING_PHASE_INIT;
    p_act->homing_last_phase_end_time = 0U;
    p_act->extend_time                = 0U;
    p_act->shrink_time                = 0U;

    actuator_shrink(p_act);     /* Start immediately */
}

void actuator_extend(ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return;
    }

    p_act->state = ACTUATOR_EXTENDING;

    set_outputs(p_act,
                p_act->config.extend_active_level,
                (uint8_t)(!p_act->config.shrink_active_level),
                1U,
                0U);
}

void actuator_shrink(ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return;
    }

    p_act->state = ACTUATOR_SHRINKING;

    set_outputs(p_act,
                (uint8_t)(!p_act->config.extend_active_level),
                p_act->config.shrink_active_level,
                0U,
                1U);
}

void actuator_stop(ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return;
    }

    p_act->state = ACTUATOR_IDLE;

    set_outputs(p_act,
                (uint8_t)(!p_act->config.extend_active_level),
                (uint8_t)(!p_act->config.shrink_active_level),
                0U,
                0U);
}

/* -------------------------------------------------------------------------- */
/*   Queries                                                                  */
/* -------------------------------------------------------------------------- */

ActuatorState_t actuator_get_state(const ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return ACTUATOR_IDLE;
    }
    return p_act->state;
}

uint8_t actuator_is_homing(const ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return 0U;
    }
    return p_act->is_homing;
}

uint8_t actuator_is_error(const ActuatorControl_t *p_act)
{
    if (p_act == NULL) {
        return 0U;
    }
    return (p_act->state == ACTUATOR_ERROR) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */
/*   Private helpers                                                          */
/* -------------------------------------------------------------------------- */

static void set_outputs(const ActuatorControl_t *p_act,
                        uint8_t extend_active,
                        uint8_t shrink_active,
                        uint8_t led_extend,
                        uint8_t led_shrink)
{
    if (p_act == NULL) {
        return;
    }

    /* Cast void* back to the HAL type at the last responsible moment */
    HAL_GPIO_WritePin((GPIO_TypeDef*)p_act->config.extend_control_port,
                      p_act->config.extend_control_pin,
                      (GPIO_PinState)extend_active);

    HAL_GPIO_WritePin((GPIO_TypeDef*)p_act->config.shrink_control_port,
                      p_act->config.shrink_control_pin,
                      (GPIO_PinState)shrink_active);

    HAL_GPIO_WritePin((GPIO_TypeDef*)p_act->config.led_extend_port,
                      p_act->config.led_extend_pin,
                      (GPIO_PinState)led_extend);

    HAL_GPIO_WritePin((GPIO_TypeDef*)p_act->config.led_shrink_port,
                      p_act->config.led_shrink_pin,
                      (GPIO_PinState)led_shrink);
}

static void update_switches(ActuatorControl_t *p_act, uint32_t current_time)
{
    if (p_act == NULL) {
        return;
    }

    button_debounce_update(&p_act->extend_switch,
                           (uint8_t)HAL_GPIO_ReadPin(
                               (GPIO_TypeDef*)p_act->config.extend_switch_port,
                               p_act->config.extend_switch_pin),
                           current_time);

    button_debounce_update(&p_act->shrink_switch,
                           (uint8_t)HAL_GPIO_ReadPin(
                               (GPIO_TypeDef*)p_act->config.shrink_switch_port,
                               p_act->config.shrink_switch_pin),
                           current_time);
}