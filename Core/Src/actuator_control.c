/**
 * @file    actuator_control.c
 * @brief   Actuator control state machine with non-blocking homing sequence.
 *
 * Implements the full actuator lifecycle: extend, shrink, stop, and an
 * automatic homing routine that measures travel times and parks the actuator
 * at the midpoint.
 */

#include "actuator_control.h"
#include "gpio.h"

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
 * @param  act_cntrl    Actuator control structure.
 * @param  current_time Current tick count.
 */
static void update_switches(ActuatorControl_t *act_cntrl, uint32_t current_time);

/**
 * @brief  Run one iteration of the homing state machine.
 * @param  act_cntrl    Actuator control structure.
 * @param  current_time Current tick count.
 */
static void handle_homing_sequence(ActuatorControl_t *act_cntrl, uint32_t current_time);

/**
 * @brief  Set a GPIO output pair and the two direction LEDs.
 * @param  act_cntrl     Actuator control structure.
 * @param  extend_active Value for the extend control pin.
 * @param  shrink_active Value for the shrink control pin.
 * @param  led_extend    Value for the extend-direction LED.
 * @param  led_shrink    Value for the shrink-direction LED.
 */
static void set_outputs(const ActuatorControl_t *act_cntrl,
                        GPIO_PinState extend_active,
                        GPIO_PinState shrink_active,
                        GPIO_PinState led_extend,
                        GPIO_PinState led_shrink);

/* -------------------------------------------------------------------------- */
/*   Public API                                                               */
/* -------------------------------------------------------------------------- */

void actuator_init(ActuatorControl_t *act_cntrl, const ActuatorConfig_t *config)
{
    act_cntrl->config                      = *config;
    act_cntrl->state                       = ACTUATOR_IDLE;
    act_cntrl->is_homing                   = 0U;
    act_cntrl->homing_phase                = HOMING_PHASE_INIT;
    act_cntrl->homing_last_phase_end_time  = 0U;
    act_cntrl->extend_time                 = 0U;
    act_cntrl->shrink_time                 = 0U;

    button_init(&act_cntrl->extend_switch,
                config->extend_active_level,
                config->debounce_time_ms);
    button_init(&act_cntrl->shrink_switch,
                config->shrink_active_level,
                config->debounce_time_ms);
}

void actuator_update(ActuatorControl_t *act_cntrl, uint32_t current_time)
{
    update_switches(act_cntrl, current_time);

    /* ---- Homing takes priority over normal operation ---- */
    if (act_cntrl->is_homing != 0U) {
        handle_homing_sequence(act_cntrl, current_time);
        return;
    }

    switch (act_cntrl->state) {
        case ACTUATOR_EXTENDING:
            if (button_is_pressed(&act_cntrl->extend_switch)) {
                actuator_stop(act_cntrl);
            }
            break;

        case ACTUATOR_SHRINKING:
            if (button_is_pressed(&act_cntrl->shrink_switch)) {
                actuator_stop(act_cntrl);
            }
            break;

        case ACTUATOR_IDLE:
            /* Nothing to do */
            break;

        case ACTUATOR_ERROR:
            actuator_stop(act_cntrl);
            break;
    }
}

/* -------------------------------------------------------------------------- */
/*   Homing sequence                                                          */
/* -------------------------------------------------------------------------- */

static void handle_homing_sequence(ActuatorControl_t *act_cntrl, uint32_t current_time)
{
    /* ---- First invocation — initialise ---- */
    if (act_cntrl->homing_last_phase_end_time == 0U) {
        act_cntrl->homing_phase               = HOMING_PHASE_INIT;
        act_cntrl->homing_last_phase_end_time = current_time;
        actuator_shrink(act_cntrl);
        return;
    }

    switch (act_cntrl->homing_phase) {

        case HOMING_PHASE_INIT:
            /* Shrink until the shrink switch is pressed */
            if (act_cntrl->state == ACTUATOR_SHRINKING &&
                button_is_pressed(&act_cntrl->shrink_switch)) {
                act_cntrl->homing_phase               = HOMING_PHASE_EXTEND;
                act_cntrl->homing_last_phase_end_time = current_time;
                actuator_extend(act_cntrl);
            }
            break;

        case HOMING_PHASE_EXTEND:
            /* Extend until the extend switch is pressed */
            if (act_cntrl->state == ACTUATOR_EXTENDING &&
                button_is_pressed(&act_cntrl->extend_switch)) {
                act_cntrl->extend_time  = current_time - act_cntrl->homing_last_phase_end_time;
                act_cntrl->homing_phase = HOMING_PHASE_SHRINK;
                act_cntrl->homing_last_phase_end_time = current_time;
                actuator_shrink(act_cntrl);
            }
            break;

        case HOMING_PHASE_SHRINK:
            /* Shrink until the shrink switch is pressed */
            if (act_cntrl->state == ACTUATOR_SHRINKING &&
                button_is_pressed(&act_cntrl->shrink_switch)) {
                act_cntrl->shrink_time  = current_time - act_cntrl->homing_last_phase_end_time;
                act_cntrl->homing_phase = HOMING_PHASE_MIDDLE;
                act_cntrl->homing_last_phase_end_time = current_time;
                actuator_extend(act_cntrl);
            }
            break;

        case HOMING_PHASE_MIDDLE:
        {
            /* Move half the extend time back toward centre */
            const uint32_t move_time = act_cntrl->extend_time / 2U;
            if (current_time - act_cntrl->homing_last_phase_end_time >= move_time) {
                actuator_stop(act_cntrl);
                act_cntrl->is_homing = 0U;
            }
            break;
        }
    }

    /* ---- Homing safety timeout ---- */
    if ((current_time - act_cntrl->homing_last_phase_end_time) > HOMING_TIMEOUT_MS) {
        actuator_stop(act_cntrl);
        act_cntrl->state     = ACTUATOR_ERROR;
        act_cntrl->is_homing = 0U;
    }
}

/* -------------------------------------------------------------------------- */
/*   Commands                                                                 */
/* -------------------------------------------------------------------------- */

void actuator_start_homing(ActuatorControl_t *act_cntrl)
{
    if (act_cntrl->state != ACTUATOR_IDLE) {
        actuator_stop(act_cntrl);
    }

    act_cntrl->is_homing                  = 1U;
    act_cntrl->homing_phase               = HOMING_PHASE_INIT;
    act_cntrl->homing_last_phase_end_time = 0U;
    act_cntrl->extend_time                = 0U;
    act_cntrl->shrink_time                = 0U;

    actuator_shrink(act_cntrl);     /* Start immediately */
}

void actuator_extend(ActuatorControl_t *act_cntrl)
{
    act_cntrl->state = ACTUATOR_EXTENDING;

    set_outputs(act_cntrl,
                (GPIO_PinState)act_cntrl->config.extend_active_level,
                (GPIO_PinState)(!act_cntrl->config.shrink_active_level),
                GPIO_PIN_SET,
                GPIO_PIN_RESET);
}

void actuator_shrink(ActuatorControl_t *act_cntrl)
{
    act_cntrl->state = ACTUATOR_SHRINKING;

    set_outputs(act_cntrl,
                (GPIO_PinState)(!act_cntrl->config.extend_active_level),
                (GPIO_PinState)act_cntrl->config.shrink_active_level,
                GPIO_PIN_RESET,
                GPIO_PIN_SET);
}

void actuator_stop(ActuatorControl_t *act_cntrl)
{
    act_cntrl->state = ACTUATOR_IDLE;

    set_outputs(act_cntrl,
                (GPIO_PinState)(!act_cntrl->config.extend_active_level),
                (GPIO_PinState)(!act_cntrl->config.shrink_active_level),
                GPIO_PIN_RESET,
                GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/*   Queries                                                                  */
/* -------------------------------------------------------------------------- */

ActuatorState_t actuator_get_state(const ActuatorControl_t *act_cntrl)
{
    return act_cntrl->state;
}

uint8_t actuator_is_homing(const ActuatorControl_t *act_cntrl)
{
    return act_cntrl->is_homing;
}

uint8_t actuator_is_error(const ActuatorControl_t *act_cntrl)
{
    return (act_cntrl->state == ACTUATOR_ERROR) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */
/*   Private helpers                                                          */
/* -------------------------------------------------------------------------- */

static void set_outputs(const ActuatorControl_t *act_cntrl,
                        GPIO_PinState extend_active,
                        GPIO_PinState shrink_active,
                        GPIO_PinState led_extend,
                        GPIO_PinState led_shrink)
{
    HAL_GPIO_WritePin(act_cntrl->config.extend_control_port,
                      act_cntrl->config.extend_control_pin,
                      extend_active);

    HAL_GPIO_WritePin(act_cntrl->config.shrink_control_port,
                      act_cntrl->config.shrink_control_pin,
                      shrink_active);

    HAL_GPIO_WritePin(act_cntrl->config.led_extend_port,
                      act_cntrl->config.led_extend_pin,
                      led_extend);

    HAL_GPIO_WritePin(act_cntrl->config.led_shrink_port,
                      act_cntrl->config.led_shrink_pin,
                      led_shrink);
}

static void update_switches(ActuatorControl_t *act_cntrl, uint32_t current_time)
{
    button_update(&act_cntrl->extend_switch,
                  HAL_GPIO_ReadPin(act_cntrl->config.extend_switch_port,
                                   act_cntrl->config.extend_switch_pin),
                  current_time);

    button_update(&act_cntrl->shrink_switch,
                  HAL_GPIO_ReadPin(act_cntrl->config.shrink_switch_port,
                                   act_cntrl->config.shrink_switch_pin),
                  current_time);
}