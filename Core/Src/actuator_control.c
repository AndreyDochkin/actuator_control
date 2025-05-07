#include "actuator_control.h"

static void update_switches(ActuatorControl_t* actuator_control, uint32_t current_time);

void actuator_init(ActuatorControl_t* actuator_control, const ActuatorConfig_t* config) {
    actuator_control->config = *config;
    actuator_control->state = ACTUATOR_IDLE;
    actuator_control->last_update_time = 0;
    actuator_control->homing_start_time = 0;
    actuator_control->homing_direction = HOMING_DIR_NONE;
    actuator_control->extend_time = 0;
    actuator_control->shrink_time = 0;

    button_init(&actuator_control->extend_switch, config->extend_active_level, config->debounce_time_ms);
    button_init(&actuator_control->shrink_switch, config->shrink_active_level, config->debounce_time_ms);
}

void actuator_update(ActuatorControl_t* actuator_control, uint32_t current_time) {
    
    update_switches(actuator_control, current_time); // Update switch states with debouncing

    switch (actuator_control->state) {
        case ACTUATOR_HOMING:
            // Initialize homing if not started
            if (actuator_control->homing_start_time == 0) {
                actuator_control->homing_direction = HOMING_DIR_SHRINK;  // Start with shrinking to initial position
                actuator_shrink(actuator_control);
                break;
            }

            // [0] Move to initial position (shrink)
            if (actuator_control->homing_direction == HOMING_DIR_SHRINK) {
                if (button_is_pressed(&actuator_control->shrink_switch)) {
                    actuator_control->homing_start_time = current_time;  // Start timing from here
                    actuator_control->homing_direction = HOMING_DIR_EXTEND;  // Switch to extending
                    actuator_extend(actuator_control);
                }
                break;
            }

            // [1] Find first limit switch
            if (actuator_control->homing_direction == HOMING_DIR_EXTEND) {
                if (button_is_pressed(&actuator_control->extend_switch)) {
                    actuator_control->extend_time = current_time - actuator_control->homing_start_time;
                    actuator_control->homing_direction = HOMING_DIR_SHRINK;  // Switch to shrinking
                    actuator_shrink(actuator_control);
                }
                break;
            }

            // [2] Find second limit switch
            if (actuator_control->homing_direction == HOMING_DIR_SHRINK) {
                if (button_is_pressed(&actuator_control->shrink_switch)) {
                    actuator_control->shrink_time = current_time - actuator_control->homing_start_time - actuator_control->extend_time;
                    actuator_control->homing_direction = HOMING_DIR_MIDDLE;  // Move to middle
                    actuator_extend(actuator_control);
                }
                break;
            }

            // [3] Move to middle position
            if (actuator_control->homing_direction == HOMING_DIR_MIDDLE) {
                uint32_t move_time = actuator_control->extend_time / 2;
                if (current_time - actuator_control->homing_start_time - actuator_control->extend_time - actuator_control->shrink_time >= move_time) {
                    actuator_stop(actuator_control);
                    actuator_control->state = ACTUATOR_IDLE;
                }
            }
            break;

        case ACTUATOR_EXTENDING:
            if (button_is_pressed(&actuator_control->extend_switch)) {
                actuator_stop(actuator_control);
                actuator_control->state = ACTUATOR_IDLE;
            }
            break;

        case ACTUATOR_SHRINKING:
            if (button_is_pressed(&actuator_control->shrink_switch)) {
                actuator_stop(actuator_control);
                actuator_control->state = ACTUATOR_IDLE;
            }
            break;

        case ACTUATOR_IDLE:
        case ACTUATOR_ERROR:
            actuator_stop(actuator_control);
            break;
    }

    actuator_control->last_update_time = current_time;
}

void actuator_start_homing(ActuatorControl_t* actuator_control) {
    if (actuator_control->state == ACTUATOR_IDLE) {
        actuator_control->state = ACTUATOR_HOMING;
        actuator_control->homing_direction = HOMING_DIR_NONE;
        actuator_control->homing_start_time = 0;
        actuator_control->extend_time = 0;
        actuator_control->shrink_time = 0;
    }
}

ActuatorState_t actuator_get_state(const ActuatorControl_t* actuator_control) {
    return actuator_control->state;
}

uint8_t actuator_error(const ActuatorControl_t* actuator_control) {
    return actuator_control->state == ACTUATOR_ERROR;
}

void actuator_extend(ActuatorControl_t *actuator_control)
{
    HAL_GPIO_WritePin(actuator_control->config.extend_control_port,
                      actuator_control->config.extend_control_pin,
                      actuator_control->config.extend_active_level);
    HAL_GPIO_WritePin(actuator_control->config.shrink_control_port,
                      actuator_control->config.shrink_control_pin,
                      !actuator_control->config.shrink_active_level);
    HAL_GPIO_WritePin(actuator_control->config.led_extend_port,
                      actuator_control->config.led_extend_pin,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(actuator_control->config.led_shrink_port,
                      actuator_control->config.led_shrink_pin,
                      GPIO_PIN_RESET);
}

void actuator_shrink(ActuatorControl_t *actuator_control)
{
    HAL_GPIO_WritePin(actuator_control->config.extend_control_port,
                      actuator_control->config.extend_control_pin,
                      !actuator_control->config.extend_active_level);
    HAL_GPIO_WritePin(actuator_control->config.shrink_control_port,
                      actuator_control->config.shrink_control_pin,
                      actuator_control->config.shrink_active_level);
    HAL_GPIO_WritePin(actuator_control->config.led_extend_port,
                      actuator_control->config.led_extend_pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(actuator_control->config.led_shrink_port,
                      actuator_control->config.led_shrink_pin,
                      GPIO_PIN_SET);
}

void actuator_stop(ActuatorControl_t *actuator_control)
{
    HAL_GPIO_WritePin(actuator_control->config.extend_control_port,
                      actuator_control->config.extend_control_pin,
                      !actuator_control->config.extend_active_level);
    HAL_GPIO_WritePin(actuator_control->config.shrink_control_port,
                      actuator_control->config.shrink_control_pin,
                      !actuator_control->config.shrink_active_level);
    HAL_GPIO_WritePin(actuator_control->config.led_extend_port,
                      actuator_control->config.led_extend_pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(actuator_control->config.led_shrink_port,
                      actuator_control->config.led_shrink_pin,
                      GPIO_PIN_RESET);
}

static void update_switches(ActuatorControl_t *actuator_control, uint32_t current_time)
{
    button_update(&actuator_control->extend_switch,
                  HAL_GPIO_ReadPin(actuator_control->config.extend_switch_port,
                                   actuator_control->config.extend_switch_pin),
                  current_time);

    button_update(&actuator_control->shrink_switch,
                  HAL_GPIO_ReadPin(actuator_control->config.shrink_switch_port,
                                   actuator_control->config.shrink_switch_pin),
                  current_time);
}