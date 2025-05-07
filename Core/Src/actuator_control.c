#include "actuator_control.h"
#include "gpio.h"

static void update_switches(ActuatorControl_t* act_cntrl, uint32_t current_time);
static void handle_homing_sequence(ActuatorControl_t* act_cntrl, uint32_t current_time);

void actuator_init(ActuatorControl_t* act_cntrl, const ActuatorConfig_t* config) {
    act_cntrl->config = *config;
    act_cntrl->state = ACTUATOR_IDLE;
    act_cntrl->homing_start_time = 0;
    act_cntrl->homing_direction = HOMING_DIR_NONE;
    act_cntrl->extend_time = 0;
    act_cntrl->shrink_time = 0;

    button_init(&act_cntrl->extend_switch, config->extend_active_level, config->debounce_time_ms);
    button_init(&act_cntrl->shrink_switch, config->shrink_active_level, config->debounce_time_ms);
}

void actuator_update(ActuatorControl_t* act_cntrl, uint32_t current_time) {
    
    update_switches(act_cntrl, current_time); // Update switch states with debouncing

    switch (act_cntrl->state) {
        case ACTUATOR_HOMING:
            handle_homing_sequence(act_cntrl, current_time);
            break;

        case ACTUATOR_EXTENDING:
            if (button_is_pressed(&act_cntrl->extend_switch)) {
                actuator_stop(act_cntrl);
                act_cntrl->state = ACTUATOR_IDLE;
            }
            break;

        case ACTUATOR_SHRINKING:
            if (button_is_pressed(&act_cntrl->shrink_switch)) {
                actuator_stop(act_cntrl);
                act_cntrl->state = ACTUATOR_IDLE;
            }
            break;

        case ACTUATOR_IDLE:
        case ACTUATOR_ERROR:
            actuator_stop(act_cntrl);
            break;
    }
}

static void handle_homing_sequence(ActuatorControl_t* act_cntrl, uint32_t current_time) {
    // Initialize homing if not started
    if (act_cntrl->homing_start_time == 0) {
        act_cntrl->homing_direction = HOMING_DIR_SHRINK;  // Start with shrinking
        act_cntrl->homing_start_time = current_time;      // Set start time when homing begins
        actuator_shrink(act_cntrl);
        return;
    }

    switch (act_cntrl->homing_direction) {
        case HOMING_DIR_SHRINK:
            if (button_is_pressed(&act_cntrl->shrink_switch)) {
                if (act_cntrl->extend_time == 0) {
                    // 1 shrink phase - just reached initial position
                    act_cntrl->homing_start_time = current_time;  // Start timing from here
                    act_cntrl->homing_direction = HOMING_DIR_EXTEND;  // Switch to extending
                } else {
                    // 3 shrink phase - measuring shrink time
                    act_cntrl->shrink_time = current_time - act_cntrl->homing_start_time - act_cntrl->extend_time;
                    act_cntrl->homing_direction = HOMING_DIR_MIDDLE;  // Move to middle
                }
                actuator_extend(act_cntrl);
            }
            break;

        case HOMING_DIR_EXTEND:
            // 2 extend phase - measuring extend time
            if (button_is_pressed(&act_cntrl->extend_switch)) {
                act_cntrl->extend_time = current_time - act_cntrl->homing_start_time;
                act_cntrl->homing_direction = HOMING_DIR_SHRINK;  // Switch to shrinking
                actuator_shrink(act_cntrl);
            }
            break;

        case HOMING_DIR_MIDDLE:
            {
                uint32_t move_time = act_cntrl->extend_time / 2;
                if (current_time - act_cntrl->homing_start_time - act_cntrl->extend_time - act_cntrl->shrink_time >= move_time) {
                    actuator_stop(act_cntrl);
                    act_cntrl->state = ACTUATOR_IDLE;
                }
            }
            break;

        default:
            break;
    }
}

void actuator_start_homing(ActuatorControl_t *act_cntrl)
{
    if (act_cntrl->state != ACTUATOR_IDLE)
    {
        actuator_stop(act_cntrl);
    }

    act_cntrl->state = ACTUATOR_HOMING;
    act_cntrl->homing_direction = HOMING_DIR_NONE;
    act_cntrl->homing_start_time = 0;
    act_cntrl->extend_time = 0;
    act_cntrl->shrink_time = 0;
}

ActuatorState_t actuator_get_state(const ActuatorControl_t* act_cntrl) {
    return act_cntrl->state;
}

uint8_t actuator_error(const ActuatorControl_t* act_cntrl) {
    return act_cntrl->state == ACTUATOR_ERROR;
}

void actuator_extend(ActuatorControl_t *act_cntrl)
{
    HAL_GPIO_WritePin(act_cntrl->config.extend_control_port,
                      act_cntrl->config.extend_control_pin,
                      act_cntrl->config.extend_active_level);
    HAL_GPIO_WritePin(act_cntrl->config.shrink_control_port,
                      act_cntrl->config.shrink_control_pin,
                      !act_cntrl->config.shrink_active_level);
    HAL_GPIO_WritePin(act_cntrl->config.led_extend_port,
                      act_cntrl->config.led_extend_pin,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(act_cntrl->config.led_shrink_port,
                      act_cntrl->config.led_shrink_pin,
                      GPIO_PIN_RESET);
}

void actuator_shrink(ActuatorControl_t *act_cntrl)
{
    HAL_GPIO_WritePin(act_cntrl->config.extend_control_port,
                      act_cntrl->config.extend_control_pin,
                      !act_cntrl->config.extend_active_level);
    HAL_GPIO_WritePin(act_cntrl->config.shrink_control_port,
                      act_cntrl->config.shrink_control_pin,
                      act_cntrl->config.shrink_active_level);
    HAL_GPIO_WritePin(act_cntrl->config.led_extend_port,
                      act_cntrl->config.led_extend_pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(act_cntrl->config.led_shrink_port,
                      act_cntrl->config.led_shrink_pin,
                      GPIO_PIN_SET);
}

void actuator_stop(ActuatorControl_t *act_cntrl)
{
    HAL_GPIO_WritePin(act_cntrl->config.extend_control_port,
                      act_cntrl->config.extend_control_pin,
                      !act_cntrl->config.extend_active_level);
    HAL_GPIO_WritePin(act_cntrl->config.shrink_control_port,
                      act_cntrl->config.shrink_control_pin,
                      !act_cntrl->config.shrink_active_level);
    HAL_GPIO_WritePin(act_cntrl->config.led_extend_port,
                      act_cntrl->config.led_extend_pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(act_cntrl->config.led_shrink_port,
                      act_cntrl->config.led_shrink_pin,
                      GPIO_PIN_RESET);
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
