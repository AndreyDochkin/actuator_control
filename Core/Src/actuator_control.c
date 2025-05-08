#include "actuator_control.h"
#include "gpio.h"

static void update_switches(ActuatorControl_t* act_cntrl, uint32_t current_time);
static void handle_homing_sequence(ActuatorControl_t* act_cntrl, uint32_t current_time);

void actuator_init(ActuatorControl_t* act_cntrl, const ActuatorConfig_t* config) {
    act_cntrl->config = *config;
    act_cntrl->state = ACTUATOR_IDLE;
    act_cntrl->is_homing = 0;
    act_cntrl->homing_phase = HOMING_PHASE_INIT;
    act_cntrl->homing_last_phase_end_time = 0;
    act_cntrl->extend_time = 0;
    act_cntrl->shrink_time = 0;

    button_init(&act_cntrl->extend_switch, config->extend_active_level, config->debounce_time_ms);
    button_init(&act_cntrl->shrink_switch, config->shrink_active_level, config->debounce_time_ms);
}

void actuator_update(ActuatorControl_t* act_cntrl, uint32_t current_time) {
    update_switches(act_cntrl, current_time);

    // SPECIAL CASE
    // if homing, handle homing sequence
    if (act_cntrl->is_homing) {
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
            // actuator do nothing
            break;

        case ACTUATOR_ERROR:
            actuator_stop(act_cntrl);
            break;
    }
}

static void handle_homing_sequence(ActuatorControl_t* act_cntrl, uint32_t current_time) {
    // Initialize homing if not started
    if (act_cntrl->homing_last_phase_end_time == 0) {
        act_cntrl->homing_phase = HOMING_PHASE_INIT;
        act_cntrl->homing_last_phase_end_time = current_time;
        actuator_shrink(act_cntrl);
        return;
    }

    switch (act_cntrl->homing_phase) {
        case HOMING_PHASE_INIT: // shrink to set start position
            if (act_cntrl->state == ACTUATOR_SHRINKING && button_is_pressed(&act_cntrl->shrink_switch)) {
                act_cntrl->homing_phase = HOMING_PHASE_EXTEND;
                act_cntrl->homing_last_phase_end_time = current_time;
                actuator_extend(act_cntrl);
            }
            break;

        case HOMING_PHASE_EXTEND:  // extend to mesure extend time
            if (act_cntrl->state == ACTUATOR_EXTENDING && button_is_pressed(&act_cntrl->extend_switch)) {
                act_cntrl->extend_time = current_time - act_cntrl->homing_last_phase_end_time;
                act_cntrl->homing_phase = HOMING_PHASE_SHRINK;
                act_cntrl->homing_last_phase_end_time = current_time;
                actuator_shrink(act_cntrl);
            }
            break;

        case HOMING_PHASE_SHRINK: // shrink to measuring time shrink time
            if (act_cntrl->state == ACTUATOR_SHRINKING && button_is_pressed(&act_cntrl->shrink_switch)) {
                act_cntrl->shrink_time = current_time - act_cntrl->homing_last_phase_end_time;
                act_cntrl->homing_phase = HOMING_PHASE_MIDDLE;
                act_cntrl->homing_last_phase_end_time = current_time;
                actuator_extend(act_cntrl);
            }
            break;

        case HOMING_PHASE_MIDDLE: // move to middle position 
            {
                // we use extend time because we start from shrink position at this point
                // so it's really does't matter if extend and shrink times are different
                uint32_t move_time = act_cntrl->extend_time / 2; 
                if (current_time - act_cntrl->homing_last_phase_end_time >= move_time) {
                    actuator_stop(act_cntrl);
                    act_cntrl->is_homing = 0;
                }
            }
            break;
    }
}

void actuator_start_homing(ActuatorControl_t *act_cntrl)
{
    if (act_cntrl->state != ACTUATOR_IDLE)
    {
        actuator_stop(act_cntrl);
    }

    act_cntrl->is_homing = 1;
    act_cntrl->homing_phase = HOMING_PHASE_INIT;
    act_cntrl->homing_last_phase_end_time = 0;
    act_cntrl->extend_time = 0;
    act_cntrl->shrink_time = 0;
    actuator_shrink(act_cntrl);  //start immediately
}

ActuatorState_t actuator_get_state(const ActuatorControl_t* act_cntrl) {
    return act_cntrl->state;
}

ActuatorState_t actuator_is_homing(const ActuatorControl_t* act_cntrl) {
    return act_cntrl->is_homing;
}

uint8_t actuator_error(const ActuatorControl_t* act_cntrl) {
    return act_cntrl->state == ACTUATOR_ERROR;
}

void actuator_extend(ActuatorControl_t *act_cntrl)
{
    act_cntrl->state = ACTUATOR_EXTENDING;
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
    act_cntrl->state = ACTUATOR_SHRINKING;
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
    act_cntrl->state = ACTUATOR_IDLE;
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
