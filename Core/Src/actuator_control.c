#include "actuator_control.h"
#include "main.h"

static void update_switches(ActuatorControl_t* actuator_control, uint32_t current_time);

void actuator_init(ActuatorControl_t* actuator_control, const ActuatorConfig_t* config) {
    actuator_control->config = *config;
    
    actuator_control->state = ACTUATOR_IDLE;
    actuator_control->last_update_time = 0;
    actuator_control->homing_start_time = 0;
    actuator_control->homing_duration = 0;
    actuator_control->homing_direction = 0;

    button_init(&actuator_control->extend_switch, config->extend_active_level, config->debounce_time_ms);
    button_init(&actuator_control->shrink_switch, config->shrink_active_level, config->debounce_time_ms);
}

void actuator_update(ActuatorControl_t* actuator_control, uint32_t current_time) {
    update_switches(actuator_control, current_time);

    switch (actuator_control->state) {
        case ACTUATOR_HOMING:
            break;

        case ACTUATOR_EXTENDING:
            break;

        case ACTUATOR_SHRINKING:
            break;

        case ACTUATOR_IDLE:
        case ACTUATOR_ERROR:
            break;
    }

    actuator_control->last_update_time = current_time;
}

void actuator_start_homing(ActuatorControl_t* actuator_control) {
    if (actuator_control->state == ACTUATOR_IDLE) {
        actuator_control->state = ACTUATOR_HOMING;
        actuator_control->homing_direction = 0;
        actuator_control->homing_start_time = 0;
        actuator_control->homing_duration = 0;
    }
}

ActuatorState_t actuator_get_state(const ActuatorControl_t* actuator_control) {
    return actuator_control->state;
}

uint8_t actuator_error(const ActuatorControl_t* actuator_control) {
    return actuator_control->state == ACTUATOR_ERROR;
}

// Control actuator movement
void extend_actuator(ActuatorControl_t* actuator_control) {
    HAL_GPIO_WritePin(GPIOB, EXTEND_CNTR_Pin, actuator_control->config.extend_active_level);
    HAL_GPIO_WritePin(GPIOB, SHRINK_CNTR_Pin, !actuator_control->config.shrink_active_level);
    HAL_GPIO_WritePin(GPIOB, LED_EXTEND_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED_SHRINK_Pin, GPIO_PIN_RESET);
}

void shrink_actuator(ActuatorControl_t* actuator_control) {
    HAL_GPIO_WritePin(GPIOB, EXTEND_CNTR_Pin, !actuator_control->config.extend_active_level);
    HAL_GPIO_WritePin(GPIOB, SHRINK_CNTR_Pin, actuator_control->config.shrink_active_level);
    HAL_GPIO_WritePin(GPIOB, LED_EXTEND_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_SHRINK_Pin, GPIO_PIN_SET);
}

void stop_actuator(ActuatorControl_t* actuator_control) {
    HAL_GPIO_WritePin(GPIOB, EXTEND_CNTR_Pin, !actuator_control->config.extend_active_level);
    HAL_GPIO_WritePin(GPIOB, SHRINK_CNTR_Pin, !actuator_control->config.shrink_active_level);
    HAL_GPIO_WritePin(GPIOB, LED_EXTEND_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_SHRINK_Pin, GPIO_PIN_RESET);
}

static void update_switches(ActuatorControl_t* actuator_control, uint32_t current_time) {
    button_update(&actuator_control->extend_switch, 
        HAL_GPIO_ReadPin(GPIOB, EXTEND_CNTR_Pin),
        current_time);

    button_update(&actuator_control->shrink_switch,
        HAL_GPIO_ReadPin(GPIOB, SHRINK_CNTR_Pin),
        current_time);
} 