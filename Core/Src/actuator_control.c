#include "actuator_control.h"
#include "main.h"

void actuator_init(ActuatorControl_t* actuator_control, const ActuatorConfig_t* config) {
 
}

void actuator_update(ActuatorControl_t* actuator_control, uint32_t current_time) {
  
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

}

void actuator_start_homing(ActuatorControl_t* actuator_control) {

}

ActuatorState_t actuator_get_state(const ActuatorControl_t* actuator_control) {
    return actuator_control->state;
}



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

// Private functions
static void update_switches(ActuatorControl_t* actuator_control, uint32_t current_time) {
    button_update(&actuator_control->extend_switch,
                  HAL_GPIO_ReadPin(GPIOB, EXTEND_CNTR_Pin),
                  current_time);

    button_update(&actuator_control->shrink_switch,
                  HAL_GPIO_ReadPin(GPIOB, SHRINK_CNTR_Pin),
                  current_time);
} 