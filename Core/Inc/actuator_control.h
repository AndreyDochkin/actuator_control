#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <stdint.h>
#include "button_debounce.h"

typedef enum {
    ACTUATOR_IDLE,
    ACTUATOR_EXTENDING,
    ACTUATOR_RETRACTING,
    ACTUATOR_HOMING,
    ACTUATOR_ERROR
} ActuatorState_t;

typedef struct {
    uint32_t extend_speed;      
    uint32_t shrink_speed;  
    uint32_t debounce_time_ms; 
    uint8_t extend_active_level; 
    uint8_t shrink_active_level; 
} ActuatorConfig_t;

typedef struct {
    ActuatorState_t state;
    ActuatorConfig_t config;
    Button extend_switch;
    Button retract_switch;
    uint32_t last_update_time;
    uint32_t homing_start_time;
    uint32_t homing_duration;
    uint8_t homing_direction;
} ActuatorControl_t;

void actuator_init(ActuatorControl_t* actuator_control, const ActuatorConfig_t* config);
void actuator_update(ActuatorControl_t* actuator_control, uint32_t current_time);
void actuator_start_homing(ActuatorControl_t* actuator_control);
ActuatorState_t actuator_get_state(const ActuatorControl_t* actuator_control);
uint8_t actuator_error(const ActuatorControl_t* actuator_control);
void extend_actuator(ActuatorControl_t* actuator_control);
void shrink_actuator(ActuatorControl_t* actuator_control);
void stop_actuator(ActuatorControl_t* actuator_control);

#endif