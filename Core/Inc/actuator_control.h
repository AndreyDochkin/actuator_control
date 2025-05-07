#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <stdint.h>
#include "button_debounce.h"

// Actuator states
typedef enum
{
    ACTUATOR_IDLE = 0,
    ACTUATOR_HOMING,
    ACTUATOR_EXTENDING,
    ACTUATOR_SHRINKING,
    ACTUATOR_ERROR
} ActuatorState_t;

// Actuator configuration
typedef struct
{
    uint8_t extend_active_level;
    uint8_t shrink_active_level;
    uint32_t debounce_time_ms;
} ActuatorConfig_t;

// Actuator control structure
typedef struct
{
    ActuatorConfig_t config;
    ActuatorState_t state;
    uint32_t last_update_time;
    uint32_t homing_start_time;
    int8_t homing_direction;
    uint32_t extend_time;
    uint32_t shrink_time;
    Button extend_switch;
    Button shrink_switch;
} ActuatorControl_t;

void actuator_init(ActuatorControl_t *actuator_control, const ActuatorConfig_t *config);
void actuator_update(ActuatorControl_t *actuator_control, uint32_t current_time);
void actuator_start_homing(ActuatorControl_t *actuator_control);
ActuatorState_t actuator_get_state(const ActuatorControl_t *actuator_control);
uint8_t actuator_error(const ActuatorControl_t *actuator_control);

void actuator_extend(ActuatorControl_t *actuator_control);
void actuator_shrink(ActuatorControl_t *actuator_control);
void actuator_stop(ActuatorControl_t *actuator_control);

#endif /* ACTUATOR_CONTROL_H */
