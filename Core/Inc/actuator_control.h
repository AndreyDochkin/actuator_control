#ifndef ACTUATOR_CONTROL_H /* ACTUATOR_CONTROL_H */
#define ACTUATOR_CONTROL_H

#include "button_debounce.h"
#include "main.h"

// Actuator states
typedef enum
{
    ACTUATOR_IDLE = 0,
    ACTUATOR_EXTENDING,
    ACTUATOR_SHRINKING,
    ACTUATOR_ERROR
} ActuatorState_t;

// Homing phase
typedef enum
{
    HOMING_PHASE_INIT = 0,      // Initial phase - moving to shrink position
    HOMING_PHASE_EXTEND,        // Measuring extend time
    HOMING_PHASE_SHRINK,        // Measuring shrink time
    HOMING_PHASE_MIDDLE         // Moving to middle position
} HomingPhase_t;

// Actuator configuration
typedef struct
{
    uint8_t extend_active_level;
    uint8_t shrink_active_level;
    uint32_t debounce_time_ms;
    GPIO_TypeDef* extend_control_port;
    uint16_t extend_control_pin;
    GPIO_TypeDef* shrink_control_port;
    uint16_t shrink_control_pin;
    GPIO_TypeDef* extend_switch_port;
    uint16_t extend_switch_pin;
    GPIO_TypeDef* shrink_switch_port;
    uint16_t shrink_switch_pin;
    GPIO_TypeDef* led_extend_port;
    uint16_t led_extend_pin;
    GPIO_TypeDef* led_shrink_port;
    uint16_t led_shrink_pin;
} ActuatorConfig_t;

// Actuator control structure
typedef struct
{
    ActuatorConfig_t config;
    ActuatorState_t state;
    uint8_t is_homing;                   // Flag homing in progress
    HomingPhase_t homing_phase;          // Current phase of homing
    uint32_t homing_last_phase_end_time; // time of last phase end
    uint32_t extend_time;
    uint32_t shrink_time;
    Button extend_switch; // end switch for extend
    Button shrink_switch; // end switch for shrink
} ActuatorControl_t;

void actuator_init(ActuatorControl_t *act_cntrl, const ActuatorConfig_t *config);
void actuator_update(ActuatorControl_t *act_cntrl, uint32_t current_time); // Call this function constatly from loop
void actuator_start_homing(ActuatorControl_t *act_cntrl);                  // Call this function to start homing
ActuatorState_t actuator_get_state(const ActuatorControl_t *act_cntrl);
uint8_t actuator_is_homing(const ActuatorControl_t *act_cntrl);
uint8_t actuator_error(const ActuatorControl_t *act_cntrl);

void actuator_extend(ActuatorControl_t *act_cntrl);
void actuator_shrink(ActuatorControl_t *act_cntrl);
void actuator_stop(ActuatorControl_t *act_cntrl);

#endif /* ACTUATOR_CONTROL_H */
