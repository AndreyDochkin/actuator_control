# Actuator Control

## Notes

* STM32CubeIDE v1.18.1 and STM32CubeMX v6.14.1 were used for the project.
* Implements a state machine for actuator control.
* Homing is part of the actuator state machine but treated as a special case.
* Used my own `button_debouncer` library. https://github.com/AndreyDochkin/button_debounce
* Uses extend time to set the middle position, starting from the retracted position (extend/shrink time differences are negligible).
* Error handling can be improved.

Andrei Dochkin
https://www.linkedin.com/in/andrei-dochkin/
