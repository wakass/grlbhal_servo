#ifndef _SERVO_H_
#define _SERVO_H_

#include "pwm.h"

void servo_init (void);


#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

typedef struct {
    uint8_t port; //Port number, referring to (analog) HAL port number
    
    float min_pulse_width;
    float max_pulse_width;
    
    float min_angle; 
    float max_angle;

    float min_duty;
    float max_duty;

    float angle; //Current setpoint for the angle. (degrees)
//Private:
    pwm_t* pwm_data; //Backend pwm info, maybe not needed

} servo_t;

#endif //_SERVO_H