/*

  servo.c - plugin for M280, Marlin style servo commands

  Part of grblHAL

  Public domain.

  Usage:
    M280[P<id>][S[<position>]]

  If no words are specified all servo positions are reported.
  If no position is specified the specific servo position is returned.

  https://marlinfw.org/docs/gcode/M280.html

*/

#include <math.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>

#include "servo.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/ioports.h"

#ifndef N_SERVOS
    #define N_SERVOS 1
#endif

static uint32_t delay_until = 0;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static stepper_enable_ptr on_stepper_enable;
static axes_signals_t stepper_enabled = {0};
static on_execute_realtime_ptr on_execute_realtime;

static char sbuf[65]; // string buffer for reports

static bool can_map_ports = false, is_executing = false;

#define DEFAULT_MIN_ANGLE 0.0f
#define DEFAULT_MAX_ANGLE 180.0f

//These are the min and max pulse width in microseconds that are expected by servo. These correspond to the minimum and maximum angle.
#define DEFAULT_MIN_PULSE_WIDTH 544e-6
#define DEFAULT_MAX_PULSE_WIDTH 2400e-6
#define DEFAULT_PWM_FREQ 50.0f

static uint8_t n_servos = 0;
static servo_t *servos, *current_servo = NULL;

/// @brief 
/// @param servo Servo number
/// @param angle Angle (in degrees) to set servo to
/// @return 
bool set_angle(uint8_t servo, float angle) {
    //Set the position/pwm
    //Servo position is defined from 0 to 180 degrees (left, right)
    //90 degree is the half duty cycle position
    servo_t s = servos[servo];
    servos[servo].angle = angle;

    hal.port.analog_out(s.port, angle);
    return true;
}

float get_angle(uint8_t servo) {
    //Get the servo angle
    if (servo < N_SERVOS) {
        servo_t s = servos[servo];
        return s.angle;
    }
    else return -1.0;
}

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)280
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;
    switch((uint16_t)gc_block->user_mcode) {
        // M280 P<index> S<pos> 
        case 280:
        //Servo index
            if(gc_block->words.p && isnanf(gc_block->values.p))
                state = Status_BadNumberFormat;
            if(gc_block->words.p && (gc_block->values.p > N_SERVOS))
                state = Status_BadNumberFormat;
        //Servo position, can be set to value or omitted for readout
            // if(gc_block->words.s && isnanf(gc_block->values.s))
            //     state = Status_BadNumberFormat;
            gc_block->words.s = gc_block->words.p = Off;
            break;

        default:
            state = Status_Unhandled;
            break;
    }
    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;
    float angle = 0.0f;
    uint8_t servo = 0;

    if (state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {
        
        // M280 P<index> S<pos> 
         case 280: // Servo mode
                if(gc_block->words.p)
                    {
                        //check servo number exists
                        //If not return invalid
                        if ((servo = gc_block->values.p) >= N_SERVOS) {
                                hal.stream.write("Servo number does not exist." ASCII_EOL);
                            }
                    }

                if(gc_block->words.s) 
                    if ((angle = gc_block->values.s) >= 0.0f) {
                        hal.stream.write("Setting servo position" ASCII_EOL);                        
                        set_angle(servo,angle);

                    }
                    else {
                        //Reads the position/pwm
                        float value = get_angle(servo);
                        if (value >= 0.0)
                            sprintf(sbuf, "[Servo position: %5.2f degrees]" ASCII_EOL,  value);
                        hal.stream.write(sbuf);
                        
                    }

            break;

        default:
            handled = false;
            break;
    }
    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Servo v0.01]" ASCII_EOL);
}

bool init_servo_default(servo_t* servo) {
    servo->pwm_data = malloc(sizeof(pwm_t));
    servo->pwm_data->freq = DEFAULT_PWM_FREQ;

    servo->max_angle = DEFAULT_MAX_ANGLE;
    servo->min_angle = DEFAULT_MIN_ANGLE;

    servo->angle = 0.0;

    servo->min_pulse_width = DEFAULT_MIN_PULSE_WIDTH;
    servo->max_pulse_width = DEFAULT_MAX_PULSE_WIDTH;

    //Convert the pulse-widths to "count" equivalents
    servo->min_duty = servo->min_pulse_width * servo->pwm_data->freq;
    servo->max_duty = servo->max_pulse_width * servo->pwm_data->freq;
    return true;
}

bool servo_claim_from_io() {
    uint8_t n_ports;
    bool ok = (n_ports = ioports_available(Port_Analog, Port_Output)) >= N_SERVOS;

    if(ioport_can_claim_explicit()) {
        // Driver does not support explicit pin claiming, claim the highest numbered ports instead.
        // A bit clunky, since it doesnt consider already claimed analog pins
        uint_fast8_t idx = N_SERVOS;
        xbar_t *portinfo;

        do {
            idx--;
            servos[idx].port = idx;
            //We require a PWM port
            if((portinfo = hal.port.get_pin_info(Port_Analog, Port_Output, idx))) {
                if(!portinfo->cap.claimed && (portinfo->cap.pwm)) {
                    if(!(ok = ioport_claim(Port_Analog, Port_Output, &servos[idx].port, "Servo pin")))
                        servos[idx].port = 0xFF;
                    else {
                        //Initialize default values, and properly configure the pwm
                        init_servo_default(&servos[idx]);
                        xbar_t * port = hal.port.get_pin_info(Port_Analog, Port_Output, servos[idx].port);
                        pwm_config_t config = {
                            .freq_hz = 50.0f,
                            .min = 0.0f,
                            .max = 180.0f,
                            .off_value = 0.0f,
                            .min_value = DEFAULT_MIN_PULSE_WIDTH*DEFAULT_PWM_FREQ * 100.0f,
                            .max_value = DEFAULT_MAX_PULSE_WIDTH*DEFAULT_PWM_FREQ * 100.0f, //Percents of duty cycle
                            .invert = Off
                        };

                        port->config(port, (void*)&config);
                    }
                }
            }
        } while(idx);
    }
            
    return ok;
}

void servo_init (void)
{
    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    hal.user_mcode.check = mcode_check;
    hal.user_mcode.validate = mcode_validate;
    hal.user_mcode.execute = mcode_execute;
    servos = malloc(sizeof(servo_t) * N_SERVOS);

    if (servo_claim_from_io()) {
        // hal.port.analog_out(servos[0].port,0);
    }

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}
