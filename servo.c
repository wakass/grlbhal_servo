/*

  my_plugin.c - plugin for M280, Marlin style servo commands

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
// #include <stdlib.h>

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
static uint8_t n_ports;
uint8_t port[N_SERVOS];

#define MIN_ANGLE 0.0f
#define MAX_ANGLE 180.0f

//These are the min and max pulse width in microseconds that are expected by servo. These correspond to the minimum and maximum angle.
#define MIN_PULSE_WIDTH 544e-6
#define MAX_PULSE_WIDTH 2400e-6
#define PWM_FREQ 50.0f

//Convert here to duty
#define MIN_DUTY MIN_PULSE_WIDTH*PWM_FREQ
#define MAX_DUTY MAX_PULSE_WIDTH*PWM_FREQ

// static uint8_t n_servo = 0;
// static const servo_ptrs_t *servos[N_SERVOS], *current_servo = NULL;

/// @brief 
/// @param servo 
/// @param angle 
/// @return 
static bool set_angle(uint8_t servo, float angle) {
    //Set the position/pwm
    //Servo position is defined from 0 to 180 degrees (left, right)
    //90 degree is the half duty cycle position

    float duty = angle / MAX_ANGLE;
    duty = MIN_DUTY + duty * (MAX_DUTY - MIN_DUTY);
    hal.port.analog_out(port[servo], duty);
    return true;
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
                        if ((servo = gc_block->values.p) > N_SERVOS) {
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
                        int value = 0;
                        sprintf(sbuf, "[Servo position :%d:%lx]" ASCII_EOL,  value,value);
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


void single_init() {

    bool ok = (n_ports = ioports_available(Port_Analog, Port_Output)) >= N_SERVOS;

    // if(ok && !(can_map_ports = ioport_can_claim_explicit())) {

        // Driver does not support explicit pin claiming, claim the highest numbered ports instead.

        uint_fast8_t idx = N_SERVOS;

        do {
            idx--;
            if(!(ok = ioport_claim(Port_Analog, Port_Output, &port[idx], "Servo pin")))
                port[idx] = 0xFF;
        } while(idx);
    // }
}

void servo_init (void)
{
    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    hal.user_mcode.check = mcode_check;
    hal.user_mcode.validate = mcode_validate;
    hal.user_mcode.execute = mcode_execute;

    single_init();
    hal.port.analog_out(port[0],0);
    

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}
