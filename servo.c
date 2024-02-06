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

#include "driver.h"

#if PWM_SERVO_ENABLE == 1

#include <math.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/ioports.h"

#ifndef N_PWM_SERVOS
#define N_PWM_SERVOS 1
#endif

#if N_PWM_SERVOS > 4
#undef N_PWM_SERVOS
#define N_PWM_SERVOS 4
#endif

typedef struct {
    uint8_t port; //Port number, referring to (analog) HAL port number
    xbar_t xport; //Handle to ioport xbar object, obtained at init
    float min_angle;
    float max_angle;
    float angle; //Current setpoint for the angle. (degrees)
} servo_t;

static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

#define DEFAULT_MIN_ANGLE 0.0f
#define DEFAULT_MAX_ANGLE 180.0f

//These are the min and max pulse width in microseconds that are expected by servo. These correspond to the minimum and maximum angle.
#define DEFAULT_MIN_PULSE_WIDTH 544e-6
#define DEFAULT_MAX_PULSE_WIDTH 2400e-6
#define DEFAULT_PWM_FREQ 50.0f

static uint8_t n_servos = 0;
static servo_t servos[N_PWM_SERVOS];

#ifdef DEBUGOUT

// Write CRLF terminated string to current stream
static void write_line_debug (const char *format, ...)
{
    char buffer[100];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    debug_writeln(buffer);
}

#endif

/// @brief 
/// @param servo Servo number
/// @param angle Angle (in degrees) to set servo to
/// @return 
static bool pwm_servo_set_angle(uint8_t servo, float angle)
{
    //Set the position/pwm
    //Servo position is defined from 0 to 180 degrees (left, right)
    //90 degree is the half duty cycle position
    if(servo < n_servos) {
        servos[servo].angle = angle;
        hal.port.analog_out(servos[servo].port, angle);
    }

    return servo < n_servos;
}

static float pwm_servo_get_angle(uint8_t servo)
{
    //Get the servo angle
    return servo < n_servos ? (servos[servo].xport.get_value ? servos[servo].xport.get_value(&servos[servo].xport) : servos[servo].angle) : -1.0f;
}

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == PWMServo_SetPosition
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {
        // M280 P<index> S<pos> 
        case PWMServo_SetPosition:
        //Servo index
            if(gc_block->words.p && (isnanf(gc_block->values.p) || !isintf(gc_block->values.p)))
                state = Status_BadNumberFormat;
            if(gc_block->words.p && (gc_block->values.p >= n_servos))
                state = Status_GcodeValueOutOfRange;
            if(gc_block->words.s && (gc_block->values.s < servos[(uint32_t)gc_block->values.p].min_angle || gc_block->values.s > servos[(uint32_t)gc_block->values.p].max_angle))
                state = Status_GcodeValueOutOfRange;
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

    if (state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

        // M280 P<index> S<pos> 
         case PWMServo_SetPosition:; // Servo mode
            uint8_t servo = (uint8_t)gc_block->values.p;
            if(gc_block->words.s) {
#ifdef DEBUGOUT
                write_line_debug("Setting servo position");
#endif
                pwm_servo_set_angle(servo, gc_block->values.s);
            } else {
                //Reads the position/pwm
                float value = pwm_servo_get_angle(servo);
                if (value >= 0.0f) {
                    char buf[20];
#ifdef DEBUGOUT
                    write_line_debug("[Servo position: %5.2f degrees]",  value);
#endif
                    // TODO: check Marlin format?
                    strcpy(buf, "[Servo ");
                    strcat(buf, uitoa(servo));
                    strcat(buf, " position: ");
                    strcat(buf, ftoa(value, 2));
                    strcat(buf, " degrees]" ASCII_EOL);
                    hal.stream.write(buf);
                }
            }
            break;

        default:
            handled = false;
            break;
    }
    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Servo v0.02]" ASCII_EOL);
}

static bool init_servo_default (servo_t* servo)
{
    servo->max_angle = DEFAULT_MAX_ANGLE;
    servo->min_angle = DEFAULT_MIN_ANGLE;
    servo->angle = 0.0f;

    return true;
}

static bool servo_attach (xbar_t *pwm_pin, uint8_t port, void *data)
{
    static const char *descr[] = {
        "PWM Servo 0",
        "PWM Servo 1",
        "PWM Servo 2",
        "PWM Servo 3"
    };

    if(n_servos < N_PWM_SERVOS && !pwm_pin->cap.servo_pwm) {

        servos[n_servos].port = port;

        if(init_servo_default(&servos[n_servos])) {

            //Initialize default values, and properly configure the pwm
            pwm_config_t config = {
                .freq_hz = DEFAULT_PWM_FREQ,
                .min = DEFAULT_MIN_ANGLE,
                .max = DEFAULT_MAX_ANGLE,
                .off_value = -1.0f, // Never turn off
                .min_value = DEFAULT_MIN_PULSE_WIDTH * DEFAULT_PWM_FREQ * 100.0f,
                .max_value = DEFAULT_MAX_PULSE_WIDTH * DEFAULT_PWM_FREQ * 100.0f, //Percents of duty cycle
                .invert = Off,
                .servo_mode = On
            };

            if(pwm_pin->config(pwm_pin, (void *)&config)) {

                if(pwm_pin->get_value)
                    memcpy(&servos[n_servos].xport, pwm_pin, sizeof(xbar_t));

                if(hal.port.set_pin_description)
                    hal.port.set_pin_description(Port_Analog, Port_Output, port, descr[n_servos]);

                pwm_servo_set_angle(n_servos++, 0.0f);
            }
        }
    }

    return n_servos == N_PWM_SERVOS;
}

void pwm_servo_init (void)
{
    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    hal.user_mcode.check = mcode_check;
    hal.user_mcode.validate = mcode_validate;
    hal.user_mcode.execute = mcode_execute;

    ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .pwm = On, .claimable = On }, servo_attach, NULL);

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // PWM_SERVO_ENABLE
