/*

  bltouch.c - auto deploy & stove of bltouch probe

  Part of grblHAL

  Public domain.

*/

#include "driver.h"

#if BLTOUCH_ENABLE == 1

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"

static xbar_t servo = {0};
static uint8_t servo_port;
static uint16_t current_angle;
static on_probe_start_ptr on_probe_start;
static on_probe_completed_ptr on_probe_completed;
static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;
static bool high_speed = false;

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

// typedef bool (*on_probe_start_ptr)(axes_signals_t axes, float *target, plan_line_data_t *pl_data);
// typedef void (*on_probe_completed_ptr)(void);
// on_probe_start_ptr on_probe_start;
// on_probe_completed_ptr on_probe_completed;


//// SIGNAL AND DELAY DEFINITIONS
/// This is from Marlin firmware, seems reasonable.
// BLTouch commands are sent as servo angles
typedef unsigned char BLTCommand;

#define STOW_ALARM            true
#define BLTOUCH_DEPLOY          10
#define BLTOUCH_STOW            90
#define BLTOUCH_SW_MODE         60
#define BLTOUCH_SELFTEST       120
#define BLTOUCH_MODE_STORE     130
#define BLTOUCH_5V_MODE        140
#define BLTOUCH_OD_MODE        150
#define BLTOUCH_RESET          160

/**
 * The following commands require different minimum delays.
 *
 * 500ms required for a reliable Reset.
 *
 * 750ms required for Deploy/Stow, otherwise the alarm state
 *       will not be seen until the following move command.
 */

// Safety: The probe needs time to recognize the command.
//         Minimum command delay (ms). Enable and increase if needed.
#define BLTOUCH_DELAY 500

#ifndef BLTOUCH_SET5V_DELAY
  #define BLTOUCH_SET5V_DELAY   150
#endif
#ifndef BLTOUCH_SETOD_DELAY
  #define BLTOUCH_SETOD_DELAY   150
#endif
#ifndef BLTOUCH_MODE_STORE_DELAY
  #define BLTOUCH_MODE_STORE_DELAY 150
#endif
#ifndef BLTOUCH_DEPLOY_DELAY
  #define BLTOUCH_DEPLOY_DELAY   750
#endif
#ifndef BLTOUCH_STOW_DELAY
  #define BLTOUCH_STOW_DELAY     750
#endif
#ifndef BLTOUCH_RESET_DELAY
  #define BLTOUCH_RESET_DELAY    500
#endif

bool bltouch_cmd (uint16_t cmd, uint16_t ms)
{
    // current_angle = (uint16_t) get_angle(BLTOUCH_SERVO_PORT);
    // If the new command is the same, skip it (and the delay).
    // The previous write should've already delayed to detect the alarm.

#ifdef DEBUGOUT
    write_line_debug("Command bltouch: {%d}",cmd);
#endif

    if(cmd != (servo.get_value ? servo.get_value(&servo) : current_angle)) {

        current_angle = cmd;

        hal.port.analog_out(servo_port, (float)cmd);

        delay_sec(max((float)ms / 1e3f, (float)BLTOUCH_DELAY / 1e3f), DelayMode_SysSuspend);
    }

    return true;
}

static status_code_t bltoutch_selftest (sys_state_t state, char *args)
{
    // NOOP for now

    return Status_OK;
}

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == Probe_Deploy || mcode == Probe_Stow
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case Probe_Deploy:

            if(gc_block->words.s) {
                if(isnanf(gc_block->values.s) || !isintf(gc_block->values.s))
                    state = Status_BadNumberFormat;
                else if(gc_block->values.s < -0.0f || gc_block->values.s > 1.0f)
                    state = Status_GcodeValueOutOfRange;
            }
            gc_block->words.h = gc_block->words.s = Off;
            break;

        case Probe_Stow:
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

    if(state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

         case Probe_Deploy:
             if(gc_block->words.s)
                 high_speed = gc_block->values.s != 0.0f;
             if(gc_block->words.h) {
                 hal.stream.write("[PROBE HS:");
                 hal.stream.write(uitoa(high_speed));
                 hal.stream.write("]" ASCII_EOL);
             }
             if(!(gc_block->words.s || gc_block->words.h))
                 bltouch_cmd(BLTOUCH_DEPLOY, BLTOUCH_DEPLOY_DELAY);
             break;

         case Probe_Stow:;
             bltouch_cmd(BLTOUCH_STOW, BLTOUCH_STOW_DELAY);
             break;

         default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static bool onProbeStart (axes_signals_t axes, float *target, plan_line_data_t *pl_data)
{
    bool ok = on_probe_start == NULL || on_probe_start(axes, target, pl_data);

    if(!high_speed && ok)
        bltouch_cmd(BLTOUCH_DEPLOY, BLTOUCH_DEPLOY_DELAY);

    return ok;
}

static void onProbeCompleted (void)
{
    if(!high_speed)
        bltouch_cmd(BLTOUCH_STOW, BLTOUCH_STOW_DELAY);

    if(on_probe_completed)
        on_probe_completed();
}

const sys_command_t bltouch_command_list[] = {
    {"BLTEST", bltoutch_selftest, {}, { .str = "perform BLTouch probe self-test" } },
};

static sys_commands_t bltouch_commands = {
    .n_commands = sizeof(bltouch_command_list) / sizeof(sys_command_t),
    .commands = bltouch_command_list
};

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:BLTouch v0.02]" ASCII_EOL);
}

static bool claim_servo (xbar_t *servo_pwm, uint8_t port, void *data)
{
    servo_port = port;

    if(ioport_claim(Port_Analog, Port_Output, &servo_port, "BLTouch probe")) {

        if(servo_pwm->get_value)
            memcpy(&servo, servo_pwm, sizeof(xbar_t));

        return true;
    }

    return false;
}

void bltouch_init (void)
{
    if(ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .servo_pwm = On, .claimable = On }, claim_servo, NULL)) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

        on_probe_start = grbl.on_probe_start;
        grbl.on_probe_start = onProbeStart;

        on_probe_completed = grbl.on_probe_completed;
        grbl.on_probe_completed = onProbeCompleted;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        system_register_commands(&bltouch_commands);

        bltouch_cmd(BLTOUCH_STOW, BLTOUCH_STOW_DELAY);
    } else
        protocol_enqueue_foreground_task(report_warning, "No servo PWM output available for BLTouch!");
}

#endif // BLTOUCH_ENABLE
