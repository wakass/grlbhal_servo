#include <stdio.h>

#include "bltouch.h"
#include "servo/servo.h"

#include "hardware/gpio.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"


#ifndef BLTOUCH_SERVO_PORT
    #define BLTOUCH_SERVO_PORT 0 
#endif
//Output_Analog_Aux0

void bltouchInit(){
//Initialization
    _reset();
    _stow();

}

//Hopefully only called on state-change
void bltouchConfigure(bool is_probe_away, bool probing) {
    
    write_line_debug("Configure bltouch. var probing: {%d}",probing);

    probe_state_t probe = hal.probe.get_state();

    probe.triggered = false;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

    gpio_set_inover(PROBE_PIN, probe.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

    if ((probe.is_probing = probing))
        gpio_set_irq_enabled(PROBE_PIN, probe.inverted ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH, true);
    else
        gpio_set_irq_enabled(PROBE_PIN, GPIO_IRQ_ALL, false);
   
    if (probing) {
        //Deploy procedure
        bltouchDeploy();
    }
    else {
        //Stow procedure
        bltouchStow();
    }
}


bool bltouchIsTriggered() {
    //get digital signal? or..
    return hal.probe.get_state().triggered;
}


bool bltouchCommand(uint16_t cmd, uint16_t ms) {
    uint16_t current_angle = (uint16_t) get_angle(BLTOUCH_SERVO_PORT);
    // If the new command is the same, skip it (and the delay).
    // The previous write should've already delayed to detect the alarm.

    write_line_debug("Command bltouch: {%d}",cmd);

    if (cmd != current_angle) {
        set_angle(BLTOUCH_SERVO_PORT,(float)cmd);
        // protocol_execute_realtime();
        // hal.delay_ms(MAX(ms, (uint32_t)BLTOUCH_DELAY),NULL);
        delay_sec(MAX(ms, (uint32_t)BLTOUCH_DELAY)/1e3, DelayMode_SysSuspend);
    }
    return bltouchIsTriggered();
}


//Wrapped functions
void bltouchReset() {}

void bltouchNotifyTriggered() {
    write_line_debug("Notify triggered");
    // (void) bltouchStow();
}

bool bltouchStow() {
    write_line_debug("Stow requested");
    if (_stow_query_alarm()){
        _reset();
        if (_stow_query_alarm()){
            write_line_debug("Stow failed");
            return true;
        }
    }
}

bool bltouchDeploy() {
    write_line_debug("Deploy requested");
    if (_deploy_query_alarm()) {
        bltouchClear();
        if (_deploy_query_alarm()) {
            write_line_debug("Deploy failed");
            return true;
        }
    }
    _SetSWMode();
    return false; // Return success
}

void bltouchClear() {
    _reset();
    _stow();
    _deploy();
    _stow();

}