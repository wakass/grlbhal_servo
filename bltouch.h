#ifndef _BLTOUCH_H_
#define _BLTOUCH_H_

#include "grbl/hal.h"

void bltouchConfigure(bool is_probe_away, bool probing);
bool bltouchCommand(uint16_t cmd, uint16_t ms);
void bltouchInit();
bool bltouchStow();
bool bltouchDeploy();
void bltouchReset();
void bltouchClear();

void bltouchNotifyStow();

#define BLTOUCH_LIM_DEBOUNCE 40


// Write CRLF terminated string to current stream
static void write_line (char *s)
{
    strcat(s, ASCII_EOL);
    hal.stream.write(s);
}

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

static bool _deploy_query_alarm() { return bltouchCommand(BLTOUCH_DEPLOY, BLTOUCH_DEPLOY_DELAY); }
static bool _stow_query_alarm()   { return bltouchCommand(BLTOUCH_STOW, BLTOUCH_STOW_DELAY) == STOW_ALARM; }

//Raw bltouch-native
inline void _reset() {
    bltouchCommand(BLTOUCH_RESET, BLTOUCH_RESET_DELAY);
}
inline void _stow() {
    bltouchCommand(BLTOUCH_STOW, BLTOUCH_STOW_DELAY);
}
inline void _deploy() { 
    bltouchCommand(BLTOUCH_DEPLOY, BLTOUCH_DEPLOY_DELAY); 
}
inline void _SetSWMode() {
    bltouchCommand(BLTOUCH_SW_MODE, BLTOUCH_SW_MODE);
}
#endif