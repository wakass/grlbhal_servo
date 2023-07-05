# Servo 
The servo plugin implements pwm control of servos. 

The plugin allows abstraction of the pwm-pins and the use of the ```set_angle``` and ```get_angle``` functions. It also provides Marlin-style M280 commands for easy interfacing and manual control.

It relies on the "ioport" output ports defined in grblhal. The port should specifically be of the analog type and pwm-capable.


## Enable Servo
Required:
```
#define HAS_IOPORTS
#define SERVO
#define AUXOUTPUT0_PWM_PIN  15 //Define the GPIO pin that will output pwm and is attached to the servo. Done for every N_SERVO
```
Optional:
```
#define N_SERVOS 1 //Default, servos are claimed sequentially from available PWM ports
```




## IOport pins
For context, "ioport" pins are defined in driver.c 
```
#ifdef AUXOUTPUT0_PWM_PIN
    { .id = Output_Analog_Aux0, .port = GPIO_OUTPUT, .pin = AUXOUTPUT0_PWM_PIN, .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
```



## M280 Commands
M280 P\<index\> S\<pos\>


```M280 P0 S10``` 
sets the servo angle 

```M280 P0 S``` 
prints the current set servo angle

## BLtouch
The bltouch plugin depends on the servos that are defined. This is because the module takes PWM-like servo commands.

To enable use:
```#define HAS_BLTOUCH            31 //Version 3.1```

This overwrites other probes that might be attached to the system.

### Port
As the bltouch plugin relies on servo, the ports are defined in servo-number land. The plugin takes the 0th servo port by default.

```#define BLTOUCH_SERVO_PORT 0```

### Probe pin 
This ofcourse should be defined to trigger the probe.

```#define PROBE_PIN           28```
