#ifndef TANK_CONSTANTS_H
#define TANK_CONSTANTS_H

#define DEBUG_OUTPUT 0

#define SOUND_ENABLED 1

#define MOTORS_ENABLED

#define CONTROL_CODE_RIGHT_MOTOR_FORWARD 4
#define CONTROL_CODE_RIGHT_MOTOR_REVERSE 8
#define CONTROL_CODE_LEFT_MOTOR_FORWARD 1
#define CONTROL_CODE_LEFT_MOTOR_REVERSE 2
#define CONTROL_CODE_TURRET_MOTOR_FORWARD 32
#define CONTROL_CODE_TURRET_MOTOR_REVERSE 16

// delay between motor changing directions. to reduce strain on motors.
#define MOTOR_CHANGE_DIRECTION_DELAY_MILLIS 250
// if no speed is specified
#define MOTOR_DEFAULT_SPEED 255
// the ratio of turret drive encoder counts and one degree of angle.
// this should be possible to determine mathematically, using the turret gearing
// however, i have not done this and this is just an empirically-derived number.
// it is certainly not exact and I think it cannot be exact.
// the larger this number is, the farther the turret will travel to achieve one degree 
// of angle change.
#define TURRET_GEAR_RATIO 1.36

// bump status won't change faster than this delay.
// this effectively debounces the bump switches.
#define BUMP_DETECTION_DELAY_MILLIS 50

// the ratio between a wheel encoder sensor trigger and distance in cms
// this is experimentally determined but could probably be figured out
// by factoring-in wheel diameter and gear ratio.
// this will never be perfect because the tracks can slip somewhat
// and because the wheel encoders have a limited number of poles, which affects
// resolution
#define WHEEL_ENCODER_DISTANCE_RATIO 2

// only check motor targets every X milliseconds.
// this includes drive and turret targets.
// this check involves some math so i don't wish to do it every iteration.
#define MOTOR_TARGET_CHECK_DELAY_MILLIS 50

// similarily for the turret target degrees
#define TURRET_TARGET_DEGREES_CHECK_DELAY_MILLIS 50

#endif
