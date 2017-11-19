#include "Arduino.h"
#include "PinChangeInterrupt.h"

#include "tank.h"

volatile long _turret_encoder_count = 0;
volatile bool _turret_has_been_calibrated = false;
volatile unsigned long _last_turret_calibration_millis = 0;
short _turret_direction = 0;

void turret_calibration_interrupt()
{
    unsigned long current_millis = millis();
    if (current_millis < _last_turret_calibration_millis + TURRET_CALIBRATION_DELAY_MILLIS) {
        return;
    }

    _turret_has_been_calibrated = true;
    _turret_encoder_count = 0;
    _last_turret_calibration_millis = current_millis;
}

void turret_encoder_interrupt()
{
    _turret_encoder_count += _turret_direction;
}

Tank::Tank(int turret_motor_left_pin, int turret_motor_right_pin, int turret_encoder_pin, int turret_calibration_pin)
{
    _turret_motor_left_pin = turret_motor_left_pin;
    _turret_motor_right_pin = turret_motor_right_pin;
    _turret_encoder_pin = turret_encoder_pin;
    _turret_calibration_pin = turret_calibration_pin;

    _last_output_millis = 0;

    pinMode(_turret_motor_left_pin, OUTPUT);
    pinMode(_turret_motor_right_pin, OUTPUT);

    pinMode(_turret_encoder_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(_turret_encoder_pin), turret_encoder_interrupt, CHANGE);

    pinMode(_turret_calibration_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);
}

void Tank::turret_stop()
{
    analogWrite(_turret_motor_left_pin, 0);
    analogWrite(_turret_motor_right_pin, 0);
}

void Tank::turret_left()
{
    if (_turret_direction != -1) {
        turret_stop();
    }
    analogWrite(_turret_motor_left_pin, 255);
    analogWrite(_turret_motor_right_pin, 0);
    _turret_direction = -1;
}

void Tank::turret_right()
{
    if (_turret_direction != 1) {
        turret_stop();
    }
    analogWrite(_turret_motor_left_pin, 0);
    analogWrite(_turret_motor_right_pin, 255);
    _turret_direction = 1;
}

int Tank::turret_position()
{
    if (!_turret_has_been_calibrated) {
        return -1;
    }

    int turret_position = int(_turret_encoder_count / (TURRET_GEAR_RATIO / 360));

    if (turret_position > 360) {
        turret_position -= 360;
    } else if(turret_position < 0) {
        turret_position += 360;
    }

    return turret_position;
}

// debugging function
int Tank::turret_encoder_count()
{
    return _turret_encoder_count;
}

bool Tank::turret_has_been_calibrated()
{
    return _turret_has_been_calibrated;
}

short Tank::turret_direction()
{
    return _turret_direction;
}
