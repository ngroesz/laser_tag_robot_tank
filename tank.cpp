#include "Arduino.h"
#include "PinChangeInterrupt.h"
#include "IRremote.h"
#include <Wire.h>
#include <PVision.h>

#include "tank.h"

// all the global variables must be intialized here, even if we reinitialize them in the constructor
volatile long _turret_encoder_count = 0;
volatile bool _turret_has_been_calibrated = false;
volatile unsigned long _last_turret_calibration_millis = 0;
short _turret_direction = 0;
volatile unsigned int _sonar_front_distance = -1;
volatile short _sonar_front_state = 0;
volatile unsigned long _sonar_front_timer = 0;
volatile unsigned int _sonar_rear_distance = -1;
volatile short _sonar_rear_state = 0;
volatile unsigned long _sonar_rear_timer = 0;
volatile unsigned short _last_front_distances[3] = {};
volatile unsigned short _last_rear_distances[3] = {};

unsigned long last_debug_output_millis = 0;

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

void sonar_front_interrupt()
{
    //_last_front_distances[1], _last_front_distances[2] = _last_front_distances[0], _last_front_distances[1];
    _last_front_distances[0] = int(((micros() - _sonar_front_timer) / 2) / SONAR_FACTOR);
    _sonar_front_timer = micros();
    _sonar_front_state = 3;
}

void sonar_rear_interrupt()
{
    //_last_rear_distances[1], _last_rear_distances[2] = _last_rear_distances[0], _last_rear_distances[1];
    _last_rear_distances[0] = int(((micros() - _sonar_rear_timer) / 2) / SONAR_FACTOR);
    _sonar_rear_timer = micros();
    _sonar_rear_state = 3;
}

Tank::Tank(
    int motor_enable_pin,
    int drive_left_pin_1,
    int drive_left_pin_2,
    int drive_right_pin_1,
    int drive_right_pin_2,
    int turret_motor_pin_1,
    int turret_motor_pin_2,
    int turret_encoder_pin,
    int turret_calibration_pin,
    int led_pin_1,
    int led_pin_2,
    int sonar_pin_front,
    int sonar_pin_rear
)
{
    _motor_enable_pin       = motor_enable_pin;
    _drive_left_pin_1       = drive_left_pin_1;
    _drive_left_pin_2       = drive_left_pin_2;
    _drive_right_pin_1      = drive_right_pin_1;
    _drive_right_pin_2      = drive_right_pin_2;
    _turret_motor_pin_1     = turret_motor_pin_1;
    _turret_motor_pin_2     = turret_motor_pin_2;
    _turret_encoder_pin     = turret_encoder_pin;
    _turret_calibration_pin = turret_calibration_pin;
    _led_pin_1              = led_pin_1;
    _led_pin_2              = led_pin_2;
    _sonar_pin_front        = sonar_pin_front;
    _sonar_pin_rear         = sonar_pin_rear;

    _left_motor_requested_value               = 0;
    _left_motor_direction                     = 0;
    _left_motor_is_changing_direction         = false;
    _left_motor_last_direction_change_millis  = 0;

    _right_motor_requested_value              = 0;
    _right_motor_direction                    = 0;
    _right_motor_is_changing_direction        = false;
    _right_motor_last_direction_change_millis = 0;

    _led_1_on_delay      = 0;
    _led_1_off_delay     = 0;
    _led_1_state         = 0;
    _led_1_change_millis = 0;
    _led_2_on_delay      = 0;
    _led_2_off_delay     = 0;
    _led_2_state         = 0;
    _led_2_change_millis = 0;

    pinMode(_motor_enable_pin, OUTPUT);
    pinMode(_drive_left_pin_1, OUTPUT);
    pinMode(_drive_left_pin_2, OUTPUT);
    pinMode(_drive_right_pin_1, OUTPUT);
    pinMode(_drive_right_pin_2, OUTPUT);
    pinMode(_turret_motor_pin_1, OUTPUT);
    pinMode(_turret_motor_pin_2, OUTPUT);

    pinMode(_turret_encoder_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(_turret_encoder_pin), turret_encoder_interrupt, CHANGE);

    pinMode(_turret_calibration_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);
}

void Tank::do_loop()
{
    _do_sonar(_sonar_pin_rear, _sonar_rear_timer, _sonar_rear_state, sonar_rear_interrupt);
    _do_motors();
    _update_leds();
}

void Tank::enable_motors()
{
    digitalWrite(_motor_enable_pin, HIGH);
}

void Tank::_do_motors()
{
    _control_motor(
        _drive_left_pin_1,
        _drive_left_pin_2,
        _left_motor_requested_value,
        _left_motor_direction,
        _left_motor_is_changing_direction,
        _left_motor_last_direction_change_millis
    );

    _control_motor(
        _drive_right_pin_1,
        _drive_right_pin_2,
        _right_motor_requested_value,
        _right_motor_direction,
        _right_motor_is_changing_direction,
        _right_motor_last_direction_change_millis
    );

    if (_left_motor_direction == 1) {
        _led_1_on_delay = 1000;
        _led_1_off_delay = 0;
    } else if (_left_motor_direction == -1) {
        _led_1_on_delay = 500;
        _led_1_off_delay = 500;
    } else {
        _led_1_on_delay = 0;
        _led_1_off_delay = 1000;
    }

    if (_right_motor_direction == 1) {
        _led_2_on_delay = 1000;
        _led_2_off_delay = 0;
    } else if (_right_motor_direction == -1) {
        _led_2_on_delay = 500;
        _led_2_off_delay = 500;
    } else {
        _led_2_on_delay = 0;
        _led_2_off_delay = 1000;
    }
}

void Tank::_control_motor(
    const unsigned short &motor_pin_1,
    const unsigned short &motor_pin_2,
    const int &motor_requested_value,
    short &motor_direction,
    bool &motor_is_changing_direction,
    unsigned long &last_motor_direction_change_millis
)
{
    unsigned long current_millis = millis();

    // check if motor is changing direction and if so, set the changing direction flag and stop the motor, and start the timer
    if(!motor_is_changing_direction && ((motor_requested_value >= 0 && motor_direction == -1) || (motor_requested_value <= 0 && motor_direction == 1))) {
        motor_is_changing_direction = true;
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
        last_motor_direction_change_millis = current_millis;
    }

    if (motor_is_changing_direction && current_millis <= last_motor_direction_change_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
        return;
    }

    motor_is_changing_direction = false;

    if (current_millis >= last_debug_output_millis + 1000) {
        Serial.print("motor request direction: ");
        Serial.println(motor_requested_value);
    }

    if(motor_requested_value > 0) {
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        motor_direction = 1;
    } else if(motor_requested_value < 0) {
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        motor_direction = -1;
    } else {
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
        motor_direction = 0;
    }

}

void Tank::_do_sonar(const unsigned short sonar_pin, volatile unsigned long &sonar_timer, volatile short &sonar_state, void (&interrupt)())
{
    switch(sonar_state) {
        case 1: // state ready
            // initialize sonar, send pulse, and set interrupt for return
            pinMode(sonar_pin, OUTPUT);
            // these signals are required to tell the sonar device to send
            // normally we try to avoid delaying in our program
            // but these delays are so brief that it should be okay
            digitalWrite(sonar_pin, LOW);
            delayMicroseconds(2);
            digitalWrite(sonar_pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(sonar_pin, LOW);
            pinMode(sonar_pin, INPUT);
            sonar_state = 2;
            sonar_timer = micros();
            attachPCINT(digitalPinToPCINT(sonar_pin), interrupt, CHANGE);
            break;
        case 2: // state wait
            // wait for sonar return (handled by interrupt)
            break;
        case 3: // state delay
            // if sonar has returned a pulse and the delay between readings is over, set to ready state
            if (micros() >= sonar_timer + SONAR_DELAY_MICROS) {
                sonar_state = 1;
            }
            break;
        default:
            // if undefined, set to ready state
            sonar_state = 1;
    }
}

void Tank::drive(int drive_left_value, int drive_right_value)
{
    drive_left_track(drive_left_value);
    drive_right_track(drive_right_value);
}

void Tank::drive_forward()
{
    drive_left_track(255);
    drive_right_track(255);
}

void Tank::drive_reverse()
{
    drive_left_track(-255);
    drive_right_track(-255);
}

void Tank::drive_stop()
{
    drive_left_track(0);
    drive_right_track(0);
}

void Tank::drive_turn_left()
{
    drive_left_track(-255);
    drive_right_track(255);
}

void Tank::drive_turn_right()
{
    drive_left_track(255);
    drive_right_track(-255);
}

void Tank::drive_left_track(int drive_value)
{
    _left_motor_requested_value = drive_value;
}

void Tank::drive_right_track(int drive_value)
{
    _right_motor_requested_value = drive_value;
}

void Tank::set_led_1(unsigned int led_on_delay, unsigned int led_off_delay)
{
    _led_1_on_delay = led_on_delay;
    _led_1_off_delay = led_off_delay;
}

void Tank::set_led_2(unsigned int led_on_delay, unsigned int led_off_delay)
{
    _led_2_on_delay = led_on_delay;
    _led_2_off_delay = led_off_delay;
}

void Tank::_update_leds()
{
    _update_led(_led_pin_1);
    _update_led(_led_pin_2);
}

void Tank::_update_led(unsigned short led_pin)
{
    unsigned int &on_delay = led_pin == _led_pin_1 ? _led_1_on_delay : _led_2_on_delay;
    unsigned int &off_delay = led_pin == _led_pin_1 ? _led_1_off_delay : _led_2_off_delay;
    unsigned short &led_state = led_pin == _led_pin_1 ? _led_1_state : _led_2_state;
    unsigned long &led_change_millis = led_pin == _led_pin_1 ? _led_1_change_millis : _led_2_change_millis;

    unsigned long current_millis = millis();

    if (led_state == 0 && current_millis >= led_change_millis + off_delay) {
        led_state = 1;
        digitalWrite(led_pin, HIGH);
        led_change_millis = current_millis;
    } else if (led_state == 1 && current_millis >= led_change_millis + on_delay) {
        led_state = 0;
        digitalWrite(led_pin, LOW);
        led_change_millis = current_millis;
    }
}

const int Tank::front_distance()
{
    return int(_last_front_distances[0]);
    return int((_last_front_distances[0] + _last_front_distances[1] + _last_front_distances[2]) / 3);
}

const int Tank::rear_distance()
{
    return int(_last_rear_distances[0]);
    return int((_last_rear_distances[0] + _last_rear_distances[1] + _last_rear_distances[2]) / 3);
}

void Tank::turret_stop()
{
    analogWrite(_turret_motor_pin_1, 0);
    analogWrite(_turret_motor_pin_2, 0);
}

void Tank::turret_left()
{
    if (_turret_direction != -1) {
        turret_stop();
    }
    analogWrite(_turret_motor_pin_1, 255);
    analogWrite(_turret_motor_pin_2, 0);
    _turret_direction = -1;
}

void Tank::turret_right()
{
    if (_turret_direction != 1) {
        turret_stop();
    }
    analogWrite(_turret_motor_pin_1, 0);
    analogWrite(_turret_motor_pin_2, 255);
    _turret_direction = 1;
}

const int Tank::turret_position()
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

const bool Tank::turret_has_been_calibrated()
{
    return _turret_has_been_calibrated;
}

const short Tank::turret_direction()
{
    return _turret_direction;
}
