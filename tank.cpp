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
    _last_front_distances[1], _last_front_distances[2] = _last_front_distances[0], _last_front_distances[1];
    _last_front_distances[0] = int(((micros() - _sonar_front_timer) / 2) / SONAR_FACTOR);
    _sonar_front_timer = micros();
    _sonar_front_state = 3;
}

void sonar_rear_interrupt()
{
    _last_rear_distances[1], _last_rear_distances[2] = _last_rear_distances[0], _last_rear_distances[1];
    _last_rear_distances[0] = int(((micros() - _sonar_rear_timer) / 2) / SONAR_FACTOR);
    _sonar_rear_timer = micros();
    _sonar_rear_state = 3;
}

Tank::Tank(
    int turret_motor_left_pin,
    int turret_motor_right_pin,
    int turret_encoder_pin,
    int turret_calibration_pin,
    int led_pin_1,
    int led_pin_2,
    int sonar_pin_front,
    int sonar_pin_rear
)
{
    _turret_motor_left_pin = turret_motor_left_pin;
    _turret_motor_right_pin = turret_motor_right_pin;
    _turret_encoder_pin = turret_encoder_pin;
    _turret_calibration_pin = turret_calibration_pin;
    _led_pin_1 = led_pin_1;
    _led_pin_2 = led_pin_2;
    _sonar_pin_front = sonar_pin_front;
    _sonar_pin_rear = sonar_pin_rear;

    _led_1_on_delay = 0;
    _led_1_off_delay = 0;
    _led_1_state = 0;
    _led_1_change_millis = 0;
    _led_2_on_delay = 0;
    _led_2_off_delay = 0;
    _led_2_state = 0;
    _led_2_change_millis = 0;

    pinMode(_turret_motor_left_pin, OUTPUT);
    pinMode(_turret_motor_right_pin, OUTPUT);

    pinMode(_turret_encoder_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(_turret_encoder_pin), turret_encoder_interrupt, CHANGE);

    pinMode(_turret_calibration_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);
}

void Tank::do_loop()
{
    _do_sonar(_sonar_pin_front);
    _do_sonar(_sonar_pin_rear);
    _update_leds();
}

void Tank::_do_sonar(unsigned short sonar_pin)
{
    volatile unsigned long &sonar_timer = sonar_pin == _sonar_pin_front ? _sonar_front_timer : _sonar_rear_timer;
    volatile short &sonar_state = sonar_pin == _sonar_pin_front ? _sonar_front_state : _sonar_rear_state;
    void (&interrupt)() = sonar_pin == _sonar_pin_front ? sonar_front_interrupt : sonar_rear_interrupt;

    switch(sonar_state) {
        case 1: // state ready
            // initialize sonar, send pulse, and set interrupt for return
            pinMode(sonar_pin, OUTPUT);
            digitalWrite(sonar_pin, LOW);
            // these signals are required to tell the sonar devie to send
            // normally we try to avoid delaying in our program
            // but these delays are so brief that it should be okay
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

int Tank::front_distance()
{
    return int((_last_front_distances[0] + _last_front_distances[1] + _last_front_distances[2]) / 3);
}

int Tank::rear_distance()
{
    return int((_last_rear_distances[0] + _last_rear_distances[1] + _last_rear_distances[2]) / 3);
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

bool Tank::turret_has_been_calibrated()
{
    return _turret_has_been_calibrated;
}

short Tank::turret_direction()
{
    return _turret_direction;
}
