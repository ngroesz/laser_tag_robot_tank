#include "Arduino.h"
#include "PinChangeInterrupt.h"
#include <IRremote.h>
#include <Wire.h>
#include <PVision.h>

#include "tank_ir.h"
#include "tank.h"

// all the global variables must be intialized here, even if we reinitialize them in the constructor
volatile long           __turret_encoder_count = 0;
volatile bool           __turret_has_been_calibrated = false;
volatile unsigned long  __last_turret_calibration_millis = 0;
short                   __turret_direction = 0;
volatile unsigned int   __sonar_front_distance = -1;
volatile short          __sonar_front_state = 0;
volatile unsigned long  __sonar_front_timer = 0;
volatile unsigned int   __sonar_rear_distance = -1;
volatile short          __sonar_rear_state = 0;
volatile unsigned long  __sonar_rear_timer = 0;
volatile unsigned short __last_front_distances[3] = {};
volatile unsigned short __last_rear_distances[3] = {};
volatile boolean        __ir_data_receiving = false;

unsigned long last_debug_output_millis = 0;

void turret_calibration_interrupt()
{
    unsigned long current_millis = millis();
    if (current_millis < __last_turret_calibration_millis + TURRET_CALIBRATION_DELAY_MILLIS) {
        return;
    }

    __turret_has_been_calibrated = true;
    __turret_encoder_count = 0;
    __last_turret_calibration_millis = current_millis;
}

void turret_encoder_interrupt()
{
    __turret_encoder_count += __turret_direction;
}

void sonar_front_interrupt()
{
    //_last_front_distances[1], _last_front_distances[2] = _last_front_distances[0], _last_front_distances[1];
    __last_front_distances[0] = int(((micros() - __sonar_front_timer) / 2) / SONAR_FACTOR);
    __sonar_front_timer = micros();
    __sonar_front_state = 3;
}

void sonar_rear_interrupt()
{
    //_last_rear_distances[1], _last_rear_distances[2] = _last_rear_distances[0], _last_rear_distances[1];
    __last_rear_distances[0] = int(((micros() - __sonar_rear_timer) / 2) / SONAR_FACTOR);
    __sonar_rear_timer = micros();
    __sonar_rear_state = 3;
}

void ir_receiver_interrupt()
{
    __ir_data_receiving = true;
}

Tank::Tank(
    uint8_t ir_receiver_pin,
    uint8_t motor_enable_pin,
    uint8_t shift_clear_pin,
    uint8_t shift_clock_pin,
    uint8_t shift_data_pin,
    uint8_t turret_encoder_pin,
    uint8_t turret_calibration_pin,
    uint8_t sonar_pin_front,
    uint8_t sonar_pin_rear,
    uint8_t led_pin_1,
    uint8_t led_pin_2,
    uint8_t led_pin_3,
    uint8_t led_pin_4
)
{
    _ir_receiver_pin        = ir_receiver_pin;
    _motor_enable_pin       = motor_enable_pin;
    _shift_clear_pin        = shift_clear_pin;
    _shift_clock_pin        = shift_clock_pin;
    _shift_data_pin         = shift_data_pin;
    _turret_encoder_pin     = turret_encoder_pin;
    _turret_calibration_pin = turret_calibration_pin;
    _sonar_pin_front        = sonar_pin_front;
    _sonar_pin_rear         = sonar_pin_rear;
    _led_pin_1              = led_pin_1;
    _led_pin_2              = led_pin_2;
    _led_pin_3              = led_pin_3;
    _led_pin_4              = led_pin_4;

    _ir_disabled_millis     = 0;
    __ir_data_receiving     = false;
    _ir_is_disabled         = false;
}

void Tank::setup()
{
    _initialize_motors();
    _initalize_turret();
    _initialize_ir();
    _tank_led.setup(_led_pin_1, _led_pin_2, _led_pin_3, _led_pin_4);
    _tank_led.led_1_set_state((const uint16_t[]){500, 500, 500, 500}, 4);
}

void Tank::_initialize_motors()
{
    _left_motor_state.direction = stop;

    pinMode(_motor_enable_pin, OUTPUT);
}

void Tank::_initalize_turret()
{
    pinMode(_turret_encoder_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_encoder_pin), turret_encoder_interrupt, CHANGE);

    pinMode(_turret_calibration_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);
}

void Tank::_initialize_ir()
{
    pinMode(_ir_receiver_pin, INPUT);
    //attachPCINT(digitalPinToPCINT(_ir_receiver_pin), ir_receiver_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_ir_receiver_pin), ir_receiver_interrupt, LOW);
}

void Tank::loop()
{
    _update_ir();
    //_update_motors();
    _tank_led.loop();
    //_update_sonar(_sonar_pin_rear, _sonar_rear_timer, _sonar_rear_state, sonar_rear_interrupt);
}

void Tank::enable_motors()
{
    digitalWrite(_motor_enable_pin, HIGH);
}

void Tank::_update_motors()
{
    _update_motor_directions();
}

void Tank::_update_motor_directions()
{
    _update_motor_direction(_left_motor_state);
    _update_motor_direction(_right_motor_state);
    _update_motor_direction(_turret_motor_state);

    unsigned char new_motor_control_code = _create_motor_control_code();
    if (new_motor_control_code != _motor_control_code) {
        _motor_control_code = new_motor_control_code;
        _write_motor_control_code(_motor_control_code);
    }
}

void Tank::_update_motor_direction(struct motor_state & motor_state)
{
    unsigned long current_millis = millis();

    if (motor_state.direction != motor_state.requested_direction) {
        if (current_millis > motor_state.direction_change_request_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
            motor_state.direction = motor_state.requested_direction;
        }
    }
}

unsigned char Tank::_create_motor_control_code()
{
    unsigned char control_code = 0;

    if (_turret_motor_state.direction == forward) {
        control_code |= _motor_control_mapping.turret_left;
    } else if (_turret_motor_state.direction == reverse) {
        control_code |= _motor_control_mapping.turret_right;
    }

    if (_left_motor_state.direction == forward) {
        control_code |= _motor_control_mapping.left_track_forward;
    } else if (_left_motor_state.direction == reverse) {
        control_code |= _motor_control_mapping.left_track_reverse;
    }

    if (_right_motor_state.direction == forward) {
        control_code |= _motor_control_mapping.right_track_forward;
    } else if (_right_motor_state.direction == reverse) {
        control_code |= _motor_control_mapping.right_track_reverse;
    }

    return control_code;
}

void Tank::_write_motor_control_code(const unsigned char & control_code)
{
}

void Tank::_update_sonar(const unsigned short sonar_pin, volatile unsigned long &sonar_timer, volatile short &sonar_state, void (&interrupt)())
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
            //attachPCINT(digitalPinToPCINT(sonar_pin), interrupt, CHANGE);
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

void Tank::drive(const motor_direction left_direction, const motor_direction right_direction, const uint8_t left_speed, const uint8_t right_speed)
{
    control_left_motor(left_direction, left_speed);
    control_right_motor(right_direction, right_speed);
}

void Tank::drive_forward(const uint8_t left_speed, const uint8_t right_speed)
{
    control_left_motor(forward, left_speed);
    control_right_motor(forward, right_speed);
}

void Tank::drive_reverse(const uint8_t left_speed, const uint8_t right_speed)
{
    control_left_motor(reverse, left_speed);
    control_right_motor(reverse, right_speed);
}

void Tank::drive_stop()
{
    control_left_motor(stop);
    control_right_motor(stop);
}

void Tank::drive_turn_left(uint8_t left_speed, const uint8_t right_speed)
{
    control_left_motor(reverse, left_speed);
    control_right_motor(forward, right_speed);
}

void Tank::drive_turn_right(uint8_t left_speed, const uint8_t right_speed)
{
    control_left_motor(forward, left_speed);
    control_right_motor(reverse, right_speed);
}

void Tank::control_left_motor(const motor_direction direction, const uint8_t speed)
{
    _left_motor_state.requested_direction = direction;

    if (direction == stop) {
        _left_motor_state.direction = stop;
        _left_motor_state.speed = 0;
    } else if (_left_motor_state.requested_direction != _left_motor_state.direction) {
        _left_motor_state.direction_change_request_millis = millis();
    }
}

void Tank::control_right_motor(const motor_direction direction, const uint8_t speed)
{
    _right_motor_state.requested_direction = direction;

    if (direction == stop) {
        _right_motor_state.direction = stop;
        _right_motor_state.speed = 0;
    } else if (_right_motor_state.requested_direction != _right_motor_state.direction) {
        _right_motor_state.direction_change_request_millis = millis();
    }

}

void Tank::turret_left(const uint8_t speed)
{
    control_turret_motor(reverse, speed);
}

void Tank::turret_right(const uint8_t speed)
{
    control_turret_motor(forward, speed);
}

void Tank::turret_stop()
{
    control_turret_motor(stop);
}

void Tank::control_turret_motor(const motor_direction direction, const uint8_t speed)
{
    _turret_motor_state.requested_direction = direction;

    if (direction == stop) {
        _turret_motor_state.direction = stop;
        _turret_motor_state.speed = 0;
    } else if (_turret_motor_state.requested_direction != _turret_motor_state.direction) {
        _turret_motor_state.direction_change_request_millis = millis();
    }
}

const int Tank::front_distance()
{
    return int(__last_front_distances[0]);
    return int((__last_front_distances[0] + __last_front_distances[1] + __last_front_distances[2]) / 3);
}

const int Tank::rear_distance()
{
    return int(__last_rear_distances[0]);
    return int((__last_rear_distances[0] + __last_rear_distances[1] + __last_rear_distances[2]) / 3);
}

const int Tank::turret_position()
{
    if (!__turret_has_been_calibrated) {
        return -1;
    }

    int turret_position = int(__turret_encoder_count / (TURRET_GEAR_RATIO / 360));

    if (turret_position > 360) {
        turret_position -= 360;
    } else if(turret_position < 0) {
        turret_position += 360;
    }

    return turret_position;
}

const bool Tank::turret_has_been_calibrated()
{
    return __turret_has_been_calibrated;
}

const short Tank::turret_direction()
{
    return __turret_direction;
}

void Tank::_update_ir()
{
    unsigned long current_millis = millis();
    if (__ir_data_receiving && !_ir_is_disabled) {
        _ir_is_disabled = 1;
        unsigned long result = read_ir_data(_ir_receiver_pin);
        _ir_disabled_millis = current_millis;
        __ir_data_receiving = false;

        //if (result != 0) {
        //    _process_ir_code(result);
        //}
    }

    if (_ir_is_disabled && current_millis > _ir_disabled_millis + IR_DELAY_BETWEEN_TRANSMISSIONS_MILLIS) {
        _ir_is_disabled = 0;
    }
}

void Tank::_process_ir_code(unsigned long & ir_code)
{
    if (ir_code == IR_CODE_A) {
        //_tank_led.led_1_set_state((const uint16_t[]){1000, 500, 1000, 500, 1000, 500}, 6);
    } else if(ir_code == IR_CODE_B) {
        //_tank_led.led_1_set_state((const uint16_t[]){500, 500, 500, 500}, 4);
    }
}
