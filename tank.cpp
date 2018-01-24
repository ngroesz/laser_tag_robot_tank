#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <IRremote.h>
#include <Wire.h>
#include <PVision.h>

#include "tank.h"
#include "ir_codes.h"

// all the global variables must be intialized here, even if we reinitialize them in the constructor
volatile bool __turret_calibration_interrupt_flag;
volatile bool __turret_encoder_interrupt_flag;
volatile bool __sonar_front_interrupt_flag;
volatile bool __sonar_rear_interrupt_flag;

void turret_calibration_interrupt()
{
    __turret_calibration_interrupt_flag = true;
}

void turret_encoder_interrupt()
{
    __turret_encoder_interrupt_flag = true;
}

void sonar_front_interrupt()
{
    __sonar_front_interrupt_flag = true;
}

void sonar_rear_interrupt()
{
    __sonar_rear_interrupt_flag = true;
}

Tank::Tank(
    uint8_t ir_receiver_pin,
    uint8_t motor_enable_pin,
    uint8_t left_motor_pwm_pin,
    uint8_t right_motor_pwm_pin,
    uint8_t turret_motor_pwm_pin,
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
    _left_motor_pwm_pin     = left_motor_pwm_pin;
    _right_motor_pwm_pin    = right_motor_pwm_pin;
    _turret_motor_pwm_pin   = turret_motor_pwm_pin;
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
}

void Tank::setup()
{
    // initialize motors
    pinMode(_motor_enable_pin, OUTPUT);
    pinMode(_left_motor_pwm_pin, OUTPUT);
    pinMode(_right_motor_pwm_pin, OUTPUT);
    pinMode(_turret_motor_pwm_pin, OUTPUT);
    pinMode(_shift_clear_pin, OUTPUT);
    pinMode(_shift_clock_pin, OUTPUT);
    pinMode(_shift_data_pin, OUTPUT);
    _write_motor_control_code(0);

    // initialize turret
    pinMode(_turret_encoder_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_encoder_pin), turret_encoder_interrupt, CHANGE);
    pinMode(_turret_calibration_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);

    // initialize IR
    irrecv = new IRrecv(_ir_receiver_pin);
    irrecv->enableIRIn();

    // initialize LEDs
    _tank_led.setup(_led_pin_1, _led_pin_2, _led_pin_3, _led_pin_4);
    _tank_led.led_1_set_state((const uint16_t[]){500, 500, 500, 500}, 4);
}

void Tank::loop()
{
    if (irrecv->decode(&results)) {
        _process_ir_code(results.value);
        irrecv->resume();
    }

    _process_interrupts();
    _update_motors();
    _tank_led.loop();
    _update_sonar(_sonar_pin_front, _sonar_front_timer, _sonar_front_state, sonar_front_interrupt);
    _update_sonar(_sonar_pin_rear, _sonar_rear_timer, _sonar_rear_state, sonar_rear_interrupt);
}

void Tank::enable_motors(bool enable)
{
    _motors_enabled = enable;
    if (_motors_enabled) {
        digitalWrite(_motor_enable_pin, HIGH);
    } else {
        digitalWrite(_motor_enable_pin, LOW);
    }
}

void Tank::drive(const motor_direction left_direction, const motor_direction right_direction, const uint8_t left_speed, const uint8_t right_speed)
{
    _control_motor(_left_motor_state, left_direction, left_speed);
    _control_motor(_right_motor_state, right_direction, right_speed);
}

void Tank::drive_forward(const uint8_t left_speed, const uint8_t right_speed)
{
    drive(motor_forward, motor_forward, left_speed, right_speed);
}

void Tank::drive_reverse(const uint8_t left_speed, const uint8_t right_speed)
{
    drive(motor_reverse, motor_reverse, left_speed, right_speed);
}

void Tank::drive_turn_left(uint8_t left_speed, const uint8_t right_speed)
{
    drive(motor_reverse, motor_forward, left_speed, right_speed);
}

void Tank::drive_turn_right(uint8_t left_speed, const uint8_t right_speed)
{
    drive(motor_forward, motor_reverse, left_speed, right_speed);
}

void Tank::drive_stop()
{
    _control_motor(_left_motor_state, motor_stop, 0);
    _control_motor(_right_motor_state, motor_stop, 0);
}

void Tank::turret_left(const uint8_t speed)
{
    _control_motor(_turret_motor_state, motor_reverse, speed);
}

void Tank::turret_right(const uint8_t speed)
{
    _control_motor(_turret_motor_state, motor_forward, speed);
}

void Tank::turret_stop()
{
    _control_motor(_turret_motor_state, motor_stop, 0);
}

const uint16_t Tank::turret_position()
{
    if (!turret_has_been_calibrated()) {
        return -1;
    }

    uint16_t turret_position = int(_turret_encoder_count / (TURRET_GEAR_RATIO / 360));

    if (turret_position > 360) {
        turret_position -= 360;
    } else if(turret_position < 0) {
        turret_position += 360;
    }

    return turret_position;
}

const short Tank::turret_direction()
{
    return _turret_direction;
}

const bool Tank::turret_has_been_calibrated()
{
    return _turret_has_been_calibrated;
}

const uint8_t Tank::front_distance()
{
    return _sonar_front_distance;
}

const uint8_t Tank::rear_distance()
{
    return _sonar_rear_distance;
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

    if (_tank_mode == mode_debug_sonar) {
        if (_sonar_front_distance <= SONAR_DISTANCE_EMERGENCY) {
            if (_sonar_front_distance_warning != distance_warning_emergency) {
                _tank_led.led_3_set_state((const uint16_t[]){500, 500}, 2);
                _tank_led.led_4_set_state((const uint16_t[]){500, 500}, 2);
                _sonar_front_distance_warning = distance_warning_emergency;
            }
        } else if (_sonar_front_distance <= SONAR_DISTANCE_WARNING) {
            Serial.println("hwy");
            if (_sonar_front_distance_warning != distance_warning_warning) {
                _tank_led.led_3_set_state((const uint16_t[]){1000, 500}, 2);
                _sonar_front_distance_warning = distance_warning_warning;
            }
        } else {
            Serial.print("herE: ");
            Serial.println(_sonar_front_distance);
            if (_sonar_front_distance_warning != distance_warning_none) {
                _tank_led.led_3_turn_off();
                _tank_led.led_4_turn_off();
                _sonar_front_distance_warning = distance_warning_none;
            }
        }
    }
}

void Tank::_process_ir_code(unsigned long & ir_code)
{
    if (ir_code == IR_CODE_A) {
        _tank_mode = mode_fight;
        _tank_led.led_1_turn_off();
        _tank_led.led_2_turn_off();
        _tank_led.led_4_turn_off();
    } else if(ir_code == IR_CODE_B) {
        _tank_led.led_1_turn_on();
        _tank_led.led_4_turn_off();
        if (_tank_mode == mode_debug_drive) {
            _tank_mode = mode_debug_turret;
            _tank_led.led_2_set_state((const uint16_t[]){500, 500, 500, 2000}, 4);
        } else if (_tank_mode == mode_debug_turret) {
            _tank_mode = mode_debug_sonar;
            _tank_led.led_2_set_state((const uint16_t[]){500, 500, 500, 500, 500, 2000}, 6);
        } else {
            _tank_mode = mode_debug_drive;
            _tank_led.led_2_set_state((const uint16_t[]){500, 2000}, 2);
        }
    } else if(ir_code == IR_CODE_C) {
        _tank_led.led_1_set_state((const uint16_t[]){900, 100}, 2);
    } else if(ir_code == IR_CODE_UP) {
        _tank_led.led_1_set_state((const uint16_t[]){100, 900}, 2);
        drive_forward(255, 255);
    } else if(ir_code == IR_CODE_DOWN) {
        _tank_led.led_2_set_state((const uint16_t[]){100, 900}, 2);
        drive_reverse(255, 255);
    } else if(ir_code == IR_CODE_LEFT) {
    } else if(ir_code == IR_CODE_RIGHT) {
        _tank_led.led_4_set_state((const uint16_t[]){100, 900}, 2);
    } else if(ir_code == IR_CODE_SELECT) {
        drive_stop();
        _tank_led.led_1_turn_off();
        _tank_led.led_2_turn_off();
        _tank_led.led_4_turn_off();
    }

}

void Tank::_update_motors()
{
    _update_motor(_left_motor_pwm_pin, _left_motor_state);
    _update_motor(_right_motor_pwm_pin, _right_motor_state);
    _update_motor(_turret_motor_pwm_pin, _turret_motor_state);

    unsigned char new_motor_control_code = _create_motor_control_code();

    if (new_motor_control_code != _motor_control_code) {
        _motor_control_code = new_motor_control_code;
        _write_motor_control_code(_motor_control_code);
    }

}

void Tank::_update_motor(const uint8_t motor_pin, motor_state & motor_state)
{
    if (motor_state.direction != motor_state.requested_direction) {
        // here we check if the motor is recently switching directions
        // and, if so, we just write zero to the speed-control pin
        // and skip the rest of the function
        if (motor_state.direction != motor_stop
            && millis() < motor_state.direction_change_request_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
            motor_state.last_speed = 0;
            digitalWrite(motor_pin, 0);
            return;
        } else {
            motor_state.direction = motor_state.requested_direction;
        }
    }

    if (motor_state.speed != motor_state.last_speed) {
        digitalWrite(motor_pin, motor_state.speed);
        motor_state.last_speed = motor_state.speed;
    }
}

void Tank::_control_motor(struct motor_state & state, const motor_direction direction, const uint8_t speed)
{
    state.requested_direction = direction;
    state.speed = speed;

    if (direction == motor_stop) {
        state.direction = motor_stop;
        state.speed = 0;
    } else if (state.requested_direction != state.direction) {
        state.direction_change_request_millis = millis();
    }
}

unsigned char Tank::_create_motor_control_code()
{
    unsigned char control_code = 0;

    if (_left_motor_state.direction == motor_forward) {
        control_code |= _motor_control_mapping.left_track_forward;
    } else if (_left_motor_state.direction == motor_reverse) {
        control_code |= _motor_control_mapping.left_track_reverse;
    }

    if (_right_motor_state.direction == motor_forward) {
        control_code |= _motor_control_mapping.right_track_forward;
    } else if (_right_motor_state.direction == motor_reverse) {
        control_code |= _motor_control_mapping.right_track_reverse;
    }

    if (_turret_motor_state.direction == motor_forward) {
        control_code |= _motor_control_mapping.turret_left;
    } else if (_turret_motor_state.direction == motor_reverse) {
        control_code |= _motor_control_mapping.turret_right;
    }

    return control_code;
}

void Tank::_write_motor_control_code(const unsigned char & control_code)
{
    if (!_motors_enabled) {
        return;
    }

    char write_code = control_code;

    digitalWrite(_motor_enable_pin, LOW);
    digitalWrite(_shift_clear_pin, LOW);
    digitalWrite(_shift_clear_pin, HIGH);
    _shift_bit(0);
    _shift_bit(0);
    for (int i = 0; i < 6; ++i) {
        if (write_code % 2 == 1) {
            _shift_bit(1);
        } else {
            _shift_bit(0);
        }
        write_code >>= 1;
    }
    digitalWrite(_shift_clock_pin, LOW);
    digitalWrite(_shift_clock_pin, HIGH);
    digitalWrite(_motor_enable_pin, HIGH);
}

void Tank::_shift_bit(bool bit)
{
    digitalWrite(_shift_clock_pin, LOW);
    if (bit) {
        digitalWrite(_shift_data_pin, HIGH);
    } else {
        digitalWrite(_shift_data_pin, LOW);
    }
    digitalWrite(_shift_clock_pin, HIGH);
}

void Tank::_process_interrupts()
{
    if (__turret_calibration_interrupt_flag) {
        _process_turret_calibration_interrupt();
        __turret_calibration_interrupt_flag = false;
    }

    if (__turret_encoder_interrupt_flag) {
        _process_turret_encoder_interrupt();
        __turret_encoder_interrupt_flag = false;
    }

    if ( __sonar_front_interrupt_flag) {
        _process_sonar_front_interrupt();
        __sonar_front_interrupt_flag = false;
    }

    if ( __sonar_rear_interrupt_flag) {
        _process_sonar_rear_interrupt();
        __sonar_rear_interrupt_flag = false;
    }
}

void Tank::_process_turret_calibration_interrupt()
{
    unsigned long current_millis = millis();
    if (current_millis < _last_turret_calibration_millis + TURRET_CALIBRATION_DELAY_MILLIS) {
        return;
    }

    _turret_has_been_calibrated = true;
    _turret_encoder_count = 0;
    _last_turret_calibration_millis = current_millis;
}

void Tank::_process_turret_encoder_interrupt()
{
    _turret_encoder_count += _turret_direction;
}

void Tank::_process_sonar_front_interrupt()
{
    _sonar_front_distance = uint16_t(((micros() - _sonar_front_timer) / 2) / SONAR_FACTOR);
    _sonar_front_timer = micros();
    _sonar_front_state = 3;
}

void Tank::_process_sonar_rear_interrupt()
{
    _sonar_rear_distance = uint16_t(((micros() - _sonar_rear_timer) / 2) / SONAR_FACTOR);
    _sonar_rear_timer = micros();
    _sonar_rear_state = 3;
}
