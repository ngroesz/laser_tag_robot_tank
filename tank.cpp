#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <IRremote.h>
#include <Wire.h>
#include <PVision.h>

#include "tank.h"
#include "ir_codes.h"

// all the global variables must be initialized here, even if we reinitialize them in the constructor
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
    uint8_t pins[] = {_led_pin_1, _led_pin_2, _led_pin_3, _led_pin_4};
    _tank_led.setup(pins);

    _tank_led.led_set_state(0, (const uint16_t[]){500, 500, 500, 500}, 4);
}

void Tank::loop()
{

    if (irrecv->decode(&results)) {
        _process_ir_code(results.value);
        irrecv->resume();
    }

    //_process_interrupts();

    _update_motors();
    _tank_led.loop();


    //Serial.println("motors off");
    //_motors_enabled = 1;
    //analogWrite(_left_motor_pwm_pin, 0);
    //analogWrite(_right_motor_pwm_pin, 0);
    //_write_motor_control_code(0);
    //delay(5000);
    //Serial.println("motors on");
    //analogWrite(_left_motor_pwm_pin, 255);
    //analogWrite(_right_motor_pwm_pin, 255);
    //_write_motor_control_code(40);
    //delay(5000);
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
#ifdef DEBUG
    Serial.println("drive forward");
#endif
    drive(motor_forward, motor_forward, left_speed, right_speed);
}

void Tank::drive_reverse(const uint8_t left_speed, const uint8_t right_speed)
{
#ifdef DEBUG
    Serial.println("drive reverse");
#endif
    drive(motor_reverse, motor_reverse, left_speed, right_speed);
}

void Tank::drive_turn_left(uint8_t left_speed, const uint8_t right_speed)
{
#ifdef DEBUG
    Serial.println("drive left");
#endif
    drive(motor_reverse, motor_forward, left_speed, right_speed);
}

void Tank::drive_turn_right(uint8_t left_speed, const uint8_t right_speed)
{
#ifdef DEBUG
    Serial.println("drive right");
#endif
    drive(motor_forward, motor_reverse, left_speed, right_speed);
}

void Tank::drive_stop()
{
#ifdef DEBUG
    Serial.println("drive stop");
#endif
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

void Tank::_process_ir_code(unsigned long & ir_code)
{
    if (ir_code == IR_CODE_END) {
        return;
    }

#ifdef DEBUG
    Serial.print("Processing IR code: ");
    Serial.println(ir_code, HEX);
#endif
    if (ir_code == IR_CODE_A) {
        _tank_mode = mode_fight;
        _tank_led.led_turn_off(0);
        _tank_led.led_turn_off(1);
        _tank_led.led_turn_off(3);
    } else if(ir_code == IR_CODE_B) {
#ifdef DEBUG
        Serial.println("IR_CODE_B");
#endif
        _tank_led.led_turn_on(0);
        _tank_mode = mode_debug_drive;
        _tank_led.led_set_state(1, (const uint16_t[]){500, 2000}, 2);
    } else if(ir_code == IR_CODE_C) {
    } else if(ir_code == IR_CODE_UP) {
        if (_tank_mode == mode_debug_drive) {
            turret_left();
            drive_forward();
        }
    } else if(ir_code == IR_CODE_DOWN) {
        if (_tank_mode == mode_debug_drive) {
            drive_reverse();
        }
    } else if(ir_code == IR_CODE_LEFT) {
        if (_tank_mode == mode_debug_drive) {
            if (_left_motor_state.requested_speed >= 10) {
                _left_motor_state.requested_speed -= 10;
            } else {
                _left_motor_state.requested_speed = 0;
            }
            if (_right_motor_state.requested_speed >= 10) {
                _right_motor_state.requested_speed -= 10;
            } else {
                _right_motor_state.requested_speed = 0;
            }
        }
    } else if(ir_code == IR_CODE_RIGHT) {
        if (_tank_mode == mode_debug_drive) {
            if (_left_motor_state.requested_speed <= 245) {
                _left_motor_state.requested_speed += 10;
            } else {
                _left_motor_state.requested_speed = 255;
            }
            if (_right_motor_state.requested_speed <= 245) {
                _right_motor_state.requested_speed += 10;
            } else {
                _right_motor_state.requested_speed = 255;
            }
        }
    } else if(ir_code == IR_CODE_SELECT) {
        if (_tank_mode == mode_debug_drive) {
            drive_stop();
        }
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
    unsigned long current_millis = millis();
    if (motor_state.direction != motor_state.requested_direction) {
        // here we check if the motor is recently switching directions
        // and, if so, we just write zero to the speed-control pin
        // and skip the rest of the function
        if (motor_state.direction != motor_stop
            && current_millis < motor_state.direction_change_request_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
            motor_state.last_speed = 0;
            analogWrite(motor_pin, 0);
            return;
        } else {
            motor_state.direction = motor_state.requested_direction;
        }
    }

    if (motor_state.speed != motor_state.requested_speed && current_millis > motor_state.speed_change_millis + MOTOR_SPEED_CHANGE_DELAY_MILLIS) {
        motor_state.speed += motor_state.speed < motor_state.requested_speed ? 1 : -1;
        motor_state.speed_change_millis = current_millis;
    }

    if (motor_state.speed != motor_state.last_speed) {
        analogWrite(motor_pin, motor_state.speed);
        motor_state.last_speed = motor_state.speed;
    }
}

void Tank::_control_motor(struct motor_state & state, const motor_direction direction, const uint8_t speed)
{
    state.requested_direction = direction;
    state.requested_speed = speed;

    if (direction == motor_stop) {
        state.direction = motor_stop;
        state.requested_speed = 0;
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
    _update_motor_debug_leds();

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

void Tank::_update_motor_debug_leds()
{
    if (_tank_mode == mode_debug_drive) {
        if (_left_motor_state.direction == motor_forward) {
            _tank_led.led_turn_on(2);
        } else if (_left_motor_state.direction == motor_reverse) {
            _tank_led.led_set_state(2, (const uint16_t[]){500, 500}, 2);
        } else {
            _tank_led.led_turn_off(2);
        }

        if (_right_motor_state.direction == motor_forward) {
            _tank_led.led_turn_on(3);
        } else if (_right_motor_state.direction == motor_reverse) {
            _tank_led.led_set_state(3, (const uint16_t[]){500, 500}, 2);
        } else {
            _tank_led.led_turn_off(3);
        }
    }
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
