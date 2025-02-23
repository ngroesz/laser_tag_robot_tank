#include "tank.h"

#include <irmp.hpp>
// all the global variables must be initialized here, even if we reinitialize them in the constructor
volatile bool __turret_calibration_interrupt_flag;
volatile bool __turret_encoder_interrupt_flag;
volatile bool __bump_interrupt_flag;
volatile byte __last_ir_code;

void _turret_calibration_interrupt()
{
    __turret_calibration_interrupt_flag = true;
}

void _turret_encoder_interrupt()
{
    __turret_encoder_interrupt_flag = true;
}

void _bump_interrupt()
{
    __bump_interrupt_flag = true;
}

void _ir_interrupt()
{
    IRMP_DATA irmp_data;
    irmp_get_data(&irmp_data);
    interrupts();
    __last_ir_code = irmp_data.command;
}

Tank::Tank()
{
}

void Tank::setup()
{
    _write_motor_control_code(0);

    // initialize LEDs
    uint8_t pins[] = {LED_PIN_1, LED_PIN_2, LED_PIN_3};
    _tank_led.setup(pins);

    _tank_led.all_off();

    _tank_led.led_turn_on(0);
    delay(250);

    // initialize motors
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(TURRET_MOTOR_PWM_PIN, OUTPUT);
    pinMode(SHIFT_CLEAR_PIN, OUTPUT);
    pinMode(SHIFT_CLOCK_PIN, OUTPUT);
    pinMode(SHIFT_DATA_PIN, OUTPUT);

    _left_motor_state = {
        .direction = motor_stop,
        .requested_direction = motor_stop,
        .speed = 0,
        .direction_change_requested = false,
        .direction_change_request_millis = 0
    };
    _right_motor_state = {
        .direction = motor_stop,
        .requested_direction = motor_stop,
        .speed = 0,
        .direction_change_requested = false,
        .direction_change_request_millis = 0
    };


    // initialize turret
    //pinMode(TURRET_ENCODER_PIN, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(TURRET_ENCODER_PIN), turret_encoder_interrupt, CHANGE);
    //pinMode(_turret_calibration_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);

    _tank_led.led_turn_on(1);
    delay(250);

    // initialize IR
    //irmp_init();
    //irmp_register_complete_callback_function(&_ir_interrupt);

    _tank_led.led_turn_on(2);
    delay(250);

    _tank_led.all_off();
}

void Tank::loop()
{
    //if (irrecv.decode(&results)) {
    //    uint16_t resultCode = (results.value & 0xFFFF);
    //    Serial.print("IR code: " );
    //    Serial.println(resultCode, HEX);
    //    _process_ir_code(results.value);
    //    irrecv.resume();
    //}

    ////if (turret_has_been_calibrated() {
    ////} else {
    ////    _tank_led.led_turn_on(1);

    //if (irrecv.decode(&results)) {
    //    uint16_t resultCode = (results.value & 0xFFFF);
    //    Serial.print("IR code: " );
    //    Serial.println(resultCode, HEX);
    //    _process_ir_code(results.value);
    //    irrecv.resume();
    //}

    //_process_interrupts();

    _update_motors();
    //_tank_led.loop();


    ////Serial.println("motors off");
    //_motors_enabled = 1;
    ////analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    ////analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
    ////_write_motor_control_code(0);
    ////delay(5000);
    //Serial.println("motors on");
    //analogWrite(LEFT_MOTOR_PWM_PIN, 200);
    //analogWrite(RIGHT_MOTOR_PWM_PIN, 200);
    //_write_motor_control_code(40);
    ////delay(10000);
}

byte Tank::last_ir_code()
{
    return __last_ir_code;
}

void Tank::enable_motors(bool enable)
{
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

//void Tank::_process_ir_code(const unsigned long & ir_code)
//{
//    if (ir_code == IR_CODE_END) {
//        return;
//    }
//
//    _last_ir_code = ir_code;
//
//#ifdef DEBUG_OUTPUT
//    Serial.print("Processing IR code: ");
//    Serial.println(ir_code, HEX);
//#endif
//    if (ir_code == IR_CODE_A) {
//        _tank_mode = mode_fight;
//        _tank_led.led_turn_off(0);
//        _tank_led.led_turn_off(1);
//        _tank_led.led_turn_off(3);
//    } else if(ir_code == IR_CODE_B) {
//#ifdef DEBUG_OUTPUT
//        Serial.println("IR_CODE_B");
//#endif
//        _tank_led.led_turn_on(0);
//        _tank_mode = mode_debug_drive;
//        _tank_led.led_set_state(1, (const uint16_t[]){500, 2000}, 2);
//    } else if(ir_code == IR_CODE_C) {
//    } else if(ir_code == IR_CODE_UP) {
//        if (_tank_mode == mode_debug_drive) {
//            drive_forward();
//        }
//    } else if(ir_code == IR_CODE_DOWN) {
//        if (_tank_mode == mode_debug_drive) {
//            drive_reverse();
//        }
//    } else if(ir_code == IR_CODE_LEFT) {
//        if (_tank_mode == mode_debug_drive) {
//            drive_turn_left();
//        }
//    } else if(ir_code == IR_CODE_RIGHT) {
//        if (_tank_mode == mode_debug_drive) {
//            drive_turn_right();
//        }
//    } else if(ir_code == IR_CODE_SELECT) {
//        if (_tank_mode == mode_debug_drive) {
//            drive_stop();
//        }
//    }
//
//}

void Tank::_update_motors()
{
    unsigned char new_motor_control_code = 0;
    new_motor_control_code |= _update_motor(CONTROL_CODE_LEFT_MOTOR_FORWARD, CONTROL_CODE_LEFT_MOTOR_REVERSE, LEFT_MOTOR_PWM_PIN, _left_motor_state);
    new_motor_control_code |= _update_motor(CONTROL_CODE_RIGHT_MOTOR_FORWARD, CONTROL_CODE_RIGHT_MOTOR_REVERSE, RIGHT_MOTOR_PWM_PIN, _right_motor_state);

    if (new_motor_control_code != _motor_control_code) {
        _motor_control_code = new_motor_control_code;
        _write_motor_control_code(_motor_control_code);
    }
}

uint8_t Tank::_update_motor(const uint8_t forward_code, const uint8_t reverse_code, const uint8_t motor_pin, motor_state & motor_state)
{
    analogWrite(motor_pin, motor_state.speed);

    unsigned long current_millis = millis();
    if (motor_state.direction_change_requested) {
        if (current_millis > motor_state.direction_change_request_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
            motor_state.direction = motor_state.requested_direction;
            motor_state.direction_change_requested = false;
        } else {
            analogWrite(motor_pin, 0);
            return 0;
        }
    }

    analogWrite(motor_pin, motor_state.speed);

    if (motor_state.direction == motor_forward) {
        return forward_code;
    } else if (motor_state.direction == motor_reverse) {
        return reverse_code;
    } else {
        return 0;
    }
}

void Tank::_control_motor(struct motor_state & state, const motor_direction direction, const uint8_t speed)
{
    state.requested_direction = direction;
    state.speed = speed;

    if (direction == motor_stop) {
        state.direction = motor_stop;
        state.speed = 0;
    } else if (state.requested_direction != state.direction && !state.direction_change_requested) {
        state.direction_change_requested = true;
        state.direction_change_request_millis = millis();
    }
}

void Tank::_write_motor_control_code(const unsigned char & control_code)
{
    digitalWrite(SHIFT_CLEAR_PIN, LOW);
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, control_code);
    digitalWrite(SHIFT_CLEAR_PIN, HIGH);
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
