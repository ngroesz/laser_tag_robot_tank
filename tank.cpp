#include "Arduino.h"
#include "PinChangeInterrupt.h"
#include "IRremote.h"
#include <Wire.h>
#include <PVision.h>

#include "ir_codes.h"
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
    int ir_receiver_pin,
    int motor_enable_pin,
    int turret_encoder_pin,
    int turret_calibration_pin,
    int sonar_pin_front,
    int sonar_pin_rear,
    int led_pin_1,
    int led_pin_2,
    int led_pin_3,
    int led_pin_4
)
{
    _ir_receiver_pin        = ir_receiver_pin;
    _motor_enable_pin       = motor_enable_pin;
    _turret_encoder_pin     = turret_encoder_pin;
    _turret_calibration_pin = turret_calibration_pin;
    _sonar_pin_front        = sonar_pin_front;
    _sonar_pin_rear         = sonar_pin_rear;
    _led_pin_1              = led_pin_1;
    _led_pin_2              = led_pin_2;
    _led_pin_3              = led_pin_3;
    _led_pin_4              = led_pin_4;

    _ir_disabled_millis     = 0;
    __ir_data_receiving      = false;
    _ir_is_disabled         = false;
}

void Tank::setup()
{
    _initialize_motors();
    _initalize_turret();
    _initialize_leds();
    _initialize_ir();
}

void Tank::_initialize_motors()
{
    _left_motor_requested_value               = 0;
    _left_motor_direction                     = 0;
    _left_motor_is_changing_direction         = false;
    _left_motor_last_direction_change_millis  = 0;

    _right_motor_requested_value              = 0;
    _right_motor_direction                    = 0;
    _right_motor_is_changing_direction        = false;
    _right_motor_last_direction_change_millis = 0;

    pinMode(_motor_enable_pin, OUTPUT);
}

void Tank::_initalize_turret()
{
    pinMode(_turret_encoder_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_encoder_pin), turret_encoder_interrupt, CHANGE);

    pinMode(_turret_calibration_pin, INPUT_PULLUP);
    //attachPCINT(digitalPinToPCINT(_turret_calibration_pin), turret_calibration_interrupt, RISING);
}

void Tank::_initialize_leds()
{
    pinMode(_led_pin_1, OUTPUT);
    pinMode(_led_pin_2, OUTPUT);
    pinMode(_led_pin_3, OUTPUT);
    pinMode(_led_pin_4, OUTPUT);
    digitalWrite(_led_pin_1, HIGH);
    digitalWrite(_led_pin_2, HIGH);
    digitalWrite(_led_pin_3, HIGH);
    digitalWrite(_led_pin_4, HIGH);
    set_blink_mode(0, -1, 0, -1, 0, -1, 0, -1);
}

void Tank::_initialize_ir()
{
    pinMode(_ir_receiver_pin, INPUT);
    attachPCINT(digitalPinToPCINT(_ir_receiver_pin), ir_receiver_interrupt, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(_ir_receiver_pin), ir_receiver_interrupt, LOW);
}

void Tank::loop()
{
    _check_for_ir_data();
    //_do_sonar(_sonar_pin_rear, _sonar_rear_timer, _sonar_rear_state, sonar_rear_interrupt);
//    _do_motors();
    _update_leds();
    //if (ir_receiver->decode(&results)) {
    //    Serial.print("received:");
    //    Serial.println(results.value);
    //    if (results.value == IR_CODE_A) {
    //        Serial.println("A");
    //        set_blink_mode(-1, 0, 0, -1, -1, 0, -1, 0);
    //    } else if (results.value == IR_CODE_B) {
    //        Serial.println("B");
    //        set_blink_mode(0, -1, -1, 0, 0, -1, 0, -1);
    //    } 
    //    ir_receiver->resume();
    //} 
}

void Tank::enable_motors()
{
    digitalWrite(_motor_enable_pin, HIGH);
}

//void Tank::_do_motors()
//{
//    _control_motor(
//        _drive_left_pin_1,
//        _drive_left_pin_2,
//        _left_motor_requested_value,
//        _left_motor_direction,
//        _left_motor_is_changing_direction,
//        _left_motor_last_direction_change_millis
//    );
//
//    _control_motor(
//        _drive_right_pin_1,
//        _drive_right_pin_2,
//        _right_motor_requested_value,
//        _right_motor_direction,
//        _right_motor_is_changing_direction,
//        _right_motor_last_direction_change_millis
//    );
//}
//
//void Tank::_control_motor(
//    const unsigned short &motor_pin_1,
//    const unsigned short &motor_pin_2,
//    const int &motor_requested_value,
//    short &motor_direction,
//    bool &motor_is_changing_direction,
//    unsigned long &last_motor_direction_change_millis
//)
//{
//    unsigned long current_millis = millis();
//
//    // check if motor is changing direction and if so, set the changing direction flag and stop the motor, and start the timer
//    if(!motor_is_changing_direction && ((motor_requested_value >= 0 && motor_direction == -1) || (motor_requested_value <= 0 && motor_direction == 1))) {
//        motor_is_changing_direction = true;
//        digitalWrite(motor_pin_1, LOW);
//        digitalWrite(motor_pin_2, LOW);
//        last_motor_direction_change_millis = current_millis;
//    }
//
//    if (motor_is_changing_direction && current_millis <= last_motor_direction_change_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
//        return;
//    }
//
//    motor_is_changing_direction = false;
//
//    if (current_millis >= last_debug_output_millis + 1000) {
//        Serial.print("motor request direction: ");
//        Serial.println(motor_requested_value);
//    }
//
//    if(motor_requested_value > 0) {
//        digitalWrite(motor_pin_1, HIGH);
//        digitalWrite(motor_pin_2, LOW);
//        motor_direction = 1;
//    } else if(motor_requested_value < 0) {
//        digitalWrite(motor_pin_1, LOW);
//        digitalWrite(motor_pin_2, HIGH);
//        motor_direction = -1;
//    } else {
//        digitalWrite(motor_pin_1, LOW);
//        digitalWrite(motor_pin_2, LOW);
//        motor_direction = 0;
//    }
//
//}

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

//void Tank::turret_stop()
//{
//    analogWrite(_turret_motor_pin_1, 0);
//    analogWrite(_turret_motor_pin_2, 0);
//}
//
//void Tank::turret_left()
//{
//    if (_turret_direction != -1) {
//        turret_stop();
//    }
//    analogWrite(_turret_motor_pin_1, 255);
//    analogWrite(_turret_motor_pin_2, 0);
//    _turret_direction = -1;
//}
//
//void Tank::turret_right()
//{
//    if (_turret_direction != 1) {
//        turret_stop();
//    }
//    analogWrite(_turret_motor_pin_1, 0);
//    analogWrite(_turret_motor_pin_2, 255);
//    _turret_direction = 1;
//}

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

void Tank::set_blink_mode
(
    short led1_on_delay,
    short led1_off_delay,
    short led2_on_delay,
    short led2_off_delay,
    short led3_on_delay,
    short led3_off_delay,
    short led4_on_delay,
    short led4_off_delay
)
{
    blink_mode.led1_on_delay  = led1_on_delay;
    blink_mode.led1_off_delay = led1_off_delay;
    blink_mode.led2_on_delay  = led2_on_delay;
    blink_mode.led2_off_delay = led2_off_delay;
    blink_mode.led3_on_delay  = led3_on_delay;
    blink_mode.led3_off_delay = led3_off_delay;
    blink_mode.led4_on_delay  = led4_on_delay;
    blink_mode.led4_off_delay = led4_off_delay;
}

void Tank::_update_led(int led_pin, unsigned long current_millis, short &on_delay, short &off_delay, boolean &state, unsigned long &last_change_millis)
{
    if (state == false && off_delay != -1 && last_change_millis + off_delay < current_millis) {
        // because the LEDs are wired switched-ground, this actually turns the LED on
        digitalWrite(led_pin, LOW);
        last_change_millis = current_millis;
        state = true;
    } else if(state == true && on_delay != -1 && last_change_millis + on_delay < current_millis) {
        // turn the LED off
        digitalWrite(led_pin, HIGH);
        last_change_millis = current_millis;
        state = false;
    }
}

void Tank::_update_leds()
{
    unsigned long current_millis = millis();
    _update_led(_led_pin_1, current_millis, blink_mode.led1_on_delay, blink_mode.led1_off_delay, led_timings.led1_state, led_timings.led1_last_change_millis);
    _update_led(_led_pin_2, current_millis, blink_mode.led2_on_delay, blink_mode.led2_off_delay, led_timings.led2_state, led_timings.led2_last_change_millis);
    _update_led(_led_pin_3, current_millis, blink_mode.led3_on_delay, blink_mode.led3_off_delay, led_timings.led3_state, led_timings.led3_last_change_millis);
    _update_led(_led_pin_4, current_millis, blink_mode.led4_on_delay, blink_mode.led4_off_delay, led_timings.led4_state, led_timings.led4_last_change_millis);
}

void Tank::_check_for_ir_data()
{
    unsigned long current_millis = millis();
    if (__ir_data_receiving && !_ir_is_disabled) {
        _ir_is_disabled = 1;
        noInterrupts();
        unsigned long result = read_ir_data(_ir_receiver_pin);
        _ir_disabled_millis = current_millis;
        __ir_data_receiving = false;
        interrupts();

        Serial.print("result: ");
        Serial.println(result);
        if (result == IR_CODE_A) {
            set_blink_mode(0, -1, 500, 500, 500, 500, 0, -1);
        } else if(result == IR_CODE_B) {
            set_blink_mode(500, 500, 500, 500, 500, 500, 500, 500);
        }
    }

    if (_ir_is_disabled && current_millis > _ir_disabled_millis + IR_DELAY_BETWEEN_TRANSMISSIONS_MILLIS) {
        _ir_is_disabled = 0;
    }
}
