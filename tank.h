#ifndef tank_h
#define tank_h

#include <Arduino.h>
#include <IRremote.h>

#include "tank_led.h"

// delay between motor changing directions. to reduce strain on motors.
#define MOTOR_CHANGE_DIRECTION_DELAY_MILLIS 1000
#define MOTOR_SPEED_CHANGE_DELAY_MILLIS 10
#define MOTOR_DEFAULT_SPEED 200
// gear ratio determined through expermentation. if an incorrect turret position is being reported, this value could be responsible.
#define TURRET_GEAR_RATIO 24000
// this calibration delay is so that the calibration is not triggered multiple times. probably safe to change.
#define TURRET_CALIBRATION_DELAY_MILLIS 2000

// delay between sonar pulses, front and rear. safe to change but don't make it too short or too long
// too short would mean that the sonar does not have enough time to return a reading before it is sending another pulse
// too long would mean that the tank is running into objects before they can be registered
// note that this is in microseconds, not milliseconds
#define SONAR_DELAY_MICROS 50000

// this is related to the speed of sound so should not need to be changed. unless you're operating well above sea level. ha. ha.
#define SONAR_FACTOR 29.1
#define SONAR_DISTANCE_WARNING 40
#define SONAR_DISTANCE_EMERGENCY 20

/* begin global variables */
// Interupt routines and variables cannot be members of a class
// So these are declared outside of Tank
extern volatile bool __turret_calibration_interrupt_flag;
extern volatile bool __turret_encoder_interrupt_flag;
extern volatile bool __sonar_front_interrupt_flag;
extern volatile bool __sonar_rear_interrupt_flag;
/* end global variables */

void turret_calibration_interrupt();
void turret_encoder_interrupt();

void front_sonar_interrupt();
void rear_sonar_interrupt();

enum tank_mode {
    mode_fight,
    mode_debug_drive,
    mode_debug_turret,
    mode_debug_sonar
};

enum motor_direction {
    motor_forward,
    motor_reverse,
    motor_stop
};

enum distance_warning {
    distance_warning_none,
    distance_warning_warning,
    distance_warning_emergency
};

struct sonar_state {
    uint8_t pin;
    uint8_t state;
    unsigned long timer;
    uint8_t distance;
    uint8_t debug_led_index;
    distance_warning warning;
};

struct motor_state {
    motor_direction direction;
    motor_direction requested_direction;
    uint8_t speed;
    uint8_t requested_speed;
    uint8_t last_speed;
    unsigned long speed_change_millis;
    unsigned long direction_change_request_millis;
};

struct motor_control_mapping {
    uint8_t left_track_reverse;
    uint8_t left_track_forward;
    uint8_t right_track_reverse;
    uint8_t right_track_forward;
    uint8_t turret_left;
    uint8_t turret_right;
};

class Tank
{
    public:
        Tank(
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
        );

        void setup();
        void loop();
        void enable_motors(bool enable);

        void drive(const motor_direction left_direction, const motor_direction right_direction, const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_forward(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_reverse(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_turn_left(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_turn_right(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_stop();

        void turret_left(const uint8_t speed = MOTOR_DEFAULT_SPEED);
        void turret_right(const uint8_t speed = MOTOR_DEFAULT_SPEED);
        void turret_stop();
        const uint16_t turret_position();
        const short turret_direction();
        const bool turret_has_been_calibrated();

        const uint8_t front_distance();
        const uint8_t rear_distance();

    private:
        void _update_sonar(struct sonar_state & state, void (&interrupt)());

        void _process_ir_code(unsigned long & ir_code);

        void _update_motors();
        void _update_motor(const uint8_t motor_pin, struct motor_state & state);
        void _control_motor(struct motor_state & state, const motor_direction direction, const uint8_t speed = MOTOR_DEFAULT_SPEED);
        unsigned char _create_motor_control_code();
        void _write_motor_control_code(const unsigned char & control_code);
        void _shift_bit(bool bit);
        void _update_motor_debug_leds();

        void _process_interrupts();
        void _process_turret_calibration_interrupt();
        void _process_turret_encoder_interrupt();
        void _process_sonar_interrupt(struct sonar_state & sonar);

        IRrecv *irrecv;
        decode_results results;

        TankLED _tank_led;

        tank_mode _tank_mode;

        uint8_t _ir_receiver_pin;
        uint8_t _motor_enable_pin;
        uint8_t _left_motor_pwm_pin;
        uint8_t _right_motor_pwm_pin;
        uint8_t _turret_motor_pwm_pin;
        uint8_t _shift_clear_pin;
        uint8_t _shift_clock_pin;
        uint8_t _shift_data_pin;
        uint8_t _turret_encoder_pin;
        uint8_t _turret_calibration_pin;
        uint8_t _led_pin_1;
        uint8_t _led_pin_2;
        uint8_t _led_pin_3;
        uint8_t _led_pin_4;

        struct sonar_state _front_sonar_state;
        struct sonar_state _rear_sonar_state;

        struct motor_control_mapping _motor_control_mapping = {32, 16, 8, 4, 2, 1};
        char _motor_control_code = 0;
        struct motor_state _left_motor_state = {
            .direction = motor_stop,
            .requested_direction = motor_stop,
            .speed = 0,
            .requested_speed = 0,
            .last_speed = 0,
            .speed_change_millis = 0,
            .direction_change_request_millis = 0
        };
        struct motor_state _right_motor_state = {
            .direction = motor_stop,
            .requested_direction = motor_stop,
            .speed = 0,
            .requested_speed = 0,
            .last_speed = 0,
            .speed_change_millis = 0,
            .direction_change_request_millis = 0
        };
        struct motor_state _turret_motor_state = {
            .direction = motor_stop,
            .requested_direction = motor_stop,
            .speed = 0,
            .requested_speed = 0,
            .last_speed = 0,
            .speed_change_millis = 0,
            .direction_change_request_millis = 0
        };

        bool           _motors_enabled = false;
        long           _turret_encoder_count = 0;
        bool           _turret_has_been_calibrated = false;
        unsigned long  _last_turret_calibration_millis = 0;
        short          _turret_direction = 0;
};

#endif
