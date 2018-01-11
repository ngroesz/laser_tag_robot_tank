#ifndef tank_h
#define tank_h

#include "IRremote.h"
#include "Arduino.h"
#include "tank_led.h"

// delay between motor changing directions. to reduce strain on motors.
#define MOTOR_CHANGE_DIRECTION_DELAY_MILLIS 100
#define MOTOR_DEFAULT_SPEED 255
// gear ratio determined through expermentation. if an incorrect turret position is being reported, this value could be responsible.
#define TURRET_GEAR_RATIO 24000
// this calibration delay is so that the calibration is not triggered multiple times. probably safe to change.
#define TURRET_CALIBRATION_DELAY_MILLIS 2000

// delay between sonar pulses, front and rear. safe to change but don't make it too short or too long
// too short would mean that the sonar does not have enough time to return a reading before it is sending another pulse
// too long would mean that the tank is running into objects before they can be registered
// note that this is in microseconds, not milliseconds
#define SONAR_DELAY_MICROS 100000

// this is related to the speed of sound so should not need to be changed. unless you're operating well above sea level. ha. ha.
#define SONAR_FACTOR 29.1

/* begin global variables */
// Interupt routines and variables cannot be members of a class
// So these are declared outside of Tank
extern volatile long           __turret_encoder_count;
extern volatile bool           __turret_has_been_calibrated;
extern volatile unsigned long  __last_turret_calibration_millis;
extern short                   __turret_direction;
extern volatile unsigned int   __sonar_front_distance;
extern volatile short          __sonar_front_state;
extern volatile unsigned long  __sonar_front_timer;
extern volatile unsigned int   __sonar_rear_distance;
extern volatile short          __sonar_rear_state;
extern volatile unsigned long  __sonar_rear_timer;
extern volatile unsigned short __last_front_distances[];
extern volatile unsigned short __last_rear_distances[];
extern volatile boolean        __ir_data_receiving;
/* end global variables */

void turret_calibration_interrupt();
void turret_encoder_interrupt();

void front_sonar_interrupt();
void rear_sonar_interrupt();

enum tank_mode {
    fight,
    drive,
    turret
};

enum motor_direction {
    forward,
    reverse,
    stop
};

struct motor_state {
    motor_direction direction;
    motor_direction requested_direction;
    uint8_t speed;
    unsigned long direction_change_request_millis;
};

struct motor_control_mapping {
    unsigned int turret_left;
    unsigned int turret_right;
    unsigned int left_track_reverse;
    unsigned int left_track_forward;
    unsigned int right_track_reverse;
    unsigned int right_track_forward;
};

class Tank
{
    public:
        Tank(
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
        );

        void setup();
        void loop();
        void enable_motors();

        void drive(const motor_direction left_direction, const motor_direction right_direction, const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_forward(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_reverse(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_turn_left(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void drive_turn_right(const uint8_t left_speed = MOTOR_DEFAULT_SPEED, const uint8_t right_speed = MOTOR_DEFAULT_SPEED);
        void control_left_motor(const motor_direction direction, const uint8_t speed = MOTOR_DEFAULT_SPEED);
        void control_right_motor(const motor_direction direction, const uint8_t speed = MOTOR_DEFAULT_SPEED);
        void drive_stop();

        void turret_left(const uint8_t speed = MOTOR_DEFAULT_SPEED);
        void turret_right(const uint8_t speed = MOTOR_DEFAULT_SPEED);
        void turret_stop();
        void control_turret_motor(const motor_direction direction, const uint8_t speed = MOTOR_DEFAULT_SPEED);
        const int turret_position();
        const short turret_direction();
        const bool turret_has_been_calibrated();

        const int front_distance();
        const int rear_distance();

    private:
        void _initialize_motors();
        void _initalize_turret();
        void _initialize_ir();
        void _update_sonar(unsigned short sonar_pin, volatile unsigned long &sonar_timer, volatile short &sonar_state, void (&interrupt)());
        void _update_ir();
        void _process_ir_code(unsigned long & ir_code);
        void _update_motors();
        void _update_motor_directions();
        void _update_motor_direction(struct motor_state & state);
        unsigned char _create_motor_control_code();
        void _write_motor_control_code(const unsigned char & control_code);

        TankLED _tank_led;

        tank_mode _tank_mode;

        uint8_t _ir_receiver_pin;
        uint8_t _motor_enable_pin;
        uint8_t _shift_clear_pin;
        uint8_t _shift_clock_pin;
        uint8_t _shift_data_pin;
        uint8_t _turret_encoder_pin;
        uint8_t _turret_calibration_pin;
        uint8_t _sonar_pin_front;
        uint8_t _sonar_pin_rear;
        uint8_t _led_pin_1;
        uint8_t _led_pin_2;
        uint8_t _led_pin_3;
        uint8_t _led_pin_4;

        struct motor_control_mapping _motor_control_mapping = {32, 16, 8, 4, 2, 1};
        char _motor_control_code;
        struct motor_state _left_motor_state, _right_motor_state, _turret_motor_state;

        unsigned long _last_turret_calibration_millis;

        boolean _ir_code_is_queued, _ir_is_disabled;
        unsigned long _ir_code_that_is_queued, _ir_disabled_millis;
};

#endif
