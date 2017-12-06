#ifndef tank_h
#define tank_h

#include "Arduino.h"

// delay between motor changing directions. to reduce strain on motors.
#define MOTOR_CHANGE_DIRECTION_DELAY_MILLIS 100
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
// variables related to turret
extern volatile long _turret_encoder_count;
extern volatile bool _turret_has_been_calibrated;
extern volatile unsigned long _last_turret_calibration_millis;
extern short _turret_direction;

// variables related to sonar
extern volatile unsigned int _sonar_front_distance;
extern volatile short _sonar_front_state;
extern volatile unsigned long _sonar_front_timer;
extern volatile unsigned int _sonar_rear_distance;
extern volatile short _sonar_rear_state;
extern volatile unsigned long _sonar_rear_timer;
extern volatile unsigned short _last_front_distances[];
extern volatile unsigned short _last_rear_distances[];
/* end global variables */

void turret_calibration_interrupt();
void turret_encoder_interrupt();

void front_sonar_interrupt();
void rear_sonar_interrupt();

class Tank {
    public:
        Tank(
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
        );

        void do_loop();
        void enable_motors();

        void drive(int drive_left_value, int drive_right_value);
        void drive_forward();
        void drive_reverse();
        void drive_stop();
        void drive_turn_left();
        void drive_turn_right();
        void drive_left_track(int drive_value);
        void drive_right_track(int drive_value);

        void turret_stop();
        void turret_left();
        void turret_right();
        const int turret_position();
        const short turret_direction();
        const bool turret_has_been_calibrated();

        void set_led_1(unsigned int led_on_delay, unsigned int led_off_delay);
        void set_led_2(unsigned int led_on_delay, unsigned int led_off_delay);

        const int front_distance();
        const int rear_distance();
    private:
        void _update_leds();
        void _update_led(unsigned short led_pin);
        void _do_sonar(unsigned short sonar_pin, volatile unsigned long &sonar_timer, volatile short &sonar_state, void (&interrupt)());
        void _do_motors();
        void _control_motor(
            const unsigned short &motor_pin_1,
            const unsigned short &motor_pin_2,
            const int &motor_requested_value,
            short &motor_direction,
            bool &motor_is_changing_direction,
            unsigned long &last_change_millis
        );

        // pin values, set by the constructor
        unsigned short _motor_enable_pin;
        unsigned short _drive_left_pin_1;
        unsigned short _drive_left_pin_2;
        unsigned short _drive_right_pin_1;
        unsigned short _drive_right_pin_2;
        unsigned short _turret_motor_pin_1;
        unsigned short _turret_motor_pin_2;
        unsigned short _turret_encoder_pin;
        unsigned short _turret_calibration_pin;
        unsigned short _led_pin_1;
        unsigned short _led_pin_2;
        unsigned short _sonar_pin_front;
        unsigned short _sonar_pin_rear;

        // values for control of left motor
        // _left_motor is the desired direction and speed of the motor, as requested by the user.
        // should be between -255 and 255, with negative values being reverse, 0 stopped, and postitive values being forward
        short _left_motor_requested_value;
        short _left_motor_direction; // -1 reverse, 0 stopped, 1 forward
        bool _left_motor_is_changing_direction;
        unsigned long _left_motor_last_direction_change_millis; // the time elapsed since the motor drive pins were written to

        // same as above but for the right motor
        short _right_motor_requested_value;
        short _right_motor_direction;
        bool _right_motor_is_changing_direction;
        unsigned long _right_motor_last_direction_change_millis;

        unsigned int _led_1_on_delay;
        unsigned int _led_1_off_delay;
        unsigned short _led_1_state;
        unsigned long _led_1_change_millis;
        unsigned int _led_2_on_delay;
        unsigned int _led_2_off_delay;
        unsigned short _led_2_state;
        unsigned long _led_2_change_millis;

        unsigned long _last_turret_calibration_millis;
};

#endif
