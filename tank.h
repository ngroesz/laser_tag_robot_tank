#ifndef tank_h
#define tank_h

#include "Arduino.h"

// gear ratio determined through expermentation. if an incorrect turret position is being reported, this value could be responsible.
#define TURRET_GEAR_RATIO 24000
// this calibration delay is so that the calibration is not triggered multiple times. probably safe to change.
#define TURRET_CALIBRATION_DELAY_MILLIS 2000

// delay between sonar pulses, front and rear. safe to change but don't make it too short or too long
// too short would mean that the sonar does not have enough time to return a reading before it is sending another pulse
// too long would mean that the tank is running into objects before they can be registered
// note that this is in microseconds, not milliseconds
#define SONAR_DELAY_MICROS 100000

// this is related to the speed of sound so should not need to be changed. unless you're operating well above sea level.
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
            int turret_motor_left_pin,
            int turret_motor_right_pin,
            int turret_encoder_pin,
            int turret_calibration_pin,
            int led_pin_1,
            int led_pin_2,
            int sonar_pin_front,
            int sonar_pin_rear
        );

        void do_loop();
        void turret_stop();
        void turret_left();
        void turret_right();
        int turret_position();
        short turret_direction();
        bool turret_has_been_calibrated();

        void set_led_1(unsigned int led_on_delay, unsigned int led_off_delay);
        void set_led_2(unsigned int led_on_delay, unsigned int led_off_delay);

        int front_distance();
        int rear_distance();
    private:
        void _update_leds();
        void _update_led(unsigned short led_pin);
        void _do_sonar(unsigned short sonar_pin);

        unsigned short _turret_motor_left_pin;
        unsigned short _turret_motor_right_pin;
        unsigned short _turret_encoder_pin;
        unsigned short _turret_calibration_pin;
        unsigned short _led_pin_1;
        unsigned short _led_pin_2;
        unsigned short _sonar_pin_front;
        unsigned short _sonar_pin_rear;

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
