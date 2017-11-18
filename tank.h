#ifndef tank_h
#define tank_h

#include "Arduino.h"

#define TURRET_GEAR_RATIO 24000
#define TURRET_CALIBRATION_DELAY_MILLIS 2000

/* Interupt routines and variables cannot be members of a class
So these are declared outside of Tank */
extern volatile long _turret_encoder_count;
extern volatile bool _turret_has_been_calibrated;
extern volatile unsigned long _last_turret_calibration_millis;
extern short _turret_direction;

void turret_calibration_interrupt();
void turret_encoder_interrupt();

class Tank {
    public:
        Tank(int turret_motor_left_pin, int turret_motor_right_pin, int turret_encoder_pin, int turret_calibration_pin);

        void turret_stop();
        void turret_left();
        void turret_right();
        int turret_position();
        short turret_direction();
        bool turret_has_been_calibrated();

        // debugging
        int turret_encoder_count();
    private:
        short _turret_motor_left_pin;
        short _turret_motor_right_pin;
        short _turret_encoder_pin;
        short _turret_calibration_pin;

        unsigned long _last_output_millis;
        unsigned long _last_turret_calibration_millis;
};

#endif
