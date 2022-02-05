#ifndef tank_h
#define tank_h

#define IRMP_ENABLE_PIN_CHANGE_INTERRUPT
#define IRMP_INPUT_PIN  3
#define IRMP_SUPPORT_NEC_PROTOCOL 1
#define IRMP_USE_COMPLETE_CALLBACK 1

#include "constants.h"
#include "ir_codes.h"
#include "pins.h"
#include "tank.h"
#include "tank_led.h"

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <PVision.h>

#define DEBUG_OUTPUT 1

/* begin global variables */
// Interupt routines and variables cannot be members of a class
// So these are declared outside of Tank
extern volatile bool __turret_calibration_interrupt_flag;
extern volatile bool __turret_encoder_interrupt_flag;
extern volatile byte __last_ir_code;
/* end global variables */

void turret_calibration_interrupt();
void turret_encoder_interrupt();

enum tank_mode {
    mode_fight,
    mode_debug_drive,
    mode_debug_turret
};

enum tank_direction {
    forward,
    reverse,
    left,
    right,
    stopped
};

enum motor_direction {
    motor_forward,
    motor_reverse,
    motor_stop
};

struct motor_state {
    motor_direction direction;
    motor_direction requested_direction;
    uint8_t speed;
    bool direction_change_requested;
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
        Tank();

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

        byte last_ir_code();

    private:
        //void _process_ir_code(const unsigned long & ir_code);

        void _update_motors();
        uint8_t _update_motor(const uint8_t forward_code, const uint8_t reverse_code, const uint8_t motor_pin, struct motor_state & state);
        void _control_motor(struct motor_state & state, const motor_direction direction, const uint8_t speed = MOTOR_DEFAULT_SPEED);
        unsigned char _create_motor_control_code();
        void _write_motor_control_code(const unsigned char & control_code);
        void _shift_bit(bool bit);
        void _update_motor_debug_leds();

        void _process_interrupts();
        void _process_turret_calibration_interrupt();
        void _process_turret_encoder_interrupt();

        TankLED _tank_led;

        tank_mode _tank_mode;
        tank_direction _tank_direction;

        struct motor_control_mapping _motor_control_mapping = {32, 16, 8, 4, 2, 1};
        char _motor_control_code = 0;
        struct motor_state _left_motor_state;
        struct motor_state _right_motor_state;
        struct motor_state _turret_motor_state;

        bool          _motors_enabled = false;
        bool          _turret_calibration_requested = false;
        bool          _turret_has_been_calibrated = false;
        long          _turret_encoder_count = 0;
        short         _turret_direction = 0;
        unsigned long _last_turret_calibration_millis = 0;
};

#endif
