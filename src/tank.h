#ifndef tank_h
#define tank_h

// it's important to define these constants before including TinyIRReceiver.hpp (which is imported in tank.cpp)
#define IR_RECEIVE_PIN IR_RX_PIN
#define USE_CALLBACK_FOR_TINY_RECEIVER

#include "ir_codes.h"
#include "pins.h"
#include "tank_constants.h"
#include "tank_led.h"

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <PVision.h>
#include <Wire.h>

typedef void (*CallbackFunction) ();
typedef void (*CallbackFunctionWithInt) (uint16_t);

// TODO: are these definitions necessary?
/* begin global routines */
void _turret_encoder_interrupt();
void _turret_calibration_interrupt();
void _bump_0_interrupt();
/* end global routines */

//bool game_mode_active;

enum MotorDirection {
  motor_stop,
  motor_forward,
  motor_reverse
};

enum TurretDirection {
  left,
  right
};

struct MotorStatus {
  MotorDirection direction;
  MotorDirection requested_direction;
  MotorDirection last_direction;
  bool direction_change_requested;
  unsigned long direction_change_request_millis;
};

struct BumpStatus {
  uint32_t bump_front_millis = 0;
  uint32_t bump_rear_millis = 0;
  uint8_t bump_front = RISING;
  uint8_t bump_rear = 0;
};

// TODO: use this?
struct IRStatus {
  uint16_t last_command = 0;
  CallbackFunctionWithInt ir_command_callback = NULL;
};

struct TurretStatus {
  bool calibrated = false;
  int16_t encoder_count = 0;
  bool has_target = false;
  int16_t target_encoder_count = 0;
  TurretDirection target_direction;
  CallbackFunction target_callback;
};

// TankStatus is a data structure intended to be returned to the client. It should contain
// all the information that the client will care about.
struct TankStatus {
  uint8_t hit_count = 0;
  bool bump_front = false;
  bool bump_rear = false;
  bool drive_target_distance_reached = false;
  bool drive_target_degrees_reached = false;
  int16_t wheel_encoder_count_left = 0;
  int16_t wheel_encoder_count_right = 0;
  // drive_target_distance has a sentinel value of 0, meaning that there is no drive-target
  int16_t drive_target_distance = 0;
  // drive_target_degrees has a sentinel value of -1, meaning that there is no drive-target
  // possible TODO: both drive_target_distance and drive_target_degrees could have same sentinel value of 0, if i change degree representation to be 1-360, instead of 0-359
  int16_t drive_target_degrees = -1;
};

class Tank
{
  public:
    Tank();

    /// @brief Initializes the tank by setting input/output pins, initializing variables, etc. MUST be called before anything else is done within the Tank library.
    /// This should probably be called sometime during your setup() function.
    void initialize();

    /// @brief Waits for OK signal, calibrates the turret, and then waits for OK signal again.
    /// The idea is that, for any game-playing robot, to call this at the end of your setup() function.
    /// Once setup_routine() returns, the turret will have been calibrated and the tank will have been given the go-ahead.
    /// It isn't necessary to use this during development. If you just need the turret to be calibrated, you can call
    /// turret_calibrate instead.
    void setup_routine();

    /// @brief Updates the tank. Must be called continuously.
    /// This function must be called in order for the tank to do anything and for it to update its own state.
    /// This function should be called from your own bot's loop() function.
    void loop();

    void set_bump_front_callback(CallbackFunction);
    void set_bump_rear_callback(CallbackFunction);
    // Note that set_ir_command_callback is useful for development purposes but you should not have to use this function for normal gameplay
    void set_ir_command_callback(CallbackFunctionWithInt);

    void drive_forward(const uint8_t speed = MOTOR_DEFAULT_SPEED);
    // TODO: could consider to make whether tank stops at target an optional flag
    void drive_forward_target(const int16_t target_distance, CallbackFunction target_callback, const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_reverse(const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_reverse_target(const int16_t target_distance, CallbackFunction target_callback, const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_turn_left(const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_turn_right(const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_turn_degrees(int16_t degrees, CallbackFunction target_callback = NULL, const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_turn_left_degrees(int16_t degrees, CallbackFunction target_callback, const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_turn_right_degrees(int16_t degrees, CallbackFunction target_callback, const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void drive_stop();

    void turret_calibrate();
    void turret_left();
    void turret_right();
    void turret_left_degrees(const uint16_t degrees);
    void turret_right_degrees(const uint16_t degrees);
    void turret_left_degrees(const uint16_t degrees, CallbackFunction target_callback);
    void turret_right_degrees(const uint16_t degrees, CallbackFunction target_callback);
    void turret_set_degrees(const uint16_t target_degrees);
    void turret_set_degrees(const uint16_t target_degrees, CallbackFunction target_callback);
    void turret_stop();
    const int16_t turret_get_degrees();
    // TODO: we might end up using this?
    const bool turret_has_been_calibrated();

    TankStatus get_status();

    // TODO: maybe I move this to a utility file?
    const int16_t normalize_angle(const int16_t degrees);

    MiniLed _tank_led;

  private:
    void _process_interrupt_flags(unsigned long current_millis);
    void _process_encoder_flags(unsigned long current_millis);
    void _process_bump_flags(unsigned long current_millis);
    void _process_ir_flags();

    void _drive(const MotorDirection left_direction, const MotorDirection right_direction, const uint8_t speed = MOTOR_DEFAULT_SPEED);
    void _drive_stop();
    void _check_drive_targets();
    void _check_drive_distance_target();
    void _check_drive_turn_target();
    void _check_turret_target();
    void _turret_target_reached();

    void _turret_left();
    void _turret_right();
    void _update_motors();
    uint8_t _determine_motor_control_code(const uint8_t forward_code, const uint8_t reverse_code, MotorStatus & status);
    void _control_motor(MotorStatus & status, const MotorDirection direction);
    unsigned char _create_motor_control_code();
    void _write_motor_control_code(const unsigned char & control_code);
    unsigned char _current_motor_control_code;

    bool game_mode_active;

    uint8_t _requested_speed;
    uint8_t _current_speed;

    MotorStatus _left_motor_status;
    MotorStatus _right_motor_status;
    MotorStatus _turret_motor_status;

    struct BumpStatus _bump_status;
    struct IRStatus _ir_status;
    struct TurretStatus _turret_status;
    struct TankStatus _tank_status;

    CallbackFunction _bump_front_callback = NULL;
    CallbackFunction _bump_rear_callback = NULL;
    CallbackFunction _drive_target_callback = NULL;
};

#endif
