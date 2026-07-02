#include "tank.h"

#include "TinyIRReceiver.hpp"

// initalize global variables
volatile bool _turret_encoder_interrupt_flag = false;
volatile bool _turret_calibration_interrupt_flag = false;
volatile uint8_t _bump_front_interrupt_flag = 0;
volatile uint8_t _bump_rear_interrupt_flag = 0;
volatile bool _left_wheel_encoder_interrupt_flag = false;
volatile bool _right_wheel_encoder_interrupt_flag = false;
volatile bool _ir_command_received = false;
volatile uint16_t _ir_command;

void _turret_encoder_interrupt() {
  _turret_encoder_interrupt_flag = true;
}

void _turret_calibration_interrupt() {
  _turret_calibration_interrupt_flag = true;
}

void _left_wheel_encoder_interrupt() {
  _left_wheel_encoder_interrupt_flag = true;
}

void _right_wheel_encoder_interrupt() {
  _right_wheel_encoder_interrupt_flag = true;
}

void _bump_front_interrupt() {
  _bump_front_interrupt_flag = getPinChangeInterruptTrigger(digitalPinToPCINT(BUMP_PIN_FRONT));
}

void _bump_rear_interrupt() {
  _bump_rear_interrupt_flag = getPinChangeInterruptTrigger(digitalPinToPCINT(BUMP_PIN_REAR));
}

void handleReceivedTinyIRData() {
  if (TinyIRReceiverData.Flags != IRDATA_FLAGS_IS_REPEAT && TinyIRReceiverData.Flags != IRDATA_FLAGS_PARITY_FAILED) {
    _ir_command_received = true;
    _ir_command = TinyIRReceiverData.Command;
  }
}

Tank::Tank()
{
  _current_motor_control_code = 0;
}

/**
* initialize - initializes Tank
*
* Does a bunch of stuff to initialize the tank. This needs to be called in
* the beginning of your program, probably from the setup() routine.
* The tank probably will not do anything useful until this function is called.
*/
void Tank::initialize()
{
#ifdef TANK_DEBUG_OUTPUT
  Serial.println(F("Tank initializing ..."));
#endif

  game_mode_active = false;

  // initialize LEDs
  uint8_t pins[] = {LED_PIN_1, LED_PIN_2, LED_PIN_3};
  _tank_led.setup(pins);
  _tank_led.on(0);
  delay(250);

  // initialize motors
  digitalWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(SHIFT_CLEAR_PIN, 0);
  digitalWrite(SHIFT_CLOCK_PIN, 0);
  digitalWrite(SHIFT_DATA_PIN, 0);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(SHIFT_CLEAR_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_DATA_PIN, OUTPUT);

  _left_motor_status = {
    .direction = motor_stop,
    .requested_direction = motor_stop,
    .last_direction = motor_stop,
    .direction_change_requested = false,
    .direction_change_request_millis = 0
  };
  _right_motor_status = {
    .direction = motor_stop,
    .requested_direction = motor_stop,
    .last_direction = motor_stop,
    .direction_change_requested = false,
    .direction_change_request_millis = 0
  };
  _turret_motor_status = {
    .direction = motor_stop,
    .requested_direction = motor_stop,
    .last_direction = motor_stop,
    .direction_change_requested = false,
    .direction_change_request_millis = 0
  };

  _tank_led.on(1);
  delay(250);

  // initialize bump detectors
  pinMode(BUMP_PIN_FRONT, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUMP_PIN_FRONT), _bump_front_interrupt, CHANGE);
  pinMode(BUMP_PIN_REAR, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUMP_PIN_REAR), _bump_rear_interrupt, CHANGE);

  // initalize wheel encoders
  pinMode(WHEEL_ENCODER_PIN_LEFT, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(WHEEL_ENCODER_PIN_LEFT), _left_wheel_encoder_interrupt, CHANGE);
  pinMode(WHEEL_ENCODER_PIN_RIGHT, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(WHEEL_ENCODER_PIN_RIGHT), _right_wheel_encoder_interrupt, CHANGE);

  // initialize turret encoder
  pinMode(TURRET_ENCODER_PIN, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(TURRET_ENCODER_PIN), _turret_encoder_interrupt, CHANGE);
  pinMode(TURRET_CALIBRATION_PIN, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(TURRET_CALIBRATION_PIN), _turret_calibration_interrupt, FALLING);

  // TODO: I think that I should create an InternalTankStatus data structure
  _bump_status.bump_front = 0;
  _bump_status.bump_front_millis = 0;
  _bump_status.bump_rear = 0;
  _bump_status.bump_rear_millis = 0;

  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println(F("could not initialize IR"));
  }

  _tank_led.on(2);

  #ifdef TANK_DEBUG_OUTPUT
    Serial.println(F("... tank initialized."));
  #endif
}

void Tank::setup_routine() {
  _tank_led.all_off();
  _tank_led.set_blinks(0, (const uint16_t[]){500, 500}, 2);

  do {
    loop();
  } while(_ir_status.last_command != IR_CODE_OK);
  _ir_status.last_command = 0;

  _tank_led.set_blinks(1, (const uint16_t[]){500, 500}, 2);

  // turret_calibrate will not return until turret is calibrated
  turret_calibrate();

  _tank_led.set_blinks(2, (const uint16_t[]){500, 500}, 2);

  do {
    loop();
  } while(_ir_status.last_command != IR_CODE_OK);
  _ir_status.last_command = 0;

  _tank_led.all_off();
  game_mode_active = true;
}

void Tank::loop() {
  unsigned long current_millis = millis();

  _update_motors();
  _tank_led.loop();
  _process_interrupt_flags(millis());
}

TankStatus Tank::get_status() {
  return _tank_status;
}

void Tank::set_bump_front_callback(CallbackFunction callback) {
  _bump_front_callback = callback;
}

void Tank::set_bump_rear_callback(CallbackFunction callback) {
  _bump_rear_callback = callback;
}

void Tank::set_ir_command_callback(CallbackFunctionWithInt callback) {
  _ir_status.ir_command_callback = callback;
}

void Tank::drive_forward(const uint8_t speed) {
  _tank_status.drive_target_distance = 0;
  _tank_status.drive_target_degrees = 0;
  _drive(motor_forward, motor_forward, speed);
}

void Tank::drive_forward_target(const int16_t target_distance, CallbackFunction target_callback, const uint8_t speed) {
  _tank_status.wheel_encoder_count_left = 0;
  _tank_status.wheel_encoder_count_right = 0;
  _tank_status.drive_target_distance_reached = false;
  _tank_status.drive_target_distance = target_distance;
  _drive_target_callback = target_callback;
  _drive(motor_forward, motor_forward, speed);
}

void Tank::drive_reverse(const uint8_t speed) {
  _tank_status.drive_target_distance = 0;
  _tank_status.drive_target_degrees = 0;
  _drive(motor_reverse, motor_reverse, speed);
}

void Tank::drive_reverse_target(const int16_t target_distance, CallbackFunction target_callback, const uint8_t speed) {
  _tank_status.wheel_encoder_count_left = 0;
  _tank_status.wheel_encoder_count_right = 0;
  _tank_status.drive_target_distance_reached = false;
  _tank_status.drive_target_distance = -target_distance;
  _drive_target_callback = target_callback;
  _drive(motor_reverse, motor_reverse, speed);
}

void Tank::drive_turn_left(uint8_t speed) {
  _tank_status.drive_target_distance = 0;
  _tank_status.drive_target_degrees = 0;
  _drive(motor_reverse, motor_forward, speed);
}

void Tank::drive_turn_right(uint8_t speed) {
  _tank_status.drive_target_distance = 0;
  _tank_status.drive_target_degrees = 0;
  _drive(motor_forward, motor_reverse, speed);
}

void Tank::drive_turn_left_degrees(int8_t degrees, CallbackFunction target_callback, uint8_t speed) {
  _tank_status.drive_target_distance = 0;
  _tank_status.wheel_encoder_count_left = 0;
  _tank_status.wheel_encoder_count_right = 0;
  _tank_status.drive_target_degrees_reached = false;
  _tank_status.drive_target_degrees = -degrees % 360;
  _drive_target_callback = target_callback;
  _drive(motor_reverse, motor_forward, speed);
}

void Tank::drive_turn_right_degrees(int8_t degrees, CallbackFunction target_callback, uint8_t speed) {
  _tank_status.drive_target_distance = 0;
  _tank_status.wheel_encoder_count_left = 0;
  _tank_status.wheel_encoder_count_right = 0;
  _tank_status.drive_target_degrees_reached = false;
  _tank_status.drive_target_degrees = degrees % 360;
  _drive_target_callback = target_callback;
  _drive(motor_forward, motor_reverse, speed);
}

void Tank::drive_stop() {
  _tank_status.drive_target_distance = 0;
  _tank_status.drive_target_degrees = 0;
  _requested_speed = 0;
  // TODO: should this be changed to a _drive(motor_stop, motor_stop) call?
  _control_motor(_left_motor_status, motor_stop);
  _control_motor(_right_motor_status, motor_stop);
}

void Tank::turret_calibrate() {
#ifdef TANK_DEBUG_OUTPUT
  Serial.println(F("turret calibration"));
#endif
  _turret_status.calibrated = false;

  turret_left();
  while (!_turret_status.calibrated) {
    loop();
  }

  turret_stop();
#ifdef TANK_DEBUG_OUTPUT
  Serial.println(F("turret has been calibrated"));
#endif
}

void Tank::turret_left() {
  _turret_status.has_target = false;
  _turret_status.target_callback = NULL;
  _turret_left();
}

void Tank::turret_right() {
  _turret_status.has_target = false;
  _turret_status.target_callback = NULL;
  _turret_right();
}

void Tank::turret_left_degrees(const uint16_t degrees) {
  turret_left_degrees(degrees, NULL);
}

void Tank::turret_right_degrees(const uint16_t degrees) {
  turret_right_degrees(degrees, NULL);
}

void Tank::turret_left_degrees(const uint16_t degrees, CallbackFunction target_callback) {
  _turret_status.has_target = true;
  _turret_status.target_direction = left;
  _turret_status.target_encoder_count = _turret_status.encoder_count - round(degrees * TURRET_GEAR_RATIO);
  _turret_status.target_callback = target_callback;
  _turret_left();
}

void Tank::turret_right_degrees(const uint16_t degrees, CallbackFunction target_callback) {
  _turret_status.has_target = true;
  _turret_status.target_direction = right;
  _turret_status.target_encoder_count = _turret_status.encoder_count + round(degrees * TURRET_GEAR_RATIO);
  _turret_status.target_callback = target_callback;
  _turret_right();
}

void Tank::turret_set_degrees(const uint16_t target_degrees) {
  int16_t current_degrees = turret_get_degrees();

  // refuse to do anything if turret is not calibrated
  if (current_degrees == -1) {
    return;
  }

  _turret_status.has_target = true;
  int16_t clockwise_degrees = 360 - normalize_angle(current_degrees - target_degrees);
  int16_t counter_clockwise_degrees = normalize_angle(current_degrees - target_degrees);

  // if turning clockwise (right) gets us to target_degrees in 180 degrees or less, let's do that
  if (clockwise_degrees <= 180) {
    _turret_status.target_direction = right;
    _turret_status.target_encoder_count = _turret_status.encoder_count + round(clockwise_degrees * TURRET_GEAR_RATIO);
    _turret_right();
  // otherwise, we will turn counter-clockwise (left)
  } else {
    _turret_status.target_direction = left;
    _turret_status.target_encoder_count = _turret_status.encoder_count - round(counter_clockwise_degrees * TURRET_GEAR_RATIO);
    _turret_left();
  }
}

void Tank::turret_set_degrees(const uint16_t target_degrees, CallbackFunction target_callback) {
  _turret_status.target_callback = target_callback;
  turret_set_degrees(target_degrees);
}

void Tank::turret_stop() {
  _turret_status.has_target = false;
  _control_motor(_turret_motor_status, motor_stop);
}

const int16_t Tank::turret_get_degrees() {
  if (!_turret_status.calibrated) {
    return -1;
  }

  int16_t turret_position = round(_turret_status.encoder_count / TURRET_GEAR_RATIO);
  int16_t normalized_turret_position = normalize_angle(turret_position);

  return normalized_turret_position;
}

const bool Tank::turret_has_been_calibrated() {
  return _turret_status.calibrated;
}

const int16_t Tank::normalize_angle(const int16_t degrees) {
  return degrees < 0 ? (degrees + 1 % 360) + 360 - 1 : degrees % 360;
}
// end public functions

// begin private functions
void Tank::_turret_left() {
  _control_motor(_turret_motor_status, motor_reverse);
}

void Tank::_turret_right() {
  _control_motor(_turret_motor_status, motor_forward);
}

void Tank::_update_motors() {
  uint8_t left_motor_control_code = _determine_motor_control_code(CONTROL_CODE_LEFT_MOTOR_FORWARD, CONTROL_CODE_LEFT_MOTOR_REVERSE, _left_motor_status);
  uint8_t right_motor_control_code = _determine_motor_control_code(CONTROL_CODE_RIGHT_MOTOR_FORWARD, CONTROL_CODE_RIGHT_MOTOR_REVERSE, _right_motor_status);
  uint8_t turret_motor_control_code = _determine_motor_control_code(CONTROL_CODE_TURRET_MOTOR_FORWARD, CONTROL_CODE_TURRET_MOTOR_REVERSE, _turret_motor_status);

  uint8_t new_motor_control_code = left_motor_control_code | right_motor_control_code | turret_motor_control_code;

  if (new_motor_control_code != _current_motor_control_code) {
#ifdef TANK_DEBUG_OUTPUT
    Serial.print(F("writing new control_code "));
    Serial.println(new_motor_control_code);
#endif
    _current_motor_control_code = new_motor_control_code;
    _write_motor_control_code(_current_motor_control_code);
  }

  uint8_t motor_speed = 0;
  if (left_motor_control_code == 0 && right_motor_control_code == 0) {
    motor_speed = 0;
  } else {
    motor_speed = _requested_speed;
  }

  if (motor_speed != _current_speed) {
    analogWrite(MOTOR_PWM_PIN, motor_speed);
    _current_speed = motor_speed;
  }
}

// incorporate delay logic and return the appropriate control_code for the motor
uint8_t Tank::_determine_motor_control_code(const uint8_t forward_code, const uint8_t reverse_code, MotorStatus & motor_status) {
  if (motor_status.direction_change_requested) {
    // if it is desired that the motor be traveling in the opposite direction and MOTOR_CHANGE_DIRECTION_DELAY_MILLIS has lapsed
    // then we change motor direction
    if (millis() > motor_status.direction_change_request_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
      motor_status.direction = motor_status.requested_direction;
      motor_status.direction_change_requested = false;
      if (motor_status.direction != motor_stop) {
        motor_status.last_direction = motor_status.direction;
      }
    // otherwise, control code is set to zero for this motor. the motor should stop before spinning the other direction.
    } else {
      return 0;
    }
  }

  if (motor_status.direction == motor_forward) {
    return forward_code;
  } else if (motor_status.direction == motor_reverse) {
    return reverse_code;
  } else {
    return 0;
  }
}

// update motor status based on requested direction
void Tank::_control_motor(MotorStatus & status, const MotorDirection direction) {
  status.requested_direction = direction;

  if (direction == motor_stop) {
    status.direction = motor_stop;
  // if the desired motor direction is not the current direction, we set the direction_change_requested flag and set the timer
  } else if (status.requested_direction != status.direction && !status.direction_change_requested) {
    status.direction_change_requested = true;
    status.direction_change_request_millis = millis();
  }
}

void Tank::_write_motor_control_code(const unsigned char & control_code) {
#ifdef MOTORS_ENABLED
#ifdef TANK_DEBUG_OUTPUT
  Serial.print(F("Writing motor control code: "));
  Serial.println(control_code, BIN);
#endif

  digitalWrite(SHIFT_CLEAR_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, control_code);
  digitalWrite(SHIFT_CLEAR_PIN, HIGH);
#endif
}

void Tank::_process_interrupt_flags(unsigned long current_millis) {
  _process_encoder_flags(current_millis);
  _process_bump_flags(current_millis);
  _process_ir_flags();
}

void Tank::_process_encoder_flags(unsigned long current_millis) {
  if (_turret_encoder_interrupt_flag) {
    _turret_encoder_interrupt_flag = false;

    if (_turret_motor_status.last_direction == motor_forward) {
      _turret_status.encoder_count++;
    } else if (_turret_motor_status.last_direction == motor_reverse) {
      _turret_status.encoder_count--;
    }
    _check_turret_target();
#ifdef TANK_DEBUG_OUTPUT
    if (_turret_status.encoder_count % 10 == 0) {
      Serial.print("encoder_count: ");
      Serial.println(_turret_status.encoder_count);
    }
#endif
  }


  if (_turret_calibration_interrupt_flag) {
    _turret_calibration_interrupt_flag = false;
#ifdef TANK_DEBUG_OUTPUT
    Serial.println(F("turret calibration interrupt"));
#endif
    // when the turret passes the calibration point (the 0 degree point), we reset
    // the encoder_count variable. since we track the traversal distance to a target
    // using a target_encoder_count, we have to account for the difference when
    // we set the encoder_count variable back to 0
    if (_turret_status.target_encoder_count != 0) {
      int16_t encoder_difference = abs(_turret_status.target_encoder_count - _turret_status.encoder_count);
      if (_turret_status.target_direction == left) {
        _turret_status.target_encoder_count = -encoder_difference;
      } else if (_turret_status.target_direction == right) {
        _turret_status.target_encoder_count = +encoder_difference;
      }
#ifdef TANK_DEBUG_OUTPUT
      Serial.print(F("new target encoder count: "));
      Serial.println(_turret_status.target_encoder_count);
#endif
    }

    _turret_status.calibrated = true;
    _turret_status.encoder_count = 0;
  }

  if (_left_wheel_encoder_interrupt_flag) {
    _left_wheel_encoder_interrupt_flag = false;
    if (_left_motor_status.direction == motor_forward) {
        _tank_status.wheel_encoder_count_left++;
    } else if(_left_motor_status.direction == motor_reverse) {
        _tank_status.wheel_encoder_count_left--;
    }
    _check_drive_targets();
  }

  if (_right_wheel_encoder_interrupt_flag) {
    _right_wheel_encoder_interrupt_flag = false;
    if (_right_motor_status.direction == motor_forward) {
        _tank_status.wheel_encoder_count_right++;
    } else if(_right_motor_status.direction == motor_reverse) {
        _tank_status.wheel_encoder_count_right--;
    }
    _check_drive_targets();
  }
}

void Tank::_process_bump_flags(unsigned long current_millis) {
  if (_bump_front_interrupt_flag != _bump_status.bump_front && current_millis > _bump_status.bump_front_millis + BUMP_DETECTION_DELAY_MILLIS) {
    _bump_status.bump_front = _bump_front_interrupt_flag;
    _bump_status.bump_front_millis = current_millis;
#ifdef TANK_DEBUG_OUTPUT
    Serial.print(F("Bump front status: "));
    Serial.println(_bump_front_interrupt_flag);
#endif
    if (_bump_front_interrupt_flag == FALLING) {
      _tank_status.bump_front = true;
      if (_bump_front_callback) {
        _bump_front_callback();
      }
    } else {
      _tank_status.bump_front = false;
    }
  }

  if (_bump_rear_interrupt_flag != _bump_status.bump_rear && current_millis > _bump_status.bump_rear_millis + BUMP_DETECTION_DELAY_MILLIS) {
    _bump_status.bump_rear = _bump_rear_interrupt_flag;
    _bump_status.bump_rear_millis = current_millis;
#ifdef TANK_DEBUG_OUTPUT
    Serial.print(F("Bump rear status: "));
    Serial.println(_bump_rear_interrupt_flag);
#endif
    if (_bump_rear_interrupt_flag == FALLING) {
      _tank_status.bump_rear = true;
    } else {
      _tank_status.bump_rear = false;
    }
  }
}

void Tank::_process_ir_flags() {
  if (_ir_command_received) {
    _ir_command_received = false;
    _ir_status.last_command = _ir_command;
    if (_ir_status.ir_command_callback) {
      _ir_status.ir_command_callback(_ir_status.last_command);
    }
  }
}

// note that this function does not reset drive target distance/degrees variables. whereas public drive functions do
// reset these variables.
void Tank::_drive(const MotorDirection left_direction, const MotorDirection right_direction, const uint8_t speed) {
  _requested_speed = speed;
  _control_motor(_left_motor_status, left_direction);
  _control_motor(_right_motor_status, right_direction);
}

void Tank::_check_drive_targets() {
  _check_drive_distance_target();
  _check_drive_turn_target();
}

void Tank::_check_drive_distance_target() {
  // check if we have a drive target
  if (_tank_status.drive_target_distance != 0) {
    if (
      // if the target is in a forward direction, then we want the encoder count to be more than the target distance
      (_tank_status.drive_target_distance > 0
        && (_tank_status.wheel_encoder_count_left / WHEEL_ENCODER_DISTANCE_RATIO >= _tank_status.drive_target_distance && _tank_status.wheel_encoder_count_right / WHEEL_ENCODER_DISTANCE_RATIO >= _tank_status.drive_target_distance))
      ||
      // if the target is in a reverse direction, then we want the encoder count to be LESS than the target distance (because the encoder count goes in reverse when the wheel direction is in reverse)
      (_tank_status.drive_target_distance < 0
        && (_tank_status.wheel_encoder_count_left / WHEEL_ENCODER_DISTANCE_RATIO <= _tank_status.drive_target_distance && _tank_status.wheel_encoder_count_right / WHEEL_ENCODER_DISTANCE_RATIO <= _tank_status.drive_target_distance))
    ) {
#ifdef TANK_DEBUG_OUTPUT
        Serial.println(F("Drive distance target reached"));
#endif
        _tank_status.drive_target_distance_reached = true;
        _tank_status.drive_target_distance = 0;
        if (_drive_target_callback) {
          _drive_target_callback();
        }
        _drive_target_callback = NULL;
    }
  }
}

void Tank::_check_drive_turn_target() {
  // check if there exists a drive turn (degrees) target
  if (_tank_status.drive_target_degrees != 0) {
    if (
      // if target degrees are less than zero, we are turning left.
      (_tank_status.drive_target_degrees < 0 
        && _tank_status.wheel_encoder_count_left - _tank_status.wheel_encoder_count_right < _tank_status.drive_target_degrees * WHEEL_ENCODER_TURN_RATIO)
      ||
      (_tank_status.drive_target_degrees > 0 
        && _tank_status.wheel_encoder_count_left - _tank_status.wheel_encoder_count_right > _tank_status.drive_target_degrees * WHEEL_ENCODER_TURN_RATIO)
    ) {
#ifdef TANK_DEBUG_OUTPUT
        Serial.println(F("Drive degrees target reached"));
#endif
        _tank_status.drive_target_degrees_reached = true;
        _tank_status.drive_target_degrees = 0;
        if (_drive_target_callback) {
          _drive_target_callback();
        }
        _drive_target_callback = NULL;
    }
  }
}

void Tank::_check_turret_target() {
  if (_turret_status.has_target) {
    if (_turret_status.target_direction == left && _turret_status.encoder_count <= _turret_status.target_encoder_count) {
      _turret_target_reached();
    } else if (_turret_status.target_direction == right && _turret_status.encoder_count >= _turret_status.target_encoder_count) {
      _turret_target_reached();
    }
  }
}

void Tank::_turret_target_reached() {
  turret_stop();
  if (_turret_status.target_callback) {
    _turret_status.target_callback();
  }
  _turret_status.has_target = false;
  _turret_status.target_callback = NULL;
}
