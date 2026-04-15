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
#ifdef DEBUG_OUTPUT
  Serial.println(F("Tank initializing"));
#endif

  // initialize LEDs
  uint8_t pins[] = {LED_PIN_1, LED_PIN_2, LED_PIN_3};
  _tank_led.setup(pins);
  _tank_led.turn_on(0);
  delay(250);

  // initialize motors
  digitalWrite(LEFT_MOTOR_PWM_PIN, 0);
  digitalWrite(RIGHT_MOTOR_PWM_PIN, 0);
  digitalWrite(TURRET_MOTOR_PWM_PIN, 0);
  digitalWrite(SHIFT_CLEAR_PIN, 0);
  digitalWrite(SHIFT_CLOCK_PIN, 0);
  digitalWrite(SHIFT_DATA_PIN, 0);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(TURRET_MOTOR_PWM_PIN, OUTPUT);
  pinMode(SHIFT_CLEAR_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_DATA_PIN, OUTPUT);

  _left_motor_status = {
    .direction = motor_stop,
    .requested_direction = motor_stop,
    .last_direction = motor_stop,
    .requested_speed = 0,
    .current_speed = 0,
    .direction_change_requested = false,
    .direction_change_request_millis = 0
  };
  _right_motor_status = {
    .direction = motor_stop,
    .requested_direction = motor_stop,
    .last_direction = motor_stop,
    .requested_speed = 0,
    .current_speed = 0,
    .direction_change_requested = false,
    .direction_change_request_millis = 0
  };
  _turret_motor_status = {
    .direction = motor_stop,
    .requested_direction = motor_stop,
    .last_direction = motor_stop,
    .requested_speed = 0,
    .current_speed = 0,
    .direction_change_requested = false,
    .direction_change_request_millis = 0
  };

  _tank_led.turn_on(1);
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

  _tank_led.turn_on(2);
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
  _ir_command_callback = callback;
}

void Tank::drive(const MotorDirection left_direction, const MotorDirection right_direction, const uint8_t left_speed, const uint8_t right_speed) {
  Serial.println("drive");
  _control_motor(_left_motor_status, left_direction, left_speed);
  _control_motor(_right_motor_status, right_direction, right_speed);
}

void Tank::drive_forward(const uint8_t left_speed, const uint8_t right_speed) {
  drive(motor_forward, motor_forward, left_speed, right_speed);
}

void Tank::drive_reverse(const uint8_t left_speed, const uint8_t right_speed) {
  drive(motor_reverse, motor_reverse, left_speed, right_speed);
}

void Tank::drive_reverse_target(const int16_t target_distance, CallbackFunction target_callback, const uint8_t left_speed, const uint8_t right_speed) {
  _tank_status.wheel_encoder_count_left = 0;
  _tank_status.wheel_encoder_count_right = 0;
  _tank_status.drive_target_reached = false;
  _tank_status.drive_target_distance = -target_distance;
  _drive_target_callback = target_callback;
  drive_reverse(left_speed, right_speed);
}

void Tank::drive_turn_left(uint8_t left_speed, const uint8_t right_speed) {
  drive(motor_reverse, motor_forward, left_speed, right_speed);
}

void Tank::drive_turn_right(uint8_t left_speed, const uint8_t right_speed) {
  drive(motor_forward, motor_reverse, left_speed, right_speed);
}

void Tank::drive_stop() {
  _control_motor(_left_motor_status, motor_stop, 0);
  _control_motor(_right_motor_status, motor_stop, 0);
}

void Tank::turret_calibrate(const uint8_t speed) {
#ifdef DEBUG_OUTPUT
  Serial.println(F("turret calibration"));
#endif
  _turret_status.calibrated = false;

  turret_left(speed);
  while (!_turret_status.calibrated) {
    loop();
  }

  turret_stop();
#ifdef DEBUG_OUTPUT
  Serial.println(F("turret has been calibrated"));
#endif
}

void Tank::turret_left(const uint8_t speed) {
  _turret_status.has_target = false;
  _turret_status.target_callback = NULL;
  _turret_left(speed);
}

void Tank::turret_right(const uint8_t speed) {
  _turret_status.has_target = false;
  _turret_status.target_callback = NULL;
  _turret_right(speed);
}

void Tank::turret_left_degrees(const uint16_t degrees, const uint8_t speed) {
  _turret_status.has_target = true;
  _turret_status.target_direction = left;
  _turret_status.target_encoder_count = _turret_status.encoder_count - round(degrees * TURRET_GEAR_RATIO);
  _turret_status.target_callback = NULL;
  _turret_left(speed);
}

void Tank::turret_right_degrees(const uint16_t degrees, const uint8_t speed) {
  _turret_status.has_target = true;
  _turret_status.target_direction = right;
  _turret_status.target_encoder_count = _turret_status.encoder_count + round(degrees * TURRET_GEAR_RATIO);
  _turret_status.target_callback = NULL;
  _turret_right(speed);
}

void Tank::turret_left_degrees(const uint16_t degrees, CallbackFunction target_callback, const uint8_t speed) {
  _turret_status.target_callback = target_callback;
  turret_left_degrees(degrees, speed);
}

void Tank::turret_right_degrees(const uint16_t degrees, CallbackFunction target_callback, const uint8_t speed) {
  _turret_status.target_callback = target_callback;
  turret_right_degrees(degrees, speed);
}

void Tank::turret_set_degrees(const uint16_t target_degrees, const uint8_t speed) {
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
    _turret_right(speed);
  // otherwise, we will turn counter-clockwise (left)
  } else {
    _turret_status.target_direction = left;
    _turret_status.target_encoder_count = _turret_status.encoder_count - round(counter_clockwise_degrees * TURRET_GEAR_RATIO);
    _turret_left(speed);
  }
}

void Tank::turret_set_degrees(const uint16_t target_degrees, CallbackFunction target_callback, const uint8_t speed) {
  _turret_status.target_callback = target_callback;
  turret_set_degrees(target_degrees, speed);
}

void Tank::turret_stop() {
  _turret_status.has_target = false;
  _control_motor(_turret_motor_status, motor_stop, 0);
}

const int16_t Tank::turret_get_degrees() {
  if (!_turret_status.calibrated) {
    return -1;
  }

  int16_t turret_position = round(_turret_status.encoder_count / TURRET_GEAR_RATIO);
  int16_t normalized_turret_position = normalize_angle(turret_position);

  return normalized_turret_position;
}

const int16_t Tank::normalize_angle(const int16_t degrees) {
  return degrees < 0 ? (degrees + 1 % 360) + 360 - 1 : degrees % 360;
}
// end public functions

// begin private functions
void Tank::_turret_left(const uint8_t speed) {
  _control_motor(_turret_motor_status, motor_reverse, speed);
}

void Tank::_turret_right(const uint8_t speed) {
  _control_motor(_turret_motor_status, motor_forward, speed);
}

void Tank::_update_motors() {
  uint8_t left_motor_control_code = _update_motor(CONTROL_CODE_LEFT_MOTOR_FORWARD, CONTROL_CODE_LEFT_MOTOR_REVERSE, LEFT_MOTOR_PWM_PIN, _left_motor_status);
  uint8_t right_motor_control_code = _update_motor(CONTROL_CODE_RIGHT_MOTOR_FORWARD, CONTROL_CODE_RIGHT_MOTOR_REVERSE, RIGHT_MOTOR_PWM_PIN, _right_motor_status);
  uint8_t turret_motor_control_code = _update_motor(CONTROL_CODE_TURRET_MOTOR_FORWARD, CONTROL_CODE_TURRET_MOTOR_REVERSE, TURRET_MOTOR_PWM_PIN, _turret_motor_status);

  uint8_t new_motor_control_code = left_motor_control_code | right_motor_control_code | turret_motor_control_code;

  if (new_motor_control_code != _current_motor_control_code) {
#ifdef DEBUG_OUTPUT
    Serial.print(F("writing new control_code "));
    Serial.println(new_motor_control_code);
#endif
    _current_motor_control_code = new_motor_control_code;
    _write_motor_control_code(_current_motor_control_code);
  }
}

uint8_t Tank::_update_motor(uint8_t control_code_motor_forward, uint8_t control_code_motor_reverse, uint8_t motor_pin, MotorStatus & motor_status) {
  uint8_t motor_control_code;
  uint8_t motor_speed;
  _determine_motor_update(control_code_motor_forward, control_code_motor_reverse, motor_status, motor_control_code, motor_speed);
  if (motor_speed != motor_status.current_speed) {
#ifdef DEBUG_OUTPUT
    Serial.print(F("writing new speed to pin "));
    Serial.print(motor_pin);
    Serial.print(F(": "));
    Serial.println(motor_speed);
#endif
    digitalWrite(motor_pin, HIGH);
    //analogWrite(motor_pin, motor_speed);
    motor_status.current_speed = motor_speed;
  }

  return motor_control_code;
}

// incorporate delay logic and update the passed MotorStatus data structure,
// updating the control code and speed
void Tank::_determine_motor_update(const uint8_t forward_code, const uint8_t reverse_code, MotorStatus & motor_status, uint8_t & control_code, uint8_t & speed) {
  if (motor_status.direction_change_requested) {
    // if it is desired that the motor be traveling in the opposite direction and MOTOR_CHANGE_DIRECTION_DELAY_MILLIS has lapsed
    // then we change motor direction
    if (millis() > motor_status.direction_change_request_millis + MOTOR_CHANGE_DIRECTION_DELAY_MILLIS) {
      motor_status.direction = motor_status.requested_direction;
      motor_status.direction_change_requested = false;
      if (motor_status.direction != motor_stop) {
        motor_status.last_direction = motor_status.direction;
      }
    // otherwise speed is set to zero and control code is set to zero for this motor. the motor should stop before spinning the other direction
    } else {
      control_code = 0;
      speed = 0;
      return;
    }
  }

  if (motor_status.direction == motor_forward) {
    control_code = forward_code;
    speed = motor_status.requested_speed;
  } else if (motor_status.direction == motor_reverse) {
    control_code = reverse_code;
    speed = motor_status.requested_speed;
  } else {
    control_code = 0;
    speed = 0;
  }
}

// update motor status based on requested direction and speed
void Tank::_control_motor(MotorStatus & status, const MotorDirection direction, const uint8_t speed) {
  status.requested_direction = direction;
  status.requested_speed = speed;

  if (direction == motor_stop) {
    status.direction = motor_stop;
    status.requested_speed = 0;
  // if the desired motor direction is not the current direction, we set the direction_change_requested flag and set the timer
  } else if (status.requested_direction != status.direction && !status.direction_change_requested) {
    status.direction_change_requested = true;
    status.direction_change_request_millis = millis();
  }
}

void Tank::_write_motor_control_code(const unsigned char & control_code) {
#ifdef DEBUG_OUTPUT
  Serial.print(F("Writing motor control code: "));
  Serial.println(control_code, BIN);
#endif
  digitalWrite(SHIFT_CLEAR_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, control_code);
  digitalWrite(SHIFT_CLEAR_PIN, HIGH);
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
    //if (_turret_status.encoder_count % 10 == 0) {
    //  Serial.print("encoder_count: ");
    //  Serial.println(_turret_status.encoder_count);
    //}
  }


  if (_turret_calibration_interrupt_flag) {
    _turret_calibration_interrupt_flag = false;
#ifdef DEBUG_OUTPUT
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
#ifdef DEBUG_OUTPUT
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
    _check_drive_distance_target();
  }

  if (_right_wheel_encoder_interrupt_flag) {
    _right_wheel_encoder_interrupt_flag = false;

    if (_right_motor_status.direction == motor_forward) {
        _tank_status.wheel_encoder_count_right++;
    } else if(_right_motor_status.direction == motor_reverse) {
        _tank_status.wheel_encoder_count_right--;
    }
    _check_drive_distance_target();
  }
}

void Tank::_process_bump_flags(unsigned long current_millis) {
  if (_bump_front_interrupt_flag != _bump_status.bump_front && current_millis > _bump_status.bump_front_millis + BUMP_DETECTION_DELAY_MILLIS) {
    _bump_status.bump_front = _bump_front_interrupt_flag;
    _bump_status.bump_front_millis = current_millis;
#ifdef DEBUG_OUTPUT
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
#ifdef DEBUG_OUTPUT
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
    if (_ir_command_callback) {
      _ir_command_callback(_ir_command);
    }
  }
}

void Tank::_check_drive_distance_target() {
  // check if we have a drive target
  if (_tank_status.drive_target_distance != -1) {
    // check if the drive target is in the reverse direction
    if (_tank_status.drive_target_distance < 0) {
      // check if we've reached or passed our target
      if (_tank_status.wheel_encoder_count_left / WHEEL_ENCODER_DISTANCE_RATIO <= _tank_status.drive_target_distance && _tank_status.wheel_encoder_count_right / WHEEL_ENCODER_DISTANCE_RATIO <= _tank_status.drive_target_distance) {
#ifdef DEBUG_OUTPUT
        Serial.println(F("Drive distance target reached"));
#endif
        _tank_status.drive_target_reached = true;
        _tank_status.drive_target_distance = -1;
        if (_drive_target_callback) {
          _drive_target_callback();
        }
        _drive_target_callback = NULL;
      }
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
