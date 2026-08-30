// TODO: add bumper test
// also, I want a target test (or, tank will always have to be acting as a target)
// If I add more than one mode, I will have to consolidate modes?


#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/ir_codes.h"
#include "src/pins.h"
#include "src/tank_constants.h"
#include "src/tank.h"

Tank tank;
MiniLed tank_led;
PVision ircam;
VL53L0X distance_sensor;

byte camera_result;
bool target_active;
uint16_t target_x;

typedef void (*ModeFunction) ();
ModeFunction mode_function = NULL;

int last_ir_command;
bool state_switched = false;

unsigned long last_distance_read_millis = 0;
unsigned long last_camera_read_millis = 0;

void setup()
{
  Wire.begin(); // this is necessary to communicate with I2C devices

#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));
  Serial.println(F("Initializing ..."));
#endif

  tank.initialize();

  // it is not great that the LEDs are controlled by both the Tank class
  // and by this sketch. however, this is just to demonstrate that the LEDs
  // are working properly.
  // you should not mess with the LEDs in your own combat robot, unless you
  // are doing so as part of debugging feedback that you will eventually remove.
  uint8_t pins[] = {LED_PIN_1, LED_PIN_2, LED_PIN_3};
  tank_led.setup(pins, LOW);

  tank.set_ir_command_callback(process_ir_command);
#ifdef DEBUG_OUTPUT
  Serial.println(F("Tank initialized."));
#endif

  initialize();
}

void initialize()
{
  tank.set_bump_front_callback(NULL);
  tank.set_bump_rear_callback(NULL);

  tank_led.all_off();

  camera_init();
  distance_sensor_init();
}

void loop()
{
  if (mode_function) {
    if (state_switched) {
      initialize();
      state_switched = false;
    }
    mode_function();
  }
  tank.loop();
  tank_led.loop();
}

void camera_init()
{
  Serial.println(F("Initializing camera ..."));
  ircam.init();
#ifdef DEBUG_OUTPUT
  Serial.println(F("Camera initialized."));
#endif

  target_active = false;
}

void distance_sensor_init()
{
  Serial.println(F("Initializing distance sensor ..."));
  distance_sensor.setTimeout(500);
  if (!distance_sensor.init()) {
#ifdef DEBUG_OUTPUT
    Serial.println(F("Failed to detect and initialize sensor!"));
#endif
  }
  distance_sensor.startContinuous(DISTANCE_READ_DELAY);
#ifdef DEBUG_OUTPUT
  Serial.println(F("Distance sensor initialized."));
#endif
}

void drive_test()
{
  switch (last_ir_command) {
    case IR_CODE_UP:
      Serial.println(F("drive forward"));
      tank.drive_forward();
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      Serial.println(F("turn right"));
      tank.drive_turn_right();
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      Serial.println(F("drive reverse"));
      tank.drive_reverse();
      last_ir_command = 0;
      break;
    case IR_CODE_LEFT:
      Serial.println(F("turn left"));
      tank.drive_turn_left();
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      Serial.println(F("drive stop"));
      tank.drive_stop();
      last_ir_command = 0;
      break;
  }
}

void drive_stop()
{
  tank.drive_stop();
}

void drive_measured_test()
{
  switch (last_ir_command) {
    case IR_CODE_UP:
      Serial.println(F("drive forward 10 cm"));
      tank.drive_forward_target(10, drive_stop);
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      Serial.println(F("drive reverse 10 cm"));
      tank.drive_reverse_target(10, drive_stop);
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      Serial.println(F("drive turn right 90 degrees"));
      tank.drive_turn_right_degrees(90, drive_stop);
      last_ir_command = 0;
      break;
    case IR_CODE_LEFT:
      Serial.println(F("drive turn left 90 degrees"));
      tank.drive_turn_left_degrees(90, drive_stop);
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      Serial.println(F("drive stop"));
      tank.drive_stop();
      last_ir_command = 0;
      break;
  }
}

void turret_test()
{
  switch (last_ir_command) {
    case IR_CODE_LEFT:
      tank.turret_left();
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      tank.turret_right();
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      tank.turret_stop();
      last_ir_command = 0;
      break;
  }
}

void turret_measured_test()
{
  switch (last_ir_command) {
    case IR_CODE_LEFT:
      tank.turret_left_degrees(90);
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      tank.turret_right_degrees(90);
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      tank.turret_stop();
      last_ir_command = 0;
      break;
  }
}

void setup_routine_test()
{
  tank.setup_routine();
}

void led_test()
{
  switch (last_ir_command) {
    case IR_CODE_UP:
      Serial.println("led test: all on");
      tank_led.all_on();
      last_ir_command = 0;
      break;
    case IR_CODE_LEFT:
      Serial.println("led test: led 1 on");
      tank_led.all_on();
      tank_led.all_off();
      tank_led.on(0);
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      Serial.println("led test: led 2 on");
      tank_led.all_off();
      tank_led.on(1);
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      Serial.println("led test: led 3 on");
      tank_led.all_off();
      tank_led.on(2);
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      Serial.println("led test: all off");
      tank_led.all_off();
      last_ir_command = 0;
      break;
  }
}

void distance_sensor_test()
{
  if (millis() > last_distance_read_millis + DISTANCE_READ_DELAY) {
    last_distance_read_millis = millis();
    uint16_t distance = distance_sensor.readRangeContinuousMillimeters();

    if (distance > 0 && distance < DISTANCE_MAX) {
#ifdef DEBUG_OUTPUT
      Serial.print(F("Distance: "));
      Serial.println(distance);
#endif
      tank_led.all_off();
      if (distance <= 250) {
        tank_led.all_on();
      } else if (distance > 250 && distance <= 750) {
        tank_led.on(0);
        tank_led.on(1);
      } else {
        tank_led.on(0);
      }
    } else {
      tank_led.all_off();
    }
  }
}

void camera_test()
{
  if (millis() > last_camera_read_millis + CAMERA_READ_DELAY) {
    last_camera_read_millis = millis();
    camera_result = ircam.read();
    if (camera_result & BLOB1) {
      Serial.print(F("Target detected. X:"));
      Serial.print(ircam.Blob1.X);
      Serial.print(F(" Y:"));
      Serial.print(ircam.Blob1.Y);
      Serial.print(F(" Size:"));
      Serial.println(ircam.Blob1.Size);

      target_x = ircam.Blob1.X;
      target_active = true;
    } else {
      target_active = false;
    }
  }

  if (target_active) {
    if (target_x < 462) {
      tank_led.all_off();
      tank_led.on(0);
      tank.turret_right();
    } else if (target_x > 562) {
      tank_led.all_off();
      tank_led.on(2);
      tank.turret_left();
    } else {
      tank_led.all_on();
      tank.turret_stop();
    }
  } else {
    tank_led.all_off();
    tank.turret_stop();
  }
}

void speaker_test()
{
  if (last_ir_command == IR_CODE_OK) {
#ifdef SOUND_ENABLED
    tone(SPEAKER_PIN, 33, 500);
#endif
    last_ir_command = 0;
  }
}

void bump_test()
{
  tank.set_bump_front_callback(bump_test_front_callback);
  tank.set_bump_rear_callback(bump_test_rear_callback);
}

void bump_test_front_callback(bool state)
{
  if (state) {
    tank_led.on(0);
  } else {
    tank_led.off(0);
  }
}

void bump_test_rear_callback(bool state)
{
  if (state) {
    tank_led.on(1);
  } else {
    tank_led.off(1);
  }
}

void process_ir_command(int ir_command)
{
#ifdef DEBUG_OUTPUT
  Serial.print(F("Received IR command: "));
  Serial.println(ir_command);
#endif

  switch (ir_command) {
    // numerical IR commands are used to switch test states
    case IR_CODE_ONE:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Drive Test"));
#endif
      mode_function = drive_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_TWO:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Drive Measured Test"));
#endif
      mode_function = drive_measured_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_THREE:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Turret Test"));
#endif
      mode_function = turret_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_FOUR:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Turret Measured Test"));
#endif
      mode_function = turret_measured_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_FIVE:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Setup routine"));
#endif
      mode_function = setup_routine_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_SIX:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Distance Sensor Test"));
#endif
      mode_function = distance_sensor_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_SEVEN:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Camera Test"));
#endif
      mode_function = camera_test;
      state_switched = true;
      last_ir_command = 0;
      break;

   case IR_CODE_EIGHT:
#ifdef DEBUG_OUTPUT
      Serial.println(F("LED Test"));
#endif
      mode_function = led_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_NINE:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Speaker Test"));
#endif
      mode_function = speaker_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    case IR_CODE_ZERO:
#ifdef DEBUG_OUTPUT
      Serial.println(F("Bump Test"));
#endif
      mode_function = bump_test;
      state_switched = true;
      last_ir_command = 0;
      break;

    // if ir_command is not a state-switching command, then remember the ir_command
    default:
      last_ir_command = ir_command;
  }
}
