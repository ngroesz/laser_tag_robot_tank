#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/ir_codes.h"
#include "src/pins.h"
#include "src/tank_constants.h"
#include "src/tank.h"

Tank tank;
TankLed tank_led;
PVision ircam;
VL53L0X distance_sensor;

// TODO: will this be used with camera test?
byte camera_result;
struct target {
  uint16_t x;
  uint16_t y;
  uint16_t size;
};

typedef void (*CallbackFunction) ();

CallbackFunction mode_function = NULL;

int last_ir_command;

unsigned long last_distance_read_millis = 0;
unsigned long last_camera_read_millis = 0;

void setup()
{
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));
  Serial.println("Initializing ...");
#endif

  tank.initialize();

  // it is not great that the LEDs are controlled by both the Tank class
  // and by this sketch. however, this is just to demonstrate that the LEDs
  // are working properly.
  // you should not mess with the LEDs in your own combat robot, unless you
  // are doing so as part of debugging feedback that you will eventually remove.
  uint8_t pins[] = {LED_PIN_1, LED_PIN_2, LED_PIN_3};
  tank_led.setup(pins);
  tank_led.all_off();

  tank.set_ir_command_callback(process_ir_command);
#ifdef DEBUG_OUTPUT
  Serial.println("Tank initialized.");
#endif
}

void loop()
{
  if (mode_function) {
    mode_function();
  }
  tank.loop();
  tank_led.loop();
}

void camera_init()
{
  Serial.println("Initializing camera ...");
  ircam.init();
#ifdef DEBUG_OUTPUT
  Serial.println("Camera initialized.");
#endif
}

void distance_sensor_init()
{
  distance_sensor.setTimeout(500);
  if (!distance_sensor.init()) {
#ifdef DEBUG_OUTPUT
    Serial.println("Failed to detect and initialize sensor!");
#endif
    while (1) {}
  }
  distance_sensor.startContinuous(DISTANCE_READ_DELAY);
#ifdef DEBUG_OUTPUT
  Serial.println("Distance sensor initialized.");
#endif
}

void drive_test()
{
  switch (last_ir_command) {
    case IR_CODE_UP:
      Serial.println("drive forward");
      tank.drive_forward();
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      Serial.println("turn right");
      tank.drive_turn_right();
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      Serial.println("drive reverse");
      tank.drive_reverse();
      last_ir_command = 0;
      break;
    case IR_CODE_LEFT:
      Serial.println("turn left");
      tank.drive_turn_left();
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      Serial.println("drive stop");
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
      Serial.println("drive forward 10 cm");
      tank.drive_forward_target(10, drive_stop);
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      Serial.println("drive reverse 10 cm");
      tank.drive_reverse_target(10, drive_stop);
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      Serial.println("drive stop");
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
}

void turret_calibration_test()
{
  switch (last_ir_command) {
    case IR_CODE_OK:
      tank.turret_calibrate();
      last_ir_command = 0;
      break;
  }
}

void led_test()
{
  switch (last_ir_command) {
    case IR_CODE_UP:
      tank_led.all_on();
      last_ir_command = 0;
      break;
    case IR_CODE_LEFT:
      tank_led.all_off();
      tank_led.turn_on(0);
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      tank_led.all_off();
      tank_led.turn_on(1);
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      tank_led.all_off();
      tank_led.turn_on(2);
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
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
      Serial.print("Distance: ");
      Serial.println(distance);
#endif
      tank_led.all_off();
      if (distance <= 100) {
        tank_led.all_on();
      } else if (distance > 100 && distance <= 500) {
        tank_led.turn_on(1);
        tank_led.turn_on(2);
      } else {
        tank_led.turn_on(1);
      }
    }
  }
}

void camera_test()
{
  if (millis() > last_camera_read_millis + CAMERA_READ_DELAY) {
    last_camera_read_millis = millis();
    camera_result = ircam.read();
    if (camera_result & BLOB1) {
      Serial.print("Target detected. X:");
      Serial.print(ircam.Blob1.X);
      Serial.print(" Y:");
      Serial.print(ircam.Blob1.Y);
      Serial.print(" Size:");
      Serial.println(ircam.Blob1.Size);
    }
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

void process_ir_command(int ir_command)
{
#ifdef DEBUG_OUTPUT
  Serial.print("Received IR command: ");
  Serial.println(ir_command);
#endif

  switch (ir_command) {
    // numerical IR commands are used to switch test states
    case IR_CODE_ONE:
#ifdef DEBUG_OUTPUT
      Serial.println("Drive Test");
#endif
      mode_function = drive_test;
      last_ir_command = 0;
      break;

    case IR_CODE_TWO:
#ifdef DEBUG_OUTPUT
      Serial.println("Drive Measured Test");
#endif
      mode_function = drive_measured_test;
      last_ir_command = 0;
      break;

    case IR_CODE_THREE:
#ifdef DEBUG_OUTPUT
      Serial.println("Turret Test");
#endif
      mode_function = turret_test;
      last_ir_command = 0;
      break;

    case IR_CODE_FOUR:
#ifdef DEBUG_OUTPUT
      Serial.println("Turret Measured Test");
#endif
      mode_function = turret_measured_test;
      last_ir_command = 0;
      break;

    case IR_CODE_FIVE:
#ifdef DEBUG_OUTPUT
      Serial.println("Turret Calibration Test");
#endif
      mode_function = turret_calibration_test;
      last_ir_command = 0;
      break;

    case IR_CODE_SIX:
#ifdef DEBUG_OUTPUT
      Serial.println("Distance Sensor Test");
#endif
      mode_function = distance_sensor_test;
      last_ir_command = 0;
      distance_sensor_init();
      break;

    case IR_CODE_SEVEN:
#ifdef DEBUG_OUTPUT
      Serial.println("Camera Test");
#endif
      mode_function = camera_test;
      last_ir_command = 0;
      camera_init();
      break;

   case IR_CODE_EIGHT:
#ifdef DEBUG_OUTPUT
      Serial.println("LED Test");
#endif
      mode_function = led_test;
      last_ir_command = 0;
      break;

    case IR_CODE_NINE:
#ifdef DEBUG_OUTPUT
      Serial.println("Speaker Test");
#endif
      mode_function = speaker_test;
      last_ir_command = 0;
      break;
    // if ir_command is not a state-switching command, then remember the ir_command
    default:
      last_ir_command = ir_command;
  }
}
