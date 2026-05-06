#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/tank_constants.h"
#include "src/ir_codes.h"
#include "src/pins.h"
#include "src/tank.h"

Tank tank;

PVision ircam;
VL53L0X sensor;

struct target {
  uint16_t x;
  uint16_t y;
  uint16_t size;
};

int last_ir_command;

typedef void (*CallbackFunction) ();

CallbackFunction mode_function = NULL;

void setup()
{
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));
  Serial.println("Initializing ...");
#endif

  tank.initialize();

  tank.set_ir_command_callback(process_ir_command);
#ifdef DEBUG_OUTPUT
  Serial.println("Tank initialized.");
#endif

  if (CAMERA_ENABLED) {
    ircam.init();
#ifdef DEBUG_OUTPUT
    Serial.println("Camera initialized.");
#endif
  }

  if (DISTANCE_ENABLED) {
    sensor.setTimeout(500);
    if (!sensor.init()) {
#ifdef DEBUG_OUTPUT
      Serial.println("Failed to detect and initialize sensor!");
#endif
      while (1) {}
    }
    sensor.startContinuous(SENSOR_READ_DELAY);
#ifdef DEBUG_OUTPUT
    Serial.println("Distance sensor initialized.");
#endif
  }
}

void loop()
{
  if (mode_function) {
    mode_function();
  }
  tank.loop();
}

void drive_test()
{
  switch (last_ir_command) {
    case IR_CODE_UP:
      Serial.println("forward!");
      tank.drive_forward();
      last_ir_command = 0;
      break;
    case IR_CODE_RIGHT:
      tank.drive_turn_right();
      last_ir_command = 0;
      break;
    case IR_CODE_DOWN:
      tank.drive_reverse();
      last_ir_command = 0;
      break;
    case IR_CODE_LEFT:
      tank.drive_turn_left();
      last_ir_command = 0;
      break;
    case IR_CODE_OK:
      tank.drive_stop();
      last_ir_command = 0;
      break;
  }
}

void drive_measured_test()
{
}

void turret_test()
{
}

void turret_measured_test()
{
}

void speaker_test()
{
  if (last_ir_command == IR_CODE_OK) {
    Serial.println("beep");
    tone(SPEAKER_PIN, 33, 500);
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
      mode_function = drive_test;
      last_ir_command = 0;
      break;
    case IR_CODE_TWO:
      mode_function = drive_measured_test;
      last_ir_command = 0;
      break;
    case IR_CODE_THREE:
      mode_function = turret_test;
      last_ir_command = 0;
      break;
    case IR_CODE_FOUR:
      mode_function = turret_measured_test;
      last_ir_command = 0;
      break;
    case IR_CODE_NINE:
      mode_function = speaker_test;
      last_ir_command = 0;
      break;
    // if ir_command is not a state-switching command, then remember the ir_command
    default:
      last_ir_command = ir_command;
  }
}
