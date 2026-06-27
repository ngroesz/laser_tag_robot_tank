/*
* motor_stress_test.ino:
* The purpose of this sketch is to stress the motor. Issues with the motors, specifically the
* motor contoller and the Arduino, have bedevilled me since the beginning of this project.
* The most common failure I have experienced is that turning a motor on or changing its direction
* will cause the microcontoller to reset, either through a deprivation of power or from the electrical
* noise caused by the motor.
* Anyway, the point of this program is to constantly run the motors, in various directions, in order
* to cause maximum load. This uses the Tank library so the motor-direction-change delay in place there
* is in effect here.
* The front bump sensor is used to turn the motors on or off. The reason for this is that I wanted to
* be able to run this stress test with just the chassis and motors, and no hull. Because of this, the
* tank has no IR or other way to receive input, unless I were to jury-rig an IR sensor directly on to the
* chassis.
* So this sketch should have no particular interest to most. But I am maintaining it in the repository,
* as I am nearly certain motor troubles will continue to haunt me.
*/


#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/ir_codes.h"
#include "src/pins.h"
#include "src/tank_constants.h"
#include "src/tank.h"

#define DRIVE_CHANGE_DELAY 5000

Tank tank;


enum State {
  wait,
  drive
};

State current_state;

unsigned long last_drive_change_millis = 0;

void setup()
{
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));
  Serial.println("Initializing ...");
#endif

  tank.initialize();

#ifdef DEBUG_OUTPUT
  Serial.println("Tank initialized.");
#endif

  current_state = wait;
  tank.set_bump_front_callback(bump_front);

  randomSeed(analogRead(0));
}

void bump_front()
{
  if (current_state == wait) {
    Serial.println("driving");
    current_state = drive;
  } else {
    Serial.println("waiting");
    current_state = wait;
  }
}

void loop()
{
  if (current_state == wait) {
      tank.drive_stop();
  } else {
    if (millis() > last_drive_change_millis + DRIVE_CHANGE_DELAY) {
      Serial.println("change drive direction");
      last_drive_change_millis = millis();
      int drive_change = random(0, 4);

      switch (drive_change) {
        case 0:
          Serial.println("drive forward");
          tank.drive_forward();
          break;
        case 1:
          Serial.println("drive reverse");
          tank.drive_reverse();
          break;
        case 2:
          Serial.println("drive left");
          tank.drive_turn_left();
          break;
        case 3:
          Serial.println("drive right");
          tank.drive_turn_right();
          break;
      }

      switch (random(0, 2)) {
        case 0:
          Serial.println("turret left");
          tank.turret_left();
          break;
        case 1:
          Serial.println("turret right");
          tank.turret_right();
          break;
      }
    }
  }

  tank.loop();
}
