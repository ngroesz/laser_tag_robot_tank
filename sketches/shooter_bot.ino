/* shooter_bot.ino
A bot that finds a target and shoots at it. Not a particularly cunning foe.
As of now, does not drive.
*/

#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "src/tank.h"

#define CAMERA_ENABLED
#define CAMERA_READ_DELAY 100
#define DEBUG_OUTPUT
#define DISTANCE_ENABLED
#define DISTANCE_MAX 8190
#define DISTANCE_READ_DELAY 100

unsigned long last_camera_millis = 0;
unsigned long last_distance_millis = 0;

Tank tank;
PVision ircam;
VL53L0X sensor;

bool target_active = false;
struct target {
  uint16_t x;
  uint16_t y;
  uint16_t size;
};
struct target my_target;

bool find_target(struct target& t) {
  byte result = ircam.read();

  if (result & BLOB1) {
    t.x = ircam.Blob1.X;
    t.y = ircam.Blob1.Y;
    t.size = ircam.Blob1.Size;
    return true;
  }

  return false;
}

bool find_distance(uint16_t& distance) {
  uint16_t sensor_read = sensor.readRangeContinuousMillimeters();

  if (sensor_read > 0 && sensor_read < DISTANCE_MAX) {
    Serial.println(sensor_read);
    distance = sensor_read;
    return true;
  }

  return false;
}

void setup() {
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println("Initializing ...");
#endif

  Wire.begin();

  tank.initialize();

#ifdef DEBUG_OUTPUT
  Serial.println("Tank initialized.");
#endif

#ifdef CAMERA_ENABLED
    ircam.init();
#ifdef DEBUG_OUTPUT
    Serial.println("Camera initialized.");
#endif
#endif

#ifdef DISTANCE_ENABLED
  sensor.setTimeout(500);
  if (!sensor.init()) {
#ifdef DEBUG_OUTPUT
    Serial.println("Failed to detect and initialize sensor!");
#endif
    while (1) {
    }
  }
  sensor.startContinuous(DISTANCE_READ_DELAY);
#ifdef DEBUG_OUTPUT
  Serial.println("Distance sensor initialized.");
#endif
#endif
  tank.setup_routine();
}

void loop() {
  unsigned long current_millis = millis();

  if (current_millis > last_camera_millis + CAMERA_READ_DELAY) {
    if (find_target(my_target)) {
#ifdef DEBUG_OUTPUT
      Serial.println("Target Detected");
      Serial.print(" X: ");
      Serial.print(my_target.x);
      Serial.print(" Y: ");
      Serial.print(my_target.y);
      Serial.print(" Size: ");
      Serial.println(my_target.size);
#endif
      target_active = true;
    } else {
      target_active = false;
    }

    last_camera_millis = current_millis;
  }

  if (target_active) {
    if (my_target.x < 462) {
      tank.turret_right();
    } else if (my_target.x > 562) {
      tank.turret_left();
    } else {
      tank.turret_stop();
    }
  } else {
    tank.turret_stop();
  }



  if (current_millis > last_distance_millis + CAMERA_READ_DELAY) {
//    uint16_t distance;
//    if (find_distance(distance)) {
//#ifdef DEBUG_OUTPUT
//      Serial.print("distance: ");
//      Serial.println(distance);
//#endif
//    }
    last_distance_millis = current_millis;
  }

  tank.loop();
}
