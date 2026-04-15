#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/tank.h"

unsigned long last_update_millis = 0;

Tank tank;
PVision ircam;
VL53L0X sensor;

struct target {
  uint16_t x;
  uint16_t y;
  uint16_t size;
};

bool find_target(struct target & t)
{
  byte result = ircam.read();

  if (result & BLOB1) {
      t.x = ircam.Blob1.X;
      t.y = ircam.Blob1.Y;
      t.size = ircam.Blob1.Size;
      return true;
  }

  return false;
}

bool find_distance(uint16_t & distance)
{
  uint16_t sensor_read = sensor.readRangeContinuousMillimeters();

  if (sensor_read > 0 && sensor_read < DISTANCE_MAX) {
    Serial.println(sensor_read);
    distance = sensor_read;
    return true;
  }

  return false;
}


void setup()
{
  #ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println("Initializing ...");
  #endif

  Wire.begin();

  tank.initialize();

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
  unsigned long current_millis = millis();

  if (current_millis > last_update_millis + SENSOR_READ_DELAY) {
    struct target my_target;
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
    }

    uint16_t distance;
    if (find_distance(distance)) {
      #ifdef DEBUG_OUTPUT
      Serial.print("distance: ");
      Serial.println(distance);
      #endif
    }
    last_update_millis = current_millis;
  }
  tank.loop();
}
