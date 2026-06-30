/* wander_bot.ino
A bot that roams around and avoids obstacles. It uses the distance sensor
to build a 360-degree view and tracks the distance to the nearest obstacle
that exists within 8 45-degree "radar" slices.
The robot is a pacifist and does not shoot targets nor seek them.
*/

#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/tank.h"

unsigned long last_update_millis = 0;

Tank tank;
VL53L0X sensor;

enum Mode {
  wait,
  calibrate,
  wander
}

struct State {
  Mode mode
};

// radar array tracks nearest obstacle within a 45 degree slice
// radar[0]: [-22.5,22.5), radar[1]: [22.5,45), etc
uint16t radar[8] = {0};

State state;

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

void loop()
{
  switch (state.mode) {
    case Mode.wait:
      break;
    case Mode.calibrate:
      break;
    case Mode.wander:
      wander();
      break;
  }
}

void wander()
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
